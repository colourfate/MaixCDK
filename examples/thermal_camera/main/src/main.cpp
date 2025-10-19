/**
 * @file main
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "maix_basic.hpp"
#include "maix_display.hpp"
#include "maix_lvgl.hpp"
#include "maix_gpio.hpp"
#include "maix_pinmap.hpp"
#include "lvgl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/freetype.hpp"
#include "ui.h"
#include "uvc_camera.h"
#include "thermal_sensor.h"
#include "image_fusion.h"
#include "common.h"
#include "platform.h"
#include "sophgo_middleware.hpp"
#include <cstdint>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

using namespace maix;

static int g_disp_pipe[2];
static bool g_proc_run;

static void set_fusion_info(thermal_info *fusion_info, float *ratio, thermal_info *thm_info)
{
    image_fusion_config config;
    float tmp_ratio;

    image_fusion_get_config(&config);
    tmp_ratio = config.target_width / config.thm_width;

    // flip and resize
    fusion_info->max_pos[0] = (THERMAL_FRAME_WIDTH - 1 - thm_info->max_pos[0]) * tmp_ratio;
    fusion_info->max_pos[1] = thm_info->max_pos[1] * tmp_ratio;
    fusion_info->min_pos[0] = (THERMAL_FRAME_WIDTH - 1 - thm_info->min_pos[0]) * tmp_ratio;
    fusion_info->min_pos[1] = thm_info->min_pos[1] * tmp_ratio;
    *ratio = tmp_ratio;
}

static void process_task_cb(lv_timer_t *tmr)
{
    video_buffer *buf;
    static float *thm_frame = NULL;
    static uint8_t thm_img[SENSOR_FRAME_SIZE];
    static thermal_info thm_info;
    float *tmp_thm_frame;
    fusion_frame out_frame;
    float range;
    int i, ret;
    uint32_t retry_cnt = 50;
#define MAX_RANGE 20.0f

    buf = uvc_camera_capture_frame();
    if (buf == NULL) {
        perr("capture frame failed\n");
        return;
    }

    while (thm_frame == NULL) {
        common_probe_1;
        thm_frame = thermal_sensor_read_packet(&thm_info);
        common_probe_2;
        pinfo("First wait thermal frame: %d\n", common_duration_us);
        if (retry_cnt-- == 0) {
            pinfo("Read thermal frame failed, app exit\n");
            g_proc_run = false;
            app::set_exit_flag(true);
            return;
        }
        time::sleep_ms(50);
    }

    tmp_thm_frame = thermal_sensor_read_packet(&thm_info);
    if (tmp_thm_frame != NULL) {
        pdbg("update thermal frame\n");
        thm_frame = tmp_thm_frame;
    } else {
        pdbg("skip thermal frame\n");
    }

    range = thm_info.max_temp - thm_info.min_temp;
    for (i = 0; i < SENSOR_FRAME_SIZE; i++) {
        float norm = (thm_frame[i] - thm_info.min_temp) / range;
        thm_img[i] = (uint8_t)((norm * norm) * 255);
    }

    cv::Mat im_cam(buf->height, buf->width, CV_8UC2, buf->start);
    cv::Mat im_cam_y;
    cvtColor(im_cam, im_cam_y, cv::COLOR_YUV2GRAY_YUYV);
    //common_show_mat(im_cam_y);
    ret = image_fusion_get_frame(&out_frame.buf, im_cam_y.data, im_cam_y.rows * im_cam_y.cols, thm_img, SENSOR_FRAME_SIZE);
    if (ret != 0) {
        perr("fusion failed\n");
        return;
    }

    out_frame.info.max_temp = thm_info.max_temp;
    out_frame.info.min_temp = thm_info.min_temp;
    out_frame.info.center_temp = thm_info.center_temp;
    set_fusion_info(&out_frame.info, &out_frame.ratio, &thm_info);
    ret = write(g_disp_pipe[1], &out_frame, sizeof(fusion_frame));
    if (ret != sizeof(fusion_frame)) {
        perr("Write disp frame failed\n");
    }

    ret = uvc_camera_release_frame(buf);
    if (ret != 0) {
        perr("release frame failed\n");
        return;
    }
}

static void *process_task(void *arg)
{
    thermal_sensor_start2();
    while (g_proc_run) {
        process_task_cb(NULL);
    }

    return NULL;
}

static void get_frame_and_display(void)
{
    fusion_frame out_frame;
    int ret;

    common_probe_1;
    ret = read(g_disp_pipe[0], &out_frame, sizeof(fusion_frame));
    common_probe_2;
    pdbg("read interval: %d\n", common_duration_us());
    if (ret < 0) {
        pdbg("No data in disp pipe\n");
        platform_display_show(NULL);
        return;
    } else if (ret != sizeof(fusion_frame)) {
        perr("read cam pipe failed: %d, len: %d\n", errno, ret);
        return;
    }

    //pdbg("text pos: %f(%d, %d), %f(%d, %d)\n", out_frame.info.max_temp, out_frame.info.max_pos[0], out_frame.info.max_pos[1],
    //    out_frame.info.min_temp, out_frame.info.min_pos[0], out_frame.info.min_pos[1]);
    platform_display_show(&out_frame);

    common_probe_1;
    image_fusion_put_frame(&out_frame.buf);
    common_probe_2;
    pdbg("put interval: %d\n", common_duration_us());
}

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        pinfo("App exit\n");
        app::set_exit_flag(true);
    }
}

#if 1

int _main(int argc, char* argv[])
{
    int ret, flag;
    pthread_t process_thread_id;
    void *phandle;

    common_init();

    pinfo("thermal sensor init\n");
    ret = thermal_sensor_open2();
    if (ret != 0) {
        perr("open thermal failed\n");
        return -1;
    }

    // TODO: replace the number to marco
    platform_device_init(480, 360);
    phandle = platform_vdsp_init();
    display::Display *other_disp = platform_display_add();
    touchscreen::TouchScreen touchscreen = touchscreen::TouchScreen();
    // init lvgl
    maix::lvgl_init(other_disp, &touchscreen);

    // init lvgl ui
    pinfo("ui init\n");
    ui_init();

    pinfo("uvc camera init\n");
    ret = uvc_camera_open();
    if (ret != 0) {
        perr("open camera failed\n");
        return -1;
    }

    pinfo("image fusion init\n");
    image_fusion_config config = {
        .cam_width = 640,
        .cam_height = 480,
        .thm_width = THERMAL_FRAME_WIDTH,
        .thm_height = THERMAL_FRAME_HEIGHT,
        .target_width = 480,
        .target_height = 360
    };
    image_fusion_init(&config);
    image_fusion_set_mode(FUSION_MODE_JOINT);
    image_fusion_set_colormap(FUSION_COLORMAP_PLASMA);

    pinfo("pipe init\n");
    ret = pipe(g_disp_pipe);
    if (ret != 0) {
        perr("pipe failed: %d\n", errno);
        goto exit1;
    }

    flag = fcntl(g_disp_pipe[0], F_GETFL);
    flag |= O_NONBLOCK;
    fcntl(g_disp_pipe[0], F_SETFL, flag);

    if (signal(SIGINT, sig_handler) == SIG_ERR) {
        perr("signal INT failed\n");
        goto exit1;
    }

    g_proc_run = true;
    pthread_create(&process_thread_id, NULL, process_task, NULL);

    // main ui loop
    while (!app::need_exit()) {
        /* Periodically call the lv_task handler.
            * It could be done in a timer interrupt or an OS task too.
            */

        get_frame_and_display();
        uint32_t time_till_next = lv_timer_handler();
        pdbg("sleep %d ms\n", time_till_next);
        time::sleep_ms(time_till_next);
    }

    g_proc_run = false;
    pthread_join(process_thread_id, NULL);
    ui_uninit();
exit1:
    image_fusion_uninit();
    lvgl_destroy();
    thermal_sensor_close2();
    uvc_camera_close();

    platform_vdsp_uninit(phandle);
    platform_device_uninit();

    return 0;
}
#else

#include <cviruntime.h>
#define INPUT_SCALE 1

int _main(int argc, char* argv[])
{
    uint32_t cnt = 0;
    static thermal_info thm_info;
    common_init();

    pinfo("thermal sensor init\n");
    thermal_sensor_open2();

    pinfo("thermal sensor start\n");
    thermal_sensor_start2();

    while (cnt++ < 100) {
        time::sleep_ms(50);
        float *tmp_thm_frame = thermal_sensor_read_packet(&thm_info);
        if (tmp_thm_frame != NULL) {
            pinfo("update thermal frame\n");
        } else {
            pinfo("skip thermal frame\n");
            continue;
        }

        pinfo("cnt: %d, max: %.2f, min: %.2f\n", cnt, thm_info.max_temp, thm_info.min_temp);

    }

    return 0;
}

#endif

int main(int argc, char* argv[])
{
    // Catch signal and process
    sys::register_default_signal_handle();

    // Use CATCH_EXCEPTION_RUN_RETURN to catch exception,
    // if we don't catch exception, when program throw exception, the objects will not be destructed.
    // So we catch exception here to let resources be released(call objects' destructor) before exit.
    CATCH_EXCEPTION_RUN_RETURN(_main, -1, argc, argv);
}

