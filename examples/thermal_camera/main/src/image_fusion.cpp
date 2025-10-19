#include "image_fusion.h"
#include "common.h"
#include "platform.h"
#include <cviruntime.h>
#include <cstdint>
#include <fcntl.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

#define USK_W 0.5

using namespace cv;
using namespace std;
using namespace maix;

/* Resolution     software
 * 256x192        27 s
 * 128x96         7 s
 * 64x48          1.2 s
 * 32x24          0.3 s
 */
#define RUNTIME_WIDTH 256
#define RUNTIME_HEIGHT 192

typedef struct {
    image_fusion_config config;
    image_fusion_mode mode;
    image_high_freq_ratio ratio;
    image_fusion_colormap colormap;

    CVI_MODEL_HANDLE model;
    CVI_TENSOR *input_tensors;
    CVI_TENSOR *output_tensors;
    CVI_TENSOR *input;
    CVI_TENSOR *output;
    uint32_t input_width;
    uint32_t input_height;
    uint32_t output_width;
    uint32_t output_height;

    platform_image *im_dst_y;
    platform_image *im_cam_scale;
    platform_image *im_therm_scale;
    platform_image *im_cam_l;
    platform_image *im_cam_h;
    platform_image *im_out;
} image_fusion_context;

static image_fusion_context g_img_ctx;

static void mix_mode_ive_init(void)
{
    image_fusion_config *config = &g_img_ctx.config;

    g_img_ctx.im_dst_y = platform_vdsp_create_image(RUNTIME_WIDTH, RUNTIME_HEIGHT);
    g_img_ctx.im_cam_scale = platform_vdsp_create_image(RUNTIME_WIDTH, RUNTIME_HEIGHT);
    g_img_ctx.im_therm_scale = platform_vdsp_create_image(RUNTIME_WIDTH, RUNTIME_HEIGHT);
    g_img_ctx.im_cam_l = platform_vdsp_create_image(RUNTIME_WIDTH, RUNTIME_HEIGHT);
    g_img_ctx.im_cam_h = platform_vdsp_create_image(RUNTIME_WIDTH, RUNTIME_HEIGHT);
    g_img_ctx.im_out = platform_vdsp_create_image(config->target_width, config->target_height);
    if (g_img_ctx.im_dst_y == NULL || g_img_ctx.im_cam_scale == NULL ||
        g_img_ctx.im_therm_scale == NULL || g_img_ctx.im_cam_l == NULL ||
        g_img_ctx.im_cam_h == NULL || g_img_ctx.im_out == NULL) {
        perr("create image failed\n");
        image_fusion_uninit();
        return;
    }
}

static void mix_mode_ive_uninit(void)
{
    platform_vdsp_free_image(g_img_ctx.im_dst_y);
    platform_vdsp_free_image(g_img_ctx.im_cam_scale);
    platform_vdsp_free_image(g_img_ctx.im_therm_scale);
    platform_vdsp_free_image(g_img_ctx.im_cam_l);
    platform_vdsp_free_image(g_img_ctx.im_cam_h);
    platform_vdsp_free_image(g_img_ctx.im_out);
}

static void joint_mode_model_init(void)
{
    // FIXME: 返回失败
    int ret = CVI_NN_RegisterModel("joint_cv181x_f16.cvimodel", &g_img_ctx.model);
    if (ret != CVI_RC_SUCCESS) {
        perr("CVI_NN_RegisterModel failed, err 0x%x\n", ret);
        return;
    }
    pinfo("CVI_NN_RegisterModel succeeded\n");

    // get input output tensors
    CVI_TENSOR *input_tensors;
    CVI_TENSOR *output_tensors;
    int32_t input_num;
    int32_t output_num;
    CVI_NN_GetInputOutputTensors(g_img_ctx.model, &input_tensors, &input_num, &output_tensors,
                                &output_num);
    CVI_TENSOR *input = CVI_NN_GetTensorByName(CVI_NN_DEFAULT_TENSOR, input_tensors, input_num);
    pinfo("input, name: %s, num: %d, type: %d, pix_fmt: %d\n", input->name, input_num, input->fmt, input->pixel_format);
    CVI_TENSOR *output = CVI_NN_GetTensorByName(CVI_NN_DEFAULT_TENSOR, output_tensors, output_num);
    pinfo("output, name: %s, num: %d, type: %d, pix_fmt: %d\n", output->name, output_num, output->fmt, output->pixel_format);

    CVI_SHAPE shape = CVI_NN_TensorShape(input);
    // nchw
    g_img_ctx.input_height = shape.dim[2];
    g_img_ctx.input_width = shape.dim[3];

    shape = CVI_NN_TensorShape(output);
    // nchw
    g_img_ctx.output_height = shape.dim[2];
    g_img_ctx.output_width = shape.dim[3];

    g_img_ctx.input_tensors = input_tensors;
    g_img_ctx.output_tensors = output_tensors;
    g_img_ctx.input = input;
    g_img_ctx.output = output;
}

static void joint_mode_model_uninit(void)
{
    CVI_NN_CleanupModel(g_img_ctx.model);
}

void image_fusion_init(image_fusion_config *config)
{
    if (config == NULL) {
        perr("Param is NULL\n");
        return;
    }

    g_img_ctx.config = *config;
    g_img_ctx.mode = FUSION_MODE_MAX;
    g_img_ctx.colormap = FUSION_COLORMAP_PLASMA;

    joint_mode_model_init();
    mix_mode_ive_init();
}

void image_fusion_uninit(void)
{
    joint_mode_model_uninit();
    mix_mode_ive_uninit();
}

int image_fusion_get_config(image_fusion_config *config)
{
    if (config == NULL) {
        perr("Param is NULL\n");
        return -1;
    }

    *config = g_img_ctx.config;
    return 0;
}

void image_fusion_set_mode(image_fusion_mode mode)
{
    if (mode == g_img_ctx.mode) {
        return;
    }

    g_img_ctx.mode = mode;
}

image_fusion_mode image_fusion_get_mode(void)
{
    return g_img_ctx.mode;
}

void image_fusion_set_colormap(image_fusion_colormap colormap)
{
    g_img_ctx.colormap = colormap;
}

image_fusion_colormap image_fusion_get_colormap(void)
{
    return g_img_ctx.colormap;
}

static ColormapTypes get_cv_color_map(image_fusion_colormap colormap)
{
    ColormapTypes type;
    switch (colormap) {
    case FUSION_COLORMAP_PLASMA:
        type = COLORMAP_PLASMA;
        break;
    case FUSION_COLORMAP_JET:
        type = COLORMAP_JET;
        break;
    default:
        type = COLORMAP_PLASMA;
    }

    return type;
}

static void get_mix_frame_vdsp(image_fusion_config *config, video_buffer *img_buffer, uint8_t *cam_data, uint8_t *therm_data)
{
    platform_image *im_cam_scale = g_img_ctx.im_cam_scale;
    platform_image *im_therm_scale = g_img_ctx.im_therm_scale;
    platform_image *im_cam_l = g_img_ctx.im_cam_l;
    platform_image *im_cam_h = g_img_ctx.im_cam_h;
    platform_image *im_dst_y = g_img_ctx.im_dst_y;
    platform_image *im_out = g_img_ctx.im_out;
    platform_image *im_cam;
    platform_image *im_therm;

    common_probe_1;
    Mat cv_therm(config->thm_height, config->thm_width, CV_8UC1, therm_data);
    Mat cv_therm_flip;
    flip(cv_therm, cv_therm_flip, 1);
    im_therm = platform_vdsp_read_array(cv_therm_flip.data, config->thm_width, config->thm_height);
    platform_vdsp_resize(im_therm_scale, im_therm);
    im_cam = platform_vdsp_read_array(cam_data, config->cam_width, config->cam_height);
    platform_vdsp_resize(im_cam_scale, im_cam);
    common_probe_2;
    pdbg("camera frame resize: %d\n", common_duration_us());

    common_probe_1;
    platform_vdsp_guassian_filter(im_cam_l, im_cam_scale);
    platform_vdsp_sub(im_cam_h, im_cam_scale, im_cam_l);
    platform_vdsp_add(im_dst_y, im_cam_h, im_therm_scale);
    platform_vdsp_resize(im_out, im_dst_y);
    common_probe_2;
    pdbg("guassian: %d\n", common_duration_us());

    common_probe_1;
    Mat cv_result(img_buffer->height, img_buffer->width, CV_8UC3, img_buffer->start);
    Mat cv_dst_y(img_buffer->height, img_buffer->width, CV_8UC1,
        platform_image_get_addr(im_out), platform_image_get_stride(im_out));
    applyColorMap(cv_dst_y, cv_result, get_cv_color_map(g_img_ctx.colormap));
    common_probe_2;
    pdbg("apply color: %d\n", common_duration_us());

    platform_vdsp_free_image(im_cam);
    platform_vdsp_free_image(im_therm);
}

static void get_joint_fusing_frame_tpu(image_fusion_config *config, video_buffer *img_buffer, uint8_t *cam_data, uint8_t *therm_data)
{
    uint32_t width = g_img_ctx.input_width;
    uint32_t height = g_img_ctx.input_height;
    CVI_TENSOR *input = g_img_ctx.input;
    CVI_TENSOR *output = g_img_ctx.output;
    CVI_TENSOR *input_tensors = g_img_ctx.input_tensors;
    CVI_TENSOR *output_tensors = g_img_ctx.output_tensors;

    common_probe_1;
    Mat im_cam(config->cam_height, config->cam_width, CV_8UC1, cam_data);
    Mat im_cam_scale;
    resize(im_cam, im_cam_scale, Size(width, height), 0, 0, INTER_LINEAR);
    common_probe_2;
    pdbg("camera frame resize: %d\n", common_duration_us());

    common_probe_1;
    Mat im_thm(config->thm_height, config->thm_width, CV_8UC1, therm_data);
    Mat im_therm_mirror, im_therm_scale;
    flip(im_thm, im_therm_mirror, 1);
    resize(im_therm_mirror, im_therm_scale, Size(width, height), 0, 0, INTER_LINEAR);
    common_probe_2;
    pdbg("thermal frame flip and resize: %d\n", common_duration_us());

    im_cam = im_cam_scale;
    im_thm = im_therm_scale;
    im_cam.convertTo(im_cam, CV_32FC1, 1, 0);
    im_thm.convertTo(im_thm, CV_32FC1, 1, 0);
    pdbg("convertTo f32\n");

    // fill to input tensor
    float *ptr = (float *)CVI_NN_TensorPtr(input);
    int channel_size = height * width;
    memcpy(ptr, im_cam.data, channel_size * sizeof(float));
    memcpy(ptr + channel_size, im_thm.data, channel_size * sizeof(float));
    pdbg("fill image to input tensor success\n");

    common_probe_1;
    CVI_NN_Forward(g_img_ctx.model, input_tensors, 1, output_tensors, 1);
    common_probe_2;
    pdbg("CVI_NN_Forward succeeded, elipsed: %d\n", common_duration_us());

    ptr = (float *)CVI_NN_TensorPtr(output);
    Mat im_out(height, width, CV_32FC1);
    memcpy(im_out.data, ptr, channel_size * sizeof(float));
    im_out.convertTo(im_out, CV_8UC1, 1, 0);

    common_probe_1;
    Mat im_result(img_buffer->height, img_buffer->width, CV_8UC3, img_buffer->start);
    resize(im_out, im_out, Size(img_buffer->width, img_buffer->height), 0, 0, INTER_LINEAR);
    applyColorMap(im_out, im_result, get_cv_color_map(g_img_ctx.colormap));
    common_probe_2;
    pdbg("apply color: %d\n", common_duration_us());
}

static void get_camera_frame(image_fusion_config *config, video_buffer *img_buffer, uint8_t *cam_data)
{
    common_probe_1;
    Mat im_cam(config->cam_height, config->cam_width, CV_8UC1, cam_data);
    Mat im_cam_scale;
    resize(im_cam, im_cam_scale, Size(config->target_width, config->target_height), 0, 0, INTER_LINEAR);
    common_probe_2;
    pdbg("camera frame resize: %d\n", common_duration_us());

    common_probe_1;
    Mat im_result(img_buffer->height, img_buffer->width, CV_8UC3, img_buffer->start);
    applyColorMap(im_cam_scale, im_result, get_cv_color_map(g_img_ctx.colormap));
    common_probe_2;
    pdbg("apply color: %d\n", common_duration_us());
}

static void get_thermal_frame(image_fusion_config *config, video_buffer *img_buffer, uint8_t *therm_data)
{
    common_probe_1;
    Mat im_therm(config->thm_height, config->thm_width, CV_8UC1, therm_data);
    Mat im_therm_mirror, im_therm_scale;
    flip(im_therm, im_therm_mirror, 1);
    resize(im_therm_mirror, im_therm_scale, Size(config->target_width, config->target_height), 0, 0, INTER_LINEAR);
    common_probe_2;
    pdbg("thermal frame flip and resize: %d\n", common_duration_us());

    common_probe_1;
    Mat im_result(img_buffer->height, img_buffer->width, CV_8UC3, img_buffer->start);
    applyColorMap(im_therm_scale, im_result, get_cv_color_map(g_img_ctx.colormap));
    common_probe_2;
    pdbg("apply color: %d\n", common_duration_us());
}

int image_fusion_get_frame(video_buffer *buf, uint8_t *cam_data, uint32_t cam_len, uint8_t *therm_data, uint32_t therm_len)
{
    image_fusion_config *config = &g_img_ctx.config;
    uint32_t expect_cam_len = config->cam_width * config->cam_height;
    uint32_t expect_therm_len = config->thm_width * config->thm_height;

    if (buf == NULL || cam_data == NULL || therm_data == NULL) {
        perr("Param is NULL\n");
        return -1;
    }

    if (cam_len != expect_cam_len) {
        perr("Error camera data length: %d, expect: %d\n", cam_len, expect_cam_len);
        return -1;
    }
    if (therm_len != expect_therm_len) {
        perr("Error camera data length: %d, expect: %d\n", therm_len, expect_therm_len);
        return -1;
    }

    buf->width = config->target_width;
    buf->height = config->target_height;
    buf->length = config->target_width * config->target_height * 3;
    buf->start = (uint8_t *)malloc(buf->length);
    if (buf->start == NULL) {
        perr("malloc failed\n");
        return -1;
    }

    switch (g_img_ctx.mode) {
    case FUSION_MODE_MIX: {
        pdbg("mix mode\n");
        get_mix_frame_vdsp(config, buf, cam_data, therm_data);
        break;
    }
    case FUSION_MODE_CAMERA:
        pdbg("camera mode\n");
        get_camera_frame(config, buf, cam_data);
        break;
    case FUSION_MODE_THERMAL:
        pdbg("thermal mode\n");
        get_thermal_frame(config, buf, therm_data);
        break;
    case FUSION_MODE_JOINT:
        pdbg("joint mode\n");
        get_joint_fusing_frame_tpu(config, buf, cam_data, therm_data);
        break;
    default:
        perr("Not support mode\n");
        free(buf->start);
        return NULL;
    }

    return 0;
}

void image_fusion_put_frame(video_buffer *buf)
{
    free(buf->start);
}
