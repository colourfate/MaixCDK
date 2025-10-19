#include "platform.h"
#include <unistd.h>
#include "maix_gpio.hpp"
#include "maix_adc.hpp"
#include "maix_pinmap.hpp"
#include "cvi_ive.h"

using namespace maix;
using namespace maix::peripheral;

#define WHEEL_COUNT_THRESHOLD 1

struct platform_image_s {
    IVE_IMAGE_S *ive_img;
};

typedef struct {
    display::Display *disp;
    uint32_t w, h;
    float max_temp, min_temp, center_temp;
    bool rect;
    bool text;
} display_context;

typedef struct {
    pthread_t button_thread_id;
    bool button_poll;
    uint32_t wheel_diff = 0;
    bool usr_btn_act;
    bool pwr_btn_act;
    float batter_vol;
} peri_device_status;

static display_context g_disp_ctx;
static IVE_HANDLE g_ive_handle;
static peri_device_status g_peri_status;

static float get_adc_vol(uint32_t raw_data)
{
    const float vref = 1.5;
    const float r6 = 10.0f;
    const float r10 = 5.1f;
    const float vin_max = 4.81f;

    float measure_vol = raw_data / 4096.0f * vref / (r10 / (r6 + r10));
    pinfo("raw data: %d, measure: %f\n", raw_data, measure_vol);

    return measure_vol + 5 - vin_max;
}

static void *button_poll_thread(void *arg)
{
    pinfo("button thread start\n");

    pinmap::set_pin_function("A24", "GPIOA24");
    pinmap::set_pin_function("A23", "GPIOA23");
    pinmap::set_pin_function("A27", "GPIOA27");
    pinmap::set_pin_function("A26", "GPIOA26");
    pinmap::set_pin_function("A15", "GPIOA15");

    gpio::GPIO usr_btn = gpio::GPIO("GPIOA24", gpio::Mode::IN, gpio::Pull::PULL_NONE);
    gpio::GPIO wheel_a = gpio::GPIO("GPIOA23", gpio::Mode::IN, gpio::Pull::PULL_NONE);
    gpio::GPIO wheel_btn = gpio::GPIO("GPIOA27", gpio::Mode::IN, gpio::Pull::PULL_NONE);
    gpio::GPIO wheel_b = gpio::GPIO("GPIOA26", gpio::Mode::IN, gpio::Pull::PULL_NONE);
    gpio::GPIO charge_check = gpio::GPIO("GPIOA15", gpio::Mode::IN, gpio::Pull::PULL_NONE);

    adc::ADC bat_vol(0, adc::RES_BIT_12);
    float vol = get_adc_vol(bat_vol.read());
    pinfo("Battery voltage: %.2f\n", vol);

    int32_t wheel_counter = 0;
    uint32_t cnt = 0;

    while (g_peri_status.button_poll) {
        static uint32_t a = wheel_a.value();
        static uint32_t b = wheel_b.value();
        static uint32_t u = usr_btn.value();
        static uint32_t p = wheel_btn.value();
        uint32_t a_tmp = wheel_a.value();
        uint32_t b_tmp = wheel_b.value();
        uint32_t u_tmp = usr_btn.value();
        uint32_t p_tmp = wheel_btn.value();

        if (a_tmp != a || b_tmp != b) {
            if (a == 1 && b == 1) {
                if (a_tmp == 1 && b_tmp == 0) {
                    wheel_counter++;
                    pdbg("Anticlockwise: %d\n", wheel_counter);

                } else if (a_tmp == 0 && b_tmp == 1) {
                    wheel_counter--;
                    pdbg("Clockwise: %d\n", wheel_counter);
                }
            }

            pdbg("AB [%d, %d] --> [%d, %d]\n", a, b, a_tmp, b_tmp);
            a = a_tmp;
            b = b_tmp;
        }

        if (wheel_counter == WHEEL_COUNT_THRESHOLD) {
            pdbg("wheel action: 1\n");
            g_peri_status.wheel_diff = 1;
            wheel_counter = 0;
        } else if (wheel_counter == -WHEEL_COUNT_THRESHOLD) {
            pdbg("wheel action: -1\n");
            g_peri_status.wheel_diff = -1;
            wheel_counter = 0;
        }

        if (u_tmp != u) {
            if (u_tmp == 0) {
                pdbg("User button action: %d %d\n", u, u_tmp);
                g_peri_status.usr_btn_act = true;
            }

            u = u_tmp;
        }

        if (p_tmp != p) {
            if (p_tmp == 0) {
                pdbg("Power button action: %d %d\n", p, p_tmp);
                g_peri_status.pwr_btn_act = true;
            }

            p = p_tmp;
        }

        if (cnt++ % 100 == 0) {
            g_peri_status.batter_vol = get_adc_vol(bat_vol.read());
            pinfo("Battery voltage: %.2f\n", vol);
        }

        usleep(10 * 1000);
    }

    return NULL;
}

void platform_device_init(uint32_t img_w, uint32_t img_h)
{
    // init display
    g_disp_ctx.disp = new display::Display();
    err::check_bool_raise(g_disp_ctx.disp->is_opened(), "display open failed");
    g_disp_ctx.rect = true;
    g_disp_ctx.text = true;
    g_disp_ctx.w = img_w;
    g_disp_ctx.h = img_h;

    g_peri_status.button_poll = true;
    pthread_create(&g_peri_status.button_thread_id, NULL, button_poll_thread, NULL);
}

void platform_device_uninit(void)
{
    delete g_disp_ctx.disp;

    g_peri_status.button_poll = false;
    pthread_join(g_peri_status.button_thread_id, NULL);
}

int32_t platform_wheel_get_diff(void)
{
    int32_t wheel_diff = g_peri_status.wheel_diff;
    g_peri_status.wheel_diff = 0;

    return wheel_diff;
}

bool platform_button_get_act(platform_button_type type)
{
    bool *act;

    if (type == BUTTON_TYPE_PWR) {
        act = &g_peri_status.pwr_btn_act;
    } else if (type == BUTTON_TYPE_USER) {
        act = &g_peri_status.usr_btn_act;
    } else {
        perr("Invalid button: %d\n", type);
        return false;
    }

    if (*act) {
        *act = false;
        return true;
    }

    return false;
}

uint32_t platform_batter_get_level(void)
{
    const float full_vol = 4.2;
    const float low_vol = 3.3;
    const float drop_vol = 0.2;

    float per_level = (g_peri_status.batter_vol - (low_vol - drop_vol)) / (full_vol - low_vol);
    per_level = fmin(per_level, 1);

    return (uint32_t)(per_level * 100);
}

// This object(other_disp) is depend on disp, so we must keep disp.show() running.;
display::Display *platform_display_add(void)
{
    display::Display *other_disp = g_disp_ctx.disp->add_channel();
    return other_disp;
}

void platform_display_draw_rect(bool en)
{
    g_disp_ctx.rect = en;
}

void platform_display_draw_text(bool en)
{
    g_disp_ctx.text = en;
}

static inline void show_image(image::Image &img)
{
    static image::Image disp_img = image::Image(552, 368, image::Format::FMT_BGR888);

    // TODO: 使用DMA方式
    disp_img.draw_image(0, 0, img);
    g_disp_ctx.disp->show(disp_img);
}

void platform_display_show(fusion_frame *frame)
{
    static image::Image cam_img = image::Image(g_disp_ctx.w, g_disp_ctx.h, image::Format::FMT_BGR888);

    if (frame == NULL) {
        show_image(cam_img);
        return;
    }

    if (frame->buf.width != g_disp_ctx.w || frame->buf.height != g_disp_ctx.h) {
        perr("Frame(%dx%d) != Init size(%dx%d)\n", frame->buf.width, frame->buf.height, g_disp_ctx.w, g_disp_ctx.h);
        return;
    }

    common_probe_1;
    cam_img.update(frame->buf.width, frame->buf.height, image::Format::FMT_BGR888, frame->buf.start, frame->buf.length, true);
    common_probe_2;
    pdbg("image interval: %d\n", common_duration_us());

    // TODO: Move rect postion
    common_probe_1;
    if (g_disp_ctx.rect) {
        cam_img.draw_rect(frame->info.max_pos[0], frame->info.max_pos[1], (uint32_t)frame->ratio, (uint32_t)frame->ratio, image::COLOR_RED);
        cam_img.draw_rect(frame->info.min_pos[0], frame->info.min_pos[1], (uint32_t)frame->ratio, (uint32_t)frame->ratio, image::COLOR_BLUE);
        cam_img.draw_cross(frame->buf.width / 2, frame->buf.height / 2, image::COLOR_WHITE);
    }
    common_probe_2;
    pdbg("draw interval: %d\n", common_duration_us());

    g_disp_ctx.max_temp = frame->info.max_temp;
    g_disp_ctx.min_temp = frame->info.min_temp;
    g_disp_ctx.center_temp = frame->info.center_temp;

    common_probe_1;
    show_image(cam_img);
    common_probe_2;
    pdbg("image show: %d\n", common_duration_us());
}

void platform_display_get_temp(float *max_temp, float *min_temp, float *center_temp)
{
    if (max_temp == NULL || min_temp == NULL || center_temp == NULL) {
        return;
    }

    *max_temp = g_disp_ctx.max_temp;
    *min_temp = g_disp_ctx.min_temp;
    *center_temp = g_disp_ctx.center_temp;
}

typedef struct IVE_IMAGE_FILL_U8C3_PLANAR_S {
    CVI_U8 u8Type;
    CVI_U32 u32SrcPhyAddr[3];
    CVI_U32 u32DstPhyAddr[3];
    CVI_U16 u16SrcStride[3];
    CVI_U16 u16DstStride[3];
    CVI_U16 u16SrcWidth;
    CVI_U16 u16SrcHeight;
    CVI_U16 u16DstWidth;
    CVI_U16 u16DstHeight;
    CVI_U16 u16XScale;
    CVI_U16 u16YScale;
} IVE_IMAGE_FILL_U8C3_PLANAR_S;

static IVE_MEM_INFO_S g_resize_mem_info;

void *platform_vdsp_init(void)
{
    g_ive_handle = CVI_IVE_CreateHandle();
    if (g_ive_handle == NULL) {
        perr("CVI_IVE_CreateHandle failed\n");
        return NULL;
    }

    CVI_IVE_CreateMemInfo(g_ive_handle, &g_resize_mem_info,
        sizeof(IVE_IMAGE_FILL_U8C3_PLANAR_S) * 2);

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return g_ive_handle;
}

void platform_vdsp_uninit(void *handle)
{
    if (handle == NULL) {
        perr("handle is NULL\n");
        return;
    }

    CVI_SYS_FreeM(handle, &g_resize_mem_info);
    CVI_IVE_DestroyHandle(handle);
    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
}

platform_image *platform_vdsp_create_image(uint32_t w, uint32_t h)
{
    IVE_IMAGE_S *ive_img;
    int ret;

    ive_img = (IVE_IMAGE_S *)malloc(sizeof(IVE_IMAGE_S));
    if (ive_img == NULL) {
        perr("malloc failed\n");
        return NULL;
    }

    ret = CVI_IVE_CreateImage(g_ive_handle, ive_img, IVE_IMAGE_TYPE_U8C1, w, h);
    if (ret != CVI_SUCCESS) {
        perr("CVI_IVE_CreateImage failed: %d\n", ret);
        free(ive_img);
        return NULL;
    }

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");

    return (platform_image *)ive_img;
}

platform_image *platform_vdsp_read_array(uint8_t *src, uint32_t w, uint32_t h)
{
    IVE_IMAGE_S *ive_img;
    int ret;

    if (src == NULL) {
        perr("src or dst_y is NULL\n");
        return NULL;
    }

    ive_img = (IVE_IMAGE_S *)malloc(sizeof(IVE_IMAGE_S));
    if (ive_img == NULL) {
        perr("malloc failed\n");
        return NULL;
    }

    ret = CVI_IVE_ReadImageArray(g_ive_handle, ive_img, (char *)src, IVE_IMAGE_TYPE_U8C1, w, h);
    if (ret != CVI_SUCCESS) {
        perr("CVI_IVE_ReadImageArray failed: %d\n", ret);
        return NULL;
    }

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return (platform_image *)ive_img;
}

void platform_vdsp_free_image(platform_image *img)
{
    if (img == NULL) {
        perr("img is NULL\n");
        return;
    }

    CVI_SYS_FreeI(g_ive_handle, (IVE_IMAGE_S *)img);
    free(img);

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
}

int platform_vdsp_guassian_filter(platform_image *plt_dst_y, platform_image *plt_src_y)
{
    int ret;
	IVE_DST_IMAGE_S *dst_y = (IVE_DST_IMAGE_S *)plt_dst_y;
    IVE_IMAGE_S *src_y = (IVE_IMAGE_S *)plt_src_y;
	IVE_FILTER_CTRL_S iveFltCtrl;
    CVI_S8 arr5by5[25] = {
		1, 2, 3, 2, 1,
        2, 5, 6, 5, 2,
        3, 6, 8, 6, 3,
        2, 5, 6, 5, 2,
        1, 2, 3, 2, 1,
	};

    if (src_y == NULL || dst_y == NULL) {
        perr("src_y or dst_y is NULL\n");
        return -1;
    }

    memcpy(iveFltCtrl.as8Mask, arr5by5, 5 * 5 * sizeof(CVI_S8));
	iveFltCtrl.u8Norm = 7;

	ret = CVI_IVE_Filter(g_ive_handle, src_y, dst_y, &iveFltCtrl, 0);
    if (ret != CVI_SUCCESS) {
        perr("CVI_IVE_Filter failed: %d\n", ret);
        return -1;
    }

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return 0;
}

int platform_vdsp_sub(platform_image *plt_dst_y, platform_image *plt_src1_y, platform_image *plt_src2_y)
{
    int ret;
    IVE_DST_IMAGE_S *dst_y = (IVE_DST_IMAGE_S *)plt_dst_y;
    IVE_IMAGE_S *src1_y = (IVE_IMAGE_S *)plt_src1_y;
    IVE_IMAGE_S *src2_y = (IVE_IMAGE_S *)plt_src2_y;
    IVE_SUB_CTRL_S sub_ctrl = { .enMode = IVE_SUB_MODE_ABS };

    if (src1_y == NULL || src2_y == NULL || dst_y == NULL) {
        perr("src_y or src2_y or dst_y is NULL\n");
        return -1;
    }

    ret = CVI_IVE_Sub(g_ive_handle, src1_y, src2_y, dst_y, &sub_ctrl, 0);
    if (ret != CVI_SUCCESS) {
        perr("CVI_IVE_Sub failed: %d\n", ret);
        return -1;
    }

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return 0;
}

int platform_vdsp_add(platform_image *plt_dst_y, platform_image *plt_src1_y, platform_image *plt_src2_y)
{
    int ret;
    IVE_DST_IMAGE_S *dst_y = (IVE_DST_IMAGE_S *)plt_dst_y;
    IVE_IMAGE_S *src1_y = (IVE_IMAGE_S *)plt_src1_y;
    IVE_IMAGE_S *src2_y = (IVE_IMAGE_S *)plt_src2_y;
    IVE_ADD_CTRL_S add_ctrl = { .u0q16X = 65535, .u0q16Y = 65535 };

    if (src1_y == NULL || src2_y == NULL || dst_y == NULL) {
        perr("src1_y, src2_y or dst_y is NULL\n");
        return -1;
    }

    ret = CVI_IVE_Add(g_ive_handle, src1_y, src2_y, dst_y, &add_ctrl, 0);
    if (ret != CVI_SUCCESS) {
        perr("CVI_IVE_Sub failed: %d\n", ret);
        return -1;
    }

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return 0;
}

int platform_vdsp_resize(platform_image *plt_dst_y, platform_image *plt_src_y)
{
    int ret;
    IVE_DST_IMAGE_S *dst_y = (IVE_DST_IMAGE_S *)plt_dst_y;
    IVE_IMAGE_S *src_y = (IVE_IMAGE_S *)plt_src_y;
    IVE_RESIZE_CTRL_S resize_ctrl = {
        .enMode = IVE_RESIZE_MODE_LINEAR,
        .u16Num = 1,
    };

    if (src_y == NULL || dst_y == NULL) {
        perr("src_y or dst_y is NULL\n");
        return -1;
    }

    resize_ctrl.stMem = g_resize_mem_info;
    ret = CVI_IVE_Resize(g_ive_handle, src_y, dst_y, &resize_ctrl, 0);
    if (ret != CVI_SUCCESS) {
        perr("CVI_IVE_Resize failed: %d\n", ret);
        return -1;
    }

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return 0;
}

int platform_vdsp_flip(platform_image *plt_dst_y, platform_image *plt_src_y)
{
    int ret;
    IVE_DST_IMAGE_S *dst_y = (IVE_DST_IMAGE_S *)plt_dst_y;
    IVE_IMAGE_S *src_y = (IVE_IMAGE_S *)plt_src_y;
    IVE_DATA_S src_data, dst_data;
    IVE_DMA_CTRL_S dma_ctrl = {
        .enMode = IVE_DMA_MODE_DIRECT_COPY,
    };

    if (src_y == NULL || dst_y == NULL) {
        perr("src_y or dst_y is NULL\n");
        return -1;
    }

    // FixMe: 检查宽高，stride等
    src_data.u32Stride = src_y->u32Stride[0];
    src_data.u32Width = src_y->u32Width;
    src_data.u32Height = 1;
    dst_data.u32Stride = dst_y->u32Stride[0];
    dst_data.u32Width = dst_y->u32Width;
    dst_data.u32Height = 1;
    for (uint32_t i = 0; i < src_y->u32Height; i++) {
        uint32_t src_pos = i * src_y->u32Stride[0];
        uint32_t dst_pos = (src_y->u32Height - 1 - i) * dst_y->u32Stride[0];

        src_data.u64PhyAddr = src_y->u64PhyAddr[0] + src_pos;
        src_data.u64VirAddr = src_y->u64VirAddr[0] + src_pos;
        dst_data.u64PhyAddr = dst_y->u64PhyAddr[0] + dst_pos;
        dst_data.u64VirAddr = dst_y->u64VirAddr[0] + dst_pos;
        ret = CVI_IVE_DMA(g_ive_handle, &src_data, &dst_data, &dma_ctrl, 0);
        if (ret != CVI_SUCCESS) {
            perr("CVI_IVE_DMA failed: %d\n", ret);
            return -1;
        }
    }

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return 0;
}

uint32_t platform_image_get_stride(platform_image *img)
{
    if (img == NULL) {
        perr("img is NULL\n");
        return 0;
    }

    return ((IVE_IMAGE_S *)img)->u32Stride[0];
}

uint8_t *platform_image_get_addr(platform_image *img)
{
    if (img == NULL) {
        perr("img is NULL\n");
        return NULL;
    }

    return (uint8_t *)((IVE_IMAGE_S *)img)->u64VirAddr[0];
}

void platfrom_image_save(platform_image *plt_dst, char *path)
{
    IVE_IMAGE_S *img = (IVE_IMAGE_S *)plt_dst;
    CVI_IVE_WriteImg(g_ive_handle, path, img);
}

// No implement
int platform_vdsp_mirror(platform_image *plt_dst_y, platform_image *plt_src_y)
{
    int ret;
    IVE_DST_IMAGE_S *dst_y = (IVE_DST_IMAGE_S *)plt_dst_y;
    IVE_IMAGE_S *src_y = (IVE_IMAGE_S *)plt_src_y;
    IVE_DATA_S src_data, dst_data;
    IVE_DMA_CTRL_S dma_ctrl = {
        .enMode = IVE_DMA_MODE_DIRECT_COPY,
    };

    if (src_y == NULL || dst_y == NULL) {
        perr("src_y or dst_y is NULL\n");
        return -1;
    }

    common_probe_1;
    // FixMe: 检查宽高，stride等
    src_data.u32Stride = 1;
    src_data.u32Width = 1;
    src_data.u32Height = src_y->u32Height;
    dst_data.u32Stride = 1;
    dst_data.u32Width = 1;
    dst_data.u32Height = dst_y->u32Height;
    for (uint32_t i = 0; i < src_y->u32Width; i++) {
        uint32_t src_pos = i;
        uint32_t dst_pos = src_y->u32Width - 1 - i;

        src_data.u64PhyAddr = src_y->u64PhyAddr[0] + src_pos;
        src_data.u64VirAddr = src_y->u64VirAddr[0] + src_pos;
        dst_data.u64PhyAddr = dst_y->u64PhyAddr[0] + dst_pos;
        dst_data.u64VirAddr = dst_y->u64VirAddr[0] + dst_pos;
        ret = CVI_IVE_DMA(g_ive_handle, &src_data, &dst_data, &dma_ctrl, 0);
        if (ret != CVI_SUCCESS) {
            perr("CVI_IVE_DMA failed: %d\n", ret);
            return -1;
        }
    }
    common_probe_2;
    pdbg("mirror interval: %d\n", common_duration_us());

    //system("cat /sys/kernel/debug/dma_buf/bufinfo");
    return 0;
}
