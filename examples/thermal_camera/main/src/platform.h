#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "maix_basic.hpp"
#include "maix_display.hpp"
#include "stdint.h"
#include "thermal_sensor.h"
#include "common.h"

typedef struct {
    video_buffer buf;
    thermal_info info;
    // TODO: Support x,y ratio
    float ratio;
} fusion_frame;

typedef enum {
    BUTTON_TYPE_USER,
    BUTTON_TYPE_PWR,
    BUTTON_TYPE_MAX,
} platform_button_type;

typedef struct platform_image_s platform_image;

void platform_device_init(uint32_t img_w, uint32_t img_h);
void platform_device_uninit(void);

maix::display::Display *platform_display_add(void);
void platform_display_show(fusion_frame *frame);
void platform_display_draw_rect(bool en);
void platform_display_get_temp(float *max_temp, float *min_temp, float *center_temp);

int32_t platform_wheel_get_diff(void);
bool platform_button_get_act(platform_button_type type);
uint32_t platform_batter_get_level(void);

void *platform_vdsp_init(void);
void platform_vdsp_uninit(void *handle);
platform_image *platform_vdsp_create_image(uint32_t w, uint32_t h);
platform_image *platform_vdsp_read_array(uint8_t *src, uint32_t w, uint32_t h);
void platform_vdsp_free_image(platform_image *img);

int platform_vdsp_guassian_filter(platform_image *plt_dst_y, platform_image *src_y);
int platform_vdsp_sub(platform_image *plt_dst_y, platform_image *plt_src1_y, platform_image *plt_src2_y);
int platform_vdsp_add(platform_image *plt_dst_y, platform_image *plt_src1_y, platform_image *plt_src2_y);
int platform_vdsp_resize(platform_image *plt_dst_y, platform_image *plt_src_y);
int platform_vdsp_flip(platform_image *plt_dst_y, platform_image *plt_src_y);
uint32_t platform_image_get_stride(platform_image *img);
uint8_t *platform_image_get_addr(platform_image *img);
void platfrom_image_save(platform_image *plt_dst, char *path);
int platform_vdsp_mirror(platform_image *plt_dst_y, platform_image *plt_src_y);
#endif