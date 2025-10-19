#include <cstdint>
#ifndef _IMAGE_FUSION_H_

#include "common.h"

typedef enum {
    FUSION_MODE_MIX,
    FUSION_MODE_CAMERA,
    FUSION_MODE_THERMAL,
    FUSION_MODE_JOINT,
    FUSION_MODE_MAX
} image_fusion_mode;

typedef enum {
    HIGH_FREQ_RATIO_NONE,
    HIGH_FREQ_RATIO_LOW,
    HIGH_FREQ_RATIO_MEDIUM,
    HIGH_FREQ_RATIO_HIGH,
    HIGH_FREQ_RATIO_MAX
} image_high_freq_ratio;

typedef enum {
    FUSION_COLORMAP_PLASMA,
    FUSION_COLORMAP_JET,
    FUSION_COLORMAP_MAX
} image_fusion_colormap;

typedef struct {
    uint32_t cam_width, cam_height;
    uint32_t thm_width, thm_height;
    uint32_t target_width, target_height;
} image_fusion_config;

void image_fusion_init(image_fusion_config *config);
void image_fusion_uninit(void);
void image_fusion_set_mode(image_fusion_mode mode);
int image_fusion_get_config(image_fusion_config *config);
void image_fusion_set_colormap(image_fusion_colormap colormap);
image_fusion_colormap image_fusion_get_colormap(void);
image_fusion_mode image_fusion_get_mode(void);
int image_fusion_get_frame(video_buffer *buf, uint8_t *cam_data, uint32_t cam_len,
    uint8_t *therm_data, uint32_t therm_len);
void image_fusion_put_frame(video_buffer *buf);

#endif