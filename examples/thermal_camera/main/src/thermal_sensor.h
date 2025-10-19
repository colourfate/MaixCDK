#ifndef _THERMAL_SENSOR_H_
#define _THERMAL_SENSOR_H_

#define THERMAL_FRAME_WIDTH 32
#define THERMAL_FRAME_HEIGHT 24
#define SENSOR_FRAME_SIZE (THERMAL_FRAME_WIDTH * THERMAL_FRAME_HEIGHT)
#include <stdint.h>

typedef struct {
    float max_temp;
    float min_temp;
    float center_temp;
    uint32_t max_pos[2];
    uint32_t min_pos[2];
    uint32_t center_pos[2];
} thermal_info;

typedef enum {
    SENSOR_EVENT_CMD,
    SENSOR_EVENT_DATA,
    SENSOR_EVENT_LOG,
    SENSOR_EVENT_MAX
} thermal_event;

// Use libusb
int thermal_sensor_open(void);
void thermal_sensor_test(void);
float *thermal_sensor_read_frame(thermal_info *info);
void thermal_sensor_start(void);
void thermal_sensor_set_fps(uint8_t fps);
void thermal_sensor_close(void);

// Use linux acm drvier
int thermal_sensor_open2(void);
float *thermal_sensor_read_frame2(thermal_info *info);
void thermal_sensor_start2(void);
void thermal_sensor_set_fps2(uint8_t fps);
void thermal_sensor_close2(void);
float *thermal_sensor_read_packet(thermal_info *info);
// NOTE: Not implemented
thermal_event thermal_sensor_event_process(void);
#endif