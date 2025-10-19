#ifndef _UVC_CAMERA_H_
#define _UVC_CAMERA_H_

#include "common.h"

int uvc_camera_open(void);
video_buffer *uvc_camera_capture_frame(void);
int uvc_camera_release_frame(video_buffer *buffer);
void uvc_camera_close(void);

#endif
