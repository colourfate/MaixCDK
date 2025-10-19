#ifndef _COMMON_H_
#define _COMMON_H_

#include "elog.h"
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <stdint.h>
#include <stdio.h>

#define perr log_e
#define pwarn log_w
#define pinfo log_i
#define pdbg log_d

static struct timeval _tv1, _tv2;
#define common_probe_1 gettimeofday(&_tv1, NULL)
#define common_probe_2 gettimeofday(&_tv2, NULL)
static inline int common_duration_us(void)
{

    int s = _tv2.tv_sec - _tv1.tv_sec;
    int u = _tv2.tv_usec - _tv1.tv_usec;

    return s * 1000000 + u;
}

typedef struct video_buffer {
    uint8_t *start;
    uint32_t length;
    uint32_t width;
    uint32_t height;
} video_buffer;

static inline void common_show_mat(cv::Mat &mat)
{
    printf("image width: %d, height: %d, deep: %d, elem size: %zu, total: %zu, data: %p\n",
        mat.cols, mat.rows, mat.channels(), mat.elemSize(), mat.total(), mat.data);
}

void common_init(void);

#endif