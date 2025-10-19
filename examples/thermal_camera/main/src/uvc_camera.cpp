#include "uvc_camera.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>              /* low-level i/o */
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

typedef struct {
    video_buffer buf;
    struct v4l2_buffer info;
} video_buffer_desc;

static const int CAP_BUF_NUM = 4;
static video_buffer_desc *g_all_bufs;
static int fd;
static uint32_t g_cam_width, g_cam_height;

static void setup_camera_format(void)
{
    //输出所有支持的格式
    struct v4l2_fmtdesc fmtdesc;
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    pinfo("Support format:\n");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        pinfo("\t%d.%s\n", fmtdesc.index + 1, fmtdesc.description);
        fmtdesc.index++;
    }
    pinfo("enum done\n");

    //查看当前的输出格式
    struct v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_G_FMT, &fmt);

    pinfo("Current data format information : \n\twidth: % d\n\theight: % d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
    g_cam_width = fmt.fmt.pix.width;
    g_cam_height = fmt.fmt.pix.height;

    struct v4l2_fmtdesc fmtdesc2;
    fmtdesc2.index = 0;
    fmtdesc2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc2) != -1) {
        if (fmtdesc2.pixelformat & fmt.fmt.pix.pixelformat) {
            pinfo("\tformat: %s\n", fmtdesc2.description);
            break;
        }
        fmtdesc2.index++;
    }
}

int uvc_camera_open()
{
    fd = open("/dev/video0", O_RDWR);

    setup_camera_format();

    //设置视频格式
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    //设置设备捕获视频的格式
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perr("set format failed\n");
        close(fd);
        return -1;
    }
    //如果摄像头不支持我们设置的分辨率格式，则 fmt.fmt.pix.width 会被修改，所以此处建议再次检查 fmt.fmt.pix. 的各种信息

    //向驱动申请帧缓存
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = CAP_BUF_NUM;  //申请一个拥有四个缓冲帧的缓冲区
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        if (EINVAL == errno) {
            perr(" does not support memory mapping\n");
            close(fd);
            return -1;
        } else {
            perr("does not support memory mapping, unknow error\n");
            close(fd);
            return -1;
        }
    } else {
        pinfo("alloc success\n");
    }
    if (req.count < CAP_BUF_NUM) {
        perr("Insufficient buffer memory\n");
        close(fd);
        return -1;
    } else {
        pinfo("get %d bufs\n", req.count);
    }

    g_all_bufs = (video_buffer_desc *)calloc(req.count, sizeof(video_buffer_desc));
    struct v4l2_buffer buf;
    for (uint32_t numBufs = 0; numBufs < req.count; numBufs++) {//映射所有的缓存
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = numBufs;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {//获取到对应index的缓存信息，此处主要利用length信息及offset信息来完成后面的mmap操作。
            perr("unexpect error %d\n", numBufs);
            free(g_all_bufs);
            close(fd);
            return -1;
        }

        g_all_bufs[numBufs].buf.length = buf.length;
        g_all_bufs[numBufs].buf.width = g_cam_width;
        g_all_bufs[numBufs].buf.height = g_cam_height;
        // 转换成相对地址
        g_all_bufs[numBufs].buf.start = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset); // #include <sys/mman.h>
        if (g_all_bufs[numBufs].buf.start == MAP_FAILED) {
            printf("%d map failed errno %d\n", numBufs, errno);
            free(g_all_bufs);
            close(fd);
            return -1;
        }
        //addr 映射起始地址，一般为NULL ，让内核自动选择
        //prot 标志映射后能否被读写，其值为PROT_EXEC,PROT_READ,PROT_WRITE, PROT_NONE
        //flags 确定此内存映射能否被其他进程共享，MAP_SHARED,MAP_PRIVATE
        //fd,offset, 确定被映射的内存地址 返回成功映射后的地址，不成功返回MAP_FAILED ((void*)-1)
        //int munmap(void* addr, size_t length);// 最后记得断开映射

        //把缓冲帧加入缓冲队列
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perr("add buf to queue failed %d, errno: %d\n", numBufs, errno);
            free(g_all_bufs);
            close(fd);
            return -1;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    /* 打开设备视频流 */
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perr("stream open failed, errno: %d\n", errno);
        free(g_all_bufs);
        close(fd);
        return -1;
    }

    return 0;
}

void uvc_camera_close(void)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    //清理资源
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < CAP_BUF_NUM; i++) {
        munmap(g_all_bufs[i].buf.start, g_all_bufs[i].buf.length);
    }
    free(g_all_bufs);
    close(fd);
}

video_buffer *uvc_camera_capture_frame(void)
{
    struct v4l2_buffer capture_buf;

    memset(&capture_buf, 0, sizeof(capture_buf));
    capture_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    capture_buf.memory = V4L2_MEMORY_MMAP;
    /* 将已经捕获好视频的内存拉出已捕获视频的队列 */
    if (ioctl(fd, VIDIOC_DQBUF, &capture_buf) < 0) {
        perr("get frame failed\n");
        return NULL;
    }

    g_all_bufs[capture_buf.index].info = capture_buf;
    return &g_all_bufs[capture_buf.index].buf;
}

int uvc_camera_release_frame(video_buffer *buffer)
{
    video_buffer_desc *desc = (video_buffer_desc *)buffer;

    if (ioctl(fd, VIDIOC_QBUF, &desc->info) == -1) {
        perr("insert buf failed\n");
        return -1;
    }

    return 0;
}
