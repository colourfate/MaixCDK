#include "thermal_sensor.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include "maix_basic.hpp"
#include "maix_gpio.hpp"
#include "maix_adc.hpp"
#include "maix_pinmap.hpp"
#include "libusb.h"
#include "common.h"

#define USB_DATA_HEAD 0x8000
typedef enum {
    USB_DATA_TYPE_CMD,
    USB_DATA_TYPE_DATA,
    USB_DATA_TYPE_LOG,
    USB_DATA_TYPE_MAX
} usb_data_type;

typedef enum {
	USB_CMD_START,
	USB_CMD_FPS,
	USB_CMD_TYPE_MAX
} usb_cmd_type;

typedef struct {
    uint16_t header;
    uint16_t type;
    uint16_t len;
    uint8_t data[];
} usb_data_format;

#define FRAME_DATA_LEN SENSOR_FRAME_SIZE * 2 + sizeof(usb_data_format)

/* You may want to change the VENDOR_ID and PRODUCT_ID
 * depending on your device.
 */
#define VENDOR_ID      0x0483
#define PRODUCT_ID     0x5740

#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02

/* We use a global variable to keep the device handle
 */
static struct libusb_device_handle *devh = NULL;

/* The Endpoint address are hard coded. You should use lsusb -v to find
 * the values corresponding to your device.
 */
static int ep_in_addr  = 0x81;
static int ep_out_addr = 0x01;
static uint32_t g_max_packet_size;

static void write_chars(unsigned char *data, int size)
{
    int actual_length;
    if (libusb_bulk_transfer(devh, ep_out_addr, data, size,
                             &actual_length, 0) < 0) {
        fprintf(stderr, "Error while sending char, actual length: %d\n", actual_length);
    }
}

static void write_char(unsigned char c)
{
    /* To send a char to the device simply initiate a bulk_transfer to the
     * Endpoint with address ep_out_addr.
     */
    int actual_length;
    if (libusb_bulk_transfer(devh, ep_out_addr, &c, 1,
                             &actual_length, 0) < 0) {
        fprintf(stderr, "Error while sending char\n");
    }
}

static int read_chars(unsigned char * data, int size)
{
    /* To receive characters from the device initiate a bulk_transfer to the
     * Endpoint with address ep_in_addr.
     */
    int actual_length;
    int rc = libusb_bulk_transfer(devh, ep_in_addr, data, size, &actual_length, 30);
    if (rc == LIBUSB_ERROR_TIMEOUT) {
        pinfo("timeout (%d)\n", actual_length);
        return -1;
    } else if (rc < 0) {
        perr("Error while waiting for char, rc: %d, actual_length: %d\n", rc, actual_length);
        return -1;
    }
    pdbg("read %d bytes\n", actual_length);

    return actual_length;
}

int thermal_sensor_open(void)
{
    int rc;

    /* Initialize libusb
     */
    rc = libusb_init(NULL);
    if (rc < 0) {
        fprintf(stderr, "Error initializing libusb: %s\n", libusb_error_name(rc));
        exit(1);
    }

    /* Set debugging output to max level.
     */
    libusb_set_debug(NULL, 3);

    /* Look for a specific device and open it.
     */
    devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!devh) {
        fprintf(stderr, "Error finding USB device\n");
        goto out;
    }

    /* As we are dealing with a CDC-ACM device, it's highly probable that
     * Linux already attached the cdc-acm driver to this device.
     * We need to detach the drivers from all the USB interfaces. The CDC-ACM
     * Class defines two interfaces: the Control interface and the
     * Data interface.
     */
    for (int if_num = 0; if_num < 2; if_num++) {
        if (libusb_kernel_driver_active(devh, if_num)) {
            libusb_detach_kernel_driver(devh, if_num);
        }
        rc = libusb_claim_interface(devh, if_num);
        if (rc < 0) {
            fprintf(stderr, "Error claiming interface: %s\n",
                    libusb_error_name(rc));
            goto out;
        }
    }

    g_max_packet_size = libusb_get_max_packet_size(libusb_get_device(devh), ep_in_addr & 0x7f);
    pinfo("max packet size: %d\n", g_max_packet_size);

    libusb_clear_halt(devh, ep_in_addr);

    return 0;

out:
    if (devh)
        libusb_close(devh);
    libusb_exit(NULL);

    return -1;
}

void thermal_sensor_close(void)
{
    libusb_release_interface(devh, 0);
    if (devh)
        libusb_close(devh);
    libusb_exit(NULL);
}

static uint8_t g_rx_buf[FRAME_DATA_LEN] = {0};

void thermal_sensor_test(void)
{
    uint32_t len;

    len = read_chars(g_rx_buf, sizeof(g_rx_buf));
    if (len != sizeof(g_rx_buf)) {
        perr("Read size error: %d\n", len);
        return;
    }

    printf("Read frame:\n");
    for (uint32_t i = 0; i < sizeof(g_rx_buf); i++) {
        printf("%02x ", g_rx_buf[i]);
    }
    printf("\n");
}

static float g_frame[SENSOR_FRAME_SIZE];

float *thermal_sensor_read_frame(thermal_info *info)
{
    int ret;
    usb_data_format *usb_format;
    uint32_t i;

    if (info == NULL) {
        perr("Param is NULL\n");
        return NULL;
    }

    /*
    ret = read_bulk_data(g_rx_buf, sizeof(g_rx_buf));
    if (ret != 0) {
        perr("Read device failed\n");
        return NULL;
    }
    */
    ret = read_chars(g_rx_buf, sizeof(g_rx_buf));
    if (ret <= 0) {
        perr("Read device failed: %d\n", ret);
        return NULL;
    }

    usb_format = (usb_data_format *)g_rx_buf;
    if (usb_format->header != USB_DATA_HEAD) {
        perr("Error data header: %#x\n", usb_format->header);
        return NULL;
    }
    if (usb_format->type != USB_DATA_TYPE_DATA) {
        //pwarn("Not data type: %d\n", usb_format->type);
        if (usb_format->type == USB_DATA_TYPE_LOG) {
            printf("-S- %s", usb_format->data);
        }
        return NULL;
    }
    if (usb_format->len != SENSOR_FRAME_SIZE * 2) {
        perr("Error data len: %d\n", usb_format->len);
        return NULL;
    }

    for (i = 0; i < SENSOR_FRAME_SIZE; i++) {
        uint16_t low_byte = usb_format->data[i * 2];
        uint16_t high_byte = usb_format->data[i * 2 + 1];

        g_frame[i] = ((int)(low_byte | (high_byte << 8))) / 100.0f;

        if (i == 0) {
            info->max_temp = info->min_temp = g_frame[0];
            info->max_pos[0] = info->max_pos[1] = 0;
            info->min_pos[0] = info->min_pos[1] = 0;
            continue;
        }

        if (g_frame[i] > info->max_temp) {
            info->max_temp = g_frame[i];
            info->max_pos[0] = i % THERMAL_FRAME_WIDTH;
            info->max_pos[1] = i / THERMAL_FRAME_WIDTH;
        }
        if (g_frame[i] < info->min_temp) {
            info->min_temp = g_frame[i];
            info->min_pos[0] = i % THERMAL_FRAME_WIDTH;
            info->min_pos[1] = i / THERMAL_FRAME_WIDTH;
        }
    }

    return g_frame;
}

void thermal_sensor_start(void)
{
    uint8_t tmp[7];
    usb_data_format *usb_format = (usb_data_format *)tmp;

    usb_format->header = USB_DATA_HEAD;
    usb_format->type = USB_DATA_TYPE_CMD;
    usb_format->len = 1;
    usb_format->data[0] = USB_CMD_START;

    pinfo("Thermal sensor start\n");
    write_chars(tmp, sizeof(tmp));
}

void thermal_sensor_set_fps(uint8_t fps)
{
    uint8_t tmp[8];
    usb_data_format *usb_format = (usb_data_format *)tmp;

    usb_format->header = USB_DATA_HEAD;
    usb_format->type = USB_DATA_TYPE_CMD;
    usb_format->len = 2;
    usb_format->data[0] = USB_CMD_FPS;
    usb_format->data[1] = fps;

    pinfo("Set thermal fps to %d\n", fps);
    write_chars(tmp, sizeof(tmp));
}

static int open_serial_port2(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        perror("open_serial_port: Unable to open port");
    } else {
        fcntl(fd, F_SETFL, 0);
    }

    return fd;
}

static void configure_serial_port2(int fd) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    pinfo("\n");
    if (tcgetattr(fd, &tty) != 0) {
        perror("configure_serial_port: Error from tcgetattr");
        return;
    }
    cfsetospeed(&tty, B2000000);
    cfsetispeed(&tty, B2000000);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;

    tty.c_iflag &= ~(INLCR|ICRNL);     // Linux会把0x0d替换成0x0a，加上这个就不会了
    tty.c_iflag &= ~(IXON);            // 解决缺少了0x11 0x13问题
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("configure_serial_port: Error from tcsetattr");
    }
}

using namespace maix::peripheral;
using namespace maix;
static int g_serial_fd = -1;
gpio::GPIO *g_pwr_pin;

int thermal_sensor_open2(void)
{
    const char *port = "/dev/ttyACM0";
    uint32_t cnt = 0;

    g_serial_fd = open_serial_port2(port);
    if (g_serial_fd != -1) {
        pinfo("Open serial port ok\n");
        goto done;
    }

    pinmap::set_pin_function("A29", "GPIOA29");
    g_pwr_pin = new gpio::GPIO("GPIOA29", gpio::Mode::OUT, gpio::Pull::PULL_NONE);
    do {
        pinfo("Sensor power down\n");
        g_pwr_pin->value(0);
        time::sleep_ms(1000);
        pinfo("Sensor power on\n");
        g_pwr_pin->value(1);
        pinfo("Wait sensor init\n");
        time::sleep_ms(5000);

        g_serial_fd = open_serial_port2(port);
        if (cnt++ > 3) {
            perr("Open serial port failed\n");
            delete g_pwr_pin;
            return -1;
        }
    } while (g_serial_fd == -1);

done:
    configure_serial_port2(g_serial_fd);
    return 0;
}

void thermal_sensor_start2(void)
{
    uint8_t tmp[7];
    usb_data_format *usb_format = (usb_data_format *)tmp;

    usb_format->header = USB_DATA_HEAD;
    usb_format->type = USB_DATA_TYPE_CMD;
    usb_format->len = 1;
    usb_format->data[0] = USB_CMD_START;

    pinfo("Thermal sensor start\n");
    write(g_serial_fd, tmp, sizeof(tmp));
}

float *thermal_sensor_read_frame2(thermal_info *info)
{
    usb_data_format *usb_format;
    uint32_t i;

    if (info == NULL) {
        perr("Param is NULL\n");
        return NULL;
    }

    int exp_len = sizeof(g_rx_buf);
    uint8_t *p = g_rx_buf;
    while (exp_len > 0) {
        int n = read(g_serial_fd, p, exp_len);
        if (n < 0) {
            perr("Failed to read from serial port: errno: %d\n", errno);
            return NULL;
        }
        pdbg("%x %x, read: %d, last: %d\n", p[0], p[1], n, exp_len);
        exp_len -= n;
        p += n;

    }

    usb_format = (usb_data_format *)g_rx_buf;
    if (usb_format->header != USB_DATA_HEAD) {
        perr("Error data header: %#x\n", usb_format->header);
        return NULL;
    }
    if (usb_format->type != USB_DATA_TYPE_DATA) {
        perr("Not data type: %d\n", usb_format->type);
        return NULL;
    }
    if (usb_format->len != SENSOR_FRAME_SIZE * 2) {
        perr("Error data len: %d\n", usb_format->len);
        return NULL;
    }

    for (i = 0; i < SENSOR_FRAME_SIZE; i++) {
        uint16_t low_byte = usb_format->data[i * 2];
        uint16_t high_byte = usb_format->data[i * 2 + 1];

        g_frame[i] = ((int)(low_byte | (high_byte << 8))) / 100.0f;

        if (i == 0) {
            info->max_temp = info->min_temp = g_frame[0];
            info->max_pos[0] = info->max_pos[1] = 0;
            info->min_pos[0] = info->min_pos[1] = 0;
            continue;
        }

        if (g_frame[i] > info->max_temp) {
            info->max_temp = g_frame[i];
            info->max_pos[0] = i % THERMAL_FRAME_WIDTH;
            info->max_pos[1] = i / THERMAL_FRAME_WIDTH;
        }
        if (g_frame[i] < info->min_temp) {
            info->min_temp = g_frame[i];
            info->min_pos[0] = i % THERMAL_FRAME_WIDTH;
            info->min_pos[1] = i / THERMAL_FRAME_WIDTH;
        }
    }

    return g_frame;
}

void thermal_sensor_set_fps2(uint8_t fps)
{
    uint8_t tmp[8];
    usb_data_format *usb_format = (usb_data_format *)tmp;

    usb_format->header = USB_DATA_HEAD;
    usb_format->type = USB_DATA_TYPE_CMD;
    usb_format->len = 2;
    usb_format->data[0] = USB_CMD_FPS;
    usb_format->data[1] = fps;

    pinfo("Set thermal fps to %d\n", fps);
    write(g_serial_fd, tmp, sizeof(tmp));
}

void thermal_sensor_close2(void)
{
    close(g_serial_fd);
    g_serial_fd = -1;

    delete g_pwr_pin;
}

static float calculate_center_temp(float *frame)
{
    float center[4];
    uint32_t x, y;

    x = THERMAL_FRAME_HEIGHT / 2 - 1;
    y = THERMAL_FRAME_WIDTH / 2 - 1;
    center[0] = g_frame[x * THERMAL_FRAME_WIDTH + y];
    y = THERMAL_FRAME_WIDTH / 2;
    center[1] = g_frame[x * THERMAL_FRAME_WIDTH + y];

    x = THERMAL_FRAME_HEIGHT / 2;
    y = THERMAL_FRAME_WIDTH / 2 - 1;
    center[2] = g_frame[x * THERMAL_FRAME_WIDTH + y];
    y = THERMAL_FRAME_WIDTH / 2;
    center[3] = g_frame[x * THERMAL_FRAME_WIDTH + y];

    return (center[0] + center[1] + center[2] + center[3]) / 4.0f;
}

static void process_cmd_data(thermal_info *info, usb_data_format *usb_format)
{
    uint32_t i;

    for (i = 0; i < SENSOR_FRAME_SIZE; i++) {
        uint16_t low_byte = usb_format->data[i * 2];
        uint16_t high_byte = usb_format->data[i * 2 + 1];

        g_frame[i] = ((int)(low_byte | (high_byte << 8))) / 100.0f;

        if (i == 0) {
            info->max_temp = info->min_temp = g_frame[0];
            info->max_pos[0] = info->max_pos[1] = 0;
            info->min_pos[0] = info->min_pos[1] = 0;
            continue;
        }

        if (g_frame[i] > info->max_temp) {
            info->max_temp = g_frame[i];
            info->max_pos[0] = i % THERMAL_FRAME_WIDTH;
            info->max_pos[1] = i / THERMAL_FRAME_WIDTH;
        }
        if (g_frame[i] < info->min_temp) {
            info->min_temp = g_frame[i];
            info->min_pos[0] = i % THERMAL_FRAME_WIDTH;
            info->min_pos[1] = i / THERMAL_FRAME_WIDTH;
        }
    }

    info->center_temp = calculate_center_temp(g_frame);
}

static int read_cmd_data(uint8_t *p, uint32_t exp_len)
{
    int n;
    while (exp_len > 0) {
        n = read(g_serial_fd, p, exp_len);
        if (n < 0) {
            perr("Failed to read from serial port: errno: %d\n", errno);
            return -1;
        }
        pdbg("%x %x, read: %d, last: %d\n", p[0], p[1], n, exp_len);
        exp_len -= n;
        p += n;
    }

    return 0;
}

float *thermal_sensor_read_packet(thermal_info *info)
{
    usb_data_format *usb_format;
    uint32_t n;

    if (info == NULL) {
        perr("Param is NULL\n");
        return NULL;
    }

    usb_format = (usb_data_format *)g_rx_buf;
    memset(usb_format, 0, sizeof(g_rx_buf));

    while ((n = read(g_serial_fd, &usb_format->header, sizeof(usb_format->header))) > 0) {
        if (usb_format->header == USB_DATA_HEAD) {
            pdbg("Find Header\n");
            break;
        }
        pwarn("Skip 2 byte\n");
    }

    if (n <= 0) {
        perr("Failed to read from serial port: errno: %d\n", errno);
        return NULL;
    }

    n = read(g_serial_fd, &usb_format->type, sizeof(usb_format->type));
    if (n != sizeof(usb_format->type) || usb_format->type > USB_DATA_TYPE_MAX) {
        perr("Invalid data type: %d, n: %d\n", usb_format->type, n);
        return NULL;
    }

    n = read(g_serial_fd, &usb_format->len, sizeof(usb_format->len));
    if (n != sizeof(usb_format->len) || usb_format->len > SENSOR_FRAME_SIZE * 2) {
        perr("Invalid data len: %d, n: %d\n", usb_format->len, n);
        return NULL;
    }

    pdbg("Header: %#x, type: %d, len: %d\n", usb_format->header, usb_format->type, usb_format->len);
    if (read_cmd_data(usb_format->data, usb_format->len) < 0) {
        perr("Failed to read_cmd_data\n");
        return NULL;
    }

    switch (usb_format->type) {
        case USB_DATA_TYPE_CMD:
            pinfo("USB_CMD: %d\n", usb_format->data[0]);
            return NULL;
        case USB_DATA_TYPE_LOG:
            printf("-S- %s", usb_format->data);
            return NULL;
        case USB_DATA_TYPE_DATA:
            process_cmd_data(info, usb_format);
            break;
        default:
            perr("Invalid data type: %d\n", usb_format->type);
            return NULL;
    }

    return g_frame;
}
