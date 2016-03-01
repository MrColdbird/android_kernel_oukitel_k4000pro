#ifndef LINUX_SPI_JMT_MERGE_H
#define LINUX_SPI_JMT_MERGE_H

#define JMT_BASE_IO_OFFSET          0
#define JMT101_IOCTL_MAGIC_NO       0xFC
#define JMT_IOCTL_BASE(x)           JMT_BASE_IO_OFFSET+x
#define JMT_IOCTL_START_CAPTURE     _IO(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(0))
#define JMT_IOCTL_ABORT_CAPTURE     _IO(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(1))
#define JMT_IOCTL_READ_REGISTER     _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(2), unsigned int)
#define JMT_IOCTL_WRITE_REGISTER    _IOW(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(3), unsigned int)
#define JMT_IOCTL_SOFT_RESET        _IOW(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(5), unsigned int)
#define JMT_IOCTL_INIT_REG          _IOW(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(6), unsigned int)
#define JMT_IOCTL_READ_ATTR         _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(9), unsigned int) 
#define JMT_IOCTL_RV          _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(10), unsigned int)
#define JMT_IOCTL_SET_REG           _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(11), unsigned int)
#define JMT_IOCTL_SET_OP           _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(12), unsigned int)
#define JMT_IOCTL_SET_CLOCK_SPEED           _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(13), unsigned int)

#define JMT_IOCTL_WRITE_ATTR         _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(14), unsigned int) 
#define JMT_IOCTL_SET_VR            _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(15), unsigned int) 
#define JMT_IOCTL_READ_RAW            _IOWR(JMT101_IOCTL_MAGIC_NO, JMT_IOCTL_BASE(16), unsigned int) 


#define JMT_SPI_CLOCK_SPEED         (16 * 1000 * 1000) 
#define JMT_VER_ADDR                0xff
#define JMT_HEARTBEAT_COUNT         8000
#define JMT_HIST_LEN                16
#define JMT_HISTOGRAM_TOO_WHITE     950
#define JMT_TOGGLE_RESET            0x01
#define JMT_INTENSITY_TOO_WHITE     1
#define JMT_INTENSITY_WHITE         60
#define JMT_INTENSITY_MEAN_WHITE    160
#define JMT_INTENSITY_NORMAL        180
#define JMT_INTENSITY_MEAN_BLACK    200
#define JMT_INTENSITY_BLACK         245
#define JMT_INTENSITY_TOO_BLACK     255

#define JMT_TOGGLE_RESET            0x01
#define JMT_SWITCH_CALIBRATE        0x01
#define JMT_SWITCH_TIMESTAMP        0x02
#define JMT_SWITCH_PARAMETER        0x04

#define JMT_REG_FRAME_DATA          0xD0
#define JMT_REG_SOFTWARE_RESET      0xD1
#define JMT_REG_MODE_SELECT         0xD2
#define JMT_REG_STATUS              0xD3
#define JMT_REG_POWER_CTRL          0xE4
#define JMT_REG_VER_ID_MINOR        0xFD
#define JMT_REG_VER_ID_MAJOR        0xFF

//#define JMT_IMAGE_BUFFER_SIZE       (256 * 128 * 128)
#define JMT_IMAGE_BUFFER_SIZE       (256 * 128 * 8)
#define JMT_FRAME_READY_LIMIT       500

struct jmt101_nav_settings {
   u8 finger_down_min;
   u8 finger_down_mid;
   u8 finger_down_max;
   u8 finger_detect_thr;
   u8 finger_lost_thr;
   u8 dx_thr;
   u8 dy_thr;
   u8 nav_cntr;
   u8 adc_offset;
   u8 adc_gain;
   u8 pixel_setup;
   u8 off_x_axis_thr;
   u8 off_y_axis_thr;
   u8 update;
   u8 enabled;
};

struct jmt101_platform_data {
   u32 irq_gpio;
   u32 reset_gpio;
   struct jmt101_nav_settings nav;
};

#endif
