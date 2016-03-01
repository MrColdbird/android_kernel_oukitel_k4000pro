/* JMT101 Swipe Sensor Driver
 *
 * Copyright (c) 2014 J-Metrics <larrygump@moredna.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>

#ifdef CONFIG_ARCH_MT6735M
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/eint.h>
#include <cust_eint.h>
#else
#include <linux/gpio.h>
#endif

#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/mm.h>

#include <linux/earlysuspend.h>
//#include <linux/spi/jmt101.h>
//#include <linux/spi/jmt.h>
//#include <linux/spi/jmt_ioctl.h>
#include "jmt_merge.h"

#define DRIVER_VERSION                 "1.0.6.0"
#define JMT101A_FRAME_WIDTH             128
#define JMT101A_POS_GAIN                (JMT101A_FRAME_WIDTH - 2)
#define JMT101A_POS_VRT                 (JMT101A_POS_GAIN + 1)
#define JMT101A_POS_VRB                 (JMT101A_POS_GAIN + 2)
#define JMT101A_POS_OFFSET              (JMT101A_POS_GAIN + 3)
#define JMT101A_POS_TIMESTAMP           (JMT101A_FRAME_WIDTH + JMT101A_FRAME_WIDTH - 2)

#define JMT101B_POS_GAIN                (JMT101A_FRAME_WIDTH - 1)
#define JMT101B_POS_VRT                 (JMT101A_FRAME_WIDTH*2 - 1)
#define JMT101B_POS_VRB                 (JMT101A_FRAME_WIDTH*3 - 1)
#define JMT101B_POS_OFFSET              (JMT101A_FRAME_WIDTH*4 - 1)
#define JMT101B_POS_TIMESTAMP1          (JMT101A_FRAME_WIDTH*5 - 1)
#define JMT101B_POS_TIMESTAMP2          (JMT101A_FRAME_WIDTH*6 - 1)
#define JMT101B_POS_TIMESTAMP3          (JMT101A_FRAME_WIDTH*7 - 1)
#define JMT101B_POS_TIMESTAMP4          (JMT101A_FRAME_WIDTH*8 - 1)
/* -------------------------------------------------------------------- */
/* jmt sensor commands and registers                                 */
/* -------------------------------------------------------------------- */

#define JMT101_TRACE_MASK              0xFF
#define JMT101_TRACE_DPATTERN          0x01
#define JMT101_TRACE_SPATTERN          0x02
#define JMT101_TRACE_DEVERR            0x04

/* -------------------------------------------------------------------- */
/* jmt101 driver constants                                              */
/* -------------------------------------------------------------------- */
#define JMT101_MAJOR                   225

struct sensor_attr{
    unsigned int    sensor_width;
    unsigned int    sensor_height;
    unsigned int    sensor_size;
    unsigned int    frame_width;
    unsigned int    frame_height;
    unsigned int    frame_size;
    unsigned int    max_frames;
    /*
    unsigned char   gap;
    unsigned char   vrt;
    unsigned char   vrb;
    */
    unsigned char   frame_ready_limit;
    unsigned long long  buffer_size;
    unsigned long long  buffer_limit;
    unsigned char   intensity_too_white;
    unsigned char   intensity_white;
    unsigned char   intensity_mean_white;
    unsigned char   intensity_normal;
    unsigned char   intensity_mean_black;
    unsigned char   intensity_black;
    unsigned char   intensity_too_black;
    unsigned char   vrt_vrb_gap;
    unsigned char   version;
    unsigned char   hw_dvr;
};

struct jmt_op{
    u8 creg;
    u8 cread;
    u8 cwrite;
};

//#define JMT101_SPI_CLOCK_SPEED         (8 * 1000 * 1000)

#define JMT101_DEV_NAME                "jmt101"
#define JMT101_CLASS_NAME              "fpsensor"
#define JMT101_WORKER_THREAD_NAME      "jmt101worker"


#ifdef CONFIG_ARCH_MT6735M
#define RSUCCESS                       0
#define EINT0                          3
#define JMT101_INT_IRQNO               EINT0
#define JMT101_IRQ_GPIO                (GPIO19 | 0x80000000)
#define JMT101_RESET_GPIO              GPIO_FIGERPRINT_RST
#else
#define EINT0                          3
#define JMT101_INT_IRQNO               EINT0
#define JMT101_IRQ_GPIO                (19 | 0x80000000)
#define JMT101_RESET_GPIO              GPIO_FIGERPRINT_RST
#endif
#define GPIO_FPS_POWER_PIN      GPIO_FIGERPRINT_PWR_EN_PIN    //  pin 

static struct jmt101_platform_data jmt101_pdata_mt6577 = {
   .irq_gpio = JMT101_IRQ_GPIO,
   .reset_gpio = JMT101_RESET_GPIO,
   .nav = {
      .finger_down_min = 50,
      .finger_down_mid = 120,
      .finger_down_max = 255,
      .finger_detect_thr = 8,  // 25
      .finger_lost_thr = 250,  // 25
      .dx_thr = 80,
      .dy_thr = 50,
      .nav_cntr = 90,
      .adc_offset = 0xf0,  // 30
      .adc_gain = 0xf0,  // 12
      .pixel_setup = 0x7f,  // 44
      .off_x_axis_thr = 20,
      .off_y_axis_thr = 20,
      .enabled = 1,
   },
};

// charles modified
#ifdef CONFIG_ARCH_MT6735M

static struct mt_chip_conf spi_conf_mt6577 = {
   .setuptime = 3,
   .holdtime = 3,
   .high_time = 8,  // for mt6589, 100000khz/(4+4) = 125000khz
   .low_time = 8,
   .cs_idletime = 2,
   .ulthgh_thrsh = 0,

   .cpol = 1,
   .cpha = 1,

   .rx_mlsb = 1,
   .tx_mlsb = 1,

   .tx_endian = 0,
   .rx_endian = 0,

   .com_mod = DMA_TRANSFER,
   .pause = 0,
   .finish_intr = 1,
   .deassert = 0,
   .ulthigh = 0,
   .tckdly = 0,
};


#if 0
static struct mt_chip_conf spi_conf_mt6577 =
{
	.setuptime = 1,
	.holdtime = 1,
	.high_time = 4,
	.low_time = 4,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,
	.cpol = 1,
	.cpha = 1,
	.rx_mlsb = SPI_MSB, 
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
#ifdef TRANSFER_MODE_DMA
	.com_mod = DMA_TRANSFER,
#else
	.com_mod = FIFO_TRANSFER,
#endif
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif
#endif

enum {
   JMT101_THREAD_IDLE_MODE = 0,
   JMT101_THREAD_CAPTURE_MODE,
   JMT101_THREAD_NAV_MODE,
   JMT101_THREAD_EXIT
};

/* -------------------------------------------------------------------- */
/* jmt101 data types                                                    */
/* -------------------------------------------------------------------- */
struct jmt101_thread_task {
   int mode;
   int should_stop;
   struct semaphore sem_idle;
   wait_queue_head_t wait_job;
   struct task_struct *thread;
};

struct jmt101_control {
   u32 speed;
   u8 gain;
   u8 vrt;
   u8 vrb;
   u8 offset;
   u8 normal;
   u8 mean_black;
   u8 mean_white;
   u8 too_white;
   u16 too_white_px;
   u8 toggle;
   u8 flag;
};

struct jmt101_diag {
   u8 selftest;
   u32 capture_time;
   u32 frames_captured;
   u32 frames_stored;
   u32 frame_retry;
   u32 avail_data;
   u32 data_offset;
   u32 current_frame;
   u32 capture_done;
   u8 suspended;
   u32 fps;
   u32 count_poll;
   u32 count_read;
   u32 count_ioctl;
   u32 count_file;
};

struct jmt101_debug {
   u32 version;
   u8 trace_mask;
};


struct jmt101_data {
   struct spi_device *spi;
   struct class *class;
   struct device *device;
   struct cdev cdev;
   dev_t devno;
   u32 spi_speed;
   u32 reset_gpio;
   u32 irq_gpio;
   u32 irq;
   u32 data_offset;
   u32 avail_data;
   wait_queue_head_t waiting_data_avail;
   int interrupt_done;
   wait_queue_head_t waiting_interrupt_return;
   struct semaphore mutex;
   u8 *huge_buffer;
   u32 current_frame;
   int capture_done;
   int frame_retry;
   struct jmt101_thread_task thread_task;
   struct jmt101_control control;
   struct jmt101_diag diag;
   struct jmt101_debug debug;
   struct early_suspend early_suspend;
   struct jmt_op jop;
	struct sensor_attr sa;
};

struct jmt101_attribute {
   struct device_attribute attr;
   size_t offset;
};

/* -------------------------------------------------------------------- */
/* global variables                                                     */
/* -------------------------------------------------------------------- */
static int jmt101_device_count = 0;
static u8* spi_common_buf = NULL;
static u8* spi_reg;
static u8* spi_cmd;
static u8* spi_buf;
static u8 suspended;

struct jmt101_data *g_jmt101 = NULL;

/* -------------------------------------------------------------------- */
/* 0xE0 Gain, 0xE1 VRT, 0xE2 VRB, 0xE6 DC offset                        */
/* -------------------------------------------------------------------- */
enum {
   REG_GAIN,
   REG_VRT,
   REG_VRB,
   REG_DCOFFSET
};

static u8 regs_table[][2] = {
   {0xE0, 0x00},
   {0xE1, 0x00},
   {0xE2, 0x00},
   {0xE6, 0x00}
};

/* -------------------------------------------------------------------- */
/* function prototypes                                                  */
/* -------------------------------------------------------------------- */
static int __init jmt101_init (void);
static void __exit jmt101_exit (void);
//static int __devinit jmt101_probe (struct spi_device *spi);
//static int __devexit jmt101_remove (struct spi_device *spi);
static int jmt101_probe (struct spi_device *spi);
static int jmt101_remove (struct spi_device *spi);
static int jmt101_suspend (struct device *dev);
static int jmt101_resume (struct device *dev);

static int jmt101_open (struct inode *inode, struct file *file);
static ssize_t jmt101_write (struct file *file, const char *buff, size_t count, loff_t *ppos);
static ssize_t jmt101_read (struct file *file, char *buff, size_t count, loff_t *ppos);
static int jmt101_release (struct inode *inode, struct file *file);
static unsigned int jmt101_poll (struct file *file, poll_table *wait);
static long jmt101_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);

static int jmt101_reset (struct jmt101_data *jmt101);
static int jmt101_soft_reset (struct jmt101_data *jmt101);

static ssize_t jmt101_show_attr_control (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t jmt101_store_attr_control (struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t jmt101_show_attr_diag (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t jmt101_store_attr_diag (struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t jmt101_show_attr_debug (struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t jmt101_store_attr_debug (struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static void jmt101_early_suspend (struct early_suspend *h);
static void jmt101_late_resume (struct early_suspend *h);

static int jmt101_spi_rd_reg (struct jmt101_data *jmt101, u8 addr, u8 *value);
static int jmt101_spi_wr_reg (struct jmt101_data *jmt101, u8 addr, u8 value);
static int jmt101_spi_rd_image (struct jmt101_data *jmt101);
static int jmt101_selftest (struct jmt101_data *jmt101);

extern void dct_pmic_VCAMD_enable(kal_bool dctEnable);
extern void dct_pmic_VCAM_IO_enable(kal_bool dctEnable);
extern void dct_pmic_VCAMA_enable(kal_bool dctEnable);
extern void dct_pmic_VIO18_enable(kal_bool dctEnable);
extern void dct_pmic_VIO28_enable(kal_bool dctEnable);


/* -------------------------------------------------------------------- */
/* External interface                                                   */
/* -------------------------------------------------------------------- */
module_init (jmt101_init);
module_exit (jmt101_exit);

static const struct dev_pm_ops jmt101_pm = {
   .suspend = jmt101_suspend,
   .resume = jmt101_resume
};

static struct spi_driver jmt101_driver = {
   .driver = {
      .name = JMT101_DEV_NAME,
      .bus = &spi_bus_type,
      .owner = THIS_MODULE,
      .pm = &jmt101_pm,
   },
   .probe = jmt101_probe,
//   .remove = __devexit_p(jmt101_remove)
   .remove = jmt101_remove
};

static const struct file_operations jmt101_fops = {
   .owner = THIS_MODULE,
   .open = jmt101_open,
   .write = jmt101_write,
   .read = jmt101_read,
   .release = jmt101_release,
   .poll = jmt101_poll,
   .unlocked_ioctl = jmt101_ioctl
};

u8* tmp_buf = NULL;
static u8* rd_buf  = NULL;
/* -------------------------------------------------------------------- */
/* devfs                                                                */
/* -------------------------------------------------------------------- */
#define JMT101_ATTR(__grp, __field, __mode) \
{ \
   .attr = __ATTR(__field, (__mode), \
   jmt101_show_attr_##__grp, \
   jmt101_store_attr_##__grp), \
   .offset = offsetof(struct jmt101_##__grp, __field) \
}

#define JMT101_DEV_ATTR(_grp, _field, _mode) \
struct jmt101_attribute jmt101_attr_##_field = \
   JMT101_ATTR(_grp, _field, (_mode))

#define CONTROL_MODE                   0666

static JMT101_DEV_ATTR(control, speed, CONTROL_MODE);
static JMT101_DEV_ATTR(control, gain, CONTROL_MODE);
static JMT101_DEV_ATTR(control, vrt, CONTROL_MODE);
static JMT101_DEV_ATTR(control, vrb, CONTROL_MODE);
static JMT101_DEV_ATTR(control, offset, CONTROL_MODE);
static JMT101_DEV_ATTR(control, normal, CONTROL_MODE);
static JMT101_DEV_ATTR(control, mean_black, CONTROL_MODE);
static JMT101_DEV_ATTR(control, mean_white, CONTROL_MODE);
static JMT101_DEV_ATTR(control, too_white, CONTROL_MODE);
static JMT101_DEV_ATTR(control, too_white_px, CONTROL_MODE);
static JMT101_DEV_ATTR(control, toggle, CONTROL_MODE);
static JMT101_DEV_ATTR(control, flag, CONTROL_MODE);

static struct attribute *jmt101_control_attrs [] = {
   &jmt101_attr_speed.attr.attr,
   &jmt101_attr_gain.attr.attr,
   &jmt101_attr_vrt.attr.attr,
   &jmt101_attr_vrb.attr.attr,
   &jmt101_attr_offset.attr.attr,
   &jmt101_attr_normal.attr.attr,
   &jmt101_attr_mean_black.attr.attr,
   &jmt101_attr_mean_white.attr.attr,
   &jmt101_attr_too_white.attr.attr,
   &jmt101_attr_too_white_px.attr.attr,
   &jmt101_attr_toggle.attr.attr,
   &jmt101_attr_flag.attr.attr,
   NULL
};

static const struct attribute_group jmt101_control_attr_group = {
   .attrs = jmt101_control_attrs,
   .name = "control"
};

#define DIAG_MODE                      (S_IRUSR | S_IRGRP | S_IROTH)

static JMT101_DEV_ATTR(diag, selftest, DIAG_MODE);
static JMT101_DEV_ATTR(diag, capture_time, DIAG_MODE);
static JMT101_DEV_ATTR(diag, frames_captured, DIAG_MODE);
static JMT101_DEV_ATTR(diag, frames_stored, DIAG_MODE);
static JMT101_DEV_ATTR(diag, frame_retry, DIAG_MODE);
static JMT101_DEV_ATTR(diag, avail_data, DIAG_MODE);
static JMT101_DEV_ATTR(diag, data_offset, DIAG_MODE);
static JMT101_DEV_ATTR(diag, current_frame, DIAG_MODE);
static JMT101_DEV_ATTR(diag, capture_done, DIAG_MODE);
static JMT101_DEV_ATTR(diag, suspended, DIAG_MODE);
static JMT101_DEV_ATTR(diag, fps, DIAG_MODE);
static JMT101_DEV_ATTR(diag, count_poll, DIAG_MODE);
static JMT101_DEV_ATTR(diag, count_read, DIAG_MODE);
static JMT101_DEV_ATTR(diag, count_ioctl, DIAG_MODE);
static JMT101_DEV_ATTR(diag, count_file, DIAG_MODE);

static struct attribute *jmt101_diag_attrs [] = {
   &jmt101_attr_selftest.attr.attr,
   &jmt101_attr_capture_time.attr.attr,
   &jmt101_attr_frames_captured.attr.attr,
   &jmt101_attr_frames_stored.attr.attr,
   &jmt101_attr_frame_retry.attr.attr,
   &jmt101_attr_avail_data.attr.attr,
   &jmt101_attr_data_offset.attr.attr,
   &jmt101_attr_current_frame.attr.attr,
   &jmt101_attr_capture_done.attr.attr,
   &jmt101_attr_suspended.attr.attr,
   &jmt101_attr_fps.attr.attr,
   &jmt101_attr_count_poll.attr.attr,
   &jmt101_attr_count_read.attr.attr,
   &jmt101_attr_count_ioctl.attr.attr,
   &jmt101_attr_count_file.attr.attr,
   NULL
};

static const struct attribute_group jmt101_diag_attr_group = {
   .attrs = jmt101_diag_attrs,
   .name = "diag"
};

#define JMT101_DEBUG_MODE              0666

static JMT101_DEV_ATTR(debug, version, JMT101_DEBUG_MODE);
static JMT101_DEV_ATTR(debug, trace_mask, JMT101_DEBUG_MODE);

static struct attribute *jmt101_debug_attrs [] = {
   &jmt101_attr_version.attr.attr,
   &jmt101_attr_trace_mask.attr.attr,
   NULL
};

static const struct attribute_group jmt101_debug_attr_group = {
   .attrs = jmt101_debug_attrs,
   .name = "debug"
};

/* -------------------------------------------------------------------- */
/* function definitions                                                 */
/* -------------------------------------------------------------------- */
static ssize_t jmt101_show_attr_control (struct device *dev,
   struct device_attribute *attr, char *buf)
{
   int error = 0;
   u8 m_val;

   struct jmt101_data *jmt101;
   struct jmt101_attribute *jmt_attr;

   jmt101 = dev_get_drvdata(dev);
   jmt_attr = container_of(attr, struct jmt101_attribute, attr);

   /* show spi speed */
   if (jmt_attr->offset == offsetof(struct jmt101_control, speed)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->spi_speed);
   }

   /* show gain */
   if (jmt_attr->offset == offsetof(struct jmt101_control, gain)) {
      if (!down_interruptible(&jmt101->mutex)) {
         error = jmt101_spi_rd_reg(jmt101, regs_table[REG_GAIN][0], &m_val);
         up (&jmt101->mutex);
         if (!error) {
            regs_table[REG_GAIN][1] = m_val;
            jmt101->control.gain = regs_table[REG_GAIN][1];
            return scnprintf(buf, PAGE_SIZE, "%s: %d[0x%.02X]\n", attr->attr.name, m_val, m_val);
         }
      }

      return scnprintf(buf, PAGE_SIZE, "%s: error[0x%.08X]\n", attr->attr.name, error);
   }

   /* show offset */
   if (jmt_attr->offset == offsetof(struct jmt101_control, offset)) {
      if (!down_interruptible(&jmt101->mutex)) {
         error = jmt101_spi_rd_reg(jmt101, regs_table[REG_DCOFFSET][0], &m_val);
         up (&jmt101->mutex);
         if (!error) {
            regs_table[REG_DCOFFSET][1] = m_val;
            jmt101->control.offset = regs_table[REG_DCOFFSET][1];
            return scnprintf(buf, PAGE_SIZE, "%s: %d[0x%.02X]\n", attr->attr.name, m_val, m_val);
         }
      }

      return scnprintf(buf, PAGE_SIZE, "%s: error[0x%.08X]\n", attr->attr.name, error);
   }

   /* show vrt */
   if (jmt_attr->offset == offsetof(struct jmt101_control, vrt)) {
      if (!down_interruptible(&jmt101->mutex)) {
         error = jmt101_spi_rd_reg(jmt101, regs_table[REG_VRT][0], &m_val);
         up (&jmt101->mutex);
         if (!error) {
            regs_table[REG_VRT][1] = m_val;
            jmt101->control.vrt = regs_table[REG_VRT][1];
            return scnprintf(buf, PAGE_SIZE, "%s: %d[0x%.02X]\n", attr->attr.name, m_val, m_val);
         }
      }

      return scnprintf(buf, PAGE_SIZE, "%s: error[0x%.08X]\n", attr->attr.name, error);
   }

   /* show vrb */
   if (jmt_attr->offset == offsetof(struct jmt101_control, vrb)) {
      if (!down_interruptible(&jmt101->mutex)) {
         error = jmt101_spi_rd_reg(jmt101, regs_table[REG_VRB][0], &m_val);
         up (&jmt101->mutex);
         if (!error) {
            regs_table[REG_VRB][1] = m_val;
            jmt101->control.vrb = regs_table[REG_VRB][1];
            return scnprintf(buf, PAGE_SIZE, "%s: %d[0x%.02X]\n", attr->attr.name, m_val, m_val);
         }
      }

      return scnprintf(buf, PAGE_SIZE, "%s: error[0x%.08X]\n", attr->attr.name, error);
   }

   /* show intensity normal */
   if (jmt_attr->offset == offsetof(struct jmt101_control, normal)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->control.normal);
   }

   /* show intensity mean black */
   if (jmt_attr->offset == offsetof(struct jmt101_control, mean_black)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->control.mean_black);
   }

   /* show intensity mean white */
   if (jmt_attr->offset == offsetof(struct jmt101_control, mean_white)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->control.mean_white);
   }

   /* show intensity too white */
   if (jmt_attr->offset == offsetof(struct jmt101_control, too_white)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->control.too_white);
   }

   /* show histogram too white */
   if (jmt_attr->offset == offsetof(struct jmt101_control, too_white_px)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->control.too_white_px);
   }

   /* show control toggle */
   if (jmt_attr->offset == offsetof(struct jmt101_control, toggle)) {
      return scnprintf(buf, PAGE_SIZE, "0x%.02X\n", jmt101->control.toggle);
   }

   /* show control flag */
   if (jmt_attr->offset == offsetof(struct jmt101_control, flag)) {
      return scnprintf(buf, PAGE_SIZE, "0x%.02X\n", jmt101->control.flag);
   }

   return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t jmt101_store_attr_control (struct device *dev,
   struct device_attribute *attr,
   const char *buf, size_t count)
{
   u32 tmp;

   struct jmt101_data *jmt101;
   struct jmt101_attribute *jmt_attr;

   jmt101 = dev_get_drvdata(dev);
   jmt_attr = container_of(attr, struct jmt101_attribute, attr);

   if ((sscanf(buf, "%u", &tmp)) <= 0) {
      printk ("[jmt101]%s: control (invalid para)\n", __func__);
      return -EINVAL;
   }

   /* store spi speed */
   if (jmt_attr->offset == offsetof(struct jmt101_control, speed)) {
      if (!down_interruptible(&jmt101->mutex)) {
         jmt101->spi_speed = tmp;
         up (&jmt101->mutex);
      }
   }

   /* store gain */
   if (jmt_attr->offset == offsetof(struct jmt101_control, gain)) {
      if (!down_interruptible(&jmt101->mutex)) {
         jmt101_spi_wr_reg (jmt101, regs_table[REG_GAIN][0], (u8)tmp);
         up (&jmt101->mutex);
      }
      jmt101->control.gain = (u8)tmp;
      regs_table[REG_GAIN][1] = jmt101->control.gain;
   }

   /* store vrt */
   if (jmt_attr->offset == offsetof(struct jmt101_control, vrt)) {
      if (!down_interruptible(&jmt101->mutex)) {
         jmt101_spi_wr_reg (jmt101, regs_table[REG_VRT][0], (u8)tmp);
         up (&jmt101->mutex);
      }
      jmt101->control.vrt = (u8)tmp;
      regs_table[REG_VRT][1] = jmt101->control.vrt;
   }

   /* store vrb */
   if (jmt_attr->offset == offsetof(struct jmt101_control, vrb)) {
      if (!down_interruptible(&jmt101->mutex)) {
         jmt101_spi_wr_reg (jmt101, regs_table[REG_VRB][0], (u8)tmp);
         up (&jmt101->mutex);
      }
      jmt101->control.vrb = (u8)tmp;
      regs_table[REG_VRB][1] = jmt101->control.vrb;
   }

   /* store dc offset */
   if (jmt_attr->offset == offsetof(struct jmt101_control, offset)) {
      if (!down_interruptible(&jmt101->mutex)) {
         jmt101_spi_wr_reg (jmt101, regs_table[REG_DCOFFSET][0], (u8)tmp);
         up (&jmt101->mutex);
      }
      jmt101->control.offset = (u8)tmp;
      regs_table[REG_DCOFFSET][1] = jmt101->control.offset;
   }

   /* store intensity normal */
   if (jmt_attr->offset == offsetof(struct jmt101_control, normal)) {
      jmt101->control.normal = (u8)tmp;
   }

   /* store intensity mean black */
   if (jmt_attr->offset == offsetof(struct jmt101_control, mean_black)) {
      jmt101->control.mean_black = (u8)tmp;
   }

   /* store intensity mean white */
   if (jmt_attr->offset == offsetof(struct jmt101_control, mean_white)) {
      jmt101->control.mean_white = (u8)tmp;
   }

   /* store intensity too white */
   if (jmt_attr->offset == offsetof(struct jmt101_control, too_white)) {
      jmt101->control.too_white = (u8)tmp;
   }

   /* store histogram too white */
   if (jmt_attr->offset == offsetof(struct jmt101_control, too_white_px)) {
      jmt101->control.too_white_px = (u16)tmp;
   }

   /* trigger control toggle */
   if (jmt_attr->offset == offsetof(struct jmt101_control, toggle)) {
      jmt101->control.toggle = (u8)tmp;

      if (tmp & JMT_TOGGLE_RESET) {
         if (!down_interruptible(&jmt101->mutex)) {
            jmt101_reset (jmt101);
            up (&jmt101->mutex);
         }
      }
   }

   /* store control flag */
   if (jmt_attr->offset == offsetof(struct jmt101_control, flag)) {
      jmt101->control.flag = (u8)tmp;
   }

   return strnlen(buf, count);
}

/* -------------------------------------------------------------------- */
static ssize_t jmt101_show_attr_diag (struct device *dev,
   struct device_attribute *attr, char *buf)
{
   struct jmt101_data *jmt101;
   struct jmt101_attribute *jmt_attr;

   jmt101 = dev_get_drvdata(dev);
   jmt_attr = container_of(attr, struct jmt101_attribute, attr);

   /* self test */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, selftest)) {
      jmt101_selftest (jmt101);
      return scnprintf(buf, PAGE_SIZE, "0x%.02X\n", jmt101->diag.selftest);
   }

   /* show capture time */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, capture_time)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->diag.capture_time);
   }

   /* show frames captured */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, frames_captured)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->diag.frames_captured);
   }

   /* show frames stored */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, frames_stored)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->diag.frames_stored);
   }

   /* show frames retry count */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, frame_retry)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->frame_retry);
   }

   /* show available data count */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, avail_data)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->avail_data);
   }

   /* show available data offset */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, data_offset)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->data_offset);
   }

   /* show current frame */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, current_frame)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->current_frame);
   }

   /* show capture done flag */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, capture_done)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->capture_done);
   }

   /* show suspended flag */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, suspended)) {
      return scnprintf(buf, PAGE_SIZE, "0x%.02X\n", suspended);
   }

   /* show fps */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, fps)) {
      if (jmt101->diag.capture_time > 0) {
         return scnprintf(buf, PAGE_SIZE,
            "captured %lu frames (%lu kept) in %lu ms (%lu fps)\n",
            (long unsigned int)jmt101->diag.frames_captured,
            (long unsigned int)jmt101->diag.frames_stored,
            (long unsigned int)jmt101->diag.capture_time,
            (long unsigned int)(jmt101->diag.frames_captured * MSEC_PER_SEC / jmt101->diag.capture_time));
      } else {
         return scnprintf(buf, PAGE_SIZE, "capture time = 0\n");
      }
   }

   /* show poll counter */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, count_poll)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->diag.count_poll);
   }

   /* show read counter */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, count_read)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->diag.count_read);
   }

   /* show ioctl counter */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, count_ioctl)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->diag.count_ioctl);
   }

   /* show file counter */
   if (jmt_attr->offset == offsetof(struct jmt101_diag, count_file)) {
      return scnprintf(buf, PAGE_SIZE, "%u\n", jmt101->diag.count_file);
   }

   return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t jmt101_store_attr_diag (struct device *dev,
   struct device_attribute *attr,
   const char *buf, size_t count)
{
   return strnlen(buf, count);
}

/* -------------------------------------------------------------------- */
static ssize_t jmt101_show_attr_debug (struct device *dev,
   struct device_attribute *attr, char *buf)
{
   struct jmt101_data *jmt101;
   struct jmt101_attribute *jmt_attr;

   jmt101 = dev_get_drvdata(dev);
   jmt_attr = container_of(attr, struct jmt101_attribute, attr);

   /* show version no. */
   if (jmt_attr->offset == offsetof(struct jmt101_debug, version)) {
      return scnprintf(buf, PAGE_SIZE, "%s\n", DRIVER_VERSION);
   }

   /* show trace mask */
   if (jmt_attr->offset == offsetof(struct jmt101_debug, trace_mask)) {
      return scnprintf(buf, PAGE_SIZE, "%s: %d[0x%.02X]\n", attr->attr.name, jmt101->debug.trace_mask, jmt101->debug.trace_mask);
   }

   return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t jmt101_store_attr_debug (struct device *dev,
   struct device_attribute *attr,
   const char *buf, size_t count)
{
   u8 tmp;

   struct jmt101_data *jmt101;
   struct jmt101_attribute *jmt_attr;

   jmt101 = dev_get_drvdata(dev);
   jmt_attr = container_of(attr, struct jmt101_attribute, attr);

   if ((sscanf(buf, "%hhu", &tmp)) <= 0) {
      printk ("[jmt101]%s: debug (invalid para)\n", __func__);
      return -EINVAL;
   }

   /* store trace mask */
   if (jmt_attr->offset == offsetof(struct jmt101_debug, trace_mask)) {
      /* sensor test pattern */
      if (tmp & JMT101_TRACE_SPATTERN) {
         if (!down_interruptible(&jmt101->mutex)) {
            jmt101_spi_wr_reg (jmt101, 0xF1, 0x02);
            up (&jmt101->mutex);
         }
      } else {
         if (!down_interruptible(&jmt101->mutex)) {
            jmt101_spi_wr_reg (jmt101, 0xF1, 0x00);
            up (&jmt101->mutex);
         }
      }

      jmt101->debug.trace_mask = tmp;
   }

   return strnlen(buf, count);
}

/* -------------------------------------------------------------------- */
static int jmt101_spi_rd_reg (struct jmt101_data *jmt101, u8 addr, u8 *value)
{
    int error;

    struct spi_message m;

    struct spi_transfer tr_reg = {
        .cs_change = 1,
        .delay_usecs = 0,
        .speed_hz = jmt101->spi_speed,
        .tx_buf = spi_reg,
        .rx_buf = NULL,
        .len = 2,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 8,
    };

    struct spi_transfer tr_dat = {
        .cs_change = 0,
        .delay_usecs = 0,
        .speed_hz = jmt101->spi_speed,
        .tx_buf = spi_cmd,
        .rx_buf = spi_buf,
        .len = 2,
        .tx_dma = 0,
        .rx_dma = 0,
        .bits_per_word = 8,
    };
    /* select register */
    // Charles edited, Date: 2014_0905
    //
    spi_reg[0] = jmt101->jop.creg;
    spi_reg[1] = addr;
/*
    spi_message_init (&m);
    spi_message_add_tail (&tr_reg, &m);
    error = spi_sync(jmt101->spi, &m);
    if (error) {
        dev_err (&jmt101->spi->dev, "spi_sync failed.\n");
    }
*/
    /* read value */
    // Charles edited, Date: 2014_0905
    spi_cmd[0] = jmt101->jop.cread;
    spi_cmd[1] = 0xFF;
    spi_buf[1] = 0x00;

    spi_message_init (&m);
    spi_message_add_tail (&tr_reg, &m);
    spi_message_add_tail (&tr_dat, &m);

    error = spi_sync(jmt101->spi, &m);
    if (error) {
        dev_err (&jmt101->spi->dev, "spi_sync failed.\n");
    }
    *value = spi_buf[1];
    return (error);
}

/* -------------------------------------------------------------------- */
static int jmt101_spi_wr_reg (struct jmt101_data *jmt101, u8 addr, u8 value)
{
   int error;

   struct spi_message m;

   struct spi_transfer tr_reg = {
      .cs_change = 1,
      .delay_usecs = 0,
      .speed_hz = jmt101->spi_speed,
      .tx_buf = spi_reg,
      .rx_buf = NULL,
      .len = 2,
      .tx_dma = 0,
      .rx_dma = 0,
      .bits_per_word = 8,
   };

   struct spi_transfer tr_dat = {
      .cs_change = 0,
      .delay_usecs = 0,
      .speed_hz = jmt101->spi_speed,
      .tx_buf = spi_cmd,
      .rx_buf = NULL,
      .len = 2,
      .tx_dma = 0,
      .rx_dma = 0,
      .bits_per_word = 8,
   };

   /* select register */
   spi_reg[0] = jmt101->jop.creg;
   spi_reg[1] = addr;

/*
   spi_message_init (&m);
   spi_message_add_tail (&tr_reg, &m);

   error = spi_sync(jmt101->spi, &m);
   if (error) {
      dev_err (&jmt101->spi->dev, "spi_sync failed.\n");
   }
*/

   /* write value */
   spi_cmd[0] = jmt101->jop.cwrite;
   spi_cmd[1] = value;

   spi_message_init (&m);
   spi_message_add_tail (&tr_reg, &m);
   spi_message_add_tail (&tr_dat, &m);

   error = spi_sync(jmt101->spi, &m);
   if (error) {
      dev_err (&jmt101->spi->dev, "spi_sync failed.\n");
   }

   return (error);
}

/* -------------------------------------------------------------------- */
static int jmt101_soft_reset (struct jmt101_data *jmt101)
{
#if 0
    int error;
    int try_num;

    /* soft reset sensor */
    error = jmt101_spi_wr_reg(jmt101, JMT_REG_SOFTWARE_RESET, 0x80);
    //struct sensor_reg* sr = &jmt101->sr;
    if (error)
        goto err;
    /* check sensor version id to confirm sensor ready */
    for (try_num = 0; try_num < 400; try_num++) {
        error = jmt101_spi_rd_reg(jmt101, JMT_REG_VER_ID_MAJOR, jmt101->huge_buffer + jmt101->data_offset);
        if (error)
            break;

        if (jmt101->huge_buffer[jmt101->data_offset] == JMT_VER_ID_MAJOR) {
            break;
        } else {
            error = -EIO;
        }

        msleep_interruptible (1);
    }

    if (error)
        goto err;
err:
    return (error);
#else
    return 0;
#endif
}

/* -------------------------------------------------------------------- */
static int jmt101_reset (struct jmt101_data *jmt101)
{
   int error;

#ifdef CONFIG_ARCH_MT6735M
   /* toggle reset_gpio pin */
   mt_set_gpio_out (jmt101->reset_gpio, GPIO_OUT_ZERO);
   udelay (1000);
   mt_set_gpio_out (jmt101->reset_gpio, GPIO_OUT_ONE);
   udelay (1250);
#endif

   /* disable irq */

   error = jmt101_soft_reset(jmt101);

   /* enable irq */

   // printk ("[jmt101]%s: OUT\n", __func__);
   return (error);
}

// ......+.........+.........+.........+.........+.........+.........+.........+
/* up left -> down left, left -> right col by col                             */
// ......+.........+.........+.........+.........+.........+.........+.........+
static void arrange_frame (struct jmt101_data* jmt101, u8 *fbuffer)
{
	int row, col;
	int source_base, target_base;
	int source_off, target_off;
	//u8 tmp_buf [1024]={0};
	//u8 tmp_buf [JMT101_SENSOR_SIZE];
//	u8* tmp_buf = kmalloc(JMT_SENSOR_SIZE, GFP_KERNEL);
//
    struct sensor_attr* sa = &jmt101->sa; 
    if(!sa->sensor_size) return;
	//u8* tmp_buf = kmalloc(sa->sensor_size, GFP_KERNEL);
	if (!tmp_buf) return;
    memset(tmp_buf, 0, sa->sensor_size);
	/* handle error ... */
    switch(jmt101->sa.version){
        case 0:
            target_base = (sa->sensor_height- 1) * sa->sensor_width;
            source_base = 0;
            target_off = 0;
            source_off = 0;

            for (row = 0; row < sa->sensor_height; row++) {
                for (col = 0; col < sa->sensor_width ; col++) {
                    tmp_buf[target_base + target_off] = fbuffer[source_base + source_off];
                    target_off++;
                    source_off += sa->sensor_height;
                }
                target_base -= sa->sensor_width;
                source_base++;
                target_off = 0;
                source_off = 0;
            }
            break;
        case 1:
        //target_base = 7 * 128;
        // __1017
            target_base = (sa->sensor_height- 1) * sa->sensor_width;
            source_base = 0;
            target_off = 0;
            source_off = 0;

        // __1017
        for (row = 0; row < sa->sensor_height; row++) {
            for (col = 0; col < sa->sensor_width ; col++) {
        //for (row = 0; row < 8; row++) {
            //for (col = 0; col < 128 ; col++) {
                    tmp_buf[target_base + target_off] = fbuffer[source_base + source_off];
                    target_off++;
                    source_off += sa->sensor_height;
                    //source_off += 8;
                }
                target_base -= sa->sensor_width;
                //target_base -= 128;
                source_base++;
                target_off = 0;
                source_off = 0;
            }

            break;
        case 2:
        //target_base = (JMT101_SENSOR_HEIGHT - 1) * JMT101_SENSOR_WIDTH;
        target_base = 0;//(JMT101_SENSOR_HEIGHT - 1) * JMT101_SENSOR_WIDTH;
        source_base = 0;
        target_off = 0;
        source_off = 0;
        //for (row = 0; row < JMT_SENSOR_HEIGHT; row++) {
        //	for (col = 0; col < JMT_SENSOR_WIDTH ; col++) {
        for (row = 0; row < sa->sensor_height; row++) {
            for (col = 0; col < sa->sensor_width ; col++) {
                tmp_buf[target_base + target_off] = fbuffer[source_base + source_off];
                target_off++;
                //source_off += JMT_SENSOR_HEIGHT;
                source_off += sa->sensor_height;
            }
            //target_base -= JMT_SENSOR_WIDTH;
            //target_base += JMT_SENSOR_WIDTH;
            target_base += sa->sensor_width;
            source_base++;
            target_off = 0;
            source_off = 0;
        }
        break;
        default:
        break;
    }
	//memcpy (fbuffer, tmp_buf, JMT_SENSOR_SIZE);
	memcpy (fbuffer, tmp_buf, sa->sensor_size);
	//if(tmp_buf) kfree(tmp_buf);
}

/* -------------------------------------------------------------------- */
static void dump_buf (uint8_t *buf, int len)
{
   int idx;

   printk ("------------------------------------------------------------\n");

   for (idx = 0; idx < len; idx++) {
      if (!(idx % 16))
         printk ("\n");
      printk ("%.2X ", buf[idx]);
   }
   printk ("\n");
}


/*----------------------------------------------------------------------
 * Calculate intensity
+---------------------------------------------------------------------*/
int calc_intensity (struct jmt101_data* jmt101, u8 *fp_frame, int up, int down, int left, int right)
{
   int row;
   int col;

   int pixel_idx;
   int pixel_cnt = 0;
   int pixel_sum = 0;

   /* todo: parameter boundary check */

   //pixel_idx = up * JMT_FRAME_WIDTH + left;
   struct sensor_attr* sa = &jmt101->sa;
   if(!sa->frame_width) return 0;
   pixel_idx = up * sa->frame_width + left;

   for (row = up; row <= down; row++) {
      for (col = left; col <= right; col++) {
         pixel_sum += fp_frame[pixel_idx++];
         ++pixel_cnt;
      }
      pixel_idx += (sa->frame_width - (right - left + 1));
   }

   return (pixel_sum / pixel_cnt);
}

/*----------------------------------------------------------------------
 * Calculate histogram
+---------------------------------------------------------------------*/
void calc_histogram (u8 *fp_frame, int fp_size, int hist[JMT_HIST_LEN])
{
   int idx;

   for (idx = 0; idx < JMT_HIST_LEN; idx++) {
     hist[idx] = 0;
   }

   for (idx = 0; idx < fp_size; idx++) {
     hist[fp_frame[idx] >> 4]++;
   }
}


// Charles: need to modify for multiple versions later 
/*----------------------------------------------------------------------
 * add VRB
+---------------------------------------------------------------------*/
int add_vrb (struct jmt101_data* jmt101, int add_no)
{
   int vrb;
   int vrt;
   int ret = 0;
   int vr_max = (!jmt101->sa.version)?0x3F:0x28;
   int bias = (!jmt101->sa.version)?12:3;

   do {
      if (add_no <= 0) {
         break;
      }

      //if (regs_table[REG_VRB][1] == 0x3F) {
      if (regs_table[REG_VRB][1] == vr_max) {
         break;
      }
      ret = 1;

      vrb = regs_table[REG_VRB][1] + add_no;
      if(vrb > vr_max){
        vrb = vr_max;
      }

      regs_table[REG_VRB][1] = vrb;

      vrb = 100 + 25 * regs_table[REG_VRB][1];
      vrt = (20 * vrb) / 1000 + (jmt101->sa.vrt_vrb_gap * 2) - bias;

      regs_table[REG_VRT][1] = (u8)(vrt);

   } while (0);

   return (ret);
}

// Charles: need to modify for multiple versions later 
/*----------------------------------------------------------------------
 * sub VRB
+---------------------------------------------------------------------*/
int sub_vrb (struct jmt101_data* jmt101, int sub_no)
{
   int vrb;
   int vrt;
   int ret = 0;

   int bias = (!jmt101->sa.version)?12:3;
   do {
      if (sub_no <= 0) {
         break;
      }

      if (regs_table[REG_VRB][1] == 0x00) {
         break;
      }
      ret = 1;

      vrb = regs_table[REG_VRB][1] - sub_no;
      if (vrb < 0x00) {
         vrb = 0x00;
      }

      regs_table[REG_VRB][1] = vrb;

      vrb = 100 + 25 * regs_table[REG_VRB][1];
      vrt = (20 * vrb) / 1000 + (jmt101->sa.vrt_vrb_gap * 2) - bias;

      regs_table[REG_VRT][1] = (u8)(vrt);

   } while (0);

   return (ret);
}

// Charles: need to modify for multiple versions later 
/*----------------------------------------------------------------------
 * Intensity calibration
+---------------------------------------------------------------------*/
void do_calibrate (struct jmt101_data *jmt101, u8 *fp_frame)
{
    struct sensor_attr *sa = &jmt101->sa;
	int intensity;
    int hist [JMT_HIST_LEN];
	int ret = 0;
    int up, down, left, right;
    up = down = left =right =0;
    switch(sa->version){
        case 0:
            up = 1; down=6; left=5; right=122;
            break;
        case 1:
            up = 1; down=6; left=5; right=122;
            break;
        case 2:
            up = 20; down=108; left=20; right=108;
            break;
        default:
            up = 1; down=6; left=5; right=122;
            break;
    }
	//intensity = calc_intensity(fp_frame, 1, 6, 5, 122);
// charles modify 0903
	//intensity = calc_intensity(fp_frame, 1, 122, 5, 122);
	intensity = calc_intensity(jmt101, fp_frame, 
           // 1, JMT_SENSOR_HEIGHT -1, 1, JMT_SENSOR_WIDTH-1);
            up, down, left, right);
   //calc_histogram (fp_frame, JMT_FRAME_SIZE, hist);
   calc_histogram (fp_frame, sa->frame_size, hist);
   //calc_histogram (fp_frame, 1024, hist);
/* no finger touch */
   if ((hist[0] > jmt101->control.too_white_px) || (intensity < jmt101->control.too_white)) {
      return;
   }

   if (intensity < jmt101->control.mean_black && intensity > jmt101->control.mean_white) {
      return;
   }

   if (intensity > jmt101->control.normal) {
	//if (intensity > sa->intensity_normal) {
		ret = add_vrb(jmt101, 1);
	} else {
		ret = sub_vrb(jmt101, 1);
	}

	if (ret) {
		jmt101_spi_wr_reg (jmt101, regs_table[REG_VRB][0], regs_table[REG_VRB][1]);
		jmt101_spi_wr_reg (jmt101, regs_table[REG_VRT][0], regs_table[REG_VRT][1]);
	}
}

/*----------------------------------------------------------------------
 * Check register d3 
+---------------------------------------------------------------------*/
int check_dvr_enable(struct jmt101_data *jmt101)
{
    u8 data = 0;

    if (down_interruptible(&jmt101->mutex)) goto err;
    jmt101_spi_rd_reg (jmt101, JMT_REG_STATUS, &data);
    up (&jmt101->mutex);

err:
    printk("charles %s: get 0xD3 reg data=0x%x, return 0x%x\n",__func__, data, data&0x04);
    
    return (data & 0x20);

}

/* -------------------------------------------------------------------- */
static int jmt101_spi_rd_image (struct jmt101_data *jmt101)
{
   int error;

   //u8 rd_buf[JMT101_FRAME_SIZE] = {0x00};
   //u8 rd_buf[1024] = {0x00};
	//u8* rd_buf ; 

   struct spi_message spi_mess;
   struct spi_transfer tr_reg;
   struct spi_transfer tr_dat1;
   struct spi_transfer tr_dat2;

   int rx_off;

   int row;
   int buf_source, buf_target;
   int idx;
   int value ;
   u8 pixel;
   struct timespec ts_now;
   u32 ts_stamp;
   struct sensor_attr* sa = &jmt101->sa;

   if(!sa->frame_size) return 0;

   memset (&tr_reg, 0, sizeof(struct spi_transfer));
   memset (&tr_dat1, 0, sizeof(struct spi_transfer));
   memset (&tr_dat2, 0, sizeof(struct spi_transfer));

   tr_reg.cs_change = 1;
   tr_reg.delay_usecs = 0;
   tr_reg.speed_hz = jmt101->spi_speed;
   tr_reg.tx_buf = spi_reg;
   tr_reg.rx_buf = NULL;
   tr_reg.len = 2;

   /* select register */
   spi_reg[0] = jmt101->jop.creg;
   spi_reg[1] = JMT_REG_FRAME_DATA;

   rx_off = jmt101->current_frame * sa->frame_size;

   spi_cmd[0] = jmt101->jop.cread;
   spi_cmd[1] = 0xFF;

   if(sa->version == 0) {
       tr_dat1.cs_change = 1;
       tr_dat1.delay_usecs = 0;
       tr_dat1.speed_hz = jmt101->spi_speed;
       tr_dat1.tx_buf = spi_cmd;
       tr_dat1.rx_buf = jmt101->huge_buffer + rx_off;
       tr_dat1.len = sa->sensor_size+1;
    
       spi_message_init (&spi_mess);
       spi_message_add_tail (&tr_reg, &spi_mess);
       spi_message_add_tail (&tr_dat1, &spi_mess);
       error = spi_sync(jmt101->spi, &spi_mess);
       if (error) {
          dev_err (&jmt101->spi->dev, "spi_sync failed.\n");
          return error;
       }

   } else {
       tr_dat1.cs_change = 1;
       tr_dat1.delay_usecs = 0;
       tr_dat1.speed_hz = jmt101->spi_speed;
       tr_dat1.tx_buf = spi_cmd;
       tr_dat1.rx_buf = jmt101->huge_buffer + rx_off;
       tr_dat1.len = sa->sensor_size;
    
       tr_dat2.cs_change = 1;
       tr_dat2.delay_usecs = 0;
       tr_dat2.speed_hz = jmt101->spi_speed;
       tr_dat2.tx_buf = spi_cmd;
       tr_dat2.rx_buf = rd_buf;
       tr_dat2.len = 2;

       spi_message_init (&spi_mess);
       spi_message_add_tail (&tr_reg, &spi_mess);
       spi_message_add_tail (&tr_dat1, &spi_mess);
       spi_message_add_tail (&tr_dat2, &spi_mess);
       error = spi_sync(jmt101->spi, &spi_mess);
       if (error) {
          dev_err (&jmt101->spi->dev, "spi_sync failed.\n");
          return error;
       }
          
       memcpy(jmt101->huge_buffer + rx_off + sa->sensor_size, rd_buf + 1, 1);
   }
   
   /* re-organize frame data */
	arrange_frame (jmt101, jmt101->huge_buffer + rx_off + 1);
#if 1
    switch(sa->version){
        case 0:
            buf_source = 1;
            buf_target = 2;
            memset (rd_buf, 0x00, sa->frame_size);
            for (row = 0; row < sa->frame_height; row++) {
                memcpy (&rd_buf[buf_target], jmt101->huge_buffer + rx_off + buf_source, sa->sensor_width);
                buf_source += sa->sensor_width;
                buf_target += sa->frame_width;
            }

            /* test pattern */
            if (jmt101->debug.trace_mask & JMT101_TRACE_DPATTERN) {
                pixel = 0x00;
                for (idx = 0; idx < sa->frame_size; idx++) {
                    rd_buf[idx] = pixel;
                    pixel = (pixel + 8) & 0xFF;
                }
            }

            /* tag frame calibration parameter */
                if (jmt101->control.flag & JMT_SWITCH_PARAMETER) {
                    rd_buf[JMT101A_POS_GAIN] = regs_table[REG_GAIN][1];
                        rd_buf[JMT101A_POS_VRT] = regs_table[REG_VRT][1];
                        rd_buf[JMT101A_POS_VRB] = regs_table[REG_VRB][1];
                        rd_buf[JMT101A_POS_OFFSET] = regs_table[REG_DCOFFSET][1];
                }
            
                /* tag frame time stamp */
                if (jmt101->control.flag & JMT_SWITCH_TIMESTAMP) {
                    getnstimeofday (&ts_now);
                        /* resolution is .1ms */
                        ts_stamp = ts_now.tv_sec * 10000 + ts_now.tv_nsec / 100;
                        rd_buf[JMT101A_POS_TIMESTAMP] = ts_stamp & 0x00FF;
                        rd_buf[JMT101A_POS_TIMESTAMP + 1] = (ts_stamp >> 8) & 0x00FF;
                        rd_buf[JMT101A_POS_TIMESTAMP + 2] = (ts_stamp >> 16) & 0x00FF;
                        rd_buf[JMT101A_POS_TIMESTAMP + 3] = (ts_stamp >> 24) & 0x00FF;
                }
            
                memcpy (jmt101->huge_buffer + rx_off, rd_buf, sa->frame_size);

            /* do intensity calibration */
            if (jmt101->control.flag & JMT_SWITCH_CALIBRATE) {
                do_calibrate (jmt101, rd_buf);
            }
            break;
        case 1:
            buf_source = 1;
            buf_target = 0;

           // memset (rd_buf, 0x00, sa->frame_size);
            memset (rd_buf, 0x00, 1024);
           // for (row = 0; row < sa->frame_height; row++) {
            for (row = 0; row < 8; row++) {
                //memcpy (&rd_buf[buf_target], jmt101->huge_buffer + rx_off + buf_source, sa->frame_width);
                memcpy (&rd_buf[buf_target], jmt101->huge_buffer + rx_off + buf_source, 128);
                buf_source += 128;
                buf_target += 128;
                //buf_source += sa->sensor_width;
                //buf_target += sa->frame_width;
            }

            /* test pattern */
            if (jmt101->debug.trace_mask & JMT101_TRACE_DPATTERN) {
                pixel = 0x00;
                //for (idx = 0; idx < sa->frame_size; idx++) {
                for (idx = 0; idx < 1024; idx++) {
                    rd_buf[idx] = pixel;
                    pixel = (pixel + 8) & 0xFF;
                }
            }

            /* tag frame calibration parameter */

            if (jmt101->control.flag & JMT_SWITCH_PARAMETER) {
                rd_buf[JMT101B_POS_GAIN] = regs_table[REG_GAIN][1];
                rd_buf[JMT101B_POS_VRT] = regs_table[REG_VRT][1];
                rd_buf[JMT101B_POS_VRB] = regs_table[REG_VRB][1];
                rd_buf[JMT101B_POS_OFFSET] = regs_table[REG_DCOFFSET][1];
            }

            /* tag frame time stamp */
            if (jmt101->control.flag & JMT_SWITCH_TIMESTAMP) {
                getnstimeofday (&ts_now);
                /* resolution is .1ms */
                ts_stamp = ts_now.tv_sec * 10000 + ts_now.tv_nsec / 100;
                rd_buf[JMT101B_POS_TIMESTAMP1] = ts_stamp & 0x00FF;
                rd_buf[JMT101B_POS_TIMESTAMP2] = (ts_stamp >> 8) & 0x00FF;
                rd_buf[JMT101B_POS_TIMESTAMP3] = (ts_stamp >> 16) & 0x00FF;
                rd_buf[JMT101B_POS_TIMESTAMP4] = (ts_stamp >> 24) & 0x00FF;
            }

            memcpy (jmt101->huge_buffer + rx_off, rd_buf, sa->frame_size);
            //memcpy (jmt101->huge_buffer + rx_off, rd_buf, 1024);

            /* do intensity calibration */
            if (jmt101->control.flag & JMT_SWITCH_CALIBRATE) {

                do_calibrate (jmt101, rd_buf);
            }

            break;
        case 2:
            memcpy(rd_buf, jmt101->huge_buffer+ rx_off, sa->frame_size);
            if(!sa->hw_dvr) {
                do_calibrate(jmt101, rd_buf);
            }
            break;
        default:
            break;
    }
#else
        buf_source = 1;
            buf_target = 0;

           // memset (rd_buf, 0x00, sa->frame_size);
            memset (rd_buf, 0x00, 1024);
           // for (row = 0; row < sa->frame_height; row++) {
            for (row = 0; row < 8; row++) {
                //memcpy (&rd_buf[buf_target], jmt101->huge_buffer + rx_off + buf_source, sa->frame_width);
                memcpy (&rd_buf[buf_target], jmt101->huge_buffer + rx_off + buf_source, 128);
                buf_source += 128;
                buf_target += 128;
                //buf_source += sa->sensor_width;
                //buf_target += sa->frame_width;
            }

            /* test pattern */
            if (jmt101->debug.trace_mask & JMT101_TRACE_DPATTERN) {
                pixel = 0x00;
                //for (idx = 0; idx < sa->frame_size; idx++) {
                for (idx = 0; idx < 1024; idx++) {
                    rd_buf[idx] = pixel;
                    pixel = (pixel + 8) & 0xFF;
                }
            }

            /* tag frame calibration parameter */

            if (jmt101->control.flag & JMT_SWITCH_PARAMETER) {

                rd_buf[JMT101B_POS_GAIN] = regs_table[REG_GAIN][1];

                rd_buf[JMT101B_POS_VRT] = regs_table[REG_VRT][1];

                rd_buf[JMT101B_POS_VRB] = regs_table[REG_VRB][1];

                rd_buf[JMT101B_POS_OFFSET] = regs_table[REG_DCOFFSET][1];

            }


            /* tag frame time stamp */

            if (jmt101->control.flag & JMT_SWITCH_TIMESTAMP) {

                getnstimeofday (&ts_now);

                /* resolution is .1ms */

                ts_stamp = ts_now.tv_sec * 10000 + ts_now.tv_nsec / 100;

                rd_buf[JMT101B_POS_TIMESTAMP1] = ts_stamp & 0x00FF;

                rd_buf[JMT101B_POS_TIMESTAMP2] = (ts_stamp >> 8) & 0x00FF;

                rd_buf[JMT101B_POS_TIMESTAMP3] = (ts_stamp >> 16) & 0x00FF;

                rd_buf[JMT101B_POS_TIMESTAMP4] = (ts_stamp >> 24) & 0x00FF;

            }


            //memcpy (jmt101->huge_buffer + rx_off, rd_buf, sa->frame_size);
            memcpy (jmt101->huge_buffer + rx_off, rd_buf, 1024);

            /* do intensity calibration */
            if (jmt101->control.flag & JMT_SWITCH_CALIBRATE) {

                do_calibrate (jmt101, rd_buf);
            }

        //    break;
#endif
   /* do intensity calibration */
   //if (rd_buf) {
      //free_pages ((unsigned long)rd_buf, get_order(JMT_FRAME_SIZE));
     // free_pages ((unsigned long)rd_buf, get_order(sa->frame_size));
   //}
   return 0;
}


static int jmt101_sync_reg (struct jmt101_data *jmt101)
{
   int error = 0;
   int i;

   for (i = 0; i < ARRAY_SIZE(regs_table); i++) {
      error = jmt101_spi_rd_reg(jmt101, regs_table[i][0], &(regs_table[i][1]));
      if (error) {
         break;
      }
   }

   return (error);
}
/* -------------------------------------------------------------------- */
static int jmt101_frame_ready (struct jmt101_data *jmt101)
{
    u8 data = 0;

    if (down_interruptible(&jmt101->mutex)) goto err;
    jmt101_spi_rd_reg (jmt101, JMT_REG_STATUS, &data);
    up (&jmt101->mutex);

err:
  //  printk("charles %s: get 0xD3 reg data=0x%x, return 0x%x\n",__func__, data, data&0x04);
    
    return (data & 0x04);
}

static int jmt101_capture_task (struct jmt101_data *jmt101)
{
   int error = 0;
  // int frame_ready_cnt = 0;

   // bool finger_present = false;

   u32 total_captures = 0;
   struct timespec ts_start, ts_end, ts_delta;

   // dev_err (&jmt101->spi->dev, "[jmt101]%s: IN\n", __func__);

   getnstimeofday (&ts_start);
   jmt101->frame_retry = 0;

   while (1) {
      if (jmt101->thread_task.should_stop || suspended) {
   //      printk( "[jmt101]%s: thread_task.should_stop\n", __func__);
         error = -EINTR;
         break;
      }

      if (!jmt101_frame_ready(jmt101)) {
         /* endless protect */
         if (jmt101->frame_retry++ > JMT_FRAME_READY_LIMIT) {
     //       printk( "[jmt101]%s: jmt101->frame_retry++ > JMT_FRAME_READY_LIMIT\n", __func__);
         //if (frame_ready_cnt++ > jmt101->sa.frame_ready_limit) {
            // jmt101_soft_reset (jmt101);
            error = -EIO;
            break;
         }

         continue;
      }
      jmt101->frame_retry = 0;
      //frame_ready_cnt = 0;

      //if (jmt101->current_frame >= (JMT_MAX_FRAMES - 1)) {
        //    printk( "[jmt101]%s: jmt101->current_frame >= (JMT_MAX_FRAMES - 1)\n", __func__);
      if (jmt101->current_frame >= (jmt101->sa.max_frames - 1)) {
      //    dev_err (&jmt101->spi->dev, "[jmt101]%s: out of buffer, current_frame=%d\n", __func__, jmt101->current_frame);
         break;
      }

      if (down_interruptible(&jmt101->mutex)) return -ERESTARTSYS;
           // printk( "[jmt101]%s: call jmt101_spi_rd_image\n", __func__);
      error = jmt101_spi_rd_image(jmt101);
      up (&jmt101->mutex);

      if (error) {
         if (jmt101->debug.trace_mask & JMT101_TRACE_DEVERR) {
            printk("jmt101_spi_rd_capture %i\n", error);
         }
         break;
      }

      /* waiting for finger present */
      /* remove finger present check to avoid read block
      if (!finger_present) {
         finger_present = (frame_gray_sum(jmt101->huge_buffer + jmt101->current_frame * JMT101_FRAME_SIZE, JMT101_FRAME_SIZE) > 10) ? true : false;

         if (!finger_present) {
            continue;
         }
      }
      */

      if (down_interruptible(&jmt101->mutex)) return -ERESTARTSYS;
      jmt101->current_frame++;
      //jmt101->avail_data += JMT_FRAME_SIZE;
      //      dev_err (&jmt101->spi->dev, " avail_data = %i\n", jmt101->avail_data);

      jmt101->avail_data += jmt101->sa.frame_size;
      up (&jmt101->mutex);
      wake_up_interruptible (&jmt101->waiting_data_avail);

      total_captures++;
   }

   getnstimeofday (&ts_end);
   ts_delta = timespec_sub(ts_end, ts_start);

   jmt101->diag.capture_time = ts_delta.tv_nsec / NSEC_PER_MSEC;
   jmt101->diag.capture_time += (ts_delta.tv_sec * MSEC_PER_SEC);

   jmt101->diag.frames_stored = jmt101->current_frame;
   jmt101->diag.frames_captured = total_captures;

   if (error) {
      if (down_interruptible(&jmt101->mutex)) return -ERESTARTSYS;
      jmt101->avail_data = 0;
      up (&jmt101->mutex);

   //    dev_err (&jmt101->spi->dev, "capture_task failed with error %i\n", error);
   }

   jmt101->capture_done = 1;
   wake_up_interruptible (&jmt101->waiting_data_avail);

   // dev_err (&jmt101->spi->dev, "[jmt101]%s: OUT with error %d\n", __func__, error);
   return error;
}

/* -------------------------------------------------------------------- */
static int threadfn (void *_jmt101)
{
   struct jmt101_data *jmt101 = _jmt101;

   while (!kthread_should_stop()) {
      up (&jmt101->thread_task.sem_idle);
      wait_event_interruptible (jmt101->thread_task.wait_job,
         (jmt101->thread_task.mode != JMT101_THREAD_IDLE_MODE) || kthread_should_stop());
      down (&jmt101->thread_task.sem_idle);

      if (kthread_should_stop()) {
         jmt101->thread_task.mode = JMT101_THREAD_EXIT;
         break;
      }

      switch (jmt101->thread_task.mode) {
      case JMT101_THREAD_CAPTURE_MODE:
         jmt101_capture_task(jmt101);
         break;

      default:
         break;
      }

      if (jmt101->thread_task.mode != JMT101_THREAD_EXIT) {
         jmt101->thread_task.mode = JMT101_THREAD_IDLE_MODE;
      }
   }

   return 0;
}

/* -------------------------------------------------------------------- */
static int jmt101_abort_capture (struct jmt101_data *jmt101)
{
   // dev_err (&jmt101->spi->dev, "[jmt101]%s: IN\n", __func__);

   jmt101->thread_task.should_stop = 1;
   jmt101->thread_task.mode = JMT101_THREAD_IDLE_MODE;

   /* wait for thread_task to stop */
   if (down_interruptible(&jmt101->thread_task.sem_idle))
      return -ERESTARTSYS;
   up (&jmt101->thread_task.sem_idle);

   jmt101->current_frame = 0;
   jmt101->avail_data = 0;
   jmt101->data_offset = 0;

   return (0);
}

/* -------------------------------------------------------------------- */
static int jmt101_start_capture (struct jmt101_data *jmt101)
{
   // dev_err (&jmt101->spi->dev, "[jmt101]%s: IN\n", __func__);

   jmt101_abort_capture (jmt101);

   jmt101->thread_task.should_stop = 0;
   jmt101->thread_task.mode = JMT101_THREAD_CAPTURE_MODE;
   wake_up_interruptible (&jmt101->thread_task.wait_job);

   jmt101->capture_done = 0;

   return (0);
}

/* -------------------------------------------------------------------- */
#ifdef CONFIG_ARCH_MT6735M
static void jmt101_interrupt (void)
#else
irqreturn_t jmt101_interrupt (int irq, void *_jmt101)
#endif
{
#ifdef CONFIG_ARCH_MT6735M
   if (mt_get_gpio_in(g_jmt101->irq_gpio)) {
      g_jmt101->interrupt_done = 1;
      // printk ("[jmt101]%s: OUT WakeupInterruptible\n", __func__);
      wake_up_interruptible (&g_jmt101->waiting_interrupt_return);
   }

#else
   struct jmt101_data *jmt101 = _jmt101;

   if (gpio_get_value(jmt101->irq_gpio)) {
      jmt101->interrupt_done = 1;
      wake_up_interruptible (&jmt101->waiting_interrupt_return);
      return IRQ_HANDLED;
   }

   return IRQ_NONE;
#endif
}

/* -------------------------------------------------------------------- */
static int jmt101_selftest (struct jmt101_data *jmt101)
{
   int error = -1;

   error = jmt101_abort_capture(jmt101);
   if (error) {
      dev_err (&jmt101->spi->dev, "jmt101 selftest, " "reset fail on entry.\n");
      goto err;
   }

   if (down_interruptible(&jmt101->mutex)) goto err;
   error = jmt101_spi_rd_reg(jmt101, JMT_REG_VER_ID_MAJOR, jmt101->huge_buffer + jmt101->data_offset);
   up (&jmt101->mutex);
   if (error)
      goto err;

#if 0
   if (jmt101->huge_buffer[jmt101->data_offset] != JMT_VER_ID_MAJOR) {
      dev_err (&jmt101->spi->dev, "selftest id mismatch: %x expected %x\n", jmt101->huge_buffer[jmt101->data_offset], JMT_VER_ID_MAJOR);
      error = -EIO;
      goto err;
   }
#endif
err:
   if (error == 0) {
      jmt101->diag.selftest = jmt101->huge_buffer[jmt101->data_offset];
   } else {
      jmt101->diag.selftest = 0x00;
   }

   return error;
}

/* -------------------------------------------------------------------- */
static int jmt101_open (struct inode *inode, struct file *file)
{
    int error = 0;
    struct jmt101_data *jmt101;
    jmt101 = container_of(inode->i_cdev, struct jmt101_data, cdev);
        // dev_err (&jmt101->spi->dev, "[jmt101]%s: IN\n", __func__);

    if (down_interruptible(&jmt101->mutex))
        return -ERESTARTSYS;

    file->private_data = jmt101;

// Charles ####
#if 0  // _1016_
    // Charles edited 2014_0905 
printk("%s: call jmt101_sync_reg", __func__);
   error = jmt101_sync_reg(jmt101);
    if (error)
       goto out;
#endif
#if 0 
printk("%s: call jmt101_soft_reset", __func__);
    error =jmt101_soft_reset (jmt101);
    if (error)
       goto out;
#endif
    // Charles edited 2014_0905 
    //error = jms0101_init_sensor(jmt101);
    //if (error)
    //   goto out;

    // Charles edited 2014_0905 
    /* check sensor mode */
out:
   up (&jmt101->mutex);

   if (!error) {
      /* race condition possible */
printk("%s: call atomic_read", __func__);
      jmt101->diag.count_file = atomic_read(&file->f_count);
   }
    return error;
}

/* -------------------------------------------------------------------- */
static int jmt101_release (struct inode *inode, struct file *file)
{
      struct jmt101_data *jmt101 = file->private_data;

   jmt101_abort_capture (jmt101);

   /* race condition possible */
   jmt101->diag.count_file = atomic_read(&file->f_count);

   if (!atomic_read(&file->f_count)) {
      /* go to sleep or shutdown mode */
   }

   return (0);
}

/* -------------------------------------------------------------------- */
static ssize_t jmt101_read (struct file *file, char *buff,
        size_t count, loff_t *ppos)
{
    int error = 0;
    struct jmt101_data *jmt101 = file->private_data;
    unsigned int max_dat;
    struct sensor_attr* sa = &jmt101->sa;
    //dev_err (&jmt101->spi->dev, "[jmt101]%s: IN count=%d avail_data=%d, capture_done=%d\n", __func__, count, jmt101->avail_data, jmt101->capture_done);

    if (!jmt101->capture_done) {
        wait_event_interruptible(jmt101->waiting_data_avail, (jmt101->capture_done || jmt101->avail_data));
    }

    if (down_interruptible(&jmt101->mutex)){
        return -ERESTARTSYS;
    }
    /* protect data_offset overflow */
    //if ((count + jmt101->data_offset) > JMT_IMAGE_BUFFER_LIMIT) {
      //  count = (JMT_IMAGE_BUFFER_LIMIT - jmt101->data_offset);
        if ((count + jmt101->data_offset) > sa->buffer_limit) {
         count = (sa->buffer_limit - jmt101->data_offset);
    }

    max_dat = (count > jmt101->avail_data) ? jmt101->avail_data : count;
    if (max_dat) {
        error = copy_to_user(buff, &jmt101->huge_buffer[jmt101->data_offset], max_dat);
        if (error)
            goto out;
        jmt101->data_offset += max_dat;
        jmt101->avail_data -= max_dat;
    }

    error = max_dat;

out:
    up (&jmt101->mutex);
    if (jmt101->diag.count_read++ > JMT_HEARTBEAT_COUNT) {
        jmt101->diag.count_read = 0;
    }

    //printk( "[jmt101]%s: error ret=%d\n", __func__, error);
    return error;
    }

/* -------------------------------------------------------------------- */
static ssize_t jmt101_write (struct file *file, const char *buff,
   size_t count, loff_t *ppos)
{
   return -ENOTTY;
}

/* -------------------------------------------------------------------- */
static unsigned int jmt101_poll (struct file *file, poll_table *wait)
{
   unsigned int ret = 0;
   struct jmt101_data *jmt101 = file->private_data;

    //dev_err (&jmt101->spi->dev, "[jmt101]%s: IN\n", __func__);

   if (jmt101->avail_data == 0 && !jmt101->capture_done)
      poll_wait (file, &jmt101->waiting_data_avail, wait);

    //dev_err (&jmt101->spi->dev, "[jmt101]%s: avail_data = %d capture_done = %d\n", __func__, jmt101->avail_data, jmt101->capture_done);

   /* data ready */
   if (jmt101->avail_data > 0){
      ret |= (POLLIN | POLLRDNORM);
   }

   /* eof */
   else if (jmt101->capture_done)
      ret |= POLLHUP;

   if (jmt101->diag.count_poll++ > JMT_HEARTBEAT_COUNT) {
      jmt101->diag.count_poll = 0;
   }

   return ret;
}

void dump_sensor_attr(struct sensor_attr* sa)
{
    printk("%s: ss_width    =%d\n",__func__,  sa->sensor_width);
    printk("%s: ss_height   =%d\n",__func__,  sa->sensor_height);
    printk("%s: ss_size     =%d\n",__func__,  sa->sensor_size);
    printk("%s: frame_height=%d\n",__func__,  sa->frame_height);
    printk("%s: frame_width =%d\n",__func__,  sa->frame_width);
    printk("%s: frame_size  =%d\n",__func__,  sa->frame_size);
    printk("%s: max_frames  =%d\n",__func__,  sa->max_frames);
    printk("%s: FRL         =%d\n",__func__,  sa->frame_ready_limit);
    return;
}

/* -------------------------------------------------------------------- */
static long jmt101_ioctl (struct file *filp, unsigned int cmd,
        unsigned long arg) {
    int error;
    struct jmt101_data *jmt101 = filp->private_data;

    unsigned int user_regval;
    u8 m_reg;
    u8 m_val;
    u8 out;
    u8 ver = 0xff;

    // dev_err (&jmt101->spi->dev, "[jmt101]%s: IN cmd=0x%.8X\n", __func__, cmd);
    switch (cmd) {
        case JMT_IOCTL_START_CAPTURE:
            if (suspended) {
     //printk( "[jmt101]%s: error =-EBUSY\n", __func__);
                error = -EBUSY;
            } else {
                if (jmt101->control.flag & (JMT_SWITCH_CALIBRATE | JMT_SWITCH_PARAMETER)) {
                    if (!down_interruptible(&jmt101->mutex)) {
                        error = jmt101_sync_reg(jmt101);
                            up (&jmt101->mutex);
                    }
                }
                error = jmt101_start_capture(jmt101);
            }
            break;

        case JMT_IOCTL_ABORT_CAPTURE:
            if (suspended) {
                error = -EBUSY;
            } else {
                error = jmt101_abort_capture(jmt101);
            }
            break;

        case JMT_IOCTL_READ_REGISTER:
            if (copy_from_user(&user_regval, (void __user*)arg, sizeof(user_regval)) != 0) {
                error = -EFAULT;
                break;
            }

            m_reg = (user_regval >> 16) & 0xFF;

            if (down_interruptible(&jmt101->mutex)) return -ERESTARTSYS;
            error = jmt101_spi_rd_reg(jmt101, m_reg, &m_val);
            up (&jmt101->mutex);
            if (error) {
                dev_err (&jmt101->spi->dev, "jmt101_ioctl cmd : %X error : %X\n", cmd, error);

            } else {
                user_regval = m_val;
                if (copy_to_user((void __user*)arg, &user_regval, sizeof(user_regval)) != 0) {
                    error = -EFAULT;
                }
            }

            //dev_err (&jmt101->spi->dev, "JMT_IOC_RD_REG : [%.2X %.2X]\n", m_reg, m_val);

            break;

        case JMT_IOCTL_WRITE_REGISTER:
            if (get_user(user_regval, (unsigned int __user *)arg)) {
                error = -EFAULT;
                break;
            }

            m_reg = (user_regval >> 16) & 0xFF;
            m_val = user_regval & 0xFF;

            if (down_interruptible(&jmt101->mutex)) return -ERESTARTSYS;
            error = jmt101_spi_wr_reg(jmt101, m_reg, m_val);
            up (&jmt101->mutex);
            if (error) {
                dev_err (&jmt101->spi->dev, "jmt101_ioctl cmd : %X error : %X\n", cmd, error);
            }

            // dev_err (&jmt101->spi->dev, "JMT_IOC_WR_REG : [%.2X %.2X]\n", m_reg, m_val);

            break;

        case JMT_IOCTL_WRITE_ATTR:
            if (get_user(user_regval, (unsigned int __user *)arg)) {
                error = -EFAULT;
                //printk("%s: get_user failed       \n",__func__  );
                break;
            }

            struct sensor_attr* p_sa = (struct senssor_attr*)arg;
            memcpy(&jmt101->sa, p_sa, sizeof(struct sensor_attr)); 
	        if(!rd_buf)     rd_buf  = (u8 *)__get_free_pages(GFP_KERNEL, get_order(p_sa->frame_size));
	        if(!tmp_buf)    tmp_buf = (u8 *)kmalloc(p_sa->sensor_size, GFP_KERNEL);
            error=0;
            break;
        case JMT_IOCTL_READ_ATTR:
            if (get_user(user_regval, (unsigned int __user *)arg)) {
                error = -EFAULT;
            //printk("%s: get_user failed       \n",__func__  );
                break;
            }	
            if (copy_to_user((void __user*)arg, &jmt101->sa, sizeof(jmt101->sa)) != 0) {
                    error = -EFAULT;
            }
            dev_err (&jmt101->spi->dev, "JMT_IOCTL_READ_ATTR : [%.2X %.2X]\n", m_reg, m_val);
            error=0;
            break;

        case JMT_IOCTL_RV:
             //if(error = jmt_spi_rd_version(jmt101, &ver)){
            if (down_interruptible(&jmt101->mutex)) return -ERESTARTSYS;
            error = jmt101_spi_rd_reg(jmt101, 0xff, &ver);
            up (&jmt101->mutex);
            if (error) {
                dev_err (&jmt101->spi->dev, "jmt101_ioctl cmd : %X error : %X\n", cmd, error);
                break;
            }

             if (copy_to_user((void __user*)arg, &ver, sizeof(u8)) != 0) {
                 error = -EFAULT;
             }
             break;

        case JMT_IOCTL_SET_OP:
	     printk("jmt101 ioctl JMT_IOCTL_SET_OP \n");
             if (get_user(user_regval, (unsigned int __user *)arg)) {
                 //printk("%s: INIt_REG  get user failed      \n",__func__  );
                 error = -EFAULT;
                 break;
             }	
             struct jmt_op* pop= (struct jmt_op*)arg;
             memcpy(&jmt101->jop, pop, sizeof(struct jmt_op));
	     printk("jmt101_jop.reg is 0x%x \n",jmt101->jop.creg);
	     printk("jmt101_jop.read is 0x%x \n",jmt101->jop.cread);
             error=0;
             break;

        case JMT_IOCTL_SET_CLOCK_SPEED:
             if (get_user(user_regval, (unsigned int __user *)arg)) {
                 //printk("%s: INIt_REG  get user failed      \n",__func__  );
                 error = -EFAULT;
                 break;
             }	
             u32* pspeed = (u32*)arg;
             jmt101->spi_speed = *pspeed;
             //    dev_err (&jmt101->spi->dev,"%s: Set SPI Speed =%d \n",__func__, jmt101->spi_speed  );
                 error=0;
                 break;
        case JMT_IOCTL_SET_VR:
                 if (get_user(user_regval, (unsigned int __user *)arg)) {
                     //printk("%s: INIt_REG  get user failed      \n",__func__  );
                     error = -EFAULT;
                     break;
                 }	
                 u8* vr_regs = (u8*)arg;
                 regs_table[0][1] = vr_regs[0];
                 regs_table[1][1] = vr_regs[1];
                 regs_table[2][1] = vr_regs[2];
                 regs_table[3][1] = vr_regs[3];
                error=0;
                 break;
        default:
             error = -ENOTTY;
             break;
    }

    // dev_err (&jmt101->spi->dev, "[jmt101]%s: OUT error=0x%.8X\n", __func__, error);
    if (jmt101->diag.count_ioctl++ > JMT_HEARTBEAT_COUNT) {
        jmt101->diag.count_ioctl = 0;
    }
    return error;
}

/* -------------------------------------------------------------------- */
static int jmt101_cleanup (struct jmt101_data *jmt101)
{
   if (jmt101->thread_task.thread) {
      jmt101->thread_task.should_stop = 1;
      jmt101->thread_task.mode = JMT101_THREAD_EXIT;
      wake_up_interruptible (&jmt101->thread_task.wait_job);
      kthread_stop (jmt101->thread_task.thread);
   }

   if (!IS_ERR_OR_NULL(jmt101->device))
      device_destroy (jmt101->class, jmt101->devno);

   class_destroy (jmt101->class);

#ifndef CONFIG_ARCH_MT6735M
   if (jmt101->irq >= 0)
      free_irq (jmt101->irq, jmt101);

   if (gpio_is_valid(jmt101->irq_gpio))
      gpio_free (jmt101->irq_gpio);

   if (gpio_is_valid(jmt101->reset_gpio))
      gpio_free(jmt101->reset_gpio);
#endif

   if (jmt101->huge_buffer) {
      free_pages ((unsigned long)jmt101->huge_buffer, get_order(JMT_IMAGE_BUFFER_SIZE));
      //free_pages ((unsigned long)jmt101->huge_buffer, get_order(jmt101->sa.buffer_size));
   }

   if (spi_common_buf) {
      kfree (spi_common_buf);
   }

   if (jmt101) {
      kfree (jmt101);
   }
    
    if(tmp_buf) kfree(tmp_buf);
		
   if (rd_buf) {
      free_pages ((unsigned long)rd_buf, get_order(jmt101->sa.frame_size));
   }
   return 0;
}

/* -------------------------------------------------------------------- */
//static int __devinit jmt101_probe (struct spi_device *spi)
static int jmt101_probe (struct spi_device *spi)
{
   int error;
   struct jmt101_platform_data *jmt101_pdata;
   struct jmt101_data *jmt101 = NULL;

   error = 0;

   jmt101 = kzalloc(sizeof(*jmt101), GFP_KERNEL);
   if (!jmt101) {
      dev_err (&spi->dev, "failed to allocate memory for struct jmt101_data\n");
      return -ENOMEM;
   }
   jmt101->huge_buffer = (u8 *)__get_free_pages(GFP_KERNEL, get_order(JMT_IMAGE_BUFFER_SIZE));
   if (!jmt101->huge_buffer) {
      dev_err (&spi->dev, "failed to get free pages\n");
      kfree (jmt101);
      return -ENOMEM;
   }

   spi_common_buf = kzalloc(16, GFP_KERNEL);
   if (!spi_common_buf) {
      dev_err (&spi->dev, "failed to get spi_common_buf\n");
      if(jmt101->huge_buffer)free_pages ((unsigned long)jmt101->huge_buffer, get_order(JMT_IMAGE_BUFFER_SIZE));
      kfree (jmt101);
      return -ENOMEM;
   }

   /* 4 bytes alignment */
   spi_reg = (u8 *)((u32)(spi_common_buf + 3) & ~ 0x03);
   spi_cmd = spi_reg + 4;
   spi_buf = spi_cmd + 4;

   g_jmt101 = jmt101;
   spi_set_drvdata (spi, jmt101);
   jmt101->spi = spi;
   jmt101->spi_speed = JMT_SPI_CLOCK_SPEED;
   jmt101->reset_gpio = -EINVAL;
   jmt101->irq_gpio = -EINVAL;
   jmt101->irq = -EINVAL;

   /* initial index variable */
   jmt101->current_frame = 0;
   jmt101->capture_done = 1;
   jmt101->data_offset = 0;
   jmt101->avail_data = 0;

   printk ("[jmt101]%s: IN\n", __func__);

   mt_set_gpio_mode(GPIO_FPS_POWER_PIN, GPIO_MODE_00);
   mt_set_gpio_dir(GPIO_FPS_POWER_PIN, GPIO_DIR_OUT);
   mt_set_gpio_out(GPIO_FPS_POWER_PIN, GPIO_OUT_ONE);

   init_waitqueue_head (&jmt101->waiting_interrupt_return);
   init_waitqueue_head (&jmt101->waiting_data_avail);
   jmt101_pdata = spi->dev.platform_data;

   memset (&(jmt101->diag), 0, sizeof(jmt101->diag));
   jmt101->diag.count_poll = 0;
   jmt101->diag.count_read = 0;
   jmt101->diag.count_ioctl = 0;
   jmt101->diag.count_file = 0;

   memset (&(jmt101->control), 0, sizeof(jmt101->control));
   jmt101->control.normal = JMT_INTENSITY_NORMAL;
   jmt101->control.mean_black = JMT_INTENSITY_MEAN_BLACK;
   jmt101->control.mean_white = JMT_INTENSITY_MEAN_WHITE;
   jmt101->control.too_white = JMT_INTENSITY_TOO_WHITE;
   jmt101->control.too_white_px = JMT_HISTOGRAM_TOO_WHITE;
   jmt101->control.flag = (JMT_SWITCH_CALIBRATE | JMT_SWITCH_TIMESTAMP | JMT_SWITCH_PARAMETER);

   memset (&(jmt101->debug), 0, sizeof(jmt101->debug));
   jmt101->debug.trace_mask = 0x00;
   if (!jmt101_pdata) {
      dev_info (&jmt101->spi->dev, "use default jmt101_pdata_mt6577.\n");
      jmt101_pdata = &jmt101_pdata_mt6577;
   }

   /* setup reset_gpio code */
#ifdef CONFIG_ARCH_MT6735M
   error = mt_set_gpio_mode(jmt101_pdata->reset_gpio, GPIO_MODE_00);
   if (error != RSUCCESS) {
      dev_err (&jmt101->spi->dev, "jmt101_probe - mt_set_gpio_mode (reset) failed.\n");
      goto err;
   }

   error = mt_set_gpio_dir(jmt101_pdata->reset_gpio, GPIO_DIR_OUT);
   if (error != RSUCCESS) {
      dev_err (&jmt101->spi->dev, "jmt101_probe - mt_set_gpio_dir (reset) failed.\n");
      goto err;
   }

   mt_set_gpio_out (jmt101_pdata->reset_gpio, GPIO_OUT_ZERO);
   udelay (1000);
   mt_set_gpio_out (jmt101_pdata->reset_gpio, GPIO_OUT_ONE);
   udelay (1250);
#endif
   jmt101->reset_gpio = jmt101_pdata->reset_gpio;
   error = jmt101_reset(jmt101);
   dev_err (&jmt101->spi->dev, "jmt101_probe - reset ret %i[0x%.02X]\n", error, error);

   /* setup irq_gpio code */
#ifdef CONFIG_ARCH_MT6735M
   /*
   error = mt_set_gpio_mode(jmt101_pdata->irq_gpio, 0);
   if (error != RSUCCESS) {
      dev_err(&jmt101->spi->dev, "jmt101_probe - mt_set_gpio_mode (irq) failed.\n");
      goto err;
   }
   */

   error = mt_set_gpio_dir(jmt101_pdata->irq_gpio, GPIO_DIR_IN);
   if (error != RSUCCESS) {
      dev_err(&jmt101->spi->dev, "jmt101_probe - mt_set_gpio_dir (irq) failed.\n");
      goto err;
   }

   error = mt_set_gpio_pull_enable(jmt101_pdata->irq_gpio, GPIO_PULL_ENABLE);
   if (error != RSUCCESS) {
      dev_err (&jmt101->spi->dev, "jmt101_probe - mt_set_gpio_pull_enable (irq) failed.\n");
      goto err;
   }
#endif
   jmt101->irq_gpio = jmt101_pdata->irq_gpio;

#ifdef CONFIG_ARCH_MT6735M
   jmt101->irq = JMT101_INT_IRQNO;
#else
   /* not use real irq
   jmt101->irq = gpio_to_irq(jmt101->irq_gpio);
   */
   jmt101->irq = JMT101_INT_IRQNO;
#endif

   if (jmt101->irq < 0) {
      dev_err (&jmt101->spi->dev, "gpio_to_irq invalid.\n");
      error = jmt101->irq;
      goto err;
   }

   /* setup irq handler code */
//#ifdef CONFIG_ARCH_MT6735M
#if 0
   error = request_irq(jmt101->irq, jmt101_interrupt, IRQF_TRIGGER_RISING, "jmt101", jmt101);

   if (error) {
      dev_err(&jmt101->spi->dev, "request_irq %i failed.\n", jmt101->irq);
      jmt101->irq = -EINVAL;
      goto err;
   }
#endif

#ifdef CONFIG_ARCH_MT6735M
   /*** remember config controller_data for mtk platform ***/
   jmt101->spi->controller_data = (void*)&spi_conf_mt6577;
#endif

   jmt101->spi->mode = SPI_MODE_3;
   jmt101->spi->bits_per_word = 8;

   /* setup spi */
   error = spi_setup(jmt101->spi);
   if (error) {
      dev_err (&jmt101->spi->dev, "spi_setup failed\n");
      goto err;
   }

   jmt101->class = class_create(THIS_MODULE, JMT101_CLASS_NAME);
   if (IS_ERR(jmt101->class)) {
      dev_err (&jmt101->spi->dev, "failed to create class.\n");
      error = PTR_ERR(jmt101->class);
      goto err;
   }

   jmt101->devno = MKDEV(JMT101_MAJOR, jmt101_device_count++);

   jmt101->device = device_create(jmt101->class, NULL, jmt101->devno, NULL, "%s", JMT101_DEV_NAME);
   if (IS_ERR(jmt101->device)) {
      dev_err (&jmt101->spi->dev, "device_create failed.\n");
      error = PTR_ERR(jmt101->device);
      goto err;
   }

   sema_init (&jmt101->mutex, 0);

   error = sysfs_create_group(&spi->dev.kobj, &jmt101_control_attr_group);
   if (error) {
      dev_err (&jmt101->spi->dev, "sysf_create_group failed.\n");
      goto err;
   }

   error = sysfs_create_group(&spi->dev.kobj, &jmt101_diag_attr_group);
   if (error) {
      dev_err (&jmt101->spi->dev, "sysf_create_group failed.\n");
      goto err_sysf_1;
   }

   error = sysfs_create_group(&spi->dev.kobj, &jmt101_debug_attr_group);
   if (error) {
      dev_err (&jmt101->spi->dev, "sysf_create_group failed.\n");
      goto err_sysf_2;
   }

   error = register_chrdev_region(jmt101->devno, 1, JMT101_DEV_NAME);
   if (error) {
       dev_err (&jmt101->spi->dev, "jmt101_probe - register_chrdev_region failed.\n");
       goto err_sysf_3;
   }
   cdev_init (&jmt101->cdev, &jmt101_fops);
   jmt101->cdev.owner = THIS_MODULE;

   error = cdev_add(&jmt101->cdev, jmt101->devno, 1);
   if (error) {
      dev_err (&jmt101->spi->dev, "cdev_add failed.\n");
      goto err_chrdev;
   }

   sema_init (&jmt101->thread_task.sem_idle, 0);
   init_waitqueue_head (&jmt101->thread_task.wait_job);

   jmt101->thread_task.mode = JMT101_THREAD_IDLE_MODE;
   jmt101->thread_task.thread = kthread_run(threadfn, jmt101, "%s", JMT101_WORKER_THREAD_NAME);

   if (IS_ERR(jmt101->thread_task.thread)) {
      dev_err (&jmt101->spi->dev, "kthread_run failed.\n");
      goto err_chrdev;
   }

   suspended = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
   jmt101->early_suspend.suspend = jmt101_early_suspend,
   jmt101->early_suspend.resume = jmt101_late_resume,
   jmt101->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
   register_early_suspend (&jmt101->early_suspend);
#endif

   /* go to sleep mode */

   /* unmask mtk eint irq */
#ifdef CONFIG_ARCH_MT6735M
   // mt_eint_set_sens (jmt101->irq, CUST_EINT_EDGE_SENSITIVE);

   mt_eint_set_polarity (jmt101->irq,1);
   mt_eint_set_hw_debounce (jmt101->irq, 1);
   mt_eint_registration (jmt101->irq, CUST_EINTF_TRIGGER_RISING, jmt101_interrupt, 1);
   mt_eint_unmask (jmt101->irq);
#endif
    
// set sensor attr
   jmt101->sa.vrt_vrb_gap=7;

   up (&jmt101->mutex);

   printk ("[jmt101]%s: OUT\n", __func__);
   return 0;

   cdev_del (&jmt101->cdev);

err_chrdev:
   unregister_chrdev_region (jmt101->devno, 1);

err_sysf_3:
   sysfs_remove_group (&spi->dev.kobj, &jmt101_debug_attr_group);

err_sysf_2:
   sysfs_remove_group (&spi->dev.kobj, &jmt101_diag_attr_group);

err_sysf_1:
   sysfs_remove_group (&spi->dev.kobj, &jmt101_control_attr_group);

err:
   jmt101_cleanup (jmt101);
   spi_set_drvdata (spi, NULL);

  
   printk ("[jmt101]%s: OUT ret=%d\n", __func__, error);
   return error;
}

/* -------------------------------------------------------------------- */
static void jmt101_early_suspend (struct early_suspend *h)
{
   printk ("[jmt101]%s:\n", __func__);
   suspended = 1;
}

/* -------------------------------------------------------------------- */
static void jmt101_late_resume (struct early_suspend *h)
{
   printk ("[jmt101]%s:\n", __func__);
   suspended = 0;
}

/* -------------------------------------------------------------------- */
//static int __devexit jmt101_remove (struct spi_device *spi)
static int jmt101_remove (struct spi_device *spi)
{
   struct jmt101_data *jmt101 = spi_get_drvdata(spi);

   sysfs_remove_group (&jmt101->spi->dev.kobj, &jmt101_control_attr_group);
   sysfs_remove_group (&jmt101->spi->dev.kobj, &jmt101_diag_attr_group);
   sysfs_remove_group (&jmt101->spi->dev.kobj, &jmt101_debug_attr_group);

   /* go to sleep or shutdown mode */

   cdev_del (&jmt101->cdev);
   unregister_chrdev_region (jmt101->devno, 1);
   jmt101_cleanup (jmt101);
   spi_set_drvdata (spi, NULL);

   return 0;
}

/* -------------------------------------------------------------------- */
static int jmt101_suspend (struct device *dev)
{
#ifdef CONFIG_ARCH_MT6735M
   struct jmt101_data *jmt101 = dev_get_drvdata(dev);
#endif

   printk ("[jmt101]%s: IN\n", __func__);

#ifndef CONFIG_HAS_EARLYSUSPEND
#ifdef CONFIG_ARCH_MT6735M
   mt_eint_mask (jmt101->irq);
#endif
#endif

   printk ("[jmt101]%s: OUT\n", __func__);
   return 0;
}

/* -------------------------------------------------------------------- */
static int jmt101_resume (struct device *dev)
{
#ifdef CONFIG_ARCH_MT6735M
   struct jmt101_data *jmt101 = dev_get_drvdata(dev);

   mt_eint_unmask (jmt101->irq);
#endif

   printk ("[jmt101]%s: IN\n", __func__);
   return 0;
}

static struct spi_board_info jmt301 __initdata = {
    .modalias = "jmt101",
    .platform_data = NULL,
    .mode = SPI_MODE_3,
    .max_speed_hz = 16 * 1000 * 1000,
    .bus_num = 0,
    .chip_select = 0,
};

/* -------------------------------------------------------------------- */
static int __init jmt101_init (void)
{
   spi_register_board_info(&jmt301, 1);
   if (spi_register_driver(&jmt101_driver))
      return -EINVAL;

   return 0;
}

/* -------------------------------------------------------------------- */
static void __exit jmt101_exit (void)
{
   spi_unregister_driver (&jmt101_driver);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("J-Matrics <larrygump@moredna.com>");
MODULE_DESCRIPTION("JMT101 swipe sensor driver v0101.");
