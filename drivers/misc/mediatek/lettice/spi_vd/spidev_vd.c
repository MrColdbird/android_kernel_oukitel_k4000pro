/*
 * spi-sensor-hub-spi-fpga.c
 *
 * (c) 2014 Shmuel Ungerfeld <sungerfeld@sensorplatforms.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/workqueue.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/firmware.h>
#include <mach/mt_gpio.h>

#include <linux/input.h>
#include <linux/spi/spi.h>
#include "spidev_vd.h"

#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <mach/mt_spi.h>
#include <mach/eint.h>

#include <mach/mtk_rtc.h>
#include "cust_gpio_usage.h" 
#include "cust_eint.h"

//#define Jay_Debug

#define LATTICE_VD_LOG(fmt, args...) printk("[lattice]:" fmt"\n", ##args) 
#define LATTICE_VD_ERR(fmt, args...) printk(KERN_ERR"[lattice]" fmt"\n", ##args )

//#include "spi-sensor-hub-priv-fpga.h"
//#include <linux/sensor_relay.h>


#define SHOW_SENSOR_DATA 0
#define SHOW_CHANGE_DETECTOR_DATA 1
#define IMPLEMENT_SPIDEV_INTDETECT 1

#if IMPLEMENT_SPIDEV_INTDETECT
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/compat.h>
#include <linux/spi/spidev.h>
#include <asm/uaccess.h>
#include <linux/debugfs.h>

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */
#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */
#define STATUS_ADDR			0x02
#define READ_BYTE			0x01
static DECLARE_BITMAP(minors, N_SPI_MINORS);

/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)

#define SENSHUB_MAJOR 32
#define SENSHUB_NAME "intrdetect"
#define COPY_FROM_USER 1
#define COPY_TO_USER   2 

#endif

#define SUPPORTED_PLATFORMS_LAT_VCD_FW_NAME "vcd_bitmap.bin"

#define DO_HOTPLUG 1

#define SENSHUB_VER_REG_RD     				0X80								/* Version Register Address */
#define SENSHUB_ISR_REG_RD        			0X82								/* ISR Register read Address */
#define SENSHUB_ISR_REG_WR        			0X02								/* ISR Register write Address */

#define SENSHUB_ALT_STATUS_REG_RD  			0X8A								/* BMP085 Status Register Address */
#define	SENSHUB_COMP_STATUS_REG_RD			0X9A								/* Compass Status Register Address */
#define SENSHUB_ACCIL_STATUS_REG_RD			0XAA								/* Accelerometer Status Register Address */
#define SENSHUB_RGB_STATUS_REG_RD			0XBA								/* RGB Light sensor Status Register Address */
#define SENSHUB_HUMID_STATUS_REG_RD			0XCA								/* Humidity sensor Status Register Address */
#define SENSHUB_GYRO_STATUS_REG_RD			0XDA								/* Gyroscope sensor Status Register Address */

#define SENSHUB_ALT_CTRL_REG_RD     		0X88								/* BMP085 Control register read address */
#define	SENSHUB_COMP_CTRL_REG_RD			0X98								/* Compass Control register read address */
#define SENSHUB_ACCIL_CTRL_REG_RD			0XA8								/* Accelerometer Control register read address */
#define SENSHUB_RGB_CTRL_REG_RD				0XB8								/* RGB Control register read address */
#define SENSHUB_HUMID_CTRL_REG_RD			0XC8								/* Humidity Control register read address */
#define SENSHUB_GYRO_CTRL_REG_RD			0XD8								/* Gyroscope Control register read address */

#define SENSHUB_ALT_CTRL_REG_WR     		0X08								/* BMP085 Control register write address */
#define	SENSHUB_COMP_CTRL_REG_WR			0X18								/* Compass Control register write address */
#define SENSHUB_ACCIL_CTRL_REG_WR			0X28								/* Accelerometer Control register write address */
#define SENSHUB_RGB_CTRL_REG_WR				0X38								/* RGB Control register write address */
#define SENSHUB_HUMID_CTRL_REG_WR			0X48								/* Humidity Control Register Write Address */
#define SENSHUB_GYRO_CTRL_REG_WR			0X58								/* Gyroscope Control register write address */

#define SENSHUB_ALT_FIFO_RD					0X8C								/* BMP085 data FIFO address  */
#define	SENSHUB_COMP_FIFO_RD				0X9C								/* Compass data FIFO Address */
#define SENSHUB_ACCEL_FIFO_RD				0XAC								/* Accelerometer data FIFO Address */
#define SENSHUB_RGB_FIFO_RD					0XBC								/* RGB data FIFO Address */
#define SENSHUB_HUMID_FIFO_RD				0xCC								/* Humidity data FIFO Address */
#define SENSHUB_GYRO_FIFO_RD				0xDC								/* Gyroscope data FIFO Address */

#define SENSHUB_ISR_REG_MASK				0X3F									

#define SENSHUB_ISR_RESET					0x00

#define SENSHUB_SOFT_RESET					0x06

#define DATA_REGISTER_RD					0X8c

#define __ENABLE_TEMP															/* Device 0 Temperature/Pressure	BMP085 		*/	
#define __ENABLE_COMP															/* Device 1 Compass 				LSM303DLHC 	*/
#define __ENABLE_ACCEL															/* Device 2 Accelerometer 			LSM330DLC 	*/
#define __ENABLE_RGB															/* Device 3 RGB 					MAX44006 	*/
#define __ENABLE_HUMIDITY														/* Device 4 Humidity 				SHT20 		*/
#define __ENABLE_GYRO															/* Device 5 Gyroscope 				LSM330DLC 	*/


extern void voicewakup_report();

#define LATTICE_VD_LOG(fmt, args...) printk("[lattice]:" fmt"\n", ##args) 
#define LATTICE_VD_ERR(fmt, args...) printk(KERN_ERR"[lattice]" fmt"\n", ##args )

#define SPI_SH_RETRY_COUNT	3

//lattice add
//#define GPIO_SPI_CS_PIN_M_SPI_CSA       1
//#define  GPIO_SPI_SCK_PIN_M_SPI_CKA          1
//#define GPIO_SPI_MISO_PIN_M_SPI_MIA      1
//#define GPIO_SPI_MOSI_PIN_M_SPI_MOA      1
//end

#ifdef GPIO_SPI_CS_PIN
#define SH_VD_SPI_CS_PIN			        GPIO_SPI_CS_PIN
#define SH_VD_SPI_CS_MODE                        GPIO_SPI_CS_PIN_M_SPI_CSA

#define SH_VD_SPI_SCK_PIN                          GPIO_SPI_SCK_PIN
#define SH_VD_SPI_SCK_MODE                        GPIO_SPI_SCK_PIN_M_SPI_CKA

#define SH_VD_SPI_MISO_PIN                         GPIO_SPI_MISO_PIN
#define SH_VD_SPI_MISO_MODE                      GPIO_SPI_MISO_PIN_M_SPI_MIA

#define SH_VD_SPI_MOSI_PIN                         GPIO_SPI_MOSI_PIN
#define SH_VD_SPI_MOSI_MODE                      GPIO_SPI_MOSI_PIN_M_SPI_MOA

#elif defined GPIO_SPI2_CS_PIN
#define SH_VD_SPI_CS_PIN			        GPIO_SPI2_CS_PIN
#define SH_VD_SPI_CS_MODE                        GPIO_SPI2_CS_PIN_M_SPI2_CSB

#define SH_VD_SPI_SCK_PIN                          GPIO_SPI2_SCK_PIN
#define SH_VD_SPI_SCK_MODE                        GPIO_SPI2_SCK_PIN_M_CLK

#define SH_VD_SPI_MISO_PIN                         GPIO_SPI2_MISO_PIN
#define SH_VD_SPI_MISO_MODE                      GPIO_SPI2_MISO_PIN_M_SPI2_MI

#define SH_VD_SPI_MOSI_PIN                         GPIO_SPI2_MOSI_PIN
#define SH_VD_SPI_MOSI_MODE                      GPIO_SPI2_MOSI_PIN_M_SPI2_MO


#else
    error 
#endif


#define GPIO_CRESET		        GPIO_LETTIC_ICE_RESET_PIN	
#define GPIO_CS			        SH_VD_SPI_CS_PIN
#define SPI_SH_HOST_GPIO_IRQ	CUST_EINT_LETTIC_ICE_EINT_NUM

static struct spi_driver hct_spi_sh_driver;

static struct spi_sh_platform_data spi_sh_pdata = {
   	.hostInterruptGpio = SPI_SH_HOST_GPIO_IRQ,
   	.fpgaCresetGpio = GPIO_CRESET,
   	.fpgaSsGpio = GPIO_CS,
};

static int start_vd =0; 

struct spi_sh * g_pspi_sh;
static struct spi_transfer tr[2];

static struct mt_chip_conf spi_conf;
static struct mt_chip_conf *spi_par = NULL;
static int spi_sh_set_clock_freq(int freq);

#define H_BYTE(X) ((u8)((X >> 8) & 0x00FF))
#define L_BYTE(X) ((u8)((X) & 0x00FF))


struct spi_sh {
    
    struct spi_device *spi_client;
	struct spi_sh_platform_data *pdata;
	
	struct input_dev *input_dev_detect;
	
	enum SUPPORTED_PLATFORMS activePlatform;
	char *firmwareFileName;
	uint8_t isSpiSensorhub;
	uint8_t isInterruptsConfigured;
	uint8_t isPlatformConfigured;
	uint8_t isGpioAllocated;

	/*
	 * The mutex prevents a race between the enable_irq()
	 * *  in the workqueue and the free_irq() in the remove function.
	 */
	struct mutex mutex;
	int exiting;
	bool suspended;
	int irq;

	/* Caliberation const for Temp & Pressure */
	uint16_t AC4, AC5, AC6;

    /* Caliberation const for Temp & Pressure */

	int sampleCounter;    /* for debug purpose only */


#if IMPLEMENT_SPIDEV_INTDETECT
	dev_t			devt;
	spinlock_t		spi_lock;
	struct list_head	device_entry;
	struct list_head	ir_device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex	buf_lock;
	unsigned		users;
	uint8_t			*buffer;
	uint8_t			*bufferrx;

#endif
	
};

static struct spi_device *spi_irq;

struct delayed_work input_work_rx;
/* Forward definitions */
int configure_iCE(struct spi_sh *spi_sh);

#define SH_VD_IMAGE_BUFSIZE  (96 * 1024)

static unsigned char *sh_vd_image_buf = NULL;

//lattice add
static unsigned char *g_send_buf = NULL;
static unsigned char *g_reciver_bufer = NULL;


#if IMPLEMENT_SPIDEV_INTDETECT
static LIST_HEAD(device_list);
static LIST_HEAD(ir_device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 9216;						//4096  //8192
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

static unsigned char *sh_vd_tx_buf = NULL;
static unsigned char *sh_vd_rx_buf = NULL;

static unsigned int counter = 0;
static char string [128];
static int data;
static int interrupt_occured = 0;
struct dentry *file_test1;
DECLARE_WAIT_QUEUE_HEAD(senshub_intr_wait);

static void ice_spi_report_values(struct spi_device *spi)
{
	struct spi_sh *spidev = spi_get_drvdata(spi);
     	LATTICE_VD_LOG("%s\n",__func__);
#if 0
	input_report_key(spidev->input_dev_detect, KEY_POWER, 0);
	input_sync(spidev->input_dev_detect);
	
	input_report_key(spidev->input_dev_detect, KEY_POWER, 1);
	input_sync(spidev->input_dev_detect);
#else
     voicewakup_report();
#endif	
}

static void spi_work_func_rx(struct work_struct *work)
{
	struct spi_device *spi=spi_irq;
	if(start_vd == 1){
		ice_spi_report_values(spi);
		start_vd =0;
	}
	return;
}


static int senshub_intr_open (struct inode *inode, struct file *file) {
    struct spi_sh *spi_sh = NULL;

    printk("\nIR DETECT device open\n");
    
	mutex_lock(&device_list_lock);
    list_for_each_entry(spi_sh, &ir_device_list, ir_device_entry) {
        file->private_data = spi_sh;
        break;
    }
	mutex_unlock(&device_list_lock);

    return 0;
}

static int senshub_intr_release (struct inode *inode, struct file *file) {
    printk("\nIR DETECT device release\n");
    return 0;
}

static ssize_t senshub_intr_read (struct file *file, char *buf, size_t count, loff_t *ppos) {
    int ret;
	if(!interrupt_occured)
     		   interruptible_sleep_on_timeout(&senshub_intr_wait,8*HZ); 
     if(interrupt_occured == 0){
		return 5;
	}

    ret = interrupt_occured;
    interrupt_occured = 0;
    return ret;
}

static ssize_t senshub_intr_write (struct file *file, const char *buf, size_t count, loff_t *ppos) {
    int err;
    printk("Driver-senshub intr write\n");	
    err = copy_from_user(string,buf,count);
    if (err != 0)
    	return -EFAULT;
    counter += count;
    return count;
}

static long senshub_intr_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    int retval = 0;
    printk("Driver-senshub-ioctl\n");
    switch (cmd) {
	case COPY_FROM_USER:
	    if (copy_from_user(&data, (int *)arg, sizeof(int)))
		return -EFAULT;
		break;
	case COPY_TO_USER:
	    if (copy_to_user((int *)arg, &data, sizeof(int)))
		return -EFAULT;
		break;
	default:
	    retval = -EINVAL;
	}
	return retval;
}


struct file_operations intr_detect_fops = {
    .owner = THIS_MODULE,
    .llseek = NULL,
    .read = senshub_intr_read,
    .write = senshub_intr_write,
    .readdir = NULL,
    .poll  = NULL,
    .unlocked_ioctl = senshub_intr_ioctl,
    .mmap = NULL,
    .open = senshub_intr_open,
    .flush = NULL,
    .release = senshub_intr_release,
    .fsync = NULL,
    .fasync = NULL,
    .lock = NULL,
};



/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct spi_sh *spi_sh, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int err;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&spi_sh->spi_lock);
	if (spi_sh->spi_client == NULL)
		err = -ESHUTDOWN;
	else
		err = spi_async(spi_sh->spi_client, message);
	spin_unlock_irq(&spi_sh->spi_lock);

	if (err == 0) {
		wait_for_completion(&done);
		err = message->status;
		if (err == 0)
			err = message->actual_length;
	}
	return err;
}

static inline ssize_t
spidev_sync_write(struct spi_sh *spi_sh, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= spi_sh->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spi_sh, &m);
}

static inline ssize_t
spidev_sync_read(struct spi_sh *spi_sh, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= spi_sh->buffer,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spidev_sync(spi_sh, &m);
}


/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct spi_sh *spi_sh;
	ssize_t			err = 0;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spi_sh = filp->private_data;

	mutex_lock(&spi_sh->buf_lock);
	err = spidev_sync_read(spi_sh, count);
	if (err > 0) {
		unsigned long	missing;

		missing = copy_to_user(buf, spi_sh->buffer, err);
		if (missing == err)
			err = -EFAULT;
		else
			err = err - missing;
	}
	mutex_unlock(&spi_sh->buf_lock);

	return err;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct spi_sh *spi_sh;
	ssize_t			err = 0;
	unsigned long		missing;

	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spi_sh = filp->private_data;

	mutex_lock(&spi_sh->buf_lock);
	missing = copy_from_user(spi_sh->buffer, buf, count);
	if (missing == 0) {
		err = spidev_sync_write(spi_sh, count);
	} else
		err = -EFAULT;
	mutex_unlock(&spi_sh->buf_lock);

	return err;
}

static int spidev_message(struct spi_sh *spi_sh,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;     // kernel transfer	
	struct spi_transfer	k_tail_tmp;     // kernel transfer
	struct spi_ioc_transfer *u_tmp;   // user transfer
	unsigned	 n, total;
        u32 tempv = 0;
	u8			*buf, *bufrx;
	int			err = -EFAULT;
	int pkt_count = 0;
	int remainder = 0;


	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spi_sh->buffer;
	bufrx = spi_sh->bufferrx;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			err = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = bufrx;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
		}

               if(k_tmp->len > 1024 )
               {
                   pkt_count = k_tmp->len / 1024;
                   remainder = k_tmp->len % 1024;
                   k_tmp->len = 1024 * pkt_count;
               }
               
               buf += k_tmp->len;
               bufrx += k_tmp->len;

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef Jay_Debug
		LATTICE_VD_LOG("spidev_msg: xfer len %zd %x-%x%s%dbits %u usec %uHz\n",
			u_tmp->len,
			k_tmp->rx_buf , //? "rx " : "",
			k_tmp->tx_buf, // ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u_tmp->bits_per_word ? : spi_sh->spi_client->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spi_sh->spi_client->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);

               if(remainder)
               {
                    k_tail_tmp.len = remainder;
                    if (u_tmp->rx_buf) 
                    {
                        k_tail_tmp.rx_buf = bufrx;
                    }
                    if (u_tmp->tx_buf) 
                    {
                        k_tail_tmp.rx_buf = buf;
                    }
                   
                   k_tail_tmp.cs_change = !!u_tmp->cs_change;
                   k_tail_tmp.bits_per_word = u_tmp->bits_per_word;
                   k_tail_tmp.delay_usecs = u_tmp->delay_usecs;
                   k_tail_tmp.speed_hz = u_tmp->speed_hz;
                   
                   buf += k_tail_tmp.len;
                   bufrx += k_tail_tmp.len;
                   spi_message_add_tail(&k_tail_tmp, &msg);
               }


	}

	err = spidev_sync(spi_sh, &msg);
	if (err < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	buf = spi_sh->bufferrx;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
#ifdef Jay_Debug
                        printk("spidev: read[%d] from %x:",u_tmp->len,buf);
                        for(tempv=0;tempv<u_tmp->len;tempv++)
                           printk("=%d=",*(buf+tempv));
                        printk("--==----finished\n");
#endif
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				err = -EFAULT;
				goto done;
			}
		}
		buf += u_tmp->len;
	}
	err = total;

done:
	kfree(k_xfers);
	return err;
}

static int ice40_spi_dma_xfer( struct spi_sh *spi_sh ,unsigned char *txbuf,unsigned char *rxbuf, int len);

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct spi_sh *spi_sh;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;
        struct spi_ioc_transfer transfer[2];
        void __user *ptr = (void __user*) arg;
        char *log_buf;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spi_sh = filp->private_data;
	spin_lock_irq(&spi_sh->spi_lock);
	spi = spi_dev_get(spi_sh->spi_client);
	spin_unlock_irq(&spi_sh->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spi_sh->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;
        
        case SPI_IOC_WR_VD_WORK_MODE:
            retval = __get_user(tmp, (__u32 __user *)arg);
            if (retval == 0) {
                start_vd = tmp;
            }
            break;

        case SPI_IOC_RD_VD_WORK_MODE:
    		retval = __put_user(start_vd, (__u32 __user *)arg);
            break;

            
#if 0
        case SPI_IOC_MESSAGE(1):
            
            LATTICE_VD_LOG("ioctl write");
            if(err = copy_from_user(&transfer, ptr, sizeof(transfer[0])))
            {
                LATTICE_VD_ERR("copy_from_user fail");
                return -1;
            }
            
            LATTICE_VD_LOG("write cmd len=%d", transfer[0].len);
            log_buf = (char *)transfer[0].tx_buf;


            if(err = copy_from_user(sh_vd_tx_buf, transfer[0].tx_buf, transfer[0].len))
            {
                LATTICE_VD_ERR("copy_from_user fail");
                return -1;
            }
            
            LATTICE_VD_LOG("sh_vd_tx_buf=%X, %X, %X, %X", sh_vd_tx_buf[0], sh_vd_tx_buf[1], sh_vd_tx_buf[2], sh_vd_tx_buf[3]);

            //ice40_set_spi_mode(0);
            if(err = ice40_spi_dma_xfer(spi_sh, sh_vd_tx_buf, NULL, transfer[0].len))
            {
                LATTICE_VD_ERR("%s, spi transfer err", __func__);
                return -1;
            }

        break;
    
    
    //read ice40 data
    case SPI_IOC_MESSAGE(2):
        
        LATTICE_VD_LOG("ioctl read");
        if(err = copy_from_user(&transfer, ptr, 2 * sizeof(transfer[0])))
        {
            LATTICE_VD_ERR("copy_from_user fail");
            return -1;
        }
        LATTICE_VD_LOG("write cmd len=%d, read len=%d", transfer[0].len, transfer[1].len);
        log_buf = (char *)transfer[0].tx_buf;
        LATTICE_VD_LOG("write cmd=%X, %X, %X", log_buf[0], log_buf[1], log_buf[2]);
        
        if(err = copy_from_user(sh_vd_tx_buf, transfer[0].tx_buf, transfer[0].len))
        {
            LATTICE_VD_ERR("copy_from_user fail");
            return -1;
        }
        
        //write and read in one cycle
        if(err = ice40_spi_dma_xfer(spi_sh,sh_vd_tx_buf, sh_vd_rx_buf, transfer[0].len + transfer[1].len))
        {
            LATTICE_VD_ERR("%s, spi transfer err", __func__);
            return -1;
        }
        
        //don't care dummy bytes in sh_vd_rx_buf
        if(err = copy_to_user(transfer[1].rx_buf, &sh_vd_rx_buf[transfer[0].len], transfer[1].len))
        {
            LATTICE_VD_ERR("copy_to_user fail");
            return -1;
        }
        
    break;
    
#else

    default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		pr_err("%s: ----------------------------GPIO CS made Low-------------------\n", __func__);
		retval = spidev_message(spi_sh, ioc, n_ioc);
		pr_err("%s: ----------------------------GPIO CS made High-------------------\n", __func__);

		kfree(ioc);
#endif        
    break;
	}

	mutex_unlock(&spi_sh->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return spidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct spi_sh *spi_sh;
	int			err = 0;

#if 1
	mutex_lock(&device_list_lock);

	list_for_each_entry(spi_sh, &device_list, device_entry) {
		if (spi_sh->devt == inode->i_rdev) {
			err = 0;
			break;
		}
	}
	if (err == 0) {
	    dev_err(&spi_sh->spi_client->dev, "spidev_open called\n");
		if (!spi_sh->buffer) {
			spi_sh->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spi_sh->buffer) {
				dev_dbg(&spi_sh->spi_client->dev, "open/ENOMEM\n");
				err = -ENOMEM;
			}
		}
		if (!spi_sh->bufferrx) {
			spi_sh->bufferrx = kmalloc(bufsiz, GFP_KERNEL);
			if (!spi_sh->bufferrx) {
				dev_dbg(&spi_sh->spi_client->dev, "open/ENOMEM\n");
				kfree(spi_sh->buffer);
				spi_sh->buffer = NULL;
				err = -ENOMEM;
			}
		}
		if (err == 0) {
			spi_sh->users++;
			filp->private_data = spi_sh;
			nonseekable_open(inode, filp);
        }
	} else
		pr_debug("spi_sh: nothing for minor %d\n", iminor(inode));

	mutex_unlock(&device_list_lock);
#endif    
	return err;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct spi_sh *spi_sh;
	int			err = 0;
#if 1
	mutex_lock(&device_list_lock);
	spi_sh = filp->private_data;
	filp->private_data = NULL;

	/* last close? */
	spi_sh->users--;
	if (!spi_sh->users) {
		kfree(spi_sh->buffer);
		spi_sh->buffer = NULL;
		kfree(spi_sh->bufferrx);
		spi_sh->bufferrx = NULL;
	}
	mutex_unlock(&device_list_lock);
    
    LATTICE_VD_LOG("spidev_release-users=%d",spi_sh->users);
#endif
	return err;
}


static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *spidev_class;

#endif


static int ice40_spi_dma_xfer( struct spi_sh *spi_sh ,unsigned char *txbuf,unsigned char *rxbuf, int len)
{
	struct spi_message msg;
	spi_message_init(&msg);

	int const pkt_count = len / 1024;
	int const remainder = len % 1024;

	LATTICE_VD_LOG("len=%d, txbuf=0x%p,rxbuf=0x%p",len,txbuf,rxbuf);

	if(len > 1024){	
		tr[0].tx_buf =(txbuf==NULL)?NULL: txbuf;
		tr[0].rx_buf =(rxbuf==NULL)?NULL: rxbuf;
		tr[0].len = 1024 * pkt_count;
		spi_message_add_tail(&tr[0], &msg);

		if(0 != remainder)	 { 
			tr[1].tx_buf =(txbuf==NULL)?NULL:txbuf+ (1024 * pkt_count);
			tr[1].rx_buf =(rxbuf==NULL)?NULL:rxbuf+ (1024 * pkt_count);
			tr[1].len = remainder;
			spi_message_add_tail(&tr[1], &msg);
		}
	}
	else{
		tr[0].tx_buf =(txbuf==NULL)?NULL: txbuf;
		tr[0].rx_buf =(rxbuf==NULL)?NULL: rxbuf;
		tr[0].len = len;
		spi_message_add_tail(&tr[0], &msg);
	}
	
	if(spi_sync(spi_sh->spi_client,&msg))
		return -1;	
	else
		return 0;
}


static irqreturn_t spi_sh_irq(void * ptr)
{

    struct spi_sh *spi_sh = g_pspi_sh;
	
    printk(KERN_INFO "Enter into interrupt\n"); 

    spi_irq=g_pspi_sh->spi_client;	
    LATTICE_VD_LOG("%s\n",__func__);
    if(start_vd == 1)
    {
        schedule_delayed_work(&input_work_rx, 0);
    }
    wake_up_interruptible(&senshub_intr_wait);
    mt_eint_unmask(spi_sh->pdata->hostInterruptGpio);
    if(start_vd != 1){
        interrupt_occured = 1;
    }
    return IRQ_HANDLED;
}



static uint8_t  tx_header_1[] = { 0x7E, 0xAA, 0x99, 0x7E, 0x01, 0x0E };
static uint8_t  tx_header_2[] = { 0x83, 0x00, 0x00, 0x26, 0x11 };
static uint8_t  tx_header_3[] = { 0x83, 0x00, 0x00, 0x27, 0x21 };
static uint8_t  tx_header_4[] = { 0x81 };
static uint8_t  tx_header_8_dummy_bits[1] = { 0 };         // 8 bits       = 1 dummy byte transfered
static uint8_t  tx_header_100_dummy_clocks[13] = { 0 };    // 100 clocks  ~= 13 dummy bytes transfered
static uint8_t  tx_header_13000_dummy_clocks[1625] = { 0 };// 13000 clocks = 1625 dummy bytes transfered




static int spi_vd_int_config(void)
{
    mt_set_gpio_mode(GPIO_LETTIC_ICE_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_LETTIC_ICE_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_LETTIC_ICE_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_LETTIC_ICE_EINT_PIN, GPIO_PULL_UP);

    mt_eint_set_hw_debounce(CUST_EINT_LETTIC_ICE_EINT_NUM, CUST_EINT_LETTIC_ICE_EINT_DEBOUNCE_CN);
    mt_eint_registration(CUST_EINT_LETTIC_ICE_EINT_NUM, CUST_EINTF_TRIGGER_RISING, spi_sh_irq, 1);
    mt_eint_mask(CUST_EINT_LETTIC_ICE_EINT_NUM);

}


static void spi_sh_disable_interrupts(struct spi_sh *spi_sh) {

   	mutex_lock(&spi_sh->mutex);
	spi_sh->exiting = 1;
	mutex_unlock(&spi_sh->mutex);

	if (spi_sh->isInterruptsConfigured)
        {
            printk(KERN_ERR "spi_sh_disable_interrupts GPIO %d \n", spi_sh->pdata->hostInterruptGpio);
            mt_eint_mask(spi_sh->pdata->hostInterruptGpio);
	}
//	cancel_delayed_work_sync(&input_work_rx);
}

static int spi_sh_enable_interrupts(struct spi_sh *spi_sh) {
    int err = 0;
   	pr_err("spi_sh_enable_interrupts \n");
   
   	if (!spi_sh->isInterruptsConfigured) {
   	    
   	    printk(KERN_ERR "spi_sh_enable_interrupts   GPIO %d\n", spi_sh->pdata->hostInterruptGpio );

        if (spi_sh->isSpiSensorhub) {
            mutex_lock(&spi_sh->mutex);
            spi_sh->exiting = 0;
            mutex_unlock(&spi_sh->mutex);
             spi_vd_int_config();
        }  else {
            mutex_lock(&spi_sh->mutex);
            spi_sh->exiting = 0;
            mutex_unlock(&spi_sh->mutex);
            spi_vd_int_config();

            mt_eint_unmask(spi_sh->pdata->hostInterruptGpio);
        }
        if (!err)
            spi_sh->isInterruptsConfigured = 1;
    } 
    else 
    {
        spi_sh_enable_interrupts(spi_sh);
        err = 0;
	interrupt_occured=0;
    }
    
    return err;
}

/*
 mode: 1: means workon mode
       0: standby mode
*/
static int  spi_set_vd_gpio_wrok_mode(int mode)
{

    mt_set_gpio_mode(GPIO_LETTIC_ICE_CD_DONE_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LETTIC_ICE_CD_DONE_PIN, GPIO_DIR_IN);

    if(mode)
    {
        mt_set_gpio_mode(GPIO_LETTIC_ICE_RESET_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LETTIC_ICE_RESET_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LETTIC_ICE_RESET_PIN, 1);

        mt_set_gpio_mode(SH_VD_SPI_CS_PIN, SH_VD_SPI_CS_MODE);
        mt_set_gpio_dir(SH_VD_SPI_CS_PIN, GPIO_DIR_OUT);
        mt_set_gpio_pull_enable(SH_VD_SPI_CS_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(SH_VD_SPI_CS_PIN, GPIO_PULL_UP);

        mt_set_gpio_mode(SH_VD_SPI_SCK_PIN, SH_VD_SPI_SCK_MODE);
        mt_set_gpio_dir(SH_VD_SPI_SCK_PIN, GPIO_DIR_OUT);
        mt_set_gpio_pull_enable(SH_VD_SPI_SCK_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(SH_VD_SPI_SCK_PIN, GPIO_PULL_DOWN);

        mt_set_gpio_mode(SH_VD_SPI_MISO_PIN, SH_VD_SPI_MISO_MODE); 
        mt_set_gpio_dir(SH_VD_SPI_MISO_PIN, GPIO_DIR_IN);          
        mt_set_gpio_pull_enable(SH_VD_SPI_MISO_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(SH_VD_SPI_MISO_PIN, GPIO_PULL_DOWN);

        mt_set_gpio_mode(SH_VD_SPI_MOSI_PIN, SH_VD_SPI_MOSI_MODE); 
        mt_set_gpio_dir(SH_VD_SPI_MOSI_PIN, GPIO_DIR_OUT); 
        mt_set_gpio_pull_enable(SH_VD_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(SH_VD_SPI_MOSI_PIN, GPIO_PULL_DOWN);
    }
   else
    {
        mt_set_gpio_dir(GPIO_LETTIC_ICE_RESET_PIN, GPIO_DIR_IN);

        mt_set_gpio_mode(SH_VD_SPI_CS_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(SH_VD_SPI_CS_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(SH_VD_SPI_CS_PIN, 0);
        
        mt_set_gpio_mode(SH_VD_SPI_SCK_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(SH_VD_SPI_SCK_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(SH_VD_SPI_SCK_PIN, GPIO_PULL_DISABLE);
        
        mt_set_gpio_mode(SH_VD_SPI_MISO_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(SH_VD_SPI_MISO_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(SH_VD_SPI_MISO_PIN, GPIO_PULL_DISABLE);
        
        mt_set_gpio_mode(SH_VD_SPI_MOSI_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(SH_VD_SPI_MOSI_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(SH_VD_SPI_MOSI_PIN, GPIO_PULL_DISABLE);
    }
    return 0;
}

static int spi_sh_read_image_in_kernel(char *path, char * buf, int *bin_size)
{
    struct file *fp = NULL;
    mm_segment_t fs;
    loff_t pos;
    int image_size;

    fs = get_fs();
    set_fs(KERNEL_DS);

    fp = filp_open(path, O_RDONLY, 0);
    //fp = filp_open(ICE40_BIN_PATH, O_RDONLY, 0666);
    if(IS_ERR(fp))
    {
        
        set_fs(fs);
        printk("file(%s)_open fail,err=%x",path,(unsigned int)fp);
        return -1;
    }

    pos = 0;
    image_size = vfs_read(fp, buf, SH_VD_IMAGE_BUFSIZE, &pos);
    LATTICE_VD_LOG("%s, image_size=%d", __func__, image_size);
    *bin_size = image_size;

    filp_close(fp, NULL);
    set_fs(fs);

    return 0;
}


static void spi_sh_power_control(int power)
{
    if(power)
    {
        mt_set_gpio_mode(GPIO_LETTIC_ICE_PWR_EN1_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LETTIC_ICE_PWR_EN1_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LETTIC_ICE_PWR_EN1_PIN, 1);
        mdelay(5);
        mt_set_gpio_mode(GPIO_LETTIC_ICE_PWR_EN2_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LETTIC_ICE_PWR_EN2_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LETTIC_ICE_PWR_EN2_PIN, 1);
    }
    else
    {  		
        mt_set_gpio_out(GPIO_LETTIC_ICE_PWR_EN2_PIN, 0);
        mdelay(10);
        mt_set_gpio_out(GPIO_LETTIC_ICE_PWR_EN1_PIN, 0);
    }
}



static int spi_sh_single_read_reg(struct spi_sh *spi_sh ,u16 cmd, unsigned char *val)
{
    //u8 send_buf[5] = {0};
    //u8 reciver_bufer[5]={0};
    g_send_buf[0]=H_BYTE(cmd);
    g_send_buf[1]=L_BYTE(cmd);
    g_send_buf[2]=0x01;

    LATTICE_VD_LOG("spi_sh_single_read_reg:g_send_buf[0]=0x%x", g_send_buf[0]);
    LATTICE_VD_LOG("spi_sh_single_read_reg:g_send_buf[1]=0x%x", g_send_buf[1]);
    LATTICE_VD_LOG("spi_sh_single_read_reg:g_send_buf[2]=0x%x", g_send_buf[2]);

    ice40_spi_dma_xfer(spi_sh,g_send_buf, g_reciver_bufer, 5);

    *val=g_reciver_bufer[4];

 //LATTICE_VD_LOG("spi_sh_single_read_reg[%x]=%x", cmd, *data);
 //LATTICE_VD_LOG("spi_sh_single_read_reg:reciver_bufer[0]=0x%x", g_reciver_bufer[0]);
 //LATTICE_VD_LOG("spi_sh_single_read_reg:reciver_bufer[1]=0x%x", g_reciver_bufer[1]);
 //LATTICE_VD_LOG("spi_sh_single_read_reg:reciver_bufer[2]=0x%x", g_reciver_bufer[2]);
 //LATTICE_VD_LOG("spi_sh_single_read_reg:reciver_bufer[3]=0x%x", g_reciver_bufer[3]);
 LATTICE_VD_LOG("spi_sh_single_read_reg:reciver_bufer[4]=0x%x", g_reciver_bufer[4]);

    return 0;
}

static int spi_sh_single_write_reg(struct spi_sh *spi_sh ,u16 addr, u8 val)
{
    //u8 send_buf[5] = {0};
    u8 i=0;
    //u8 reciver_bufer[5]={0};
    g_send_buf[0]=H_BYTE(addr);
    g_send_buf[1]=L_BYTE(addr);
    g_send_buf[2]=0x00;

    LATTICE_VD_LOG("spi_sh_single_write_reg:H_BYTE=0x%x", g_send_buf[0]);
    LATTICE_VD_LOG("spi_sh_single_write_reg:L_BYTE=0x%x", g_send_buf[1]);
    //LATTICE_VD_LOG("spi_sh_single_write_reg:g_send_buf[2]=0x%x", g_send_buf[2]);
   // for (i=0; i<0xf; i++)
    {
      g_send_buf[3]= val;
      LATTICE_VD_LOG("spi_sh_single_write_reg:data=0x%x", g_send_buf[3]);
      ice40_spi_dma_xfer(spi_sh,g_send_buf, NULL, 4);
      mdelay(10);
    }

    return 0;
    //*data=reciver_bufer[4];

 //LATTICE_VD_LOG("spi_sh_single_write_reg[%x]=%x", cmd, *data);
 //LATTICE_VD_LOG("spi_sh_single_write_reg:reciver_bufer[0]=0x%x", reciver_bufer[0]);
 //LATTICE_VD_LOG("spi_sh_single_write_reg:reciver_bufer[1]=0x%x", reciver_bufer[1]);
 //LATTICE_VD_LOG("spi_sh_single_write_reg:reciver_bufer[2]=0x%x", reciver_bufer[2]);
 //LATTICE_VD_LOG("spi_sh_single_write_reg:reciver_bufer[3]=0x%x", reciver_bufer[3]);

}


/**************************************************************************************************
 * Function   : configure_iCE
 * Description: This function configures iCE FPGA using SPI lines 
 * Parameters : 
 *   None
 * Returns    : 
 *   0				: On Success
 *   SENSHUB_SPI_SLAVE_INIT_FAILURE: On Failure.
**************************************************************************************************/
#define ICE40_BIN_PATH  "/system/usr/ice40.bin"

int configure_iCE(struct spi_sh *spi_sh)
{
    int rc = 0;

    struct spi_device *spi_client = spi_sh->spi_client;

    uint8_t txdata[3];
    uint8_t rxdata[3];
    int imageSize = 0;
    unsigned char version_no;
    spi_sh->isSpiSensorhub = 0;														
    spi_sh->firmwareFileName = ICE40_BIN_PATH;				
    

    if (1)//(!spi_sh->isGpioAllocated)
    {
        
        spi_sh_power_control(1);
        mdelay(5);
        spi_set_vd_gpio_wrok_mode(1);

        spi_sh->isGpioAllocated = 1;
    }
    spi_sh_disable_interrupts(spi_sh);
	
    if (spi_sh->firmwareFileName != NULL) {
        /* 
         * Extra bytes appended to the data to generate extra 100 clocks after transfering actual data
         */
         rc = spi_sh_read_image_in_kernel(spi_sh->firmwareFileName, sh_vd_image_buf, &imageSize);

        LATTICE_VD_LOG("%s, call_read_img,ret=%d", __func__, rc);

         if(rc)
            goto read_image_failed;

        memset(sh_vd_image_buf + imageSize, 0, sizeof(tx_header_100_dummy_clocks));  /* 100 clocks  ~= 13 dummy bytes transfered */
    

        //reset
        mt_set_gpio_out(GPIO_LETTIC_ICE_RESET_PIN, 0);
        
        mt_set_gpio_mode(SH_VD_SPI_CS_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(SH_VD_SPI_CS_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(SH_VD_SPI_CS_PIN, 0);
        
        mt_set_gpio_mode(SH_VD_SPI_SCK_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(SH_VD_SPI_SCK_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(SH_VD_SPI_SCK_PIN, 1);

#if 0        
        mt_set_gpio_mode(SH_VD_SPI_MISO_PIN, GPIO_MODE_00);
        mt_set_gpio_dir(SH_VD_SPI_MISO_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(SH_VD_SPI_MISO_PIN, 1);
#endif

        ndelay(300);//300 ns
        mt_set_gpio_out(GPIO_LETTIC_ICE_RESET_PIN, 1);
        
        mdelay(2);//800 us
        
        mt_set_gpio_mode(SH_VD_SPI_SCK_PIN, SH_VD_SPI_SCK_MODE);   //spi clk config

        rc = ice40_spi_dma_xfer (spi_sh, sh_vd_image_buf, NULL,imageSize + sizeof(tx_header_100_dummy_clocks));
    
     
        LATTICE_VD_LOG("%s, spi_transfer_dma_ret=%d", __func__, rc);

         if(rc)
            goto spi_down_fw_failed;

        mt_set_gpio_out(SH_VD_SPI_CS_PIN, 1);    

        udelay(1000);

        if(mt_get_gpio_in(GPIO_LETTIC_ICE_CD_DONE_PIN))
        {
            printk("GPIO_LETTIC_ICE_CD_DONE_PIN high level\n"); 
            //lattice test
	    mt_set_gpio_mode(SH_VD_SPI_CS_PIN, SH_VD_SPI_CS_MODE);   //spi clk config
            mt_set_gpio_mode(SH_VD_SPI_MISO_PIN, SH_VD_SPI_MISO_MODE);
            rc =0;
        }
        else
        {
            printk("config_ice:image download fail~~~~~\n");            
            mt_set_gpio_mode(SH_VD_SPI_CS_PIN, SH_VD_SPI_CS_MODE);   //spi clk config
            mt_set_gpio_mode(SH_VD_SPI_MISO_PIN, SH_VD_SPI_MISO_MODE);
            rc=-2; //
//            goto cd_down_failed;
        }

    }
    //lattice add
    //spi_sh_set_clock_freq(1000000);  
    mdelay(100);

    spi_sh_single_read_reg(spi_sh, 0x0003,&version_no);
 
    spi_sh_enable_interrupts(spi_sh);
    spi_sh->isPlatformConfigured = 1;
    
    
   spi_sh_set_clock_freq(1000000);  // set clock to 1M 
    
down_fw_ok:    
    printk("config_ice:image download sucess -----------\n");
    return rc;

cd_down_failed:

spi_down_fw_failed:
    printk("config_ice: spi_down_image_faile\n");

read_image_failed:
     printk("config_ice: read_image_faile\n");
     return rc;

}

#if 0
static ssize_t spi_sh_show_platform(struct device *dev, struct device_attribute *attr,
                char *buf)
{
            return sprintf(buf, "Lattice Sensor Hub SPIDEV interface\n");			
}

static ssize_t spi_sh_store_platform(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct spi_sh *spi_sh = dev_get_drvdata(dev);
	if ((count >= 3) && (strncmp(buf, "VCD", 3) == 0)) {						
		spi_sh->activePlatform = SUPPORTED_PLATFORMS_VCD;				
				}									
    dev_err(&spi_sh->spi_client->dev, "Switching iCE to  %d\n",
        spi_sh->activePlatform);
    

    if (configure_iCE(spi_sh)) {
        dev_err(&spi_sh->spi_client->dev, "unable to configure_iCE\n");
        return count;
    }
	pr_err("%s: Configuration Complete---------------------------\n", __func__);					
	return count;
}

static ssize_t spi_sh_show_normal_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "Enter into normal mode\n");
	start_vd = 1;
	return 0;
}

static DEVICE_ATTR(platformx, S_IRUGO | S_IWUSR, spi_sh_show_platform, spi_sh_store_platform);
static DEVICE_ATTR(normal_mode_en, S_IRUGO | S_IWUSR, spi_sh_show_normal_mode, NULL);


static struct attribute *spi_sh_attributes[] = {
     &dev_attr_platformx.attr,
     &dev_attr_normal_mode_en.attr,	
     NULL
};

static const struct attribute_group spi_sh_group_attr  = {
    .attrs = spi_sh_attributes,
};
#else


static ssize_t spi_sh_show_platform(struct device_driver *ddri, char *buf)
{
            return sprintf(buf, "Lattice Sensor Hub SPIDEV interface\n");			
}

static ssize_t spi_sh_store_platform
(struct device_driver *ddri, char *buf, size_t count)
{
    struct spi_sh *spi_sh = g_pspi_sh;
	if ((count >= 3) && (strncmp(buf, "VCD", 3) == 0)) {						
		spi_sh->activePlatform = SUPPORTED_PLATFORMS_VCD;				
				}									
    dev_err(&spi_sh->spi_client->dev, "Switching iCE to  %d\n",
        spi_sh->activePlatform);
    

    if (configure_iCE(spi_sh)) {
        dev_err(&spi_sh->spi_client->dev, "unable to configure_iCE\n");
        return count;
    }
	pr_err("%s: Configuration Complete---------------------------\n", __func__);					
	return count;
}

static ssize_t spi_sh_show_normal_mode(struct device_driver *ddri, char *buf)
{
	printk(KERN_INFO "Enter into normal mode\n");
	start_vd = 1;
	return 0;
}

static DEVICE_ATTR(platformx, 0666, spi_sh_show_platform, spi_sh_store_platform);
static DEVICE_ATTR(normal_mode_en, 0666, spi_sh_show_normal_mode, NULL);

//lattice add
static ssize_t spi_sh_store_fw_ver
(struct device_driver *ddri, const char *buf, size_t count)
{
        char *p=(char*)buf;
	unsigned long addr_rx = 0;
	unsigned long data_tx = 0;
	unsigned short addr = 0;
        unsigned char val = 0;
	int ret;
	struct spi_sh *spi_sh = g_pspi_sh;

	LATTICE_VD_LOG("enter %s count=%d\n",__func__,count);
	LATTICE_VD_LOG("enter %s buf[0]:%c\n",__func__,buf[0]);	
	LATTICE_VD_LOG("enter %s buf[1]:%c\n",__func__,buf[1]);	
	LATTICE_VD_LOG("enter %s buf[2]:%c\n",__func__,buf[2]);	
	LATTICE_VD_LOG("enter %s buf[3]:%c\n",__func__,buf[3]);	
	LATTICE_VD_LOG("enter %s buf[4]:%c\n",__func__,buf[4]);	
	LATTICE_VD_LOG("enter %s buf[5]:%c\n",__func__,buf[5]);

	if ((count >= 2) && (strncmp(buf, "wt", 2) == 0)) {						
             ret = kstrtoul(&buf[2],16, &data_tx);
		LATTICE_VD_LOG("enter %s addr&data:0x%x\n",__func__,(int)data_tx);
		addr = (unsigned short)((data_tx>>8)&0xffff);
		val = (u8)(data_tx&0xff);
		spi_sh_single_write_reg(spi_sh, addr,val);
        LATTICE_VD_LOG("enter %s write addr:0x%x data:0x%x\n",__func__,addr,val);
	}									
 
	if ((count >= 2) && (strncmp(buf, "rd", 2) == 0)) {						
                ret = kstrtoul(&buf[2],16, &addr_rx);
		//LATTICE_VD_LOG("enter %s addr:0x%x\n",__func__,(int)addr_rx);
		addr = (unsigned short)(addr_rx);
		spi_sh_single_read_reg(spi_sh, addr,&val);
        LATTICE_VD_LOG("enter %s read addr:0x%x data:0x%x\n",__func__,addr,val);
	}
	
	pr_err("%s: Complete---------------------------\n", __func__);	
    return count;	
}

static ssize_t spi_sh_show_fw_ver(struct device_driver *ddri, char *buf)
{
        struct spi_sh *spi_sh = g_pspi_sh;
        unsigned char ice_version;
        //int data=0;
	LATTICE_VD_LOG("enter %s\n",__func__);
	spi_sh_single_read_reg(spi_sh, 0x0003,&ice_version);
        //data = 0x0001;
        //spi_sh_single_write_reg(spi_sh, data);
	return 0;
}
static DEVICE_ATTR(fw_ver, 0666, spi_sh_show_fw_ver, spi_sh_store_fw_ver);

static struct driver_attribute *spi_sh_attr_list[] = {
    &dev_attr_platformx,
    &dev_attr_normal_mode_en, 
    &dev_attr_fw_ver,
};

static int spi_sh_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(spi_sh_attr_list)/sizeof(spi_sh_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, spi_sh_attr_list[idx]))
		{            
			printk("driver_create_file (%s) = %d\n", spi_sh_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int spi_sh_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(spi_sh_attr_list)/sizeof(spi_sh_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, spi_sh_attr_list[idx]);
	}
	return err;
}
#endif
static int spidev_input_init(struct spi_sh *spidev)
{
	int err;
     	LATTICE_VD_LOG("spidev input init start = %s\n",__func__);
#if 0        
	spidev->input_dev_detect = input_allocate_device();
	if (!spidev->input_dev_detect) {
		err = -ENOMEM;
    		LATTICE_VD_ERR("input allocation failed\n");
		return err;
	}
	
	spidev->input_dev_detect->open = NULL;
	spidev->input_dev_detect->close = NULL;
	spidev->input_dev_detect->name = "voice_event";
	spidev->input_dev_detect->id.bustype = BUS_SPI;
	spidev->input_dev_detect->dev.parent = &spidev->spi_client->dev;
	
	input_set_drvdata(spidev->input_dev_detect, spidev);
	
	set_bit(EV_KEY, spidev->input_dev_detect->evbit);
	set_bit(KEY_POWER, spidev->input_dev_detect->keybit);

	err = input_register_device(spidev->input_dev_detect);
	if (err) {
    		LATTICE_VD_ERR("input register device failed :%s\n",spidev->input_dev_detect->name);
		return err;
	}
#endif
	return 0;
}

static void spidev_input_cleanup(struct spi_sh *spidev)
{
#if 0
	input_unregister_device(spidev->input_dev_detect);
	input_free_device(spidev->input_dev_detect);
#endif    
}

static int spi_sh_set_clock_freq(int freq)
{
    int err;
    struct spi_device * spi_client;
    struct spi_sh *spi_sh = g_pspi_sh;
   spi_client = g_pspi_sh->spi_client;

    spi_par =&spi_conf;
    
    if(!spi_par)
    {
        LATTICE_VD_ERR("spi par is NULL");
        return -1;
    }

    if(10000000 == freq)//10M HZ
    {
        spi_par->setuptime = 15;
        spi_par->holdtime = 15;
        spi_par->high_time = 10;  //   \D3\EBlow_time  \B9\B2ͬ\BE\F6\B6\A8\D7\C5SPI CLK \B5\C4\D6\DC\C6\DA
        spi_par->low_time = 10;
        spi_par->cs_idletime = 20;
    }
    else if(1000000 == freq)// 1M HZ
    {
        spi_par->setuptime = 100;//15;
        spi_par->holdtime = 100;//15;
        spi_par->high_time = 50;//10;  //   \D3\EBlow_time  \B9\B2ͬ\BE\F6\B6\A8\D7\C5SPI CLK \B5\C4\D6\DC\C6\DA
        spi_par->low_time = 50;//10;
        spi_par->cs_idletime = 100;//20;
    }
    
    //lattice add
    //spi_client->mode = (SPI_MODE_0); 

    if(err = spi_setup(spi_client))
    {
        LATTICE_VD_ERR("spi_setup fail");
        return -1;
    }

    return 0;
       
}       
static int hct_spi_sh_probe(struct spi_device *spi_client) {

#if IMPLEMENT_SPIDEV_INTDETECT
	unsigned long		minor;
	int i;
#endif

	int err = 0;

	struct spi_sh *spi_sh = NULL;

	LATTICE_VD_LOG("hct_spi_sh_probe!!!!!!!!!!!\n");

	spi_sh = kzalloc(sizeof(struct spi_sh), GFP_KERNEL);
	if (!spi_sh) {
		dev_dbg(&spi_client->dev, "unable to allocate memory\n");
		err = -ENOMEM;
		goto exit;
	}


	spi_sh->spi_client = spi_client;

	spi_client->dev.platform_data = &spi_sh_pdata;	
	spi_sh->pdata = spi_client->dev.platform_data;				

	printk(KERN_EMERG "Pbobing Sensor Platform driver 4\n");


	if (!spi_sh->pdata) {										
		dev_dbg(&spi_client->dev,								
			"No platform data - aborting\n");					
		err = -EINVAL;											
		goto exit;												
	}															

	INIT_DELAYED_WORK(&input_work_rx, spi_work_func_rx);
	printk(KERN_EMERG "Pbobing Sensor Platform driver 5\n");

	printk(KERN_EMERG "Pbobing Sensor Platform driver 6\n");
	if (spi_sh->pdata->hostInterruptGpio <= 0) {
		dev_err(&spi_client->dev, "no IRQ?\n");
		err = -EINVAL;
		goto exit;
	}
	printk(KERN_ERR "hct_spi_sh_probe  interrupt GPIO %d \n", spi_sh->pdata->hostInterruptGpio);

	spi_sh->activePlatform = spi_sh->pdata->activePlatform;

	spi_sh->exiting = 0;

	spi_sh->suspended = false;

	/* Configure the SPI bus */
	/* not setting SPI_CS_HIGH SPI_NO_CS SPI_LSB_FIRST SPI_3WIRE SPI_READY */
	/* so it is MSB out first, CS active low, not 3 wire mode, no SPI ready support */  
	spi_client->mode = (SPI_MODE_3); 
	spi_client->bits_per_word = 8;



        spi_client->controller_data = (void*)&spi_conf;
        spi_par =&spi_conf;
              
        spi_par->setuptime = 15;//15;
        spi_par->holdtime = 15;//15;
        spi_par->high_time = 10;//10;  //   \D3\EBlow_time  \B9\B2ͬ\BE\F6\B6\A8\D7\C5SPI CLK \B5\C4\D6\DC\C6\DA
        spi_par->low_time = 10;//10;
        spi_par->cs_idletime = 20;//20;
        spi_par->rx_mlsb = 1;       
        spi_par->tx_mlsb = 1;     //mlsb=1  \B1\EDʾ\B8\DFbitλ\CFȴ\AB\A3\ACͨ\B3\A3\B2\BB\D0\E8Ҫ\B8\C4
        spi_par->tx_endian = 0;     //tx_endian  =1 \B1\EDʾ\B4\F3\B6\CBģʽ\A3\AC\B6\D4\D3\DADMAģʽ\A3\AC\D0\E8Ҫ\B8\F9\BE\DD\C9豸\B5\C4Spec \C0\B4ѡ\D4\F1\A3\ACͨ\B3\A3Ϊ\B4\F3\B6ˡ\A3
        spi_par->rx_endian = 0;
        spi_par->cpol = 0;      //  \D5\E2\C0ﲻ\D0\E8Ҫ\D4\D9\C9\E8\D6\C3 \A3\AC  ice40_spi_client->mode = SPI_MODE_0 \D5\E2\C0\EF\C9\E8\D6ü\B4\BF\C9
        spi_par->cpha = 0;     //   \D5\E2\C0ﲻ\D0\E8Ҫ\D4\D9\C9\E8\D6\C3 \A3\AC  ice40_spi_client->mode = SPI_MODE_0 \D5\E2\C0\EF\C9\E8\D6ü\B4\BF\C9
        spi_par->com_mod = DMA_TRANSFER;     // DMA  or FIFO
        //spi_par->com_mod = FIFO_TRANSFER;
        spi_par->pause = 0;     //\D3\EB deassert  \B5\C4\D2\E2˼\CF෴\A1\A3\BC\B4\CAǷ\F1֧\B3\D6\D4\DDͣģʽ\A3\AC\C8\E7\B4\CB\D7\F6SPI_CS \D4ڶ\E0\B4\CEtransfer֮\BC\E4 \B2\BB\BBᱻde-active
        spi_par->finish_intr = 1;  
        //spi_par->deassert = 0;
        spi_par->ulthigh = 0;
        spi_par->tckdly = 0;
    
	spi_setup(spi_client);

	printk(KERN_EMERG "Pbobing Sensor Platform driver 10\n");

	spi_set_drvdata(spi_client, spi_sh);


	err = spidev_input_init(spi_sh);   
	if (err < 0) {
    		LATTICE_VD_ERR("spidev input init failed \n");
		return err;
	}

	mutex_init(&spi_sh->mutex);
    
        printk(KERN_EMERG "Pbobing Sensor Platform driver 13\n");

#if 0
        err = sysfs_create_group(&spi_client->dev.kobj, &spi_sh_group_attr);
        if (err)
            goto err_remove_files;
#else
        err = spi_sh_create_attr(&hct_spi_sh_driver.driver);

        if (err)
            goto err_remove_files;
#endif

       spi_set_vd_gpio_wrok_mode(0);

#if 0	
    err = configure_iCE(spi_sh);
	if (err) {
		dev_err(&spi_client->dev, "unable to configure_iCE\n");
		goto err_remove_files;
	}
#endif

#if IMPLEMENT_SPIDEV_INTDETECT
	spin_lock_init(&spi_sh->spi_lock);
	mutex_init(&spi_sh->buf_lock);

	INIT_LIST_HEAD(&spi_sh->device_entry);
	INIT_LIST_HEAD(&spi_sh->ir_device_entry);

	BUILD_BUG_ON(N_SPI_MINORS > 256);

	printk(KERN_EMERG "Pbobing Sensor Platform driver 14\n");
	err = register_chrdev(SPIDEV_MAJOR, "spi", &spidev_fops);
	if (err < 0) {
	    dev_err(&spi_client->dev, "register_chrdev spi failed!\n");
		goto err_remove_files;
	}
	err = 0;
	printk(KERN_EMERG "Pbobing Sensor Platform driver 15\n");
	spidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(spidev_class)) {
	    dev_err(&spi_client->dev, "class_create spidev failed!\n");

		err = PTR_ERR(spidev_class);
		goto err_remove_spidev_chrdev;
	}
	err = 0;
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

        printk(KERN_EMERG "Pbobing Sensor Platform driver 16\n");
    		spi_sh->devt = MKDEV(SPIDEV_MAJOR, minor);
            
        printk(KERN_EMERG "Pbobing Sensor Platform driver 17\n");
        
        dev = device_create(spidev_class, &spi_client->dev, spi_sh->devt,
            spi_sh, "spidev%d.%d",	
            spi_client->master->bus_num, spi_client->chip_select);
        
        printk(KERN_EMERG "Pbobing Sensor Platform driver 18\n");		
		if (IS_ERR(dev)) {
		    err = PTR_ERR(dev);
		    dev_err(&spi_client->dev, "device_create failed!\n");
		    goto err_remove_class;
		}
	} else {
		dev_err(&spi_client->dev, "no minor number available!\n");
		err = -ENODEV;
		goto err_remove_class;
	}

//	err = spi_sh_enable_interrupts(spi_sh);
    set_bit(minor, minors);
    list_add(&spi_sh->device_entry, &device_list);
    list_add(&spi_sh->ir_device_entry, &ir_device_list);

    mutex_unlock(&device_list_lock);

    file_test1 = debugfs_create_file("INTR_detect", 0666, NULL, NULL, &intr_detect_fops);						
    if(file_test1 == NULL) 
    {
	    dev_err(&spi_client->dev, "debugfs_create_file IR_detect failed!\n");
		err = -EIO;	
		goto err_remove_spidev_list;
    }
    
    i = register_chrdev (SENSHUB_MAJOR, SENSHUB_NAME, &intr_detect_fops);		
    if (i != 0) {
        err = -EIO;
	    dev_err(&spi_client->dev, "register_chrdev IR_detect failed!\n");
   		goto err_remove_intdetect_file;
    }
#endif

    if(!(sh_vd_image_buf = kzalloc(SH_VD_IMAGE_BUFSIZE, GFP_KERNEL)))
    {
        LATTICE_VD_ERR("kzalloc fail");
        err = -ENOMEM;
        goto err_kalloc_failed;
    }
    
    //lattice add
    if(!(g_send_buf = kzalloc(1024, GFP_KERNEL)))
    {
        LATTICE_VD_ERR("kzalloc fail");
        err = -ENOMEM;
        goto err_kalloc_failed;
    }
    
    if(!(g_reciver_bufer = kzalloc(1024, GFP_KERNEL)))
    {
        LATTICE_VD_ERR("kzalloc fail");
        err = -ENOMEM;
        goto err_kalloc_failed;
    }

    rtc_gpio_enable_32k(RTC_GPIO_USER_PMIC);
    
    g_pspi_sh = spi_sh;

    dev_err(&spi_client->dev, "system configuration done\n");


    goto exit;

#if IMPLEMENT_SPIDEV_INTDETECT

#if 0
err_remove_intdetect_chrdev:
	unregister_chrdev(SPIDEV_MAJOR, SENSHUB_NAME);
#endif

err_remove_intdetect_file:
    debugfs_remove(file_test1);

err_remove_spidev_list:
	list_del(&spi_sh->device_entry);
	list_del(&spi_sh->ir_device_entry);
	device_destroy(spidev_class, spi_sh->devt);
	
err_remove_class:
	class_destroy(spidev_class);

err_remove_spidev_chrdev:
	unregister_chrdev(SPIDEV_MAJOR, SENSHUB_NAME);
#endif

err_remove_files:
   spi_sh_delete_attr(&hct_spi_sh_driver.driver); 
err_kalloc_failed:
    
exit:
	return err;
}


static int hct_spi_sh_remove(
	struct spi_device *spi_client) {

	struct spi_sh *spi_sh = spi_get_drvdata(spi_client);


	spi_sh_disable_interrupts(spi_sh);
		
//	sysfs_remove_group(&spi_client->dev.kobj, &spi_sh_group_attr); 
       spi_sh_delete_attr(&hct_spi_sh_driver.driver);


#if IMPLEMENT_SPIDEV_INTDETECT
        unregister_chrdev(SPIDEV_MAJOR, SENSHUB_NAME);
        debugfs_remove(file_test1);

        list_del(&spi_sh->device_entry);
        list_del(&spi_sh->ir_device_entry);
        device_destroy(spidev_class, spi_sh->devt);
        clear_bit(MINOR(spi_sh->devt), minors);

        class_destroy(spidev_class);

        unregister_chrdev(SPIDEV_MAJOR, SENSHUB_NAME);
        spidev_input_cleanup(spi_sh);          /* remove event entry from /dev/input */

#endif

	spi_set_drvdata(spi_client, NULL);

#if IMPLEMENT_SPIDEV_INTDETECT
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	
	if (spi_sh->users == 0)
		kfree(spi_sh);
	mutex_unlock(&device_list_lock);
	
#else	
	kfree(spi_sh);
#endif
//lattice add
        kfree(g_send_buf);
        kfree(g_reciver_bufer);

	return 0;
}


struct spi_device_id sh_spi_id_table = {"spi-ice40", 0};

static struct spi_driver hct_spi_sh_driver = {
	.driver = {
		.name = "spi-ice40",
		.owner = THIS_MODULE,
	},
	.probe = hct_spi_sh_probe,
	.remove= hct_spi_sh_remove,
//	.id_table = &sh_spi_id_table,
};
static struct spi_board_info sh_spi_board_devs[] __initdata = {
	[0] = {        	
            .modalias="spi-ice40",
            .bus_num = 0,
            .chip_select=0,
            .max_speed_hz		= 15000000,
            .mode = SPI_MODE_3,
	},
};

static int __init sh_spi_dev_init(void)
{
        int err=-1;
	err=spi_register_board_info(sh_spi_board_devs, ARRAY_SIZE(sh_spi_board_devs));   
	LATTICE_VD_LOG("SPI_DEV_INIT,add_err= %x\n",err);
	return spi_register_driver(&hct_spi_sh_driver);
}

static void __exit sh_spi_dev_exit(void)
{
	LATTICE_VD_LOG("SPI_DEV_EXIT.\n");
	spi_unregister_driver(&hct_spi_sh_driver);
	
	return;
}

module_init(sh_spi_dev_init);
module_exit(sh_spi_dev_exit);


MODULE_DESCRIPTION("Sensor Platforms Sensor Hub Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Shmuel Ungerfeld <sungerfeld@sensorplatforms.com>");
