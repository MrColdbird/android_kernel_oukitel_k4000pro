/*! @file vfsSpiDrv.h
*******************************************************************************
**  SPI Driver Interface Functions
**
**  This file contains the SPI driver interface functions.
**
**  Copyright 2011-2012 Validity Sensors, Inc. All Rights Reserved.
*/

#ifndef VFSSPIDRV_H_
#define VFSSPIDRV_H_

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <asm-generic/uaccess.h>
#include <linux/irq.h>

#include <asm-generic/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/jiffies.h>

#if 1//PLATFOM_MTK
#include <mach/mt_gpio.h>
#include <mach/mt_spi.h>
#include <mach/eint.h>
#include <cust_eint.h>
//#include <mach/pmic_mt6320_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pm_ldo.h>
//#include <linux/hw_module_info.h>
#include <linux/earlysuspend.h>
#endif
//#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include "cust_gpio_usage.h" 

#define DEBUG 1
#if  DEBUG
//#define DPRINTK(fmt, args...) printk(KERN_DEBUG "vfsspi:"fmt, ## args)
#define DPRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__, ## args)
#else
#define DPRINTK(fmt, args...)
#endif
#if 1//PLATFORM_MTK
#define MAX_READ_FRAME_SIZE 1024
#define PLATFORM_BIG_ENDIAN 1
#endif
/* Major number of device ID.
 * A device ID consists of two parts: a major number, identifying the class of
 * the device, and a minor ID, identifying a specific instance of a device in
 * that class. A device ID is represented using the type dev_t. The minor number
 * of the Validity device is 0. */
#define VFSSPI_MAJOR         (221)

/* Maximum transfer size */
#define DEFAULT_BUFFER_SIZE  (4096 * 5)

#if 0//VCS_FEATURE_SENSOR_WINDSOR
#define DRDY_ACTIVE_STATUS      1
#define BITS_PER_WORD           8
#define DRDY_IRQ_FLAG           IRQF_TRIGGER_RISING
/* Maximum transfer size */
#define DEFAULT_BUFFER_SIZE  (4096 * 6)

#else /* VCS_FEATURE_SENSOR_WINDSOR */
#define DRDY_ACTIVE_STATUS      0
#define BITS_PER_WORD           16
#define DRDY_IRQ_FLAG           IRQF_TRIGGER_FALLING
/* Maximum transfer size */
#define DEFAULT_BUFFER_SIZE  (4096 * 5)
#endif /* VCS_FEATURE_SENSOR_WINDSOR */

/* Timeout value for polling DRDY signal assertion */
#define DRDY_TIMEOUT_MS      40

/* Magic number of IOCTL command */
#define VFSSPI_IOCTL_MAGIC    'k'

/*
 * Definitions of structures which are used by IOCTL commands
 */

/* Pass to VFSSPI_IOCTL_SET_USER_DATA and VFSSPI_IOCTL_GET_USER_DATA commands */
typedef struct vfsspi_iocUserData {
	void *buffer;
	unsigned int len;
}vfsspi_iocUserData_t;

/**
 * vfsspi_iocTransfer - structure to pass to VFSSPI_IOCTL_RW_SPI_MESSAGE command
 * @rxBuffer:pointer to retrieved data
 * @txBuffer:pointer to transmitted data
 * @len:transmitted/retrieved data size
 */
typedef struct vfsspi_iocTransfer {
	unsigned char *rxBuffer;
	unsigned char *txBuffer;
	unsigned int len;
} vfsspi_iocTransfer_t;

/* Pass to VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL command */
/**
 * vfsspi_iocRegSignal - structure to pass to VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL
 *			command
 * @userPID:Process ID to which SPI driver sends signal indicating that DRDY
 *			is asserted
 * @signalID:signalID
*/
typedef struct vfsspi_iocRegSignal {
	int userPID;
	int signalID;
} vfsspi_iocRegSignal_t;

/* Pass to VFSSPI_IOCTL_GET_FREQ_TABLE command */
/**
* vfsspi_iocFreqTable - structure to get supported SPI baud rates
*
* @table:table which contains supported SPI baud rates
* @tblSize:table size
*/
typedef struct vfsspi_iocFreqTable {
    unsigned int *table;
    unsigned int  tblSize;
} vfsspi_iocFreqTable_t;

/**
 * IOCTL commands definitions
 */

/* Transmit data to the device and retrieve data from it simultaneously */
#define VFSSPI_IOCTL_RW_SPI_MESSAGE         _IOWR(VFSSPI_IOCTL_MAGIC,   \
							1, unsigned int)

/* Hard reset the device */
#define VFSSPI_IOCTL_DEVICE_RESET           _IO(VFSSPI_IOCTL_MAGIC,   2)

/* Set the baud rate of SPI master clock */
#define VFSSPI_IOCTL_SET_CLK                _IOW(VFSSPI_IOCTL_MAGIC,    \
							3, unsigned int)

/* Get level state of DRDY GPIO */
#define VFSSPI_IOCTL_CHECK_DRDY             _IO(VFSSPI_IOCTL_MAGIC,   4)

/* Register DRDY signal. It is used by SPI driver for indicating host
 * that DRDY signal is asserted. */
#define VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL   _IOW(VFSSPI_IOCTL_MAGIC,    \
							5, unsigned int)

/* Store the user data into the SPI driver. Currently user data is a
 * device info data, which is obtained from announce packet. */
#define VFSSPI_IOCTL_SET_USER_DATA          _IOW(VFSSPI_IOCTL_MAGIC,    \
							6, unsigned int)

/* Retrieve user data from the SPI driver*/
#define VFSSPI_IOCTL_GET_USER_DATA          _IOWR(VFSSPI_IOCTL_MAGIC,   \
							7, unsigned int)

/* Enable/disable DRDY interrupt handling in the SPI driver */
#define VFSSPI_IOCTL_SET_DRDY_INT           _IOW(VFSSPI_IOCTL_MAGIC,    \
							8, unsigned int)

/* Put device in Low power mode */
#define VFSSPI_IOCTL_DEVICE_SUSPEND         _IO(VFSSPI_IOCTL_MAGIC,   9)

/* Indicate the fingerprint buffer size for read */
#define VFSSPI_IOCTL_STREAM_READ_START      _IOW(VFSSPI_IOCTL_MAGIC,	\
							10, unsigned int)
/* Indicate that fingerprint acquisition is completed */
#define VFSSPI_IOCTL_STREAM_READ_STOP       _IO(VFSSPI_IOCTL_MAGIC,   11)

/* Retrieve supported SPI baud rate table */
#define VFSSPI_IOCTL_GET_FREQ_TABLE         _IOWR(VFSSPI_IOCTL_MAGIC,	\
							12, unsigned int)

/* Turn on the power to the sensor */
#define VFSSPI_IOCTL_POWER_ON                _IO(VFSSPI_IOCTL_MAGIC,   13)

/* Turn off the power to the sensor */
#define VFSSPI_IOCTL_POWER_OFF               _IO(VFSSPI_IOCTL_MAGIC,   14)

/* Disable SPI core clock */
#define VFSSPI_IOCTL_DISABLE_SPI_CLOCK 	     _IO(VFSSPI_IOCTL_MAGIC,   15)

/* Initialize and enable the SPI core (configure with last set (or
 * defaults, if not set, gpios, clks, etc.) */
#define VFSSPI_IOCTL_SET_SPI_CONFIGURATION   _IO(VFSSPI_IOCTL_MAGIC,   16)

/* Uninitialize and disable the SPI core */
#define VFSSPI_IOCTL_RESET_SPI_CONFIGURATION _IO(VFSSPI_IOCTL_MAGIC,   17)

/* Retrieive sensor mount orientation:
 * 0 - right side up (swipe primary first);
 * 1 - upside down (swipe secondary first) */
#define VFSSPI_IOCTL_GET_SENSOR_ORIENTATION  _IOR(VFSSPI_IOCTL_MAGIC,	\
							 18, unsigned int)
							 
/*
 * Definitions of structures which are used by IOCTL commands
 */

/* The spi driver private structure. */
/**
 * vfsspi_devData - The spi driver private structure
 * @devt:Device ID
 * @vfsSpiLock:The lock for the spi device
 * @spi:The spi device
 * @deviceEntry:Device entry list
 * @bufferMutex:The lock for the transfer buffer
 * @isOpened:Indicates that driver is opened
 * @buffer:buffer for transmitting data
 * @nullBuffer:buffer for transmitting zeros
 * @streamBuffer:buffer for transmitting data stream
 * @streamBufSize:streaming buffer size
 * @freqTable:table which contains supported SPI frequencies.
 * @freqTableSize:table size
 * @userInfoData:Storing user info data
	(device info obtained from announce packet)
 * @drdyPin:DRDY GPIO pin number
 * @sleepPin:Sleep GPIO pin number
 * @userPID:User process ID, to which the kernel signal
	indicating DRDY event is to be sent
 * @signalID:Signal ID which kernel uses to indicating
	user mode driver that DRDY is asserted
 * @curSpiSpeed:Current baud rate of SPI master clock
 * @isDrdyIrqEnabled:Indicates that DRDY irq is enabled
*/
struct vfsspi_devData {
	dev_t devt;
	spinlock_t vfsSpiLock;
	struct spi_device *spi;
	struct list_head deviceEntry;
	struct mutex bufferMutex;
	struct regulator *vdd;
        struct regulator *vcc_io;
	unsigned int isOpened;
	unsigned char *buffer;
	unsigned char *nullBuffer;
	unsigned char *streamBuffer;
	unsigned int *freqTable;
	unsigned int freqTableSize;
	size_t streamBufSize;
	struct vfsspi_iocUserData userInfoData;
	unsigned int drdyPin;
	unsigned int sleepPin;
	int userPID;
	int signalID;
	unsigned int curSpiSpeed;
        unsigned int isDrdyIrqEnabled;
};
#if 1

#if defined(GPIO_FIGERPRINT_PWR_EN_PIN)
#define FP_3V3_EN 		GPIO_FIGERPRINT_PWR_EN_PIN
#endif

//#define FP_1V8_EN (GPIO88 | 0x80000000)

#define VFSSPI_INT_IRQNO	CUST_EINT_FIGERPRINT_INT_NUM
/* DRDY GPIO pin number */
#define GPIO_FPC_EINT_PIN    	GPIO_FIGERPRINT_INT
/* Sleep GPIO pin number */
#define VFSSPI_SLEEP_PIN	GPIO_FIGERPRINT_RST

#endif

#define DRDY_IRQ_ENABLE 1
#define DRDY_IRQ_DISABLE 0



/* The initial baud rate for communicating with Validity sensor.
 * The initial clock is configured with low speed as sensor can boot
 * with external oscillator clock. */
#if 1//PLATFORM_MTK
#define SLOW_BAUD_RATE  2600000
/* Max baud rate supported by Validity sensor. */
#define MAX_BAUD_RATE   13000000
#else
#define SLOW_BAUD_RATE  4800000
/* Max baud rate supported by Validity sensor. */
#define MAX_BAUD_RATE   15000000
#endif
/* The coefficient which is multiplying with value retrieved from the
 * VFSSPI_IOCTL_SET_CLK IOCTL command for getting the final baud rate. */
#define BAUD_RATE_COEF  1000

#endif              /* VFSSPIDRV_H_ */
