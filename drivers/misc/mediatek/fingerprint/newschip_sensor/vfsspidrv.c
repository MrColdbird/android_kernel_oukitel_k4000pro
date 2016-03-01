/*! @file vfsSpiDrv.c
*******************************************************************************
**  SPI Driver Interface Functions
**
**  This file contains the SPI driver interface functions.
**
**  Copyright 2011 Validity Sensors, Inc. All Rights Reserved.
*/

#include "vfsspidrv.h"
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
//#include <linux/of_gpio.h>
#include <linux/irqchip/mt-eic.h>


static struct spi_board_info spi_boardinfo[] = {
        {
            .modalias  = "validity_fingerprint",
            .max_speed_hz  = 48000000,
            .bus_num  = 0,
            .chip_select  = 0,
            .mode = SPI_MODE_0,
            //.controller_data = &spidev_mcspi_config, 
        },
};


static int vfsspi_open(struct inode *inode, struct file *filp);

static int vfsspi_release(struct inode *inode, struct file *filp);

static int vfsspi_probe(struct spi_device *spi);

static int vfsspi_remove(struct spi_device *spi);

static ssize_t vfsspi_read(struct file *filp, char __user *buf,
					 size_t count, loff_t *fPos);

static ssize_t vfsspi_write(struct file *filp, const char __user *buf,
						 size_t count, loff_t *fPos);

static int vfsspi_xfer(struct vfsspi_devData *vfsSpiDev,
				struct vfsspi_iocTransfer *tr);

static inline ssize_t vfsspi_readSync(struct vfsspi_devData *vfsSpiDev,
					unsigned char *buf, size_t len);

static inline ssize_t vfsspi_writeSync(struct vfsspi_devData *vfsSpiDev,
					unsigned char *buf, size_t len);

static long vfsspi_ioctl(struct file *filp, unsigned int cmd,
						 unsigned long arg);

static int vfsspi_sendDrdySignal(struct vfsspi_devData *vfsSpiDev);

static void vfsspi_hardReset(struct vfsspi_devData *vfsSpiDev);

static void vfsspi_suspend(struct vfsspi_devData *vfsSpiDev);

static inline void shortToLittleEndian(char *buf, size_t len);

static int vfsspi_enableIrq(struct vfsspi_devData *vfsSpiDev);
static int vfsspi_disableIrq(struct vfsspi_devData *vfsSpiDev);


static int  vfsspi_platformInit(struct vfsspi_devData *vfsSpiDev);

static void vfsspi_platformUninit(struct vfsspi_devData *vfsSpiDev);

struct vfsspi_devData *g_vfsSpiDev = NULL;

void vfsspi_irq(void);

#if 1//PLATFORM_MTK
struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 4, //for mt6582, 104000khz/(4+4) = 13000khz
	.low_time = 4,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,
	.tx_mlsb = 1,

	.tx_endian = 0,
	.rx_endian = 0,

	.com_mod = DMA_TRANSFER,//DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};
#endif 
#if 0//PLATFORM_QCM
unsigned int freqTable[] = {
	960000,
	4800000,
	9600000,
	15000000,
	19200000,
	25000000,
	50000000,
};
#endif
#if 1//PLATFORM_MTK
unsigned int freqTable[] = {
	2000000,	//52/2 = 26
	2080000,	//50/2 = 25
	2600000,	//40/2 = 20
	3250000,	//32/2 = 16
	4000000,	//26/2 = 13
	5200000,	//20/2 = 10
	6500000,	//16/2 = 8
	10400000,	//10/2 = 5
	13000000,	//8/2 = 4
	26000000,	//4/2 = 2
	52000000,	//2/2 = 1
};
#endif
#if 0
unsigned int freqTable[] = {
	1100000,
	5400000,
	10800000,
	15060000,
	24000000,
	25600000,
	27000000,
	48000000,
	51200000,
};
#endif
static struct of_device_id vfs_of_match[]  = {
  { .compatible = "vfsspi,fps", },

  {}
};

#ifdef VFSSPI_32BIT
/*
 * Used by IOCTL compat command:
 *         VFSSPI_IOCTL_RW_SPI_MESSAGE
 *
 * @rx_buffer:pointer to retrieved data
 * @tx_buffer:pointer to transmitted data
 * @len:transmitted/retrieved data size
 */
struct vfsspi_compat_ioctl_transfer {
	compat_uptr_t rx_buffer;
	compat_uptr_t tx_buffer;
	unsigned int len;
};
#endif

/* SPI driver info */
struct spi_driver vfsspi_spi = {
	.driver = {
		.name  = "validity_fingerprint",
		.owner = THIS_MODULE,
        //.of_match_table =vfs_of_match,
	},
	.probe  = vfsspi_probe,
	.remove = vfsspi_remove,
   // 	.suspend            = vfsspi_pm_suspend,
   // 	.resume             = vfsspi_pm_resume,
};

/* file operations associated with device */
static const struct file_operations vfsspi_fops = {
	.owner   = THIS_MODULE,
	.write   = vfsspi_write,
	.read    = vfsspi_read,
	.unlocked_ioctl   = vfsspi_ioctl,
	.open    = vfsspi_open,
	.release = vfsspi_release,
};

struct spi_device *gDevSpi;
struct class *vfsSpiDevClass;
int gpio_irq;

static DECLARE_WAIT_QUEUE_HEAD(wq);
static LIST_HEAD(deviceList);
static DEFINE_MUTEX(deviceListMutex);
static DEFINE_MUTEX(kernel_lock);
static int dataToRead;

inline void shortToLittleEndian(char *buf, size_t len)
{
#if PLATFORM_BIG_ENDIAN
	int i = 0;
	int j = 0;
	char LSB, MSB;
	for (i = 0; i < len; i++, j++) {
		
		LSB = buf[i];
		i++;
		MSB = buf[i];
		buf[j] = MSB;
		//DPRINTK("shortToLittleEndian:%d: %x\n", j, MSB);


		j++;
		buf[j] = LSB;
		//DPRINTK("shortToLittleEndian:%d: %x\n", j, LSB);

	}
#else
    /* Empty function */
#endif
	DPRINTK("vfsspi_shortToLittleEndian: %x_%x_%x_%x_%x_%x_%x_%x\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
}   /* shortToLittleEndian */

static int vfsspi_enableIrq(struct vfsspi_devData *vfsSpiDev)
{
	DPRINTK("vfsspi_enableIrq\n");

	if (vfsSpiDev->isDrdyIrqEnabled == DRDY_IRQ_ENABLE) {
		DPRINTK("DRDY irq already enabled\n");
		return -EINVAL;
	}
	#if 1//PLATFORM_MTK
	mt_eint_unmask(gpio_irq);
	#else
	enable_irq(gpio_irq);
	#endif
	vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_ENABLE;

	return 0;
}

static int vfsspi_disableIrq(struct vfsspi_devData *vfsSpiDev)
{
	DPRINTK("vfsspi_disableIrq\n");

	if (vfsSpiDev->isDrdyIrqEnabled == DRDY_IRQ_DISABLE) {
		DPRINTK("DRDY irq already disabled\n");
		return -EINVAL;
	}
	#if 1//PLATFORM_MTK
	mt_eint_mask(gpio_irq);
	#else
	disable_irq_nosync(gpio_irq);
	#endif
	vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_DISABLE;

	return 0;
}

void vfsspi_irq(void)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	vfsSpiDev = g_vfsSpiDev;
	DPRINTK("vfsspi_irq vfsspi_sendDrdySignal\n");
	/* Linux kernel is designed so that when you disable
	   an edge-triggered interrupt, and the edge happens while
	   the interrupt is disabled, the system will re-play the
	   interrupt at enable time.
	   Therefore, we are checking DRDY GPIO pin state to make sure
	   if the interrupt handler has been called actually by DRDY
	   interrupt and it's not a previous interrupt re-play */
#if 1//PLATFORM_MTK
	if (mt_get_gpio_in(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS) {
#else
	if (gpio_get_value(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS) {
#endif
	dataToRead = 1;
	wake_up_interruptible(&wq);
	vfsspi_sendDrdySignal(vfsSpiDev);
	}

	return;
}
#if 1//PLATFORM_MTK
static int vfsspi_setup(u32 br)
{
	struct mt_chip_conf* vfs_conf;
	vfs_conf = g_vfsSpiDev->spi->controller_data;

	u32 div = 104000000/br/2;

	DPRINTK("vfsspi_setup div:%d  br:%d\n", div,br);

	if(div == vfs_conf->high_time)
	{
		DPRINTK("Don't need update clk\n");
		return 0;
	}

	vfs_conf->high_time = div;
	vfs_conf->low_time = div;

	if(spi_setup(g_vfsSpiDev->spi))
	{
		DPRINTK("spi_setup fail\n");
	}
    
	return 0;
}
#endif
static int vfsspi_sendDrdySignal(struct vfsspi_devData *vfsSpiDev)
{
	struct task_struct *t;
	int ret = 0;
	DPRINTK("vfsspi_sendDrdySignal\n");
	if (vfsSpiDev->userPID != 0) {
		rcu_read_lock();
		/* find the task_struct associated with userpid */
		DPRINTK("Searching task with PID=%08x\n", vfsSpiDev->userPID);
		t = pid_task(find_pid_ns(vfsSpiDev->userPID, &init_pid_ns),
								 PIDTYPE_PID);
		if (t == NULL) {
			DPRINTK("No such pid\n");
			rcu_read_unlock();
			return -ENODEV;
		}
		rcu_read_unlock();
		/* notify DRDY signal to user process */
		ret = send_sig_info(vfsSpiDev->signalID,
					 (struct siginfo *)1, t);
		if (ret < 0)
			DPRINTK("Error sending signal\n");
	} else {
		DPRINTK("pid not received yet\n");
	}
	return ret;
}

/* Return no.of bytes written to device. Negative number for errors */
static inline ssize_t vfsspi_writeSync(struct vfsspi_devData *vfsSpiDev,
					unsigned char *buf,size_t len)
{
	int    status = 0;
	struct spi_message m;
	struct spi_transfer t;
	DPRINTK("vfsspi_writeSync\n");

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.rx_buf = vfsSpiDev->nullBuffer;
	t.tx_buf = buf;
	t.len = len;
	t.speed_hz = vfsSpiDev->curSpiSpeed;
#if 1//PLATFORM_MTK
	vfsspi_setup(t.speed_hz);
#endif
	spi_message_add_tail(&t, &m);

	status = spi_sync(vfsSpiDev->spi, &m);

	if (status == 0) {
		status = m.actual_length;
	DPRINTK("vfsspi_writeSync,length=%d\n", m.actual_length);
	#if 0
	{
	  int i;
	  for(i=0;i<len;i++)
	  {
	       printk("vfsSpiDev->buffer[%d]=%0X \n",i,vfsSpiDev->buffer[i]);
	  }
	}
	#endif
	} else {
		DPRINTK("spi_sync fail, status=%d\n", status);
	}

	return status;
}

/* Return no.of bytes read >0. negative integer incase of error. */
static inline ssize_t vfsspi_readSync(struct vfsspi_devData *vfsSpiDev,
		unsigned char *buf, size_t len)
{
	int    status = 0;
	struct spi_message m;
	struct spi_transfer t;
	DPRINTK("vfsspi_readSync\n");

	spi_message_init(&m);
	memset(&t, 0x0, sizeof(t));
	t.tx_buf = vfsSpiDev->nullBuffer;
	t.rx_buf = buf;
	t.len = len;
	t.speed_hz = vfsSpiDev->curSpiSpeed;
#if 1//PLATFORM_MTK
	vfsspi_setup(t.speed_hz);
#endif
	spi_message_add_tail(&t, &m);
	status = spi_sync(vfsSpiDev->spi, &m);

	if (status == 0) {
		#if 1//PLATFORM_MTK
		status = m.actual_length;
		#else
		status = len;
		#endif
		DPRINTK("vfsspi_readSync length=%d\n", status);

		/* FIXME: This is temporary workaround for Fluid,
		instead of returning actual read data length
		spi_sync is returning 0 */
		/* status = len; */
		#if 0
		{
		  int i;
		  printk("vfsspi_readsync status=%d,actual_length =%d\n",status,m.actual_length);
		  for(i=0;i<len;i++)
		  {
		       printk("vfsSpiDev->buffer[%d]=%0X \n",i,vfsSpiDev->buffer[i]);
		  }
		}
		#endif
	} else {
		DPRINTK("spi_sync fail, status=%d\n", status);
	}

	return status;
}

static ssize_t vfsspi_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *fPos)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	ssize_t               status = 0;
	DPRINTK("vfsspi_write\n");

	if (count > DEFAULT_BUFFER_SIZE || count <= 0) {
		DPRINTK("passed incorrect buffer length - %d\n", count);
		return -EMSGSIZE;
	}

	vfsSpiDev = filp->private_data;
	#if 1 //PLATFORM_MTK	
	if(count >1024)
		printk("vfsspi_write transfer is long\n");
	#endif
	mutex_lock(&vfsSpiDev->bufferMutex);
	if (vfsSpiDev->buffer) {
		unsigned long missing = 0;
		missing = copy_from_user(vfsSpiDev->buffer, buf, count);
		shortToLittleEndian((char *)vfsSpiDev->buffer, count);
		if (missing == 0){
			//DPRINTK("vfsspi_write: %x_%x_%x_%x_%x_%x_%x_%x\n", vfsSpiDev->buffer[0], vfsSpiDev->buffer[1], 
				//vfsSpiDev->buffer[2], vfsSpiDev->buffer[3], vfsSpiDev->buffer[4], 
				//vfsSpiDev->buffer[5], vfsSpiDev->buffer[6], vfsSpiDev->buffer[7]);
			status = vfsspi_writeSync(vfsSpiDev, vfsSpiDev->buffer,
									count);
		}
		else
			status = -EFAULT;
	}
	mutex_unlock(&vfsSpiDev->bufferMutex);
	return status;
}

static ssize_t vfsspi_read(struct file *filp, char __user *buf, size_t count,
		loff_t *fPos)
{

#ifdef MAX_READ_FRAME_SIZE
	u32 frames=0;
	u32 rest=0;
	u32 firstRun=0;
	u32 i=0;
#endif
	struct vfsspi_devData *vfsSpiDev = NULL;
	unsigned char * readBuf = NULL;
	ssize_t status = 0;
	DPRINTK("vfsspi_read, %d\n", count);
	vfsSpiDev = filp->private_data;
	if (vfsSpiDev->streamBuffer != NULL && count <= vfsSpiDev->streamBufSize) {
		readBuf = vfsSpiDev->streamBuffer;
	} else if (count <= DEFAULT_BUFFER_SIZE) {
		readBuf = vfsSpiDev->buffer;
	} else {
		DPRINTK("passed incorrect buffer length - %d\n", count);
		return -EMSGSIZE;
	}
	if (buf == NULL) {
		DPRINTK("passed buffer is NULL\n");
		return -EFAULT;
	}

	mutex_lock(&vfsSpiDev->bufferMutex);

#ifdef MAX_READ_FRAME_SIZE 		

#if 1
	if(count >= MAX_READ_FRAME_SIZE)
	{
		frames = count / MAX_READ_FRAME_SIZE;
		rest = count % MAX_READ_FRAME_SIZE;
		firstRun = count - rest;
		DPRINTK("vfsspi_MAX_READ_FRAME_SIZE: firstRun %d\n",firstRun);
		status	= vfsspi_readSync(vfsSpiDev, readBuf, firstRun);
		if(rest)
		{
			DPRINTK("vfsspi_MAX_READ_FRAME_SIZE: rest %d\n",rest);
			status	= vfsspi_readSync(vfsSpiDev, readBuf+firstRun, rest);
		}	
	}
	else if(count>0 && count<MAX_READ_FRAME_SIZE)
	{
		DPRINTK("vfsspi_MAX_READ_FRAME_SIZE: between 0 and MAX count %d\n",count);
		status  = vfsspi_readSync(vfsSpiDev, readBuf, count);
	}
#else	
		#if 0	
		frames = count / MAX_FRAME_SIZE;
		rest = count % MAX_FRAME_SIZE;			
		for(i=0; i<frames; i++)
		{
			status  = vfsspi_readSync(vfsSpiDev, readBuf+i*MAX_FRAME_SIZE, MAX_FRAME_SIZE);			
		}
		
		if(rest)
		{
			status  = vfsspi_readSync(vfsSpiDev, readBuf+frames*MAX_FRAME_SIZE, rest);
		}
		#endif
		
		status = vfsspi_readSync(vfsSpiDev, readBuf, count);
#endif
		status = count;
		if (status > 0) {
			unsigned long missing = 0;
			/* data read. Copy to user buffer.*/
			shortToLittleEndian((char *)readBuf, status);
			missing = copy_to_user(buf, readBuf, status);
			if (missing == status) {
				DPRINTK("copy_to_user failed \n");
				/* Nothing was copied to user space buffer. */
				status = -EFAULT;
			} else {
				status = status - missing;
			}
		}
#else
	status  = vfsspi_readSync(vfsSpiDev, readBuf, count);
	if (status > 0) {
		unsigned long missing = 0;
		/* data read. Copy to user buffer.*/
		shortToLittleEndian((char *)readBuf, status);
		missing = copy_to_user(buf, readBuf, status);
		if (missing == status) {
			DPRINTK("copy_to_user failed\n");
			/* Nothing was copied to user space buffer. */
			status = -EFAULT;
		} else {
			status = status - missing;
		}
	}
#endif
	mutex_unlock(&vfsSpiDev->bufferMutex);
	return status;
}

static int vfsspi_xfer(struct vfsspi_devData *vfsSpiDev,
		struct vfsspi_iocTransfer *tr)
{
	int status = 0;
	struct spi_message m;
	struct spi_transfer t;
	DPRINTK("vfsspi_xfer\n");
	if (vfsSpiDev == NULL || tr == NULL) {
		DPRINTK("passed NULL parameter\n");
		return -EFAULT;
	}

	if (tr->len > DEFAULT_BUFFER_SIZE || tr->len <= 0) {
		DPRINTK("passed incorrect buffer length - %d\n", tr->len);
		return -EMSGSIZE;
	}

	if (tr->txBuffer != NULL) {
		if (copy_from_user(vfsSpiDev->nullBuffer, tr->txBuffer,
							tr->len) != 0)
			return -EFAULT;
		shortToLittleEndian((char *)vfsSpiDev->nullBuffer, tr->len);
		
		//DPRINTK("vfsspi_xfer: %x_%x_%x_%x_%x_%x_%x_%x\n", vfsSpiDev->nullBuffer[0], vfsSpiDev->nullBuffer[1], 
			//vfsSpiDev->nullBuffer[2], vfsSpiDev->nullBuffer[3], vfsSpiDev->nullBuffer[4], 
			//vfsSpiDev->nullBuffer[5], vfsSpiDev->nullBuffer[6], vfsSpiDev->nullBuffer[7]);
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.tx_buf = vfsSpiDev->nullBuffer;
	t.rx_buf = vfsSpiDev->buffer;
	t.len = tr->len;
	#if 1//PLATFORM_MTK
	if(tr->len >1024)
		printk("vfsspi_xfer transfer is long\n");
	#endif
	t.speed_hz = vfsSpiDev->curSpiSpeed;
	#if 1//PLATFORM_MTK
	vfsspi_setup(t.speed_hz);
	#endif
	spi_message_add_tail(&t, &m);
	status = spi_sync(vfsSpiDev->spi, &m);

	if (status == 0) {
		if (tr->rxBuffer != NULL) {
			unsigned missing = 0;
			shortToLittleEndian((char *)vfsSpiDev->buffer, tr->len);
			#if 0
			{
		          int i =0;
		          printk("vfsspi_xfer status=%d,actual_length =%d\n",status,m.actual_length);
		          for(i=0;i<tr->len;i++)
		         {
		             printk("vfsSpiDev->buffer[%d]=%0X \n",i,vfsSpiDev->buffer[i]);
		        }
		     }
			#endif
			missing = copy_to_user(tr->rxBuffer,
						 vfsSpiDev->buffer, tr->len);
			if (missing != 0)
				tr->len = tr->len - missing;
		}
	}
	DPRINTK("vfsspi_xfer,tr->length=%d\n", tr->len);
	return status;
}

long vfsspi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retVal = 0;
	struct vfsspi_devData *vfsSpiDev = NULL;
	DPRINTK("vfsspi_ioctl\n");
	if (_IOC_TYPE(cmd) != VFSSPI_IOCTL_MAGIC) {
		DPRINTK("invalid magic. cmd= 0x%X Received= 0x%X "
				"Expected= 0x%X\n", cmd, _IOC_TYPE(cmd),
						VFSSPI_IOCTL_MAGIC);
		return -ENOTTY;
	}
	vfsSpiDev = filp->private_data;
	mutex_lock(&vfsSpiDev->bufferMutex);
	switch (cmd) {
	case VFSSPI_IOCTL_DEVICE_SUSPEND:
	{
		DPRINTK("VFSSPI_IOCTL_DEVICE_SUSPEND:\n");
		vfsspi_suspend(vfsSpiDev);
		break;
	}
	case VFSSPI_IOCTL_DEVICE_RESET:
	{
		DPRINTK("VFSSPI_IOCTL_DEVICE_RESET:\n");
		vfsspi_hardReset(vfsSpiDev);
		break;
	}
	case VFSSPI_IOCTL_RW_SPI_MESSAGE:
	{
		struct vfsspi_iocTransfer *dup = NULL;
		DPRINTK("VFSSPI_IOCTL_RW_SPI_MESSAGE\n");
		#ifdef VFSSPI_32BIT
    struct vfsspi_compat_ioctl_transfer   dup_compat;
		#endif
		dup = kmalloc(sizeof(struct vfsspi_iocTransfer), GFP_KERNEL);
				if (dup == NULL) {
					DPRINTK("dup is NULL\n");
					retVal = -ENOMEM;
					break;
				}
				#ifdef VFSSPI_32BIT
				if (copy_from_user(&dup_compat, (void __user *)arg,
			   sizeof(struct vfsspi_compat_ioctl_transfer)) != 0) {
				#else
				if (copy_from_user(dup, (void *)arg,
							sizeof(struct vfsspi_iocTransfer)) == 0) {
				#endif
					retVal = vfsspi_xfer(vfsSpiDev, dup);
					if (retVal == 0) {
						if (copy_to_user((void *)arg, dup,
									sizeof(struct vfsspi_iocTransfer)) != 0) {
							DPRINTK("copy to user failed\n");
							retVal = -EFAULT;
						}
					}
				} else {
					DPRINTK("copy from user failed\n");
					retVal = -EFAULT;
				}
				kfree(dup);
				break;
			}
		case VFSSPI_IOCTL_SET_CLK:
			{
				unsigned short clock = 0;
				DPRINTK("VFSSPI_IOCTL_SET_CLK\n");
				if (copy_from_user(&clock, (void *)arg,
							sizeof(unsigned short)) == 0) {

					struct spi_device *spidev = NULL;
					spin_lock_irq(&vfsSpiDev->vfsSpiLock);
					spidev = spi_dev_get(vfsSpiDev->spi);
					spin_unlock_irq(&vfsSpiDev->vfsSpiLock);

					if (spidev != NULL) {
						switch (clock) {
							case 0:
								{
									/* Running baud rate. */
									DPRINTK("Running baud rate.\n");
									spidev->max_speed_hz = MAX_BAUD_RATE;
									vfsSpiDev->curSpiSpeed = MAX_BAUD_RATE;
									break;
								}
							case 0xFFFF:
								{
									/* Slow baud rate */
									DPRINTK("slow baud rate.\n");
									spidev->max_speed_hz = SLOW_BAUD_RATE;
									vfsSpiDev->curSpiSpeed = SLOW_BAUD_RATE;
									break;
								}
							default:
								{
									DPRINTK("baud rate is %d.\n", clock);
									vfsSpiDev->curSpiSpeed =
										clock * BAUD_RATE_COEF;
									if (vfsSpiDev->curSpiSpeed >
											MAX_BAUD_RATE)
										vfsSpiDev->curSpiSpeed =
											MAX_BAUD_RATE;
									spidev->max_speed_hz =
										vfsSpiDev->curSpiSpeed;
									break;
								}
						}
						spi_dev_put(spidev);

						//vfsspi_setup(spidev->max_speed_hz);
					}
				} else {
					DPRINTK("Failed copy from user.\n");
					retVal = -EFAULT;
				}
				break;
			}
		case VFSSPI_IOCTL_CHECK_DRDY:
			{
				DPRINTK("VFSSPI_IOCTL_CHECK_DRDY\n");
				retVal = ETIMEDOUT;
				dataToRead = 0;
				#if 1 //PLATFORM_MTK
				if (mt_get_gpio_in(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS) {
				#else
				if (gpio_get_value(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS) {
				#endif
					DPRINTK("VFSSPI_IOCTL_CHECK_DRDY:OK\n");
					retVal = 0;
				} else {
					unsigned long timeout = msecs_to_jiffies(DRDY_TIMEOUT_MS);
					unsigned long startTime = jiffies;
					unsigned long endTime = 0;

					do {
						wait_event_interruptible_timeout(wq,dataToRead != 0, timeout);
						dataToRead = 0;
						#if 1 //PLATFORM_MTK
						if (mt_get_gpio_in(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS) {
						#else
						if (gpio_get_value(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS) {
						#endif
							retVal = 0;
							break;
						}

						endTime = jiffies;
						if (endTime - startTime >= timeout) {
							/* Timed out happens for waiting
							   to wake up event. */
							timeout = 0;
						} else {
							/* Thread is woke up by spurious event.
							   Calculate a new timeout and continue to
							   wait DRDY signal assertion. */
							timeout -= endTime - startTime;
							startTime = endTime;
						}
					} while (timeout > 0);
				}
				dataToRead = 0;
				break;
			}
		case VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:
			{
				struct vfsspi_iocRegSignal usrSignal;
				DPRINTK("VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL\n");
				if (copy_from_user(&usrSignal, (void *)arg,
							sizeof(usrSignal)) != 0) {
					DPRINTK("copy from user failed.\n");
					retVal = -EFAULT;
				} else {
					vfsSpiDev->userPID = usrSignal.userPID;
					vfsSpiDev->signalID = usrSignal.signalID;
				}
				break;
			}
		case VFSSPI_IOCTL_SET_USER_DATA:
			{
				struct vfsspi_iocUserData tmpUserData;
				DPRINTK("VFSSPI_IOCTL_SET_USER_DATA\n");
				if ((void *)arg == NULL) {
					DPRINTK("passed arg is NULL\n");
					retVal = -EINVAL;
					break;
				}
				if (copy_from_user(&tmpUserData, (void *)arg,
							sizeof(struct vfsspi_iocUserData)) != 0) {
					DPRINTK("tmpUserData: copy from user failed\n");
					retVal = -EFAULT;
					break;
				}
				if (vfsSpiDev->userInfoData.buffer != NULL)
					kfree(vfsSpiDev->userInfoData.buffer);
				vfsSpiDev->userInfoData.buffer =
					kmalloc(tmpUserData.len, GFP_KERNEL);

				if (vfsSpiDev->userInfoData.buffer == NULL) {
					DPRINTK("userInfoData.buffer is NULL\n");
					retVal = -ENOMEM;
					break;
				}

				vfsSpiDev->userInfoData.len = tmpUserData.len;
				if (tmpUserData.buffer != NULL) {
					if (copy_from_user(vfsSpiDev->userInfoData.buffer,
								tmpUserData.buffer, tmpUserData.len) != 0) {
						DPRINTK("copy from user failed\n");
						retVal = -EFAULT;
					}
				} else {
					DPRINTK("tmpUserData.buffer is NULL\n");
					retVal = -EFAULT;
				}
				break;
			}
		case VFSSPI_IOCTL_GET_USER_DATA:
			{
				struct vfsspi_iocUserData tmpUserData;

				DPRINTK("VFSSPI_IOCTL_GET_USER_DATA\n");
				retVal = -EFAULT;

				if (vfsSpiDev->userInfoData.buffer == NULL ||
						(void *)arg == NULL) {
					DPRINTK("Invalid argument is passed\n");
					break;
				}

				if (copy_from_user(&tmpUserData, (void *)arg,
							sizeof(struct vfsspi_iocUserData)) != 0) {
					DPRINTK("tmpUserData: copy from user failed\n");
					break;
				}

				if (tmpUserData.len != vfsSpiDev->userInfoData.len ||
						tmpUserData.buffer == NULL) {
					DPRINTK("userInfoData parameter is incorrect\n");
					break;
				}

				if (copy_to_user(tmpUserData.buffer,
							vfsSpiDev->userInfoData.buffer,
							tmpUserData.len) != 0) {
					DPRINTK("userInfoData: copy to user failed\n");
					break;
				}

				if (copy_to_user((void *)arg, &(tmpUserData),
							sizeof(vfsspi_iocUserData_t)) != 0) {
					DPRINTK("tmpUserData: copy to user failed\n");
					break;
				}

				retVal = 0;
		break;
	}
	case VFSSPI_IOCTL_SET_DRDY_INT:
	{
		unsigned short drdy_enable_flag;
				DPRINTK("VFSSPI_IOCTL_SET_DRDY_INT\n");
		if (copy_from_user(&drdy_enable_flag, (void *)arg,
					 sizeof(drdy_enable_flag)) != 0) {
			DPRINTK("Failed copy from user.\n");
			retVal = -EFAULT;
		} else {
			if (drdy_enable_flag == 0)
				vfsspi_disableIrq(vfsSpiDev);
				//free_irq(gpio_irq, vfsSpiDev);
			else {
				vfsspi_enableIrq(vfsSpiDev);
			
				/* Workaround the issue where the system
				misses DRDY notification to host when
				DRDY pin was asserted before enabling
				device.*/
				#if 1//PLATFORM_MTK
				if (mt_get_gpio_in(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS)
				#else
				if (gpio_get_value(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS)
				#endif
					vfsspi_sendDrdySignal(vfsSpiDev);	
			}
		}
		break;
	}
	case VFSSPI_IOCTL_STREAM_READ_START:
	{
		unsigned int streamDataSize;
				DPRINTK("VFSSPI_IOCTL_STREAM_READ_START\n");
				if (copy_from_user(&streamDataSize, (void *)arg,
							sizeof(unsigned int)) != 0) {
					DPRINTK("Failed copy from user.\n");
					retVal = -EFAULT;
					break;
				}

				if (vfsSpiDev->streamBuffer != NULL)
					kfree(vfsSpiDev->streamBuffer);

				vfsSpiDev->streamBuffer = kmalloc(streamDataSize, GFP_KERNEL);
				if (vfsSpiDev->streamBuffer == NULL) {
					DPRINTK("Failed to allocate stream buffer.\n");
					retVal = -ENOMEM;
				} else {
					vfsSpiDev->streamBufSize = streamDataSize;
				}
				break;
			}
		case VFSSPI_IOCTL_STREAM_READ_STOP:
			{
				DPRINTK("VFSSPI_IOCTL_STREAM_READ_STOP\n");
				if (vfsSpiDev->streamBuffer != NULL) {
					kfree(vfsSpiDev->streamBuffer);
					vfsSpiDev->streamBuffer = NULL;
					vfsSpiDev->streamBufSize = 0;
				}
				break;
			}
		case VFSSPI_IOCTL_GET_FREQ_TABLE:
			{
				vfsspi_iocFreqTable_t tmpFreqData;

				DPRINTK("VFSSPI_IOCTL_GET_FREQ_TABLE\n");

				retVal = -EINVAL;
				if (copy_from_user(&tmpFreqData, (void *)arg,
							sizeof(vfsspi_iocFreqTable_t)) != 0) {
					DPRINTK("Failed copy from user.\n");
					break;
				}

				tmpFreqData.tblSize = 0;
				if (vfsSpiDev->freqTable != NULL) {
					tmpFreqData.tblSize = vfsSpiDev->freqTableSize;
					if (tmpFreqData.table != NULL) {
						if (copy_to_user(tmpFreqData.table,
									vfsSpiDev->freqTable,
									vfsSpiDev->freqTableSize) != 0) {
							DPRINTK("Failed copy to user.\n");
							break;
						}
					}
				}
				if (copy_to_user((void *)arg, &(tmpFreqData),
							sizeof(vfsspi_iocFreqTable_t)) == 0) {
					retVal = 0;
				} else {
					DPRINTK("tmpFreqData: copy to user failed\n");
				}
				break;
			}
		default:
			DPRINTK("vfsspi_ioctl default %d\n",cmd);
			retVal = -EFAULT;
			break;

	}
	mutex_unlock(&vfsSpiDev->bufferMutex);

	DPRINTK("exit vfsspi_ioctl\n");

	return retVal;
}

void vfsspi_hardReset(struct vfsspi_devData *vfsSpiDev)
{
	DPRINTK("vfsspi_hardReset\n");
	if (vfsSpiDev != NULL) {

#if 0//VCS_FEATURE_SENSOR_WINDSOR
		#if 0//PLATFORM_MTK	
		spin_lock(&vfsSpiDev->vfsSpiLock);
		dataToRead = 0;
		mt_set_gpio_out(vfsSpiDev->sleepPin, 0);
		mdelay(10);
		mt_set_gpio_out(vfsSpiDev->sleepPin, 1);
		spin_unlock(&vfsSpiDev->vfsSpiLock);
		mdelay(50);
		#else
		spin_lock(&vfsSpiDev->vfsSpiLock);
		dataToRead = 0;
		gpio_set_value(vfsSpiDev->sleepPin, 0);
		mdelay(1);
		gpio_set_value(vfsSpiDev->sleepPin, 1);
		spin_unlock(&vfsSpiDev->vfsSpiLock);
		mdelay(5);
		#endif

#else
		#if 1//PLATFORM_MTK
		spin_lock(&vfsSpiDev->vfsSpiLock);
		dataToRead = 0;
		mt_set_gpio_out(vfsSpiDev->sleepPin, 1);
		mdelay(10);
		mt_set_gpio_out(vfsSpiDev->sleepPin, 0);
		spin_unlock(&vfsSpiDev->vfsSpiLock);
		mdelay(100);
		#else
		spin_lock(&vfsSpiDev->vfsSpiLock);
		dataToRead = 0;
		gpio_set_value(vfsSpiDev->sleepPin, 1);
		mdelay(1);
		gpio_set_value(vfsSpiDev->sleepPin, 0);
		spin_unlock(&vfsSpiDev->vfsSpiLock);
		mdelay(5);
		#endif		
#endif		
	}
}

void vfsspi_suspend(struct vfsspi_devData *vfsSpiDev)
{
	DPRINTK("vfsspi_suspend\n");
	if (vfsSpiDev != NULL) {
#if 0//VCS_FEATURE_SENSOR_WINDSOR
		spin_lock(&vfsSpiDev->vfsSpiLock);
		dataToRead = 0;
		#if 0 //PLATFORM_MTK
		mt_set_gpio_out(vfsSpiDev->sleepPin, 0);
		#else
		gpio_set_value(vfsSpiDev->sleepPin, 0);
		#endif
		spin_unlock(&vfsSpiDev->vfsSpiLock);
#else
		spin_lock(&vfsSpiDev->vfsSpiLock);
		dataToRead = 0;
		#if 1 //PLATFORM_MTK
		mt_set_gpio_out(vfsSpiDev->sleepPin, 1);
		#else
		gpio_set_value(vfsSpiDev->sleepPin, 1);
		#endif
		spin_unlock(&vfsSpiDev->vfsSpiLock);
#endif
	}
}

static int vfsspi_open(struct inode *inode, struct file *filp)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	int status = -ENXIO;
	DPRINTK("vfsspi_open\n");
	mutex_lock(&kernel_lock);
	mutex_lock(&deviceListMutex);
	list_for_each_entry(vfsSpiDev, &deviceList, deviceEntry) {
		if (vfsSpiDev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (vfsSpiDev->isOpened != 0) {
			status = -EBUSY;
		} else {
			vfsSpiDev->userPID = 0;
			if (vfsSpiDev->buffer == NULL) {
				vfsSpiDev->nullBuffer =
					kmalloc(DEFAULT_BUFFER_SIZE,
							GFP_KERNEL);
				vfsSpiDev->buffer =
					kmalloc(DEFAULT_BUFFER_SIZE,
							GFP_KERNEL);

				if (vfsSpiDev->buffer == NULL ||
					vfsSpiDev->nullBuffer == NULL) {
					DPRINTK("Failed to allocate buffer\n");
					status = -ENOMEM;
				} else {
					vfsSpiDev->isOpened = 1;
					filp->private_data = vfsSpiDev;
					nonseekable_open(inode, filp);
				}
			}
		}
	}
	mutex_unlock(&deviceListMutex);
	mutex_unlock(&kernel_lock);
	return status;
}

int vfsspi_release(struct inode *inode, struct file *filp)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	int                   status     = 0;
	DPRINTK("vfsspi_release\n");
	mutex_lock(&deviceListMutex);
	vfsSpiDev = filp->private_data;
	filp->private_data = NULL;
	vfsSpiDev->isOpened = 0;
	if (vfsSpiDev->buffer != NULL) {
		kfree(vfsSpiDev->buffer);
		vfsSpiDev->buffer = NULL;
	}
	if (vfsSpiDev->nullBuffer != NULL) {
		kfree(vfsSpiDev->nullBuffer);
		vfsSpiDev->nullBuffer = NULL;
	}
	if (vfsSpiDev->streamBuffer != NULL) {
		kfree(vfsSpiDev->streamBuffer);
		vfsSpiDev->streamBuffer = NULL;
		vfsSpiDev->streamBufSize = 0;
	}
	mutex_unlock(&deviceListMutex);
	return status;
}

static int vfsspi_check_sensor_alive(struct vfsspi_devData *vfsSpiDev)
{

        int ret = 0;
        int status = 0;

	char tx_buf[64] = {0};
	char rx_buf[64] = {0};

	struct spi_device *spi = vfsSpiDev->spi;

	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = SLOW_BAUD_RATE,
		.tx_buf = tx_buf,           //vfsSpiDev->nullBuffer,
		.rx_buf = rx_buf,
		.len = 64,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 16,
	};

	struct spi_message m;
	int i = 0;
	DPRINTK("ValiditySensor : vfsspi_check_sensor_alive\n");

	vfsspi_hardReset(vfsSpiDev);
        DPRINTK("vfsspi_hardReset succeed\n");

	mdelay(5);

	spi->bits_per_word = BITS_PER_WORD;
	spi->max_speed_hz = SLOW_BAUD_RATE;
	spi->mode = SPI_MODE_0;

	status = spi_setup(spi);
         DPRINTK("spi_setup:status = %d\n",status);  

           
	spi_message_init(&m);
        DPRINTK("spi_message_init\n");
	spi_message_add_tail(&t, &m);
        DPRINTK("spi_message_add_tail\n");

	printk(KERN_ERR "ValiditySensor : spi_sync returned %d \n", spi_sync(spi, &m));

	// First four bytes dumped should be announce packet - fe dc ba 98
  	if(rx_buf[0] == 0x98 && rx_buf[1] == 0xba && rx_buf[2]== 0xdc && rx_buf[3] == 0xfe){
		DPRINTK("vfsspi check sensor successful\n");
	}else{
		DPRINTK( "vfsspi check sensor faild %x %x %x %x\n",rx_buf[0], rx_buf[1],rx_buf[2],rx_buf[3]);
	}

        DPRINTK( "vfsspi check sensor %x %x %x %x\n",rx_buf[0], rx_buf[1],rx_buf[2],rx_buf[3]);	
	return 0;
}

#if 0
static int vfsspi_parse_dt(struct device *dev,
	struct vfsspi_devData *data)
{
	struct device_node *np = dev->of_node;
	enum of_gpio_flags flags;
	int errorno = 0;
	int gpio;

	gpio = of_get_named_gpio_flags(np, "vfsspi-sleepPin",
		0, &flags);
	if (gpio < 0) {
		errorno = gpio;
		goto dt_exit;
	} else {
		data->sleepPin = gpio;
		pr_info("%s: sleepPin=%d\n",
			__func__, data->sleepPin);
	}
	gpio = of_get_named_gpio_flags(np, "vfsspi-drdyPin",
		0, &flags);
	if (gpio < 0) {
		gpio = errorno;
		goto dt_exit;
	} else {
		data->drdyPin = gpio;
		pr_info("%s: drdyPin=%d\n",
			__func__, data->drdyPin);
	}

dt_exit:
	return errorno;
}
#endif

int vfsspi_probe(struct spi_device *spi)
{
	int status = 0;
	int err = 0;
	struct vfsspi_devData *vfsSpiDev = NULL;
	struct device *dev = NULL;
	DPRINTK("vfsspi_probe\n");
#if defined(FP_1V8_EN)
    	mt_set_gpio_pull_enable(FP_1V8_EN, GPIO_PULL_DISABLE);
    	mt_set_gpio_mode(FP_1V8_EN, GPIO_MODE_GPIO);
    	mt_set_gpio_dir(FP_1V8_EN,GPIO_DIR_OUT);
    	mt_set_gpio_out(FP_1V8_EN,GPIO_OUT_ONE);
#endif
    	mt_set_gpio_pull_enable(FP_3V3_EN, GPIO_PULL_DISABLE);
    	mt_set_gpio_mode(FP_3V3_EN, GPIO_MODE_GPIO);
    	mt_set_gpio_dir(FP_3V3_EN,GPIO_DIR_OUT);
    	mt_set_gpio_out(FP_3V3_EN,GPIO_OUT_ONE);
    
	vfsSpiDev = kzalloc(sizeof(*vfsSpiDev), GFP_KERNEL);

	if (vfsSpiDev == NULL) {
		DPRINTK("Failed to allocate buffer\n");
		return -ENOMEM;
	}
  	#if 0
	if (spi->dev.of_node) {
		status = vfsspi_parse_dt(&spi->dev, vfsSpiDev);
		if (status) {
			pr_err("%s - Failed to parse DT\n", __func__);
			kfree(vfsSpiDev);
		}
	}
	#endif
	
	/* Initialize driver data. */
	vfsSpiDev->curSpiSpeed = SLOW_BAUD_RATE;
	vfsSpiDev->userInfoData.buffer = NULL;
	vfsSpiDev->spi = spi;

	#if 1 //PLATFORM_MTK
	vfsSpiDev->spi->controller_data = (void*)&spi_conf_mt65xx;
	#endif
	
	g_vfsSpiDev = vfsSpiDev;
	
	spin_lock_init(&vfsSpiDev->vfsSpiLock);
	mutex_init(&vfsSpiDev->bufferMutex);
	INIT_LIST_HEAD(&vfsSpiDev->deviceEntry);
	status = vfsspi_platformInit(vfsSpiDev);
    	#if 1//TEST_SENSOR
    	vfsspi_check_sensor_alive(vfsSpiDev);
    	#endif
	if (status == 0) {
	        DPRINTK("vfsspi_probe\n");
		spi->bits_per_word = BITS_PER_WORD;
		spi->max_speed_hz = SLOW_BAUD_RATE;
		spi->mode = SPI_MODE_0;
		status = spi_setup(spi);
		if (status == 0) {
	                DPRINTK("spi_setup succeed\n");
			mutex_lock(&deviceListMutex);
			/* Create device node */
			vfsSpiDev->devt = MKDEV(VFSSPI_MAJOR, 0);
			dev = device_create(vfsSpiDevClass, &spi->dev,
				 vfsSpiDev->devt, vfsSpiDev, "vfsspi");
			status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
			if (status == 0)
				list_add(&vfsSpiDev->deviceEntry, &deviceList);
			mutex_unlock(&deviceListMutex);
			if (status == 0)
				spi_set_drvdata(spi, vfsSpiDev);
			else {
				DPRINTK("device_create failed with "
						"status= %d\n", status);
				kfree(vfsSpiDev);
			}
	        
		} else {
			gDevSpi = spi;
			DPRINTK("spi_setup() is failed! status= %d\n", status);
			vfsspi_platformUninit(vfsSpiDev);
			kfree(vfsSpiDev);
		}
	} else {
		vfsspi_platformUninit(vfsSpiDev);
		kfree(vfsSpiDev);
	}

	DPRINTK("vfsspi_probe succeed\n");
	return status;
}

int vfsspi_remove(struct spi_device *spi)
{
	int status = 0;
	struct vfsspi_devData *vfsSpiDev = NULL;
	DPRINTK("vfsspi_remove\n");
	vfsSpiDev = spi_get_drvdata(spi);
	if (vfsSpiDev != NULL) {
		gDevSpi = spi;
		spin_lock_irq(&vfsSpiDev->vfsSpiLock);
		vfsSpiDev->spi = NULL;
		spi_set_drvdata(spi, NULL);
		spin_unlock_irq(&vfsSpiDev->vfsSpiLock);
		mutex_lock(&deviceListMutex);
		vfsspi_platformUninit(vfsSpiDev);
		if (vfsSpiDev->userInfoData.buffer != NULL)
			kfree(vfsSpiDev->userInfoData.buffer);
		/* Remove device entry. */
		list_del(&vfsSpiDev->deviceEntry);
		device_destroy(vfsSpiDevClass, vfsSpiDev->devt);
		kfree(vfsSpiDev);
		mutex_unlock(&deviceListMutex);
	}
	return status;
}


int vfsspi_platformInit(struct vfsspi_devData *vfsSpiDev)
{
	int status = 0;
	DPRINTK("vfsspi_platformInit\n");
	if (vfsSpiDev == NULL) {
		DPRINTK("vfsspi_platformInit: vfsSpiDev is NULL\n");
		return -EFAULT;
	}		
		#if 1 //PLATFORM_MTK
		vfsSpiDev->drdyPin = GPIO_FPC_EINT_PIN;
		vfsSpiDev->sleepPin  = VFSSPI_SLEEP_PIN;
	
/*		mt_set_gpio_mode(vfsSpiDev->drdyPin, GPIO_MODE_00),
		mt_set_gpio_dir(vfsSpiDev->drdyPin, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(vfsSpiDev->drdyPin, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(vfsSpiDev->drdyPin, GPIO_PULL_UP);
*/			
 		if (mt_set_gpio_mode(vfsSpiDev->sleepPin, GPIO_MODE_00))
			return -EBUSY;
		//if (mt_set_gpio_mode(GPIO_SPI_CS_PIN, GPIO_MODE_00))
		///	return -EBUSY;
		status =  mt_set_gpio_dir(vfsSpiDev->sleepPin, GPIO_DIR_OUT);
		mt_set_gpio_out(vfsSpiDev->sleepPin,GPIO_OUT_ZERO);//add
	
		//status =  mt_set_gpio_dir(GPIO_SPI_CS_PIN, GPIO_DIR_OUT);
		//mt_set_gpio_out(GPIO_SPI_CS_PIN,GPIO_OUT_ZERO);//add
               
		if (status < 0) {
			DPRINTK("gpio_direction_input DRDY failed\n");
			return -EBUSY;
		}
	
		gpio_irq=VFSSPI_INT_IRQNO;
		if (gpio_irq < 0) {
			DPRINTK("gpio_to_irq failed\n");
			return -EBUSY;
		}
		
		mt_eint_set_sens(gpio_irq, 0);
		mt_eint_set_polarity(gpio_irq,1);
		mt_eint_set_hw_debounce(gpio_irq, 1);
		mt_eint_registration(gpio_irq, EINTF_TRIGGER_FALLING, vfsspi_irq,1);
		
		#else
		//vfsSpiDev->drdyPin = VFSSPI_DRDY_PIN;
		//vfsSpiDev->sleepPin = VFSSPI_SLEEP_PIN;

		status = gpio_request(vfsSpiDev->drdyPin, "vfsspi_drdy");
		if (status < 0) {
			DPRINTK("gpio_request(DRDY) is failed! status=%d\n",
					status);
			return -EBUSY;
		}

		status = gpio_request(vfsSpiDev->sleepPin, "vfsspi_sleep");
		if (status < 0) {
			DPRINTK("gpio_request(SLEEP)is failed! status=%d\n",
					status);
			return -EBUSY;
		}

	#if 0//VCS_FEATURE_SENSOR_WINDSOR
		status = gpio_direction_output(vfsSpiDev->sleepPin, 1);
	#else
	    status = gpio_direction_output(vfsSpiDev->sleepPin, 0);
	#endif
		if (status < 0) {
			DPRINTK("gpio_direction_output is failed! status=%d\n",
					status);
			return -EBUSY;
		}

		status = gpio_direction_input(vfsSpiDev->drdyPin);
		if (status < 0) {
			DPRINTK("gpio_direction_input is failed! status=%d\n",
					status);
			return -EBUSY;
		}

		gpio_irq = gpio_to_irq(vfsSpiDev->drdyPin);
		if (gpio_irq < 0) {
			DPRINTK("gpio_to_irq failed! gpio_irq=%d\n", gpio_irq);
			return -EBUSY;
		}

		status = request_irq(gpio_irq, vfsspi_irq, DRDY_IRQ_FLAG, "vfsspi_irq",
			vfsSpiDev);
		if (status < 0) {
			DPRINTK("request_irq failed! status=%d\n", status);
			vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_DISABLE;
			return -EBUSY;
		}		
		#endif
		
		vfsSpiDev->freqTable = freqTable;
		vfsSpiDev->freqTableSize = sizeof(freqTable);
		vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_ENABLE;
		
		DPRINTK("vfsspi_platofrminit: status=%d\n", status);
		return status;
}

void vfsspi_platformUninit(struct vfsspi_devData *vfsSpiDev)
{
	DPRINTK("vfsspi_platformUninit\n");
	if (vfsSpiDev != NULL) {
		vfsSpiDev->freqTable = NULL;
		vfsSpiDev->freqTableSize = 0;
		#if 1 //PLATFORM_MTK
		#else
		free_irq(gpio_irq, vfsSpiDev);
		#endif
		vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_DISABLE;
		#if 1 //PLATFORM_MTK
		#else
		gpio_free(vfsSpiDev->sleepPin);
		gpio_free(vfsSpiDev->drdyPin);
		#endif
	}
}

static int __init vfsspi_init(void)
{
	int status = 0;
	DPRINTK("vfsspi_init\n");
	/* register major number for character device */
	status = register_chrdev(VFSSPI_MAJOR, "validity_fingerprint",
							 &vfsspi_fops);
	if (status < 0) {
		DPRINTK("register_chrdev failed\n");
		return status;
	}
	vfsSpiDevClass = class_create(THIS_MODULE, "validity_fingerprint");
	if (IS_ERR(vfsSpiDevClass)) {
		DPRINTK("vfsspi_init: class_create() is failed "
						"- unregister chrdev.\n");
		unregister_chrdev(VFSSPI_MAJOR, vfsspi_spi.driver.name);
		return PTR_ERR(vfsSpiDevClass);
	}
    	spi_register_board_info(spi_boardinfo, ARRAY_SIZE(spi_boardinfo));
	status = spi_register_driver(&vfsspi_spi);
	if (status < 0) {
		DPRINTK("vfsspi_init: spi_register_driver() is failed "
						"- unregister chrdev.\n");
		class_destroy(vfsSpiDevClass);
		unregister_chrdev(VFSSPI_MAJOR, vfsspi_spi.driver.name);
		return status;
	}
	DPRINTK("init is successful\n");
	return status;
}

static void __exit vfsspi_exit(void)
{
	DPRINTK("vfsspi_exit\n");
	spi_unregister_driver(&vfsspi_spi);
	class_destroy(vfsSpiDevClass);
	unregister_chrdev(VFSSPI_MAJOR, vfsspi_spi.driver.name);
}

module_init(vfsspi_init);
module_exit(vfsspi_exit);

MODULE_LICENSE("GPL");
