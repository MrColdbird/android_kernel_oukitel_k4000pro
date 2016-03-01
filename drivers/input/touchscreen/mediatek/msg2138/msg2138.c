/* Copyright Statement:
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */


#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/version.h>

#include <linux/dma-mapping.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

//{{TYD-LG
//#include <linux/hw_module_info.h>
//#include <../../../../../../platform/mt6575/kernel/core/include/mach/mt6575_boot.h>
//#include "tpd_custom_msg2133.h"
//}}TYD-LG

#include "tpd_custom_msg2138.h"
#include "cust_gpio_usage.h"

// add define for I/O, tyd-lg
#define GPIO_CTP_RST_MSG2133_PIN           GPIO_CTP_RST_PIN
#define GPIO_CTP_RST_MSG2133_PIN_M_GPIO    GPIO_MODE_00
#if 0
#define CTP_EN_ENABLE 	GPIO_OUT_ZERO
#define CTP_EN_DISABLE 	GPIO_OUT_ONE
#else
#define CTP_EN_ENABLE 	GPIO_OUT_ONE 
#define CTP_EN_DISABLE 	GPIO_OUT_ZERO
#endif

//#define TP_COMPATIBLE

#define N_BYTE_PER_TIME (8)//adair:1024的约数,根据平台修改
#define UPDATE_TIMES (1024/N_BYTE_PER_TIME)

#if defined(Z62_HUATAI_H938_QHD_PROJ)
#undef HCT_TPD_ROTATION
#endif

#define TOUCH_IOC_MAGIC 				'A'

//lkp add 
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

static int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
static int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;
//lkp add over

#define TPD_DEVICE_MSG2138A   "msg2138a"
extern struct tpd_device *tpd;

static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);


static void tpd_eint_interrupt_handler(void);
static void tpd_re_init(void);	/* for ESD protect */

#if 0
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int  tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
static BOOTMODE bootMode = NORMAL_BOOT; 

static u8 huangze_tp_fw = 0;

//#define HAVE_TOUCH_KEY
#ifdef HAVE_TOUCH_KEY
static const u16 tpd_keys_local[]={
	KEY_SEARCH,
	KEY_BACK,
	KEY_MENU,			
	KEY_HOMEPAGE
};
static const u16 tpd_keys_local_fact[]={
	KEY_SEARCH,
	KEY_BACK,
	KEY_MENU,			
	KEY_HOME
};
static int key_value;
#define MAX_KEY_NUM (sizeof(tpd_keys_local) / sizeof(tpd_keys_local[0]))
#endif



#if defined(CTP_PROXIMITY_SUPPORT)||defined(K36_XG_U800_PROJ)||defined(K36_JY_S036_PROJ)||defined(K36_TX_TX038_PROJ)
#define TP_PROXIMITY_SENSOR_NEW
#endif


#ifdef TP_PROXIMITY_SENSOR_NEW
static int PROXIMITY =0;
static int PROXIMITY_STATE = 0;

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <mach/battery_common.h>
#endif

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

//#define TPD_CLOSE_POWER_IN_SLEEP

//#define TP_FIRMWARE_UPDATE

//#define TP_PROXIMITY_SENSOR

//#undef TPD_DEBUG
//#define MSG_TPD_DEBUG
#ifdef MSG_TPD_DEBUG
#define	dbg_print(format, args...)		\
	printk("MSG2133 TPD" format, __func__, ##args)
#else
#define dbg_print(format, args...)
#endif

#define TPD_OK 0

#ifdef TP_PROXIMITY_SENSOR
char ps_data_state[1] = {0};
enum
{
    DISABLE_CTP_PS,
    ENABLE_CTP_PS,
    RESET_CTP_PS
};
#endif
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,0,0))	
	#define TOUCH_ADDR_MSG20XX   (0x4C>>1)
	#define FW_ADDR_MSG20XX      (0xC4>>1)//0x62
	#define FW_UPDATE_ADDR_MSG20XX   (0x92>>1)//0x49
#else
	#define TOUCH_ADDR_MSG20XX   (0x4C)
	#define FW_ADDR_MSG20XX      (0xC4)//0x62
	#define FW_UPDATE_ADDR_MSG20XX   (0x92)//0x49
#endif
///int SMC_SWITCH=0;
#if  0//def TP_FIRMWARE_UPDATE
#define U8 unsigned char
#define S8 signed char
#define U16 unsigned short
#define S16 signed short
#define U32 unsigned int
#define S32 signed int
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,0,0))	
	#define TOUCH_ADDR_MSG20XX   (0x4C>>1)
	#define FW_ADDR_MSG20XX      (0xC4>>1)//0x62
	#define FW_UPDATE_ADDR_MSG20XX   (0x92>>1)//0x49
#else
	#define TOUCH_ADDR_MSG20XX   (0x4C)
	#define FW_ADDR_MSG20XX      (0xC4)//0x62
	#define FW_UPDATE_ADDR_MSG20XX   (0x92)//0x49
#endif

static  short *fw_version;
#define DWIIC_MODE_ISP    0
#define DWIIC_MODE_DBBUS  1
static U8 temp[94][1024];
U32 crc_tab[256];

static int FwDataCnt;
//static int FwVersion;
static struct class *firmware_class;
static struct device *firmware_cmd_dev;

static int update_switch = 0;
#define ENABLE_DMA      1
#if ENABLE_DMA
static u8 *gpDMABuf_va = NULL;
static u32 gpDMABuf_pa = NULL;
#endif

#endif
static int update_switch = 0;
/////////////////////////////////////////////////////
static U8 g_dwiic_info_data[1024];   // Buffer for info data
//////////////////////////////////////////////////////

#ifdef VELOCITY_CUSTOM
extern int tpd_v_magnify_x;
extern int tpd_v_magnify_y;
#endif

struct touch_info
{
    unsigned short y[3];
    unsigned short x[3];
    unsigned short p[3];
    unsigned short count;
};

typedef struct
{
    unsigned short pos_x;
    unsigned short pos_y;
    unsigned short pos_x2;



    unsigned short pos_y2;
    unsigned short temp2;
    unsigned short temp;
    short dst_x;
    short dst_y;
    unsigned char checksum;
} SHORT_TOUCH_STATE;


static const struct i2c_device_id tpd_id[] = {{TPD_DEVICE_MSG2138A , 0}, {}};//msg2133_resistor genzy
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,0,0))	
static struct i2c_board_info __initdata msg2138_i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE_MSG2138A , (0x4c>>1))};
#else
static unsigned short force[] = {0, TOUCH_ADDR_MSG20XX, I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const forces[] = { force, NULL };
static struct i2c_client_address_data addr_data = { .forces = forces, };
#endif

#if 0
static hw_module_info hw_info = {
	.type = HW_MODULE_TYPE_CTP,
	.id = TOUCH_ADDR_MSG20XX,
	.priority = HW_MODULE_PRIORITY_CTP,
	.name = "MSG2133",
	.vendor = "Mstar",
	.more = "320 x 480"
};
#endif

#ifdef HCT_TP_GESTRUE
#define MSG_GESTURE_FUNCTION
static char tpgesture_value[10] = {};
extern void tpgesture_hander();
#endif

///terry add 
#ifdef MSG_GESTURE_FUNCTION
#define MSG_GESTURE_FUNCTION_NODE_PROC
#define CTP_GESTURE_FUNCTION_AUTHORITY_PROC 0777 
#define CTP_UPDATE_GESTURE_AUTHORITY_SYS 0777


#define MSG_GESTURE_FUNCTION_DOUBLECLICK_FLAG  0x01    ///0000 0001
#define MSG_GESTURE_FUNCTION_UPDIRECT_FLAG     0x02    ///0000 0010
#define MSG_GESTURE_FUNCTION_DOWNDIRECT_FLAG   0x04    ///0000 0100
#define MSG_GESTURE_FUNCTION_LEFTDIRECT_FLAG   0x08    ///0000 1000
#define MSG_GESTURE_FUNCTION_RIGHTDIRECT_FLAG  0x10    ///0001 0000

static u8 tpd_gesture_flag = 0;////if 1,enter gesture mode success;
static int last_gesture_status = 0;

///if 1; the tp return mode is this mode 
static u8 tpd_gesture_double_click_mode = 0;
static u8 tpd_gesture_up_direct_mode = 0;
static u8 tpd_gesture_down_direct_mode = 0;
static u8 tpd_gesture_left_direct_mode = 0;
static u8 tpd_gesture_right_direct_mode = 0;

static u8 set_gesture_flag = 0;  

/////1:want to open this mode
static u8 set_gesture_double_click_mode = 0;
static u8 set_gesture_up_direct_mode = 0;
static u8 set_gesture_down_direct_mode = 0;
static u8 set_gesture_left_direct_mode = 0; 
static u8 set_gesture_right_direct_mode = 0;

////right_flag | left_flag | down_flag | up_flag | doubleclick_flag
static u8 set_gesture_mode = 0;

#define MSG_GESTURE_ON_OFF_LOOP_COUNT   10
#endif
///terry add end

#define __FIRMWARE_UPDATE__
#ifdef __FIRMWARE_UPDATE__

/*adair:0777为打开apk升级功能，0664为关闭apk升级功能，无需将宏__FIRMWARE_UPDATE__关闭*/
#define CTP_AUTHORITY 0777//0664

//#define ENABLE_AUTO_UPDATA
#if 0
#define TP_DEBUG(format, ...)	printk(KERN_INFO "MSG2133_MSG21XXA_update_INFO***" format "\n", ## __VA_ARGS__)
#else
#define TP_DEBUG(format, ...)
#endif
#if 0//adair:正式版本关闭
#define TP_DEBUG_ERR(format, ...)	printk(KERN_ERR "MSG2133_MSG21XXA_update_ERR***" format "\n", ## __VA_ARGS__)
#else
#define TP_DEBUG_ERR(format, ...)
#endif
static  char *fw_version;
static u8 temp[94][1024];
//u8  Fmr_Loader[1024];
u32 crc_tab[256];
static u8 g_dwiic_info_data[1024];   // Buffer for info data

static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

#define N_BYTE_PER_TIME (8)//adair:1024的约数,根据平台修改
#define UPDATE_TIMES (1024/N_BYTE_PER_TIME)

#if 0//adair:根据平台不同选择不同位的i2c地址
#define FW_ADDR_MSG21XX   (0xC4)
#define FW_ADDR_MSG21XX_TP   (0x4C)
#define FW_UPDATE_ADDR_MSG21XX   (0x92)
#else
#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
#endif

//lkp add
static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
 * 	file->private_data = adxl345_i2c_client;
 *
 * 		if(file->private_data == NULL)
 * 			{
 * 					printk("tpd: null pointer!!\n");
 * 							return -EINVAL;
 * 								}
 *
 */
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
	
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	void __user *data;

	long err = 0;

	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			

			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			

			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	return err;
}

static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "MSG",
	.fops = &tpd_fops,
};

//lkp add over


/*adair:以下5个以Hal开头的函数需要根据平台修改*/
/*disable irq*/
#if 1
static void HalDisableIrq(void)
{
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1);
 
}
/*enable irq*/
static void HalEnableIrq(void)
{
	//mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1);
 
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}
/*reset the chip*/
static void _HalTscrHWReset(void)
{
/*	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, CTP_EN_DISABLE);
    msleep(120);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, Ctp_enable_level);
	msleep(300);
*/
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(10);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(50);
	TPD_DMESG(" msg2133 reset\n");
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(150);
}
static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size)
{
    int ret;
    i2c_client->addr = addr;
    ret = i2c_master_recv(i2c_client, read_data, size);
    i2c_client->addr = FW_ADDR_MSG21XX_TP;
    
    if(ret <= 0)
    {
		TP_DEBUG_ERR("HalTscrCReadI2CSeq error %d,addr = %d\n", ret,addr);
	}
}

static void i2c_write(u8 addr, u8 *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->addr = addr;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
	i2c_client->addr = FW_ADDR_MSG21XX_TP;
	i2c_client->addr|=I2C_ENEXT_FLAG;

	if(ret <= 0)
	{
		printk("i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
	}
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    int ret;
    i2c_client->addr = addr;
    ret = i2c_master_send(i2c_client, data, size);
    i2c_client->addr = FW_ADDR_MSG21XX_TP;

    if(ret <= 0)
    {
		TP_DEBUG_ERR("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", ret,addr);
	}
}
#endif
/*
static void Get_Chip_Version(void)
{
    printk("[%s]: Enter!\n", __func__);
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[2];

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCE;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    if (dbbus_rx_data[1] == 0)
    {
        // it is Catch2
        TP_DEBUG(printk("*** Catch2 ***\n");)
        //FwVersion  = 2;// 2 means Catch2
    }
    else
    {
        // it is catch1
        TP_DEBUG(printk("*** Catch1 ***\n");)
        //FwVersion  = 1;// 1 means Catch1
    }

}
*/

static void dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

    // Delay some interval to guard the next transaction
    udelay ( 150);//200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
	TP_DEBUG("\n******%s come in*******\n",__FUNCTION__);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);
    udelay ( 150 );//200 );        // delay about 0.1ms
}

static u8 drvISP_Read(u8 n, u8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0};
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay( 800 );//200);
    if (n == 1)
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
        *pDataToRead = dbbus_rx_data[0];
        TP_DEBUG("dbbus=%d,%d===drvISP_Read=====\n",dbbus_rx_data[0],dbbus_rx_data[1]);
  	}
    else
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, pDataToRead, n);
    }

    return 0;
}

static void drvISP_WriteEnable(void)
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    udelay(150);//1.16
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1);
    udelay( 150 );//200);
}

static u8 drvISP_ReadStatus(void)
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(150);//200);
    drvISP_Read(1, &bReadData);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    return bReadData;
}



static void drvISP_BlockErase(u32 addr)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
	TP_DEBUG("\n******%s come in*******\n",__FUNCTION__);
	u32 timeOutCount=0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 3);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
	//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
    timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	}
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;//0xD8;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    //bWriteData[4] = (addr & 0xFF) ;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 5);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
		//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
	timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
	}
}

static void drvISP_Program(u16 k, u8* pDataToWrite)
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
		u32 timeOutCount=0;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);
        for (i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        //msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(150);//200);
       
        timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
			timeOutCount++;
			if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
		}
  
        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, TX_data, 133);   //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    }
}

static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    return sprintf ( buf, "%s\n", fw_version );
}


static void drvISP_Verify ( u16 k, u8* pDataToVerify )
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] ={ 0x10, 0x03, 0, 0, 0 };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index = 0;
    u32 timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( u8 ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( u8 ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( u8 ) ( addr + j * 128 );
        udelay ( 100 );        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 ); //write read flash addr
        udelay ( 100 );        // delay about 100us*****
        drvISP_Read ( 128, RX_data );
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                TP_DEBUG ( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static void drvISP_ChipErase()
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
    u32 timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 3 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}

/* update the firmware part, used by apk*/
/*show the fw version*/

static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG_ERR ( "update_C2 OK\n" );
    drvISP_ExitIspMode();
    _HalTscrHWReset();
    FwDataCnt = 0;
    HalEnableIrq();	
    return size;
}

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
	ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}
static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////
    
    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size,  EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
      // Buffer for slave's firmware

    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();
    _HalTscrHWReset();
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x1C70 );


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
 
    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        TP_DEBUG_ERR ( "update_C32 FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	HalEnableIrq();		
        return ( 0 );
    }

    TP_DEBUG_ERR ( "update_C32 OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
	HalEnableIrq();	

    return size;
#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}


static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
    u8  dwiic_rx_data[4];
    u16 reg_data=0;
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );	
    mdelay ( 100 );
    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );
    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
    mdelay ( 50 );

    // recive info data
    //HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 1024 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 8 );
    return ( 1 );
}

static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
  
    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;
    drvTP_read_info_dwiic_c33();
	
    if ( 0/*g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' */)
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

		g_dwiic_info_data[8]=temp[32][8];
		g_dwiic_info_data[9]=temp[32][9];
		g_dwiic_info_data[10]=temp[32][10];
		g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
		g_dwiic_info_data[12]=life_counter[0];
		g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
		drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 50 );
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

        }
        while ( reg_data != 0x2F43 );
        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[0], 1024 );
        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
    }

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }
    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );
    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO )
			i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }
        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        #if 1
        {
            u32 n = 0;
            for(n=0;n<UPDATE_TIMES;n++)
            {
                HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i]+n*N_BYTE_PER_TIME, N_BYTE_PER_TIME );
            }
        }
        #else
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );
        #endif
        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );



    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }
    if ( !update_pass )
    {
        TP_DEBUG_ERR ( "update_C33 FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	HalEnableIrq();	
        return size;
    }
    TP_DEBUG_ERR ( "update_C33 OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
    HalEnableIrq();	
    return size;
}

#define _FW_UPDATE_C3_
#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	HalDisableIrq();

    _HalTscrHWReset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG_ERR ( "111dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG_ERR ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
		}
        else{

            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
        return firmware_update_c2 ( dev, attr, buf, size );
    } 
}
#else
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    _HalTscrHWReset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    
    return size;
}
#endif
static DEVICE_ATTR(update, CTP_AUTHORITY, firmware_update_show, firmware_update_store);
#if 0
/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	printk(" +++++++ [%s] Enter!++++++\n", __func__);
	u16 k=0,i = 0, j = 0;
	u8 bWriteData[5] =
	{
        0x10, 0x03, 0, 0, 0
	};
	u8 RX_data[256];
	u8 bWriteData1 = 0x12;
	u32 addr = 0;
	u32 timeOutCount=0;
	for (k = 0; k < 94; i++)   // total  94 KB : 1 byte per R/W
	{
		addr = k * 1024;
		for (j = 0; j < 8; j++)   //128*8 cycle
		{
			bWriteData[2] = (u8)((addr + j * 128) >> 16);
			bWriteData[3] = (u8)((addr + j * 128) >> 8);
			bWriteData[4] = (u8)(addr + j * 128);
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);

			timeOutCount=0;
			while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
			{
				timeOutCount++;
				if ( timeOutCount >= 100000 ) 
					break; /* around 1 sec timeout */
	  		}
        
			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);
			drvISP_Read(128, RX_data);
			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
			for (i = 0; i < 128; i++)   //log out if verify error
			{
				if (RX_data[i] != 0xFF)
				{
					//TP_DEBUG(printk("k=%d,j=%d,i=%d===============erase not clean================",k,j,i);)
					printk("k=%d,j=%d,i=%d  erase not clean !!",k,j,i);
				}
			}
		}
	}
	TP_DEBUG("read finish\n");
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{

	u8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	printk(" +++++++ [%s] Enter!++++++\n", __func__);
	//msctpc_LoopDelay ( 100 ); 	   // delay about 100ms*****

	// Enable slave's ISP ECO mode

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;
	
	// Disable the Watchdog
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	dbbus_tx_data[3] = 0x00;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(printk("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);

	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(printk("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	//set FRO to 50M
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(printk("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
	TP_DEBUG(printk("chip erase+\n");)
    drvISP_BlockErase(0x00000);
	TP_DEBUG(printk("chip erase-\n");)
    drvISP_ExitIspMode();
    return size;
}
static DEVICE_ATTR(clear, CTP_AUTHORITY, firmware_clear_show, firmware_clear_store);
#endif //0
/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    TP_DEBUG("*** firmware_version_show fw_version = %s***\n", fw_version);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;
/*
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

*/
    fw_version = kzalloc(sizeof(char), GFP_KERNEL);

    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    TP_DEBUG_ERR("***major = %d ***\n", major);
    TP_DEBUG_ERR("***minor = %d ***\n", minor);
    sprintf(fw_version,"%03d%03d", major, minor);
    //TP_DEBUG(printk("***fw_version = %s ***\n", fw_version);)

    return size;
}
static DEVICE_ATTR(version, CTP_AUTHORITY, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    int i;
	TP_DEBUG_ERR("***FwDataCnt = %d ***\n", FwDataCnt);
   // for (i = 0; i < 1024; i++)
    {
        memcpy(temp[FwDataCnt], buf, 1024);
    }
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, CTP_AUTHORITY, firmware_data_show, firmware_data_store);
#ifdef ENABLE_AUTO_UPDATA
static unsigned char mstar_fw_array[]=
{
	#include "FW_AW290_1.5.i"
};

#define	CTP_ID_MSG21XX		0
#define	CTP_ID_MSG21XXA		1

unsigned char getchipType(void)
{
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	
	_HalTscrHWReset();
    mdelay ( 100 );
    
	dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    if ( dbbus_rx_data[0] == 2 )
    {
    	return CTP_ID_MSG21XXA;
    }
    else
    {
    	return CTP_ID_MSG21XX;
    }
    
}

unsigned int getFWPrivateVersion(void)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;

	_HalTscrHWReset();
    mdelay ( 100 );
    
	dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );
    
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    TP_DEBUG_ERR("***FW Version major = %d ***\n", major);
    TP_DEBUG_ERR("***FW Version minor = %d ***\n", minor);
    
    _HalTscrHWReset();
    mdelay ( 100 );
    
    return ((major<<16)|(minor));
}
static int fwAutoUpdate(void *unused)
{
    firmware_update_store(NULL, NULL, NULL, 0);	
}
#endif

#endif  

#if 0//def TP_FIRMWARE_UPDATE
static void i2c_write(u8 addr, u8 *pbt_buf, int dw_lenth)
{
    int ret;
	
   // i2c_client->timing = 150;
    i2c_client->addr = addr;
    ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;
   // i2c_client->timing = 240;

    if(ret <= 0)
    {
        printk("i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
    }
}

static void i2c_read(u8 addr, u8 *pbt_buf, int dw_lenth)
{
    int ret;
	
    //i2c_client->timing = 150;
    i2c_client->addr = addr;
    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;
   // i2c_client->timing = 240;

    if(ret <= 0)
    {
        printk("i2c_read_interface error line = %d, ret = %d\n", __LINE__, ret);
    }
}

static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size)
{
   //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = read_data,
		},
	};

	rc = i2c_transfer(i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCReadI2CSeq error %d\n", rc);
	}
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
   	int rc;
	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc,addr);
	}
}


static void i2c_write_update_msg2138(u8 *pbt_buf, int dw_lenth)
{
    int ret;
    i2c_client->addr = FW_UPDATE_ADDR_MSG20XX;
    ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;

    // ret = i2c_smbus_write_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);
    if(ret <= 0)
    {
        printk("i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
    }
}

static void i2c_write_msg2138(u8 *pbt_buf, int dw_lenth)
{
    int ret;
   // i2c_client->timing = 40;
    i2c_client->addr = FW_ADDR_MSG20XX;
    ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;
    //i2c_client->timing = 240;

    // ret = i2c_smbus_write_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);
    if(ret <= 0)
    {
        printk("i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
    }
}
static void i2c_read_update_msg2138(u8 *pbt_buf, int dw_lenth)
{
    int ret;
    i2c_client->addr = FW_UPDATE_ADDR_MSG20XX;
    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;
    //  ret=i2c_smbus_read_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);

    if(ret <= 0)
    {
        printk("i2c_read_interface error line = %d, ret = %d\n", __LINE__, ret);
    }
}

static void i2c_read_msg2138(u8 *pbt_buf, int dw_lenth)
{
    int ret;
    //i2c_client->timing = 40;
    i2c_client->addr = FW_ADDR_MSG20XX;
    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;
    //i2c_client->timing = 240;
    //  ret=i2c_smbus_read_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);

    if(ret <= 0)
    {
        printk("i2c_read_interface error line = %d, ret = %d\n", __LINE__, ret);
    }
}

#if ENABLE_DMA
static ssize_t msg2138_dma_read_m_byte(u8 *returnData_va, u32 returnData_pa, U16 len)
{
    char     readData = 0;
    int     ret = 0, read_count = 0, read_length = 0;
    int    i, total_count = len;

    if(len == 0)
    {
        printk("[Error]msg2138_dma_read Read Len should not be zero!! \n");
        return 0;
    }

    //gpDMABuf_va = returnData_va; //mtk
    i2c_client->addr = FW_UPDATE_ADDR_MSG20XX;
    i2c_client->addr |= I2C_DMA_FLAG;
    returnData_va[0] = 0x11;
    ret = i2c_master_send(i2c_client, returnData_pa, 1);

    if(ret < 0)
    {
        printk("[Error]MATV sends command error!! \n");
        i2c_client->addr = TOUCH_ADDR_MSG20XX;
        return 0;
    }

    ret = i2c_master_recv(i2c_client, returnData_pa, len); // mtk
    i2c_client->addr = TOUCH_ADDR_MSG20XX;

    if(ret < 0)
    {
        printk("[Error]msg2138_dma_read reads data error!! \n");
        return 0;
    }

    //for (i = 0; i< total_count; i++)
    //    MATV_LOGD("[MATV]I2C ReadData[%d] = %x\n",i,returnData_va[i]);
    return 1;
}

//{{H: modified for transfer in DMA mode, tyd-lg
/*/ wangbo
ssize_t msg2138_dma_write_m_byte(u8 *Data, U16 len)
{
    char    write_data[8] = {0};

    int    i, ret = 0, write_count = 0, write_length = 0;
    int    total_count = len;

    if(len == 0)
    {
        printk("[Error] msg2138_dma_write Len should not be zero!! \n");
        return 0;
    }

    //pr_ch("mt5192_dma_write_m_byte, va=%x, pa=%x, len = %x\n",writeData_va,writeData_pa,len);
    gpDMABuf_va = Data;
    i2c_client->addr = FW_UPDATE_ADDR_MSG20XX;
    i2c_client->addr |= I2C_DMA_FLAG;
    ret = i2c_master_send(i2c_client, gpDMABuf_pa, len + 1);
    i2c_client->addr = TOUCH_ADDR_MSG20XX;

    if(ret < 0)
    {
        printk("[Error] msg2138_dma_write data error!! \n");
        return 0;
    }

    return 1;
}
//*/
#define MAX_CMD_LEN 255
static ssize_t msg2138_dma_write_m_byte(u8 *Data, U16 len)
{
    char addr_bak;
	u32 phyAddr = 0; 
    	u8 *buf_dma = NULL;
    	u32 old_addr = 0; 
    	int ret = 0; 
    	int retry = 3; 

	addr_bak = i2c_client->addr;
	i2c_client->addr = FW_UPDATE_ADDR_MSG20XX;
	i2c_client->addr |= I2C_ENEXT_FLAG;
	 if (len > MAX_CMD_LEN) {
        //TPD_LOGI("[i2c_master_send_ext] exceed the max write length \n"); 
        return -1; 
    }

    phyAddr = 0; 
    buf_dma = dma_alloc_coherent(0, len, &phyAddr, GFP_KERNEL);
    if (NULL == buf_dma) {
        //TPD_LOGI("[i2c_master_send_ext] Not enough memory \n"); 
        return -1; 
    }
    memcpy(buf_dma, Data, len); 
    i2c_client->addr |= I2C_DMA_FLAG;//save flag I2C_ENEXT_FLAG

    do {
        ret = i2c_master_send(i2c_client, (u8*)phyAddr, len);     
        retry --; 
        if (ret != len) {
            //TPD_LOGI("[i2c_master_send_ext] Error sent I2C ret = %d\n", ret); 
        }
    }while ((ret != len) && (retry > 0)); 

    dma_free_coherent(0, len, buf_dma, phyAddr);

    i2c_client->addr = addr_bak;

    return 1;	
}
//}}H:

static U8 drvISP_DMA_Read(U8 *pDataToRead, U32 pa_addr, U8 n)    //First it needs send 0x11 to notify we want to get flash data back.
{
    //    U8 Read_cmd = 0x11;
    //    unsigned char dbbus_rx_data[2] = {0};
    //    i2c_write_update_msg2138( &Read_cmd, 1);
    msg2138_dma_read_m_byte(pDataToRead, pa_addr, n);
    return 0;
}

#endif

static void Get_Chip_Version(void)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[2];
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCE;
    i2c_write_msg2138(&dbbus_tx_data[0], 3);
    i2c_read_msg2138(&dbbus_rx_data[0], 2);

    if(dbbus_rx_data[1] == 0)
    {
        // it is Catch2
        //FwVersion  = 2;// 2 means Catch2
    }
    else
    {
        // it is catch1
        //FwVersion  = 1;// 1 means Catch1
    }
}

static void dbbusDWIICEnterSerialDebugMode(void)
{
    U8 data[5];
    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    i2c_write_msg2138(data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    U8 data[1];
    // Stop the MCU
    data[0] = 0x37;
    i2c_write_msg2138(data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    U8 data[1];
    // IIC Use Bus
    data[0] = 0x35;
    i2c_write_msg2138(data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    U8 data[1];
    // IIC Re-shape
    data[0] = 0x71;
    i2c_write_msg2138(data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    U8 data[1];
    // IIC Not Use Bus
    data[0] = 0x34;
    i2c_write_msg2138(data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    U8 data[1];
    // Not Stop the MCU
    data[0] = 0x36;
    i2c_write_msg2138(data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    U8 data[1];
    // Exit the Serial Debug Mode
    data[0] = 0x45;
    i2c_write_msg2138(data, 1);
    // Delay some interval to guard the next transaction
    //udelay ( 200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
    U8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
    i2c_write_update_msg2138(bWriteData, 5);
    mdelay(10);           // delay about 1ms
}

static U8 drvISP_Read(U8 n, U8 *pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    U8 Read_cmd = 0x11;
    U8 i = 0;
    unsigned char dbbus_rx_data[16] = {0};
    i2c_write_update_msg2138(&Read_cmd, 1);
    //if (n == 1)
    {
        i2c_read_update_msg2138(&dbbus_rx_data[0], n + 1);

        for(i = 0; i < n; i++)
        {
            *(pDataToRead + i) = dbbus_rx_data[i + 1];
        }
    }
    //else
    {
        //     i2c_read_update_msg2138(pDataToRead, n);
    }
    return 0;
}

static void drvISP_WriteEnable(void)
{
    U8 bWriteData[2] =
    {
        0x10, 0x06
    };
    U8 bWriteData1 = 0x12;
    i2c_write_update_msg2138(bWriteData, 2);
    i2c_write_update_msg2138(&bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
    U8 bWriteData = 0x24;
    i2c_write_update_msg2138(&bWriteData, 1);
}

static U8 drvISP_ReadStatus(void)
{
    U8 bReadData = 0;
    U8 bWriteData[2] =
    {
        0x10, 0x05
    };
    U8 bWriteData1 = 0x12;
    i2c_write_update_msg2138(bWriteData, 2);
    drvISP_Read(1, &bReadData);
    i2c_write_update_msg2138(&bWriteData1, 1);
    return bReadData;
}

static void drvISP_SectorErase(U32 addr)
{
    U8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    U8 bWriteData1 = 0x12;
    dbg_print("The drvISP_ReadStatus0=%d\n", drvISP_ReadStatus());
    drvISP_WriteEnable();
    dbg_print("The drvISP_ReadStatus1=%d\n", drvISP_ReadStatus());
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2138(&bWriteData, 2);
    i2c_write_update_msg2138(&bWriteData1, 1);
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2138(bWriteData, 3);
    i2c_write_update_msg2138(&bWriteData1, 1);
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2138(bWriteData, 2);
    i2c_write_update_msg2138(&bWriteData1, 1); 
    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        ;
    }
    drvISP_WriteEnable();
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x20;        //Sector Erase
    bWriteData[2] = (( addr >> 16) & 0xFF);
    bWriteData[3] = (( addr >> 8 ) & 0xFF);
    bWriteData[4] = ( addr & 0xFF);      
    i2c_write_update_msg2138(&bWriteData, 5);
    i2c_write_update_msg2138(&bWriteData1, 1);
    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        ;
    }
}

static void drvISP_BlockErase(U32 addr)
{
    U8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    U8 bWriteData1 = 0x12;
    drvISP_WriteEnable();
    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    i2c_write_update_msg2138(bWriteData, 2);
    i2c_write_update_msg2138(&bWriteData1, 1);
    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    i2c_write_update_msg2138(bWriteData, 3);
    i2c_write_update_msg2138(&bWriteData1, 1);
    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    i2c_write_update_msg2138(bWriteData, 2);
    i2c_write_update_msg2138(&bWriteData1, 1);

    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        ;
    }

    drvISP_WriteEnable();
    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    // bWriteData[4] = (addr & 0xFF) ;
    i2c_write_update_msg2138(bWriteData, 2);
    //i2c_write_update_msg2138( &bWriteData, 5);
    i2c_write_update_msg2138(&bWriteData1, 1);

    while((drvISP_ReadStatus() & 0x01) == 0x01)
    {
        ;
    }
}

static void drvISP_Program(U16 k, U8 *pDataToWrite)
{
    U16 i = 0;
    U16 j = 0;
    //U16 n = 0;
    U8 TX_data[133];
    U8 bWriteData1 = 0x12;
    U32 addr = k * 1024;
#if ENABLE_DMA

    for(j = 0; j < 8; j++)    //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);

        for(i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }

        while((drvISP_ReadStatus() & 0x01) == 0x01)
        {
            ;    //wait until not in write operation
        }

        drvISP_WriteEnable();
        i2c_write_update_msg2138( TX_data, 133);   //write 133 byte per cycle
        //msg2138_dma_write_m_byte(TX_data, 133);
        i2c_write_update_msg2138(&bWriteData1, 1);
    }

#else

    for(j = 0; j < 512; j++)    //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 2 * j) >> 16;
        TX_data[3] = (addr + 2 * j) >> 8;
        TX_data[4] = (addr + 2 * j);

        for(i = 0; i < 2; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 2 + i];
        }

        while((drvISP_ReadStatus() & 0x01) == 0x01)
        {
            ;    //wait until not in write operation
        }

        drvISP_WriteEnable();
        i2c_write_update_msg2138(TX_data, 7);    //write 133 byte per cycle
        i2c_write_update_msg2138(&bWriteData1, 1);
    }

#endif
}


static void drvISP_Verify(U16 k, U8 *pDataToVerify)
{
    U16 i = 0, j = 0;
    U8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    U8 bWriteData1 = 0x12;
    U32 addr = k * 1024;
    U8 index = 0;
#if ENABLE_DMA
    U8 *RX_data = gpDMABuf_va; //mtk

    for(j = 0; j < 8; j++)    //128*8 cycle
    {
        bWriteData[2] = (U8)((addr + j * 128) >> 16);
        bWriteData[3] = (U8)((addr + j * 128) >> 8);
        bWriteData[4] = (U8)(addr + j * 128);

        while((drvISP_ReadStatus() & 0x01) == 0x01)
        {
            ;    //wait until not in write operation
        }

        i2c_write_update_msg2138(bWriteData, 5);     //write read flash addr
        drvISP_DMA_Read(gpDMABuf_va, gpDMABuf_pa, 128); //mtk
        i2c_write_update_msg2138(&bWriteData1, 1);     //cmd end

        for(i = 0; i < 128; i++)    //log out if verify error
        {
            if((RX_data[i] != 0) && index < 10)
            {
                index++;
            }

            if(RX_data[i] != pDataToVerify[128 * j + i])
            {
            }
        }
    }

#else
    U8 RX_data[128];

    for(j = 0; j < 256; j++)    //128*8 cycle
    {
        bWriteData[2] = (U8)((addr + j * 4) >> 16);
        bWriteData[3] = (U8)((addr + j * 4) >> 8);
        bWriteData[4] = (U8)(addr + j * 4);

        while((drvISP_ReadStatus() & 0x01) == 0x01)
        {
            ;    //wait until not in write operation
        }

        i2c_write_update_msg2138(bWriteData, 5);     //write read flash addr
        drvISP_Read(4, RX_data);
        i2c_write_update_msg2138(&bWriteData1, 1);     //cmd end

        for(i = 0; i < 4; i++)    //log out if verify error
        {
            if((RX_data[i] != 0) && index < 10)
            {
                index++;
            }

            if(RX_data[i] != pDataToVerify[4 * j + i])
            {
            }
        }
    }

#endif
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* update the firmware part, used by apk*/
/*show the fw version*/

static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
{
    U8 i;
    U8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    update_switch = 1;
    printk("tyd-tp: firmware_update_store\n");
    //drvISP_EntryIspMode();

    //drvISP_BlockErase(0x00000);
    //M by cheehwa _HalTscrHWReset();

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2138(dbbus_tx_data, 4);


    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2138(dbbus_tx_data, 4);
    
    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2138(dbbus_tx_data, 3);
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    i2c_write_msg2138(dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    i2c_write_msg2138(dbbus_tx_data, 4);
    /*
    //------------
    // ISP Speed Change to 400K
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2138( dbbus_tx_data, 3);
    i2c_read_msg2138( &dbbus_rx_data[3], 1);
    //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_tx_data[3]&0xf7;  //Clear Bit3
    i2c_write_msg2138( dbbus_tx_data, 4);
    */
    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2138(dbbus_tx_data, 4);
    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    if (huangze_tp_fw) {
        printk("HUANGZE-TP: come in...\n");
        drvISP_SectorErase(0x000000);
        drvISP_SectorErase(0x001000);
        drvISP_SectorErase(0x002000);
        drvISP_SectorErase(0x003000);
        drvISP_SectorErase(0x004000);
        drvISP_SectorErase(0x005000);
        drvISP_SectorErase(0x006000);
        drvISP_SectorErase(0x007000);
        drvISP_SectorErase(0x008000);
        drvISP_SectorErase(0x009000);
        drvISP_SectorErase(0x00a000);
        drvISP_SectorErase(0x00b000);
        drvISP_SectorErase(0x00c000);
        drvISP_SectorErase(0x00d000);
        drvISP_SectorErase(0x00e000);
        drvISP_SectorErase(0x00f000);


        for(i = 0; i < 59; i++)    // total  94 KB : 1 byte per R/W
        {
            if(i == 14)
            {
                i = 22;
            }

            dbg_print("drvISP_Program\n");
            drvISP_Program(i, temp[i]);    // program to slave's flash

        }
    } else {
        drvISP_BlockErase(0x00000);

        for(i = 0; i < 94; i++)    // total  94 KB : 1 byte per R/W
        {
            // modified for update all, tyd-lg
            //if(i == 14)
            //{
            //    i = 22;
            //}

            drvISP_Program(i, temp[i]);    // program to slave's flash
        }
    }

    drvISP_ExitIspMode();
    FwDataCnt = 0;
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
    //mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    msleep(500);
    update_switch = 0;
    return size;

}

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
	ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}
static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 dbbus_tx_data[4] = {0x10, bank, addr, data};
    i2c_write_msg2138(dbbus_tx_data, 4);
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 dbbus_tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    i2c_write_msg2138(dbbus_tx_data, 5);
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 dbbus_tx_data[3] = {0x10, bank, addr};
    u8 dbbus_rx_data[2] = {0};
    i2c_write_msg2138(dbbus_tx_data, 3);
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    return ( dbbus_rx_data[1] << 8 | dbbus_rx_data[0] );
}

static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////
    
    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size,  EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
      // Buffer for slave's firmware

    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();

    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    mdelay(500);
    
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x1C70 );


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        i2c_write ( TOUCH_ADDR_MSG20XX, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
 
    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    dbg_print ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        printk ( "update FAILED\n" );

        mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
        msleep(100);
        mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
        //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
        //mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
        mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
        mdelay(500);
        
        FwDataCnt = 0;
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        return ( 0 );
    }

    printk ( "update OK\n" );

    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    mdelay(500);
    
    FwDataCnt = 0;
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    
    return size;
#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        dbg_print ( "alex------------ERASE_EMEM_MAIN_START\n");
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}


static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
    u8  dwiic_rx_data[4];
    u16 reg_data=0;
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay ( 1 );
    drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
    mdelay ( 100 );

    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );

    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    i2c_write ( TOUCH_ADDR_MSG20XX, &dwiic_tx_data[0], 5);
    mdelay ( 50 );

    // recive info data
    //i2c_read ( TOUCH_ADDR_MSG20XX, &g_dwiic_info_data[0], 1024 );
    
    HalTscrCReadI2CSeq ( TOUCH_ADDR_MSG20XX, &g_dwiic_info_data[0], 1024 );
    return ( 1 );
}

static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
    U8  dbbus_tx_data[4];
    U8  dbbus_rx_data[2] = {0};
    U8  life_counter[2];
    U32 i, j;
    U32 crc_main, crc_main_tp;
    U32 crc_info, crc_info_tp;
  
    int update_pass = 1;
    U16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    dbg_print ( "alex------------Info_read\n");
    drvTP_read_info_dwiic_c33();
	dbg_print ( "alex------------Info_read done\n");
    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        dbg_print ( "alex------------Mstar\n");
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

		g_dwiic_info_data[8]=temp[32][8];
		g_dwiic_info_data[9]=temp[32][9];
		g_dwiic_info_data[10]=temp[32][10];
		g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
		g_dwiic_info_data[12]=life_counter[0];
		g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
		drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

        }
        while ( reg_data != 0x2F43 );

        dbg_print ( "alex------------Info_write\n");
        // transmit lk info data
       // i2c_write ( TOUCH_ADDR_MSG20XX , &g_dwiic_info_data[0], 1024 );
        HalTscrCDevWriteI2CSeq ( TOUCH_ADDR_MSG20XX , &g_dwiic_info_data[0], 1024 );
        
        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
        dbg_print ( "alex--------z----Info_write_done\n");

    }

    
    dbg_print ( "alex------------ERASE_EMEM_MAIN\n");
    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    dbg_print ( "alex------------ERASE_EMEM_MAIN_DONE\n");
    mdelay ( 1000 );

    //ResetSlave();
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);

	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    mdelay(500);

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );


    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        dbg_print ( "alex------------ enter polling 0x1C70 \n");
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
		
        dbg_print ( "alex------------ enter polling 0x1C70 done \n");
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

	dbg_print ( "alex------------ enter polling 0x2F43 \n");


    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );
	dbg_print ( "alex------------ enter polling 0x2F43 done \n");
	
    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    dbg_print ( "alex------------MAIN WRITE\n");
    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
    
    	dbg_print ( "alex------------MAIN WRITE %d \n",i);
        if ( emem_type == EMEM_INFO )
			i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
       // i2c_write ( TOUCH_ADDR_MSG20XX, temp[i], 1024 );
       
       #if 1
         
        {
            u32 n = 0;
            for(n=0;n<UPDATE_TIMES;n++)
            {
               
                HalTscrCDevWriteI2CSeq ( TOUCH_ADDR_MSG20XX, temp[i]+n*N_BYTE_PER_TIME, N_BYTE_PER_TIME );
            }
        }
       #else
        HalTscrCDevWriteI2CSeq ( TOUCH_ADDR_MSG20XX, temp[i], 1024 );
				#endif
        // polling 0x3CE4 is 0xD0BC
        
		dbg_print ( "alex------------ enter polling 0xD0BC done \n");
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }	
        while ( reg_data != 0xD0BC );		
		dbg_print ( "alex------------ enter polling 0xD0BC done \n");

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }
    dbg_print ( "alex------------MAIN WRITE_DONE\n");

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    dbg_print ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();

    dbg_print ( "alex------------CHECK_CRC\n");
    update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }

    if ( !update_pass )
    {
        printk ( "update FAILED\n" );

        mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
        mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
        msleep(100);
        mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
        //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
    	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
        mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
        mdelay(500);
        
        FwDataCnt = 0;
    	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        return ( 0 );
    }

    printk ( "update OK\n" );
    
	mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    mdelay(500);
    
    FwDataCnt = 0;
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    return size;
}








static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", fw_version);
}


#define _FW_UPDATE_C3_
#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    U8 i;
    U8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    update_switch = 1;
	printk("tyd-tp: firmware_update_store\n");
	printk(KERN_ERR"hct-mjw line %d\n",__LINE__);
    //drvISP_EntryIspMode();
    //drvISP_BlockErase(0x00000);
    //M by cheehwa _HalTscrHWReset();

    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    mdelay(500);


    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2138(dbbus_tx_data, 4);
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2138(dbbus_tx_data, 4);
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    i2c_write_msg2138(dbbus_tx_data, 3);
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    if ( dbbus_rx_data[0] == 2 )
    {
	printk(KERN_ERR"hct-mjw line %d\n",__LINE__);
        dbg_print ( "alex------------C3\n");
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        i2c_write_msg2138(dbbus_tx_data, 3);
        i2c_read_msg2138(&dbbus_rx_data[0], 2);
        dbg_print ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
            dbg_print ( "alex------------C33\n");
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
		}
        else{
            dbg_print ( "alex------------C32\n");
            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
	printk(KERN_ERR"hct-mjw line %d\n",__LINE__);
        return firmware_update_c2 ( dev, attr, buf, size );
    } 

    update_switch = 0;
}
#else


static ssize_t firmware_update_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{
    U8 i;
    U8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    update_switch = 1;
	printk("tyd-tp: firmware_update_store\n");
    //drvISP_EntryIspMode();
    //drvISP_BlockErase(0x00000);
    //M by cheehwa _HalTscrHWReset();
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    mdelay(500);
    //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
    
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2138(dbbus_tx_data, 4);

    
    


    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    dbbus_tx_data[3] = 0x00;
    i2c_write_msg2138(dbbus_tx_data, 4);

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2138(dbbus_tx_data, 4);

    
    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2138(dbbus_tx_data, 4);

    
    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2138(dbbus_tx_data, 3);
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    i2c_write_msg2138(dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    i2c_write_msg2138(dbbus_tx_data, 4);
    /*
    //------------
    // ISP Speed Change to 400K
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    i2c_write_msg2138( dbbus_tx_data, 3);
    i2c_read_msg2138( &dbbus_rx_data[3], 1);
    //pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
    dbbus_tx_data[3] = dbbus_tx_data[3]&0xf7;  //Clear Bit3
    i2c_write_msg2138( dbbus_tx_data, 4);
    */
    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2138(dbbus_tx_data, 4);
    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

	if (huangze_tp_fw) {
		printk("HUANGZE-TP: come in...\n");
		drvISP_SectorErase(0x000000);
		drvISP_SectorErase(0x001000);
		drvISP_SectorErase(0x002000);
		drvISP_SectorErase(0x003000);
		drvISP_SectorErase(0x004000);
		drvISP_SectorErase(0x005000);
		drvISP_SectorErase(0x006000);
		drvISP_SectorErase(0x007000);
		drvISP_SectorErase(0x008000);
		drvISP_SectorErase(0x009000);
		drvISP_SectorErase(0x00a000);
		drvISP_SectorErase(0x00b000);
		drvISP_SectorErase(0x00c000);
		drvISP_SectorErase(0x00d000);
		drvISP_SectorErase(0x00e000);
		drvISP_SectorErase(0x00f000);


		for(i = 0; i < 59; i++)    // total  94 KB : 1 byte per R/W
		{
		    if(i == 14)
		    {
		        i = 22;
		    }

		    dbg_print("drvISP_Program\n");
		    drvISP_Program(i, temp[i]);    // program to slave's flash

		}
	} else {
		drvISP_BlockErase(0x00000);

		for(i = 0; i < 94; i++)    // total  94 KB : 1 byte per R/W
		{
		    // modified for update all, tyd-lg
		    //if(i == 14)
		    //{
		    //    i = 22;
		    //}

		    drvISP_Program(i, temp[i]);    // program to slave's flash
		}
	}

    drvISP_ExitIspMode();
    FwDataCnt = 0;
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
    msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
    msleep(500);
    update_switch = 0;
    return size;
}

#endif

static DEVICE_ATTR(update, 0777, firmware_update_show, firmware_update_store);

/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    U16 k = 0, i = 0, j = 0;
    U8 bWriteData[5] =
    {
        0x10, 0x03, 0, 0, 0
    };
    U8 RX_data[256];
    U8 bWriteData1 = 0x12;
    U32 addr = 0;
	printk("tyd-tp: firmware_clear_show\n");
    for(k = 0; k < 94; i++)    // total  94 KB : 1 byte per R/W
    {
        addr = k * 1024;

        for(j = 0; j < 8; j++)    //128*8 cycle
        {
            bWriteData[2] = (U8)((addr + j * 128) >> 16);
            bWriteData[3] = (U8)((addr + j * 128) >> 8);
            bWriteData[4] = (U8)(addr + j * 128);

            while((drvISP_ReadStatus() & 0x01) == 0x01)
            {
                ;    //wait until not in write operation
            }

            i2c_write_update_msg2138(bWriteData, 5);     //write read flash addr
            drvISP_Read(128, RX_data);
            i2c_write_update_msg2138(&bWriteData1, 1);    //cmd end

            for(i = 0; i < 128; i++)    //log out if verify error
            {
                if(RX_data[i] != 0xFF)
                {
                }
            }
        }
    }

    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size)
{
    U8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
    //msctpc_LoopDelay ( 100 );        // delay about 100ms*****
    // Enable slave's ISP ECO mode
    /*
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();*/
	printk("tyd-tp: firmware_clear_store\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    // Disable the Watchdog
    i2c_write_msg2138(dbbus_tx_data, 4);
    //Get_Chip_Version();
    //FwVersion  = 2;
    //if (FwVersion  == 2)
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x11;
        dbbus_tx_data[2] = 0xE2;
        dbbus_tx_data[3] = 0x00;
        i2c_write_msg2138(dbbus_tx_data, 4);
    }
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    i2c_write_msg2138(dbbus_tx_data, 4);
    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    i2c_write_msg2138(dbbus_tx_data, 4);
    //Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    i2c_write_msg2138(dbbus_tx_data, 3);
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x25;
    i2c_write_msg2138(dbbus_tx_data, 3);
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    i2c_read_msg2138(&dbbus_rx_data[0], 2);
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
    i2c_write_msg2138(dbbus_tx_data, 4);
    //WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    i2c_write_msg2138(dbbus_tx_data, 4);
    //set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    i2c_write_msg2138(dbbus_tx_data, 4);
    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
	if (!huangze_tp_fw)
    	drvISP_BlockErase(0x00000);
    drvISP_ExitIspMode();
    return size;
}


static DEVICE_ATTR(clear, 0777, firmware_clear_show, firmware_clear_store);


/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_version_show\n");
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{

    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major = 0, minor = 0;

    fw_version = kzalloc(sizeof(char), GFP_KERNEL);
    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;
    i2c_write(TOUCH_ADDR_MSG20XX, &dbbus_tx_data[0], 3);
    i2c_read(TOUCH_ADDR_MSG20XX, &dbbus_rx_data[0], 4);
    major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
    minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
    sprintf(fw_version, "%03d%03d", major, minor);

    return size;
}
static DEVICE_ATTR(version, 0777, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
	printk("tyd-tp: firmware_data_show\n");
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    int i;
	printk("tyd-tp: firmware_data_store\n");
    for(i = 0; i < 1024; i++)
    {
        memcpy(temp[FwDataCnt], buf, 1024);
    }

    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, 0777, firmware_data_show, firmware_data_store);
#endif

#ifdef TP_PROXIMITY_SENSOR
static ssize_t show_proximity_sensor(struct device *dev, struct device_attribute *attr, char *buf)
{
    static char temp=2;
    buf = ps_data_state;
    if(temp!=*buf)
    {
    printk("proximity_sensor_show: buf=%d\n\n", *buf);
    temp=*buf;
    }    return 1;
}

static ssize_t store_proximity_sensor(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    U8 ps_store_data[4];

#if 0

    if(buf != NULL && size != 0)
    {
        if(DISABLE_CTP_PS == *buf)
        {
            printk("DISABLE_CTP_PS buf=%d,size=%d\n", *buf, size);
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x80;
            ps_store_data[3] = 0xa1;
            i2c_write(TOUCH_ADDR_MSG20XX, &ps_store_data[0], 4);
            msleep(2000);
            printk("RESET_CTP_PS buf=%d\n", *buf);
            mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
            mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ZERO);
            msleep(100);
            mt_set_gpio_mode(GPIO_CTP_RST_MSG2133_PIN, GPIO_CTP_RST_MSG2133_PIN_M_GPIO);
    		//mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
		//	mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
           mt_set_gpio_dir(GPIO_CTP_RST_MSG2133_PIN, GPIO_DIR_OUT);
           mt_set_gpio_out(GPIO_CTP_RST_MSG2133_PIN, GPIO_OUT_ONE);
            mdelay(500);
            mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        }
        else if(ENABLE_CTP_PS == *buf)
        {
            printk("ENABLE_CTP_PS buf=%d,size=%d\n", *buf, size);
            ps_store_data[0] = 0x52;
            ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x80;
            ps_store_data[3] = 0xa0;
            i2c_write(TOUCH_ADDR_MSG20XX, &ps_store_data[0], 4);
        }
    }

    return size;
#endif 

	return 0;
}

static DEVICE_ATTR(proximity_sensor, 0777, show_proximity_sensor, store_proximity_sensor);
#endif

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = TPD_DEVICE_MSG2138A,
#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
        .owner = THIS_MODULE,
#endif
    },
    .probe = tpd_probe,
    .remove = tpd_remove,
    .id_table = tpd_id,
#if (LINUX_VERSION_CODE<KERNEL_VERSION(3,0,0))	
    .detect = tpd_detect,
    .address_data = &addr_data,
#endif
};


static  void tpd_down(int x, int y, int p)
{
	input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);

#if defined(HCT_TPD_ROTATION)
	if(y<TPD_RES_Y)
	{
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, TPD_RES_X-x);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, TPD_RES_Y-y);
	}
	else
	{
	    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	}
#else	
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
#endif

    //pr_tp("D[%4d %4d %4d] ", x, y, p);
    input_mt_sync(tpd->dev);
    TPD_DOWN_DEBUG_TRACK(x, y);
    dbg_print(KERN_ERR"HCT-MJW TPD_RES_Y is %d\n",TPD_RES_Y);
#ifdef TPD_HAVE_BUTTON
    if (NORMAL_BOOT != get_boot_mode())
    {
        tpd_button(x, y, 1);
    }
#endif
}

static  int tpd_up(int x, int y, int *count)
{
#ifdef TPD_HAVE_BUTTON
        if (NORMAL_BOOT != get_boot_mode())
        {
        	tpd_button(x, y, 0);
        }
#endif
    if(*count > 0)
    {
		input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
        input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
        //pr_tp("U[%4d %4d %4d] ", x, y, 0);
        input_mt_sync(tpd->dev);
        TPD_UP_DEBUG_TRACK(x, y);
        (*count)--;
        return 1;
    }
    return 0;
}
static unsigned char tpd_check_sum(unsigned char *pval)
{
    int i, sum = 0;

    for(i = 0; i < 7; i++)
    {
        sum += pval[i];
    }

    return (unsigned char)((-sum) & 0xFF);
}

static int msg2138_i2c_write(u8 *pbt_buf, int dw_lenth)
{
	int ret;

	//i2c_client->timing = 40;
	i2c_client->addr = TOUCH_ADDR_MSG20XX;
	ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
	//i2c_client->timing = 100;
	if(ret <= 0) {
		printk("%s ERROR\n", __func__);
		return ret;
	}
	return ret;
}

static int msg2138_i2c_read(char *pbt_buf, int dw_lenth)
{
    int ret;

    ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
    if(ret <= 0) {
		printk("%s ERROR\n", __func__);
        return ret;
    }
    return ret;
}

#ifdef TP_PROXIMITY_SENSOR_NEW
static int CTP_Face_Mode_State(void)
{
     return PROXIMITY;
}

static void CTP_Face_Mode_Switch(int onoff_state)

{
    U8 bWriteData[4] =
    {
        0x52, 0x00, 0x4A, 0xA0
    };
//if (PROXIMITY ==1)

    //_TRACE((TSCR_LEVEL_C_TYP,"CtpMsg20xxSetIntpDynamicReportRate: IntpInterval = %d", IntpInterval));
	if(onoff_state==1)
	{
		PROXIMITY =1;
		bWriteData[3] = 0xA0;
		PROXIMITY_STATE = 0;
	}
	else
	{	
		PROXIMITY =0;
		bWriteData[3] = 0xA1;
		PROXIMITY_STATE = 0;
	}

printk("CTP_Face_Mode_Switch  onoff_state %d, PROXIMITY %d\n",onoff_state, PROXIMITY);

   // _TRACE((TSCR_LEVEL_C_TYP,"CtpMsg20xxSetIntpDynamicReportRate: bWriteData[3] = %d", bWriteData[3]));
	i2c_write(TOUCH_ADDR_MSG20XX, &bWriteData[0], 4);	
    //_msg20xx_packet_write(_ctp_dev_i2c_handle, bWriteData, sizeof(bWriteData));

   // setintpmsg20xx++;
   // setintpmsg20xxIntpInterval = IntpInterval;
   //_TRACE((TSCR_LEVEL_C_TYP,"setintpmsg20xx= %d, setintpmsg20xxIntpInterval = %d", setintpmsg20xx, setintpmsg20xxIntpInterval));
	  
    //return 1;
}
static int Get_Ctp_Face_Mode(void)
{
printk("Get_Ctp_Face_Mode PROXIMITY_STATE %d\n",PROXIMITY_STATE);
	if(PROXIMITY_STATE == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
#endif

#ifdef MSG_GESTURE_FUNCTION
/////enable 0:no open ; 1:open
/////if return 0,parameter 'enable' is wrong!

////the first bit
static int msg_SetGestureDoubleClickDirectValue( int enable )
{
	if(enable != 0 && enable != 1)
		return 0;
	if(enable)
		set_gesture_double_click_mode = MSG_GESTURE_FUNCTION_DOUBLECLICK_FLAG;
	else
		set_gesture_double_click_mode = 0;
	
	return 1;
}
////the second bit
static int msg_SetGestureUpDirectValue( int enable )
{
	if(enable != 0 && enable != 1)
		return 0;
	if(enable)
		set_gesture_up_direct_mode = MSG_GESTURE_FUNCTION_UPDIRECT_FLAG;
	else
		set_gesture_up_direct_mode = 0;
	
	return 1;
}
////the third bit
static int msg_SetGestureDownDirectValue( int enable )
{
	if(enable != 0 && enable != 1)
		return 0;
	if(enable)
		set_gesture_down_direct_mode = MSG_GESTURE_FUNCTION_DOWNDIRECT_FLAG;
	else
		set_gesture_down_direct_mode = 0;
	
	return 1;
}
////the fourth bit
static int msg_SetGestureLeftDirectValue( int enable )
{
	if(enable != 0 && enable != 1)
		return 0;
	if(enable)
		set_gesture_left_direct_mode = MSG_GESTURE_FUNCTION_LEFTDIRECT_FLAG;
	else
		set_gesture_left_direct_mode = 0;
	
	return 1;
}
////the fivth bit
static int msg_SetGestureRightDirectValue( int enable )
{
	if(enable != 0 && enable != 1)
		return 0;
	if(enable)
		set_gesture_right_direct_mode = MSG_GESTURE_FUNCTION_RIGHTDIRECT_FLAG;
	else
		set_gesture_right_direct_mode = 0;
	
	return 1;
}

static void msg_GetGestureModeValue( void )
{
	set_gesture_mode = (set_gesture_right_direct_mode)|(set_gesture_left_direct_mode)|(set_gesture_down_direct_mode)|(set_gesture_up_direct_mode)|(set_gesture_double_click_mode);
	TP_DEBUG("***msg_GetGestureModeValue set_gesture_mode = %x ***\n", set_gesture_mode);
}

///return flage
int msg_GetDoubleClickModeFlage( void )
{
	return tpd_gesture_double_click_mode;
}
int msg_GetUpDirectModeFlage( void )
{
	return tpd_gesture_up_direct_mode;
}
int msg_GetDownDirectModeFlage( void )
{
	return tpd_gesture_down_direct_mode;
}
int msg_GetLeftDirectModeFlage( void )
{
	return tpd_gesture_left_direct_mode;
}
int msg_GetRightDirectModeFlage( void )
{
	return tpd_gesture_right_direct_mode;
}

/*
	the result 1: open corection
			 0: error
*/
void msg_OpenGestureFunction( int g_Mode )
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[2] = {0};

#ifdef __MSG_DMA_MODE__
	msg_dma_alloct();
#endif

	/**********open command*********/
	dbbus_tx_data[0] = 0x58;
	
	dbbus_tx_data[1] = 0x00;
	/*
	0000 0001 DoubleClick
	0000 0010 Up Direction
	0000 0100 Down Direction
	0000 1000 Left Direction
	0001 0000 Right Direction
	0001 1111 All Of Five Funciton
	*/
	dbbus_tx_data[2] = (0xFF&g_Mode);
	
	TP_DEBUG("***msg_OpenGestureFunction MSG_Gesture_Function_type = %x ***\n", dbbus_tx_data[2]);
	if(
		(dbbus_tx_data[2] >= 0x01)&&
		(dbbus_tx_data[2] <= 0x1F)
		)
	{
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);//
		tpd_gesture_flag = 1;
		TP_DEBUG("***msg_OpenGestureFunction:the command is Ok!***\n");
	}
	else
	{
		tpd_gesture_flag = 0;
        printk("***msg_OpenGestureFunction :the command is wrong!***\n");
	}
#ifdef __MSG_DMA_MODE__
	msg_dma_release();
#endif
}
/*
	the result 1: close corection
			 0: error
*/
void msg_CloseGestureFunction( void )
{
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[2] = {0};
#ifdef __MSG_DMA_MODE__
	msg_dma_alloct();
#endif

	/*******close command********/
	dbbus_tx_data[0] = 0x59;
	
	dbbus_tx_data[1] = 0x00;
	//close command is 0x00
	dbbus_tx_data[2] = 0x00;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);

	tpd_gesture_flag = 0;

	#ifdef __MSG_DMA_MODE__
	msg_dma_release();
	#endif
}

void input_report_gesture_key(uint8_t keycode)
{
   TP_DEBUG( "\n#####MSG2133 input_report_gesture_key,keycode=0x%x#####\n" ,keycode);
    switch ( keycode )
    {
    case 0x60:
#if 1
		sprintf(tpgesture_value,"UP");
		tpgesture_hander();
#else

		input_report_key(tpd->dev, KEY_POWER, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_POWER, 0);
		input_sync(tpd->dev);
#endif
/*		  input_report_key(tpd->dev, KEY_UP, 1);
          input_sync(tpd->dev);
          input_report_key(tpd->dev, KEY_UP, 0);
          input_sync(tpd->dev);
*/
        TP_DEBUG( "\n MSG2133 Up received\n\n" );
        break;
    case 0x61:
#if 1
		sprintf(tpgesture_value,"DOWN");
		tpgesture_hander();
#else

		input_report_key(tpd->dev, KEY_POWER, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_POWER, 0);
		input_sync(tpd->dev);
#endif
/*		  input_report_key(tpd->dev, KEY_DOWN, 1);
          input_sync(tpd->dev);
          input_report_key(tpd->dev, KEY_DOWN, 0);
          input_sync(tpd->dev);*/
        TP_DEBUG( " MSG2133 down received" );
        break;
    case 0x62:
#if 1
		sprintf(tpgesture_value,"LEFT");
		tpgesture_hander();
#else

		input_report_key(tpd->dev, KEY_POWER, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_POWER, 0);
		input_sync(tpd->dev);
#endif
/*		  input_report_key(tpd->dev, KEY_LEFT, 1);
          input_sync(tpd->dev);
          input_report_key(tpd->dev, KEY_LEFT, 0);
          input_sync(tpd->dev);*/
        TP_DEBUG(" MSG2133 left received");
        break;
    case 0x63:
#if 1
		sprintf(tpgesture_value,"RIGHT");
		tpgesture_hander();
#else

		input_report_key(tpd->dev, KEY_POWER, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_POWER, 0);
		input_sync(tpd->dev);
#endif
/*		  input_report_key(tpd->dev, KEY_RIGHT, 1);
          input_sync(tpd->dev);
          input_report_key(tpd->dev, KEY_RIGHT, 0);
          input_sync(tpd->dev);*/
        TP_DEBUG(" MSG2133 right received");
        break;
	case 0x58: //double click
#if 1
		sprintf(tpgesture_value,"DOUBCLICK");
		tpgesture_hander();
#else

		input_report_key(tpd->dev, KEY_POWER, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, KEY_POWER, 0);
		input_sync(tpd->dev);
#endif
/*	   input_report_key(tpd->dev, KEY_F2, 1);
      input_sync(tpd->dev);
      input_report_key(tpd->dev, KEY_F2, 0);
      input_sync(tpd->dev);*/
		TP_DEBUG("DOUBCLICK received");	
		break;
    default:
        break;
    }
}
#endif
/////terry add end

static int tpd_touchinfo(struct touch_info *cinfo)
{
    SHORT_TOUCH_STATE ShortTouchState;
    BYTE reg_val[8] = {0};
    unsigned int  temp = 0;
    unsigned int swapxy=0;
	int index = 0;
	int i = 0;
#ifdef TPD_HAVE_BUTTON 
	int key = 0;
	int key_value = 0;
#endif
#ifdef TP_PROXIMITY_SENSOR_NEW
	int err;
	hwm_sensor_data sensor_data;
#endif

	///terry add 
#ifdef MSG_GESTURE_FUNCTION
		int closeGesturnRetval = 0;
#endif
	////terry add end

    if(update_switch)
    {
        return false;
    }

    if (msg2138_i2c_read(reg_val, 8) <= 0)
		tpd_re_init();

#ifdef MSG_GESTURE_FUNCTION
	TP_DEBUG("######MSG2133 last_gesture_status = 0x%x#########\n",last_gesture_status);
	if (tpd_gesture_flag == 1)
	{		
		if(( reg_val[0] == 0x52 ) && ( reg_val[1] == 0xFF ) && ( reg_val[2] == 0xFF ) && ( reg_val[3] == 0xFF ) && ( reg_val[4] == 0xFF ) && ( reg_val[6] == 0xFF ) /*&& ( temp == reg_val[7] )*/)
		{
			TP_DEBUG("tpd_touchinfo gesture function mode read data--%0x \n",reg_val[5]);
			if(reg_val[5] == 0x58)
			{
				tpd_gesture_double_click_mode = 1;
			}
			else if(reg_val[5] == 0x60)
			{
				tpd_gesture_up_direct_mode = 1;
			}
			else if(reg_val[5] == 0x61)
			{
				tpd_gesture_down_direct_mode = 1;
			}
			else if(reg_val[5] == 0x62)
			{
				tpd_gesture_left_direct_mode = 1;
			}
			else if(reg_val[5] == 0x63)
			{
				tpd_gesture_right_direct_mode = 1;
			}
			
			last_gesture_status = reg_val[5];
			input_report_gesture_key(reg_val[5]);
		}
		return 1;
	}
#endif
	///terry add end

    ShortTouchState.pos_x = ((reg_val[1] & 0xF0) << 4) | reg_val[2];
    ShortTouchState.pos_y = ((reg_val[1] & 0x0F) << 8) | reg_val[3];
    ShortTouchState.dst_x = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
    ShortTouchState.dst_y = ((reg_val[4] & 0x0F) << 8) | reg_val[6];

    if((ShortTouchState.dst_x) & 0x0800)
    {
        ShortTouchState.dst_x |= 0xF000;
    }

    if((ShortTouchState.dst_y) & 0x0800)
    {
        ShortTouchState.dst_y |= 0xF000;
    }

    ShortTouchState.pos_x2 = ShortTouchState.pos_x + ShortTouchState.dst_x;
    ShortTouchState.pos_y2 = ShortTouchState.pos_y + ShortTouchState.dst_y;
    dbg_print("MSG_ID=0x%x,X= %d ,Y=%d\n", reg_val[0], ShortTouchState.pos_x, ShortTouchState.pos_x);
    temp = tpd_check_sum(reg_val);
    dbg_print("check_sum=%d,reg_val_7=%d\n", temp, reg_val[7]);

    if(temp == reg_val[7])
    {
        dbg_print("TP_PS \nreg_val[1]=0x%x\nreg_val[2]=0x%x\nreg_val[5]=0x%x\nreg_val[6]=0x%x\nreg_val[7]=0x%x by cheehwa\n", reg_val[1], reg_val[2], reg_val[5], reg_val[6], reg_val[7]);

        if(reg_val[0] == 0x52) //CTP  ID
        {	
            if(reg_val[1] == 0xFF && reg_val[4] == 0xFF)
            {
        #ifdef TP_PROXIMITY_SENSOR_NEW
		        if(reg_val[5] == 0x80 && PROXIMITY == 1 ) // close to
		        {
		            dbg_print("TP_PROXIMITY_SENSOR VVV by cheehwa\n");
		            PROXIMITY_STATE = 1;
dbg_print("tpd_touchinfo PROXIMITY_STATE %d\n",PROXIMITY_STATE );
					//map and store data to hwm_sensor_data
					sensor_data.values[0] = Get_Ctp_Face_Mode();
					sensor_data.value_divide = 1;
					sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
					//let up layer to know
					if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
					{
						dbg_print("call hwmsen_get_interrupt_data fail = %d\n", err);
					}
		        }
		        else if(reg_val[5] == 0x40 && PROXIMITY == 1 ) // leave
		        {
		            dbg_print("TP_PROXIMITY_SENSOR XXX by cheehwa\n");
		            PROXIMITY_STATE = 0;
dbg_print("tpd_touchinfo PROXIMITY_STATE %d\n",PROXIMITY_STATE );
					sensor_data.values[0] = Get_Ctp_Face_Mode();
					sensor_data.value_divide = 1;
					sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
					//let up layer to know
					if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
					{
						dbg_print("call hwmsen_get_interrupt_data fail = %d\n", err);
					}
		        }
				else
		
				{
		#endif

#ifdef HAVE_TOUCH_KEY
#define DA33681

        if ( ( reg_val[5] == 0x00 ) || ( reg_val[5] == 0xFF ) )
        {
		    key_value = 0;
        }
#ifdef DA33681
        else if ( reg_val[5] == 0x01 )
        {
            key_value = 4;
        }
        else if ( reg_val[5] == 0x02 )
        {
            key_value = 1;
        }
        else if ( reg_val[5] == 0x04 )
        {
            key_value = 2;
        }
#endif 
        else
        {
            key_value = reg_val[5] & 0xf;
        }
        
#endif

            	point_num = 0;
#ifdef TPD_HAVE_BUTTON 
        if ( ( reg_val[5] == 0x00 ) || ( reg_val[5] == 0xFF ) )
        {
            key_value = 0;
        }
        else
        {
            key_value = reg_val[5] & 0xf;
	    key = 0;
	    while ((key_value >>= 1))
	    {
		key++;
	    }
	    cinfo->x[0] = tpd_keys_dim_local[key][0];
	    cinfo->y[0] = tpd_keys_dim_local[key][1];
	    point_num = 1;	
        }
#endif

		#ifdef TP_PROXIMITY_SENSOR_NEW
				}
		#endif
		        //input_sync(tpd->dev);
            }
            else if(ShortTouchState.pos_x > 2047 || ShortTouchState.pos_y > 2047)
            {
            	return  false;
            }
            else if((ShortTouchState.dst_x == 0) && (ShortTouchState.dst_y == 0))
            {	
            	#ifdef SWAP_XY
            		swapxy = ShortTouchState.pos_x;
            		ShortTouchState.pos_x =ShortTouchState.pos_y;
            		ShortTouchState.pos_y= swapxy;
            	#endif
            		
                cinfo->x[0] =(ShortTouchState.pos_x * TPD_RES_X) / 2048;
                cinfo->y[0] = (ShortTouchState.pos_y * TPD_RES_Y) / 2048;
               /* if(cinfo->x[0] < 1)
                    cinfo->x[0] = 1;
                if(cinfo->x[0] > (TPD_RES_X-1))
                    cinfo->x[0] = TPD_RES_X-1;
                if(cinfo->y[0] < 1)
                    cinfo->y[0] = 1;
                if(cinfo->y[0] > (TPD_RES_Y-1))
                    cinfo->y[0] = TPD_RES_Y-1;
		*/
                point_num = 1;
            }
            else
            {
                if(ShortTouchState.pos_x2 > 2047 || ShortTouchState.pos_y2 > 2047)
                    return false;
                    
            		#ifdef SWAP_XY
            		swapxy = ShortTouchState.pos_x;
            		ShortTouchState.pos_x =ShortTouchState.pos_y;
            		ShortTouchState.pos_y= swapxy;

					swapxy = ShortTouchState.pos_x2;
            		ShortTouchState.pos_x2 =ShortTouchState.pos_y2;
            		ShortTouchState.pos_y2= swapxy;
            		#endif


                cinfo->x[0] =  (ShortTouchState.pos_x * TPD_RES_X) / 2048;
                cinfo->y[0] = (ShortTouchState.pos_y * TPD_RES_Y) / 2048;
                cinfo->x[1] = (ShortTouchState.pos_x2 * TPD_RES_X) / 2048;
                cinfo->y[1] = (ShortTouchState.pos_y2 * TPD_RES_Y) / 2048;
                point_num = 2;
            }
			
        }

#ifdef TP_PROXIMITY_SENSOR
        else if(reg_val[7] == 0x80) // close to
        {
            printk("TP_PROXIMITY_SENSOR VVV by cheehwa\n");
            ps_data_state[0] = 1;
        }
        else if(reg_val[7] == 0x40) // leave
        {
            printk("TP_PROXIMITY_SENSOR XXX by cheehwa\n");
            ps_data_state[0] = 0;
        }
#endif
		else
		{
			return false;
		}

        return true;
    }
    else
    {
        dbg_print("tpd_check_sum_ XXXX\n");
        return  false;
    }

}

static int touch_event_handler(void *unused)
{
    struct touch_info cinfo;
    int touch_state = 3;
    unsigned long time_eclapse;
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);
	int last_key = 0;
	int key;

    do
    {
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);

        if(tpd_touchinfo(&cinfo))
        {
           // printk("PAUL==============> MSG_X = %d,MSG_Y = %d\n", cinfo.x[0], cinfo.y[0]);
	/*		if (!(strncmp(MTK_LCM_PHYSICAL_ROTATION, "180", 3)) && (bootMode == NORMAL_BOOT)) {
				cinfo.x[0] = TPD_RES_X - 1 - cinfo.x[0];
				cinfo.y[0] = TPD_RES_Y - 1 - cinfo.y[0];
				cinfo.x[1] = TPD_RES_X - 1 - cinfo.x[1];
				cinfo.y[1] = TPD_RES_Y - 1 - cinfo.y[1];
			}
			*/


            if(point_num == 1)
            {
                tpd_down(cinfo.x[0], cinfo.y[0], 1);
                dbg_print("msg_press 1 point--->\n");
                input_sync(tpd->dev);
            }
            else if(point_num == 2)
            {
                dbg_print("MSG_X2 = %d,MSG_Y2 = %d\n", cinfo.x[1], cinfo.y[1]);
                tpd_down(cinfo.x[0], cinfo.y[0], 1);
                tpd_down(cinfo.x[1], cinfo.y[1], 1);
                dbg_print("msg_press 2 points--->\n");
                input_sync(tpd->dev);
            }
            else if(point_num == 0)
            {
#ifdef HAVE_TOUCH_KEY
		dbg_print("arthur key 2 %d\n",key_value);
				if (key_value)
				{
					key = MAX_KEY_NUM - 1;
					while ((key_value >>= 1))
					{
						key--;
					}
					last_key = key;
            		dbg_print("current pressed virtual key is %d\n", key);

					input_report_key(tpd->dev, tpd_keys_local[key], 1);
				}
				else
				{
					input_report_key(tpd->dev, tpd_keys_local[last_key], 0);
				}
#endif
                dbg_print("release --->\n");
				input_report_key(tpd->dev, BTN_TOUCH, 0);
    			//input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    			//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
                input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
        }
    }
    while(!kthread_should_stop());

    return 0;
}

static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, TPD_DEVICE_MSG2138A);
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
	dbg_print("Trigger interrupt\n");
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}

static void tpd_re_init(void)
{
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	hwPowerDown(TPD_POWER_SOURCE, "TP");
	msleep(100);	
    hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(100);
    // for enable/reset pin
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
//	mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(400);

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
//    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
//    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_DOWN);

    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	dbg_print("TPD re_init\n");
}

#if defined(TP_PROXIMITY_SENSOR_NEW)
static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					CTP_Face_Mode_Switch(1);
				}
				else
				{
					CTP_Face_Mode_Switch(0);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				printk("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				
				sensor_data->values[0] = Get_Ctp_Face_Mode();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;		
			}
			break;
			
		default:
			printk("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
	
}
#endif

static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int retval = TPD_OK;
	int ret;
	int error = 0;
	BYTE reg_val[8] = {0};
	u8 buf[8];
	char data;

#if defined(TP_COMPATIBLE)
	u8 supplier_id = 0;
	u8 module_id = 0;
	u8 cmd_buf[3] = {0x53, 0x00, 0x2a};
	u8 data_buf[4];
	u8 n = 5;
#endif 

#ifdef TP_PROXIMITY_SENSOR_NEW
	int err;
	struct hwmsen_object obj_ps;
#endif

    i2c_client = client;
    printk("MSG2133 I2C slave ADDR = 0x%x", client->addr);
	msleep(50);		/* needed */

	printk("stephen probe  1111.\n");

	/* tp power on */
    hwPowerDown(TPD_POWER_SOURCE, "TP");
	msleep(100);	
    hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");
	msleep(200);
    /* for enable/reset pin */
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(50);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	printk("stephen probe  22222.\n");
	msleep(300);	/* needed */

#if defined(TP_COMPATIBLE)
//	do {
		ret = msg2138_i2c_write(cmd_buf, 3);
		msleep(50);
		ret |= msg2138_i2c_read(data_buf, 4);
//	} while (n-- && (ret <= 0));

	supplier_id = data_buf[1];
	module_id = data_buf[0];

	printk("MSG2133 0x%x, 0x%x, 0x%x, 0x%x\n", data_buf[0],
			data_buf[1], data_buf[2], data_buf[3]);

	short version_major = 0;	
	fw_version = kzalloc(sizeof(short), GFP_KERNEL);
 	memset(fw_version, 0, sizeof(short));
	version_major = (data_buf[3]<<8) + data_buf[2];
	sprintf(fw_version, "%02x", version_major);
	printk("version_major = %x",version_major);

	if (0x4 == supplier_id) 
		huangze_tp_fw = 1;
	if((module_id<=0)||(version_major!=6))
	{
		return -1;
	}
   /* if(LIGE_ID != supplier_id) {
        printk("=== msg2146 sensor ID is error!\n ");
#ifdef MT6575
		hwPowerDown(mt_POWER_LDO_VGP2, "TP");
#endif
        return -1;
    }*/
#endif
	

#if 1
    if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		printk("MSG2138 TP check I2C communication ERROR val = %d\n", ret);
		return -1; 
	}
#else
	ret = msg2138_i2c_read(buf, 8);
	if (ret <= 0) {
		printk("MSG2133 TP check I2C communication ERROR val = %d\n", ret);
		return -1;
	}

#endif

#ifdef VELOCITY_CUSTOM
	tpd_v_magnify_x = TPD_VELOCITY_CUSTOM_X;
	tpd_v_magnify_y = TPD_VELOCITY_CUSTOM_Y;

#endif


#if defined(TP_PROXIMITY_SENSOR_NEW)
{		
	hwmsen_detach(ID_PROXIMITY);
//	obj_ps.self = cm3623_obj;
	obj_ps.polling = 0;//interrupt mode
//	obj_ps.polling = 1;//need to confirm what mode is!!!
	obj_ps.sensor_operate = tpd_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
//		goto exit_create_attr_failed;
	}
}			
#endif

	
#ifdef HAVE_TOUCH_KEY
	int retry;

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()) {
			memcpy(tpd_keys_local, tpd_keys_local_fact, sizeof(tpd_keys_local));
	}

	for(retry = 0; retry < 4; retry++)
	{
			input_set_capability(tpd->dev,EV_KEY,tpd_keys_local[retry]);
	}
#endif

	printk("stephen probe  333333.\n");

#ifdef MSG_GESTURE_FUNCTION
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_F2);
	input_set_capability(tpd->dev, EV_KEY, KEY_LEFT);
	input_set_capability(tpd->dev, EV_KEY, KEY_RIGHT);
	input_set_capability(tpd->dev, EV_KEY, KEY_UP);
	input_set_capability(tpd->dev, EV_KEY, KEY_DOWN);
#endif

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
//    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
//    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_DOWN);
    //mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    //mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
 	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1);
 //   mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_POLARITY_HIGH, tpd_eint_interrupt_handler, 1);
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    tpd_load_status = 1;


	printk("stephen probe  44444.\n");
	//hw_module_info_add(&hw_info);  //  add hard info to system

    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE_MSG2138A);

    if(IS_ERR(thread))
    {
        retval = PTR_ERR(thread);
        printk(TPD_DEVICE_MSG2138A " failed to create kernel thread: %d\n", retval);
    }

//lkp add 
if((error = misc_register(&tpd_misc_device)))
{
	printk("mtk_tpd: tpd_misc_device register failed\n");

}
//lkp add over

   #ifdef __FIRMWARE_UPDATE__
	firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(firmware_class))
        pr_err("Failed to create class(firmware)!\n");
    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
    if (IS_ERR(firmware_cmd_dev))
        pr_err("Failed to create device(firmware_cmd_dev)!\n");

    // version
    if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	// clear
 //   if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
 //       pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);

	dev_set_drvdata(firmware_cmd_dev, NULL);
    #ifdef	ENABLE_AUTO_UPDATA
	TP_DEBUG_ERR("[TP] check auto updata\n");

	
	if(getchipType() == CTP_ID_MSG21XXA)
	{
		//msg2133A		
		TP_DEBUG_ERR("[TP] TP IC is msg21xxA Version = %d \n", (getFWPrivateVersion()&0xff));
		
		//	if((getFWPrivateVersion()&0xff) < 4)
			{
			    int i = 0, j=0;
				TP_DEBUG_ERR("[TP] TP FW version is less than 4\n");
				
				for (i = 0; i < 33; i++)
    			{
    				for (j = 0; j < 1024; j++)
    				{
        				temp[i][j] = mstar_fw_array[i*1024+j];
    				}
    			}
				//firmware_update_store(NULL, NULL, NULL, 0);	
				kthread_run(fwAutoUpdate, 0, client->name);
			}
	}
#endif

#endif 


#if 0//def TP_FIRMWARE_UPDATE
#if ENABLE_DMA
    gpDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &gpDMABuf_pa, GFP_KERNEL);

    if(!gpDMABuf_va)
    {
        printk("[MATV][Error] Allocate DMA I2C Buffer failed!\n");
    }

#endif
    firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");

    if(IS_ERR(firmware_class))
    {
        printk("Failed to create class(firmware)!\n");
    }

    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");

    if(IS_ERR(firmware_cmd_dev))
    {
        printk("Failed to create device(firmware_cmd_dev)!\n");
    }

    // version
    if(device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
    {
        printk("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    }

    // update /sys/class/mtk-tpd/device/update
    if(device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
    {
        printk("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    }

    // data
    if(device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
    {
        printk("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
    }

    if(device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
    {
        printk("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);
    }

#endif
#ifdef TP_PROXIMITY_SENSOR

    if(device_create_file(firmware_cmd_dev, &dev_attr_proximity_sensor) < 0) // /sys/class/mtk-tpd/device/proximity_sensor
    {
        printk("Failed to create device file(%s)!\n", dev_attr_proximity_sensor.attr.name);
    }
    dbg_print("create device file:%s\n", dev_attr_proximity_sensor.attr.name);
#endif

    printk("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
    return 0;
}

static int  tpd_remove(struct i2c_client *client)

{
    printk("TPD removed\n");
#if 0//def TP_FIRMWARE_UPDATE
#if ENABLE_DMA

    if(gpDMABuf_va)
    {
        dma_free_coherent(NULL, 4096, gpDMABuf_va, gpDMABuf_pa);
        gpDMABuf_va = NULL;
        gpDMABuf_pa = NULL;
    }

#endif
#endif
    return 0;
}


static int tpd_local_init(void)
{
	//{{C:start add for tp buton function in different boot mode,by tyd-lg,2011-07-28
 	bootMode = get_boot_mode();
	if(SW_REBOOT == bootMode || ALARM_BOOT == bootMode)
		bootMode = NORMAL_BOOT;
	//}}C:end

    printk(" msg2138 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

    if(i2c_add_driver(&tpd_i2c_driver) != 0)
    {
		printk("MTK TPD %s@%d  add i2c driver failed\n", __func__, __LINE__);
        return -1;
    }


    if(tpd_load_status == 0)
    {
		printk("add error touch panel driver msg2133.\n");
		i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
#ifdef TPD_HAVE_BUTTON
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT * 4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT * 4);
#endif
#if 0
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8 * 4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8 * 4);
#endif
#endif
    tpd_type_cap = 1;
    return 0;
}

static int tpd_resume(struct i2c_client *client)
{
    int retval = TPD_OK;
#ifdef TP_PROXIMITY_SENSOR_NEW
	int err;
	hwm_sensor_data sensor_data;
#endif
	//printk("MSG2133A: resume g_call_state=%d \n",g_call_state);
#if defined(TP_PROXIMITY_SENSOR_NEW)
	if(g_call_state == CALL_ACTIVE)
	{
		return retval;
	}
	else
	{
		//if(Get_Ctp_Face_Mode()==0)//far
		 {
					PROXIMITY_STATE = 0;
			
					sensor_data.values[0] = Get_Ctp_Face_Mode();
					sensor_data.value_divide = 1;
					printk("MSG2133A:sensor_data-value-near=%d \n",sensor_data.values[0]);
					sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
					//let up layer to know
					if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
					{
						
						//printk("MSG2133A: call mcu near fail");
					}
					CTP_Face_Mode_Switch(0);
		 }
		

	}
#endif

#ifdef MSG_GESTURE_FUNCTION
		msg_CloseGestureFunction();	
#endif

    printk("TPD wake up\n");
    hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP");
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerOn(TPD_POWER_SOURCE, VOL_3300, "TP");
#else
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	//msleep(100);
	msleep(20);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(300);
#endif

//    msleep(200);
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    return retval;
}

static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
    int retval = TPD_OK;
#ifdef TP_PROXIMITY_SENSOR_NEW
	int err;
	hwm_sensor_data sensor_data;
#endif
	//printk("MSG2133A: suspend g_call_state=%d \n",g_call_state);
#if defined(TP_PROXIMITY_SENSOR_NEW)
	if(g_call_state == CALL_ACTIVE)
	{
		return retval;
	}
	else
	{
		//if(Get_Ctp_Face_Mode()==0)//far
		 {
					PROXIMITY_STATE = 1;
			
					sensor_data.values[0] = Get_Ctp_Face_Mode();
					sensor_data.value_divide = 1;
					printk("MSG2133A:sensor_data-value-near=%d \n",sensor_data.values[0]);
					sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
					//let up layer to know
					if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
					{
						
						//printk("MSG2133A: call mcu near fail");
					}
					CTP_Face_Mode_Switch(0);
		 }
		

	}
#endif

#ifdef MSG_GESTURE_FUNCTION
	msg_OpenGestureFunction(0x1F);
	return retval;
#endif

    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);

#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerDown(TPD_POWER_SOURCE, "TP");
#else
    //i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
    //mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    //mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    //mt_set_gpio_out(GPIO_CTP_EN_PIN, CTP_EN_ENABLE);
#endif
	hwPowerDown(TPD_POWER_SOURCE, "TP");
    return retval;
}

#ifdef MSG_GESTURE_FUNCTION
static ssize_t show_tpgesture_value(struct device* dev, struct device_attribute *attr, char *buf)
{
	printk("show tp gesture value is %s \n",tpgesture_value);
	return scnprintf(buf, PAGE_SIZE, "%s\n", tpgesture_value);
}
static DEVICE_ATTR(tpgesture,  0664, show_tpgesture_value, NULL);
static struct device_attribute *tpd_attr_list[] = {
	&dev_attr_tpgesture,
};
#endif

static struct tpd_driver_t tpd_device_driver =
{
    .tpd_device_name = TPD_DEVICE_MSG2138A,
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif
#ifdef MSG_GESTURE_FUNCTION
	.attrs = {
		.attr = &tpd_attr_list,
		.num = (int)(sizeof(tpd_attr_list)/sizeof(tpd_attr_list[0])),
	},
#endif
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    if (META_BOOT == get_boot_mode())
    {   
      	return -1;
    }
#if (LINUX_VERSION_CODE>=KERNEL_VERSION(3,0,0))	
    i2c_register_board_info(1, &msg2138_i2c_tpd, 1);
#endif
    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
        printk("add MSG2138 driver failed\n");
    }

    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    printk("MediaTek MSG2138 touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
