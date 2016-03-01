
/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(1920)
#define LCM_ID_R61318 (1318)
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#define   LCM_DSI_CMD_MODE							0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

#define changcmd(a,b,c,d) ((d<<24)|(c<<16)|(b<<8)|a)

#define   LCM_DSI_CMD_MODE							0

void NT35596_DCS_write_1A_1P(unsigned char cmd, unsigned char para)
{
	unsigned int data_array[16];

	data_array[0] = (0x00023902);
	data_array[1] = (0x00000000 | (para << 8) | (cmd));
	dsi_set_cmdq(data_array, 2, 1);

}

void NT35596_DCS_write_1A_0P(unsigned char cmd)
{
	unsigned int data_array[16];

	data_array[0]=(0x00000500 | (cmd<<16));
	dsi_set_cmdq(data_array, 1, 1);

}

static void init_lcm_registers(void)
{


NT35596_DCS_write_1A_1P(0xBA,0x03);    //SET MIPI lane
NT35596_DCS_write_1A_1P(0xC2,0x03);    //SET DSI mode 
 
NT35596_DCS_write_1A_1P(0xFF,0x05);//RELOAD CMD5
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0xC5,0x01);
MDELAY(10);
NT35596_DCS_write_1A_1P(0xFF,0xEE);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x1F,0x45);
NT35596_DCS_write_1A_1P(0x24,0x4F);
NT35596_DCS_write_1A_1P(0x38,0xC8);
NT35596_DCS_write_1A_1P(0x39,0x2C);
NT35596_DCS_write_1A_1P(0x1E,0xBB);
NT35596_DCS_write_1A_1P(0x1D,0x0F);
NT35596_DCS_write_1A_1P(0x7E,0xB1);

NT35596_DCS_write_1A_1P(0xFF,0x00);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x35,0x01);

NT35596_DCS_write_1A_1P(0xFF,0x01);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x00,0x01);
NT35596_DCS_write_1A_1P(0x01,0x55);
NT35596_DCS_write_1A_1P(0x02,0x40);
NT35596_DCS_write_1A_1P(0x05,0x40);
NT35596_DCS_write_1A_1P(0x06,0x4a);
NT35596_DCS_write_1A_1P(0x07,0x24);
NT35596_DCS_write_1A_1P(0x08,0x0C);
NT35596_DCS_write_1A_1P(0x0B,0x87);
NT35596_DCS_write_1A_1P(0x0C,0x87);
NT35596_DCS_write_1A_1P(0x0E,0xB0);
NT35596_DCS_write_1A_1P(0x0F,0xB3);
NT35596_DCS_write_1A_1P(0x11,0x10);
NT35596_DCS_write_1A_1P(0x12,0x10);
NT35596_DCS_write_1A_1P(0x13,0x05);
NT35596_DCS_write_1A_1P(0x14,0x4a);
NT35596_DCS_write_1A_1P(0x15,0x18);
NT35596_DCS_write_1A_1P(0x16,0x18);
NT35596_DCS_write_1A_1P(0x18,0x00);
NT35596_DCS_write_1A_1P(0x19,0x77);
NT35596_DCS_write_1A_1P(0x1A,0x55);
NT35596_DCS_write_1A_1P(0x1B,0x13);
NT35596_DCS_write_1A_1P(0x1C,0x00);
NT35596_DCS_write_1A_1P(0x1D,0x00);
NT35596_DCS_write_1A_1P(0x1E,0x13);
NT35596_DCS_write_1A_1P(0x1F,0x00);

NT35596_DCS_write_1A_1P(0x23,0x00);
NT35596_DCS_write_1A_1P(0x24,0x00);
NT35596_DCS_write_1A_1P(0x25,0x00);
NT35596_DCS_write_1A_1P(0x26,0x00);
NT35596_DCS_write_1A_1P(0x27,0x00);
NT35596_DCS_write_1A_1P(0x28,0x00);

NT35596_DCS_write_1A_1P(0x35,0x00);

NT35596_DCS_write_1A_1P(0x66,0x00);

NT35596_DCS_write_1A_1P(0x58,0x82);
NT35596_DCS_write_1A_1P(0x59,0x02);
NT35596_DCS_write_1A_1P(0x5A,0x02);
NT35596_DCS_write_1A_1P(0x5B,0x02);
NT35596_DCS_write_1A_1P(0x5C,0x82);
NT35596_DCS_write_1A_1P(0x5D,0x82);
NT35596_DCS_write_1A_1P(0x5E,0x02);
NT35596_DCS_write_1A_1P(0x5F,0x02);

NT35596_DCS_write_1A_1P(0x72,0x31);

NT35596_DCS_write_1A_1P(0xFF,0x05);//PAGE 5
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0x00,0x01);
NT35596_DCS_write_1A_1P(0x01,0x0B);
NT35596_DCS_write_1A_1P(0x02,0x0C);
NT35596_DCS_write_1A_1P(0x03,0x09);
NT35596_DCS_write_1A_1P(0x04,0x0A);
NT35596_DCS_write_1A_1P(0x05,0x00);
NT35596_DCS_write_1A_1P(0x06,0x0F);
NT35596_DCS_write_1A_1P(0x07,0x10);
NT35596_DCS_write_1A_1P(0x08,0x00);
NT35596_DCS_write_1A_1P(0x09,0x00);
NT35596_DCS_write_1A_1P(0x0A,0x00);
NT35596_DCS_write_1A_1P(0x0B,0x00);
NT35596_DCS_write_1A_1P(0x0C,0x00);
NT35596_DCS_write_1A_1P(0x0D,0x13);
NT35596_DCS_write_1A_1P(0x0E,0x15);
NT35596_DCS_write_1A_1P(0x0F,0x17);

NT35596_DCS_write_1A_1P(0x10,0x01);
NT35596_DCS_write_1A_1P(0x11,0x0B);
NT35596_DCS_write_1A_1P(0x12,0x0C);
NT35596_DCS_write_1A_1P(0x13,0x09);
NT35596_DCS_write_1A_1P(0x14,0x0A);
NT35596_DCS_write_1A_1P(0x15,0x00);
NT35596_DCS_write_1A_1P(0x16,0x0F);
NT35596_DCS_write_1A_1P(0x17,0x10);
NT35596_DCS_write_1A_1P(0x18,0x00);
NT35596_DCS_write_1A_1P(0x19,0x00);
NT35596_DCS_write_1A_1P(0x1A,0x00);
NT35596_DCS_write_1A_1P(0x1B,0x00);
NT35596_DCS_write_1A_1P(0x1C,0x00);
NT35596_DCS_write_1A_1P(0x1D,0x13);
NT35596_DCS_write_1A_1P(0x1E,0x15);
NT35596_DCS_write_1A_1P(0x1F,0x17);

NT35596_DCS_write_1A_1P(0x20,0x00);
NT35596_DCS_write_1A_1P(0x21,0x03);
NT35596_DCS_write_1A_1P(0x22,0x01);
NT35596_DCS_write_1A_1P(0x23,0x40);
NT35596_DCS_write_1A_1P(0x24,0x40);
NT35596_DCS_write_1A_1P(0x25,0xED);
NT35596_DCS_write_1A_1P(0x29,0x58);
NT35596_DCS_write_1A_1P(0x2A,0x12);
NT35596_DCS_write_1A_1P(0x2B,0x01);

NT35596_DCS_write_1A_1P(0x4B,0x06);
NT35596_DCS_write_1A_1P(0x4C,0x11);
NT35596_DCS_write_1A_1P(0x4D,0x20);
NT35596_DCS_write_1A_1P(0x4E,0x02);
NT35596_DCS_write_1A_1P(0x4F,0x02);

NT35596_DCS_write_1A_1P(0x50,0x20);
NT35596_DCS_write_1A_1P(0x51,0x61);
NT35596_DCS_write_1A_1P(0x52,0x01);
NT35596_DCS_write_1A_1P(0x53,0x63);
NT35596_DCS_write_1A_1P(0x54,0x77);
NT35596_DCS_write_1A_1P(0x55,0xED);
NT35596_DCS_write_1A_1P(0x5B,0x00);
NT35596_DCS_write_1A_1P(0x5C,0x00);
NT35596_DCS_write_1A_1P(0x5D,0x00);
NT35596_DCS_write_1A_1P(0x5E,0x00);
NT35596_DCS_write_1A_1P(0x5F,0x15);

NT35596_DCS_write_1A_1P(0x60,0x75);
NT35596_DCS_write_1A_1P(0x61,0x00);
NT35596_DCS_write_1A_1P(0x62,0x00);
NT35596_DCS_write_1A_1P(0x63,0x00);
NT35596_DCS_write_1A_1P(0x64,0x00);
NT35596_DCS_write_1A_1P(0x65,0x00);
NT35596_DCS_write_1A_1P(0x66,0x00);
NT35596_DCS_write_1A_1P(0x67,0x00);
NT35596_DCS_write_1A_1P(0x68,0x04);
NT35596_DCS_write_1A_1P(0x69,0x00);
NT35596_DCS_write_1A_1P(0x6A,0x00);
NT35596_DCS_write_1A_1P(0x6C,0x40);

NT35596_DCS_write_1A_1P(0x75,0x01);
NT35596_DCS_write_1A_1P(0x76,0x01);
NT35596_DCS_write_1A_1P(0x7A,0x80);
NT35596_DCS_write_1A_1P(0x7B,0xC5);
NT35596_DCS_write_1A_1P(0x7C,0xD8);
NT35596_DCS_write_1A_1P(0x7D,0x60);
NT35596_DCS_write_1A_1P(0x7F,0x15);

NT35596_DCS_write_1A_1P(0x80,0x81);
NT35596_DCS_write_1A_1P(0x83,0x05);
NT35596_DCS_write_1A_1P(0x93,0x08);
NT35596_DCS_write_1A_1P(0x94,0x10);
NT35596_DCS_write_1A_1P(0x8A,0x00);
NT35596_DCS_write_1A_1P(0x9B,0x0F);
NT35596_DCS_write_1A_1P(0xEA,0xFF);
NT35596_DCS_write_1A_1P(0xEC,0x00);

NT35596_DCS_write_1A_1P(0xFF,0x01);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0xFF,0x02);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0xFF,0x04);
NT35596_DCS_write_1A_1P(0xFB,0x01);
NT35596_DCS_write_1A_1P(0xFF,0x00);
NT35596_DCS_write_1A_1P(0xd3,0x05);
NT35596_DCS_write_1A_1P(0xd4,0x04);


NT35596_DCS_write_1A_0P(0x11); 

MDELAY(120);
   
NT35596_DCS_write_1A_1P(0xFF,0x00);
NT35596_DCS_write_1A_1P(0x35,0x00);

NT35596_DCS_write_1A_0P(0x29);

MDELAY(10);

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
#if (LCM_DSI_CMD_MODE)
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#endif

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode = SYNC_EVENT_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;// 0;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count=FRAME_WIDTH*3;	
		
	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 4;//4;//16;///8
	params->dsi.vertical_frontporch					= 8;//16;///8
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 20;//8;
	params->dsi.horizontal_backporch				= 40;//40//60;
	params->dsi.horizontal_frontporch				= 40;//60;//60;//140;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
   // params->dsi.TA_GO =5;
	//params->dsi.compatibility_for_nvk = 1;

	// Bit rate calculation
#if 0
	//params->dsi.pll_div1=37;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
	
	// Bit rate calculation
	params->dsi.pll_div1=1;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	params->dsi.pll_div2=0; 		// div2=0~15: fout=fvo/(2*div2)
	params->dsi.fbk_div =17;//20; // fref=26MHz, fvco=fref*(fbk_div+1)*fbk_sel_real/(div1_real*div2_real) 		
#else
	params->dsi.PLL_CLOCK = 410;
#endif
    params->dsi.cont_clock=0; //1;
	
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20); 
	SET_RESET_PIN(1);
	MDELAY(30);

//	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	init_lcm_registers();

}



static void lcm_suspend(void)
{	
    
	SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
	MDELAY(20); 
	SET_RESET_PIN(1);
	MDELAY(30);
}


static void lcm_resume(void)
{
    lcm_init();
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[4];
	unsigned int array[16];  

	array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0x3A, buffer, 2);
#ifdef BUILD_LK
		printf("%s, myf LK r61318 0A debug:  id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2],buffer[3]);
#else
		printk("%s,  myf kernel 0A r61318 horse debug: id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__,buffer[0],buffer[1],buffer[2],buffer[3]);
#endif

	array[0] = 0x00022902;                          
	array[1] = 0x000000b0; 
	dsi_set_cmdq(array, 2, 1);

	array[0] = 0x00043700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xbf, buffer, 4);
	id = buffer[2]<<8 | buffer[3]; //we only need ID
    #ifdef BUILD_LK
		printf("%s, myf LK r61318 debug: id=0x%08x,  id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2],buffer[3]);
    #else
		printk("%s,  myf kernel r61318 horse debug: id=0x%08x,id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x,id3 = 0x%08x\n", __func__,id, buffer[0],buffer[1],buffer[2],buffer[3]);
    #endif
    #ifdef BUILD_LK
		printf("%s, LK nt35590 debug: nt35590 id = 0x%08x\n", __func__, id);
    #else
		printk("%s, kernel nt35590 horse debug: nt35590 id = 0x%08x\n", __func__, id);
    #endif

    if(id == LCM_ID_R61318)
    	return 1;
    else
        return 0;
}


static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 1);
	if(buffer[0]==0x90)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();

	return TRUE;
}



LCM_DRIVER hct_nt35596_dsi_vdo_fhd_auo = 
{
	.name		= "hct_nt35596_dsi_vdo_fhd_auo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//	.compare_id     = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	.update         = lcm_update,
#endif
};
