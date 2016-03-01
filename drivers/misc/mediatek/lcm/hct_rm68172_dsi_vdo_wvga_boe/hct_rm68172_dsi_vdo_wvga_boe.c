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
#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"
//wqtao. add. end.

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                  (480)
#define FRAME_HEIGHT                 (800)

#define LCM_ID       (0x7281)


#define REGFLAG_DELAY             	(0x1EF)
#define REGFLAG_END_OF_TABLE      	(0x100)	// END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

//#define DISPLAY_DIRECTION_180_MODE
#define LCM_MODULE_HSDA_FEER

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},//page2
	{0xF6,2,{0x60,0x40}},
	{0xFE,4,{0x01,0x80,0x09,0x09}},//add 21050413
	
	{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},//page1
	{0xB0,3,{0x0D,0x0D,0x0D}},
	{0xB1,3,{0x0D,0x0D,0x0D}},
	{0xB2,1,{0x00}},
	{0xB5,3,{0x08,0x08,0x08}},
	{0xB6,3,{0x34,0x34,0x34}},
	{0xB7,3,{0x44,0x44,0x44}},
	{0xB8,3,{0x24,0x24,0x24}},
	{0xB9,3,{0x34,0x34,0x34}},
	{0xBA,3,{0x14,0x14,0x14}},
	{0xBC,3,{0x00,0x78,0x00}},
	{0xBD,3,{0x00,0x78,0x00}},
	{0xBE,2,{0x00,0x3c}},
	{0xD1,52,{0x00,0x00,0x00,0x11,0x00,0x2E,0x00,0x47,0x00,0x5C,0x00,0x7E,0x00,0x9C,0x00,0xCD,0x00,0xF6,0x01,0x36,0x01,0x6A,0x01,0xBD,0x01,0xFF,0x02,0x01,0x02,0x3B,0x02,0x75,0x02,0x99,0x02,0xC8,0x02,0xE8,0x03,0x11,0x03,0x2B,0x03,0x48,0x03,0x59,0x03,0x65,0x03,0x71,0x03,0xFF}},
	{0xD2,52,{0x00,0x00,0x00,0x11,0x00,0x2E,0x00,0x47,0x00,0x5C,0x00,0x7E,0x00,0x9C,0x00,0xCD,0x00,0xF6,0x01,0x36,0x01,0x6A,0x01,0xBD,0x01,0xFF,0x02,0x01,0x02,0x3B,0x02,0x75,0x02,0x99,0x02,0xC8,0x02,0xE8,0x03,0x11,0x03,0x2B,0x03,0x48,0x03,0x59,0x03,0x65,0x03,0x71,0x03,0xFF}},
	{0xD3,52,{0x00,0x00,0x00,0x11,0x00,0x2E,0x00,0x47,0x00,0x5C,0x00,0x7E,0x00,0x9C,0x00,0xCD,0x00,0xF6,0x01,0x36,0x01,0x6A,0x01,0xBD,0x01,0xFF,0x02,0x01,0x02,0x3B,0x02,0x75,0x02,0x99,0x02,0xC8,0x02,0xE8,0x03,0x11,0x03,0x2B,0x03,0x48,0x03,0x59,0x03,0x65,0x03,0x71,0x03,0xFF}},
	{0xD4,52,{0x00,0x00,0x00,0x11,0x00,0x2E,0x00,0x47,0x00,0x5C,0x00,0x7E,0x00,0x9C,0x00,0xCD,0x00,0xF6,0x01,0x36,0x01,0x6A,0x01,0xBD,0x01,0xFF,0x02,0x01,0x02,0x3B,0x02,0x75,0x02,0x99,0x02,0xC8,0x02,0xE8,0x03,0x11,0x03,0x2B,0x03,0x48,0x03,0x59,0x03,0x65,0x03,0x71,0x03,0xFF}},
	{0xD5,52,{0x00,0x00,0x00,0x11,0x00,0x2E,0x00,0x47,0x00,0x5C,0x00,0x7E,0x00,0x9C,0x00,0xCD,0x00,0xF6,0x01,0x36,0x01,0x6A,0x01,0xBD,0x01,0xFF,0x02,0x01,0x02,0x3B,0x02,0x75,0x02,0x99,0x02,0xC8,0x02,0xE8,0x03,0x11,0x03,0x2B,0x03,0x48,0x03,0x59,0x03,0x65,0x03,0x71,0x03,0xFF}},
	{0xD6,52,{0x00,0x00,0x00,0x11,0x00,0x2E,0x00,0x47,0x00,0x5C,0x00,0x7E,0x00,0x9C,0x00,0xCD,0x00,0xF6,0x01,0x36,0x01,0x6A,0x01,0xBD,0x01,0xFF,0x02,0x01,0x02,0x3B,0x02,0x75,0x02,0x99,0x02,0xC8,0x02,0xE8,0x03,0x11,0x03,0x2B,0x03,0x48,0x03,0x59,0x03,0x65,0x03,0x71,0x03,0xFF}},
	
       {0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},//page3
	{0xB0,7,{0x03,0x15,0xFA,0x00,0x00,0x00,0x00}},
	{0xB2,9,{0xFB,0xFC,0xFD,0xFE,0xF0,0x10,0x00,0x83,0x04}},
	{0xB3,6,{0x5B,0x00,0xFB,0x21,0x23,0x0C}},
	{0xB4,9,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xB5,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x44}},
	{0xB6,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xB7,8,{0x00,0x00,0x20,0x20,0x20,0x20,0x00,0x00}},
	{0xB8,3,{0x00,0x00,0x00}},
	{0xB9,1,{0x82}},
	{0xBA,16,{0x45,0x8A,0x04,0x5F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF5,0x41,0xB9,0x54}},
	{0xBB,16,{0x54,0xA8,0x05,0x4F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF4,0x51,0x9B,0x45}},
	{0xBC,4,{0x47,0xFF,0xFF,0xE3}},
	{0xBD,4,{0xC7,0xFF,0xFF,0xE3}},
	{0xC0,4,{0x00,0x01,0xFA,0x00}},

	{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},//page0
	{0xB0,3,{0x00,0x10,0x10}},//modify 20150413
	{0xB1,1,{0xFC}},
	{0xB4,1,{0x10}},
	{0xBA,1,{0x01}},
	{0xBC,3,{0x02,0x02,0x02}},
	{0x35,1,{0x00}},
	{0x11,0,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,0,{0x00}},
	{REGFLAG_DELAY, 100, {}},
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},
	// Display ON
	//{0x2C, 1, {0x00}},
	//{0x13, 1, {0x00}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x01, 1, {0x00}},
      {REGFLAG_DELAY, 100, {}},

	{0x28, 1, {0x00}},
       {REGFLAG_DELAY, 100, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	{0x4f, 1, {0x01}},
       {REGFLAG_DELAY, 100, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
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
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;



	params->dsi.mode   =SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE;


	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine. 
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;
	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 16;//12 modify 20150413
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 32;
	params->dsi.horizontal_frontporch				= 32;
	//params->dsi.horizontal_blanking_pixel		       = 60;
	params->dsi.horizontal_active_pixel		       = FRAME_WIDTH;

    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;
	// Bit rate calculation
	params->dsi.PLL_CLOCK=175;//227;//254;//254//247
} 


static void init_lcm_registers(void)
{
    unsigned int data_array[16];
	
	data_array[0] = 0x00110500; 
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500; 
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(30);
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);
   // init_lcm_registers();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
	printf("[erick-lk]%s\n", __func__);
#else
	printk("[erick-k]%s\n", __func__);
#endif
}


static void lcm_suspend(void)
{
/*	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);   */

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	//SET_RESET_PIN(0);
	//MDELAY(50);
	#ifdef BUILD_LK
		printf("[erick-lk]%s\n", __func__);
	#else
		printk("[erick-k]%s\n", __func__);
	#endif
}

static void lcm_resume(void)
{
	
	lcm_init();
	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	#ifdef BUILD_LK
		printf("[erick-lk]%s\n", __func__);
	#else
		printk("[erick-k]%s\n", __func__);
	#endif
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

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);
}
#endif

#if 0	//wqtao.		
static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}
#endif

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	unsigned int id = 0;
	unsigned char buffer[6];
	unsigned int data_array[16];

	if(lcm_esd_test)
	{
	    lcm_esd_test = FALSE;
	    return TRUE;
	}

	data_array[0] = 0x00023700;
	dsi_set_cmdq(data_array, 1, 1);
	read_reg_v2(0x0a, buffer, 2);
	id = buffer[0];

	printk("[%s] esd check: id = %x\n", __FUNCTION__, id);

	if(read_reg(0x0a) == 0x9c)
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
 #if 0
    unsigned char para = 0;

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(120);
	  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
	  push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(10);
    dsi_set_cmdq_V2(0x35, 1, &para, 1);     ///enable TE
    MDELAY(10);
#else
	lcm_init();
#endif
    return TRUE;
}
extern void DSI_clk_HS_mode(unsigned char enter); 

static unsigned int lcm_compare_id(void)
{
	unsigned int id0 = 0, id1 = 0;
	unsigned char buffer[3];

	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000108;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10); 

	data_array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 
	
	read_reg_v2(0xC5, buffer, 3);//2
	id0 = buffer[0]; //we only need ID
	id1 = buffer[1]; //we test buffer 1
    id0 = (id0<<8);
	id0 += id1;
#ifdef BUILD_LK
	printf("rm68172 id0=%x, id1=%x, id2=%x\n", buffer[0], buffer[1], buffer[2]);
#else
	printk("rm68172 id0=%x, id1=%x, id2=%x\n", buffer[0], buffer[1], buffer[2]);
#endif	

	return (LCM_ID == id0)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_rm68172_dsi_vdo_wvga_boe = 
{
    .name			= "hct_rm68172_dsi_vdo_wvga_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,	
	.compare_id    = lcm_compare_id,	
//    .esd_check   = lcm_esd_check,	
//    .esd_recover   = lcm_esd_recover,		
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
    //.esd_check   = lcm_esd_check,	
    //.esd_recover   = lcm_esd_recover,	
    .update         = lcm_update,
#endif	//wqtao
};

