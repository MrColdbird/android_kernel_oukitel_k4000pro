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

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                 				(540)
#define FRAME_HEIGHT                				(960)

#define LCM_ID                      				(0x89)

#define REGFLAG_DELAY             				(0XFE)
#define REGFLAG_END_OF_TABLE      				(0x100)	// END OF REGISTERS MARKER



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

#define SET_RESET_PIN(v)    					(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 						(lcm_util.udelay(n))
#define MDELAY(n) 						(lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)						lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)


 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {

  {0xB9, 3, {0xFF,0x83,0x89}}, 


  {0xB1, 20, {0x7F,0x10,0x10,0x32,0x32,0x50,0x10,0xF2,0x58,0x80,
        0x20,0x20,0xF8,0xAA,0xAA,0xA0,0x00,0x80,0x30,0x00}}, 
  
  // Set Display 
  {0xB2, 10, {0x80,0x50,0x05,0x07,0x40,0x38,0x11,0x64,0x55,0x09}}, 
  
  // Set GIP1 
  {0xB4, 11, {0x70,0x70,0x70,0x70,0x00,0x00,0x10,0x76,0x10,0x76,0x90}}, 
  
  // Set GIP2 
  {0xD3, 35, {0x00,0x00,0x00,0x00,0x00,0x08,0x00,0x32,0x10,0x00,
        0x00,0x00,0x03,0xC6,0x03,0xC6,0x00,0x00,0x00,0x00,
        0x35,0x33,0x04,0x04,0x37,0x00,0x00,0x00,0x05,0x08,
        0x00,0x00,0x0A,0x00,0x01}}, 
  
  // Set GIP Forward 
  {0xD5, 38, {0x18,0x18,0x18,0x18,0x19,0x19,0x18,0x18,0x20,0x21,
        0x24,0x25,0x18,0x18,0x18,0x18,0x00,0x01,0x04,0x05,
        0x02,0x03,0x06,0x07,0x18,0x18,0x18,0x18,0x18,0x18,
        0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}}, 
  
  // Set GIP Backward 
  {0xD6, 38, {0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x25,0x24,
        0x21,0x20,0x18,0x18,0x18,0x18,0x07,0x06,0x03,0x02,
        0x05,0x04,0x01,0x00,0x18,0x18,0x18,0x18,0x18,0x18,
        0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}}, 
  
  // Set Gamma 2.5 2015/4/13
  {0xE0, 42, {0x00,0x09,0x0B,0x24,0x21,0x3F,0x22,0x44,0x03,0x08,0x0B,0x16,0x0f,0x13,0x16,0x13,0x15,0x0B,0x18,0x1A,0x1C,
              0x00,0x09,0x14,0x24,0x21,0x3F,0x22,0x44,0x0A,0x0D,0x0E,0x18,0x0f,0x13,0x16,0x13,0x15,0x0B,0x18,0x1A,0x1C}}, 
  
  
  // Set VP_REF=4.6V , VN_REF=-4.6V 
  {0xD2, 1, {0x33}}, 
  
  // Set TCON 
  {0xC7, 4, {0x00,0x80,0x00,0xC0}}, 
  
  // Set Display Direction 
  {0xCC, 1, {0x02}}, 

  {0x11,  0,  {0x00}},
    {REGFLAG_DELAY, 150, {0}},

  // Set VCOM 
  {0xB6, 3, {0x45,0x45,0x00}},  

    {0x29,  0,  {0x00}},
    {REGFLAG_DELAY, 20, {0}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
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
	{0x28, 1, {0x00}},
    {REGFLAG_DELAY, 150, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

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
	//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	params->dsi.mode   = SYNC_PULSE_VDO_MODE;

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
	params->dsi.packet_size						=256;
	// Video mode setting		
	params->dsi.intermediat_buffer_num 				= 2;
	params->dsi.PS							=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active				= 2;//4
	params->dsi.vertical_backporch					= 5;//10
	params->dsi.vertical_frontporch					= 9;//10
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 34;
	params->dsi.horizontal_backporch				= 34;//44
	params->dsi.horizontal_frontporch				= 34;
	//params->dsi.horizontal_blanking_pixel		       		= 60;//60
	params->dsi.horizontal_active_pixel		       		= FRAME_WIDTH;
	// Bit rate calculation
#if 0
	params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
	params->dsi.fbk_sel=4;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
	params->dsi.fbk_div =31;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#else
	params->dsi.PLL_CLOCK=230;//254//247
#endif
} 

static void lcm_init(void)
{
    unsigned int data_array[16];
	
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

#if 0
data_array[0]=0x0043902;
data_array[1]=0x8983ffb9;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0]=0x0083902;
data_array[1]=0x9341ba;
data_array[2]=0x1810a416;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0]=0x00153902;
data_array[1]=0x16167fb1;
data_array[2]=0xd0403414;
data_array[3]=0x208052ec;
data_array[4]=0xaaaaf820;
data_array[5]=0x308000a3;
data_array[6]=0x0;
dsi_set_cmdq(&data_array, 7, 1);
MDELAY(1);

data_array[0]=0x00b3902;
data_array[1]=0x1f5080b2;
data_array[2]=0x11380007;
data_array[3]=0x95d64;
dsi_set_cmdq(&data_array, 4, 1);
MDELAY(1);

data_array[0]=0x00b3902;
data_array[1]=0x545040b4;
data_array[2]=0x1000006d;
data_array[3]=0x791079;
dsi_set_cmdq(&data_array, 4, 1);
MDELAY(1);

data_array[0]=0x00243902;
data_array[1]=0x800d3;
data_array[2]=0x80301;
data_array[3]=0x161032;
data_array[4]=0x16;
data_array[5]=0x0;
data_array[6]=0x1a003300;
data_array[7]=0x371a;
data_array[8]=0x81f00;
data_array[9]=0x1000a00;
dsi_set_cmdq(&data_array, 10, 1);
MDELAY(1);

data_array[0]=0x002a3902;
data_array[1]=0x181919d5;
data_array[2]=0x19181818;
data_array[3]=0x2181819;
data_array[4]=0x181803;
data_array[5]=0x6181801;
data_array[6]=0x4181807;
data_array[7]=0x22212005;
data_array[8]=0x18181823;
data_array[9]=0x18181818;
data_array[10]=0x181818;
data_array[11]=0x0;
dsi_set_cmdq(&data_array, 12, 1);
MDELAY(1);

data_array[0]=0x00273902;
data_array[1]=0x191818d6;
data_array[2]=0x19181819;
data_array[3]=0x5181819;
data_array[4]=0x7181804;
data_array[5]=0x1181806;
data_array[6]=0x3181800;
data_array[7]=0x21222302;
data_array[8]=0x18181820;
data_array[9]=0x18181818;
data_array[10]=0x181818;
dsi_set_cmdq(&data_array, 11, 1);
MDELAY(1);

data_array[0]=0x0023902;
data_array[1]=0x2cc;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0]=0x0023902;
data_array[1]=0x99d2;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0]=0x002b3902;
data_array[1]=0x1f1900e0;
data_array[2]=0x293f312d;
data_array[3]=0xd0b073f;
data_array[4]=0x14120e17;
data_array[5]=0x12081412;
data_array[6]=0x1a001714;
data_array[7]=0x3f322d1e;
data_array[8]=0xb074028;
data_array[9]=0x110f180d;
data_array[10]=0x7131213;
data_array[11]=0x181211;
dsi_set_cmdq(&data_array, 12, 1);
MDELAY(1);

data_array[0]=0x0033902;
data_array[1]=0x4a4ab6;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(120); 

data_array[0] = 0x00290500;  
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(30); 
#endif
}


static void lcm_suspend(void)
{
#ifndef BUILD_LK
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);	//wqtao. enable
//        SET_RESET_PIN(0);
	#ifdef BUILD_LK
		printf("[erick-lk]%s\n", __func__);
	#else
		printk("[erick-k]%s\n", __func__);
	#endif
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
	lcm_init();

	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

	#ifdef BUILD_LK
		printf("[erick-lk]%s\n", __func__);
	#else
		printk("[erick-k]%s\n", __func__);
	#endif
#endif
}



static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	int array[4];
	unsigned char buffer[2];

	if(lcm_esd_test)
	{
	    lcm_esd_test = FALSE;
	    return TRUE;
	}

	/// please notice: the max return packet size is 1
	/// if you want to change it, you can refer to the following marked code
	/// but read_reg currently only support read no more than 4 bytes....
	/// if you need to read more, please let BinHan knows.
	/*
	        unsigned int data_array[16];
	        unsigned int max_return_size = 1;
	        
	        data_array[0]= 0x00003700 | (max_return_size << 16);    
	        
	        dsi_set_cmdq(&data_array, 1, 1);
	*/

	array[0]=0x00043902;
	array[1]=0x8983FFB9;// page enable
	dsi_set_cmdq(array, 2, 1);
//		MDELAY(20);

	array[0]=0x00083902; 
	array[1]=0x009341BA;// page enable 
	array[2]=0x1800A416; 
	dsi_set_cmdq(array, 3, 1); 
//		MDELAY(10); 

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
//		MDELAY(10);
	read_reg_v2(0x0a, buffer, 1);

	#ifndef BUILD_LK
//	printk("[%s] hct_hx8389b_dsi_vdo_qhd_hsd lcm esd check. arthur %x\n", __FUNCTION__, buffer[0]);
	#endif

	if(buffer[0] == 0x1c)
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


static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);
	MDELAY(120);//Must over 6 ms

	array[0]=0x00043902;
	array[1]=0x8983FFB9;// page enable
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10);

	array[0]=0x00083902; 
	array[1]=0x009341BA;// page enable 
	array[2]=0x1800A416; 
	dsi_set_cmdq(array, 3, 1); 
	MDELAY(10); 

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; 

#ifdef BUILD_LK
	printf("[HX8389c]%s,  HX8389c hsd id = 0x%x\n", __func__, id);
#else
	printk("[HX8389c]%s,  HX8389c hsd id = 0x%x\n", __func__, id);
#endif
    return (LCM_ID == id)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_hx8389c_dsi_vdo_qhd_cpt = 
{
    .name			= "hct_hx8389c_dsi_vdo_qhd_cpt",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,	
	.compare_id    = lcm_compare_id,	
	.esd_check   = lcm_esd_check,	
	.esd_recover   = lcm_esd_recover,	
};

