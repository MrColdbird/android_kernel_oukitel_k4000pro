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

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define LCM_ID                      								(0x94)

#define REGFLAG_DELAY             								(0XFE)
#define REGFLAG_END_OF_TABLE      								(0x100)	// END OF REGISTERS MARKER

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

#define SET_RESET_PIN(v)    									(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 										(lcm_util.udelay(n))
#define MDELAY(n) 										(lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
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

static struct LCM_setting_table lcm_initialization_setting[] = {
		//Set EXTC	
		{0xB9,3,	{0xFF,0x83,0x94}},		
		//Set Power	
		{0xB1,10,	{0x50,0x12,0x72,0x09,0x33,0x54,0x71,0x51,0x30,0x43}},		
		//Set MIPI	
		{0xBA,6,	{0x63,0x03,0x68,0x6B,0xB2,0xC0}},		
		//Set Display	
		{0xB2,6,	{0x00,0x80,0x64,0x0C,0x06,0x2F}},		
		//Set CYC	
		{0xB4,21,	{0x67,0x64,0x67,0x64,0x67,0x64,0x01,0x05,0x7E,0x35,
					0x00,0x3F,0x67,0x64,0x67,0x64,0x67,0x64,0x01,0x05,0x7E}},		
		//Set D3	
		{0xD3,33,	{0x00,0x00,0x00,0x00,0x00,0x00,0x12,0x10,0x32,0x10,
					0x00,0x00,0x00,0x32,0x13,0xC0,0x00,0x00,0x32,0x10,
					0x08,0x00,0x00,0x47,0x04,0x02,0x02,0x47,0x04,0x00,
					0x47,0x0C,0x40}},		
		//Set GIP	
		{0xD5,44,	{0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x20,0x21,
					0x22,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x19,0x19,0x18,0x18}},		
		//Set D6	
		{0xD6,44,	{0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x23,0x22,
					0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
					0x18,0x18,0x19,0x19}},		
		//Set VCOM	
		{0xB6,2,	{0x3C,0x3C}},		
		//Set Gamma	
		{0xE0,58,	{0x00,0x01,0x07,0x0A,0x0C,0x10,0x13,0x10,0x24,0x36,
					0x47,0x49,0x56,0x69,0x72,0x77,0x88,0x8F,0x90,0xA5,
					0xBC,0x5F,0x5F,0x66,0x6B,0x6F,0x77,0x7F,0x7F,0x00,
					0x01,0x07,0x0A,0x0C,0x0F,0x12,0x10,0x24,0x36,0x47,
					0x49,0x56,0x69,0x72,0x77,0x88,0x8F,0x90,0xA5,0xBC,
					0x5F,0x5F,0x66,0x6B,0x6F,0x77,0x7F,0x7F}},		
		//Set Panel	
		{0xCC,1,	{0x03}},		
		//Set C0	
		{0xC0,2,	{0x1F,0x73}},		
		//Set D4	
		{0xD4,1,	{0x02}},	
		{0x11,1,	{0}},
		{REGFLAG_DELAY, 150,  {}},
		{0xBF,7,	{0x40,0x81,0x50,0x02,0x1A,0xFC,0x02}},
		{0x36,1,	{0x02}},
		{0x29,1,	{0}},
		{REGFLAG_DELAY, 50,  {}},
{REGFLAG_DELAY, 30, {}},  

// Setting ending by predefined flag
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
	{0xB9, 3, {0xFF, 0x83, 0x94}},
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

	params->dsi.mode   = SYNC_PULSE_VDO_MODE;	//SYNC_PULSE_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM			= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine. 
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST; 
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      	= LCM_DSI_FORMAT_RGB888;
	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;
	// Video mode setting		
	params->dsi.intermediat_buffer_num 	= 2;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
        params->dsi.word_count=720 * 3;
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 16;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
    params->dsi.horizontal_sync_active              = 8;
    params->dsi.horizontal_backporch                = 32;
    params->dsi.horizontal_frontporch               = 32;
    params->dsi.horizontal_blanking_pixel              = 60;
	params->dsi.horizontal_active_pixel		       		= FRAME_WIDTH;
	// Bit rate calculation

    params->dsi.PLL_CLOCK=220;
}

static unsigned int lcm_init_resgister(void)
{
    int data_array[16];
    
data_array[0] = 0x00043902;
data_array[1] = 0x9483FFB9;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00073902;
data_array[1] = 0x680363BA;
data_array[2] = 0x00C0B26B;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x000B3902;
data_array[1] = 0x721250B1;
data_array[2] = 0x71543309;
data_array[3] = 0x00434E71;
dsi_set_cmdq(&data_array, 4, 1);
MDELAY(1);

data_array[0] = 0x00073902;
data_array[1] = 0x648065B2;
data_array[2] = 0x002F0705;
dsi_set_cmdq(&data_array, 3, 1);
MDELAY(1);

data_array[0] = 0x00163902;
data_array[1] = 0x016501B4;
data_array[2] = 0x01650165;
data_array[3] = 0x00357E05;
data_array[4] = 0x0165013F;
data_array[5] = 0x01650165;
data_array[6] = 0x00007E05;
dsi_set_cmdq(&data_array, 7, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x005353B6;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00223902;
data_array[1] = 0x0F0000D3;
data_array[2] = 0x1001010F;
data_array[3] = 0x00103210;
data_array[4] = 0x15320000;
data_array[5] = 0x32043504;
data_array[6] = 0x14051415;
data_array[7] = 0x00003337;
data_array[8] = 0x37000037;
data_array[9] = 0x00004005;
dsi_set_cmdq(&data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x002D3902;
data_array[1] = 0x181818D5;
data_array[2] = 0x18181818;
data_array[3] = 0x18181818;
data_array[4] = 0x25181818;
data_array[5] = 0x18262724;
data_array[6] = 0x01040518;
data_array[7] = 0x03060700;
data_array[8] = 0x53565702;
data_array[9] = 0x55505152;
data_array[10] = 0x23202154;
data_array[11] = 0x18181822;
data_array[12] = 0x00000018;
dsi_set_cmdq(&data_array, 13, 1);
MDELAY(1);

data_array[0] = 0x002D3902;
data_array[1] = 0x181818D6;
data_array[2] = 0x18181818;
data_array[3] = 0x18181818;
data_array[4] = 0x22181818;
data_array[5] = 0x18212023;
data_array[6] = 0x06030218;
data_array[7] = 0x04010007;
data_array[8] = 0x14111005;
data_array[9] = 0x12171615;
data_array[10] = 0x24272613;
data_array[11] = 0x18181825;
data_array[12] = 0x00000018;
dsi_set_cmdq(&data_array, 13, 1);
MDELAY(1);

data_array[0] = 0x003B3902;
data_array[1] = 0x0B0300E0;
data_array[2] = 0x1412100F;
data_array[3] = 0x44332411;
data_array[4] = 0x74695446;
data_array[5] = 0x8C908C7B;
data_array[6] = 0x4E51A699;
data_array[7] = 0x53525250;
data_array[8] = 0x03007F53;
data_array[9] = 0x120F0E0B;
data_array[10] = 0x33241114;
data_array[11] = 0x69544644;
data_array[12] = 0x908C7B74;
data_array[13] = 0x51A6998C;
data_array[14] = 0x5252504E;
data_array[15] = 0x007F5353;
dsi_set_cmdq(&data_array, 16, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x00731FC0;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x00000BCC;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000002D4;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

//add code
data_array[0] = 0x00023902;
data_array[1] = 0x000001bd;
dsi_set_cmdq(&data_array, 2, 1);
data_array[0] = 0x00023902;
data_array[1] = 0x000060b1;
dsi_set_cmdq(&data_array, 2, 1);
data_array[0] = 0x00023902;
data_array[1] = 0x000000bd;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00110500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(150);

data_array[0] = 0x00290500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(20);
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(180);

   // lcm_init_resgister();
   // dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
  //	dsi_set_cmdq_V3(lcm_initialization_setting,sizeof(lcm_initialization_setting)/sizeof(lcm_initialization_setting[0]),1);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
int data_array[16];

data_array[0] = 0x00280500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(20);
data_array[0] = 0x00100500;
dsi_set_cmdq(&data_array, 1, 1);
MDELAY(150);
SET_RESET_PIN(0);
	MDELAY(180);
#if 0
//#ifndef BUILD_LK
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(180);
//	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);	//wqtao. enable
//#endif
#endif 
}

static unsigned int lcm_compare_id(void);
static void lcm_resume(void)
{
#ifndef BUILD_LK
        lcm_compare_id();
	lcm_init();
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
	array[1]=0x9483FFB9;// page enable
	dsi_set_cmdq(array, 2, 1);
//		MDELAY(20);


	array[0]=0x00073902; 
	array[1]=0x680363BA;// page enable //9341
	array[2]=0x00C0B26B; 
	dsi_set_cmdq(array, 3, 1); 
//		MDELAY(10); 

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
//		MDELAY(10);
	read_reg_v2(0x0a, buffer, 1);

	#ifndef BUILD_LK
	printk("[%s] hct_hx8394d_dsi_vdo_qhd_hsd lcm esd check. arthur %x\n", __FUNCTION__, buffer[0]);
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
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(120);//Must over 6 ms

	array[0]=0x00043902;
	array[1]=0x9483FFB9;// page enable
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10);

	array[0]=0x00073902; 
	array[1]=0x680363BA;
	array[2]=0x00C0B26B; 
	dsi_set_cmdq(array, 3, 1); 
	MDELAY(10); 


	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; 

#ifdef BUILD_LK
	printf("[HX8394D]%s,  auo id = 0x%x\n", __func__, id);
#else
	printk("[HX8394D]%s,  auo id = 0x%x\n", __func__, id);
#endif

    return (LCM_ID == id)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_hx8394f_dsi_vdo_hd_boe = 
{
	.name			  = "hct_hx8394f_dsi_vdo_hd_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,	
	.compare_id     = lcm_compare_id,	
//	.esd_check   = lcm_esd_check,	
//	.esd_recover   = lcm_esd_recover,	
//	.update         = lcm_update,
};

