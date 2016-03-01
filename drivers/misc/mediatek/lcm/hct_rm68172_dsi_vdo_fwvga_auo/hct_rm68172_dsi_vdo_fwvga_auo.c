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
#define FRAME_HEIGHT                 (854)

#define LCM_ID       (0x7281)
#define LCM_ID1       (0xC1)
#define LCM_ID2       (0x80)


#define REGFLAG_DELAY             	(0XFFFE)
#define REGFLAG_END_OF_TABLE      	(0x1F00)	// END OF REGISTERS MARKER


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
{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x02}},
{0xF6, 2, {0x60, 0x40}},
{0xFE, 4, {0x01, 0x80, 0x09, 0x09}},
{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x01}},
{0xB0, 1, {0x0D}},
{0xB1, 1, {0x0D}},
{0xB2, 1, {0x00}},
{0xB6, 1, {0x34}},
{0xB7, 1, {0x44}},
{0xB8, 1, {0x24}},
{0xB9, 1, {0x34}},
{0xBA, 1, {0x24}},
{0xBC, 3, {0x00, 0x98, 0x00}},
{0xBD, 3, {0x00, 0x98, 0x00}},
{0xBE, 2, {0x00, 0x62}},
{0xD1, 52, {0x00, 0x00, 0x00, 0x11, 0x00, 0x2E, 0x00, 0x48, 0x00, 0x5C, 0x00, 0x7D, 0x00, 0x99, 0x00, 0xC9, 0x00, 0xF0, 0x01, 0x2F, 0x01, 0x61, 0x01, 0xB2, 0x01, 0xF4, 0x01, 0xF6, 0x02, 0x33, 0x02, 0x75, 0x02, 0x9E, 0x02, 0xD3, 0x02, 0xF5, 0x03, 0x22, 0x03, 0x3F, 0x03, 0x5F, 0x03, 0x6F, 0x03, 0x80, 0x03, 0x8A, 0x03, 0xFF}},
{0xD2, 52, {0x00, 0x00, 0x00, 0x11, 0x00, 0x2E, 0x00, 0x48, 0x00, 0x5C, 0x00, 0x7D, 0x00, 0x99, 0x00, 0xC9, 0x00, 0xF0, 0x01, 0x2F, 0x01, 0x61, 0x01, 0xB2, 0x01, 0xF4, 0x01, 0xF6, 0x02, 0x33, 0x02, 0x75, 0x02, 0x9E, 0x02, 0xD3, 0x02, 0xF5, 0x03, 0x22, 0x03, 0x3F, 0x03, 0x5F, 0x03, 0x6F, 0x03, 0x80, 0x03, 0x8A, 0x03, 0xFF}},
{0xD3, 52, {0x00, 0x00, 0x00, 0x11, 0x00, 0x2E, 0x00, 0x48, 0x00, 0x5C, 0x00, 0x7D, 0x00, 0x99, 0x00, 0xC9, 0x00, 0xF0, 0x01, 0x2F, 0x01, 0x61, 0x01, 0xB2, 0x01, 0xF4, 0x01, 0xF6, 0x02, 0x33, 0x02, 0x75, 0x02, 0x9E, 0x02, 0xD3, 0x02, 0xF5, 0x03, 0x22, 0x03, 0x3F, 0x03, 0x5F, 0x03, 0x6F, 0x03, 0x80, 0x03, 0x8A, 0x03, 0xFF}},
{0xD4, 52, {0x00, 0x00, 0x00, 0x11, 0x00, 0x2E, 0x00, 0x48, 0x00, 0x5C, 0x00, 0x7D, 0x00, 0x99, 0x00, 0xC9, 0x00, 0xF0, 0x01, 0x2F, 0x01, 0x61, 0x01, 0xB2, 0x01, 0xF4, 0x01, 0xF6, 0x02, 0x33, 0x02, 0x75, 0x02, 0x9E, 0x02, 0xD3, 0x02, 0xF5, 0x03, 0x22, 0x03, 0x3F, 0x03, 0x5F, 0x03, 0x6F, 0x03, 0x80, 0x03, 0x8A, 0x03, 0xFF}},
{0xD5, 52, {0x00, 0x00, 0x00, 0x11, 0x00, 0x2E, 0x00, 0x48, 0x00, 0x5C, 0x00, 0x7D, 0x00, 0x99, 0x00, 0xC9, 0x00, 0xF0, 0x01, 0x2F, 0x01, 0x61, 0x01, 0xB2, 0x01, 0xF4, 0x01, 0xF6, 0x02, 0x33, 0x02, 0x75, 0x02, 0x9E, 0x02, 0xD3, 0x02, 0xF5, 0x03, 0x22, 0x03, 0x3F, 0x03, 0x5F, 0x03, 0x6F, 0x03, 0x80, 0x03, 0x8A, 0x03, 0xFF}},
{0xD6, 52, {0x00, 0x00, 0x00, 0x11, 0x00, 0x2E, 0x00, 0x48, 0x00, 0x5C, 0x00, 0x7D, 0x00, 0x99, 0x00, 0xC9, 0x00, 0xF0, 0x01, 0x2F, 0x01, 0x61, 0x01, 0xB2, 0x01, 0xF4, 0x01, 0xF6, 0x02, 0x33, 0x02, 0x75, 0x02, 0x9E, 0x02, 0xD3, 0x02, 0xF5, 0x03, 0x22, 0x03, 0x3F, 0x03, 0x5F, 0x03, 0x6F, 0x03, 0x80, 0x03, 0x8A, 0x03, 0xFF}},
{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x03}},
{0xB0, 7, {0x03, 0x17, 0xFD, 0x55, 0x64, 0x00, 0x30}},
{0xB2, 9, {0xFF, 0x00, 0x01, 0x02, 0x10, 0x64, 0x00, 0x83, 0x04}},
{0xB3, 6, {0x5B, 0x00, 0xFF, 0x57, 0x5D, 0x03}},
{0xB4, 11, {0x01, 0x02, 0x03, 0x04, 0x00, 0x40, 0x03, 0x04, 0x46, 0x62, 0x00}},
{0xB5, 11, {0x00, 0x44, 0x01, 0x00, 0x58, 0x5D, 0x5D, 0x5D, 0x33, 0x33, 0x55}},
{0xB6, 7, {0xBC, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00}},
{0xB7, 8, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
{0xB8, 3, {0x11, 0x60, 0x00}},
{0xB9, 1, {0x82}},
{0xBA, 16, {0x0F, 0xA8, 0xEC, 0x2F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF3, 0xDF, 0x9B, 0xF1}},
{0xBB, 16, {0x3F, 0xB9, 0xDF, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xEC, 0x8A, 0xF2}},
{0xBC, 4, {0x41, 0xFF, 0xFF, 0x82}},
{0xBD, 4, {0x41, 0xFF, 0xFF, 0x82}},

{0xF0, 5, {0x55, 0xAA, 0x52, 0x08, 0x00}},
{0xB0, 2, {0x00, 0x10}},
{0xB5, 1, {0x6B}},
{0xB4, 1, {0x10}},
{0xBC, 1, {0x02}},
{0x35, 1, {0x00}},
{0x11, 1,{0x00}},
{REGFLAG_DELAY,120,{}},
{0x29,	1,	{0x00}},
{REGFLAG_DELAY,100,{}},
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
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;



	params->dsi.mode   =SYNC_EVENT_VDO_MODE;


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
	params->dsi.vertical_sync_active				= 2;
	params->dsi.vertical_backporch					= 14;
	params->dsi.vertical_frontporch					= 18;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 8;
	params->dsi.horizontal_backporch				= 16;
	params->dsi.horizontal_frontporch				= 24;
	//params->dsi.horizontal_blanking_pixel		       = 60;
	params->dsi.horizontal_active_pixel		       = FRAME_WIDTH;

    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=854;
	// Bit rate calculation
#if 0
	// Bit rate calculation
	params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4
	params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
	//params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
	params->dsi.fbk_div =12;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#else
	params->dsi.PLL_CLOCK=175;//227;//254;//254//247
#endif


} 


static void init_lcm_registers(void)
{

// hsd3.97
 unsigned int data_array[16];
    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000208;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000060f6; 
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000008;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x0000fcb1;//fc,60,06
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00023902;
    data_array[1]=0x000010b4;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00033902;
    data_array[1]=0x00006bb5;//00
    dsi_set_cmdq(data_array, 2, 1);


    data_array[0]=0x00033902;
    data_array[1]=0x030303b8;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0]=0x00043902;
    data_array[1]=0x010101BC;
    dsi_set_cmdq(data_array, 2, 1);


    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000108;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x0D0D0Db0;//0c
    dsi_set_cmdq(data_array, 2, 1);   
  
    data_array[0]=0x00043902;
    data_array[1]=0x0D0D0Db1;//0c
    dsi_set_cmdq(data_array, 2, 1);     
    
    data_array[0]=0x00043902;
    data_array[1]=0x0F0F0Fb3;//0c
    dsi_set_cmdq(data_array, 2, 1);   
    
    data_array[0]=0x00043902;
    data_array[1]=0x080808b5;//0c
    dsi_set_cmdq(data_array, 2, 1);   
  
    data_array[0]=0x00043902;
    data_array[1]=0x343434b6;
    dsi_set_cmdq(data_array, 2, 1);     
 
    data_array[0]=0x00043902;
    data_array[1]=0x343434b7;
    dsi_set_cmdq(data_array, 2, 1); 
  
    data_array[0]=0x00043902;
    data_array[1]=0x242424b8;
    dsi_set_cmdq(data_array, 2, 1);    
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434b9;
    dsi_set_cmdq(data_array, 2, 1);    
    
    data_array[0]=0x00043902;
    data_array[1]=0x242424ba;
    dsi_set_cmdq(data_array, 2, 1);   
    
    data_array[0]=0x00043902;
    data_array[1]=0x137800bc;
    dsi_set_cmdq(data_array, 2, 1);      
    
    data_array[0]=0x00043902;
    data_array[1]=0x137800bd;
    dsi_set_cmdq(data_array, 2, 1);     
 
    data_array[0]=0x00033902;
    data_array[1]=0x007000be;//60
    dsi_set_cmdq(data_array, 2, 1); 
    
   data_array[0]=0x01BF1500;//0xc5:0x0e
	dsi_set_cmdq(&data_array,1,1);	

   
//gama
	data_array[0] = 0x00353902;
	data_array[1] = 0x000000D1;//0xD1 0x00 0x00 0x00
	data_array[2] = 0x000F0001;//0x19 0x00 0x67 0x00
	data_array[3] = 0x004B002A;//0xA2 0x00 0xCC 0x01
	data_array[4] = 0x00B30082;//0x05 0x01 0x2D 0x01
	data_array[5] = 0x012E01FC;//0x61 0x01 0x85 0x01
	data_array[6] = 0x01ac0178;//0xB6 0x01 0xDA 0x02
	data_array[7] = 0x023502f8;//0x0D 0x02 0x34 0x02
	data_array[8] = 0x026a0236;//0x36 0x02 0x5A 0x02 
	data_array[9] = 0x02bc029e;//0x7F 0x02 0x94 0x02
	data_array[10] = 0x03f402df;//0xAD 0x02 0xBD 0x02
	data_array[11] = 0x03220310;//0xD1 0x02 0xDE 0x02
	data_array[12] = 0x034b033a;//0xF1 0x02 0xFF 0x03
	data_array[13] = 0x039a0363;//0x15 0x03 0x48 0x03
	data_array[14] = 0x000000ff;//0xFD
	dsi_set_cmdq(data_array, 15, 1);

//gama
	data_array[0] = 0x00353902;
	data_array[1] = 0x000000D2;//0xD1 0x00 0x00 0x00
	data_array[2] = 0x000F0001;//0x19 0x00 0x67 0x00
	data_array[3] = 0x004B002A;//0xA2 0x00 0xCC 0x01
	data_array[4] = 0x00B30082;//0x05 0x01 0x2D 0x01
	data_array[5] = 0x012E01FC;//0x61 0x01 0x85 0x01
	data_array[6] = 0x01ac0178;//0xB6 0x01 0xDA 0x02
	data_array[7] = 0x023502f8;//0x0D 0x02 0x34 0x02
	data_array[8] = 0x026a0236;//0x36 0x02 0x5A 0x02 
	data_array[9] = 0x02bc029e;//0x7F 0x02 0x94 0x02
	data_array[10] = 0x03f402df;//0xAD 0x02 0xBD 0x02
	data_array[11] = 0x03220310;//0xD1 0x02 0xDE 0x02
	data_array[12] = 0x034b033a;//0xF1 0x02 0xFF 0x03
	data_array[13] = 0x039a0363;//0x15 0x03 0x48 0x03
	data_array[14] = 0x000000ff;//0xFD
	dsi_set_cmdq(data_array, 15, 1);
//gama

	data_array[0] = 0x00353902;
	data_array[1] = 0x000000D3;//0xD1 0x00 0x00 0x00
	data_array[2] = 0x000F0001;//0x19 0x00 0x67 0x00
	data_array[3] = 0x004B002A;//0xA2 0x00 0xCC 0x01
	data_array[4] = 0x00B30082;//0x05 0x01 0x2D 0x01
	data_array[5] = 0x012E01FC;//0x61 0x01 0x85 0x01
	data_array[6] = 0x01ac0178;//0xB6 0x01 0xDA 0x02
	data_array[7] = 0x023502f8;//0x0D 0x02 0x34 0x02
	data_array[8] = 0x026a0236;//0x36 0x02 0x5A 0x02 
	data_array[9] = 0x02bc029e;//0x7F 0x02 0x94 0x02
	data_array[10] = 0x03f402df;//0xAD 0x02 0xBD 0x02
	data_array[11] = 0x03220310;//0xD1 0x02 0xDE 0x02
	data_array[12] = 0x034b033a;//0xF1 0x02 0xFF 0x03
	data_array[13] = 0x039a0363;//0x15 0x03 0x48 0x03
	data_array[14] = 0x000000ff;//0xFD
	dsi_set_cmdq(data_array, 15, 1);

//gama
	data_array[0] = 0x00353902;
	data_array[1] = 0x000000D4;//0xD1 0x00 0x00 0x00
	data_array[2] = 0x000F0001;//0x19 0x00 0x67 0x00
	data_array[3] = 0x004B002A;//0xA2 0x00 0xCC 0x01
	data_array[4] = 0x00B30082;//0x05 0x01 0x2D 0x01
	data_array[5] = 0x012E01FC;//0x61 0x01 0x85 0x01
	data_array[6] = 0x01ac0178;//0xB6 0x01 0xDA 0x02
	data_array[7] = 0x023502f8;//0x0D 0x02 0x34 0x02
	data_array[8] = 0x026a0236;//0x36 0x02 0x5A 0x02 
	data_array[9] = 0x02bc029e;//0x7F 0x02 0x94 0x02
	data_array[10] = 0x03f402df;//0xAD 0x02 0xBD 0x02
	data_array[11] = 0x03220310;//0xD1 0x02 0xDE 0x02
	data_array[12] = 0x034b033a;//0xF1 0x02 0xFF 0x03
	data_array[13] = 0x039a0363;//0x15 0x03 0x48 0x03
	data_array[14] = 0x000000ff;//0xFD
	dsi_set_cmdq(data_array, 15, 1);


//gama
	data_array[0] = 0x00353902;
	data_array[1] = 0x000000D5;//0xD1 0x00 0x00 0x00
	data_array[2] = 0x000F0001;//0x19 0x00 0x67 0x00
	data_array[3] = 0x004B002A;//0xA2 0x00 0xCC 0x01
	data_array[4] = 0x00B30082;//0x05 0x01 0x2D 0x01
	data_array[5] = 0x012E01FC;//0x61 0x01 0x85 0x01
	data_array[6] = 0x01ac0178;//0xB6 0x01 0xDA 0x02
	data_array[7] = 0x023502f8;//0x0D 0x02 0x34 0x02
	data_array[8] = 0x026a0236;//0x36 0x02 0x5A 0x02 
	data_array[9] = 0x02bc029e;//0x7F 0x02 0x94 0x02
	data_array[10] = 0x03f402df;//0xAD 0x02 0xBD 0x02
	data_array[11] = 0x03220310;//0xD1 0x02 0xDE 0x02
	data_array[12] = 0x034b033a;//0xF1 0x02 0xFF 0x03
	data_array[13] = 0x039a0363;//0x15 0x03 0x48 0x03
	data_array[14] = 0x000000ff;//0xFD
	dsi_set_cmdq(data_array, 15, 1);
	
	data_array[0] = 0x00353902;
	data_array[1] = 0x000000D6;//0xD1 0x00 0x00 0x00
	data_array[2] = 0x000F0001;//0x19 0x00 0x67 0x00
	data_array[3] = 0x004B002A;//0xA2 0x00 0xCC 0x01
	data_array[4] = 0x00B30082;//0x05 0x01 0x2D 0x01
	data_array[5] = 0x012E01FC;//0x61 0x01 0x85 0x01
	data_array[6] = 0x01ac0178;//0xB6 0x01 0xDA 0x02
	data_array[7] = 0x023502f8;//0x0D 0x02 0x34 0x02
	data_array[8] = 0x026a0236;//0x36 0x02 0x5A 0x02 
	data_array[9] = 0x02bc029e;//0x7F 0x02 0x94 0x02
	data_array[10] = 0x03f402df;//0xAD 0x02 0xBD 0x02
	data_array[11] = 0x03220310;//0xD1 0x02 0xDE 0x02
	data_array[12] = 0x034b033a;//0xF1 0x02 0xFF 0x03
	data_array[13] = 0x039a0363;//0x15 0x03 0x48 0x03
	data_array[14] = 0x000000ff;//0xFD
	dsi_set_cmdq(data_array, 15, 1);
 
 /*  
    data_array[0]=0x00063902;
    data_array[1]=0x3802D0C9;
    data_array[2]=0x00003838;
    dsi_set_cmdq(data_array, 3, 1);
*/
       
    data_array[0]=0x00350500;
    dsi_set_cmdq(data_array, 1, 1);   
    
    
    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    
    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10);

}



static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);
  //  init_lcm_registers();
push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
	printf("[erick-lk]%s\n", __func__);
#else
	printk("[erick-k]%s\n", __func__);
#endif
}


static void lcm_suspend(void)
{
 unsigned int data_array[16];
/*
#ifndef BUILD_LK
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(120);
#endif
*/
    data_array[0] = 0x00011500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(100);
    
    data_array[0]= 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);

    data_array[0]= 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);

    data_array[0]= 0x014f1500;
    dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);


}
static unsigned int lcm_compare_id(void);
static void lcm_resume(void)
{
#ifndef BUILD_LK
	
	lcm_init();
//lcm_compare_id();

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	#ifdef BUILD_LK
		printf("[erick-lk]%s\n", __func__);
	#else
		printk("[erick-k]%s\n", __func__);
	#endif
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
	unsigned char buffer[2];

	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(50);

	DSI_clk_HS_mode(1);
	MDELAY(50);
	DSI_clk_HS_mode(0);
	MDELAY(5);

	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 
	read_reg_v2(0xDB, &id0, 1);
	
#ifdef BUILD_LK
	printf("@@@@ db = 0x%08x\n", id0);
#endif

/*	
	data_array[0] = 0x00110500;		// Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);
*/
		
//*************Enable CMD2 Page1  *******************//
	data_array[0]=0x00063902;
	data_array[1]=0x52AA55F0;
	data_array[2]=0x00000108;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(10); 

	data_array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 
	
	read_reg_v2(0xC5, buffer, 2);
	id0 = buffer[0]; //we only need ID
	id1 = buffer[1]; //we test buffer 1
    id0 = (id0<<8);
	id0 += id1;
#ifdef BUILD_LK
	printf("rm68172 id = 0x%08x\n", id0);
#else
	printk("rm68172 id = 0x%08x\n", id0);
#endif	

	return (LCM_ID == id0)?1:0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_rm68172_dsi_vdo_fwvga_auo = 
{
    .name			= "hct_rm68172_dsi_vdo_fwvga_auo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,	
	.compare_id    = lcm_compare_id,	
    //.esd_check   = lcm_esd_check,	
    //.esd_recover   = lcm_esd_recover,		
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
    //.esd_check   = lcm_esd_check,	
    //.esd_recover   = lcm_esd_recover,	
    .update         = lcm_update,
#endif	//wqtao
};


