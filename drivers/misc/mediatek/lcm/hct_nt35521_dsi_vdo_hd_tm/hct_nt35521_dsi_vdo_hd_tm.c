
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
#endif

#include "lcm_drv.h"
//yufeng
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

#define FRAME_WIDTH                                          (720)
#define FRAME_HEIGHT                                         (1280)

#define REGFLAG_DELAY                                         0XFE
#define REGFLAG_END_OF_TABLE                                  0xFF   // END OF REGISTERS MARKER

#define LCM_ID_NT35521  0x5521
#define LCM_DSI_CMD_MODE                                    0

//HQ_fujin 131104
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                    lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg                                            lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

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
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
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
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;


		params->dsi.vertical_sync_active				= 4;// 3    2
		params->dsi.vertical_backporch					= 38;// 20   1
		params->dsi.vertical_frontporch					= 40; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 10;// 50  2
		params->dsi.horizontal_backporch				= 72;
		params->dsi.horizontal_frontporch				= 72 ;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8;

		// Bit rate calculation
		//1 Every lane speed
        	//params->dsi.pll_select=1;
	    params->dsi.PLL_CLOCK = 240; //this value must be in MTK suggested table
    params->dsi.compatibility_for_nvk = 1;
}

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

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xFF,4,{0xAA,0x55,0xA5,0x80}},

//{0x6F,1,{0x07}},
//{0xF7,1,{0x60}},
{0x6F,2,{0x11,0x00}},
{0xF7,2,{0x20,0x00}},

{0x6F,1,{0x1E}},
{0xFA,1,{0x20}},

{0x6F,1,{0x1E}},
{0xFA,1,{0x00}},

{0x6F,1,{0x1F}},
{0xFA,1,{0x00}},

{0x6F,1,{0x11}},
{0xF3,1,{0x01}},


{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
{0xBD,5,{0x01,0xA0,0x0C,0x08,0x01}},
{0x6F,1,{0x02}},
{0xB8,1,{0x01}}, //
{0xB1,2,{0x68,0x21}},
{0xBB,2,{0x11,0x11}},
{0xBC,2,{0x00,0x00}},
{0xB6,1,{0x01}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
{0xB0,2,{0x09,0x09}},
{0xB1,2,{0x09,0x09}},
{0xBC,2,{0x68,0x00}},
{0xBD,2,{0x68,0x00}},
{0xCA,1,{0x00}},
{0xC0,1,{0x04}},
{0xB5,2,{0x03,0x03}},
//{0xBE,1,{0x53}},  //4D~58 VCOM SETTING,After OTP,is not used
{0xB3,2,{0x19,0x19}},
{0xB4,2,{0x0F,0x0F}},
{0xB9,2,{0x26,0x26}},

//modified by Frank 20130528
{0xBA,2,{0x14,0x14}},

//---------------
//page 2   gamma

{0xF0,5,{0x55,0xAA,0x52,0x08,0x02}},

{0xEE,1,{0x01}},

//gamma 2.5
{0xB0,16,{0x00,0x00,0x00,0x58,0x00,0x88,0x00,0xA7,0x00,0xC0,0x00,0xE8,0x01,0x08,0x01,0x3C}},
 
{0xB1,16,{0x01,0x63,0x01,0xA4,0x01,0xD6,0x02,0x26,0x02,0x67,0x02,0x69,0x02,0xA5,0x02,0xE3}},
 
{0xB2,16,{0x03,0x09,0x03,0x3A,0x03,0x5E,0x03,0x8A,0x03,0xA5,0x03,0xC4,0x03,0xD8,0x03,0xED}},
 
{0xB3,4,{0x03,0xF7,0x03,0xFB}},

/*
//gamma2.2
{0xB0,16,{0x00,0x00,0x00,0x2F,0x00,0x64,0x00,0x83,0x00,0xA5,0x00,0xDF,0x00,0xFA,0x01,0x26}},

{0xB1,16,{0x01,0x4F,0x01,0x86,0x01,0xBB,0x02,0x05,0x02,0x3C,0x02,0x3C,0x02,0x75,0x02,0xAD}},

{0xB2,16,{0x02,0xC9,0x02,0xF3,0x03,0x0F,0x03,0x34,0x03,0x49,0x03,0x5E,0x03,0x6A,0x03,0x90}},

{0xB3,4,{0x03,0xF5,0x03,0xFF}},
*/
{0x6F,1,{0x02}},

{0xF7,1,{0x47}},


{0x6F,1,{0x0A}},

{0xF7,1,{0x02}},


{0x6F,1,{0x17}},

{0xF4,1,{0x70}},


{0x6F,1,{0x11}},

{0xF3,1,{0x01}},

//--------------
//page 6	 timing 

{0xF0,5,{0x55,0xAA,0x52,0x08,0x06}},

{0xB0,2,{0x12,0x10}},

{0xB1,2,{0x18,0x16}},

{0xB2,2,{0x00,0x02}},

{0xB3,2,{0x31,0x31}},

{0xB4,2,{0x31,0x34}},

{0xB5,2,{0x34,0x31}},

{0xB6,2,{0x31,0x33}}, 

{0xB7,2,{0x33,0x33}},

{0xB8,2,{0x31,0x08}},

{0xB9,2,{0x2E,0x2D}},

{0xBA,2,{0x2D,0x2E}},

{0xBB,2,{0x09,0x31}},

{0xBC,2,{0x33,0x33}},

{0xBD,2,{0x33,0x31}},

{0xBE,2,{0x31,0x34}},

{0xBF,2,{0x34,0x31}},

{0xC0,2,{0x31,0x31}},

{0xC1,2,{0x03,0x01}},

{0xC2,2,{0x17,0x19}},

{0xC3,2,{0x11,0x13}},

{0xE5,2,{0x31,0x31}},


{0xC4,2,{0x17,0x19}},

{0xC5,2,{0x11,0x13}},

{0xC6,2,{0x03,0x01}},

{0xC7,2,{0x31,0x31}},

{0xC8,2,{0x31,0x34}},

{0xC9,2,{0x34,0x31}},

{0xCA,2,{0x31,0x33}},

{0xCB,2,{0x33,0x33}},

{0xCC,2,{0x31,0x09}},

{0xCD,2,{0x2D,0x2E}},

{0xCE,2,{0x2E,0x2D}},

{0xCF,2,{0x08,0x31}},

{0xD0,2,{0x33,0x33}},

{0xD1,2,{0x33,0x31}},

{0xD2,2,{0x31,0x34}},

{0xD3,2,{0x34,0x31}},

{0xD4,2,{0x31,0x31}},

{0xD5,2,{0x00,0x02}},

{0xD6,2,{0x12,0x10}},

{0xD7,2,{0x18,0x16}},


{0xD8,5,{0x00,0x00,0x00,0x00,0x00}},

{0xD9,5,{0x00,0x00,0x00,0x00,0x00}},
  
{0xE7,1,{0x00}},

//---------
//page 5	  timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xED,1,{0x30}},

//---------
//page 3	  timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},

{0xB1,2,{0x00,0x00}},

{0xB0,2,{0x00,0x00}},

//---------
//page 5	   timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xE5,1,{0x00}},


{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xB0,2,{0x17,0x06}},

{0xB8,1,{0x00}},


{0xBD,5,{0x03,0x03,0x01,0x00,0x03}},

{0xB1,2,{0x17,0x06}},

{0xB9,2,{0x00,0x03}},

{0xB2,2,{0x17,0x06}},

{0xBA,2,{0x00,0x00}},

{0xB3,2,{0x17,0x06}},

{0xBB,2,{0x00,0x00}},

{0xB4,2,{0x17,0x06}},

{0xB5,2,{0x17,0x06}},

{0xB6,2,{0x17,0x06}},

{0xB7,2,{0x17,0x06}},

{0xBC,2,{0x00,0x03}},

{0xE5,1,{0x06}},

{0xE6,1,{0x06}},

{0xE7,1,{0x06}},

{0xE8,1,{0x06}},

{0xE9,1,{0x06}},

{0xEA,1,{0x06}},

{0xEB,1,{0x06}},

{0xEC,1,{0x06}},

//page 5		timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xC0,1,{0x0B}},	   ///

{0xC1,1,{0x09}},	   //

{0xC2,1,{0x0B}},	   //

{0xC3,1,{0x09}},	   //

//page 3		 timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},

{0xB2,5,{0x05,0x00,0x00,0x00,0x90}}, //STV falling edge

{0xB3,5,{0x05,0x00,0x00,0x00,0x90}},

{0xB4,5,{0x05,0x00,0x00,0x00,0x90}},

{0xB5,5,{0x05,0x00,0x00,0x00,0x90}},


 //page 5		 timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xC4,1,{0x10}},

{0xC5,1,{0x10}},

{0xC6,1,{0x10}},

{0xC7,1,{0x10}},

 //page 3		 timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},

{0xB6,5,{0x05,0x00,0x00,0x00,0x00}}, //RST falling edge

{0xB7,5,{0x05,0x00,0x00,0x00,0x00}},

{0xB8,5,{0x05,0x00,0x00,0x00,0x00}},

{0xB9,5,{0x05,0x00,0x00,0x00,0x00}},

 //page 5		 timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xC8,2,{0x07,0x20}},

{0xC9,2,{0x03,0x20}},

{0xCA,2,{0x07,0x00}},

{0xCB,2,{0x03,0x00}},

 //page 3		  timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x03}},

{0xBA,5,{0x44,0x00,0x00,0x00,0x70}},   //GCK falling edge  70~90

{0xBB,5,{0x44,0x00,0x00,0x00,0x70}},//53  

{0xBC,5,{0x44,0x00,0x00,0x00,0x70}},//53

{0xBD,5,{0x44,0x00,0x00,0x00,0x70}},//53


//page 5		   timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xD1,5,{0x00,0x05,0x00,0x07,0x10}},

{0xD2,5,{0x00,0x05,0x04,0x07,0x10}},

{0xD3,5,{0x00,0x00,0x0A,0x07,0x10}},

{0xD4,5,{0x00,0x00,0x0A,0x07,0x10}},


//page 5			   timing

{0xF0,5,{0x55,0xAA,0x52,0x08,0x05}},

{0xD0,7,{0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0xD5,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0xD6,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0xD7,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0xD8,5,{0x00,0x00,0x00,0x00,0x00}},

 //page 3				timing
{0xF0,5,{ 0x55,0xAA,0x52,0x08,0x03}},
{0xC4,1,{ 0x60}},
{0xC5,1,{ 0x40}},
{0xC6,1,{ 0x60}},
{0xC7,1,{ 0x40}},

{0x6F,1,{0x01}},
{0xF9,1,{0x46}},

{0x35,1,{0x00}},
{0x55,1,{0x90}},

{0x11,1,{0x00}},
{REGFLAG_DELAY, 120, {}},
// Display ON
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 50, {}},

{REGFLAG_END_OF_TABLE, 0x00, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
};

static void lcm_init(void)
{
    unsigned int data_array[16];   
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);
   // lcm_initialization();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

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
    dsi_set_cmdq(&data_array, 3, 1);
    //MDELAY(1);
   
    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(&data_array, 3, 1);
    //MDELAY(1);
   
    data_array[0]= 0x00290508;
    dsi_set_cmdq(&data_array, 1, 1);
    //MDELAY(1);
   
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(&data_array, 1, 0);
    //MDELAY(1);

}

static void lcm_suspend(void)
{
    unsigned int data_array[16];
    data_array[0]=0x00280500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(10);
    data_array[0]=0x00100500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(100);
}


static void lcm_resume(void)
{
    unsigned int data_array[16];
    data_array[0]=0x00110500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(100);
    data_array[0]=0x00290500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(10);
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id=0;
    unsigned char buffer[3];
    unsigned int array[16]; 
    unsigned int data_array[16];

    SET_RESET_PIN(1);
        MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
   
    SET_RESET_PIN(1);
    MDELAY(120);

    data_array[0] = 0x00063902;
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000108;               
    dsi_set_cmdq(&data_array, 3, 1);

    array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
   
    read_reg_v2(0xC5, buffer, 3);
    id = buffer[1]; //we only need ID
    #ifdef BUILD_LK
        printf("%s, LK nt35590 debug: nt35590 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
    #else
        printk("%s, LK nt35590 debug: nt35590 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
    #endif

   // if(id == LCM_ID_NT35521)
    if(buffer[0]==0x55 && buffer[1]==0x21)
        return 1;
    else
        return 0;


}




static int err_count = 0;

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    unsigned char buffer[8] = {0};
    unsigned int array[4];
    int i =0;

    array[0] = 0x00013700;   
    dsi_set_cmdq(array, 1,1);
    read_reg_v2(0x0A, buffer,8);

    printk( "nt35521_JDI lcm_esd_check: buffer[0] = %d,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

    if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/
    {
        printk( "nt35521_JDI lcm_esd_check buffer[0] = %d\n",buffer[0]);
        return TRUE;
    }
    else
    {
        if(buffer[3] != 0x02) //error data type is 0x02
        {
             //return FALSE;
        err_count = 0;
        }
        else
        {
             //if(((buffer[4] != 0) && (buffer[4] != 0x40)) ||  (buffer[5] != 0x80))
        if( (buffer[4] == 0x40) || (buffer[5] == 0x80))
             {
                  err_count = 0;
             }
             else
             {
                  err_count++;
             }            
             if(err_count >=2 )
             {
                 err_count = 0;
                 printk( "nt35521_JDI lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);
                 return TRUE;
             }
        }
        return FALSE;
    }
#endif
   
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();
    return TRUE;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_nt35521_dsi_vdo_hd_tm= 
{
    .name			= "hct_nt35521_dsi_vdo_hd_tm",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id    = lcm_compare_id,
#if 0//defined(LCM_DSI_CMD_MODE)
//    .set_backlight	= lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .update         = lcm_update
#endif
};

