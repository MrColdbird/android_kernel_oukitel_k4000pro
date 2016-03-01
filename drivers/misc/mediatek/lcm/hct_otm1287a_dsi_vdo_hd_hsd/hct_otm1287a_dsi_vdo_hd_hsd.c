
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

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define LCM_ID_OTM1287 (0x1287)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

//HQ_fujin 131104
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define   LCM_DSI_CMD_MODE							0

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
		params->dsi.horizontal_backporch				= 85;
		params->dsi.horizontal_frontporch				= 85 ;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8;

		// Bit rate calculation
		//1 Every lane speed
        	//params->dsi.pll_select=1;
	    params->dsi.PLL_CLOCK = 220; //this value must be in MTK suggested table}
}


static struct LCM_setting_table lcm_initialization_setting[] = 
{


        {0x00,1,{0x00}},
        {0xff,3,{0x12,0x87,0x01}},	//EXTC=1
                  
        {0x00,1,{0x80}},	        //Orise mode enable
        {0xff,2,{0x12,0x87}},


        {0x00,1,{0x92}},
        {0xFF,2,{0x30,0x02}},
                  

        {0x00,1,{0x80}},             //TCON Setting
        {0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},
                  
        {0x00,1,{0x90}},             //,{anel Timing Setting
        {0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},
                  
        {0x00,1,{0xb3}},             //Interval Scan Frame: 0 frame, column inversion
        {0xc0,2,{0x00,0x55}},
                  
        {0x00,1,{0x81}},             //frame rate:60Hz
        {0xc1,1,{0x55}},


        {0x00,1,{0xa0}},             //dcdc setting
        {0xc4,14,{0x05,0x10,0x06,0x02,0x05,0x15,0x10,0x05,0x10,0x07,0x02,0x05,0x15,0x10}},

        {0x00,1,{0xb0}},             //clam,{ voltage setting
        {0xc4,2,{0x00,0x00}},
                  
        {0x00,1,{0x91}},             //VGH=15V, VGL=-10V, ,{um,{ ratio:VGH=6x, VGL=-5x
        {0xc5,2,{0x46,0x42}},
                  
        {0x00,1,{0x00}},             //GVDD=5.008V, NGVDD=-5.008V
        {0xd8,2,{0xc7,0xc7}},
                  
        {0x00,1,{0x00}},   
        {0xd9,1,{0x5a}},
                  
        {0x00,1,{0xb3}},             //VDD_18V=1.7V, LVDSVDD=1.6V
        {0xc5,1,{0x84}},
                  
        {0x00,1,{0xbb}},             //LVD voltage level setting
        {0xc5,1,{0x8a}},
                  
        {0x00,1,{0x82}},             //cho,{,{er
        {0xc4,1,{0x0a}},
                  
        {0x00,1,{0xc6}},		//debounce 
        {0xB0,1,{0x03}}, 



        {0x00,1,{0x00}},             //ID1
        {0xd0,1,{0x40}},
                  
        {0x00,1,{0x00}},             //ID2, ID3
        {0xd1,2,{0x00,0x00}},


        {0x00,1,{0x80}}, 
        {0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0x90}}, 
        {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xa0}}, 
        {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xb0}},  
        {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xc0}}, 
        {0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x05,0x05}},

        {0x00,1,{0xd0}}, 
        {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x00}},

        {0x00,1,{0xe0}},
        {0xcb,14,{0x00,0x00,0x05,0x05,0x00,0x05,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xf0}},
        {0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},


        {0x00,1,{0x80}}, 
        {0xcc,15,{0x0E,0x10,0x0A,0x0C,0x02,0x04,0x00,0x00,0x00,0x00,0x2E,0x2D,0x00,0x29,0x2A}},

        {0x00,1,{0x90}},    
        {0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0D,0x0F,0x09,0x0B,0x01,0x03,0x00,0x00}},

        {0x00,1,{0xa0}}, 
        {0xcc,14,{0x00,0x00,0x2E,0x2D,0x00,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xb0}}, 
        {0xcc,15,{0x0B,0x09,0x0F,0x0D,0x03,0x01,0x00,0x00,0x00,0x00,0x2D,0x2E,0x00,0x29,0x2A}},

        {0x00,1,{0xc0}}, 
        {0xcc,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x0A,0x10,0x0E,0x04,0x02,0x00,0x00}},

        {0x00,1,{0xd0}},  
        {0xcc,14,{0x00,0x00,0x2D,0x2E,0x00,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},


        {0x00,1,{0x80}},             //,{anel VST setting
        {0xce,12,{0x8B,0x03,0x18,0x8A,0x03,0x18,0x89,0x03,0x18,0x88,0x03,0x18}},

        {0x00,1,{0x90}},             //,{anel VEND setting
        {0xce,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xa0}},             //,{anel CLKA1/2 setting
        {0xce,14,{0x38,0x07,0x05,0x00,0x00,0x18,0x00,0x38,0x06,0x05,0x01,0x00,0x18,0x00}},

        {0x00,1,{0xb0}},             //,{anel CLKA3/4 setting
        {0xce,14,{0x38,0x05,0x05,0x02,0x00,0x18,0x00,0x38,0x04,0x05,0x03,0x00,0x18,0x00}},

        {0x00,1,{0xc0}},             //,{anel CLKb1/2 setting
        {0xce,14,{0x38,0x03,0x05,0x04,0x00,0x18,0x00,0x38,0x02,0x05,0x05,0x00,0x18,0x00}},

        {0x00,1,{0xd0}},             //,{anel CLKb3/4 setting
        {0xce,14,{0x38,0x01,0x05,0x06,0x00,0x18,0x00,0x38,0x00,0x05,0x07,0x00,0x18,0x00}},

        {0x00,1,{0x80}},             //,{anel CLKc1/2 setting
        {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0x90}},             //,{anel CLKc3/4 setting
        {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xa0}},             //,{anel CLKd1/2 setting
        {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xb0}},             //,{anel CLKd3/4 setting
        {0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

        {0x00,1,{0xc0}},             //,{anel ECLK setting
        {0xcf,11,{0x3d,0x02,0x15,0x20,0x00,0x00,0x01,0x81,0x00,0x03,0x08}}, //gate ,{re. ena.

        {0x00,1,{0xb5}},             //TCON_GOA_OUT Setting
        {0xc5,6,{0x00,0x6f,0xfF,0x00,0x6f,0xfF}},



        {0x00,1,{0x90}},             //Mode-3
        {0xf5,4,{0x02,0x11,0x02,0x15}},
                  
        {0x00,1,{0x90}},             //2xV,{NL
        {0xc5,1,{0x50}},
                  
        {0x00,1,{0x94}},             //Freq.
        {0xc5,1,{0x66}},//55
                  

        {0x00,1,{0xb2}},             //VGLO1
        {0xf5,2,{0x00,0x00}},
                  
                  
        {0x00,1,{0xb6}},             //VGLO2
        {0xf5,2,{0x00,0x00}},
                  
                  
        {0x00,1,{0x94}},             //VCL ,{um,{ dis
        {0xf5,2,{0x00,0x00}},
                  
        {0x00,1,{0xd2}},             //VCL reg. en
        {0xf5,2,{0x06,0x15}},
                  
        {0x00,1,{0xb4}},             //VGLO1/2 ,{ull low setting
        {0xc5,1,{0xcc}},		//d[7] vglo1 d[6] vglo2 => 0: ,{ull vss, 1: ,{ull vgl
                  
        {0x00,1,{0x00}},                                                                                    ////
        {0xE1,20,{0x00,0x12,0x21,0x2c,0x3d,0x4a,0x4d,0x7a,0x6b,0x85,0x7d,0x66,0x76,0x4d,0x4a,0x39,0x28,0x1a,0x0f,0x00}},

        {0x00,1,{0x00}},
        {0xE2,20,{0x00,0x12,0x21,0x2c,0x3d,0x4a,0x4d,0x7a,0x6b,0x85,0x7d,0x66,0x76,0x4d,0x4a,0x39,0x28,0x1a,0x0f,0x00}},

        {0x00,1,{0x00}},             //Orise mode disable
        {0xff,3,{0xff,0xff,0xff}},


        {0x11,1,{0x00}},//SLEEP OUT
        {REGFLAG_DELAY,120,{}},
                                     				                                                                            
        {0x29,1,{0x00}},//Display ON 
        {REGFLAG_DELAY,20,{}},	
        {REGFLAG_END_OF_TABLE, 0x00, {}}
       	// Note
       	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


       	// Setting ending by predefined flag
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

static void lcm_init(void)
{
    unsigned int data_array[16];   
    SET_RESET_PIN(1);
	MDELAY(2); 
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
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(50);

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
    lcm_init();
  /*  unsigned int data_array[16];
    data_array[0]=0x00110500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(100);
    data_array[0]=0x00290500;
    dsi_set_cmdq(&data_array,1,1);
    MDELAY(10);*/
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id=0;
    unsigned char buffer[5],id_high,id_low;
    unsigned int array[16]; 
    unsigned int data_array[16];

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
   
    SET_RESET_PIN(1);
    MDELAY(120);


    array[0] = 0x00053700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xa1, buffer, 5);

    id_high = buffer[2];
    id_low = buffer[3];
    id = ((id_high&0xff)<<8) | id_low;

    #ifdef BUILD_LK
        printf("%s, LK otm1287a debug: otm1287a id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
    #else
        printk("%s, LK otm1287a debug: otm1287a id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
    #endif

    if(id == LCM_ID_OTM1287)
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

    printk( "otm1287_JDI lcm_esd_check: buffer[0] = %d,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

    if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/
    {
        printk( "otm1287_JDI lcm_esd_check buffer[0] = %d\n",buffer[0]);
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
                 printk( "otm1287_JDI lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);
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
    lcm_resume();

    return TRUE;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_otm1287a_dsi_vdo_hd_hsd= 
{
    .name	    = "hct_otm1287a_dsi_vdo_hd_hsd",
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

