#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <string.h>
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
#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER
//#define LCM_FL11281_ID			(0x18211f)
#define LCM_ID 0x11
#define LCM_ID1  0x28
#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef BUILD_LK
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define   LCM_DSI_CMD_MODE							0
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static const struct LCM_setting_table lcm_initialization_setting[] = {

{0xb9,3,{ 0xF1,0x12,0x81}},

{0xBA,27,{0x33,0x81,0x06,0xfD,0x0e,0x0e,0x00,0x00,0x00,0x00,              
          0x00,0x00,0x00,0x00,0x44,0x25,0x00,0xB1,0x0a,0x00,	
          0x00,0x02,0x4f,0x11,0x00,0x00,0x37}},

{0Xbf,2,{0x02,0x11}}, //新增cmd                  

{0XB8,2,{0xA6,0x22}},//a5

{0xB3,17,{0x02,0x00,0x06,0x06,0x10,0x10,0x05,0x05,
          0x00,0x00,0x00,0x03,0xFF,0x00,0x00,0x00,
          0x00}},

{0xc0,9,{0x73,0x73,0x50,0x50,0x00,0x00,0x08,0x30,0x00}},

{0xbc,1,{0x46}},

{0xcc,1,{0x0b}},//03

{0XB4,1,{0x80}},

{0XB0,1,{0x02}},

{0Xb2,2,{0xc8,0x02}},

{0Xe3,10,{0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0xc0,0x14}},

{0Xb1,10,{0x21,0x50,0xE3,0x1E,0x1E,0x33,0x77,0x01,0x9B,0x0C}},//4th,5th→0x1A

{0Xb5,2,{0x09,0x09}},//09,09

{0Xb6,2,{0x34,0x34}},//10,10

{0XE9,63,{0x02,0x00,0x0F,0x05,0x13,0x0A,0x70,0x12,0x31,0x23,
          0x37,0x12,0x0A,0x70,0x27,0x38,0x03,0x00,0x03,0x00,            
          0x00,0x00,0x03,0x00,0x03,0x00,0x00,0x00,0x75,0x75, 
          0x31,0x88,0x88,0x88,0x88,0x88,0x13,0x88,0x88,0x64, 
          0x64,0x20,0x88,0x88,0x88,0x88,0x88,0x02,0x88,0x88,
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,                 
          0x00,0x00,0x00}},//   14th-->a0

{0XEa,48,{0x02,0x21,0x00,0x00,0x02,0x46,0x02,0x88,0x88,0x88,
          0x88,0x88,0x64,0x88,0x88,0x13,0x57,0x13,0x88,0x88,
          0x57,0x02,0x31,0x75,0x88,0x88,0x00,0x1a,0x00,0x00,               
          0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0XE0,34,{0x00,0x13,0x1d,0x37,0x3a,0x3F,0x56,0x45,0x08,0x0d,0x0d,0x10,0x12,0x0f,0x11,0x11,0x19,
          0x00,0x13,0x1d,0x37,0x3a,0x3F,0x56,0x45,0x08,0x0d,0x0d,0x10,0x12,0x0f,0x11,0x11,0x19}},


{0x11, 1 ,{0x00}}, 
{REGFLAG_DELAY, 150, {}},

{0x29, 1 ,{0x00}}, 
{REGFLAG_DELAY, 100, {}},

//{0x2C, 1 ,{0x00}}, 
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
                MDELAY(1);

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

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.vertical_sync_active				= 4;// 3    2
		params->dsi.vertical_backporch					= 18;// 20   1
		params->dsi.vertical_frontporch					= 18; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 50;// 50  2
		params->dsi.horizontal_backporch				= 150;//50
		params->dsi.horizontal_frontporch				= 150;//50
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8; 

		// Bit rate calculation
		params->dsi.PLL_CLOCK = 248;//220  190   //255
		//1 Every lane speed
		//params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4	
		//params->dsi.fbk_div =17;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
			params->dsi.esd_check_enable = 1;
			params->dsi.customization_esd_check_enable = 1;

          		params->dsi.lcm_esd_check_table[0].cmd          = 0x09;
			params->dsi.lcm_esd_check_table[0].count        = 1;
			params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
		


}

static void lcm_init(void)
{
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
};

static void lcm_suspend(void)
{


	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	
	SET_RESET_PIN(0);
	MDELAY(1); // 1ms

}

//static int vcom = 0x46;
static void lcm_resume(void)
{/*lcm_initialization_setting[13].para_list[0] = vcom;//paralist[]//from left to right {0x39,0xB6,4,{0x00,0x91,0x00,0x91}},0x91 =paralist[1]
 lcm_initialization_setting[13].para_list[1] = vcom;	
 */
          lcm_init();
  /* #if defined(BUILD_LK)
	printf("vcom = %x\n",vcom);
      #else
	printk("vcom = %x\n",vcom);
      #endif
	vcom += 2;
   */	
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

#define LCM_ID_FL11281 (0x21)    //M
static unsigned int lcm_compare_id(void)
{
unsigned int id=0,id1=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
        MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120); 

	array[0]=0x00043902;
	array[1]=0x8112f1B9;// page enable
	dsi_set_cmdq(array, 2, 1);

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);


	read_reg_v2(0x04, buffer, 2);
	id  =  buffer[0];
	id1 =  buffer[1];
	/*
	array[0] = 0x00013700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);
	read_reg_v2(0xDB, buffer, 1);
	id1 =  buffer[0];
	*/
#ifdef BUILD_LK
	dprintf("FL11281 id = 0x%08x id1=%x \n",  id,id1);
#else
	//printk("FL11281 id = 0x%08x  id1=%x \n",id,id1);
#endif
	//return 1;	
	return (LCM_ID_FL11281 == id1)?1:0;//id1

}

static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];
		printk("lifan 5");
	/*if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}*/

	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0A, buffer, 2);
		printk("lifan 3");	
	if(buffer[0]==0x9C)
	{
	printk("lifan 1");
		return TRUE;
	}
	else
	{
           printk("lifan 2");			 
		return FALSE;
	}
#else
	return FALSE;
#endif

}
/*
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
		printk("lifan 1");
	if(buffer[0]==0x90)
	{
printk("lifan 2");
		return FALSE;
	}
	else
	{
printk("lifan 3");
		return TRUE;

	}
#else
	return FALSE;
#endif
*/



static unsigned int lcm_esd_recover(void)
{
  #ifndef BUILD_LK
	lcm_init();
	lcm_resume();

	printk("lifan 4");

	return TRUE;
#endif
}



LCM_DRIVER hct_fl11281_dsi_vdo_hd_ivo = 
{
    	.name	= "hct_fl11281_dsi_vdo_hd_ivo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
        .compare_id    	= lcm_compare_id,
//	.esd_check      = lcm_esd_check,
//	.esd_recover    = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE) 
    .update         = lcm_update,
#endif
    };
