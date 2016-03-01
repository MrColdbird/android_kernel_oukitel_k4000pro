#ifdef BUILD_LK
#include "mt_gpio.h"
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#include "mach/mt_gpio.h"
#endif
#include "lcm_drv.h"
//#include "gpio_const.h"

#include "cust_gpio_usage.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM8019A									0x8019

//#define LCM_DSI_CMD_MODE

//#define LCM_DEBUG
#if defined(BUILD_LK)
	#if defined(BUILD_LK)
	#define LCM_LOG(fmt, args...)    printf(fmt, ##args)
	#else
	#define LCM_LOG(fmt, args...)    printk(fmt, ##args)	
	#endif
#else
#define LCM_LOG(fmt, args...)	 printk(fmt, ##args)	
#endif

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)  (lcm_util.set_gpio_out((n), (v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


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
      {0x00, 1, {0x00}},
	{0xff, 3, {0x80,0x19,0x01}},

	{0x00, 1, {0x80}},
	{0xff, 2, {0x80,0x19}},

	{0x00, 1, {0xb4}},
	{0xc0, 1, {0x00}},

	{0x00, 1, {0x89}},
	{0xc4, 1, {0x08}},

	{0x00, 1, {0x82}},
	{0xc5, 1, {0xb0}},

	{0x00, 1, {0x90}},
	{0xc5, 2, {0x4e,0x79}},//78

	{0x00, 1, {0x00}},
	{0xd8, 2, {0x6f,0x6f}},

	{0x00, 1, {0x00}},
	{0xd9, 1, {0x40}},//44

	{0x00, 1, {0x81}},
	{0xc4, 1, {0x83}},

	{0x00, 1, {0x92}},
	{0xc5, 1, {0x01}},

	{0x00, 1, {0xb1}},
	{0xc5, 1, {0xa9}},

	{0x00, 1, {0xc0}},
	{0xc5, 1, {0x00}},

	{0x00, 1, {0x93}},
	{0xc5, 1, {0x03}},

	{0x00, 1, {0x90}},
	{0xb3, 1, {0x02}},

	{0x00, 1, {0x92}},
	{0xb3, 1, {0x45}},

	{0x00, 1, {0x80}},
	{0xc0, 9, {0x00,0x58,0x00,0x15,0x15,0x00,0x58,0x15,0x15}},

	{0x00, 1, {0x90}},
	{0xc0, 6, {0x00,0x15,0x00,0x00,0x00,0x03}},

	{0x00, 1, {0x80}},
	{0xce, 12, {0x8b,0x03,0x00,0x8a,0x03,0x00,0x89,0x03,0x00,0x88,0x03,0x00}},

	{0x00, 1, {0x90}},
	{0xce, 12, {0x38,0x0a,0x00,0x38,0x0b,0x00,0x38,0x0c,0x00,0x38,0x0d,0x00}},

	{0x00, 1, {0xa0}},
	{0xce, 14, {0x38,0x07,0x03,0x54,0x00,0x00,0x00,0x38,0x06,0x03,0x55,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xce, 14, {0x38,0x05,0x03,0x56,0x00,0x00,0x00,0x38,0x04,0x03,0x57,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xce, 14, {0x38,0x03,0x03,0x58,0x00,0x00,0x00,0x38,0x02,0x03,0x59,0x00,0x00,0x00}},

	{0x00, 1, {0xd0}},
	{0xce, 14, {0x38,0x09,0x03,0x5a,0x00,0x00,0x00,0x38,0x08,0x03,0x5b,0x00,0x00,0x00}},

	{0x00, 1, {0xc7}},
	{0xcf, 1, {0x00}},

	{0x00, 1, {0xc0}},
	{0xcb, 15, {0x01,0x01,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xd0}},
	{0xcb, 1, {0x00}},

	{0x00, 1, {0xd5}},
	{0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01}},

	{0x00, 1, {0xe0}},
	{0xcb, 6, {0x01,0x01,0x00,0x00,0x01,0x01}},

	{0x00, 1, {0x80}},
	{0xcc, 10, {0x26,0x25,0x00,0x00,0x0c,0x0a,0x10,0x0e,0x02,0x04}},

	{0x00, 1, {0x90}},
	{0xcc, 6, {0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0x9a}},
	{0xcc, 5, {0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xcc, 11, {0x00,0x03,0x01,0x0d,0x0f,0x09,0x0b,0x00,0x00,0x25,0x26}},

	{0x00, 1, {0xb0}},
	{0xcc, 10, {0x25,0x26,0x00,0x00,0x0b,0x0d,0x0f,0x09,0x06,0x08}},

	{0x00, 1, {0xc0}},
	{0xcc, 6, {0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xca}},
	{0xcc, 5, {0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xd0}},
	{0xcc, 11, {0x00,0x07,0x05,0x0a,0x10,0x0e,0x0c,0x00,0x00,0x26,0x25}},

	{0x00, 1, {0x00}},
	{0xe1, 20, {0x20,0x40,0x50,0x5c,0x6d,0x79,0x7c,0xa2,0x8f,0xa5,0x60,0x4e,0x63,0x49,0x4d,0x43,0x3e,0x3a,0x36,0x30}},

	{0x00, 1, {0x00}},
	{0xe2, 20, {0x20,0x40,0x50,0x5c,0x6d,0x79,0x7c,0xa2,0x8f,0xa6,0x60,0x4d,0x63,0x49,0x4d,0x43,0x3e,0x3a,0x36,0x30}},

	{0x00, 1, {0x80}},
	{0xc4, 1, {0x30}},

	{0x00, 1, {0x98}},
	{0xc0, 1, {0x00}},

	{0x00, 1, {0xa9}},
	{0xc0, 1, {0x0a}},

	{0x00, 1, {0xb0}},
	{0xc1, 3, {0x20,0x00,0x00}},

	{0x00, 1, {0xe1}},
	{0xc0, 2, {0x40,0x30}},

	{0x00, 1, {0x80}},
	{0xc1, 2, {0x03,0x33}},

	{0x00, 1, {0xa0}},
	{0xc1, 1, {0xe8}},

	{0x00, 1, {0x90}},
	{0xb6, 1, {0xb4}},

	{0x00, 1, {0x00}},
	{0xfb, 1, {0x01}},

	{0x00, 1, {0x00}},
	{0xff, 3, {0xff,0xff,0xff}},
  
       {0x11,1,{0x00}},//SLEEP OUT
       {REGFLAG_DELAY,120,{}},
                                 				                                                                                
       {0x29,1,{0x00}},//Display ON 
       {REGFLAG_DELAY,30,{}},

	
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};




static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 200, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{0x53, 1, {0x24}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned int cmd;
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
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if defined(LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE;
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	

		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
	
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;
		
		params->dsi.intermediat_buffer_num = 2;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577

		params->dsi.vertical_active_line=854;
		params->dsi.vertical_sync_active				= 4;
		params->dsi.vertical_backporch					= 16;
		params->dsi.vertical_frontporch					= 15;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 64;
		params->dsi.horizontal_frontporch				= 64;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
#if 0	
		// Bit rate calculations
		params->dsi.pll_div1=0;
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
		params->dsi.fbk_div=16;
#else
		params->dsi.PLL_CLOCK = 210;
#endif

}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
#ifndef BUILD_LK
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(120);
#endif
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
	lcm_init();
#endif
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
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}

static unsigned int lcm_esd_recover()
{
    lcm_init();
    return TRUE;
}


static void lcm_setpwm(unsigned int divider)
{
	// TBD
}

static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = (id_high<<8) | id_low;
	#if defined(BUILD_LK)
		printf("OTM8019A uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("OTM8019A kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif

	return (LCM_ID_OTM8019A == id)?1:0;
}

LCM_DRIVER hct_otm8019a_dsi_vdo_fwvga_tm = 
{
	.name = "hct_otm8019a_dsi_vdo_fwvga_tm",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
#if defined(LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};
