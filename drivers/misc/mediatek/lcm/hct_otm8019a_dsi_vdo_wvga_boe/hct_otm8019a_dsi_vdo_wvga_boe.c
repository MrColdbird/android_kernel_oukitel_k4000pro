#ifndef BUILD_LK
#include <linux/string.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										    (800)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
       

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
{0x00,1,{0x00}},
	{0xFF,3,{0x80,0x19,0x01}},
	
	{0x00,1,{0x80}},
	{0xFF,2,{0x80,0x19}},
	
	{0x00,1,{0x8A}},
	{0xC4,1,{0x40}},
	
	{0x00,1,{0xA6}},
	{0xB3,2,{0x20,0x01}},
	
	{0x00,1,{0x90}},
	{0xC0,6,{0x00,0x15,0x00,0x00,0x00,0x03}},
	
	{0x00,1,{0xB4}},
	{0xC0,2,{0x20,0x48}},//18·
	
	{0x00,1,{0x81}},
	{0xC1,1,{0x33}},
	
	{0x00,1,{0x81}},
	{0xC4,1,{0x81}},
	
	{0x00,1,{0x87}},
	{0xC4,1,{0x00}},
	
	{0x00,1,{0x89}},
	{0xC4,1,{0x00}},
	
	{0x00,1,{0x82}},
	{0xC5,1,{0xB0}},
	
	{0x00,1,{0x90}},
	{0xC5,7,{0x6e,0x79,0x06,0x91,0x33,0x34,0x23}},
	
	{0x00,1,{0xB1}},
	{0xC5,1,{0xA8}},
	
	{0x00,1,{0x00}},
	{0xD8,2,{0x68,0x68}},
	
	{0x00,1,{0x00}},
	{0xD9,1,{0x51}},//0x5d
	
	{0x00,1,{0x80}},
	{0xCE,12,{0x86,0x01,0x00,0x85,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0xA0}},
	{0xCE,14,{0x18,0x05,0x83,0x39,0x00,0x00,0x00,0x18,0x04,0x83,0x3A,0x00,0x00,0x00}},
	
	{0x00,1,{0xB0}},
	{0xCE,14,{0x18,0x03,0x83,0x3B,0x86,0x00,0x00,0x18,0x02,0x83,0x3C,0x88,0x00,0x00}},
	
	{0x00,1,{0xC0}},
	{0xCF,10,{0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x02,0x00,0x00}},
	
	{0x00,1,{0xD0}},
	{0xCF,1,{0x00}},
	
	{0x00,1,{0xC0}},
	{0xCB,15,{0x00,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0xD0}},
	{0xCB,1,{0x00}},
	
	{0x00,1,{0xD5}},
	{0xCB,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0xE0}},
	{0xCB,6,{0x01,0x01,0x01,0x01,0x01,0x00}},
	
	{0x00,1,{0x80}},
	{0xCC,10,{0x00,0x26,0x09,0x0B,0x01,0x25,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0x90}},
	{0xCC,6,{0x00,0x00,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0x9A}},
	{0xCC,5,{0x00,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0xA0}},
	{0xCC,11,{0x00,0x00,0x00,0x00,0x00,0x25,0x02,0x0C,0x0A,0x26,0x00}},
	
	{0x00,1,{0xB0}},
	{0xCC,10,{0x00,0x25,0x0C,0x0A,0x02,0x26,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0xC0}},
	{0xCC,6,{0x00,0x00,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0xCA}},
	{0xCC,5,{0x00,0x00,0x00,0x00,0x00}},
	
	{0x00,1,{0xD0}},
	{0xCC,11,{0x00,0x00,0x00,0x00,0x00,0x26,0x01,0x09,0x0B,0x25,0x00}},
	
	{0x00,1,{0x00}},
	{0xE1,20,{0x00,0x04,0x08,0x10,0x24,0x40,0x55,0x94,0x85,0x9B, 0x6C,0x5c,0x76,0x62,0x68,0x61,0x5B,0x4D,0x47,0x00}},
	//{0x00,1,{0x00}},
	//{0xE2,20,{0x00,0x04,0x08,0x10,0x24,0x40,0x55,0x94,0x85,0x9B, 0x6C,0x5c,0x76,0x62,0x68,0x61,0x5B,0x4D,0x47,0x00}},
		

	{0x00,1,{0x00}},
	{0xE2,20,{0x00,0x04,0x08,0x10,0x24,0x40,0x55,0x93,0x85,0x9B, 0x6C,0x5c,0x75,0x62,0x68,0x61,0x5A,0x4D,0x46,0x00}},
	
	{0x00,1,{0x98}},
	{0xC0,1,{0x00}},
	
	{0x00,1,{0xA9}},
	{0xC0,1,{0x0a}},//06 0a
	
	{0x00,1,{0xB0}},
	{0xC1,3,{0x20,0x00,0x00}},
	
	{0x00,1,{0xE1}},
	{0xC0,2,{0x40,0x30}},//18 30
	
	{0x00,1,{0x80}},
	{0xC4,1,{0x30}},
	
	{0x00,1,{0x80}},
	{0xC1,2,{0x03,0x33}},
	
	{0x00,1,{0xA0}},
	{0xC1,1,{0x02}},
	
	{0x00,1,{0x90}},
	{0xB6,1,{0xB4}},

	{0x00,1,{0x91}},
	{0xd6,1,{0xcf}},//fae

	{0x00,1,{0x95}},
	{0xc0,1,{0x08}},//fae
	
	{0x00,1,{0x00}},
	{0xFF,3,{0xFF,0xFF,0xFF}},
	
	{0x00,1,{0x00}},
	{0x3A,1,{0x77}},

	//{0x21,0,{0x00}},

	//{0x00,1,{0x00}},
	{0x11,0,{0x00}},

	
        {REGFLAG_DELAY, 120, {}},

	
	//{0x00,1,{0x00}},
	{0x29,0,{0x00}},
	{REGFLAG_DELAY, 50, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static struct LCM_setting_table lcm_set_window[] = {
    {0x2A, 4, {0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
    {0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
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
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
asd
#else
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;
#endif

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
	//params->dsi.packet_size=256;

	// Video mode setting
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_sync_active				=2;// 4;
	params->dsi.vertical_backporch					= 16;//16;
	params->dsi.vertical_frontporch					= 16;//15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 4;//10;
	params->dsi.horizontal_backporch				= 65;
	params->dsi.horizontal_frontporch				= 65;
//	params->dsi.horizontal_blanking_pixel		       = 60;
	params->dsi.horizontal_active_pixel		       = FRAME_WIDTH;
		
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 350; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 180; //this value must be in MTK suggested table
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_resume(void)
{
	lcm_init();
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


static void lcm_setpwm(unsigned int divider)
{
	// TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
	// ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
	// pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
	unsigned int pwm_clk = 23706 / (1<<divider);	
	return pwm_clk;
}

#define LCM_ID                                               0x8019
static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	int id_high=0;
	int id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(25);
	SET_RESET_PIN(1);
	MDELAY(100);

	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0xa1, buffer, 5);

	id_high = buffer[2];
	id_low = buffer[3];
	id = ((id_high&0xff)<<8) | id_low;

	#if defined(BUILD_LK)
           printf("arthur OTM8019A little kernel read id = 0x%x \n", id);

	#else
		printk("OTM8019A kernel  read id  =0x%x \n", id);

	#endif
	return ((LCM_ID == id)) ? 1 : 0;
}


LCM_DRIVER hct_otm8019a_dsi_vdo_wvga_boe = 
{
	.name = "hct_otm8019a_dsi_vdo_wvga_boe",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
   	.compare_id    = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .set_backlight	= lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .update         = lcm_update
#endif
};
