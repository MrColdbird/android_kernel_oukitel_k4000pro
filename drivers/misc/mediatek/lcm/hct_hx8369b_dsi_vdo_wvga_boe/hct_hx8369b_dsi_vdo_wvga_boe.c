#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/string.h>
#endif


#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (800)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)      

//#define LCM_DSI_CMD_MODE

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
 {0xB9,3,{0xFF,0x83,0x69}},
{REGFLAG_DELAY, 10, {0x00}},
{0xD5,92,{0x00,0x00,0x13,0x03,0x35,0x00,0x01,0x10,0x01,0x00,
	        0x00,0x00,0x01,0x7A,0x16,0x04,0x04,0x13,0x07,0x40,
	        0x13,0x00,0x00,0x00,0x20,0x10,0x00,0x00,0x00,0x00,
	        0x00,0x00,0x00,0x00,0x00,0x36,0x00,0x00,0x48,0x88,
	        0x85,0x42,0x00,0x88,0x88,0x00,0x00,0x18,0x88,0x86,
	        0x71,0x35,0x88,0x88,0x00,0x00,0x58,0x88,0x87,0x63,
	        0x11,0x88,0x88,0x00,0x00,0x08,0x88,0x84,0x50,0x24,
	        0x88,0x88,0x00,0x00,0x00,0x51,0x00,0x00,0x00,0x00,
	        0x00,0x0F,0x00,0x0F,0x00,0x00,0x0F,0x00,0x0F,0x00,
	        0x01,0x5A}},
{0xBA,15,{0x31,0x00,0x16,0xCA,0xB1,0x0A,0x00,0x10,0x28,0x02,0x21,0x21,0x9A,0x1A,0x8F}},
{0xB1,10,{0x09,0x83,0x17,0x00,0x98,0x12,0x16,0x16,0x0C,0x0a}},
{0xB3,7,{0x83,0x00,0x31,0x03,0x01,0x0b,0x08}},//15,14
{0xB4,1,{0x02}},//
{0xB5,3,{0x0b,0x0b,0x24}},
{0xB6,2,{0x92,0x92}}, //90 for flicker 
{0xE0,35,{0x00,0x05,0x0B,0x2F,0x2F,0x30,0x1B,0x3D,0x07,0x0D,0x0E,0x12,0x13,0x12,0x13,0x11,0x1A,
		  0x00,0x05,0x0B,0x2F,0x2F,0x30,0x1B,0x3D,0x07,0x0D,0x0E,0x12,0x13,0x12,0x13,0x11,0x1A,0x01}},
{0xC0,6,{0x73,0x50,0x00,0x34,0xC4,0x02}},
{0xC6,4,{0x41,0xFF,0x7d,0xFF}},
{0xCC,1,{0x02}},//02
{0xE3,4,{0x07,0x0F,0x07,0x0F}},
{0xEA,1,{0x72}},
{0x3A,1,{0x77}},	
{0x11,1,{0x00}},
{REGFLAG_DELAY, 200, {0x00}},
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 50, {0x00}},

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
    {REGFLAG_DELAY, 120, {}},// more than 150ms
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 30, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
    {REGFLAG_DELAY, 30, {}},
	{0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xB9,	3,	{0xFF, 0x83, 0x69}},
	{REGFLAG_DELAY, 10, {}},

        // Sleep Mode On
        // {0xC3, 1, {0xFF}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Video mode setting		
			params->dsi.intermediat_buffer_num = 2;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.vertical_sync_active     = 4;
		params->dsi.vertical_backporch   = 19;
		params->dsi.vertical_frontporch  = 6;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		params->dsi.horizontal_sync_active   = 60;
		params->dsi.horizontal_backporch     = 50;
		params->dsi.horizontal_frontporch    = 50;		
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
	params->dsi.PLL_CLOCK = 210; //this value must be in MTK suggested table

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;
}


static void lcm_init(void)
{
   
     unsigned int data_array[30];    

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);
#if 0	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#else         
        data_array[0]=0x00043902; 
        data_array[1]=0x6983FFB9;// page enable 
        dsi_set_cmdq(&data_array, 2, 1); 
        MDELAY(10); 

	data_array[0]=0x00103902; 
        data_array[1]=0x160031BA;// page enable 
	data_array[2]=0x000AB1CA;
	data_array[3]=0x21022810;
	data_array[4]=0x8F1A9A21;
        dsi_set_cmdq(&data_array, 5, 1); 

	data_array[0]=0x000B3902; 
        data_array[1]=0x77830BB1;// page enable //0b
	data_array[2]=0x10111100;
	data_array[3]=0x00120C10;
        dsi_set_cmdq(&data_array, 4, 1); 

	data_array[0]=0x00083902; 
        data_array[1]=0x310083B3;// page enable 
	data_array[2]=0x06130103;	
        dsi_set_cmdq(&data_array, 3, 1); 

	data_array[0]=0x00023902; 
        data_array[1]=0x000003B2;// page enable 
        dsi_set_cmdq(&data_array, 2, 1); 

	data_array[0]=0x00023902; 
        data_array[1]=0x000000B4;// page enable 
        dsi_set_cmdq(&data_array, 2, 1); 

	data_array[0]=0x00033902; 
        data_array[1]=0x00A1A1B6;// page enable 
        dsi_set_cmdq(&data_array, 2, 1); 

	data_array[0]=0x00073902; 
        data_array[1]=0x005073C0;// page enable 
	data_array[2]=0x0000C434;	
        dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]=0x00053902; 
        data_array[1]=0x7AFF41C6;// page enable 
	data_array[2]=0x000000FF;	
        dsi_set_cmdq(&data_array, 3, 1); 

	data_array[0]=0x005D3902; 
        data_array[1]=0x0D0000D5;// page enable 
	data_array[2]=0x00000000;
	data_array[3]=0x00004012;
	data_array[4]=0x37600100;
	data_array[5]=0x010F0000;
	data_array[6]=0x00004705;
	data_array[7]=0x00000003;
	data_array[8]=0x00000000;
	data_array[9]=0x00000300;
	data_array[10]=0x89000018;
	data_array[11]=0x55331100;
	data_array[12]=0x00003177;
	data_array[13]=0x44660098;
	data_array[14]=0x00020022;
	data_array[15]=0x00008900;
	data_array[16]=0x20664422;
	data_array[17]=0x00980000;
	data_array[18]=0x11335577;
	data_array[19]=0x00000013;
	data_array[20]=0x00000001;
	data_array[21]=0xFFCF0003;
	data_array[22]=0xCF0003FF;
	data_array[23]=0x8C00FFFF;
	data_array[24]=0x0000005A;
        dsi_set_cmdq(&data_array, 25, 1); 

	data_array[0]=0x00023902; 
        data_array[1]=0x000000CC;// page enable 
        dsi_set_cmdq(&data_array, 2, 1); 

	data_array[0]=0x00053902; 
        data_array[1]=0x000000E3;// page enable 
	data_array[2]=0x00000000;	
        dsi_set_cmdq(&data_array, 3, 1); 

	data_array[0]=0x00243902; 
        data_array[1]=0x020000E0;// page enable 
	data_array[2]=0x193F0C0D;
	data_array[3]=0x0E0F042D;
	data_array[4]=0x16151714;
	data_array[5]=0x00001311;
	data_array[6]=0x3F120C02;
	data_array[7]=0x08052C19;
	data_array[8]=0x1416120E;
	data_array[9]=0x01131115;
        dsi_set_cmdq(&data_array, 10, 1); 

	data_array[0]=0x00023902; 
        data_array[1]=0x00000035;// page enable 
        dsi_set_cmdq(&data_array, 2, 1); 

	data_array[0]=0x00110500; 
        dsi_set_cmdq(&data_array, 1, 1); 
	MDELAY(150); 

        data_array[0]=0x00033902; 
        data_array[1]=0x00b0b0B6;// page enable 
        dsi_set_cmdq(&data_array, 2, 1); 

	data_array[0]=0x00290500; 
       dsi_set_cmdq(&data_array, 1, 1); 
	MDELAY(10); 
#endif


}


static void lcm_suspend(void)
{
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
        SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

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

// ---------------------------------------------------------------------------
//  Get LCM ID Information
// ---------------------------------------------------------------------------
static unsigned int lcm_compare_id()
    {
        unsigned int id = 0;
        unsigned char buffer[2];
        unsigned int array[16];

        SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
        MDELAY(10);
        SET_RESET_PIN(0);
        MDELAY(10);
        SET_RESET_PIN(1);
        MDELAY(100);
    
        array[0]=0x00043902; 
        array[1]=0x6983FFB9;// page enable 
        dsi_set_cmdq(&array, 2, 1); 
        MDELAY(10); 
    
        array[0] = 0x00013700;// read id return two byte,version and id
        dsi_set_cmdq(&array, 1, 1);
        
        read_reg_v2(0xD0, buffer, 1);
        id = buffer[0]; //we only need ID
        
        #ifdef BUILD_LK
           printf("%s, id1 = 0x%08x\n", __func__, id); 
        #else
           printk("%s, id1 = 0x%08x\n", __func__, id);   
        #endif
		
        return (0x01 == id)?1:0;
    }


LCM_DRIVER hct_hx8369b_dsi_vdo_wvga_boe = 
{
    .name			= "hct_hx8369b_dsi_vdo_wvga_boe",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if defined(LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };

