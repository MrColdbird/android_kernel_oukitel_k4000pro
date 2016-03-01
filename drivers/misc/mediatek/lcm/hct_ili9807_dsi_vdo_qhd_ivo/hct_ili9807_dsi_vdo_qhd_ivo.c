//#include "lcd_ctl.h"
#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#else
	#include <linux/string.h>
	#if defined(BUILD_UBOOT)
		#include <asm/arch/mt_gpio.h>
	#else
		#include <mach/mt_gpio.h>
	#endif
#endif
#include "lcm_drv.h"






#ifndef __LCD_CTL_H__
#define __LCD_CTL_H__

//#define LCD_DEBUG

#include "lcm_drv.h"
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif

#ifdef BUILD_LK
#define LCD_PRT(format, args...) \
        do{printf("LCD_LK_PRT > %s() _%d_: " format, __FUNCTION__ , __LINE__, ## args);}while(0)
#ifdef LCD_DEBUG
#define LCD_DBG(format, args...) \
		do{printf("LCD_LK_DBG > %s() _%d_: " format, __FUNCTION__ , __LINE__, ## args);}while(0)
#else // LCD_DEBUG
#define LCD_DBG(format, args...) \
        do{}while(0)
#endif // LCD_DEBUG
#else // BUILD_LK
#define LCD_PRT(format, args...) \
		do{printk(KERN_INFO "LCD_KERN_PRT > %s() _%d_: " format, __FUNCTION__ , __LINE__, ## args);}while(0)
#ifdef LCD_DEBUG
#define LCD_DBG(format, args...) \
		do{printk(KERN_INFO "LCD_KERN_DBG > %s() _%d_: " format, __FUNCTION__ , __LINE__, ## args);}while(0)
#else // LCD_DEBUG
#define LCD_DBG(format, args...) \
        do{}while(0)
#endif // LCD_DEBUG						
#endif // BUILD_LK

#endif // __LCD_CTL_H__
// ---------------------------------------------------------------------------
// Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH (540)
#define FRAME_HEIGHT (960)

#define REGFLAG_DELAY 0xffd
#define REGFLAG_END_OF_TABLE 0xffe // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_OTM9807	0x9807


#define LCD_MODUL_ID (0x02)

#if 0
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE; ///only for ESD test

#endif

// ---------------------------------------------------------------------------
// Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
// Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 

static struct LCM_setting_table {
unsigned char cmd;
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
  #if 1
  
  {0xFF,5,{0xFF,0x98,0x07,0x00,0x01}},

{0x31,1,{0x00}},              

{0x40,1,{0x55}},              

{0x41,1,{0x55}},             

{0x42,1,{0x03}},             

{0x43,1,{0x8A}},              
            
{0x44,1,{0x07}},              
            
{0x46,1,{0xEE}},              
            
{0x47,1,{0xDD}},              

{0x50,1,{0xA0}},              
            
{0x51,1,{0xA0}},              
            
{0x52,1,{0x00}},              
            
{0x53,1,{0x15}},              

{0x60,1,{0x05}},             
            
{0x61,1,{0x08}},
            
{0x62,1,{0x08}},
            
{0x63,1,{0x08}},

{0x64,1,{0x88}},        
         
{0xA0,1,{0x00}},  
            
{0xA1,1,{0x0D}},  
            
{0xA2,1,{0x16}},
            
{0xA3,1,{0x0E}},

{0xA4,1,{0x07}},
            
{0xA5,1,{0x0D}},
            
{0xA6,1,{0x07}},
            
{0xA7,1,{0x05}},

{0xA8,1,{0x07}},
            
{0xA9,1,{0x0C}},
            
{0xAA,1,{0x12}},
            
{0xAB,1,{0x08}},

{0xAC,1,{0x0E}},
            
{0xAD,1,{0x15}},
            
{0xAE,1,{0x0D}},
            
{0xAF,1,{0x0d}},

{0xC0,1,{0x00}},  
            
{0xC1,1,{0x0D}},  
            
{0xC2,1,{0x16}},  
            
{0xC3,1,{0x0E}},  
            
{0xC4,1,{0x07}},  
            
{0xC5,1,{0x0D}},  
            
{0xC6,1,{0x07}},  
            
{0xC7,1,{0x05}},  
            
{0xC8,1,{0x07}},  
            
{0xC9,1,{0x0C}},  
            
{0xCA,1,{0x12}},  
            
{0xCB,1,{0x08}},  
            
{0xCC,1,{0x0E}},  
            
{0xCD,1,{0x15}},  
            
{0xCE,1,{0x0D}},  
            
{0xCF,1,{0x0d}}, 

{0xFF,5,{0xFF,0x98,0x07,0x00,0x06}}, 

{0x00,1,{0x21}},
            
{0x01,1,{0x06}},
            
{0x02,1,{0x20}},    
            
{0x03,1,{0x02}},
            
{0x04,1,{0x01}},
            
{0x05,1,{0x01}},
            
{0x06,1,{0x80}},    
            
{0x07,1,{0x04}},
            
{0x08,1,{0x03}},
            
{0x09,1,{0x00}},    
            
{0x0A,1,{0x00}},    
            
{0x0B,1,{0x00}},    
            
{0x0C,1,{0x01}},
            
{0x0D,1,{0x01}},
            
{0x0E,1,{0x00}},
            
{0x0F,1,{0x00}},

{0x10,1,{0xFF}},
            
{0x11,1,{0xF0}},
            
{0x12,1,{0x00}},
            
{0x13,1,{0x00}},
            
{0x14,1,{0x00}},
            
{0x15,1,{0xC0}},
            
{0x16,1,{0x08}},
            
{0x17,1,{0x00}},
            
{0x18,1,{0x00}},
            
{0x19,1,{0x00}},
            
{0x1A,1,{0x00}},
            
{0x1B,1,{0x00}},
            
{0x1C,1,{0x00}},
            
{0x1D,1,{0x00}},
            
{0x20,1,{0x01}},  
            
{0x21,1,{0x23}},
            
{0x22,1,{0x45}},
            
{0x23,1,{0x67}},
            
{0x24,1,{0x01}},
            
{0x25,1,{0x23}},
            
{0x26,1,{0x45}},
            
{0x27,1,{0x67}},
            
{0x30,1,{0x10}},  
            
{0x31,1,{0x96}},  
            
{0x32,1,{0x87}},  
            
{0x33,1,{0x96}},  
            
{0x34,1,{0x87}},  
            
{0x35,1,{0xAB}},  
            
{0x36,1,{0xCD}}, 
            
{0x37,1,{0xAB}},
            
{0x38,1,{0xCD}},
            
{0x39,1,{0xDC}},
            
{0x3A,1,{0xBA}},
            
{0x3B,1,{0xDC}},
            
{0x3C,1,{0xBA}},
            
{0x3D,1,{0x69}},
            
{0x3E,1,{0x78}},
            
{0x3F,1,{0x69}},

{0x40,1,{0x78}},
            
{0x41,1,{0x22}},
            
{0x42,1,{0x22}},
            
{0x43,1,{0x22}},

{0xFF,5,{0xFF,0x98,0x07,0x00,0x07}}, 
   
{0xE5,1,{0x5F}},      
           
{0x02,1,{0x77}},
 

	{0xFF,5,{0xFF,0x98,0x07,0x00,0x00}},	

	{0x11,1,{0x00}},

	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 20, {}},
	
  
  /*
	{0xFF,5,{0xFF,0x98,0x07,0x00,0x01}},
	
	{0x31,1,{0x00}},//02 
	
	{0x40,1,{0x55}},
	
	{0x41,1,{0x33}},

	{0x46,1,{0xdc}},
	
	{0x47,1,{0xbb}},
	
	{0x50,1,{0x80}},
	
	{0x51,1,{0x80}},
	
	{0x52,1,{0x00}},

	{0x53,1,{0x5f}}, //0x54



	{0x60,1,{0x02}},
	
	{0x61,1,{0x00}},
	
	{0x62,1,{0x08}},

	{0x63,1,{0x00}},
	
	{0x64,1,{0x88}},
	

	{0xA0,1,{0x00}},

	{0xA1,1,{0x08}},
	
	{0xA2,1,{0x0c}},

	{0xA3,1,{0xcb}},

	{0xA4,1,{0x05}},

	{0xA5,1,{0x0a}},

	{0xA6,1,{0x08}},

	{0xA7,1,{0x06}},

	{0xA8,1,{0x06}},
	
	{0xA9,1,{0x0a}},

	{0xAA,1,{0x0e}},

	{0xAB,1,{0x06}},

	{0xAC,1,{0x0b}},

	{0xAD,1,{0x18}},

	{0xAE,1,{0x0e}},

	{0xAF,1,{0x00}},


	{0xC0,1,{0x00}},
	
	{0xC1,1,{0x04}},

	{0xC2,1,{0x0c}},

	{0xC3,1,{0x0c}},
	
	{0xC4,1,{0x04}},

	{0xC5,1,{0x0a}},

	{0xC6,1,{0x08}},

	{0xC7,1,{0x06}},
	
	{0xC8,1,{0x07}},
	
	{0xC9,1,{0x0a}},
	
	{0xCA,1,{0x0d}},

	{0xCB,1,{0x06}},

	{0xCC,1,{0x0b}},

	{0xCD,1,{0x18}},

	{0xCE,1,{0x0e}},

	{0xCF,1,{0x00}},






	{0xFF,5,{0xFF,0x98,0x07,0x00,0x06}},  
	{0x00,1,{0x21}},  
	{0x01,1,{0x06}},  
	{0x02,1,{0x00}},  
	{0x03,1,{0x01}},  
	{0x04,1,{0x16}},  
	{0x05,1,{0x16}}, 
	{0x06,1,{0x80}},  
	{0x07,1,{0x01}},  
	{0x08,1,{0x06}},
	{0x09,1,{0x00}}, 
	{0x0A,1,{0x00}},  
	{0x0B,1,{0x00}}, 
	{0x0C,1,{0x0f}},  
	{0x0D,1,{0x0f}},  
	{0x0E,1,{0x1f}},  
	{0x0F,1,{0x00}}, 


	{0x10,1,{0x77}}, 
	{0x11,1,{0xe0}}, 
	{0x12,1,{0x00}},
	{0x13,1,{0x00}}, 
	{0x14,1,{0x00}}, 
	{0x15,1,{0xc0}},  
	{0x16,1,{0x08}}, 
	{0x17,1,{0x00}},
	{0x18,1,{0x00}}, 
	{0x19,1,{0x00}},  
	{0x1A,1,{0x00}}, 
	{0x1B,1,{0x00}},
	{0x1C,1,{0x00}},
	{0x1D,1,{0x00}},  


	{0x20,1,{0x01}},
	{0x21,1,{0x23}},  
	{0x22,1,{0x45}},
	{0x23,1,{0x67}}, 
	{0x24,1,{0x01}}, 
	{0x25,1,{0x23}}, 
	{0x26,1,{0x45}}, 
	{0x27,1,{0x67}}, 


	{0x30,1,{0x01}}, 
	{0x31,1,{0x22}}, 
	{0x32,1,{0x22}},  
	{0x33,1,{0x11}}, 
	{0x34,1,{0x00}},
	{0x35,1,{0x86}}, 
	{0x36,1,{0x68}}, 
	{0x37,1,{0x22}}, 
	{0x38,1,{0x22}},  
	{0x39,1,{0xda}},  
	{0x3A,1,{0xbc}}, 
	{0x3B,1,{0xcb}},  
	{0x3C,1,{0xad}},  
	{0x3D,1,{0x22}},  
	{0x3E,1,{0x22}},  
	{0x3F,1,{0x22}}, 
	{0x40,1,{0x22}}, 
	{0x41,1,{0x22}},  
	{0x42,1,{0x22}},  
	{0x43,1,{0x22}},  


	{0xFF,5,{0xFF,0x98,0x07,0x00,0x07}},	
	{0x02,1,{0x77}},	
	{0x26,1,{0xa2}}, 	


	{0xFF,5,{0xFF,0x98,0x07,0x00,0x00}},	

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}					*/																							
#endif
};

 

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}

};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
     // Display off sequence
     {0x28, 1, {0x00}},
     {REGFLAG_DELAY, 20, {}},
     // Sleep Mode Ondiv1_real*div2_real
     {0x10, 1, {0x00}},
     {REGFLAG_DELAY, 120, {}},
     {REGFLAG_END_OF_TABLE, 0x00, {}}
	 
};

#if 0
static struct LCM_setting_table lcm_backlight_level_setting[] = {
    {0x51, 1, {0xFF}},
    {0x53, 1, {0x24}},//close dimming
    {0x55, 1, {0x00}},//close cabc
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

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
    //        MDELAY(5);
        }
    }
}


// ---------------------------------------------------------------------------
// LCM Driver Implementations
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
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
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

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 8;
	params->dsi.vertical_frontporch					= 4;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 6;
	params->dsi.horizontal_backporch				= 37;
	params->dsi.horizontal_frontporch				= 37;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	//params->dsi.PLL_CLOCK=185;//208
       params->dsi.PLL_CLOCK = 237;
      // params->dsi.ssc_disable                 = 1;
#if 0
  params->dsi.pll_div1 = 1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps  0
	params->dsi.pll_div2 = 0;		// div2=0,1,2,3;div1_real=1,2,4,4	                1
	//params->dsi.fbk_sel = 1;
	params->dsi.fbk_div = 13;     // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)      17
#endif    
}





static unsigned int lcm_compare_id(void)
{



#if 1
   int array[4];
	char buffer[4];
	char buffer1[4];
	char id_high=0;
	char id_low=0;
	int id=0;
	
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(50);  

array[0] = 0x00063902;
array[1] = 0x0798FFFF;
array[2] = 0x00000100;
dsi_set_cmdq(array, 3, 1);
array[0] = 0x00023700;
dsi_set_cmdq(array, 1, 1);

read_reg_v2(0x00, buffer, 1);
id_high = buffer[1];
read_reg_v2(0x01, buffer1, 1);
id_low = buffer1[1];
//id = (id_midd &lt;&lt; 8 | id_low);	//we only need ID
	id = (id_high<<8) | id_low;

	#if defined(BUILD_LK)
		printf("ili9608e_dsi_vdo uboot %s \n", __func__);
		printf("%s id = 0x%x,0x%x\n", __func__, buffer[1],buffer[2]);
	#else
		printk("ili9608e_dsi_vdo kernel %s \n", __func__);
		printk("lj 9806e %s id = 0x%08x \n", __func__, id);
	#endif

   // return 1;

	return (LCM_ID_OTM9807 == id)?1:0;
#else
    return 1;//0;
#endif

}



static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(130);
	LCD_PRT("\n");
    //lcm_init_registers();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	#ifdef BUILD_LK

#else
    printk("lj ili9806e  init  \n" );//should be 0x8009
#endif
}


static void lcm_suspend(void)
{
	LCD_PRT("\n");
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

}

static void lcm_resume(void)
{
	LCD_PRT("\n");
#ifdef BUILD_LK

#else
    printk("lj ili9806e  resum , \n" );//should be 0x8009
#endif


   // lcm_initialization_setting[17].para_list[0]++;
    lcm_init();

    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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

#if 0
static void lcm_setbacklight(unsigned int level)
{

    unsigned int mapped_level = 0;

    //for LGE backlight IC mapping table
    if(level > 255)
    level = 255;

    mapped_level = level * 7 / 10;//to reduce power
   

    // Refresh value of backlight level.
    lcm_backlight_level_setting[0].para_list[0] = mapped_level;

#ifdef BUILD_LK
    printf("uboot:ili9806_lcm_setbacklight mapped_level = %d,level=%d\n",mapped_level,level);
#else
    printk("kernel:ili9806_lcm_setbacklight mapped_level = %d,level=%d\n",mapped_level,level);
#endif

    push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);

}

static unsigned int lcm_esd_check(void)
{
    #ifndef BUILD_LK
    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }

    if(read_reg(0x0A) == 0x9C)
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

    dsi_set_cmdq_V2(0x35, 1, &para, 1); ///enable TE
    MDELAY(10);

    return TRUE;
}
#endif


LCM_DRIVER hct_ili9807_dsi_vdo_qhd_ivo = 
{
    .name			= "hct_ili9807_dsi_vdo_qhd_ivo",
    .set_util_funcs = lcm_set_util_funcs,
    .compare_id = lcm_compare_id,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
#if (LCM_DSI_CMD_MODE)
    //.set_backlight = lcm_setbacklight,
    //.esd_check = lcm_esd_check,
    //.esd_recover = lcm_esd_recover,
    .update = lcm_update,
#endif
};

