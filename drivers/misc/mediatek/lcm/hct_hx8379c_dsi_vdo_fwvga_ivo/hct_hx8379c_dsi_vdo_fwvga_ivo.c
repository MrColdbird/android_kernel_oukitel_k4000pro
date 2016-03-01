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
#else				/* 
 */
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else				/* 
 */
#include <mach/mt_gpio.h>
#endif				/* 
 */
#endif				/* 
 */
#include "lcm_drv.h"
    


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
    
#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)
    
#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFA	//0xFFF ??   // END OF REGISTERS MARKER
    
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = { 0 };




#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
    
#define UDELAY(n) (lcm_util.udelay(n))
#if defined(BUILD_LK)
#define MDELAY(n)   mdelay(n)	// (lcm_util.mdelay(n*20000))
#else				/* 
 */
extern void msleep(unsigned int msecs);

#define MDELAY(n)   msleep(n)	// (lcm_util.mdelay(n*20000))
#endif				/* 
 */
    


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
    
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)        
    
struct LCM_setting_table {
	
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

	
	// Set EXTC
	{0xB9,3,{0xFF,0x83,0x79}},

	 // Set Power,
	{0xB1,20,
	{0x44,0x15,0x15,0x31,0x31,0x50,0xD0,0xE8,0x5C,0x80,
	0x38,0x38,0xF8,0x44,0x44,0x42,0x00,0x80,0x30,0x00}},

	//set display
	{0xB2,4,
	{0x82,0xFE,0x0B,0x14}},

	// Set CYC
	{0xB4,13,
	{0x08,0x30,0x18,0x78,0x15,0x76,0x04,0x58,0x08,0x58,
	0xB0,0x00,0xFF}},

	{0xD3,29,
	{0x00,0x00,0x00,0x00,0x00,0x08,0x08,0x32,0x10,0x08,
	0x00,0x08,0x03,0x65,0x03,0x65,0x00,0x08,0x00,0x08,
	0x37,0x33,0x0B,0x0B,0x27,0x0B,0x0B,0x27,0x08}},

	//Set GIP 1
	{0xD5,32,
	{0x18,0x18,0x18,0x18,0x18,0x18,0x23,0x22,0x21,0x20,
	0x01,0x00,0x03,0x02,0x05,0x04,0x07,0x06,0x25,0x24,
	0x27,0x26,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
	0x18,0x18}},

	//Set GIP 2
	{0xD6,32,
	{0x18,0x18,0x18,0x18,0x18,0x18,0x26,0x27,0x24,0x25,
	0x02,0x03,0x00,0x01,0x06,0x07,0x04,0x05,0x22,0x23,
	0x20,0x21,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
	0x18,0x18}},


	// Set GAMMA 2.5
	{0xE0,42,
	{0x00,0x00,0x00,0x10,0x0F,0x1A,0x21,0x34,0x00,0x05,
	0x0E,0x18,0x0E,0x13,0x16,0x15,0x15,0x08,0x13,0x14,
	0x18,0x00,0x00,0x00,0x10,0x0F,0x1A,0x21,0x34,0x00,
	0x05,0x0E,0x18,0x0E,0x13,0x16,0x15,0x15,0x08,0x13,
	0x14,0x18}},


	// Set VCOM
	{0xB6,2,{0x67,0x67}},

	// Set Display direction, RGB
	{0xCC,1,{0x02}},

	{0x35,1,{0x00}},

	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {0x00}},

	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {0x00}},
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
{REGFLAG_DELAY, 20, {}}, 
	    // Sleep Mode On
	{0x10, 1, {0x00}}, 
	    {REGFLAG_DELAY, 120, {}}, 
{REGFLAG_END_OF_TABLE, 0x00, {}} 
};



static struct LCM_setting_table lcm_backlight_level_setting[] = { 
	    {0x51, 1, {0xFF}}, 
{REGFLAG_END_OF_TABLE, 0x00, {}} 
};




//edit by Magnum 2012-12-18
static void dsi_send_cmdq_tinno(unsigned cmd, unsigned char count,
				unsigned char *para_list,
				unsigned char force_update) 
{
	
unsigned int item[16];
	
unsigned char dsi_cmd = (unsigned char) cmd;
	
unsigned char dc;
	
int index = 0, length = 0;
	

memset(item, 0, sizeof(item));
	
if (count + 1 > 60)
		
 {
		

return;
		
}
	
	    /*
	       Note :
	       
	       Data ID will depends on the following rule.
	       
	       count of parameters > 1      => Data ID = 0x39
	       count of parameters = 1      => Data ID = 0x15
	       count of parameters = 0      => Data ID = 0x05
	       
	       Structure Format :
	       
	       {DCS command, count of parameters, {parameter list}}
	       {REGFLAG_DELAY, milliseconds of time, {}},
	       
	       ...
	       
	       Setting ending by predefined flag
	       
	       {REGFLAG_END_OF_TABLE, 0x00, {}}
	     */ 
	    if (count == 0)
		
 {
		
item[0] = 0x0500 | (dsi_cmd << 16);
		
length = 1;
		
}
	
	else if (count == 1)
		
 {
		
item[0] = 0x1500 | (dsi_cmd << 16) | (para_list[0] << 24);
		
length = 1;
		
}
	
	else
		
 {
		
item[0] = 0x3902 | ((count + 1) << 16);	//Count include command.
		++length;
		
while (1)
			
 {
			
if (index == count + 1)
				
break;
			
if (0 == index) {
				
dc = cmd;
			
} else {
				
dc = para_list[index - 1];
			
}
			
			    // an item make up of 4data. 
			    item[index / 4 + 1] |=
			    (dc << (8 * (index % 4)));
			
if (index % 4 == 0)
				++length;
			
++index;
			
}
		
}
	

dsi_set_cmdq(&item, length, force_update);


}


static void push_table(struct LCM_setting_table *table,
			unsigned int count, unsigned char force_update) 
{
	
unsigned int i;
	

for (i = 0; i < count; i++) {
		
unsigned cmd;
		
cmd = table[i].cmd;
		

switch (cmd) {
		
case REGFLAG_DELAY:
			
MDELAY(table[i].count);
			
break;
		

case REGFLAG_END_OF_TABLE:
			
break;
		

default:
			
dsi_set_cmdq_V2(cmd, table[i].count,
					 table[i].para_list, force_update);
			
#if 0
			    if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
				
				    //#if defined(BUILD_UBOOT)
				    //  printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
				    //#endif
				    while (read_reg(cmd) !=
					   table[i].para_list[0]);
			
}
			
#endif				/* 
 */
		}
	
}

}





// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util) 
{
	
memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));

}




static void lcm_get_params(LCM_PARAMS * params) 
{
	
memset(params, 0, sizeof(LCM_PARAMS));
	

params->type = LCM_TYPE_DSI;
	

params->width = FRAME_WIDTH;
	
params->height = FRAME_HEIGHT;
	


	    // enable tearing-free
	    params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	
params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	

params->dsi.mode = SYNC_PULSE_VDO_MODE;
	


	    // DSI
	    /* Command mode setting */ 
	    params->dsi.LANE_NUM = LCM_TWO_LANE;
	


	    //The following defined the fomat for data coming from LCD engine.
	    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	
params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	
params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	
params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	


	    // Video mode setting               
	    params->dsi.intermediat_buffer_num = 2;
	

params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	
params->dsi.packet_size = 256;
	
	    // params->dsi.word_count=480*3;     //DSI CMD mode need set these two bellow params, different to 6577
	    // params->dsi.vertical_active_line=800;
	    
params->dsi.vertical_sync_active = 9;	//4
	params->dsi.vertical_backporch = 6;	//7
	params->dsi.vertical_frontporch = 15;	//7
	params->dsi.vertical_active_line = FRAME_HEIGHT;
	

params->dsi.horizontal_sync_active = 32;
	
params->dsi.horizontal_backporch = 32;
	
params->dsi.horizontal_frontporch = 32;
	
	    //  params->dsi.horizontal_blanking_pixel                             = 0;
	    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	


	    // Bit rate calculation
	    // Bit rate calculation
	    //  params->dsi.pll_div1=26;//26;           // fref=26MHz, fvco=fref*(div1+1)       (div1=0~63, fvco=500MHZ~1GHz)
	    //  params->dsi.pll_div2=1;                 // div2=0~15: fout=fvo/(2*div2)
	    
	    //params->dsi.pll_div1=26;//26;             // fref=26MHz, fvco=fref*(div1+1)       (div1=0~63, fvco=500MHZ~1GHz)
	    //params->dsi.pll_div2=1;                   // div2=0~15: fout=fvo/(2*div2)
	    
	params->dsi.PLL_CLOCK = 210; //this value must be in MTK suggested table
}



static unsigned int lcm_compare_id(void);



static void init_lcm_registers(void) 
{
	
unsigned int data_array[16];
	
//hx8379c_TM3.97	  
data_array[0] = 0x00043902;	//Enable external Command
data_array[1] = 0x7983FFB9;	
dsi_set_cmdq(&data_array, 2, 1);	

data_array[0] = 0x00153902;	
data_array[1] = 0x181844B1;	
data_array[2] = 0xD0505131;
data_array[3] = 0x388094EE; 
data_array[4] = 0x3233F838;	
data_array[5] = 0x30800022;	
data_array[6] = 0x00000000;
dsi_set_cmdq(&data_array, 7, 1);	

data_array[0] = 0x000A3902;	
data_array[1] = 0x083C82B2;	 
data_array[2] = 0x11503004;	
data_array[3] = 0x00001D42;		
dsi_set_cmdq(&data_array, 4, 1);	

data_array[0] = 0x000B3902;	
data_array[1] = 0x117811B4; 	
data_array[2] = 0x0A781178;	
data_array[3] = 0x007A0A7A;	
dsi_set_cmdq(&data_array, 4, 1);	

data_array[0] = 0x00023902;	
data_array[1] = 0x000002CC;	
dsi_set_cmdq(&data_array, 2, 1);	

data_array[0] = 0x00023902;	
data_array[1] = 0x000033D2;	
dsi_set_cmdq(&data_array, 2, 1);	
	
data_array[0] = 0x00083902;	
data_array[1] = 0x009341ba;
data_array[2] = 0x1810a416;
dsi_set_cmdq(&data_array, 3, 1); 

data_array[0] = 0x00023902;	
data_array[1] = 0x000008c6;	
dsi_set_cmdq(&data_array, 2, 1);	
	
data_array[0] = 0x001E3902;	
data_array[1] = 0x000000D3;	
data_array[2] = 0x0C0C0000;	
data_array[3] = 0x00041034;	
data_array[4] = 0x00000004;	
data_array[5] = 0x00000000;	
data_array[6] = 0x06111700;	
data_array[7] = 0x01011306;	
data_array[8] = 0x00000913;			
dsi_set_cmdq(&data_array, 9, 1);	

data_array[0] = 0x00213902;	
data_array[1] = 0x181919D5;	
data_array[2] = 0x18181818;	
data_array[3] = 0x18181818;	
data_array[4] = 0x00030218;	
data_array[5] = 0x18212001;	
data_array[6] = 0x18181818;	
data_array[7] = 0x18181818;	
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(&data_array, 10, 1);	

data_array[0] = 0x00213902;	
data_array[1] = 0x191818D6;	
data_array[2] = 0x18181819;	
data_array[3] = 0x18181818;	
data_array[4] = 0x03000118;	
data_array[5] = 0x18202102;	
data_array[6] = 0x18181818;	
data_array[7] = 0x18181818;	
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(&data_array, 10, 1);

data_array[0] = 0x002B3902;	
data_array[1] = 0x020301E0;	
data_array[2] = 0x22201010;	
data_array[3] = 0x13110936;	
data_array[4] = 0x1A19151B;	
data_array[5] = 0x13081717;	
data_array[6] = 0x03011814;	
data_array[7] = 0x20101002;	
data_array[8] = 0x11093622;	
data_array[9] = 0x19151B13;
data_array[10] = 0x0817171A;
data_array[11] = 0x00181413;
dsi_set_cmdq(&data_array, 12, 1);	

data_array[0] = 0x00033902;	
data_array[1] = 0x00a6a6B6;	//VCOM 0x008B8BB6	
dsi_set_cmdq(&data_array, 2, 1);	
	
data_array[0] = 0x00023902;	
data_array[1] = 0x0000773A;	
dsi_set_cmdq(&data_array, 2, 1);	
	
data_array[0] = 0x00110500;	
dsi_set_cmdq(&data_array, 1, 1);	
MDELAY(150);
	
data_array[0] = 0x00290500;	
dsi_set_cmdq(&data_array, 1, 1);	
MDELAY(30);



} 


static void lcm_init(void) 
{
	
	    //lcm_compare_id();
	    SET_RESET_PIN(1);
	
MDELAY(20);
	
SET_RESET_PIN(0);
	
MDELAY(20);
	
SET_RESET_PIN(1);
	
MDELAY(120);
	
	    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	    //init_lcm_registers();

} 

static void lcm_suspend(void) 
{
	
push_table(lcm_deep_sleep_mode_in_setting,
		    sizeof(lcm_deep_sleep_mode_in_setting) /
		    sizeof(struct LCM_setting_table), 1);
	
SET_RESET_PIN(1);
	
MDELAY(10);
	
SET_RESET_PIN(0);
	
MDELAY(20);
	
SET_RESET_PIN(1);
	
MDELAY(120);

} 


static void lcm_resume(void) 
{
	
SET_RESET_PIN(1);
	
MDELAY(10);
	
SET_RESET_PIN(0);
	
MDELAY(20);
	
SET_RESET_PIN(1);
	
MDELAY(120);
	
	    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	    //init_lcm_registers();

} 


static void lcm_setbacklight(unsigned int level) 
{
	
unsigned int data_array[16];
	



#if defined(BUILD_LK)
	    printf("%s, %d\n", __func__, level);
	
#elif defined(BUILD_UBOOT)
	    printf("%s, %d\n", __func__, level);
	
#else				/* 
 */
	    printk("lcm_setbacklight = %d\n", level);
	
#endif				/* 
 */
	    
if (level > 255)
		
level = 255;
	

data_array[0] = 0x00023902;
	
data_array[1] = (0x51 | (level << 8));
	
dsi_set_cmdq(data_array, 2, 1);

}




static void lcm_setpwm(unsigned int divider) 
{
	
	    // TBD
} 


static unsigned int lcm_getpwm(unsigned int divider) 
{
	
	    // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
	    // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
	unsigned int pwm_clk = 23706 / (1 << divider);
	


return pwm_clk;

}




#define LCM_ID 0x79

static unsigned int lcm_compare_id(void) 
{
	
unsigned int id = 0, id2 = 0, id3 = 0;
	
unsigned char buffer[3];
	
unsigned int data_array[16];
	


SET_RESET_PIN(1);	//NOTE:should reset LCM firstly
	MDELAY(10);
	
SET_RESET_PIN(0);
	
MDELAY(10);
	
SET_RESET_PIN(1);
	
MDELAY(120);
	
	    //  return 1;
	    /*  
	       data_array[0] = 0x00110500;              // Sleep Out
	       dsi_set_cmdq(data_array, 1, 1);
	       MDELAY(120);
	     */ 
	    
	    //*************Enable CMD2 Page1  *******************//
	    data_array[0] = 0x00043902;
	
data_array[1] = 0x7983FFB9;
	
	    // data_array[2]=0x00000108;
	    dsi_set_cmdq(data_array, 2, 1);
	
MDELAY(10);
	
data_array[0] = 0x00033902;
	
data_array[1] = 0x009351ba;
	
dsi_set_cmdq(data_array, 2, 1);
	

data_array[0] = 0x00013700;	// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	
MDELAY(10);
	

read_reg_v2(0xF4, buffer, 3);
	
id = buffer[0];	//we only need ID
id2 = buffer[1];	//we test buffer 1
id3 = buffer[2];	//we test buffer 1
#ifdef BUILD_LK
	printf("lcd id %x %x %x\n", id, id2,id3);
	
#endif				/* 
 */
	    return (LCM_ID == id) ? 1 : 0;

}





static unsigned int lcm_esd_check(void) 
{
	


	    //  return 0; //FALSE
	unsigned char buffer[3];
	
unsigned int data_array[6];
	
unsigned int chip_id = 0;
	
///////////////////////////////////////////////////////
	    //*************Enable CMD2 Page1  *******************//
	    

data_array[0] = 0x00043700;	// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);
	
	    //MDELAY(10); 
	    
read_reg_v2(0x0A, buffer, 1);
	


#if defined(BUILD_LK)
	    printf("[hx8379] %s buffer[0] = %x;\n", __func__, buffer[0]);
	
#elif defined(BUILD_UBOOT)
	    printf("[hx8379] %s buffer[0] = %x;\n", __func__, buffer[0]);
	
#else				/* 
 */
	    printk("[hx8379] %s buffer[0] = %x;\n", __func__, buffer[0]);
	
#endif				/* 
 */
	    
if (0x1C == buffer[0])
		
 {
		
return 0;
		
}
	
	else
		
 {
		
return 1;
		
}
	
//////////////////////////////////////////////////////////

}




static unsigned int lcm_esd_recover(void) 
{
	
//    unsigned char para = 0;
/*
#if defined(hx8379)
    printf("[ili9806C] %s\n", __func__);
#elif defined(BUILD_UBOOT)
    printf("[hx8379] %s\n", __func__);
#else
    printk("[hx8379] %s\n", __func__);
#endif
*/ 
	    lcm_init();
	
	    //lcm_resume();
	    
return 1;		//TRUE
}


//#endif        
    


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
    LCM_DRIVER hct_hx8379c_dsi_vdo_fwvga_ivo = 
 {
	
.name = "hct_hx8379c_dsi_vdo_fwvga_ivo", 
.set_util_funcs =
	    lcm_set_util_funcs, 
.get_params = lcm_get_params, 
.init =
	    lcm_init, 
.suspend = lcm_suspend, 
.resume =
	    lcm_resume, 
//.set_backlight = lcm_setbacklight, 
	    //.set_pwm        = lcm_setpwm,
	    //.get_pwm        = lcm_getpwm,
.compare_id = lcm_compare_id, 
//.esd_check = lcm_esd_check, 
//.esd_recover = lcm_esd_recover, 
};



