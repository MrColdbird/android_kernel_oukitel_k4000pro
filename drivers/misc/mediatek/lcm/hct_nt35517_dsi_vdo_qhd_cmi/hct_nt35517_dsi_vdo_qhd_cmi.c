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

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0x100   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID       (0x5517)
#define LCM_ID1       (0xC1)
#define LCM_ID2       (0x80)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_initialization_setting[] = {
			{0xFF,4,{0xAA,0x55,0x25,0x81}},
		{0xF9,42,{0x02,0xA0,0x00,0x00,0x15,0x15,0x04,0x0B,0x08,0x0A,0x09,0x10,0x12,0x05,0x00,0x15,0x0C,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x15,0x06,0x15,0x12,0x0D,0x00,0x02,0x09,0x08,0x0A,0x07,0x0E,0x15,0x15}},
		{0xF9,4,{0x00,0x1A,0x00,0x40}},
		
		{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
		{0xb6,1,{0x09}},
		{0xB7,2,{0x77,0x77}},
		{0xB8,4,{0x01,0x04,0x00,0x00}},
		{0xBC,1,{0x02}},
		{0xC1,1,{0x01}},
		{0xCA,11,{0x01,0xE4,0xE4,0xE4,0x00,0x00,0x00,0x02,0x02,0x00,0x00}},
		
		{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
		{0xb0,1,{0x0A}},
		{0xb1,1,{0x0A}},
		{0xB2,1,{0x00}},
		{0xb3,1,{0x16}},
		{0xB4,1,{0x06}},
		{0xB6,1,{0x44}},
		{0xB7,1,{0x34}},
		{0xB8,1,{0x14}},
		{0xB9,1,{0x33}},
		{0xBA,1,{0x14}},
		{0xBC,3,{0x00,0x90,0x00}},
		{0xBD,3,{0x00,0x90,0x00}},
		{0xBE,1,{0x43}},
		{0xC2,1,{0x00}},
		{0xCF,1,{0x04}},
		
		{0xD1,16,{0x00,0x00,0x00,0x01,0x00,0x09,0x00,0x1D,0x00,0x36,0x00,0x6F,0x00,0x9B,0x00,0xEA}},
		{0xD2,16,{0x01,0x18,0x01,0x65,0x01,0x95,0x01,0xE0,0x02,0x19,0x02,0x1A,0x02,0x49,0x02,0x78}},
		{0xD3,16,{0x02,0x92,0x02,0xB1,0x02,0xC3,0x02,0xDC,0x02,0xEF,0x02,0xFF,0x03,0x0F,0x03,0x28}},
		{0xD4,4,{0x03,0x65,0x03,0xD6}},

	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    
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
 /*           
                if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
                    //#if defined(BUILD_UBOOT)
                    //	printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
                    //#endif
                    while(read_reg(cmd) != table[i].para_list[0]);		
                }
*/
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
    
//    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;
    
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
	params->dsi.vertical_backporch					= 60;   //16
	params->dsi.vertical_frontporch					= 60;    //15
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 2;
	params->dsi.horizontal_backporch				= 60;  //64
	params->dsi.horizontal_frontporch				= 60;
//	params->dsi.horizontal_blanking_pixel		       = 60;
	params->dsi.horizontal_active_pixel		       = FRAME_WIDTH;
	// Bit rate calculation
#if 0 
	params->dsi.pll_div1=1;
	params->dsi.pll_div2=0;		// div2=0,1,2,3;div2_real=1,2,4,4
	params->dsi.fbk_div=17;
    // Bit rate calculation
#else
	params->dsi.PLL_CLOCK=250;
#endif

}


static void init_lcm_registers(void)
{

/*nt35510 hsd fwvga init add by hct-mjw 20130603*/
    unsigned int data_array[16];


    //*************Enable TE  *******************//
    data_array[0]= 0x00053902;
    data_array[1]= 0x2555aaff;
    data_array[2]= 0x00000001;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00093902;
    data_array[1]= 0x000201f8;
    data_array[2]= 0x00133320;
    data_array[3]= 0x00000048;
    dsi_set_cmdq(data_array, 4, 1);
    
    //*************Enable CMD2 Page1  *******************//
    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000108;
    dsi_set_cmdq(data_array, 3, 1);
    
    //************* AVDD: manual  *******************//
    data_array[0]=0x00043902;
    data_array[1]=0x0d0d0db0;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434b6;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x0d0d0db1;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434b7;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x000000b2;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x242424b8;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00023902;
    data_array[1]=0x000001bf;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x0f0f0fb3;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434b9;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x080808b5;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00023902;
    data_array[1]=0x000003c2;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x242424ba;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x007800bc;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x007800bd;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x006400be;
    dsi_set_cmdq(data_array, 2, 1);
    
    //*************Gamma Table  *******************//
    data_array[0]=0x00353902;
    data_array[1]=0x003300D1;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D2;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D3;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D4;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D5;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D6;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    MDELAY(10);

    // ********************  EABLE CMD2 PAGE 0 **************//
    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000008;
    dsi_set_cmdq(data_array, 3, 1);

    // ********************  EABLE DSI TE **************//
    data_array[0]=0x00033902;
    data_array[1]=0x0000fcb1;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00023902;
    data_array[1]=0x00006bb5;
    dsi_set_cmdq(data_array, 2, 1);    
    data_array[0]=0x00023902;
    data_array[1]=0x000005b6;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x007070b7;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00053902;
    data_array[1]=0x030301b8;
    data_array[2]=0x00000003;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x000002bc;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00063902;
    data_array[1]=0x5002d0c9;
    data_array[2]=0x00005050;
    dsi_set_cmdq(data_array, 3, 1);
    
    // ********************  EABLE DSI TE packet **************//
    data_array[0]=0x00351500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]=0x773a1500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= 0x0100002a;
    data_array[2]= 0x000000df;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= 0x0300002b;
    data_array[2]= 0x00000055;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    
    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);

    //init_lcm_registers();
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    data_array[0] = 0x014F1500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(40);
}


static void lcm_resume(void)
{
    //static unsigned int flicker=0x50;
    //flicker+=2;
    //lcm_initialization_setting[15].para_list[1]=flicker;
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
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


#if defined(BUILD_LK)
    printf("%s, %d\n", __func__, level);
#elif defined(BUILD_UBOOT)
    printf("%s, %d\n", __func__, level);
#else
    printk("lcm_setbacklight = %d\n", level);
#endif
  
    if(level > 255) 
        level = 255;
    
    data_array[0]= 0x00023902;
    data_array[1] =(0x51|(level<<8));
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
    unsigned int pwm_clk = 23706 / (1<<divider);	


    return pwm_clk;
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0, id2 = 0;
    unsigned int id3=0;
    unsigned char buffer[2];
    unsigned int data_array[16];
    

    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);	
    
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
    id2 = buffer[0]; //we only need ID
    id3= buffer[1]; //we test buffer 1
//    return 1;
    id=id2<<8|id3;
#ifdef BUILD_LK
	printf("[erick-lk]%s,  nt35517 id = 0x%08x\n", __func__, id);
#else
	printk("[erick-k]%s,  nt35517 id = 0x%08x\n", __func__, id);
#endif
    return (LCM_ID == id)?1:0;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hct_nt35517_dsi_vdo_qhd_cmi = 
{
    .name			= "hct_nt35517_dsi_vdo_qhd_cmi",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .set_backlight	= lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .compare_id    = lcm_compare_id,
    //.update         = lcm_update
};

