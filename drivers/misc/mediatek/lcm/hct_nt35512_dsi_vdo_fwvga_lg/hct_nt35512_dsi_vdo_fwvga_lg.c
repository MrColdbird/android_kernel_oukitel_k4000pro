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

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID       (0x5512)
#define LCM_ID1       (0xC1)
#define LCM_ID2       (0x80)


#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

static unsigned int lcm_esd_test = TRUE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[120];
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	   {0xF0, 5,{0x55,0xAA,0x52,0x08,0x01}},
     {0xB0, 1,{0x0D}},
     {0xB6, 1,{0x34}},

     {0xB1, 1,{0x0D}},
     {0xB7, 1,{0x24}},

     {0xB2, 1,{0x01}},
     {0xB8, 1,{0x24}},

     {0xB3, 1,{0x0C}},
     {0xB9, 1,{0x35}},
     {0xBF, 1,{0x00}},

     {0xB5, 1,{0x0b}},
     {0xBA, 1,{0x14}},

     {0xBC, 3,{0x00,0x9E,0x00}}, 

     {0xBD, 3,{0x00,0x9a,0x00}},

     {0xBE, 2,{0x00,0x64}},

     {0xD0, 4,{0x0F,0x0F,0x10,0x10}},

     {0xD1,52,{0x00,0x01,0x00,0x46,0x00,0x6B,0x00,0x80,0x00,0x8E,0x00,0xB1,0x00,0xCE,0x00,0xFC,0x01,0x21,0x01,0x57,0x01,0x86,0x01,0xCC,0x02,0x06,0x02,0x07,0x02,0x3E,0x02,0x76,0x02,0x9B,0x02,0xC8,0x02,0xEC,0x03,0x17,0x03,0x2F,0x03,0x52,0x03,0x6B,0x03,0x85,0x03,0x9D,0x03,0xA4}},
     {0xD2,52,{0x00,0x01,0x00,0x46,0x00,0x6B,0x00,0x80,0x00,0x8E,0x00,0xB1,0x00,0xCE,0x00,0xFC,0x01,0x21,0x01,0x57,0x01,0x86,0x01,0xCC,0x02,0x06,0x02,0x07,0x02,0x3E,0x02,0x76,0x02,0x9B,0x02,0xC8,0x02,0xEC,0x03,0x17,0x03,0x2F,0x03,0x52,0x03,0x6B,0x03,0x85,0x03,0x9D,0x03,0xA4}},
     {0xD3,52,{0x00,0x01,0x00,0x46,0x00,0x6B,0x00,0x80,0x00,0x8E,0x00,0xB1,0x00,0xCE,0x00,0xFC,0x01,0x21,0x01,0x57,0x01,0x86,0x01,0xCC,0x02,0x06,0x02,0x07,0x02,0x3E,0x02,0x76,0x02,0x9B,0x02,0xC8,0x02,0xEC,0x03,0x17,0x03,0x2F,0x03,0x52,0x03,0x6B,0x03,0x85,0x03,0x9D,0x03,0xA4}},
     {0xD4,52,{0x00,0x01,0x00,0x46,0x00,0x6B,0x00,0x80,0x00,0x8E,0x00,0xB1,0x00,0xCE,0x00,0xFC,0x01,0x21,0x01,0x57,0x01,0x86,0x01,0xCC,0x02,0x06,0x02,0x07,0x02,0x3E,0x02,0x76,0x02,0x9B,0x02,0xC8,0x02,0xEC,0x03,0x17,0x03,0x2F,0x03,0x52,0x03,0x6B,0x03,0x85,0x03,0x9D,0x03,0xA4}},
     {0xD5,52,{0x00,0x01,0x00,0x46,0x00,0x6B,0x00,0x80,0x00,0x8E,0x00,0xB1,0x00,0xCE,0x00,0xFC,0x01,0x21,0x01,0x57,0x01,0x86,0x01,0xCC,0x02,0x06,0x02,0x07,0x02,0x3E,0x02,0x76,0x02,0x9B,0x02,0xC8,0x02,0xEC,0x03,0x17,0x03,0x2F,0x03,0x52,0x03,0x6B,0x03,0x85,0x03,0x9D,0x03,0xA4}},
     {0xD6,52,{0x00,0x01,0x00,0x46,0x00,0x6B,0x00,0x80,0x00,0x8E,0x00,0xB1,0x00,0xCE,0x00,0xFC,0x01,0x21,0x01,0x57,0x01,0x86,0x01,0xCC,0x02,0x06,0x02,0x07,0x02,0x3E,0x02,0x76,0x02,0x9B,0x02,0xC8,0x02,0xEC,0x03,0x17,0x03,0x2F,0x03,0x52,0x03,0x6B,0x03,0x85,0x03,0x9D,0x03,0xA4}},

     {0xF0, 5,{0x55,0xAA,0x52,0x08,0x00}},

     {0xB1, 2,{0xFC,0x00}},//{0xFC,0x06}},

     {0xB5, 1,{0x6B}},

     {0xB6, 1,{0x05}},
     {0xB7, 2,{0x71,0x71}},

     {0xB8, 4,{0x01,0x05,0x05,0x05}},

     //{0xBA, 1,{0x05}},

     {0xBC, 1,{0x00}},

     {0xBD, 5,{0x01,0x78,0x14,0x14,0x00}},

     {0xC9, 5,{0xC0,0x02,0x50,0x50,0x50}},

     {0x11, 0,{}},
     {REGFLAG_DELAY, 150, {}},                                
     {0x29, 0,{}},
     {REGFLAG_DELAY, 50, {}},
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
				//MDELAY(10);//soso add or it will fail to send register
       	}
    }
	MDELAY(20);
	
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

  params->dsi.vertical_sync_active				= 4;
    params->dsi.vertical_backporch					= 12;
    params->dsi.vertical_frontporch					= 16;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active			= 4;
    params->dsi.horizontal_backporch				= 40;
    params->dsi.horizontal_frontporch				= 40;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK=210;//227;//254;//254//247


    // Bit rate calculation
/*
    // ESD or noise interference recovery For video mode LCM only. // Send TE packet to LCM in a period of n frames and check the response.
    params->dsi.lcm_int_te_monitor = FALSE;
    params->dsi.lcm_int_te_period = 1; // Unit : frames

    // Need longer FP for more opportunity to do int. TE monitor applicably.
    if(params->dsi.lcm_int_te_monitor)
    params->dsi.vertical_frontporch *= 2;

    // Monitor external TE (or named VSYNC) from LCM once per 2 sec. (LCM VSYNC must be wired to baseband TE pin.)
    params->dsi.lcm_ext_te_monitor = FALSE;
    // Non-continuous clock
    params->dsi.noncont_clock = TRUE;
    params->dsi.noncont_clock_period = 2; // Unit : frames
*/
}

static void lcm_init(void)
{
    unsigned int data_array[64];

    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(50);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(120);//Must > 120ms

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


#if 0
static void lcm_suspend(void)
{
#ifndef BUILD_LK
   unsigned int data_array[16];
   data_array[0] = 0x00002200;
   dsi_set_cmdq(data_array, 1, 1);
   data_array[0] = 0x00280500;
   dsi_set_cmdq(data_array, 1, 1);
   MDELAY(10);
   data_array[0] = 0x00100500;
   dsi_set_cmdq(data_array, 1, 1);
   MDELAY(120);


  // push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
/*
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(50);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(150);//Must > 120ms
*/
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}
#else
static void lcm_suspend(void)
{
#ifndef BUILD_LK
   push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(50);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(150);//Must > 120ms
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}
#endif





static void lcm_resume(void)
{
#ifndef BUILD_LK
	lcm_init();
    //push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0, id2 = 0;
	unsigned char buffer[2];
	unsigned int data_array[16];
	

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10); 
	
	/*	
	data_array[0] = 0x00110500; 	// Sleep Out
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
	id = buffer[0]; //we only need ID
	id2= buffer[1]; //we test buffer 1

	#if defined(BUILD_LK)
        printf("NT35512 uboot %s \n", __func__);
        printf("%s id = 0x%08x \n", __func__, id);
		printf("%s id2 = 0x%08x \n", __func__, id2);
    #else
        printk("NT35512 kernel %s \n", __func__);
        printk("%s id = 0x%08x \n", __func__, id);
		printk("%s id2 = 0x%08x \n", __func__, id2);
    #endif
	
	id = (id<<8) | id2;
	return (LCM_ID == id)?1:0;
}


int err_count = 0;
static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    unsigned char buffer[8] = {0};
    unsigned int array[4];
    int i =0;

        
    array[0] = 0x00023700;    
    dsi_set_cmdq(array, 1,1);
    read_reg_v2(0x0A, buffer,2);

    //printk( "liukunping nt35521_JDI lcm_esd_check: buffer[0] = %x,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

    if((buffer[0] == 0x9C))/*LCD work status error,need re-initalize*/
    {
        //printk( " lcm_esd_check buffer[0] = %d\n",buffer[0]);
        return FALSE;
    }
    else
    {
        return TRUE;//tek.xing
        
        if(buffer[3] != 0x02) //error data type is 0x02
        {
             return FALSE;
        }
        else
        {
             if((buffer[4] != 0) || (buffer[5] != 0x80))
             {
                  err_count++;
             }
             else
             {
                  err_count = 0;
             }             
             if(err_count >=2 )
             {
                 err_count = 0;
                 //printk( "liukunping nt35521_JDI lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);

                 return TRUE;
             }
        }
        return FALSE;
    }

#endif
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK
    lcm_init();
#endif
    return TRUE;
}


LCM_DRIVER hct_nt35512_dsi_vdo_fwvga_lg = 
{
    .name			= "hct_nt35512_dsi_vdo_fwvga_lg",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
//    .esd_check       = lcm_esd_check,
//    .esd_recover       = lcm_esd_recover,
};

