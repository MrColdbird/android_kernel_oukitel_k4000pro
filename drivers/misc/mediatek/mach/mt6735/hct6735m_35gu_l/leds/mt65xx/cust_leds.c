#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>
#include "cust_gpio_usage.h"
//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);

// Only support 64 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT 64 
// Support 256 levels of backlight (when lcd-backlight = MT65XX_LED_MODE_PWM)
#define BACKLIGHT_LEVEL_PWM_256_SUPPORT 256 

// Configure the support type "BACKLIGHT_LEVEL_PWM_256_SUPPORT" or "BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT" !!
#define BACKLIGHT_LEVEL_PWM_MODE_CONFIG BACKLIGHT_LEVEL_PWM_256_SUPPORT

#ifdef GPIO_LCM_BL_EN  // this is plus count method
#define APP_SETTING_BL_TUNE_LOWEST (30)
#define APP_SETTING_BL_LOWEST_HALF (APP_SETTING_BL_TUNE_LOWEST/2)
#define SGM3138_MAX_LEVEL (15)
#endif
#ifdef GPIO_LCM_BL_EN
#define SGM3138_GPIO_BACKLIGHT  GPIO_LCM_BL_EN

#define SGM3138_GPIO_READY() udelay(40)
#define SGM3138_GPIO_SHUTDWON() mdelay(4)
#define SGM3138_GPIO_HIGH() udelay(1)
#define SGM3138_GPIO_LOW() udelay(1)

static inline void sgm3138_ready()
{
    mt_set_gpio_out(SGM3138_GPIO_BACKLIGHT,GPIO_OUT_ONE);
	SGM3138_GPIO_READY();
}

static inline void sgm3138_shutdown()
{
	int delay = 0;
	mt_set_gpio_out(SGM3138_GPIO_BACKLIGHT,GPIO_OUT_ZERO);
	for (delay =0;delay<100;delay++){
		udelay(40);
	}
}

static inline void sgm3138_high()
{
	mt_set_gpio_out(SGM3138_GPIO_BACKLIGHT,GPIO_OUT_ONE);
	SGM3138_GPIO_HIGH();
}

static inline void sgm3138_low()
{
	mt_set_gpio_out(SGM3138_GPIO_BACKLIGHT,GPIO_OUT_ZERO);
	SGM3138_GPIO_LOW();
}

static void sgm3138_tune(int level)
{
	int j = 0;
	
	for(j=0; j<level; j++) 
        {
		sgm3138_low();
		sgm3138_high(); 					  
	}
}
void cust_setbacklight_sgm3138(unsigned int level)
{
	int j = 0;
	static int pre_level = 16;
	unsigned long flags;
	printk("hsl cust_setbacklight_sgm3138:level =%d \n",level);

        mt_set_gpio_mode(SGM3138_GPIO_BACKLIGHT, GPIO_LCM_BL_EN_M_GPIO);
        mt_set_gpio_dir(SGM3138_GPIO_BACKLIGHT, GPIO_DIR_OUT);
//	mt_set_gpio_out(SGM3138_GPIO_BACKLIGHT,GPIO_OUT_ONE);

	if(0 == level)
	{
		sgm3138_shutdown();
		pre_level = 0;
		return;
	}

	if(level > 0) 
	{
		level /= 16;
		if(level <= 0) 
			level = 1;
		printk("afterlevel=%d ",level);
	}

	if( pre_level == 0 ) {
		sgm3138_ready();
	}

	if( pre_level >= level) {
printk("pre_level=%d, level=%d tune=%d\n",pre_level,level, pre_level-level);
		sgm3138_tune(pre_level-level);
	} else {
printk("pre_level=%d, level=%d tune=%d\n",pre_level,level,16 + pre_level - level);
		sgm3138_tune(16 + pre_level - level);
	}

	pre_level = level;
}

unsigned int lcd_brightness_mapping(unsigned int level)
{
    return  (level<APP_SETTING_BL_LOWEST_HALF) ? 0 : (level/SGM3138_MAX_LEVEL); 
}

#endif

unsigned int Cust_GetBacklightLevelSupport_byPWM(void)
{
	return BACKLIGHT_LEVEL_PWM_MODE_CONFIG;
}

unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;
    
    mapped_level = level;
       
	return mapped_level;
}
/*

 * To explain How to set these para for cust_led_list[] of led/backlight
 * "name" para: led or backlight
 * "mode" para:which mode for led/backlight
 *	such as:
 *			MT65XX_LED_MODE_NONE,	
 *			MT65XX_LED_MODE_PWM,	
 *			MT65XX_LED_MODE_GPIO,	
 *			MT65XX_LED_MODE_PMIC,	
 *			MT65XX_LED_MODE_CUST_LCM,	
 *			MT65XX_LED_MODE_CUST_BLS_PWM
 *
 *"data" para: control methord for led/backlight
 *   such as:
 *			MT65XX_LED_PMIC_LCD_ISINK=0,	
 *			MT65XX_LED_PMIC_NLED_ISINK0,
 *			MT65XX_LED_PMIC_NLED_ISINK1,
 *			MT65XX_LED_PMIC_NLED_ISINK2,
 *			MT65XX_LED_PMIC_NLED_ISINK3
 * 
 *"PWM_config" para:PWM(AP side Or BLS module), by default setting{0,0,0,0,0} Or {0}
 *struct PWM_config {	 
 *  int clock_source;
 *  int div; 
 *  int low_duration;
 *  int High_duration;
 *  BOOL pmic_pad;//AP side PWM pin in PMIC chip (only 89 needs confirm); 1:yes 0:no(default)
 *};
 *-------------------------------------------------------------------------------------------
 *   for AP PWM setting as follow:
 *1.	 PWM config data
 *  clock_source: clock source frequency, can be 0/1
 *  div: clock division, can be any value within 0~7 (i.e. 1/2^(div) = /1, /2, /4, /8, /16, /32, /64, /128)
 *  low_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *  High_duration: only for BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT
 *
 *2.	 PWM freq.
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_256_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / 256  
 *
 * If BACKLIGHT_LEVEL_PWM_MODE_CONFIG = BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT,
 *	 PWM freq. = clock source / 2^(div) / [(High_duration+1)(Level')+(low_duration+1)(64 - Level')]
 *	           = clock source / 2^(div) / [(High_duration+1)*64]     (when low_duration = High_duration)
 *Clock source: 
 *	 0: block clock/1625 = 26M/1625 = 16K (MT6571)
 *	 1: block clock = 26M (MT6571)
 *Div: 0~7
 *
 *For example, in MT6571, PWM_config = {1,1,0,0,0} 
 *	 ==> PWM freq. = 26M/2^1/256 	 =	50.78 KHz ( when BACKLIGHT_LEVEL_PWM_256_SUPPORT )
 *	 ==> PWM freq. = 26M/2^1/(0+1)*64 = 203.13 KHz ( when BACKLIGHT_LEVEL_PWM_64_FIFO_MODE_SUPPORT )
 *-------------------------------------------------------------------------------------------
 *   for BLS PWM setting as follow:
 *1.	 PWM config data
 *	 clock_source: clock source frequency, can be 0/1/2/3
 *	 div: clock division, can be any value within 0~1023
 *	 low_duration: non-use
 *	 High_duration: non-use
 *	 pmic_pad: non-use
 *
 *2.	 PWM freq.= clock source / (div + 1) /1024
 *Clock source: 
 *	 0: 26 MHz
 *	 1: 104 MHz
 *	 2: 124.8 MHz
 *	 3: 156 MHz
 *Div: 0~1023
 *
 *By default, clock_source = 0 and div = 0 => PWM freq. = 26 KHz 
 *-------------------------------------------------------------------------------------------
 */


#ifdef GPIO_RGB_RED_PIN
int rgb_set_red_by_gpio(unsigned int level)
{
	if(level > 0)
	{
		mt_set_gpio_mode(GPIO_RGB_RED_PIN, GPIO_RGB_RED_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_RGB_RED_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_RGB_RED_PIN,GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_mode(GPIO_RGB_RED_PIN, GPIO_RGB_RED_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_RGB_RED_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_RGB_RED_PIN,GPIO_OUT_ZERO);
	}
	
}
#endif

#ifdef GPIO_RGB_GREEN_PIN
int rgb_set_green_by_gpio(unsigned int level)
{
	if(level > 0)
	{
		mt_set_gpio_mode(GPIO_RGB_GREEN_PIN, GPIO_RGB_GREEN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_RGB_GREEN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_RGB_GREEN_PIN,GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_mode(GPIO_RGB_GREEN_PIN, GPIO_RGB_GREEN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_RGB_GREEN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_RGB_GREEN_PIN,GPIO_OUT_ZERO);
	}
	
}
#endif

#ifdef GPIO_RGB_BLUE_PIN
int rgb_set_blue_by_gpio(unsigned int level)
{
	if(level > 0)
	{
		mt_set_gpio_mode(GPIO_RGB_BLUE_PIN, GPIO_RGB_BLUE_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_RGB_BLUE_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_RGB_BLUE_PIN,GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_mode(GPIO_RGB_BLUE_PIN, GPIO_RGB_BLUE_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_RGB_BLUE_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_RGB_BLUE_PIN,GPIO_OUT_ZERO);
	}
	
}
#endif
#ifdef GPIO_BUTTON_LED_PIN
int rgb_set_Button_led_by_gpio(unsigned int level)
{
	if(level > 0)
	{
		mt_set_gpio_mode(GPIO_BUTTON_LED_PIN, GPIO_BUTTON_LED_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_BUTTON_LED_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_BUTTON_LED_PIN,GPIO_OUT_ONE);
	}
	else
	{
		mt_set_gpio_mode(GPIO_BUTTON_LED_PIN, GPIO_BUTTON_LED_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_BUTTON_LED_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_BUTTON_LED_PIN,GPIO_OUT_ZERO);
	}
}
#endif
static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
#if defined(CONFIG_T89_PROJ)
	#if defined(CONFIG_T89D_KLT_PROJ)
	{"red",               MT65XX_LED_MODE_GPIO, (long)rgb_set_red_by_gpio,{0}},
	{"green",             MT65XX_LED_MODE_GPIO, (long)rgb_set_green_by_gpio,{0}},
	{"blue",              MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	#elif defined(CONFIG_T89J_JK_PROJ) || defined(CONFIG_T89N_TS_PROJ)||defined(CONFIG_T89P_DG_PROJ)
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	#else
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"green",             MT65XX_LED_MODE_GPIO, (long)rgb_set_green_by_gpio,{0}},
	{"blue",              MT65XX_LED_MODE_GPIO, (long)rgb_set_blue_by_gpio,{0}},
	#endif
#elif defined(CONFIG_T925_PROJ)
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
    #if defined(CONFIG_T925_TIEPIAN)
	{"green",             MT65XX_LED_MODE_GPIO, (long)rgb_set_green_by_gpio,{0}},
    #elif defined(CONFIG_T925D_NYX_M18) // used for sub flashlight
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
    #else
	{"green",             MT65XX_LED_MODE_GPIO, (long)rgb_set_green_by_gpio,{0}},
    #endif
	{"blue",              MT65XX_LED_MODE_GPIO, (long)rgb_set_blue_by_gpio,{0}},
#elif defined(CONFIG_T93_PROJ)
#if defined(CONFIG_RGB_LED_SUPPORT)
#if defined(CONFIG_T93AA_PROJ)||defined(CONFIG_T93BQ_PROJ)
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"green",             MT65XX_LED_MODE_GPIO, (long)rgb_set_blue_by_gpio,{0}},
	{"blue",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},             
#else
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"green",             MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	{"blue",              MT65XX_LED_MODE_GPIO, (long)rgb_set_blue_by_gpio,{0}},
#endif
#else
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
#endif

#elif defined(CONFIG_T985_PROJ)
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"blue",              MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0}},
	{"green",             MT65XX_LED_MODE_GPIO, (long)rgb_set_green_by_gpio,{0}},
#elif defined(CONFIG_T96_PROJ)
	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
#elif defined(CONFIG_T99_PROJ)
#if defined(CONFIG_T99C_JQ_PROJ)
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
#else

	{"red",               MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK1,{0}},
	{"green",             MT65XX_LED_MODE_GPIO, (long)rgb_set_green_by_gpio,{0}},
	{"blue",              MT65XX_LED_MODE_GPIO, (long)rgb_set_blue_by_gpio,{0}},
#endif
#else
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
#endif
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
#if defined(CONFIG_T93_PROJ)||defined(CONFIG_T985_PROJ)
		{"button-backlight",	MT65XX_LED_MODE_GPIO, (long)rgb_set_Button_led_by_gpio,{0}},
#else
		{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK0,{0,0,0,0,0}},
#endif

#if defined(CONFIG_T25_PROJ)
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, cust_setbacklight_sgm3138},
#else
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (long)disp_bls_set_backlight,{0}},
#endif
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

