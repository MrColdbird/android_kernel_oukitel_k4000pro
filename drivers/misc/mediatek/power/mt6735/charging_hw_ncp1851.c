/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <mach/charging.h>
#include "ncp1851.h"
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/upmu_hw.h>
#include <mach/upmu_sw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
//#include <mach/mt_board_type.h>  //mtk71259 build error  20140523
#include <linux/spinlock.h>
#include <cust_charging.h>
#include <mach/mt_gpio.h>
#include <linux/wakelock.h>
#include <mach/mt6311.h>

 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

#if defined(CONFIG_T89N_TS_PROJ)
#define DIS_AICL_EN
#endif
 // ============================================================ //
 //global variable
 // ============================================================ //

int gpio_off_dir  = GPIO_DIR_OUT;
int gpio_off_out  = GPIO_OUT_ONE;
int gpio_on_dir   = GPIO_DIR_OUT;
int gpio_on_out   = GPIO_OUT_ZERO;

/*#ifndef GPIO_CHR_SPM_PIN GPIO_SWCHARGER_EN_PIN
#define GPIO_CHR_SPM_PIN 65
#endif  */
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0

    #if defined(GPIO_PWR_AVAIL_WLC)
        kal_uint32 wireless_charger_gpio_number = GPIO_PWR_AVAIL_WLC; 
    #else
        kal_uint32 wireless_charger_gpio_number = 0; 
    #endif
    
#endif

static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

kal_bool charging_type_det_done = KAL_TRUE;

//As 82 platform mach/charging.h could not cover all voltage setting, just hardcoded below settings
const kal_uint32 VBAT_CV_VTH[]=
{
	3300000,    3325000,    3350000,    3375000,
	3400000,    3425000,    3450000,    3475000,
	3500000,    3525000,    3550000,    3575000,
	3600000,    3625000,    3650000,    3675000,
	3700000,    3725000,    3750000,    3775000,
	3800000,    3825000,    3850000,    3875000,
	3900000,    3925000,    3950000,    3975000,
	4000000,    4025000,    4050000,    4075000,
	4100000,    4125000,    4150000,    4175000,
	4200000,    4225000,    4250000,    4275000,
	4300000,    4325000,    4350000,    4375000,
	4400000,    4425000,    4450000,    4475000,
};

const kal_uint32 CS_VTH[]=
{
	CHARGE_CURRENT_400_00_MA,   CHARGE_CURRENT_500_00_MA,	CHARGE_CURRENT_600_00_MA, CHARGE_CURRENT_700_00_MA,
	CHARGE_CURRENT_800_00_MA,   CHARGE_CURRENT_900_00_MA,	CHARGE_CURRENT_1000_00_MA, CHARGE_CURRENT_1100_00_MA,
	CHARGE_CURRENT_1200_00_MA,   CHARGE_CURRENT_1300_00_MA,	CHARGE_CURRENT_1400_00_MA, CHARGE_CURRENT_1500_00_MA,
	CHARGE_CURRENT_1600_00_MA,
}; 

 const kal_uint32 INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_500_00_MA,  CHARGE_CURRENT_900_00_MA,
	 CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_MAX
 }; 

 const kal_uint32 VCDT_HV_VTH[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };

 // ============================================================ //
 // function prototype
 // ============================================================ //
 
 
 // ============================================================ //
 //extern variable
 // ============================================================ //
 
 // ============================================================ //
 //extern function
 // ============================================================ //
 extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
 extern bool mt_usb_is_device(void);
 extern void Charger_Detect_Init(void);
 extern void Charger_Detect_Release(void);
extern int hw_charging_get_charger_type(void);
 extern void mt_power_off(void);
 
 // ============================================================ //
 kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
 kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

    battery_xlog_printk(BAT_LOG_CRTI, "array_size = %d \r\n", array_size);
    
	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
		{
				return i;
		}
	}

    battery_xlog_printk(BAT_LOG_CRTI, "NO register value match. val=%d\r\n", val);
	//TODO: ASSERT(0);	// not find the value
	return 0;
}

//lisong 2014-8-7 [NO BUGID][input_cs_vth should large than the current setting]start
static kal_uint32 bmt_find_closest_level_high(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
{
    kal_uint32 i;
    kal_uint32 max_value_in_last_element;

    if(pList[0] < pList[1])
        max_value_in_last_element = KAL_TRUE;
    else
        max_value_in_last_element = KAL_FALSE;

    if(max_value_in_last_element == KAL_TRUE)
    {
        if(level > pList[number-1])
            return pList[number-1];
        
        for(i = (number-1); i != 0; i--)	 //max value in the last element
        {
            if((level <= pList[i]) && (level > pList[i-1]))	  
                return pList[i];
        }

        battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
        return pList[0];
        //return CHARGE_CURRENT_0_00_MA;
    }
    else
    {
        if(level > pList[0])
            return pList[0];
        
        for(i = 0; i<(number-1); i++)  // max value in the first element
        {
            if((level <= pList[i]) && (level > pList[i+1]))	
                return pList[i];
        }

        battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n"); 	 
        return pList[number -1];
        //return CHARGE_CURRENT_0_00_MA;
    }
}
//lisong 2014-8-7 [NO BUGID][input_cs_vth should large than the current setting]end

 static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
 {
	 kal_uint32 i;
	 kal_uint32 max_value_in_last_element;
 
	 if(pList[0] < pList[1])
		 max_value_in_last_element = KAL_TRUE;
	 else
		 max_value_in_last_element = KAL_FALSE;
 
	 if(max_value_in_last_element == KAL_TRUE)
	 {
		 for(i = (number-1); i != 0; i--)	 //max value in the last element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

 		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		 return pList[0];
		 //return CHARGE_CURRENT_0_00_MA;
	 }
	 else
	 {
		 for(i = 0; i< number; i++)  // max value in the first element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n"); 	 
		 return pList[number -1];
  		 //return CHARGE_CURRENT_0_00_MA;
	 }
 }

#if 0
static void hw_bc11_dump_register(void)
{
	kal_uint32 reg_val = 0;
	kal_uint32 reg_num = MT6328_CHR_CON20;
	kal_uint32 i = 0;

	for(i=reg_num ; i<=MT6328_CHR_CON21 ; i+=2)
	{
		reg_val = upmu_get_reg_value(i);
		battery_xlog_printk(BAT_LOG_FULL, "Chr Reg[0x%x]=0x%x \r\n", i, reg_val);
	}
}


 static void hw_bc11_init(void)
 {
	 Charger_Detect_Init();

	 //RG_bc11_BIAS_EN=1
    bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,1);
    //RG_bc11_VSRC_EN[1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0);
    //RG_bc11_VREF_VTH = [1:0]=00
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0);
    //RG_bc11_CMP_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0);
    //RG_bc11_IPU_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0);
    //RG_bc11_IPD_EN[1.0] = 00
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0);
    //bc11_RST=1
    bc11_set_register_value(PMIC_RG_BC11_RST,1);
    //bc11_BB_CTRL=1
    bc11_set_register_value(PMIC_RG_BC11_BB_CTRL,1);

 	 //msleep(10);
 	 mdelay(50);

	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
    		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_init() \r\n");
		hw_bc11_dump_register();
	 }	
	 
 }
 
 
 static U32 hw_bc11_DCD(void)
 {
	 U32 wChargerAvail = 0;
	 
	//RG_bc11_IPU_EN[1.0] = 10
    bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2);  
    //RG_bc11_IPD_EN[1.0] = 01
    bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
    //RG_bc11_VREF_VTH = [1:0]=01
    bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
    //RG_bc11_CMP_EN[1.0] = 10
    bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x2);
	//msleep(20);
	mdelay(80);

	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	 
	 if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	 {
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_DCD() \r\n");
		hw_bc11_dump_register();
	 }

	//RG_bc11_IPU_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
	//RG_bc11_IPD_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
	//RG_bc11_CMP_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
	//RG_bc11_VREF_VTH = [1:0]=00
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
	
	 return wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepA1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 10
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2);//upmu_set_rg_bc11_ipu_en(0x2);
	//RG_BC11_VREF_VTH = [1:0]=10
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x2);//upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 10
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x2);//upmu_set_rg_bc11_cmp_en(0x2);
	
	mdelay(80);
 
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);//upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);//upmu_set_rg_bc11_cmp_en(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepB1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	//upmu_set_rg_bc11_ipu_en(0x1);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);//upmu_set_rg_bc11_ipd_en(0x1);      
	//RG_BC11_VREF_VTH = [1:0]=10
	//upmu_set_rg_bc11_vref_vth(0x2);
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);//upmu_set_rg_bc11_vref_vth(0x0);
	//RG_BC11_CMP_EN[1.0] = 01
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);//upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);//upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);//upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);//upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepC1(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_IPU_EN[1.0] = 01
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x1);//upmu_set_rg_bc11_ipu_en(0x1);
	//RG_BC11_VREF_VTH = [1:0]=10
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x2);//upmu_set_rg_bc11_vref_vth(0x2);
	//RG_BC11_CMP_EN[1.0] = 01
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);//upmu_set_rg_bc11_cmp_en(0x1);
 
	//msleep(80);
	mdelay(80);
 
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepC1() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);//upmu_set_rg_bc11_ipu_en(0x0);
	//RG_BC11_CMP_EN[1.0] = 00
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);//upmu_set_rg_bc11_cmp_en(0x0);
	//RG_BC11_VREF_VTH = [1:0]=00
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);//upmu_set_rg_bc11_vref_vth(0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepA2(void)
 {
	U32 wChargerAvail = 0;
	  
	//RG_BC11_VSRC_EN[1.0] = 10 
   bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x2);
	//RG_BC11_IPD_EN[1:0] = 01
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x1);
	//RG_BC11_VREF_VTH = [1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
	//RG_BC11_CMP_EN[1.0] = 01
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);
 
	//msleep(80);
	mdelay(80);
 
   wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepA2() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_VSRC_EN[1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
	//RG_BC11_IPD_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
	//RG_BC11_CMP_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
 
	return  wChargerAvail;
 }
 
 
 static U32 hw_bc11_stepB2(void)
 {
	U32 wChargerAvail = 0;
 
	//RG_BC11_IPU_EN[1:0]=10
   bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x2); 
	//RG_BC11_VREF_VTH = [1:0]=10
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x1);
	//RG_BC11_CMP_EN[1.0] = 01
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x1);
 
	//msleep(80);
	mdelay(80);
 
   wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
 
	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_stepB2() \r\n");
		hw_bc11_dump_register();
	}
 
	//RG_BC11_IPU_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0); 
	//RG_BC11_CMP_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0); 
	//RG_BC11_VREF_VTH = [1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0); 
 
	return  wChargerAvail;
 }
 
 
 static void hw_bc11_done(void)
 {
	//RG_BC11_VSRC_EN[1:0]=00
   bc11_set_register_value(PMIC_RG_BC11_VSRC_EN,0x0);
	//RG_BC11_VREF_VTH = [1:0]=0
   bc11_set_register_value(PMIC_RG_BC11_VREF_VTH,0x0);
	//RG_BC11_CMP_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_CMP_EN,0x0);
	//RG_BC11_IPU_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPU_EN,0x0);
	//RG_BC11_IPD_EN[1.0] = 00
   bc11_set_register_value(PMIC_RG_BC11_IPD_EN,0x0);
	//RG_BC11_BIAS_EN=0
   bc11_set_register_value(PMIC_RG_BC11_BIAS_EN,0x0);
 
	Charger_Detect_Release();

	if(Enable_BATDRV_LOG == BAT_LOG_FULL)
	{
		battery_xlog_printk(BAT_LOG_FULL, "hw_bc11_done() \r\n");
		hw_bc11_dump_register();
	}
    
 }
#endif

 static kal_uint32 charging_hw_init(void *data)
{
    kal_uint32 ncp1851_status;
 	kal_uint32 status = STATUS_OK;

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:ncp1851] ChargerHwInit_ncp1851\n" );
    }

    ncp1851_status = ncp1851_get_chip_status();

   // bc11_set_register_value(PMIC_RG_BC11_BB_CTRL,1);//upmu_set_rg_bc11_bb_ctrl(1);    //BC11_BB_CTRL    
   // bc11_set_register_value(PMIC_RG_BC11_RST,1);//upmu_set_rg_bc11_rst(1);        //BC11_RST

    ncp1851_set_otg_en(0x0);
    ncp1851_set_ntc_en(0x0);
    ncp1851_set_trans_en(0);
    ncp1851_set_tj_warn_opt(0x0);
    ncp1851_set_jeita_opt(0x0);
//  ncp1851_set_int_mask(0x0); //disable all interrupt
    ncp1851_set_int_mask(0x1); //enable all interrupt for boost mode status monitor
   // ncp1851_set_tchg_rst(0x1); //reset charge timer
#ifdef NCP1851_PWR_PATH
    ncp1851_set_pwr_path(0x1);
#else
    ncp1851_set_pwr_path(0x0);
#endif
#ifdef DIS_AICL_EN
    ncp1851_set_aicl_en(0x0); //enable AICL as PT team suggest
#else
    ncp1851_set_aicl_en(0x1); //enable AICL as PT team suggest
#endif
	ncp1851_set_chgto_dis(0x1); //disable charge timer
    if((ncp1851_status == 0x8) || (ncp1851_status == 0x9) || (ncp1851_status == 0xA)) //WEAK WAIT, WEAK SAFE, WEAK CHARGE
        ncp1851_set_ctrl_vbat(0x1C); //VCHG = 4.0V
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
        ncp1851_set_ctrl_vbat(0x2A); //VCHG = 4.35
#else
    else if(ncp1851_status == 0x4)
        ncp1851_set_ctrl_vbat(0x28); //VCHG = 4.3V to decrease charge time
    else
        ncp1851_set_ctrl_vbat(0x24); //VCHG = 4.2V
#endif

    ncp1851_set_ieoc(0x0);//terminate current = 100mA   (0x4); // terminate current = 200mA for ICS optimized suspend power
    
#ifdef SUPER_HIGH_BATTERY_VOLTAGE_SUPPORT
        ncp1851_set_ctrl_vbat(0x2C); //VCHG = 4.4
        ncp1851_set_ieoc(0x5);//terminate current = 100mA   (0x4); // terminate current = 200mA for ICS optimized suspend power
#endif

    ncp1851_set_iweak(0x3); //weak charge current = 300mA

    ncp1851_set_ctrl_vfet(0x3); // VFET = 3.4V
    ncp1851_set_vchred(0x2); //reduce 200mV (JEITA)
    ncp1851_set_ichred(0x0); //reduce 400mA (JEITA)
    ncp1851_set_batcold(0x5);
    ncp1851_set_bathot(0x3);
    ncp1851_set_batchilly(0x0);
    ncp1851_set_batwarm(0x0);
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
		if(wireless_charger_gpio_number!=0)
		{
			mt_set_gpio_mode(wireless_charger_gpio_number,0); // 0:GPIO mode
			mt_set_gpio_dir(wireless_charger_gpio_number,0); // 0: input, 1: output
		}
#endif
	return status;
}

 static kal_uint32 charging_dump_register(void *data)
 {
 	kal_uint32 status = STATUS_OK;

    battery_xlog_printk(BAT_LOG_CRTI, "charging_dump_register\r\n");

    ncp1851_dump_register();
   	
	return status;
 }	


 static kal_uint32 charging_enable(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable)
	{
		ncp1851_set_chg_en(0x1); // charger enable
#ifdef GPIO_SWCHARGER_EN_PIN
		mt_set_gpio_mode(GPIO_SWCHARGER_EN_PIN, GPIO_MODE_00);
		mt_set_gpio_dir(GPIO_SWCHARGER_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_SWCHARGER_EN_PIN, GPIO_OUT_ONE);
#endif
	}
	else
	{
#if defined(CONFIG_USB_MTK_HDRC_HCD)
   		if(mt_usb_is_device())
#endif 			
    	{
			ncp1851_set_chg_en(0x0); // charger disable
    	}
	}
		
	return status;
 }


 static kal_uint32 charging_set_cv_voltage(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);	
	
#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT)
		cv_value = 4350000;
#else
	    //use nearest value, ncp1851 hit 4.2v level
		cv_value = 4200000;
#endif

#ifdef SUPER_HIGH_BATTERY_VOLTAGE_SUPPORT
        cv_value = 4400000;
#endif
	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH), cv_value);
	ncp1851_set_ctrl_vbat(register_value);
	return status;
 } 	


 static kal_uint32 charging_get_current(void *data)
 {
    kal_uint32 status = STATUS_OK;
    
    kal_uint8 ret_val=0;    
	    
    //Get current level
	ret_val = ncp1851_get_ichg();
    //ncp1851_read_interface(NCP1851_CON15, &ret_val, CON15_ICHG_MASK, CON15_ICHG_SHIFT);						    
    //Parsing
    ret_val = (ret_val*100) + 400;
	
    return status;
 }


 static kal_uint32 charging_set_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = *(kal_uint32 *)data;
	
	array_size = GETARRAYNUM(CS_VTH);
	set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
	register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
	//current set by SW and disable automatic charge current
	
#ifdef DIS_AICL_EN
    ncp1851_set_aicl_en(0x0); //enable AICL as PT team suggest
#else
	ncp1851_set_aicl_en(0x1); //enable AICL
#endif
	ncp1851_set_ichg(register_value);       	
	
	return status;
 } 	
 

 static kal_uint32 charging_set_input_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
    
	array_size = GETARRAYNUM(INPUT_CS_VTH);
//lisong 2014-8-7 [NO BUGID][input_cs_vth should large than the current setting]start    
//	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(kal_uint32 *)data);
    set_chr_current = bmt_find_closest_level_high(INPUT_CS_VTH, array_size, *(kal_uint32 *)data);
//lisong 2014-8-7 [NO BUGID][input_cs_vth should large than the current setting]end
	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);	
    if(register_value==4)
    {
    	register_value=3;
    }
	ncp1851_set_iinlim(register_value);
	ncp1851_set_iinset_pin_en(0x0); //Input current limit and AICL control by I2C
	ncp1851_set_iinlim_en(0x1); //enable input current limit
	
#ifdef DIS_AICL_EN
    ncp1851_set_aicl_en(0x0); //enable AICL as PT team suggest
#else
	ncp1851_set_aicl_en(0x1); //enable AICL
#endif

	return status;
 } 	


 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = ncp1851_get_chip_status();
	//check whether chargeing DONE
	if ((ret_val == 0x6) || (ret_val == 0x7))
	{
		*(kal_uint32 *)data = KAL_TRUE;
	} else {
		*(kal_uint32 *)data = KAL_FALSE;
	}
	
	return status;
 } 	

void kick_charger_wdt(void)
{
	ncp1851_set_wdto_dis(0x0);

}

 static kal_uint32 charging_reset_watch_dog_timer(void *data)
 {
	 kal_uint32 status = STATUS_OK;
 
     battery_xlog_printk(BAT_LOG_CRTI, "charging_reset_watch_dog_timer\r\n");
 
	 kick_charger_wdt();
	 return status;
 }
 
 
  static kal_uint32 charging_set_hv_threshold(void *data)
  {
	 kal_uint32 status = STATUS_OK;
 
	 kal_uint32 set_hv_voltage;
	 kal_uint32 array_size;
	 kal_uint16 register_value;
	 kal_uint32 voltage = *(kal_uint32*)(data);
	 
	 array_size = GETARRAYNUM(VCDT_HV_VTH);
	 set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	 register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
	 pmic_set_register_value(PMIC_RG_VCDT_HV_VTH,register_value);
	 return status;
  }
 
 
  static kal_uint32 charging_get_hv_status(void *data)
  {
		  kal_uint32 status = STATUS_OK;
	  
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		  *(kal_bool*)(data) = 0;
		  printk("[charging_get_hv_status] charger ok for bring up.\n");
#else		 
		 *(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif
		   
		  return status;

  }


 static kal_uint32 charging_get_battery_status(void *data)
 {
		 kal_uint32 status = STATUS_OK;
	 	 kal_uint32 val = 0;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		 *(kal_bool*)(data) = 0; // battery exist
		 printk("[charging_get_battery_status] battery exist for bring up.\n");
#else
	val=pmic_get_register_value(PMIC_BATON_TDET_EN);
	pr_info("[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
	pmic_set_register_value(PMIC_BATON_TDET_EN,1);
	pmic_set_register_value(PMIC_RG_BATON_EN,1);
	*(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else {
		*(kal_bool*)(data) =  KAL_FALSE;
	}
#endif
		  
		 return status;

 }


 static kal_uint32 charging_get_charger_det_status(void *data)
 {
		 kal_uint32 status = STATUS_OK;
		 kal_uint32 val = 0;
	 
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		 val = 1;
		 printk("[charging_get_charger_det_status] charger exist for bring up.\n"); 
#else    
	 	 val = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif
		 
		 *(kal_bool*)(data) = val;
		 if(val == 0)
			 g_charger_type = CHARGER_UNKNOWN;
			   
		 return status;

 }


kal_bool charging_type_detection_done(void)
{
	 return charging_type_det_done;
}

//extern CHARGER_TYPE hw_charger_type_detection(void);

 static kal_uint32 charging_get_charger_type(void *data)
 {
	 kal_uint32 status = STATUS_OK;
	 CHARGER_TYPE charger_type = CHARGER_UNKNOWN;
#if defined(CONFIG_POWER_EXT)
	 *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
	 charging_type_det_done = KAL_FALSE;	
	 charger_type = hw_charging_get_charger_type();
	 printk("charging_get_charger_type = %d\r\n", charger_type);
 
	 *(CHARGER_TYPE*)(data) = charger_type;
	 charging_type_det_done = KAL_TRUE;
	 g_charger_type = *(CHARGER_TYPE*)(data);
#endif
    return status;

	
#if 0 
		kal_uint32 status = STATUS_OK;
		 
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
		*(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
		
    #if defined(MTK_WIRELESS_CHARGER_SUPPORT)
		int wireless_state = 0;
		if(wireless_charger_gpio_number!=0)
		{
			wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
			if(wireless_state == WIRELESS_CHARGER_EXIST_STATE)
			{
				*(CHARGER_TYPE*)(data) = WIRELESS_CHARGER;
				printk("WIRELESS_CHARGER!\n");
				return status;
			}
		}
		else
		{
			printk("wireless_charger_gpio_number=%d\n", wireless_charger_gpio_number);
		}
		
		if(g_charger_type!=CHARGER_UNKNOWN && g_charger_type!=WIRELESS_CHARGER)
		{
			*(CHARGER_TYPE*)(data) = g_charger_type;
			printk("return %d!\n", g_charger_type);
			return status;
		}
    #endif
		
		if(is_chr_det()==0)
		{
			g_charger_type = CHARGER_UNKNOWN; 
			*(CHARGER_TYPE*)(data) = CHARGER_UNKNOWN;
			printk("[charging_get_charger_type] return CHARGER_UNKNOWN\n");
			return status;
		}
		
		charging_type_det_done = KAL_FALSE;
	
		*(CHARGER_TYPE*)(data) = hw_charging_get_charger_type();
		//*(CHARGER_TYPE*)(data) = STANDARD_HOST;
		//*(CHARGER_TYPE*)(data) = STANDARD_CHARGER;
	
		charging_type_det_done = KAL_TRUE;
	
		g_charger_type = *(CHARGER_TYPE*)(data);
		
#endif
	
		return status;
#endif
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
	kal_uint32 status = STATUS_OK;
	*(kal_bool*)(data) = KAL_FALSE;
	return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
    kal_uint32 status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)    
#else 
    printk("charging_set_platform_reset\n");
 
    arch_reset(0,NULL);
#endif
        
    return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
  
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)   
#else   
    *(kal_uint32*)(data) = get_boot_mode();

    printk("get_boot_mode=%d\n", get_boot_mode());
#endif
         
    return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
    printk("charging_set_power_off\n");
    mt_power_off();
#endif
         
    return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
	kal_uint32 status = STATUS_UNSUPPORTED;

	return status;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
#ifdef CAN_NOT_RECHARGE
	ncp1851_set_reset(0x1);
#endif
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;	
}

 static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
 };

 
 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
 kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 kal_int32 status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
 
	 return status;
 }


