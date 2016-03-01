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
 *
 * 03 05 2015 wy.chuang
 * [ALPS01921641] [L1_merge] for PMIC and charging
 * .
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <mach/charging.h>
#include "ncp1854.h"
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


 // ============================================================ //
 //global variable
 // ============================================================ //

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


//As 82 platform mach/charging.h could not cover all voltage setting, just hardcoded below settings
const kal_uint32 VBAT_CV_VTH_SUB[]=
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

/*
const kal_uint32 CS_VTH[]=
{
	CHARGE_CURRENT_450_00_MA,   CHARGE_CURRENT_550_00_MA,	CHARGE_CURRENT_650_00_MA, CHARGE_CURRENT_750_00_MA,
	CHARGE_CURRENT_850_00_MA,   CHARGE_CURRENT_950_00_MA,	CHARGE_CURRENT_1050_00_MA, CHARGE_CURRENT_1150_00_MA,
	CHARGE_CURRENT_1250_00_MA,   CHARGE_CURRENT_1350_00_MA,	CHARGE_CURRENT_1450_00_MA, CHARGE_CURRENT_1550_00_MA,
	CHARGE_CURRENT_1650_00_MA,   CHARGE_CURRENT_1750_00_MA,	CHARGE_CURRENT_1850_00_MA, CHARGE_CURRENT_1950_00_MA
}; 
*/

/* hardcoded current define which defined in NCP1854 IC spec, as common define doesnot cover all define
 * double confirmed with onsemi register set in spec has issue,below is the correct setting */
const kal_uint32 CS_VTH_SUB[]=
{
    45000,  50000,  60000,  70000,
	80000,  90000,  100000, 110000,
	120000, 130000, 140000, 150000,
	160000, 170000, 180000, 190000
};

const kal_uint32 INPUT_CS_VTH_SUB[]=
 {
	 CHARGE_CURRENT_100_00_MA,  CHARGE_CURRENT_500_00_MA
 }; 

const kal_uint32 INPUT_CS_VTH_TA_SUB[]=
 {
	 CHARGE_CURRENT_600_00_MA,  CHARGE_CURRENT_700_00_MA,  CHARGE_CURRENT_800_00_MA,
	 CHARGE_CURRENT_900_00_MA,  CHARGE_CURRENT_1000_00_MA,  CHARGE_CURRENT_1100_00_MA,
	 CHARGE_CURRENT_1200_00_MA,  CHARGE_CURRENT_1300_00_MA,  CHARGE_CURRENT_1400_00_MA,
	 CHARGE_CURRENT_1500_00_MA,  CHARGE_CURRENT_1600_00_MA,  170000,
	 180000, 190000, 200000
 }; 

const kal_uint32 VCDT_HV_VTH_SUB[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };


/* hardcoded current define which defined in NCP1854 IC spec, as common define doesnot cover all define
 * double confirmed with onsemi register set in spec has issue,below is the correct setting */

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
extern kal_uint32 mt6311_get_chip_id(void);
extern int is_mt6311_exist(void);
extern int is_mt6311_sw_ready(void);
extern void ncp1854_sub_set_otg_en(kal_uint32 val);
extern void ncp1854_sub_dump_register_sub(void);


kal_uint32 sub_current_high_flag = 0;
 // ============================================================ //

#ifdef CONFIG_HCT_NCP_HIGHER_CV
extern kal_uint32  temp_cv_reg_val;
extern kal_uint32  pre_cv_reg_val;

#endif

 kal_uint32 ncp1854_charging_hw_init_sub(void *data)
{
   //static kal_uint32 run_hw_init_once_flag=1;

    kal_uint32 ncp1854_status;
 	kal_uint32 status = STATUS_OK;

    if (Enable_BATDRV_LOG == 1) {
        pr_notice("[BATTERY:ncp1854] ChargerHwInit_ncp1854_sub\n" );
    }

    ncp1854_status = ncp1854_sub_get_chip_status();
#if 0//def DISABLE_NCP1854_FACTORY_MODE
	ncp1854_set_fctry_mode(0x0);//if you want disable factory mode,the FTRY pin and FCTRY_MOD_REG need differ
#endif
    ncp1854_sub_set_otg_en(0x0);
    ncp1854_sub_set_trans_en(0);
    ncp1854_sub_set_tj_warn_opt(0x0);//set at disabled, by MT6325 BATON
//  ncp1854_set_int_mask(0x0); //disable all interrupt
    ncp1854_sub_set_int_mask(0x1); //enable all interrupt for boost mode status monitor
   // ncp1854_set_tchg_rst(0x1); //reset charge timer
#ifdef NCP1854_PWR_PATH
    ncp1854_sub_set_pwr_path(0x1);
#else
    ncp1854_sub_set_pwr_path(0x0);
#endif

   ncp1854_sub_set_chgto_dis(0x1); //disable charge timer

#ifdef CONFIG_HCT_NCP_HIGHER_CV
           ncp1854_sub_set_ctrl_vbat(temp_cv_reg_val); //VCHG = 4.35V to decrease charge time
#else
           
#if  defined (HIGH_BATTERY_VOLTAGE_SUPPORT)
            ncp1854_sub_set_ctrl_vbat(0x2B); //VCHG = 4.375V to decrease charge time
#elif defined(CONFIG_HIGH_4400_BATTERY_VOLTAGE_SUPPORT)
            ncp1854_sub_set_ctrl_vbat(0x2D); //VCHG = 4.425V to decrease charge time
#else
            ncp1854_sub_set_ctrl_vbat(0x25); //VCHG = 4.225V to decrease charge time
#endif
#endif


   	//if(run_hw_init_once_flag)
	//{
#ifdef CONFIG_HCT_NCP_HIGHER_CV
         ncp1854_sub_set_ieoc(0x2); // terminate current = 150mA for ICS optimized suspend power
#else
         ncp1854_sub_set_ieoc(0x0); // terminate current = 100mA for ICS optimized suspend power
#endif
         ncp1854_sub_set_iweak(0x3); //weak charge current = 300mA
      // run_hw_init_once_flag=0;
	//}

	ncp1854_sub_set_aicl_en(0x1); //enable AICL as PT team suggest

	ncp1854_sub_set_iinset_pin_en(0x0); //Input current limit and AICL control by I2C

    ncp1854_sub_set_ctrl_vfet(0x3); // VFET = 3.4V

	return status;
}

 kal_uint32 ncp1854_charging_dump_register_sub(void *data)
 {
 	kal_uint32 status = STATUS_OK;

    pr_notice("charging_dump_register_sub\r\n");

    ncp1854_sub_dump_register_sub();
   	
	return status;
 }	

 kal_uint32 ncp1854_charging_disable_sub()
{
     ncp1854_sub_set_chg_en(0x0); // charger disable

}

 kal_uint32 ncp1854_charging_enable_sub(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable)
	{
		ncp1854_sub_set_chg_en(0x1); // charger enable
		//Set SPM = 1		
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
			ncp1854_sub_set_chg_en(0x0); // charger disable
    	}
	}
		
	return status;
 }


 kal_uint32 ncp1854_charging_set_cv_voltage_sub(unsigned short reg_value)
 {

    kal_uint32 status = STATUS_OK;

    ncp1854_sub_set_ctrl_vbat(reg_value);

    return status;
 } 	

extern kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val);
extern kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level);

extern kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val);




 kal_uint32 ncp1854_charging_get_current_sub(void)
 {
    kal_uint32 status = STATUS_OK;
    kal_uint32 array_size; 
    kal_uint8 ret_val=0;    
    kal_uint32    data=0;
    //Get current level
	//ret_val = ncp1854_get_ichg();
    //ncp1854_read_interface(NCP1854_CON15, &ret_val, CON15_ICHG_MASK, CON15_ICHG_SHIFT);						    
    //Parsing
   // ret_val = (ret_val*100) + 400;
	
    array_size = GETARRAYNUM(CS_VTH_SUB);
    ret_val = ncp1854_sub_get_ichg();	//IINLIM
    if(sub_current_high_flag==1)
	data = charging_value_to_parameter(CS_VTH_SUB,array_size,ret_val)+ 160000;
	else
    data = charging_value_to_parameter(CS_VTH_SUB,array_size,ret_val);
	
    return data;
 }


 kal_uint32 charging_set_current_sub(kal_uint32 data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = data;
	//kal_uint32 sub_current_high_flag = 0;

       pr_notice("ncp:charging_set_current_sub, current=%d\r\n",current_value);

    
	array_size = GETARRAYNUM(CS_VTH_SUB);
	if (current_value <=190000)
	{
	    set_chr_current = bmt_find_closest_level(CS_VTH_SUB, array_size, current_value);
	    register_value = charging_parameter_to_value(CS_VTH_SUB, array_size ,set_chr_current);
		sub_current_high_flag = 0x0;
	} else {
	    set_chr_current = bmt_find_closest_level(CS_VTH_SUB, array_size, current_value - 160000);
	    register_value = charging_parameter_to_value(CS_VTH_SUB, array_size ,set_chr_current);
		sub_current_high_flag = 0x1;
	}

	//current set by SW and disable automatic charge current
	//ncp1854_set_aicl_en(0x0); //disable AICL
	//set which register first? mmz
	ncp1854_sub_set_ichg_high(sub_current_high_flag);
	ncp1854_sub_set_ichg(register_value);       	
	
	return status;
 } 	
 

 kal_uint32 charging_set_input_current_sub(kal_uint32 data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = data;

       pr_notice("ncp:charging_set_input_current_sub, current=%d\r\n",data);
       
	if (current_value < 60000)
	{
	    array_size = GETARRAYNUM(INPUT_CS_VTH_SUB);
	    set_chr_current = bmt_find_closest_level(INPUT_CS_VTH_SUB, array_size, current_value);
	    register_value = charging_parameter_to_value(INPUT_CS_VTH_SUB, array_size ,set_chr_current);	
	    ncp1854_sub_set_iinlim(register_value);
	    ncp1854_sub_set_iinlim_ta(0x0);
	} else {
	    array_size = GETARRAYNUM(INPUT_CS_VTH_TA_SUB);
	    set_chr_current = bmt_find_closest_level(INPUT_CS_VTH_TA_SUB, array_size, current_value);
	    register_value = charging_parameter_to_value(INPUT_CS_VTH_TA_SUB, array_size ,set_chr_current);	
	    ncp1854_sub_set_iinlim_ta(register_value);
	}
        
	//ncp1854_set_iinset_pin_en(0x0); //Input current limit and AICL control by I2C
        if(current_value < 180000)
	    ncp1854_sub_set_iinlim_en(0x1); //enable input current limit
        else	
            ncp1854_sub_set_iinlim_en(0x0); //enable input current limit
	//ncp1854_sub_set_aicl_en(0x0); //disable AICL

	return status;
 }


 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = ncp1854_sub_get_chip_status();
	//check whether chargeing DONE
	if (ret_val == 0x6)
	{
		*(kal_uint32 *)data = KAL_TRUE;
	} else {
		*(kal_uint32 *)data = KAL_FALSE;
	}
	
	return status;
 } 	

void kick_charger_wdt_sub(void)
{
	ncp1854_sub_set_wdto_dis(0x1);
	//ncp1854_set_wdto_dis(0x0);
}





