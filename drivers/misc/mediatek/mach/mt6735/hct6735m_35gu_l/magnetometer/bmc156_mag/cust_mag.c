#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

#if defined(CONFIG_T925_PROJ)||defined(CONFIG_T985_PROJ)||defined(CONFIG_T93_PROJ)
	#define BMC156_MAG_I2C_BUS_NUM    2
	#define BMC156_MAG_DIRECTION      5
#elif defined(CONFIG_T92_PROJ)
	#define BMC156_MAG_I2C_BUS_NUM		2
	#define BMC156_MAG_DIRECTION		3
#elif defined(CONFIG_T99_PROJ)
	#define BMC156_MAG_I2C_BUS_NUM		2
	#define BMC156_MAG_DIRECTION		0
#else
	#define BMC156_MAG_I2C_BUS_NUM    2
	#define BMC156_MAG_DIRECTION      4
#endif

static struct mag_hw cust_mag_hw = {
    .i2c_num = BMC156_MAG_I2C_BUS_NUM,
    .direction = BMC156_MAG_DIRECTION,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .is_batch_supported = false,
};
struct mag_hw* get_cust_mag_hw(void) 
{
    return &cust_mag_hw;
}
