#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>

#if defined(CONFIG_T925_PROJ)||defined(CONFIG_T93_PROJ)||defined(CONFIG_T985_PROJ)
	#define BMC156_ACC_I2C_BUS_NUM    2
	#define BMC156_ACC_DIRECTION      5
#elif defined(CONFIG_T92_PROJ)
	#define BMC156_ACC_I2C_BUS_NUM		2
	#define BMC156_ACC_DIRECTION		3
#elif defined(CONFIG_T99_PROJ)
	#define BMC156_ACC_I2C_BUS_NUM		2
	#define BMC156_ACC_DIRECTION		0
#else
	#define BMC156_ACC_I2C_BUS_NUM    2
	#define BMC156_ACC_DIRECTION      4
#endif

/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = BMC156_ACC_I2C_BUS_NUM,
    .direction = BMC156_ACC_DIRECTION,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* bmc156_get_cust_acc_hw(void)
{
    return &cust_acc_hw;
}
