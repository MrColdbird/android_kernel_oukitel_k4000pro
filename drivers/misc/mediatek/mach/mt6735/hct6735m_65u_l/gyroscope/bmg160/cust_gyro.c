#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_gyro.h>

#if defined(CONFIG_T985_PROJ)||defined(CONFIG_T93_PROJ)
#define GYR_I2C_BUS_NUM 2
#define GYR_DIRECTION 4
#else
#define GYR_I2C_BUS_NUM 2
#define GYR_DIRECTION 3
#endif
/*---------------------------------------------------------------------------*/
static struct gyro_hw cust_gyro_hw = {
    .i2c_num = GYR_I2C_BUS_NUM,
    .direction = GYR_DIRECTION,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0,                   /*!< don't enable low pass fileter */
    .is_batch_supported = false,
};
/*---------------------------------------------------------------------------*/
struct gyro_hw* get_cust_gyro_hw(void) 
{
    return &cust_gyro_hw;
}
