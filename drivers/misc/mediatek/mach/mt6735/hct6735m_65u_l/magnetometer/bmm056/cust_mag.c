#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

#if defined(CONFIG_T99_PROJ)
#define MAG_I2C_BUS_NUM 2
#define MAG_DIRECTION 0
#elif defined(CONFIG_T985_PROJ)
#define MAG_I2C_BUS_NUM 2
#define MAG_DIRECTION 5
#else
#define MAG_I2C_BUS_NUM 2
#define MAG_DIRECTION 3
#endif

static struct mag_hw cust_mag_hw = {
    .i2c_num = MAG_I2C_BUS_NUM,
    .direction = MAG_DIRECTION,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .is_batch_supported = false,
};
struct mag_hw* get_cust_mag_hw(void) 
{
    return &cust_mag_hw;
}
