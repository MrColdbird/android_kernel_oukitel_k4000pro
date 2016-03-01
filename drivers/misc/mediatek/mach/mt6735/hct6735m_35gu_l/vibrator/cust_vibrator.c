#include <cust_vibrator.h>
#include <linux/types.h>

static struct vibrator_hw cust_vibrator_hw = {
	.vib_timer = 25,
  #ifdef CUST_VIBR_LIMIT
	.vib_limit = 9,
  #endif

#ifdef CUST_VIBR_VOL
#if defined(CONFIG_T93BM_DG_PROJ)
	.vib_vol = 0x7,//2.8V for vibr
#else
	.vib_vol = 0x5,//2.8V for vibr
#endif
#endif
};

struct vibrator_hw *get_cust_vibrator_hw(void)
{
    return &cust_vibrator_hw;
}

