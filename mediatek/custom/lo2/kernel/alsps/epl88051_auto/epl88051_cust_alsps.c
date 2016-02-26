#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
//#include <mach/mt6575_pm_ldo.h>
static struct alsps_hw epl88051_cust_alsps_hw = {
	.i2c_num    = 0,
	.polling_mode_ps = 1,
	.power_id   = MT6323_POWER_LDO_VGP1,//MT65XX_POWER_NONE,    /*LDO is not used*/
	.power_vol  = VOL_3300,//VOL_DEFAULT,          /*LDO is not used*/
	.i2c_addr   = {0x82, 0x48, 0x78, 0x00},
	.als_level	= {20, 45, 70, 90, 150, 300, 500, 700, 1150, 2250, 4500, 8000, 15000, 30000, 50000},
	.als_value	= {10, 30, 60, 80, 100, 200, 400, 600, 800, 1500, 3000, 6000, 10000, 20000, 40000, 60000},
	.ps_threshold_low = 270,
	.ps_threshold_high = 300,
};
struct alsps_hw *epl88051_get_cust_alsps_hw(void) {
    return &epl88051_cust_alsps_hw;
}

