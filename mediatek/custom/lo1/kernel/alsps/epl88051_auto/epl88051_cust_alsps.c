#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw epl88051_cust_alsps_hw = {
    .i2c_num    = 1,
    .polling_mode_ps =1,///0,
    .power_id   = MT6323_POWER_LDO_VIO28,///MT6323_POWER_LDO_VGP1,///MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_2800,///VOL_3300,///VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x82, 0x48, 0x78, 0x00},
    .als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
    .als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
    .ps_threshold_low = 270,
    .ps_threshold_high = 300,
};
struct alsps_hw *epl88051_get_cust_alsps_hw(void) {
    return &epl88051_cust_alsps_hw;
}

