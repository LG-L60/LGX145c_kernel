#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
//<2014/03/11-Yutinf Shih. Add the backlight level control and adjust(by GPIO PWM_BL mode).
#if defined( ARIMA_LO2_HW )
#include <linux/leds.h>
#endif
//>2014/03/11-Yutinf Shih.
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

//<2014/02/24-Yutinf Shih. Modified the backlight control(by GPIO PWM_BL mode).
#if defined( ARIMA_LO2_HW )
//extern int mtkfb_set_backlight_pwm(int div);
//extern int mtkfb_set_backlight_level(unsigned int level);
  extern int disp_bls_set_backlight(unsigned int level);
#else
//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);
#endif
//>2014/02/24-Yutinf Shih.

/*
#define ERROR_BL_LEVEL 0xFFFFFFFF

unsigned int brightness_mapping(unsigned int level)
{  
	return ERROR_BL_LEVEL;
}
*/
unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;
    
    mapped_level = level;
       
	return mapped_level;
}

//<2014/02/24-Yutinf Shih. Add the backlight control(by GPIO PWM_BL mode).
#if defined( ARIMA_LO2_HW )
unsigned int brightness_remap( unsigned int level )
{
unsigned int  mapped_level;
unsigned int  tmp_map;

#if defined( LED_INCREASE_LED_LEVEL_MTKPATCH )
  tmp_map = ( level * 255 ) / 1023;
#else
  tmp_map = level;
#endif
  if( tmp_map > 255 )
    mapped_level = 255;
  else
    mapped_level = tmp_map;

	return mapped_level;
}

unsigned int Cust_SetBrightness(int level)
{
    disp_bls_set_backlight(brightness_remap(level));
    return 0;
}
#endif /* End.. (ARIMA_LO2_HW) */
//>2014/02/24-Yutinf Shih.

unsigned int Cust_SetBacklight(int level, int div)
{
//<2014/02/24-Yutinf Shih. Modified the backlight control(by GPIO PWM_BL mode).
#if defined( ARIMA_LO2_HW )
  //mtkfb_set_backlight_pwm(div);
  //mtkfb_set_backlight_level(brightness_mapping(level));
    disp_bls_set_backlight(brightness_mapping(level));
#else
    //mtkfb_set_backlight_pwm(div);
    //mtkfb_set_backlight_level(brightness_mapping(level));
    disp_bls_set_backlight(brightness_mapping(level));
#endif
//>2014/02/24-Yutinf Shih.
    return 0;
}


static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1, {0}},
	{"green",             MT65XX_LED_MODE_NONE, -1, {0}},
//<2014/02/27-Yutinf Shih. Modified for pre-charge indicator.
#if defined( ARIMA_LO2_HW )
	{"blue",              MT65XX_LED_MODE_NONE, -1, {0}},
#else
	{"blue",              MT65XX_LED_MODE_NONE, -1, {0}},
#endif
//>2014/02/27-Yutinf Shih.
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1, {0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1, {0}},
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1, {0}},
//<2014/02/24-Yutinf Shih. Modified the backlight control.
#if defined( ARIMA_LO2_HW )
	//{"lcd-backlight",     MT65XX_LED_MODE_PWM, PWM1, {0,0,0,0,0}},
	//{"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, (int)Cust_SetBacklight, {0,0,0,0,0}},  /* PWM_BL */
	//{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight, {0,0,0,0,0}}
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)Cust_SetBrightness, {0,0,0,0,0}}
#else
	{"lcd-backlight",     MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_LCD_ISINK, {0}},
#endif
//>2014/02/24-Yutinf Shih.
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

