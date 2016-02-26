//#include <platform/cust_leds.h>
#include <cust_leds.h>
//#include <asm/arch/mt6577_pwm.h>
//<2014/02/24-Yutinf Shih. Add for the backlight control by PWM1.
#if defined( ARIMA_LO2_HW )
#include <platform/mt_pwm.h>
#endif
//>2014/02/24-Yutinf Shih.

//<2014/02/24-Yutinf Shih. Add the backlight control(by GPIO PWM_BL mode).
#if defined( ARIMA_LO2_HW )
//extern int DISP_SetBacklight(int level);
  extern int disp_bls_set_backlight(unsigned int level);

unsigned int brightness_remap( unsigned int level )
{
unsigned int mapped_level;
unsigned int  tmp_map;

#if 0 //defined( LED_INCREASE_LED_LEVEL_MTKPATCH )
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

//unsigned int Cust_SetBacklight(int level, int div)
//{
//    DISP_SetBacklight(brightness_mapping(level));
//    return 0;
//}

unsigned int Cust_SetBrightness(int level)
{
    disp_bls_set_backlight( brightness_remap( level ));
    return 0;
}
#else
//extern int DISP_SetBacklight(int level);
#endif /* End.. (ARIMA_LO2_HW) */
//>2014/02/24-Yutinf Shih.

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

