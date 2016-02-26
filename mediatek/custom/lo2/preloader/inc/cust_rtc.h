#ifndef __CUST_RTC_H__
#define __CUST_RTC_H__

/*
 * Default values for RTC initialization
 * Year (YEA)        : 1970 ~ 2037
 * Month (MTH)       : 1 ~ 12
 * Day of Month (DOM): 1 ~ 31
 */
#define RTC_DEFAULT_YEA		2010
#define RTC_DEFAULT_MTH		1
#define RTC_DEFAULT_DOM		1

//<2014/03/28-tedwu, Fix CQ BU2SC00144301
//#define RTC_2SEC_REBOOT_ENABLE 1 //0
#undef RTC_2SEC_REBOOT_ENABLE
//>2014/03/28-tedwu
#define RTC_2SEC_MODE 2

#endif /* __CUST_RTC_H__ */
