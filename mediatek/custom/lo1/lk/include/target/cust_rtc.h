#ifndef __CUST_RTC_H__
#define __CUST_RTC_H__

//<2014/03/28-tedwu, Fix CQ BU2SC00144301
//#define RTC_2SEC_REBOOT_ENABLE 1 //0
#undef RTC_2SEC_REBOOT_ENABLE
//>2014/03/28-tedwu
#define RTC_2SEC_REBOOT_KPOC_BOOT  1 //auto reboot to KPOC when charger in

#endif /* __CUST_RTC_H__ */
