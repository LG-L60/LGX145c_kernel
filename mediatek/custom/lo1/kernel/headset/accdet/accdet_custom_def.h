// use accdet + EINT solution
#define ACCDET_EINT

//<2014/6/20- 39550-Richardli, [5502][Headset]Roll back the hook key multiple key event disable
//<2014/6/10- 39372-davidda, [5502][bsp]disable hook of KEY_NEXTSONG
// support multi_key feature
#define ACCDET_MULTI_KEY_FEATURE
//>2014/6/10- 39372-davidda
//<2014/6/20- 39550-Richardli
// after 5s disable accdet
#define ACCDET_LOW_POWER

//#define ACCDET_PIN_RECOGNIZATION
//#define ACCDET_28V_MODE

#define ACCDET_SHORT_PLUGOUT_DEBOUNCE
#define ACCDET_SHORT_PLUGOUT_DEBOUNCE_CN 20

//extern struct headset_mode_settings* get_cust_headset_settings(void);
//extern int get_long_press_time_cust(void);
