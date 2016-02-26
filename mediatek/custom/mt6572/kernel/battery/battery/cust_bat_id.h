//<2014/04/01-tedwu, Add for battery ID checking feature.
#ifndef _BAT_ID_H_
#define _BAT_ID_H_

#define GPIO_BATTERY_DATA_PIN         GPIO20
#define GPIO_BATTERY_DATA_PIN_M_GPIO  GPIO_MODE_00

#if defined(ARIMA_LO1_HW)
#define CHK_BATTERY_ID
#define BATTERY_PACK_ERROR    0
#define BATTERY_PACK_BL_44JH  1 //1700mAh
#define BATTERY_PACK_BL_44JN  2 //1540mAh
//#define BATTERY_PACK_HW4X
#endif

extern kal_uint8 battery_pack_id;

extern void bat_id_init_pin(void);
extern void bat_id_deinit_pin(void);
//void bat_id_init_variable(void);
//void bat_id_init_spinLock(void);
//void Read_Battery_Info(void);
//kal_bool Is_MOTO_Battery(void);
//kal_bool Is_Invalid_Battery(void);
//int Get_MOTO_Battery_Vender(void);
#endif //ifndef _BAT_ID_H_
//>2014/04/01-tedwu
