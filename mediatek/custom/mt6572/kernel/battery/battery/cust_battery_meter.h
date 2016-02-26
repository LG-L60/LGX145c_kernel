#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
//#define SOC_BY_HW_FG
#define SOC_BY_SW_FG

/* ADC Channel Number */
#define CUST_TABT_NUMBER 17
#define VBAT_CHANNEL_NUMBER      7
#define ISENSE_CHANNEL_NUMBER	 6
#define VCHARGER_CHANNEL_NUMBER  4
#define VBATTEMP_CHANNEL_NUMBER  5

/* ADC resistor  */
#define R_BAT_SENSE 4					
#define R_I_SENSE 4						
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	0

//<2014/05/27-tedwu, Customize the battery charging tables.
/* Qmax for battery  */
#if defined(ARIMA_LO2_HW)
#define Q_MAX_POS_50	1538
#define Q_MAX_POS_25	1516
#define Q_MAX_POS_0		1312
#define Q_MAX_NEG_10	1139

#define Q_MAX_POS_50_H_CURRENT	1525
#define Q_MAX_POS_25_H_CURRENT	1495
#define Q_MAX_POS_0_H_CURRENT	1015
#define Q_MAX_NEG_10_H_CURRENT	 955
#elif defined(ARIMA_LO1_HW)
#define Q_MAX_POS_50_BL_44JH	1705
#define Q_MAX_POS_25_BL_44JH	1694
#define Q_MAX_POS_0_BL_44JH		1581
#define Q_MAX_NEG_10_BL_44JH	1506

#define Q_MAX_POS_50_H_CURRENT_BL_44JH	1678
#define Q_MAX_POS_25_H_CURRENT_BL_44JH	1646
#define Q_MAX_POS_0_H_CURRENT_BL_44JH	1428
#define Q_MAX_NEG_10_H_CURRENT_BL_44JH	 776

#define Q_MAX_POS_50_BL_44JN	1538
#define Q_MAX_POS_25_BL_44JN	1516
#define Q_MAX_POS_0_BL_44JN		1312
#define Q_MAX_NEG_10_BL_44JN	1139

#define Q_MAX_POS_50_H_CURRENT_BL_44JN	1525
#define Q_MAX_POS_25_H_CURRENT_BL_44JN	1495
#define Q_MAX_POS_0_H_CURRENT_BL_44JN	1015
#define Q_MAX_NEG_10_H_CURRENT_BL_44JN	 955
#else
#define Q_MAX_POS_50	1399
#define Q_MAX_POS_25	1499
#define Q_MAX_POS_0		1377
#define Q_MAX_NEG_10	1237

#define Q_MAX_POS_50_H_CURRENT	1377
#define Q_MAX_POS_25_H_CURRENT	1473
#define Q_MAX_POS_0_H_CURRENT	1247
#define Q_MAX_NEG_10_H_CURRENT	819
#endif

/* Discharge Percentage */
#if defined(ARIMA_LO1_HW) || defined(ARIMA_LO2_HW)
#define OAM_D5		 0		//  1 : D5,   0: D2
#else
#define OAM_D5		 1		//  1 : D5,   0: D2
#endif
//>2014/05/27-tedwu

/* battery meter parameter */
#define CUST_TRACKING_POINT  14
#define CUST_HW_CC 		     0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0
//<2014/05/15-tedwu, Customize the R-sense value.
#ifdef MTK_FAN5405_SUPPORT
#define CUST_R_SENSE         68
#elif defined(MTK_BQ24158_SUPPORT)
#define CUST_R_SENSE         56  //project LO1
#else
#define CUST_R_SENSE         200 //project LO2
#endif
//>2014/05/15-tedwu

#define OCV_BOARD_COMPESATE	0 //mV 
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
#define CAR_TUNE_VALUE		94 //1.00


/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			0 // mOhm, base is 20

//<2014/07/17-tedwu, Customize the tolerance of battery capacity and voltage.
#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	15
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		 5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			94 //90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30
//>2014/07/17-tedwu

/* Disable Battery check for HQA */
#ifdef MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

//<2014/06/06-tedwu, For battery temperature protection.
/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP              3600  //3.6V
#define VBAT_LOW_POWER_WAKEUP           3500  //3.5v
#define NORMAL_WAKEUP_PERIOD            200 //5400  //90 * 60 = 90 min
#define LOW_POWER_WAKEUP_PERIOD         200 //300   //5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD    30    //30 s
#define HIGH_TEMPERATURE_WAKEUP_PERIOD  30
#define MAX_TEMPERATURE_WAKEUP_PERIOD   20
//>2014/06/06-tedwu

#endif	//#ifndef _CUST_BATTERY_METER_H
