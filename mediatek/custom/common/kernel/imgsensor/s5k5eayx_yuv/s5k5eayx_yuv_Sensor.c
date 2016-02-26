/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *   
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *      
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/atomic.h>
#include <asm/uaccess.h> 
#include <linux/miscdevice.h>
//#include <mach/mt6516_pll.h>
#include <linux/kernel.h>//for printk

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"


#include "s5k5eayx_yuv_Sensor.h"
#include "s5k5eayx_yuv_Camera_Sensor_para.h"
#include "s5k5eayx_yuv_CameraCustomized.h"


#define S5K5EAYXYUV_DEBUG
#ifdef S5K5EAYXYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define S5K5EA_PRE_AFRangeWidth (1280)
#define S5K5EA_PRE_AFRangeHeight (960)
#define S5K5EA_CAP_AFRangeWidth (2560)
#define S5K5EA_CAP_AFRangeHeight (1920)

#define S5K5EA_PRV_RATIO_WIDTH (240)
#define S5K5EA_PRV_RATIO_HEIGHT (240)
#define S5K5EA_CAP_RATIO_WIDTH (480)
#define S5K5EA_CAP_RATIO_HEIGHT (480)
#define S5K5EA_FAF_TOLERANCE    (40)

#define S5K5EA_READ_SHUTTER_RATIO (4)
#define S5K5EA_SET_SHUTTER_RATIO (1)
#define S5K5EA_SET_GAIN_RATIO (1)

#define S5K5EZYX_NightMode_Off (0)
#define S5K5EZYX_NightMode_On (1)

kal_bool S5K5EAYX_video_mode = KAL_FALSE;
kal_uint32 zoom_factor = 0; 

typedef enum
{
    S5K5EAYX_SENSORMODE_PREVIEW=0,
	S5K5EAYX_SENSORMODE_VIDEO,
	S5K5EAYX_SENSORMODE_CAPTURE,
	S5K5EAYX_SENSORMODE_ZSD,
} SensorMode;
SensorMode s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_PREVIEW;

typedef enum
{
    S5K5EAYX_CAP_AEAWB_CLOSE=0,
	S5K5EAYX_CAP_AEAWB_OPEN,
} S5K5EAYX_CAP_AEAWB_STATUS;
S5K5EAYX_CAP_AEAWB_STATUS s5k5eayx_cap_aeawb_status = S5K5EAYX_CAP_AEAWB_CLOSE;


static MSDK_SENSOR_CONFIG_STRUCT S5K5EAYXSensorConfigData;
kal_uint8 S5K5EAYXYUV_sensor_write_I2C_address = S5K5EAYX_WRITE_ID;
kal_uint8 S5K5EAYXYUV_sensor_read_I2C_address = S5K5EAYX_READ_ID;
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
kal_bool S5K5EAYX_preflash = KAL_FALSE;
kal_uint8 S5K5EAYX_Count = 0;
//<2014/5/2-37049-joubert.she.
kal_bool S5K5EAYX_A_COLOR_TEMP = KAL_FALSE;
struct S5K5EAYX_sensor_struct S5K5EAYX_Sensor_Driver;
MSDK_SCENARIO_ID_ENUM S5K5EAYXCurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;


typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iSceneMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
  kal_uint32  iShutter;
  kal_uint32  iGain;
  kal_bool HDRFixed_AE_Done;
} S5K5EAYX_Status;
S5K5EAYX_Status S5K5EAYX_CurrentStatus;


extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iMultiWriteReg(u8 *pData, u16 lens, u16 i2cId);

static DEFINE_SPINLOCK(s5k5eayx_drv_lock);
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
static void S5K5EAYX_Mode_Config(kal_bool NightModeEnable);
//>2014/5/2-37049-joubert.she



static kal_uint16 S5K5EAYX_write_cmos_sensor_wID(kal_uint32 addr, kal_uint32 para, kal_uint32 id)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 4,id);

}
static kal_uint16 S5K5EAYX_read_cmos_sensor_wID(kal_uint32 addr, kal_uint32 id)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,id);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}

static kal_uint16 S5K5EAYX_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,S5K5EAYXYUV_sensor_write_I2C_address);
    return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}
static kal_uint16 S5K5EAYX_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para >> 8),(char)(para & 0xFF)};
	
	iWriteRegI2C(puSendCmd , 4,S5K5EAYXYUV_sensor_write_I2C_address);
}
/////////////////////////////////////////
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
void ReadNB(void)
{
    unsigned int flag,flag1;
    
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x0554); 
    flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x0556); 
    flag1= S5K5EAYX_read_cmos_sensor(0x0F12); 
	//<2014/5/5-37322-joubert.she, [Lo1/Lo2][DRV] update flashlight brightness driver from norman.
    SENSORDB("[5EA] NB  [0x0554] = 0x%04x,[0x0556] = 0x%04x\n", flag,flag1);

    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x213C); 
    flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x213E); 
    flag1= S5K5EAYX_read_cmos_sensor(0x0F12); 
    SENSORDB("[5EA] LEI [0x213C] = 0x%04x,[0x213E] = 0x%04x \n", flag,flag1);

    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x2562); 
    flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
    SENSORDB("[5EA] Ratio_in_ram[0x2562] = 0x%04x \n", flag);

    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x2518); 
    flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x251A); 
    flag1= S5K5EAYX_read_cmos_sensor(0x0F12); 
    SENSORDB("[5EA] LEI _by 0CF4 [0x2518] = 0x%04x,[0x251A] = 0x%04x \n", flag,flag1);

    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x2524); 
    flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x2526); 
    flag1= S5K5EAYX_read_cmos_sensor(0x0F12); 
    SENSORDB("[5EA] LEI _by 0CF6 [0x2524] = 0x%04x,[0x2526] = 0x%04x \n", flag,flag1);
	//>2014/5/5-37322-joubert.she.

}
//>2014/5/2-37049-joubert.she
static kal_uint32 S5K5EAYX_ReadShutter(void)
{
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
//    unsigned int FlashLED_temp1,FlashLED_temp2;
//>2014/5/6-37370-joubert.she.
    unsigned int interval = 0;
	//<2014/6/12-39517-joubert.she, [Lo1/Lo2][DRV] update main camera driver .
    unsigned int flag,flag1;
	//>2014/6/12-39517-joubert.she 
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
    if(S5K5EAYX_preflash == KAL_FALSE) 
    {
        SENSORDB("[5EA] s5k5eayx_Pre_Flash_Start\n" );
	
        spin_lock(&s5k5eayx_drv_lock);
        S5K5EAYX_preflash = KAL_TRUE;
        S5K5EAYX_Count=0;
        spin_unlock(&s5k5eayx_drv_lock);

        S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002E,0x2188); 
        flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
        SENSORDB("[5EA] NB [0x2188] = 0x%04x \n", flag);
        if(flag >= 0x0A ) 
        {
            S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
            S5K5EAYX_write_cmos_sensor(0x002E,0x2AC4); 
            flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
            SENSORDB("[5EA]  [0x2AC4] = 0x%04x \n", flag);
            S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
            S5K5EAYX_write_cmos_sensor(0x002E,0x2AC6); 
            flag1= S5K5EAYX_read_cmos_sensor(0x0F12); 
            SENSORDB("[5EA]  [0x2AC6] = 0x%04x \n", flag1);

            if((flag >878)&&(flag1<373))
            {
                spin_lock(&s5k5eayx_drv_lock);
                S5K5EAYX_A_COLOR_TEMP=KAL_TRUE;
                spin_unlock(&s5k5eayx_drv_lock);
                SENSORDB("[5EA]  S5K5EAYX_A_COLOR_TEMP= True \n" );
                //S5k5EAYX_Set_A_Light_Grid();
                S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
                S5K5EAYX_write_cmos_sensor(0x002a,0x0c42); 
                S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); 
                
                S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
                S5K5EAYX_write_cmos_sensor(0x002a,0x1208); 
                S5K5EAYX_write_cmos_sensor(0x0F12,0x0300); 
                S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 
                S5K5EAYX_write_cmos_sensor(0x0F12,0x0180); 
            }
        }
        
        S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x0854); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 

//        S5K5EAYX_write_cmos_sensor(0x002a,0x0994); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x0992); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0100); 


        S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); //lt_SpMode
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); //lt_SpModeChanged
        S5K5EAYX_write_cmos_sensor(0x0F12,0x000F); //lt_SpCounter
 //<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.       
        S5K5EAYX_write_cmos_sensor(0x002a,0x0cf4); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); 

        S5K5EAYX_write_cmos_sensor(0x002a,0x04c0); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); 

//>2014/5/6-37370-joubert.she.
    }
	//<2014/5/5-37322-joubert.she, [Lo1/Lo2][DRV] update flashlight brightness driver from norman.
    //ReadNB();
	//>2014/5/5-37322-joubert.she
    if(S5K5EAYX_preflash==KAL_TRUE)
    {
        spin_lock(&s5k5eayx_drv_lock);
        S5K5EAYX_Count++;
        spin_unlock(&s5k5eayx_drv_lock);
        if(S5K5EAYX_Count >15)  //check FLASHLIGHT_YUV_CONVERGENCE_FRAME (mediatek\custom\mt6572\hal\camera\Camera\Camera_custom_if.cpp  )
        {
            SENSORDB("[5EA] s5k5eayx_Pre_Flash_End\n" );
            S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
            S5K5EAYX_write_cmos_sensor(0x002a,0x0854); 
            S5K5EAYX_write_cmos_sensor(0x0F12,0x0002); //Fast AE off
			//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
            S5K5EAYX_write_cmos_sensor(0x002a,0x0CF6); 
            S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); 
			//>2014/5/6-37370-joubert.she.
            S5K5EAYX_write_cmos_sensor(0x002a,0x04C0); 
            S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 
			//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
			//>2014/5/6-37370-joubert.she.
			//<2014/6/12-39517-joubert.she, [Lo1/Lo2][DRV] update main camera driver 
            S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
            S5K5EAYX_write_cmos_sensor(0x002E,0x2188); 
            flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
            SENSORDB("[5EA] NB [0x2188] = 0x%04x \n", flag);
            if(flag < 0x0A ) 
            {
                S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
                S5K5EAYX_write_cmos_sensor(0x002a,0x0c42); 
                S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 

                S5K5EAYX_write_cmos_sensor(0x0028, 0x2000); 
                S5K5EAYX_write_cmos_sensor(0x002A, 0x0C52);
                S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); //0020: for preflash case , 0100: for main case
                S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); //0020: for preflash case , 0100: for main case
                S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); //0020: for preflash case , 0100: for main case
                S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); //0020: for preflash case , 0100: for main case
                S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); //0020: for preflash case , 0100: for main case
               
            }
			//>2014/6/12-39517-joubert.she 
        }
    }
//<2014/5/2-37049-joubert.she,
    S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,0x2340);
    interval  = S5K5EAYX_read_cmos_sensor(0x0F12);
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,0x2342);
    interval |= (S5K5EAYX_read_cmos_sensor(0x0F12) << 16);
    // reg is in terms of 1000/400 us
    interval = interval * 5 / 2; //us
 
    SENSORDB("[5EA] Shutter = %d us\n", interval);
/*
   S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x0554); 
   flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
    SENSORDB("[5EA] NB LSB[0x0554] = 0x%x \n", flag);
   S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x0556); 
   flag= S5K5EAYX_read_cmos_sensor(0x0F12); 
    SENSORDB("[5EA] NB MSB [0x0556] = 0x%x \n", flag);
*/   
    return interval;
}

void S5K5EAYX_SetShutter(kal_uint32 iShutter)
{
    // iShutter is in terms of 32us
    iShutter *= 32;
    unsigned int exposureTime = iShutter >> 3; // to hardware reg

    SENSORDB("[5EA] SetdShutter+, iShutter=%d us, 0x%08x\n", iShutter, exposureTime);
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
    if(S5K5EAYX_preflash == KAL_TRUE)
        return;

    //Change to Manual AE
    S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002A,0x051C);
    S5K5EAYX_write_cmos_sensor(0x0F12,0x0779); //Manual AE enable
    //S5K5EAYX_Sensor_Driver.manualAEStart = 1;

    S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002A,0x04E8);
    S5K5EAYX_write_cmos_sensor(0x0F12,exposureTime&0xFFFF); //Exposure time
    S5K5EAYX_write_cmos_sensor(0x0F12,exposureTime>>16); //Exposure time
    S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); //Exposure time update

    //S5K5EAYX_write_cmos_sensor(0x002A,0x04EE);
    //S5K5EAYX_write_cmos_sensor(0x0F12,totalGain);   //Total gain
    //S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);
//>2014/5/2-37049-joubert.she.
}


/*************************************************************************
* FUNCTION
*    S5K5EAYX_MIPI_ReadGain
* DESCRIPTION
*    This function get gain from sensor
* PARAMETERS
*    None
* RETURNS
*    Gain: base on 0x40
* LOCAL AFFECTED
*************************************************************************/
static kal_uint32 S5K5EAYX_ReadGain(void)
{

    //S5K5EAYX_MIPI_GetAutoISOValue();

    unsigned int A_Gain, D_Gain, ISOValue;
    S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
    S5K5EAYX_write_cmos_sensor(0x002C, 0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E, 0x2144);
    A_Gain = S5K5EAYX_read_cmos_sensor(0x0F12);
    D_Gain = S5K5EAYX_read_cmos_sensor(0x0F12);

    ISOValue = ((A_Gain * D_Gain) >> 8);

    SENSORDB("[5EA] ReadGain+, isoSpeed=%d\n", ISOValue);
    return ISOValue; 
}

static void S5K5EAYX_SetGain(kal_uint32 iGain)
{
    // Cal. Method : ((A-Gain*D-Gain)/100h)/2
    // A-Gain , D-Gain Read value is hex value.
    //   ISO 50  : 100(HEX)
    //   ISO 100 : 100 ~ 1FF(HEX)
    //   ISO 200 : 200 ~ 37F(HEX)
    //   ISO 400 : over 380(HEX)


    unsigned int totalGain = iGain;
    SENSORDB("[5EA] SetGain+, isoGain=%d\n", totalGain);
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
    if((S5K5EAYX_preflash == KAL_TRUE) &&(S5K5EAYX_Count >15))
    {
        SENSORDB("[5EA] s5k5eayx_Flash_Start\n");
        S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x2500); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0002); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x0C44); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x04C0); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0002); 
        S5K5EAYX_Mode_Config(S5K5EAYX_CurrentStatus.iNightMode);
        return;
    }
	

    //Change to Manual AE
    S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002A,0x051C);
    S5K5EAYX_write_cmos_sensor(0x0F12,0x0779); //Manual AE enable

    //S5K5EAYX_write_cmos_sensor(0x0028,0x7000);
    S5K5EAYX_write_cmos_sensor(0x002A,0x04EE);
    S5K5EAYX_write_cmos_sensor(0x0F12,totalGain);   //Total gain
    S5K5EAYX_write_cmos_sensor(0x002A,0x04F0);
    S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);
    
//<2014/5/2-37049-joubert.she.
}


void S5K5EAYX_get_exposure_gain()
{
    kal_uint32 again = 0, dgain = 0, evTime = 0,iso;

    S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
    S5K5EAYX_write_cmos_sensor(0x002C, 0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E, 0x2340);
    evTime  = S5K5EAYX_read_cmos_sensor(0x0F12);
    evTime += S5K5EAYX_read_cmos_sensor(0x0F12) << 16 ;
    spin_lock(&s5k5eayx_drv_lock);
	//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
    S5K5EAYX_CurrentStatus.iShutter= evTime * 5 / 2 ;
	//<2014/5/2-37049-joubert.she
    //S5K5EAYX_Sensor_Driver.currentExposureTime = evTime >> 2;
    spin_unlock(&s5k5eayx_drv_lock);

    S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,0x2144);
    again = S5K5EAYX_read_cmos_sensor(0x0F12); //A gain
    dgain = S5K5EAYX_read_cmos_sensor(0x0F12); //D gain
    
    iso= (dgain * again) >> 9;
    // Cal. Method : ((A-Gain*D-Gain)/100h)/2
    // A-Gain , D-Gain Read value is hex value.
    //   ISO 50  : 100(HEX)
    //   ISO 100 : 100 ~ 1FF(HEX)
    //   ISO 200 : 200 ~ 37F(HEX)
    //   ISO 400 : over 380(HEX)
    spin_lock(&s5k5eayx_drv_lock);
    if ((iso >= 200) && (iso < 896 ))
    {
       S5K5EAYX_CurrentStatus.iGain = AE_ISO_200;
    }
    else if (iso >= 896)
    {
       S5K5EAYX_CurrentStatus.iGain = AE_ISO_400;
    }
    else
    {
       S5K5EAYX_CurrentStatus.iGain = AE_ISO_100;
    }
    //S5K5EAYX_Sensor_Driver.currentAxDGain = (dgain * again) >> 8;
    spin_unlock(&s5k5eayx_drv_lock);

}

void S5K5EAYX_GetAEFlashlightInfo(UINT32 infoAddr)
{
    SENSOR_FLASHLIGHT_AE_INFO_STRUCT* pAeInfo = (SENSOR_FLASHLIGHT_AE_INFO_STRUCT*) infoAddr;
    pAeInfo->Exposuretime = S5K5EAYX_ReadShutter();
    pAeInfo->Gain = S5K5EAYX_ReadGain();
    pAeInfo->u4Fno = 26;
    pAeInfo->GAIN_BASE = 0x100;
    return;
}

#define FLASH_BV_THRESHOLD 0x25
static void S5K5EAYX_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
    unsigned int NormBr;

    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,0x2188);
    NormBr = S5K5EAYX_read_cmos_sensor(0x0F12);

    SENSORDB(" S5K5EAYX_FlashTriggerCheck NormBr=%x\n",NormBr);
	
    if (NormBr > FLASH_BV_THRESHOLD)
    {
       *pFeatureReturnPara32 = FALSE;
        return;
    }

    *pFeatureReturnPara32 = TRUE;
    return;
}
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
static void S5K5EAYX_ReadOTP(void)
{
    unsigned int flag;
    
    S5K5EAYX_write_cmos_sensor(0x0012, 0x0001);  //s/w core reset
    S5K5EAYX_write_cmos_sensor(0x006E, 0x0003);  //EFUSE/SYS external clock enable
    S5K5EAYX_write_cmos_sensor(0x0072, 0x0020);  //External Clock mode
    S5K5EAYX_write_cmos_sensor(0x1002, 0x1241);  //core status1
    S5K5EAYX_write_cmos_sensor(0x007A, 0x0000);  //all clock gating disable
    
    S5K5EAYX_write_cmos_sensor(0xA002, 0x0001);  //set the PAGE0 of OTP(0 ? the number of PAGE ? 15)
    S5K5EAYX_write_cmos_sensor(0xA000, 0x0004);  //set  read mode of NVM controller Interface1
    S5K5EAYX_write_cmos_sensor(0xA000, 0x0001);  //to wait  Tmin = 47us(the time to transfer 1page data from OTP to buffer)
    
    flag= S5K5EAYX_read_cmos_sensor(0xA006); 
    SENSORDB("[5EA] ReadOTP Module ID =0x%02x\n", flag&0x3);
    SENSORDB("[5EA] ReadOTP Year =0x%02x\n", (flag>>2)&0x3);
    flag= S5K5EAYX_read_cmos_sensor(0xA008); 
    SENSORDB("[5EA] ReadOTP Month =0x%02x\n", flag&0x3);
    SENSORDB("[5EA] ReadOTP Day =0x%02x\n", (flag>>2)&0x3);
    
    flag= S5K5EAYX_read_cmos_sensor(0xA00A); 
    SENSORDB("[5EA] ReadOTP Light source =0x%02x\n",(flag>>2)&0x3);
    
    S5K5EAYX_write_cmos_sensor(0xA000, 0x0000);  //interface off & make initial     
}
//>2014/5/6-37370-joubert.she.
static void S5K5EAYX_InitialPara(void)
{
  spin_lock(&s5k5eayx_drv_lock);

  S5K5EAYX_CurrentStatus.iNightMode = 0;
  S5K5EAYX_CurrentStatus.iSceneMode = SCENE_MODE_OFF;
  S5K5EAYX_CurrentStatus.iWB = AWB_MODE_AUTO;
  S5K5EAYX_CurrentStatus.iEffect = MEFFECT_OFF;
  S5K5EAYX_CurrentStatus.iBanding = AE_FLICKER_MODE_50HZ;
  S5K5EAYX_CurrentStatus.iEV = AE_EV_COMP_00;
  S5K5EAYX_CurrentStatus.iMirror = IMAGE_NORMAL;
  S5K5EAYX_CurrentStatus.iFrameRate = 0;
  S5K5EAYX_CurrentStatus.iShutter= 0;
  S5K5EAYX_CurrentStatus.iGain= 0;
  S5K5EAYX_CurrentStatus.HDRFixed_AE_Done=KAL_FALSE;
  
  S5K5EAYX_video_mode = KAL_FALSE;
  s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_PREVIEW;
  spin_unlock(&s5k5eayx_drv_lock);
  
}


static void S5K5EAYX_Init_Setting(void)
{
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_ReadOTP();
//>2014/5/6-37370-joubert.she.
    SENSORDB("[5EA] :S5K5EAYX_Init_Setting \n");

// s5k5eayx_init_reg1 ----------------------
S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);  
S5K5EAYX_write_cmos_sensor(0x0010, 0x0001);  //S/W Reset
S5K5EAYX_write_cmos_sensor(0x1030, 0x0000);  //contint_host_int
S5K5EAYX_write_cmos_sensor(0x0014, 0x0001);  //sw_load_complete - Release CORE (Arm) from reset state

mdelay(10);
//<2014/6/12-39517-joubert.she, [Lo1/Lo2][DRV] update main camera driver 
// s5k5eayx_init_reg2 ----------------------
S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
S5K5EAYX_write_cmos_sensor(0x002A, 0x31E4);
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB570);    // 200031E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4E26);    // 200031E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4D26);    // 200031E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4C27);    // 200031EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200031EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200031EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBE7);    // 200031F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6266);    // 200031F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x62A5);    // 200031F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4D26);    // 200031F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4824);    // 200031F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x63E8);    // 200031FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4C26);    // 200031FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4825);    // 200031FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6120);    // 20003200 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4926);    // 20003202 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4826);    // 20003204 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003206 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBE1);    // 20003208 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4926);    // 2000320A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4826);    // 2000320C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 2000320E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBDD);    // 20003210 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4926);    // 20003212 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4826);    // 20003214 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003216 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBD9);    // 20003218 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4E26);    // 2000321A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000321C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8030);    // 2000321E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8070);    // 20003220 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4925);    // 20003222 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4825);    // 20003224 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003226 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBD1);    // 20003228 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4631);    // 2000322A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x310C);    // 2000322C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6008);    // 2000322E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4620);    // 20003230 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4923);    // 20003232 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x38C0);    // 20003234 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6141);    // 20003236 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4822);    // 20003238 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x67E8);    // 2000323A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4922);    // 2000323C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4823);    // 2000323E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003240 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBC4);    // 20003242 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4922);    // 20003244 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4823);    // 20003246 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003248 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBC0);    // 2000324A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x60B0);    // 2000324C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x21FF);    // 2000324E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4821);    // 20003250 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x31C1);    // 20003252 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8081);    // 20003254 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2120);    // 20003256 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8041);    // 20003258 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2130);    // 2000325A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8101);    // 2000325C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x21FF);    // 2000325E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x31A1);    // 20003260 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80C1);    // 20003262 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x491D);    // 20003264 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x481E);    // 20003266 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003268 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBB0);    // 2000326A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x481D);    // 2000326C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6768);    // 2000326E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x491D);    // 20003270 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x481E);    // 20003272 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003274 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBAA);    // 20003276 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x481D);    // 20003278 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6260);    // 2000327A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 2000327C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 2000327E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0155);    // 20003280 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5EA1);    // 20003282 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x37FF);    // 20003284 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003286 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F34);    // 20003288 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000328A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x32FD);    // 2000328C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000328E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0008);    // 20003290 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003292 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3363);    // 20003294 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003296 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0148);    // 20003298 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000329A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x340D);    // 2000329C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000329E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5361);    // 200032A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x342D);    // 200032A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xCCC1);    // 200032A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x352F);    // 200032AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x67D7);    // 200032B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3AE0);    // 200032B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x358B);    // 200032B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x745B);    // 200032BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x363D);    // 200032C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3671);    // 200032C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3699);    // 200032C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x50F9);    // 200032CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x37C3);    // 200032D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xEB93);    // 200032D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4780);    // 200032D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3845);    // 200032DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xC77B);    // 200032E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x38CB);    // 200032E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x38DF);    // 200032E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9671);    // 200032EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200032EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x394B);    // 200032F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200032F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBA40);    // 200032F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4770);    // 200032F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBAC0);    // 200032F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4770);    // 200032FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB570);    // 200032FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48FC);    // 200032FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8C00);    // 20003300 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x07C0);    // 20003302 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD028);    // 20003304 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x49FB);    // 20003306 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8848);    // 20003308 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8809);    // 2000330A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);    // 2000330C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4308);    // 2000330E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4CF9);    // 20003310 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8A25);    // 20003312 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4AF9);    // 20003314 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 20003316 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8151);    // 20003318 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4621);    // 2000331A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3920);    // 2000331C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6989);    // 2000331E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x233D);    // 20003320 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5C5E);    // 20003322 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4BF6);    // 20003324 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2E00);    // 20003326 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD005);    // 20003328 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3140);    // 2000332A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8B89);    // 2000332C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2902);    // 2000332E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD015);    // 20003330 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2100);    // 20003332 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8219);    // 20003334 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4EF2);    // 20003336 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2100);    // 20003338 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3620);    // 2000333A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8031);    // 2000333C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03C6);    // 2000333E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C36);    // 20003340 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x825E);    // 20003342 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8151);    // 20003344 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2106);    // 20003346 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003348 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB46);    // 2000334A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8220);    // 2000334C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x207D);    // 2000334E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0180);    // 20003350 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003352 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB47);    // 20003354 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8225);    // 20003356 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003358 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB4A);    // 2000335A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 2000335C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 2000335E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE7E8);    // 20003360 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB5F8);    // 20003362 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x49E7);    // 20003364 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x880A);    // 20003366 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2A00);    // 20003368 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD114);    // 2000336A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4DE3);    // 2000336C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x89AA);    // 2000336E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2A00);    // 20003370 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD010);    // 20003372 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2700);    // 20003374 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 20003376 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD00E);    // 20003378 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8848);    // 2000337A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 2000337C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB32);    // 2000337E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48DF);    // 20003380 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3020);    // 20003382 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8007);    // 20003384 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x200A);    // 20003386 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003388 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB2C);    // 2000338A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 2000338C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8168);    // 2000338E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x200A);    // 20003390 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003392 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB27);    // 20003394 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 20003396 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48D6);    // 20003398 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8841);    // 2000339A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8800);    // 2000339C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0409);    // 2000339E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4301);    // 200033A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0348);    // 200033A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C00);    // 200033A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4CD4);    // 200033A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8A26);    // 200033A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x49D5);    // 200033AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8248);    // 200033AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x816F);    // 200033AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4AD3);    // 200033B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 200033B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3220);    // 200033B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8011);    // 200033B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8220);    // 200033B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x207D);    // 200033BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);    // 200033BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200033BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB11);    // 200033C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8226);    // 200033C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 200033C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB570);    // 200033C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4606);    // 200033C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4CCF);    // 200033CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8820);    // 200033CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 200033CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD112);    // 200033D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48CE);    // 200033D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2108);    // 200033D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8041);    // 200033D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2501);    // 200033D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8005);    // 200033DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200033DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB0E);    // 200033DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48C5);    // 200033E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x49CB);    // 200033E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8201);    // 200033E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48CB);    // 200033E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8BC0);    // 200033E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200033EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFAFB);    // 200033EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48C7);    // 200033EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3040);    // 200033F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8880);    // 200033F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8060);    // 200033F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8025);    // 200033F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2E00);    // 200033F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD003);    // 200033FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8860);    // 200033FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2102);    // 200033FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4008);    // 20003400 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 20003402 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8860);    // 20003404 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x07C0);    // 20003406 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0FC0);    // 20003408 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 2000340A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB570);    // 2000340C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4CBF);    // 2000340E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3460);    // 20003410 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8A25);    // 20003412 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2004);    // 20003414 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4385);    // 20003416 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003418 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 2000341A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFD4);    // 2000341C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);    // 2000341E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4328);    // 20003420 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8220);    // 20003422 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2021);    // 20003424 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);    // 20003426 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8060);    // 20003428 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 2000342A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB5F8);    // 2000342C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 2000342E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 20003430 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC9);    // 20003432 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4CB7);    // 20003434 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 20003436 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3420);    // 20003438 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2802);    // 2000343A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD01C);    // 2000343C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000343E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A0);    // 20003440 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48B2);    // 20003442 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x89A2);    // 20003444 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8042);    // 20003446 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 20003448 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8001);    // 2000344A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4FB1);    // 2000344C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8BF8);    // 2000344E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003450 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFAC8);    // 20003452 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4EAD);    // 20003454 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3620);    // 20003456 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8A71);    // 20003458 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48AF);    // 2000345A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4DAF);    // 2000345C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2900);    // 2000345E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD002);    // 20003460 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8902);    // 20003462 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2A00);    // 20003464 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD009);    // 20003466 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 20003468 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03C9);    // 2000346A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8069);    // 2000346C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80E9);    // 2000346E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2100);    // 20003470 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A9);    // 20003472 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8129);    // 20003474 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE01E);    // 20003476 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A1);    // 20003478 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE7E2);    // 2000347A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8069);    // 2000347C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8AB1);    // 2000347E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80E9);    // 20003480 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8AF1);    // 20003482 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB209);    // 20003484 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A9);    // 20003486 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8B32);    // 20003488 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB212);    // 2000348A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x812A);    // 2000348C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8B73);    // 2000348E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x469C);    // 20003490 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8BB3);    // 20003492 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x469E);    // 20003494 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4663);    // 20003496 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2B00);    // 20003498 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD104);    // 2000349A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x17CB);    // 2000349C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F5B);    // 2000349E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1859);    // 200034A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x10C9);    // 200034A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A9);    // 200034A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4671);    // 200034A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2900);    // 200034A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD104);    // 200034AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x17D1);    // 200034AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F49);    // 200034AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1889);    // 200034B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x10C9);    // 200034B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8129);    // 200034B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8940);    // 200034B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 200034B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD006);    // 200034BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 200034BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0280);    // 200034BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A8);    // 200034C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8168);    // 200034C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8228);    // 200034C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81E8);    // 200034C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 200034C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 200034CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A0);    // 200034CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x498F);    // 200034CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8048);    // 200034D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8008);    // 200034D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8BF8);    // 200034D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200034D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA85);    // 200034D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8BF0);    // 200034DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 200034DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD009);    // 200034DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A8);    // 200034E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x488A);    // 200034E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3040);    // 200034E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8801);    // 200034E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8169);    // 200034E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8841);    // 200034EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8229);    // 200034EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8880);    // 200034EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81E8);    // 200034F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 200034F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200034F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A0);    // 200034F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4984);    // 200034F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8048);    // 200034FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 200034FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8008);    // 200034FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8BF8);    // 20003500 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003502 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA6F);    // 20003504 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8BF0);    // 20003506 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 20003508 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD009);    // 2000350A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A8);    // 2000350C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x487F);    // 2000350E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3040);    // 20003510 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8801);    // 20003512 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8169);    // 20003514 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8841);    // 20003516 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8229);    // 20003518 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8880);    // 2000351A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81E8);    // 2000351C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 2000351E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 20003520 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0280);    // 20003522 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A8);    // 20003524 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8168);    // 20003526 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8228);    // 20003528 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81E8);    // 2000352A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 2000352C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB5F3);    // 2000352E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB091);    // 20003530 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4607);    // 20003532 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2220);    // 20003534 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x497A);    // 20003536 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xA809);    // 20003538 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 2000353A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA65);    // 2000353C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2220);    // 2000353E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4977);    // 20003540 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3120);    // 20003542 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xA801);    // 20003544 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003546 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA5F);    // 20003548 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2400);    // 2000354A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4E75);    // 2000354C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A0);    // 2000354E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xAA01);    // 20003550 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5E11);    // 20003552 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1883);    // 20003554 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2202);    // 20003556 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5E9A);    // 20003558 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9B12);    // 2000355A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4379);    // 2000355C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x435A);    // 2000355E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1889);    // 20003560 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x17CA);    // 20003562 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0E92);    // 20003564 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1851);    // 20003566 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1189);    // 20003568 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xAA09);    // 2000356A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A13);    // 2000356C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x199D);    // 2000356E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1880);    // 20003570 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8842);    // 20003572 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7828);    // 20003574 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1840);    // 20003576 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2100);    // 20003578 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 2000357A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA4B);    // 2000357C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7028);    // 2000357E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C64);    // 20003580 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2C08);    // 20003582 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD3E3);    // 20003584 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB013);    // 20003586 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF0);    // 20003588 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB5F3);    // 2000358A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB081);    // 2000358C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4C5E);    // 2000358E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4A65);    // 20003590 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x340C);    // 20003592 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6820);    // 20003594 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2701);    // 20003596 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0901);    // 20003598 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0049);    // 2000359A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0705);    // 2000359C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A53);    // 2000359E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F2D);    // 200035A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4638);    // 200035A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x40A8);    // 200035A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4383);    // 200035A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5253);    // 200035A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4851);    // 200035AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x78C0);    // 200035AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 200035AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD032);    // 200035B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x485E);    // 200035B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4D5E);    // 200035B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8D81);    // 200035B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2900);    // 200035B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD025);    // 200035BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x88A0);    // 200035BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 200035BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD122);    // 200035C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4A5A);    // 200035C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6DA8);    // 200035C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8DD2);    // 200035C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C03);    // 200035C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD006);    // 200035CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A03);    // 200035CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4359);    // 200035CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x60A1);    // 200035D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x60E0);    // 200035D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4353);    // 200035D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6123);    // 200035D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE006);    // 200035D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4341);    // 200035DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A09);    // 200035DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x60A1);    // 200035DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x60E0);    // 200035E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4350);    // 200035E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);    // 200035E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6120);    // 200035E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6920);    // 200035E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x65A8);    // 200035EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4946);    // 200035EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9801);    // 200035EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3130);    // 200035F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200035F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA15);    // 200035F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4629);    // 200035F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3158);    // 200035F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xC94E);    // 200035FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4842);    // 200035FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3020);    // 200035FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xC04E);    // 20003600 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x68E0);    // 20003602 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x65A8);    // 20003604 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A7);    // 20003606 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x88A0);    // 20003608 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2802);    // 2000360A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD106);    // 2000360C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x68A0);    // 2000360E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x65A8);    // 20003610 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2003);    // 20003612 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A0);    // 20003614 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE001);    // 20003616 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003618 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A0);    // 2000361A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9902);    // 2000361C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9801);    // 2000361E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003620 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF9FE);    // 20003622 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6820);    // 20003624 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4B40);    // 20003626 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0901);    // 20003628 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0049);    // 2000362A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0704);    // 2000362C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A5A);    // 2000362E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F24);    // 20003630 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4638);    // 20003632 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x40A0);    // 20003634 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4302);    // 20003636 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x525A);    // 20003638 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDFE);    // 2000363A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB570);    // 2000363C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 2000363E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003640 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF9F4);    // 20003642 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4C30);    // 20003644 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x340C);    // 20003646 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x88A0);    // 20003648 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2801);    // 2000364A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD10F);    // 2000364C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4620);    // 2000364E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3024);    // 20003650 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003652 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF9F1);    // 20003654 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x492C);    // 20003656 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4835);    // 20003658 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6A8D);    // 2000365A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6A4B);    // 2000365C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6A0A);    // 2000365E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6AC9);    // 20003660 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6605);    // 20003662 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x65C3);    // 20003664 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6582);    // 20003666 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6641);    // 20003668 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2002);    // 2000366A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A0);    // 2000366C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 2000366E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB570);    // 20003670 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4604);    // 20003672 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4D2F);    // 20003674 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7828);    // 20003676 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4320);    // 20003678 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD00C);    // 2000367A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x20C1);    // 2000367C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4622);    // 2000367E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 20003680 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);    // 20003682 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003684 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF9DE);    // 20003686 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2C00);    // 20003688 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD103);    // 2000368A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x20FF);    // 2000368C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x492A);    // 2000368E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3001);    // 20003690 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8048);    // 20003692 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x702C);    // 20003694 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 20003696 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB5F8);    // 20003698 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4605);    // 2000369A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x460E);    // 2000369C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4617);    // 2000369E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x461C);    // 200036A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4815);    // 200036A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2181);    // 200036A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3820);    // 200036A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6980);    // 200036A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5C09);    // 200036AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0789);    // 200036AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD509);    // 200036AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3020);    // 200036B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7801);    // 200036B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4821);    // 200036B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8880);    // 200036B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200036B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF98E);    // 200036BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2802);    // 200036BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD900);    // 200036BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E80);    // 200036C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1904);    // 200036C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x481E);    // 200036C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8047);    // 200036C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8084);    // 200036C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80C5);    // 200036CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8106);    // 200036CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 200036CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4601);    // 200036D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4341);    // 200036D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1309);    // 200036D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4A1B);    // 200036D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4B1B);    // 200036D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x434A);    // 200036DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1412);    // 200036DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A9A);    // 200036DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4351);    // 200036E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4A1A);    // 200036E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1409);    // 200036E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A51);    // 200036E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4348);    // 200036E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1340);    // 200036EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4770);    // 200036EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200036EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2370);    // 200036F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200036F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E4);    // 200036F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200036F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2EBC);    // 200036F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200036FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0140);    // 200036FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 200036FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB080);    // 20003700 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 20003702 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x12CC);    // 20003704 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003706 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3AE0);    // 20003708 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000370A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xA000);    // 2000370C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 2000370E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0BB8);    // 20003710 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003712 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A10);    // 20003714 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003716 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1224);    // 20003718 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000371A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2590);    // 2000371C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000371E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3A98);    // 20003720 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003722 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2242);    // 20003724 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003726 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1100);    // 20003728 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 2000372A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F34);    // 2000372C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000372E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2138);    // 20003730 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003732 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x256A);    // 20003734 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003736 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xC100);    // 20003738 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 2000373A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0D98);    // 2000373C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000373E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3200);    // 20003740 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 20003742 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x04F6);    // 20003744 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003746 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2951);    // 20003748 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 2000374A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6487);    // 2000374C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 2000374E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4340);    // 20003750 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1300);    // 20003752 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x498C);    // 20003754 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4A8D);    // 20003756 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4341);    // 20003758 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1409);    // 2000375A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A51);    // 2000375C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4348);    // 2000375E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x498B);    // 20003760 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x13C0);    // 20003762 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A08);    // 20003764 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE7C1);    // 20003766 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB500);    // 20003768 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 2000376A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0349);    // 2000376C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1841);    // 2000376E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0B89);    // 20003770 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x038A);    // 20003772 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A80);    // 20003774 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0789);    // 20003776 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F89);    // 20003778 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD007);    // 2000377A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2901);    // 2000377C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD008);    // 2000377E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2902);    // 20003780 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD009);    // 20003782 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 20003784 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFE4);    // 20003786 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4240);    // 20003788 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD00);    // 2000378A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 2000378C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFA0);    // 2000378E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD00);    // 20003790 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 20003792 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFDD);    // 20003794 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD00);    // 20003796 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 20003798 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF9A);    // 2000379A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4240);    // 2000379C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD00);    // 2000379E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 200037A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0389);    // 200037A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1840);    // 200037A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE7DF);    // 200037A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB510);    // 200037A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4604);    // 200037AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4620);    // 200037AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 200037AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFDB);    // 200037B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4978);    // 200037B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6048);    // 200037B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4620);    // 200037B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 200037B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFF2);    // 200037BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4976);    // 200037BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6008);    // 200037BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD10);    // 200037C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB5F8);    // 200037C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4604);    // 200037C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4875);    // 200037C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2108);    // 200037C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5E41);    // 200037CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0208);    // 200037CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF7FF);    // 200037CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFEB);    // 200037D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4E70);    // 200037D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4D72);    // 200037D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x68B1);    // 200037D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2701);    // 200037D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0908);    // 200037DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);    // 200037DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x070B);    // 200037DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A2A);    // 200037E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F1B);    // 200037E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4639);    // 200037E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4099);    // 200037E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x438A);    // 200037E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x522A);    // 200037EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4620);    // 200037EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200037EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF92F);    // 200037F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x200C);    // 200037F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5E20);    // 200037F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4968);    // 200037F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6809);    // 200037F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4341);    // 200037FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x13C9);    // 200037FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A1);    // 200037FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8121);    // 20003800 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6871);    // 20003802 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4348);    // 20003804 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x13C0);    // 20003806 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81E0);    // 20003808 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8160);    // 2000380A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4865);    // 2000380C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4966);    // 2000380E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8800);    // 20003810 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2804);    // 20003812 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD103);    // 20003814 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x888A);    // 20003816 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8062);    // 20003818 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x884A);    // 2000381A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A2);    // 2000381C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2805);    // 2000381E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD106);    // 20003820 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003822 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8120);    // 20003824 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x81A0);    // 20003826 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8908);    // 20003828 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8060);    // 2000382A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x88C8);    // 2000382C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x80A0);    // 2000382E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x68B0);    // 20003830 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0901);    // 20003832 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0049);    // 20003834 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);    // 20003836 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A6A);    // 20003838 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F00);    // 2000383A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4087);    // 2000383C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x433A);    // 2000383E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x526A);    // 20003840 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF8);    // 20003842 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB5FF);    // 20003844 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB081);    // 20003846 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4605);    // 20003848 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x461E);    // 2000384A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4669);    // 2000384C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2008);    // 2000384E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5E08);    // 20003850 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x898A);    // 20003852 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A81);    // 20003854 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD500);    // 20003856 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A11);    // 20003858 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4668);    // 2000385A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x230A);    // 2000385C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5EC3);    // 2000385E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x89C2);    // 20003860 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1A98);    // 20003862 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD500);    // 20003864 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1AD0);    // 20003866 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1808);    // 20003868 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB284);    // 2000386A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2061);    // 2000386C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5D40);    // 2000386E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9F02);    // 20003870 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 20003872 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD103);    // 20003874 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4639);    // 20003876 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4628);    // 20003878 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 2000387A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8EF);    // 2000387C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x42B4);    // 2000387E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD315);    // 20003880 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4849);    // 20003882 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8800);    // 20003884 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x42A0);    // 20003886 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD301);    // 20003888 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 2000388A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE000);    // 2000388C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2100);    // 2000388E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9A0A);    // 20003890 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4291);    // 20003892 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD90D);    // 20003894 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4639);    // 20003896 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4628);    // 20003898 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 2000389A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8E5);    // 2000389C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4639);    // 2000389E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4628);    // 200038A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200038A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8E1);    // 200038A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4639);    // 200038A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4628);    // 200038A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200038AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8DD);    // 200038AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB005);    // 200038AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBDF0);    // 200038B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x42A0);    // 200038B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD204);    // 200038B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4639);    // 200038B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4628);    // 200038B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200038BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8CF);    // 200038BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE7F6);    // 200038BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4639);    // 200038C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4628);    // 200038C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200038C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8D0);    // 200038C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE7F1);    // 200038C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB510);    // 200038CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8900);    // 200038CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4937);    // 200038CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8148);    // 200038D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4837);    // 200038D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8E00);    // 200038D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x08C0);    // 200038D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200038D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8CC);    // 200038DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD10);    // 200038DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB570);    // 200038DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4834);    // 200038E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x69C1);    // 200038E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);    // 200038E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4788);    // 200038E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4D33);    // 200038E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x462C);    // 200038EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x69E8);    // 200038EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3460);    // 200038EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2800);    // 200038F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD026);    // 200038F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6AA9);    // 200038F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2900);    // 200038F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD023);    // 200038F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);    // 200038FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 200038FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF86C);    // 200038FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8060);    // 20003900 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8860);    // 20003902 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2101);    // 20003904 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0349);    // 20003906 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4288);    // 20003908 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD300);    // 2000390A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4608);    // 2000390C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8060);    // 2000390E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4A2A);    // 20003910 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2303);    // 20003912 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1F91);    // 20003914 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003916 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8B3);    // 20003918 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8020);    // 2000391A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4A27);    // 2000391C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8860);    // 2000391E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3210);    // 20003920 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4611);    // 20003922 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2305);    // 20003924 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x390A);    // 20003926 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003928 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8AA);    // 2000392A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4922);    // 2000392C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB280);    // 2000392E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3140);    // 20003930 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8308);    // 20003932 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6AA9);    // 20003934 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4341);    // 20003936 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x481D);    // 20003938 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A09);    // 2000393A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3080);    // 2000393C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6341);    // 2000393E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD70);    // 20003940 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x20FF);    // 20003942 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3001);    // 20003944 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8060);    // 20003946 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE7DB);    // 20003948 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB508);    // 2000394A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4818);    // 2000394C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4669);    // 2000394E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3014);    // 20003950 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF000);    // 20003952 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF89B);    // 20003954 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4916);    // 20003956 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4817);    // 20003958 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6A4A);    // 2000395A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6202);    // 2000395C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6A8A);    // 2000395E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6242);    // 20003960 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x460A);    // 20003962 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3280);    // 20003964 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6B52);    // 20003966 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6282);    // 20003968 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x466B);    // 2000396A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4602);    // 2000396C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x881B);    // 2000396E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8653);    // 20003970 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x466B);    // 20003972 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x885B);    // 20003974 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8693);    // 20003976 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8F0A);    // 20003978 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8582);    // 2000397A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8F4A);    // 2000397C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x85C2);    // 2000397E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8F89);    // 20003980 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8601);    // 20003982 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD08);    // 20003984 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003986 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0FC4);    // 20003988 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 2000398A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4EEA);    // 2000398C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 2000398E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7FFF);    // 20003990 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003992 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3AE0);    // 20003994 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 20003996 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3ADC);    // 20003998 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000399A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F26);    // 2000399C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 2000399E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1100);    // 200039A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 200039A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F30);    // 200039A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200039A6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4780);    // 200039A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200039AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF2A0);    // 200039AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD000);    // 200039AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2138);    // 200039B0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200039B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0148);    // 200039B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200039B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2500);    // 200039B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200039BA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0CDA);    // 200039BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);    // 200039BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 200039C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 200039C2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 200039C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 200039C6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0895);    // 200039C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);    // 200039CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 200039CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 200039CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 200039D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 200039D2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x092B);    // 200039D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);    // 200039D6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 200039D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 200039DA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 200039DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 200039DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0DF1);    // 200039E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);    // 200039E2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 200039E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 200039E6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 200039E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 200039EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFCBF);    // 200039EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200039EE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 200039F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 200039F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 200039F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 200039F6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0895);    // 200039F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 200039FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 200039FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 200039FE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A00 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A02 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC2D);    // 20003A04 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A06 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A08 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A0A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A0C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A0E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0371);    // 20003A10 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A12 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A14 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A16 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A18 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A1A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9F27);    // 20003A1C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A1E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A20 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A22 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A24 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A26 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x745B);    // 20003A28 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A2A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A2C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A2E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A30 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A32 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x990D);    // 20003A34 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A36 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A38 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A3A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A3C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A3E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7447);    // 20003A40 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A42 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A44 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A46 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A48 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A4A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xA1DD);    // 20003A4C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A4E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A50 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A52 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A54 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A56 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xEB93);    // 20003A58 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A5A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A5C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A5E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A60 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A62 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xAC4F);    // 20003A64 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A66 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A68 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A6A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A6C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A6E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xC72F);    // 20003A70 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A72 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);    // 20003A74 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);    // 20003A76 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A78 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A7A
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9E1B);     // 20003A7C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);     // 20003A7E
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);     // 20003A80
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);     // 20003A82
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);     // 20003A84
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);     // 20003A86
S5K5EAYX_write_cmos_sensor(0x0F12, 0xA061);     // 20003A88
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);     // 20003A8A
S5K5EAYX_write_cmos_sensor(0x0F12, 0xB403);     // 20003A8C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4801);     // 20003A8E
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9001);    // 20003A90  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBD01);    // 20003A92  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xAC83);    // 20003A94  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003A96  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0083);    // 20003A98  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00FF);    // 20003A9A  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0084);    // 20003A9C  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00FF);    // 20003A9E  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0038);    // 20003AA0  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x007F);    // 20003AA2  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0039);    // 20003AA4  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x007F);    // 20003AA6  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003A);    // 20003AA8  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x007F);    // 20003AAA  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003B);    // 20003AAC  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x007F);    // 20003AAE  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003C);    // 20003AB0  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);    // 20003AB2  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003D);    // 20003AB4  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);    // 20003AB6  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);    // 20003AB8  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003ABA  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);    // 20003ABC  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003ABE  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC0);    // 20003AC0  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003AC2  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC0);    // 20003AC4  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003AC6  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC0);    // 20003AC8  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003ACA  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC0);    // 20003ACC  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003ACE  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFFC);    // 20003AD0  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003AD2  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFFC);    // 20003AD4  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003AD6  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x37FF);    // 20003AD8  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003ADA  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);    // 20003ADC  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    // 20003ADE  

// End of Patch Data(Last : 20003ADEh) 
// Total Size 2300 (0x08FC)            
// Addr : 31E4 , Size : 2298(8FAh)     
//>2014/6/12-39517-joubert.she, 
//TNP_WAKEUP_MIPI2LANE_ULPS	
//TNP_GAS_OTP_PAGE_SELECT	
//TNP_AWB_MODUL_COMP		
//TNP_USER_SHARP_BLUR		
//TNP_AE_HDR_CONTROL		
//TNP_FLS_FRAME_SKIP_FIX	
//TNP_EOL_WA_2ND_VER		
//TNP_HUE_CONTROL			
//TNP_AWB_CONVERGENCE		
//TNP_SEPIA_AQUA_CONTROL	
//TNP_SHBN_ANG_TUNE_FIX		
S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);	//OTP settings
S5K5EAYX_write_cmos_sensor(0x002A, 0x149A);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);	//ash_bUseOTPData(1: useLSC OTP)
S5K5EAYX_write_cmos_sensor(0x002A, 0x14A2);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	
S5K5EAYX_write_cmos_sensor(0x002A, 0x122C);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	// awbb_otp_disable (0:use AWB OTP)
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A32);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);	
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A2C);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);	
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A2E);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);	
S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);	
S5K5EAYX_write_cmos_sensor(0x002A, 0x1000);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);	

S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);                                         
S5K5EAYX_write_cmos_sensor(0x002A, 0x122C);                                         
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);                                         
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A32);    
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);                                         
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A2C);                                         
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);                                         
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A2E);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
S5K5EAYX_write_cmos_sensor(0x002A, 0x4780);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);  // FilterMaxThr2

S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);
S5K5EAYX_write_cmos_sensor(0x002A, 0x1000);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);

S5K5EAYX_write_cmos_sensor(0x002A, 0xF400);  //0xD000F400 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x443F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2020);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0B0D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8008);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF410);  //0xD000F410 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5777);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF414);  //0xD000F414 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF41E);  //0xD000F41E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1111);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF424);  //0xD000F424 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5300);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0209);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1037);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0081);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF432);  //0xD000F432 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0508);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0509);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x08F9);  //VPIX 80F9
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1002);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF5B8);  //0xD000F5B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0070);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A0);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B0);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x002F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x005F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x005F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x008F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x008F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00BE);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xE502);  //0xD000E502 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0820);  // bpr_ob
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // bpr_active
S5K5EAYX_write_cmos_sensor(0x002A, 0xE600);  //0xD000E600 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  // adlc_config
S5K5EAYX_write_cmos_sensor(0x002A, 0xE606);  //0xD000E606 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0125);  // adlc_enable
S5K5EAYX_write_cmos_sensor(0x002A, 0xE602);  //0xD000E602 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  // adlc_data_pedestal
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1FC0);  // adlc_data_depedestal off
S5K5EAYX_write_cmos_sensor(0x002A, 0xE61E);  //0xD000E61E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // ptune_gain_total
S5K5EAYX_write_cmos_sensor(0x002A, 0xE628);  //0xD000E628 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // ptune_offset_total
S5K5EAYX_write_cmos_sensor(0x002A, 0xE614);  //0xD000E614 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2010);  // adlc_fadlc_filter_co
S5K5EAYX_write_cmos_sensor(0x002A, 0xE62E);  //0xD000E62E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);  // adlc_fadlc_filter_config
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // adlc_refresh_level_diff_threshold
S5K5EAYX_write_cmos_sensor(0x002A, 0xF482);  //0xD000F482 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0613);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0613);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF48A);  //0xD000F48A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0615);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF48E);  //0xD000F48E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0617);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0205);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0258);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0617);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0615);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0205);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0613);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0278);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02BE);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4AA);  //0xD000F4AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00AF);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4AE);  //0xD000F4AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00BE);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4B2);  //0xD000F4B2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C1);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4B6);  //0xD000F4B6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0258);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0273);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF5F4);  //0xD000F5F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C1);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4CA);  //0xD000F4CA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0615);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4CE);  //0xD000F4CE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0104);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0301);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0611);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0226);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0216);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0226);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0208);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4EA);  //0xD000F4EA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0226);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF4F2);  //0xD000F4F2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0208);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0104);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0175);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0185);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF500);  //0xD000F500 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0301);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047A);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x048A);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0611);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0176);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020A);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0212);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0227);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0482);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0615);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x061D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0178);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0213);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0221);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0228);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0483);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0617);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x061E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0179);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0213);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0228);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0483);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0618);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x061E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0176);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0178);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0208);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0613);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0615);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF55C);  //0xD000F55C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0211);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0223);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0228);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0481);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0484);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0619);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x061E);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF574);  //0xD000F574 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0176);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0178);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0208);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x047D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0613);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0615);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF58A);  //0xD000F58A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0481);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0619);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0103);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0205);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0612);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF59E);  //0xD000F59E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A6B);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF5FA);  //0xD000F5FA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0208);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF456);  //0xD000F456 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF5A2);  //0xD000F5A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0617);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C1);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C3);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0206);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0209);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0613);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0616);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0209);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A60);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A70);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xC342);  //0xD000C342 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A72);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xC200);  //0xD000C200 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A17);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xE300);  //0xD000E300 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF430);  //0xD000F430 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0E10);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xC202);  //0xD000C202 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF422);  //0xD000F422 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000E);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF2AA);  //0xD000F2AA 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF40E);  //0xD000F40E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0071);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF42E);  //0xD000F42E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A6);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF412);  //0xD000F412 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C8);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF420);  //0xD000F420 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1000);  
S5K5EAYX_write_cmos_sensor(0x002A, 0xF40C);  //0xD000F40C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  
S5K5EAYX_write_cmos_sensor(0x002A, 0x0054);  //0xD0000054 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x002D);  // Doubler Clock Enable
S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);  //Page : 2000 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0D04);  //0x20000D04 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0044);  //                  senHal_usWidthStOfsInit 0068 FW default fault 48
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0034);  //                 senHal_usHeightStOfsInit 0052 36->34 for white line issue(20130513)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0072);  //                          senHal_uAddCols 0114 Full mode: adding line line_length_pck 2608
S5K5EAYX_write_cmos_sensor(0x0F12, 0x082A);  //                       senHal_uMinAngCols 2090  	//Bin mode: min line_length_pck 2090 //
S5K5EAYX_write_cmos_sensor(0x002A, 0x0CF8);  //0x20000CF8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                      senHal_SamplingMode 0001 0:sub sampling, 1: average subsampling
S5K5EAYX_write_cmos_sensor(0x002A, 0x0D9E);  //0x20000D9E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                      senHal_Comp1AgainTh 0064 gain > x2, use Tune2 register
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                        senHal_Comp1Tune1 0000 comp1_bias=0d
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);  //                        senHal_Comp1Tune2 0007 comp1_bias=7d
S5K5EAYX_write_cmos_sensor(0x002A, 0x09DE);  //0x200009DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       skl_bHWstbyNoDelay 0000 
S5K5EAYX_write_cmos_sensor(0x002A, 0x09E4);  //0x200009E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF400);  //                skl_usStbyBackupReg_0__0_ 62464 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x005A);  //                skl_usStbyBackupReg_0__1_ 0090 1F
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF5B8);  //                skl_usStbyBackupReg_1__0_ 62904 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0078);  //                skl_usStbyBackupReg_1__1_ 0120 1E
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE502);  //                skl_usStbyBackupReg_2__0_ 58626 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //                skl_usStbyBackupReg_2__1_ 0004 2
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE600);  //                skl_usStbyBackupReg_3__0_ 58880 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003E);  //                skl_usStbyBackupReg_3__1_ 0062 19
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF482);  //                skl_usStbyBackupReg_4__0_ 62594 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x013C);  //                skl_usStbyBackupReg_4__1_ 0316 9E
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE300);  //                skl_usStbyBackupReg_5__0_ 58112 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                skl_usStbyBackupReg_5__1_ 0002 0001
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF5F4);  //                skl_usStbyBackupReg_6__0_ 62964 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                skl_usStbyBackupReg_6__1_ 0002 0001
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                skl_usStbyBackupReg_7__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                skl_usStbyBackupReg_7__1_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                skl_usStbyBackupReg_8__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                skl_usStbyBackupReg_8__1_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                skl_usStbyBackupReg_9__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                skl_usStbyBackupReg_9__1_ 0000 
S5K5EAYX_write_cmos_sensor(0x002A, 0x14AC);  //0x200014AC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                         ash_bUseGasAlpha 0001 #ash_bUseGasAlpha
S5K5EAYX_write_cmos_sensor(0x002A, 0x149C);  //0x2000149C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                        ash_bUseAutoStart 0001 #ash_bUseAutoStart	
S5K5EAYX_write_cmos_sensor(0x002A, 0x1498);  //0x20001498 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                          ash_bUseGainCal 0001 #ash_bUseGainCal	
S5K5EAYX_write_cmos_sensor(0x002A, 0x149E);  //0x2000149E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                       ash_bWriteEdgemode 0001 #ash_bWriteEdgemode
S5K5EAYX_write_cmos_sensor(0x002A, 0x1478);  //0x20001478 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                       wbt_bUseOutdoorASH 0001 wbt_bUseOutdoorASH
S5K5EAYX_write_cmos_sensor(0x002A, 0x1480);  //0x20001480 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);  //                                      ash 0192 TVAR_ash_AwbAshCord_0_ 2300K
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00DF);  //                   TVAR_ash_AwbAshCord_1_ 0223 TVAR_ash_AwbAshCord_1_ 2750K
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //                   TVAR_ash_AwbAshCord_2_ 0256 TVAR_ash_AwbAshCord_2_ 3300K
S5K5EAYX_write_cmos_sensor(0x0F12, 0x011F);  //                   TVAR_ash_AwbAshCord_3_ 0287 TVAR_ash_AwbAshCord_3_ 4150K
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0158);  //                   TVAR_ash_AwbAshCord_4_ 0418 TVAR_ash_AwbAshCord_4_ 5250K
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0178);  //                   TVAR_ash_AwbAshCord_5_ 0508 TVAR_ash_AwbAshCord_5_ 6400K
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01FF);  //                   TVAR_ash_AwbAshCord_6_ 0511 TVAR_ash_AwbAshCord_6_ 7500K
S5K5EAYX_write_cmos_sensor(0x002A, 0x14AE);  //0x200014AE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6000);  //                  TVAR_ash_GASAlpha_0__0_ 20480 #TVAR_ash_GASAlpha_0__0_ Hor
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_0__1_ 16384 #TVAR_ash_GASAlpha_0__1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_0__2_ 16384 #TVAR_ash_GASAlpha_0__2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_0__3_ 16384 #TVAR_ash_GASAlpha_0__3_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5400);  //                  TVAR_ash_GASAlpha_1__0_ 20480 #TVAR_ash_GASAlpha_1__0_ Inca
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_1__1_ 16384 #TVAR_ash_GASAlpha_1__1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_1__2_ 16384 #TVAR_ash_GASAlpha_1__2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_1__3_ 16384 #TVAR_ash_GASAlpha_1__3_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_2__0_ 20480 #TVAR_ash_GASAlpha_2__0_ WWF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_2__1_ 16384 #TVAR_ash_GASAlpha_2__1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_2__2_ 16384 #TVAR_ash_GASAlpha_2__2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_2__3_ 16384 #TVAR_ash_GASAlpha_2__3_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4400);  //                  TVAR_ash_GASAlpha_3__0_ 17152 #TVAR_ash_GASAlpha_3__0_ CWF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_3__1_ 16384 #TVAR_ash_GASAlpha_3__1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_3__2_ 16384 #TVAR_ash_GASAlpha_3__2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_3__3_ 16384 #TVAR_ash_GASAlpha_3__3_
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4800);  //                  TVAR_ash_GASAlpha_4__0_ 16384 #TVAR_ash_GASAlpha_4__0_ D50
//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_4__1_ 12288 #TVAR_ash_GASAlpha_4__1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_4__2_ 12288 #TVAR_ash_GASAlpha_4__2_
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3D00);  //                  TVAR_ash_GASAlpha_4__3_ 13568 #TVAR_ash_GASAlpha_4__3_
//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4800);  //                  TVAR_ash_GASAlpha_5__0_ 16384 #TVAR_ash_GASAlpha_5__0_ D65
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_5__1_ 16384 #TVAR_ash_GASAlpha_5__1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_5__2_ 16384 #TVAR_ash_GASAlpha_5__2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_5__3_ 16384 #TVAR_ash_GASAlpha_5__3_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_6__0_ 16384 #TVAR_ash_GASAlpha_6__0_ D75
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_6__1_ 16384 #TVAR_ash_GASAlpha_6__1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_6__2_ 16384 #TVAR_ash_GASAlpha_6__2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //                  TVAR_ash_GASAlpha_6__3_ 16384 #TVAR_ash_GASAlpha_6__3_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4D00);  //              TVAR_ash_GASOutdoorAlpha_0_ 16384  R   //by 7500K
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //              TVAR_ash_GASOutdoorAlpha_1_ 16384  GR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //              TVAR_ash_GASOutdoorAlpha_2_ 16384  GB
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4000);  //              TVAR_ash_GASOutdoorAlpha_3_ 16384  B
//>2014/5/2-37049-joubert.she
S5K5EAYX_write_cmos_sensor(0x002A, 0x1860);  //0x20001860 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x343F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8D9);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x089B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFD19);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xEE4F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2354);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBF4);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC0F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF9BC);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0AAE);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x04A8);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xEAEC);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF85C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x06DB);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x051A);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF794);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF439);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1EBD);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0E8C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF9BB);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEC5);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03FF);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x05C9);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE8FC);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFAA8);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE2F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF19);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03BA);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0192);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x06CB);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB89);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x089E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE44);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB4E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFD2E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0212);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3720);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA7A);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x087D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF5F1);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01AE);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0E91);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFCFC);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF786);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE59);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A89);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB6A);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF957);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF5DC);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x08E2);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0025);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB7B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF966);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1092);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1095);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBA4);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0166);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0006);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x029C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF622);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC0D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBE1);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFCE2);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x06A8);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0485);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC4F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8C9);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x08C1);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E2);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF6E3);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE31);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x054E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x300F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF924);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0B8E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF2E1);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0177);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1241);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC8B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB29);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFD46);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0852);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFD64);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF70F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB41);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x082F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E5);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB87);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA82);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x11E4);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x08A2);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8A5);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE86);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0594);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFED4);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF310);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF17);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B1);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x014E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFB3);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03A5);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0522);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA9D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x058E);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFD53);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF03);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBCA);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0181);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3474);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF955);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0852);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA9B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF50C);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1D11);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFCAE);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF8A0);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFECA);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0993);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBB4);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF6A2);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF89F);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0AC8);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE49);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFB6B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFBC7);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1175);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A5B);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF9B4);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFF4);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03A0);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x007D);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF325);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0226);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA07);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03D6);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F1);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFB8);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x05AE);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF642);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C05);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFA56);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC60);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C3);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC9);  
S5K5EAYX_write_cmos_sensor(0x002A, 0x01E8);  //0x200001E8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      REG_TC_IPRM_LedGpio 0000 REG_TC_IPRM_LedGpio	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //           REG_TC_IPRM_CM_Init_AfModeType 0003 REG_TC_IPRM_CM_Init_AfModeType
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //           REG_TC_IPRM_CM_Init_PwmConfig1 0000 REG_TC_IPRM_CM_Init_PwmConfig1
S5K5EAYX_write_cmos_sensor(0x002A, 0x01F0);  //0x200001F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_TC_IPRM_CM_Init_GpioConfig1 0000 REG_TC_IPRM_CM_Init_GpioConfig1
S5K5EAYX_write_cmos_sensor(0x002A, 0x01F8);  //0x200001F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000C);  //             REG_TC_IPRM_CM_Init_Mi2cBits 0012 REG_TC_IPRM_CM_Init_Mi2cBits [Data:Clock:ID] GPIO 1,2
S5K5EAYX_write_cmos_sensor(0x002A, 0x01FC);  //0x200001FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0064);  //          REG_TC_IPRM_CM_Init_Mi2cRateKhz 0100 REG_TC_IPRM_CM_Init_Mi2cRateKhz IIC Speed 100Khz
S5K5EAYX_write_cmos_sensor(0x002A, 0x02D0);  //0x200002D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //                   REG_TC_AF_FstWinStartX 0256 REG_TC_AF_FstWinStartX
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E3);  //                   REG_TC_AF_FstWinStartY 0227 REG_TC_AF_FstWinStartY
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);  //                    REG_TC_AF_FstWinSizeX 0512 REG_TC_AF_FstWinSizeX
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0238);  //                    REG_TC_AF_FstWinSizeY 0568 REG_TC_AF_FstWinSizeY
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //                  REG_TC_AF_ScndWinStartX 0454 REG_TC_AF_ScndWinStartX
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0166);  //                  REG_TC_AF_ScndWinStartY 0358 REG_TC_AF_ScndWinStartY
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0074);  //                   REG_TC_AF_ScndWinSizeX 0116 REG_TC_AF_ScndWinSizeX
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0132);  //                   REG_TC_AF_ScndWinSizeY 0306 REG_TC_AF_ScndWinSizeY
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                REG_TC_AF_WinSizesUpdated 0001 REG_TC_AF_WinSizesUpdated
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A1A);  //0x20000A1A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00FF);  //                 skl_af_StatOvlpExpFactor 0255 skl_af_StatOvlpExpFactor
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A2A);  //0x20000A2A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                        skl_af_bAfStatOff 0000 skl_af_bAfStatOff
S5K5EAYX_write_cmos_sensor(0x002A, 0x0660);  //0x20000660 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                     af_search_usAeStable 0000 af_search_usAeStable
S5K5EAYX_write_cmos_sensor(0x002A, 0x066C);  //0x2000066C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1002);  //                af_search_usSingleAfFlags 4098 af_search_usSingleAfFlags,  Double peak , Fine search enable,
S5K5EAYX_write_cmos_sensor(0x002A, 0x0676);  //0x20000676 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                af_search_usFinePeakCount 0002 af_search_usFinePeakCount
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                 af_search_usFineMaxScale 0000 af_search_usFineMaxScale
S5K5EAYX_write_cmos_sensor(0x002A, 0x0670);  //0x20000670 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //               af_search_usMinPeakSamples 0003 af_search_usMinPeakSamples
S5K5EAYX_write_cmos_sensor(0x002A, 0x0662);  //0x20000662 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E5);  //                      af_search_usPeakThr 0229 af_search_usPeakThr,  Full search (E5 90%)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0098);  //                   af_search_usPeakThrLow 0152 af_search_usPeakThrLow
S5K5EAYX_write_cmos_sensor(0x002A, 0x06BE);  //0x200006BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF95);  //            af_search_usConfCheckOrder_1_ 65429 af_search_usConfCheckOrder_1_
S5K5EAYX_write_cmos_sensor(0x002A, 0x068E);  //0x2000068E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0280);  //                   af_search_usConfThr_4_ 0640 af_search_usConfThr_4_
S5K5EAYX_write_cmos_sensor(0x002A, 0x069A);  //0x2000069A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03A0);  //                  af_search_usConfThr_10_ 0928 af_search_usConfThr_10_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0320);  //                  af_search_usConfThr_11_ 0800 af_search_usConfThr_11_
S5K5EAYX_write_cmos_sensor(0x002A, 0x06E0);  //0x200006E0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                     af_stat_usMinStatVal 0048 af_stat_usMinStatVal
S5K5EAYX_write_cmos_sensor(0x002A, 0x0710);  //0x20000710 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0060);  //             af_scene_usSceneLowNormBrThr 0096 af_scene_usSceneLowNormBrThr
S5K5EAYX_write_cmos_sensor(0x002A, 0x06F8);  //0x200006F8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  //                      af_stat_usBpfThresh 0016 af_stat_usBpfThresh
S5K5EAYX_write_cmos_sensor(0x002A, 0x067A);  //0x2000067A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                af_search_usCapturePolicy 0000 af_search_usCapturePolicy
S5K5EAYX_write_cmos_sensor(0x002A, 0x05D4);  //0x200005D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0086);  //                 af_pos_usCaptureFixedPos 0134 af_pos_usCaptureFixedPos
S5K5EAYX_write_cmos_sensor(0x002A, 0x05B8);  //0x200005B8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                         af_pos_usHomePos 0128 af_pos_usHomePos
S5K5EAYX_write_cmos_sensor(0x002A, 0x05BC);  //0x200005BC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                      af_pos_usLowConfPos 0128 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0174);  //af_pos_usLowConfPos normal:128 , Macro: 372
S5K5EAYX_write_cmos_sensor(0x002A, 0x05C0);  //0x200005C0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0120);  //                       af_pos_usMiddlePos 0288 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0120);  //af_pos_usMiddlePos
S5K5EAYX_write_cmos_sensor(0x002A, 0x05D8);  //0x200005D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0014);  //                    af_pos_usTableLastInd 0020 af_pos_usTableLastInd
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0086);  //                        af_pos_usTable_0_ 0134 af_pos_usTable_0_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0094);  //                        af_pos_usTable_1_ 0148 af_pos_usTable_1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A2);  //                        af_pos_usTable_2_ 0162 af_pos_usTable_2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B0);  //                        af_pos_usTable_3_ 0176 af_pos_usTable_3_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00BE);  //                        af_pos_usTable_4_ 0190 af_pos_usTable_4_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00CC);  //                        af_pos_usTable_5_ 0204 af_pos_usTable_5_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00DA);  //                        af_pos_usTable_6_ 0218 af_pos_usTable_6_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E8);  //                        af_pos_usTable_7_ 0232 af_pos_usTable_7_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00F6);  //                        af_pos_usTable_8_ 0246 af_pos_usTable_8_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0104);  //                        af_pos_usTable_9_ 0260 af_pos_usTable_9_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0112);  //                       af_pos_usTable_10_ 0274 af_pos_usTable_10_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0120);  //                       af_pos_usTable_11_ 0288 af_pos_usTable_11_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x012E);  //                       af_pos_usTable_12_ 0302 af_pos_usTable_12_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x013C);  //                       af_pos_usTable_13_ 0316 af_pos_usTable_13_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x014A);  //                       af_pos_usTable_14_ 0330 af_pos_usTable_14_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0158);  //                       af_pos_usTable_15_ 0344 af_pos_usTable_15_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0166);  //                       af_pos_usTable_16_ 0358 af_pos_usTable_16_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0174);  //                       af_pos_usTable_17_ 0372 af_pos_usTable_17_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0182);  //                       af_pos_usTable_18_ 0386 af_pos_usTable_18_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0190);  //                       af_pos_usTable_19_ 0400 af_pos_usTable_19_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x019A);  //                       af_pos_usTable_20_ 0410 af_pos_usTable_20_
S5K5EAYX_write_cmos_sensor(0x002A, 0x071A);  //0x2000071A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5050);  //                     af_refocus_usFlUpLow 20560 af_refocus_usFlUpLow
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8080);  //                   af_refocus_usFlLcUpLow 32896 af_refocus_usFlLcUpLow
S5K5EAYX_write_cmos_sensor(0x002A, 0x0752);  //0x20000752 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E0);  //                 af_scd_usSceneMoveThresh 0992 Sensitivity for Normal scene
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03C0);  //              af_scd_usSceneMoveThreshLow 0960 Sensitivity for Low light scene
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E0);  //              af_scd_usSceneMoveThreshOut 0992 Sensitivity forOutdoor scene
S5K5EAYX_write_cmos_sensor(0x002A, 0x056A);  //0x2000056A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                           afd_usParam_0_ 32768 	power down flag.[15] bit set 1.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //                           afd_usParam_1_ 0004   Shift
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3FF0);  //                           afd_usParam_2_ 16368   Mask
S5K5EAYX_write_cmos_sensor(0x002A, 0x0574);  //0x20000574 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                           afd_usParam_5_ 0032   Slow motion delay
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                           afd_usParam_6_ 0048   Moving distance threshold for Slow motion delay
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  //                           afd_usParam_7_ 0016   Signal shaping delay time
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                           afd_usParam_8_ 0064 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                           afd_usParam_9_ 0128 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);  //                          afd_usParam_10_ 0192 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E0);  //                          afd_usParam_11_ 0224 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0A3A);  //0x20000A3A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                 skl_ThumbStartY_OffsetAF 0002  skl_ThumbStartY_OffsetAF
S5K5EAYX_write_cmos_sensor(0x002A, 0x02C8);  //0x200002C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //                          REG_TC_AF_AfCmd 0003 REG_TC_AF_AfCmd   init,  AF initialization
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0028, 0x2000); // 20121209 kilsung
S5K5EAYX_write_cmos_sensor(0x002A, 0x04C0);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); /* capture flash on */
//S5K5EAYX_write_cmos_sensor(0x002A, 0x183A);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0C4E);
//DV0 /DV1 flash LED
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x026A); /* AWB R point */
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x027D); /* AWB B point */
//after DV1 flash LED
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0262); /* AWB R point */
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0255); /* AWB B point */

S5K5EAYX_write_cmos_sensor(0x002A, 0x0C42);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);//fls_Use_FlashAWB_Update

S5K5EAYX_write_cmos_sensor(0x002A, 0x0C52);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case

S5K5EAYX_write_cmos_sensor(0x002A, 0x0C64);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x002A, 0x1840);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); /* Fls AE tune start */
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); /* fls_afl_FlsAFIn  Rin */
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0120);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0180);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0800);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x1000);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); /* fls_afl_FlsAFOut  Rout */
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A0);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0090);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0070);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0045);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);
//S5K5EAYX_write_cmos_sensor(0x002A, 0x1884);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); /* fls_afl_FlsNBOut  flash NB default */
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x002A, 0x1826);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100); /* fls_afl_FlashWP_Weight  flash NB default */
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030); /* fls_afl_FlashWP_Weight  flash NB default */
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0048);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0060);
//S5K5EAYX_write_cmos_sensor(0x002A, 0x4784);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A0); /* TNP_Regs_FlsWeightRIn  weight tune start in*/
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D0);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0088); /* TNP_Regs_FlsWeightROut  weight tune start out*/
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B0);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);
S5K5EAYX_write_cmos_sensor(0x002A, 0x0CD4);
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0120); /*Fls  BRIn  */
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1200);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003C); /* Fls  BROut*/
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0024);


S5K5EAYX_write_cmos_sensor(0x002A, 0x0CE0);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);// fls_afl_FlsBrRatioIn_0_ 2 20000CE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0120);// fls_afl_FlsBrRatioIn_1_ 2 20000CE2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0150);// fls_afl_FlsBrRatioIn_2_ 2 20000CE4
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);// fls_afl_FlsBrRatioIn_3_ 2 20000CE6
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);// fls_afl_FlsBrRatioIn_4_ 2 20000CE8
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);// fls_afl_FlsBrRatioout_0_ 2 20000CEA
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);// fls_afl_FlsBrRatioout_1_ 2 20000CEC
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A0);// fls_afl_FlsBrRatioout_2_ 2 20000CEE
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0090);// fls_afl_FlsBrRatioout_3_ 2 20000CF0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0060);// fls_afl_FlsBrRatioout_4_ 2 20000CF2

//S5K5EAYX_write_cmos_sensor(0x002A, 0x0C42);
//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);//fls_Use_FlashAWB_Update
//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x002A, 0x1276);  //0x20001276 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                   AFC_D_ConvAccelerPower 0001 
S5K5EAYX_write_cmos_sensor(0x002A, 0x1270);  //0x20001270 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                          AFC_Default60Hz 0001 AFC_Default BIT[0] 1:60Hz 0:50Hz
S5K5EAYX_write_cmos_sensor(0x002A, 0x051C);  //0x2000051C REG_TC_DBG_AutoAlgEnBits
S5K5EAYX_write_cmos_sensor(0x0F12, 0x077F);  //                 REG_TC_DBG_AutoAlgEnBits 1919 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0B20);  //0x20000B20 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0032);  //                                       ae 0050 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0B26);  //0x20000B26 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000F);  //                              ae_StatMode 0015 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0854);  //0x20000854 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                 lt_uInitPostToleranceCnt 0002 
S5K5EAYX_write_cmos_sensor(0x002A, 0x081C);  //0x2000081C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0111);  //                            lt_uLimitHigh 0273 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00EF);  //                             lt_uLimitLow 0239 
S5K5EAYX_write_cmos_sensor(0x002A, 0x08D4);  //0x200008D4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //              lt_ExpGain_uSubsamplingmode 0001 lt_ExpGain_uSubsamplingmode
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //               lt_ExpGain_uNonSubsampling 0001 lt_ExpGain_uNonSubsampling
S5K5EAYX_write_cmos_sensor(0x002A, 0x08DC);  //0x200008DC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_0_ 0001 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A3C);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_1_ 2620 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0D05);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_2_ 3333 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1200);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_3_ 16392 //4008
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7000);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_4_ 28672 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x9C00);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_5_ 39936 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xAD00);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_6_ 44288 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xF1D4);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_7_ 61908 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xDC00);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_8_ 56320 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xDC00);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpIn_9_ 56320 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_0_ 0001 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A3C);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_1_ 2620 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0D05);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_2_ 3333 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1408);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_3_ 13320//3408 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3408);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_4_ 13320 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3408);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_5_ 27408 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3408);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_6_ 33300 //6b10
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6B10);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_7_ 50000//9350
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD4C0);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_8_ 54464 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0xD4C0);  // lt_ExpGain_ExpCurveGainMaxStr_0__ulExpOut_9_ 54464 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  
S5K5EAYX_write_cmos_sensor(0x002A, 0x08D8);  //0x200008D8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  // lt_ExpGain_ExpCurveGainMaxStr_0__uMaxAnGain 2560 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  // lt_ExpGain_ExpCurveGainMaxStr_0__uMaxDigGain 0256 
S5K5EAYX_write_cmos_sensor(0x002A, 0x086E);  //0x2000086E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //                           lt_uMaxTotGain 2560 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0984);  //0x20000984 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3380);  //                               lt_uMaxLei 13184 
S5K5EAYX_write_cmos_sensor(0x002A, 0x097A);  //0x2000097A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000C);  //lt_uMaxLei
S5K5EAYX_write_cmos_sensor(0x002A, 0x08CC);  //0x200008CC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x673E);  //                           lt_uLeiInit_0_ 26430   lt_uLeiInit
S5K5EAYX_write_cmos_sensor(0x002A, 0x0B2E);  //0x20000B2E 
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_0_ 0256 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_1_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_2_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_3_ 0001 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_4_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_5_ 0513 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_6_ 0258 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_7_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_8_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                       ae_WeightTbl_16_9_ 0514 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_10_ 0514 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_11_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_12_ 0513 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0202);  //                      ae_WeightTbl_16_13_ 0770 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0202);  //                      ae_WeightTbl_16_14_ 0515 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_15_ 0258 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0201);  //                      ae_WeightTbl_16_16_ 0513 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0403);  //                      ae_WeightTbl_16_17_ 0770 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0304);  //                      ae_WeightTbl_16_18_ 0515 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0102);  //                      ae_WeightTbl_16_19_ 0258 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0201);  //                      ae_WeightTbl_16_20_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0303);  //                      ae_WeightTbl_16_21_ 0514 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0303);  //                      ae_WeightTbl_16_22_ 0514 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0102);  //                      ae_WeightTbl_16_23_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0201);  //                      ae_WeightTbl_16_24_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0202);  //                      ae_WeightTbl_16_25_ 0513 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0202);  //                      ae_WeightTbl_16_26_ 0258 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0102);  //                      ae_WeightTbl_16_27_ 0257 
//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_28_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_29_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_30_ 0257 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      ae_WeightTbl_16_31_ 0257 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0C3E);  //0x20000C3E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                         ae_bWeightUpdtae 0001 ae_bWeightUpdtae 2013.03.8 HKS update
S5K5EAYX_write_cmos_sensor(0x002A, 0x121C);  //0x2000121C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0580);  //                        awbb_GainsInit_0_ 1408 0580	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0428);  //                        awbb_GainsInit_1_ 1064 0428	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x07B0);  //                        awbb_GainsInit_2_ 1968 0780	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0008);  //                      awbb_WpFilterMinThr 0008 0008	//
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                      awbb_WpFilterMaxThr 0400 0190	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //                        awbb_WpFilterCoef 0160 00A0	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //                        awbb_WpFilterSize 0004 0004	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                          awbb_GridEnable 0002 0002	//
S5K5EAYX_write_cmos_sensor(0x002A, 0x1208);  //0x20001208 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                            awbb_RGainOff 0000 0000	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFE0);  //                            awbb_BGainOff 65504 0000	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                            awbb_GGainOff 0000 0000	//
S5K5EAYX_write_cmos_sensor(0x002A, 0x1210);  //0x20001210 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C2);  //                     awbb_Alpha_Comp_Mode 0194 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                  awbb_Rpl_InvalidOutDoor 0002 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                        awbb_UseGrThrCorr 0001 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0074);  //                         awbb_Use_Filters 0116 
S5K5EAYX_write_cmos_sensor(0x002A, 0x121A);  //0x2000121A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                awbb_CorrectMinNumPatches 0001 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0F88);  //0x20000F88 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0120);  //                               awbb_IntcR 0288 0120//0157	//012C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0121);  //                               awbb_IntcB 0289 0121//010C	//0121
S5K5EAYX_write_cmos_sensor(0x002A, 0x0FA2);  //0x20000FA2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x041D);  //                       awbb_MvEq_RBthresh 1053 05A8//05D5//0544	//
S5K5EAYX_write_cmos_sensor(0x002A, 0x0FA6);  //0x20000FA6 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       awbb_MovingScale10 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0771);  //                      awbb_GamutWidthThr1 1905 060F//0771//05FE	//05FD
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03A4);  //                     awbb_GamutHeightThr1 0932 03A5//03A4//039C	//036B
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0036);  //                      awbb_GamutWidthThr2 0054 0x002A//0036//0042	//0020
S5K5EAYX_write_cmos_sensor(0x0F12, 0x002A);  //                     awbb_GamutHeightThr2 0042 0020//0x002A//0x002A//001A
S5K5EAYX_write_cmos_sensor(0x002A, 0x0FBC);  //0x20000FBC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //                               awbb_LowBr 0050 0032	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //                        awbb_LowBr_NBzone 0030 001E	//
S5K5EAYX_write_cmos_sensor(0x002A, 0x0F8C);  //0x20000F8C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02DF);  //                             awbb_GLocusR 0735 02DF	//02DF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0314);  //                             awbb_GLocusB 0788 0314	//0314
S5K5EAYX_write_cmos_sensor(0x002A, 0x0FD4);  //0x20000FD4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_0__0_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_0__2_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_0__4_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_1__1_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_1__3_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_2__0_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_2__2_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_2__4_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_3__1_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_3__3_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_4__0_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_4__2_ 0000 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_4__4_ 1280 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5555);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_5__1_ 21845 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5455);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_5__3_ 21589 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0xAA55);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_6__0_ 43605 5601
S5K5EAYX_write_cmos_sensor(0x0F12, 0xAAAA);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_6__2_ 43690 55A9
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBF54);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_6__4_ 48980 0A40
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFFF);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_7__1_ 65535 FFAF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x54FE);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_7__3_ 21758 54AA
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF6F);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_8__0_ 65391 FF6F
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEFF);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_8__2_ 65279 FEFF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1B54);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_8__4_ 6996 1B54
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFFF);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_9__1_ 65535 FFFF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x54FE);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_9__3_ 21758 54FE
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF06);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_10__0_ 65286 FF06
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEFF);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_10__2_ 65279 FEFF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0154);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_10__4_ 0340 0154
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBFBF);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_11__1_ 49087 BFBF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x54BE);  // awbb_SCDetectionMap_SEC_SceneDetectionMap_11__3_ 21694 54BE
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEF7);  //         awbb_SCDetectionMap_SEC_StartR_B 65271 FEF7//FF39	//FEF7//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0021);  //          awbb_SCDetectionMap_SEC_StepR_B 0033 0021//0010	//0021//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x087E);  //          awbb_SCDetectionMap_SEC_SunnyNB 2800 0AF0//0ABD	//07D0//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0122);  //           awbb_SCDetectionMap_SEC_StepNB 0290 0AF0,	//0AF0//0080	//07D0//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x018F);  //       awbb_SCDetectionMap_SEC_LowTempR_B 0399 018F//0264	//01C8//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x005A);  //      awbb_SCDetectionMap_SEC_SunnyNBZone 0150 0096//003C	//0096//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000E);  //   awbb_SCDetectionMap_SEC_LowTempR_BZone 0014 000E//0009	//0004//
S5K5EAYX_write_cmos_sensor(0x002A, 0x0FC0);  //0x20000FC0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C0);  //                         awbb_YThreshHigh 0192 00E2	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  //                     awbb_YThreshLow_Norm 0016 0010	//
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                      awbb_YThreshLow_Low 0002 0002	//
S5K5EAYX_write_cmos_sensor(0x002A, 0x2BAE);  //0x20002BAE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0006);  //                       Mon_AWB_ByPassMode 0006 0002	//
S5K5EAYX_write_cmos_sensor(0x002A, 0x0F94);  //0x20000F94 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000C);  //                awbb_MinNumOfFinalPatches 0012 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0FA0);  //0x20000FA0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //       awbb_MinNumOfChromaClassifyPatches 0032 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0DB4);  //0x20000DB4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03D6);  //     awbb_IndoorGrZones_m_BGrid_0__m_left 0804 awbb_IndoorGrZones_m_BGrid[0]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x041C);  //    awbb_IndoorGrZones_m_BGrid_0__m_right 0924 awbb_IndoorGrZones_m_BGrid[1]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0386);  //     awbb_IndoorGrZones_m_BGrid_1__m_left 0756 awbb_IndoorGrZones_m_BGrid[2]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0436);  //    awbb_IndoorGrZones_m_BGrid_1__m_right 0930 awbb_IndoorGrZones_m_BGrid[3]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0352);  //     awbb_IndoorGrZones_m_BGrid_2__m_left 0716 awbb_IndoorGrZones_m_BGrid[4]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0438);  //    awbb_IndoorGrZones_m_BGrid_2__m_right 0902 awbb_IndoorGrZones_m_BGrid[5]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0322);  //     awbb_IndoorGrZones_m_BGrid_3__m_left 0678 awbb_IndoorGrZones_m_BGrid[6]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x042C);  //    awbb_IndoorGrZones_m_BGrid_3__m_right 0854 awbb_IndoorGrZones_m_BGrid[7]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02EC);  //     awbb_IndoorGrZones_m_BGrid_4__m_left 0652 awbb_IndoorGrZones_m_BGrid[8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x040C);  //    awbb_IndoorGrZones_m_BGrid_4__m_right 0774 awbb_IndoorGrZones_m_BGrid[9]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02B6);  //     awbb_IndoorGrZones_m_BGrid_5__m_left 0616 awbb_IndoorGrZones_m_BGrid[10]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E0);  //    awbb_IndoorGrZones_m_BGrid_5__m_right 0720 awbb_IndoorGrZones_m_BGrid[11]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0290);  //     awbb_IndoorGrZones_m_BGrid_6__m_left 0592 awbb_IndoorGrZones_m_BGrid[12]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03B4);  //    awbb_IndoorGrZones_m_BGrid_6__m_right 0700 awbb_IndoorGrZones_m_BGrid[13]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0286);  //     awbb_IndoorGrZones_m_BGrid_7__m_left 0566 awbb_IndoorGrZones_m_BGrid[14]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x037A);  //    awbb_IndoorGrZones_m_BGrid_7__m_right 0680 awbb_IndoorGrZones_m_BGrid[15]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0274);  //     awbb_IndoorGrZones_m_BGrid_8__m_left 0530 awbb_IndoorGrZones_m_BGrid[16]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x033A);  //    awbb_IndoorGrZones_m_BGrid_8__m_right 0668 awbb_IndoorGrZones_m_BGrid[17]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0248);  //     awbb_IndoorGrZones_m_BGrid_9__m_left 0510 awbb_IndoorGrZones_m_BGrid[18]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //    awbb_IndoorGrZones_m_BGrid_9__m_right 0662 awbb_IndoorGrZones_m_BGrid[19]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021C);  //    awbb_IndoorGrZones_m_BGrid_10__m_left 0496 awbb_IndoorGrZones_m_BGrid[20]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02D4);  //   awbb_IndoorGrZones_m_BGrid_10__m_right 0648 awbb_IndoorGrZones_m_BGrid[21]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01EC);  //    awbb_IndoorGrZones_m_BGrid_11__m_left 0480 awbb_IndoorGrZones_m_BGrid[22]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02AA);  //   awbb_IndoorGrZones_m_BGrid_11__m_right 0628 awbb_IndoorGrZones_m_BGrid[23]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01CA);  //    awbb_IndoorGrZones_m_BGrid_12__m_left 0466 awbb_IndoorGrZones_m_BGrid[24]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x028A);  //   awbb_IndoorGrZones_m_BGrid_12__m_right 0622 awbb_IndoorGrZones_m_BGrid[25]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01BC);  //    awbb_IndoorGrZones_m_BGrid_13__m_left 0454 awbb_IndoorGrZones_m_BGrid[26]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x026C);  //   awbb_IndoorGrZones_m_BGrid_13__m_right 0600 awbb_IndoorGrZones_m_BGrid[27]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //    awbb_IndoorGrZones_m_BGrid_14__m_left 0444 awbb_IndoorGrZones_m_BGrid[28]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0250);  //   awbb_IndoorGrZones_m_BGrid_14__m_right 0582 awbb_IndoorGrZones_m_BGrid[29]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01EC);  //    awbb_IndoorGrZones_m_BGrid_15__m_left 0436 awbb_IndoorGrZones_m_BGrid[30]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0216);  //   awbb_IndoorGrZones_m_BGrid_15__m_right 0558 awbb_IndoorGrZones_m_BGrid[31]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //    awbb_IndoorGrZones_m_BGrid_16__m_left 0436 awbb_IndoorGrZones_m_BGrid[32]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //   awbb_IndoorGrZones_m_BGrid_16__m_right 0532 awbb_IndoorGrZones_m_BGrid[33]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //    awbb_IndoorGrZones_m_BGrid_17__m_left 0434 awbb_IndoorGrZones_m_BGrid[34]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //   awbb_IndoorGrZones_m_BGrid_17__m_right 0502 awbb_IndoorGrZones_m_BGrid[35]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //    awbb_IndoorGrZones_m_BGrid_18__m_left 0432 awbb_IndoorGrZones_m_BGrid[36]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //   awbb_IndoorGrZones_m_BGrid_18__m_right 0474 awbb_IndoorGrZones_m_BGrid[37]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //    awbb_IndoorGrZones_m_BGrid_19__m_left 0000 awbb_IndoorGrZones_m_BGrid[38]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //   awbb_IndoorGrZones_m_BGrid_19__m_right 0000 awbb_IndoorGrZones_m_BGrid[39]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                 awbb_IndoorGrZones_ZInfo 0005 awbb_IndoorGrZones_m_GridStep[0]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //awbb_IndoorGrZones_m_GridStep[1]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0010);  //        awbb_IndoorGrZones_ZInfo_m_GridSz 0019 awbb_IndoorGrZones_ZInfo_m_GridSz[0]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //awbb_IndoorGrZones_ZInfo_m_GridSz[1]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E8);  //               awbb_IndoorGrZones_m_Boffs 0298 awbb_IndoorGrZones_m_Boffs[0]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //awbb_IndoorGrZones_m_Boffs[1]
//Caval add outdoor  wight modifid 0522
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E44);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);    //000C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E40);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);    //0004
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E48);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01CA);    //022E  //01FA
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E10);
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02B4);    //0250  //02EC
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);    //028C  //031C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x027A);    //0240  //02A6
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0304);    //02A6  //0322
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0250);    //0236  //0298
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);    //02AE  //0322
S5K5EAYX_write_cmos_sensor(0x0F12, 0x023A);    //022C  //028A
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02F4);    //02AE  //031A
S5K5EAYX_write_cmos_sensor(0x0F12, 0x022A);    //021C  //0282
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02E4);    //02AA  //0312
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021A);    //0212  //0276
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02D2);    //02A0  //0304
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0208);    //020A  //0272
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02BA);    //0296  //0300
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01FC);    //01FE  //0270
S5K5EAYX_write_cmos_sensor(0x0F12, 0x029A);    //028C  //02F4
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F6);    //01FA  //0270
S5K5EAYX_write_cmos_sensor(0x0F12, 0x027A);    //0286  //02E6
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0216);    //01F8  //029A
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0248);    //027A  //02BC
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    //01F8  //0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    //026E  //0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    //0218  //0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);    //024A  //0000

////	param_start	awbb_LowBrGrZones_m_GridStep         
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E7C);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0006);	//awbb_LowBrGrZones_m_GridStep[0]    
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	//awbb_LowBrGrZones_m_GridStep[1] 
//   
////	param_end	awbb_LowBrGrZones_m_GridStep           
////	param_start	awbb_LowBrGrZones_ZInfo_m_GridSz     
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E80);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);	//awbb_LowBrGrZones_ZInfo_m_GridSz[0]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	//awbb_LowBrGrZones_ZInfo_m_GridSz[1]
//
////	param_end	awbb_LowBrGrZones_ZInfo_m_GridSz       
////	param_start	awbb_LowBrGrZones_m_Boffs            
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E84);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00F8);	//awbb_LowBrGrZones_m_Boffs[0]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	//awbb_LowBrGrZones_m_Boffs[1]       
//	param_end	awbb_LowBrGrZones_m_Boffs              
//
//
////	param_start	awbb_LowBrGrZones_m_BGrid            
S5K5EAYX_write_cmos_sensor(0x002A, 0x0E4C);	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03BC);	//awbb_LowBrGrZones_m_BGrid[0]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0428);	//awbb_LowBrGrZones_m_BGrid[1]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x032E);	//awbb_LowBrGrZones_m_BGrid[2]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x044C);	//awbb_LowBrGrZones_m_BGrid[3]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02C8);	//awbb_LowBrGrZones_m_BGrid[4]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x042C);	//awbb_LowBrGrZones_m_BGrid[5]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0262);	//awbb_LowBrGrZones_m_BGrid[6]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E8);	//awbb_LowBrGrZones_m_BGrid[7]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x020C);	//awbb_LowBrGrZones_m_BGrid[8]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03A4);	//awbb_LowBrGrZones_m_BGrid[9]       
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C2);	//awbb_LowBrGrZones_m_BGrid[10]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x035E);	//awbb_LowBrGrZones_m_BGrid[11]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0188);	//awbb_LowBrGrZones_m_BGrid[12]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x031A);	//awbb_LowBrGrZones_m_BGrid[13]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017C);	//awbb_LowBrGrZones_m_BGrid[14]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02D6);	//awbb_LowBrGrZones_m_BGrid[15]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0178);	//awbb_LowBrGrZones_m_BGrid[16]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0294);	//awbb_LowBrGrZones_m_BGrid[17]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x016C);	//awbb_LowBrGrZones_m_BGrid[18]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x021E);	//awbb_LowBrGrZones_m_BGrid[19]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	//awbb_LowBrGrZones_m_BGrid[20]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	//awbb_LowBrGrZones_m_BGrid[21]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	//awbb_LowBrGrZones_m_BGrid[22]      
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);	//awbb_LowBrGrZones_m_BGrid[23]      


S5K5EAYX_write_cmos_sensor(0x002A, 0x0E88);  //0x20000E88 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0380);  //                        awbb_CrclLowT_R_c 0896 0398 //039A
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0168);  //                        awbb_CrclLowT_B_c 0360 0154 //00FE
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2D90);  //                      awbb_CrclLowT_Rad_c 11664 3A24 //2284
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  
S5K5EAYX_write_cmos_sensor(0x002A, 0x11EC);  //0x200011EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02CE);  //                        awbb_GridConst_1_0_ 0718 /02F6//02EE//02CE//02F4
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0347);  //                        awbb_GridConst_1_1_ 0839 /0346//0347//0347//034F
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03C2);  //                        awbb_GridConst_1_2_ 0962 038E//038E//03C2//03DF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x102F);  //  0FCE  //              awbb_GridConst_2_0_ 4256 0FC3//0FC3//10A0//0F9C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1099);  //  1030  //              awbb_GridConst_2_1_ 4257 100A//100A//10A1//0FF2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1138);  //  109E  //              awbb_GridConst_2_2_ 4485 1083//1083//1185//10E7
S5K5EAYX_write_cmos_sensor(0x0F12, 0x11BE);  //  110B  //              awbb_GridConst_2_3_ 4486 10F6//10F6//1186//107A
S5K5EAYX_write_cmos_sensor(0x0F12, 0x11F8);  //  118B  //              awbb_GridConst_2_4_ 4581 114D//114D//11E5//114F
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1229);  //                        awbb_GridConst_2_5_ 4582 11B4//118B//11E6//11A1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x12CA);  //                        awbb_GridConst_2_6_ 4582 1217//11D3//11E6//1204
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00AB);  //20001200                       awbb_GridCoeff_R_1 0171 00AB//00AB//00AB//00B2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00BF);  //20001202                       awbb_GridCoeff_B_1 0191 00BF//00BF//00BF//00B8
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D2);  //20001204                       awbb_GridCoeff_R_2 0210 00D2//00D2//00D2//00CA
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0093);  //20001206                       awbb_GridCoeff_B_2 0147 0093//0093//0093//009D
S5K5EAYX_write_cmos_sensor(0x002A, 0x113C);  //0x2000113C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_0__0_ 0000 0000//0000//0000//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_0__1_ 0000 FFD8//0000//FFD8//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_0__2_ 0000 FFD8//0000//FFD8//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_0__3_ 0000 FFD8//0000//FFD8//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_0__4_ 0000 FFD8//0000//FFD8//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_0__5_ 0000 FFD0//0000//FFD0//FFA0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C8);  //                    awbb_GridCorr_R_0__6_ 0000 FFD0//0000//FFD0//FFA0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_1__0_ 0000 0000//0000//0000//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF9C);      //0x0000);      //0x0000);  //                    awbb_GridCorr_R_1__1_ 0000 FFD8//0000//FFD8//FFE0//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0032);      //0xFF6A);      //0xFF4C);  // FFEC                   awbb_GridCorr_R_1__2_ 0000 FFD8//0000//FFD8//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_1__1_ 0000 FFD8//0000//FFD8//FFE0//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0064);  //0000, 005A          awbb_GridCorr_R_1__3_ 0000 FFD8//0000//FFD8//FFE0//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C8);  //0000                awbb_GridCorr_R_1__5_ 0000 FFD0//0000//008C//FFD0//FFA0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C8);  //                    awbb_GridCorr_R_1__6_ 0000 FFD0//0000//FF9C//FFD0//FFA0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_2__0_ 0000 0000//0000//0000//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_2__1_ 0000 FFD8//0000//FFD8//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0032);      //0x0000);      //0x0000);  //                    awbb_GridCorr_R_2__2_ 0000 FFD8//0000//FFD8//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_2__3_ 0000 FFD8//0000//FFD8//FFE0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_R_2__4_ 0000 FFD8//0000//FFD8//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C8);  //0000                awbb_GridCorr_R_2__5_ 0000 FFD0//0000//FFD0//FFA0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C8);  //                    awbb_GridCorr_R_2__6_ 0000 FFD0//0000//FFD0//FFA0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_0__0_ 0000 FFEC//0000//FFEC//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_0__1_ 0000 000A//0000//000A//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_0__2_ 0000 000A//0000//000A//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_0__3_ 0000 FFC4//0000//FFC4//FF38
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_0__4_ 0000 FFC4//0000//FFC4//FEF2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_0__5_ 0000 FF66//0000//FF66//FEF2
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC18);  //                    awbb_GridCorr_B_0__6_ 0000 FF66//0000//FF66//FE5C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_1__0_ 0000 FFEC//0000//FFEC//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_1__1_ 0000 000A//0000//000A//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF6A);      //0x00C8);      //0xFFB0);  // 0000                   awbb_GridCorr_B_1__2_ 0000 000A//0000//000A//FFC0//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);      //0xFF06);      //0x0000);  //   FF48                 awbb_GridCorr_B_1__3_ 0000 FFC4//0000//FFC4//FF38//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF9C);      //0x00C8);      //0x0000);  //                    awbb_GridCorr_B_1__4_ 0000 FFC4//0000//FFC4//FEF2
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE3E);  //0000                awbb_GridCorr_B_1__5_ 0000 FF66//0000//FF38//FF66//FEF2
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC18);  //                    awbb_GridCorr_B_1__6_ 0000 FF66//0000//FF9C//FF66//FE5C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_2__0_ 0000 FFEC//0000//FFEC//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_2__1_ 0000 000A//0000//000A//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF6A);      //0x0000);      //0x0000);  //                    awbb_GridCorr_B_2__2_ 0000 000A//0000//000A//FFC0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_2__3_ 0000 FFC4//0000//FFC4//FF38
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                    awbb_GridCorr_B_2__4_ 0000 FFC4//0000//FFC4//FEF2
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE3E);  //0000                awbb_GridCorr_B_2__5_ 0000 FF66//0000//FF66//FEF2
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFC18);  //                    awbb_GridCorr_B_2__6_ 0000 0000//FF66//FE5C
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //20001190          awbb_GridCorr_R_Out_0__0_ 0000 0000//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //20001192          awbb_GridCorr_R_Out_0__1_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //20001194          awbb_GridCorr_R_Out_0__2_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //20001196          awbb_GridCorr_R_Out_0__3_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //20001198          awbb_GridCorr_R_Out_0__4_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //2000119A          awbb_GridCorr_R_Out_0__5_ 0000 FFD0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //2000119C          awbb_GridCorr_R_Out_0__6_ 65488 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //2000119E          awbb_GridCorr_R_Out_1__0_ 0000 0000//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011A0          awbb_GridCorr_R_Out_1__1_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011A2          awbb_GridCorr_R_Out_1__2_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011A4          awbb_GridCorr_R_Out_1__3_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011A6          awbb_GridCorr_R_Out_1__4_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011A8          awbb_GridCorr_R_Out_1__5_ 0000 FFD0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011AA          awbb_GridCorr_R_Out_1__6_ 65488 0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011AC          awbb_GridCorr_R_Out_2__0_ 0000 0000//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011AE          awbb_GridCorr_R_Out_2__1_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011B0          awbb_GridCorr_R_Out_2__2_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011B2          awbb_GridCorr_R_Out_2__3_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011B4          awbb_GridCorr_R_Out_2__4_ 0000 FFD8//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011B6          awbb_GridCorr_R_Out_2__5_ 0000 FFD0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011B8          awbb_GridCorr_R_Out_2__6_ 0000 FFD0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011BA          awbb_GridCorr_B_Out_0__0_ 0000 FFEC//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011BC          awbb_GridCorr_B_Out_0__1_ 0000 000A//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011BE          awbb_GridCorr_B_Out_0__2_ 0000 000A//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011C0          awbb_GridCorr_B_Out_0__3_ 0000 FFC4//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011C2          awbb_GridCorr_B_Out_0__4_ 0000 FFC4//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011C4          awbb_GridCorr_B_Out_0__5_ 0000 FF66//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011C6          awbb_GridCorr_B_Out_0__6_ 0000 FF66//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0064);  //      //200011C8          awbb_GridCorr_B_Out_1__0_ 0000 FFEC//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF9C);  //      //200011CA          awbb_GridCorr_B_Out_1__1_ 0000 000A//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011CC          awbb_GridCorr_B_Out_1__2_ 0000 000A//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011CE          awbb_GridCorr_B_Out_1__3_ 0000 FFC4//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011D0          awbb_GridCorr_B_Out_1__4_ 0000 FFC4//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011D2          awbb_GridCorr_B_Out_1__5_ 0000 FF66//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011D4          awbb_GridCorr_B_Out_1__6_ 0000 FF66//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0064);  //      //200011D6          awbb_GridCorr_B_Out_2__0_ 0000 FFEC//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011D8          awbb_GridCorr_B_Out_2__1_ 0000 000A//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011DA          awbb_GridCorr_B_Out_2__2_ 0000 000A//0000//FFC0//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011DC          awbb_GridCorr_B_Out_2__3_ 0000 FFC4//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011DE          awbb_GridCorr_B_Out_2__4_ 0000 FFC4//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011E0          awbb_GridCorr_B_Out_2__5_ 0000 FF66//0000
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //      //200011E2          awbb_GridCorr_B_Out_2__6_ 65382 0000
S5K5EAYX_write_cmos_sensor(0x002A, 0x146A);  //0x2000146A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B0);  //                       SARR_AwbCcmCord_0_ 0228 0050
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //                       SARR_AwbCcmCord_1_ 0240 00F0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0130);  //                       SARR_AwbCcmCord_2_ 0256 0110
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0143);  //                       SARR_AwbCcmCord_3_ 0288 0120
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0170);  //                       SARR_AwbCcmCord_4_ 0336 0130
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0190);  //                       SARR_AwbCcmCord_5_ 0384 0162
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                       wbt_bUseOutdoorCCM 0001 
S5K5EAYX_write_cmos_sensor(0x002A, 0x145C);  //0x2000145C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4800);  //                                      wbt 18432 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);  
S5K5EAYX_write_cmos_sensor(0x002A, 0x1464);  //0x20001464 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x48D8);  //                     TVAR_wbt_pOutdoorCcm 18648 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);  
S5K5EAYX_write_cmos_sensor(0x002A, 0x4800);  //0x20004800 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x022F);  //TVAR_wbt_pBaseCcms[0] // Horizon
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFB8);  //TVAR_wbt_pBaseCcms[1]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFCF);  //TVAR_wbt_pBaseCcms[2]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF11);  //TVAR_wbt_pBaseCcms[3]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E9);  //TVAR_wbt_pBaseCcms[4]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF47);  //TVAR_wbt_pBaseCcms[5]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0017);  //TVAR_wbt_pBaseCcms[6]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFE5);  //TVAR_wbt_pBaseCcms[7]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E2);  //TVAR_wbt_pBaseCcms[8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00CE);  //TVAR_wbt_pBaseCcms[9]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B2);  //TVAR_wbt_pBaseCcms[10]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEE9);  //TVAR_wbt_pBaseCcms[11]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0214);  //TVAR_wbt_pBaseCcms[12]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF61);  //TVAR_wbt_pBaseCcms[13]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01B2);  //TVAR_wbt_pBaseCcms[14]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEE5);  //TVAR_wbt_pBaseCcms[15]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01CA);  //TVAR_wbt_pBaseCcms[16]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x012B);  //TVAR_wbt_pBaseCcms[17]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0233);  //0289  0278TVAR_wbt_pBaseCcms[18] // IncandA
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFD1);  //FFC1  FFA3TVAR_wbt_pBaseCcms[19]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFB4);  //0022  0010TVAR_wbt_pBaseCcms[20]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEFF);  //FF11  FF11TVAR_wbt_pBaseCcms[21]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  //01E5  01E5TVAR_wbt_pBaseCcms[22]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF3B);  //FF49  FF49TVAR_wbt_pBaseCcms[23]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //0018  0018TVAR_wbt_pBaseCcms[24]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFE6);  //FFE5  FFE5TVAR_wbt_pBaseCcms[25]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01DF);  //01DF  01DFTVAR_wbt_pBaseCcms[26]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E2);  //00E6  00EATVAR_wbt_pBaseCcms[27]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C7);  //00CA  00CETVAR_wbt_pBaseCcms[28]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEFF);  //FF02  FF06TVAR_wbt_pBaseCcms[29]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0213);  //0213  0213TVAR_wbt_pBaseCcms[30]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF63);  //FF63  FF63TVAR_wbt_pBaseCcms[31]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01AF);  //01AF  01AFTVAR_wbt_pBaseCcms[32]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEE7);  //FEE7  FEE7TVAR_wbt_pBaseCcms[33]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //01C6  01C6TVAR_wbt_pBaseCcms[34]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x012B);  //012B  012BTVAR_wbt_pBaseCcms[35]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x022C);  //TVAR_wbt_pBaseCcms[36] // WW
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFBB);  //TVAR_wbt_pBaseCcms[37]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFCF);  //TVAR_wbt_pBaseCcms[38]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF12);  //TVAR_wbt_pBaseCcms[39]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E5);  //TVAR_wbt_pBaseCcms[40]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF4A);  //TVAR_wbt_pBaseCcms[41]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0019);  //TVAR_wbt_pBaseCcms[42]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFE5);  //TVAR_wbt_pBaseCcms[43]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01DF);  //TVAR_wbt_pBaseCcms[44]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00CC);  //TVAR_wbt_pBaseCcms[45]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B2);  //TVAR_wbt_pBaseCcms[46]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEEB);  //TVAR_wbt_pBaseCcms[47]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0213);  //TVAR_wbt_pBaseCcms[48]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF64);  //TVAR_wbt_pBaseCcms[49]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01AF);  //TVAR_wbt_pBaseCcms[50]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEE8);  //TVAR_wbt_pBaseCcms[51]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //TVAR_wbt_pBaseCcms[52]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x012B);  //TVAR_wbt_pBaseCcms[53]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x022C);  //TVAR_wbt_pBaseCcms[54] // CW
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFBB);  //TVAR_wbt_pBaseCcms[55]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFCF);  //TVAR_wbt_pBaseCcms[56]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF12);  //TVAR_wbt_pBaseCcms[57]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E5);  //TVAR_wbt_pBaseCcms[58]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF4A);  //TVAR_wbt_pBaseCcms[59]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //TVAR_wbt_pBaseCcms[60]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFE0);  //TVAR_wbt_pBaseCcms[61]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E7);  //TVAR_wbt_pBaseCcms[62]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00CF);  //TVAR_wbt_pBaseCcms[63]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00B5);  //TVAR_wbt_pBaseCcms[64]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEE5);  //TVAR_wbt_pBaseCcms[65]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0213);  //TVAR_wbt_pBaseCcms[66]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF64);  //TVAR_wbt_pBaseCcms[67]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01AF);  //TVAR_wbt_pBaseCcms[68]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEE8);  //TVAR_wbt_pBaseCcms[69]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //TVAR_wbt_pBaseCcms[70]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x012B);  //TVAR_wbt_pBaseCcms[71]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  //TVAR_wbt_pBaseCcms[72] // D50
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC3);  //TVAR_wbt_pBaseCcms[73]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFFE);  //TVAR_wbt_pBaseCcms[74]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF0D);  //TVAR_wbt_pBaseCcms[75]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E5);  //TVAR_wbt_pBaseCcms[76]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF4F);  //TVAR_wbt_pBaseCcms[77]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x001B);  //TVAR_wbt_pBaseCcms[78]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFDD);  //TVAR_wbt_pBaseCcms[79]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E6);  //TVAR_wbt_pBaseCcms[80]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D0);  //TVAR_wbt_pBaseCcms[81]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00BC);  //TVAR_wbt_pBaseCcms[82]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEDD);  //TVAR_wbt_pBaseCcms[83]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0218);  //TVAR_wbt_pBaseCcms[84]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF65);  //TVAR_wbt_pBaseCcms[85]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01AA);  //TVAR_wbt_pBaseCcms[86]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEE8);  //TVAR_wbt_pBaseCcms[87]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C2);  //TVAR_wbt_pBaseCcms[88]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0131);  //TVAR_wbt_pBaseCcms[89]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F8);  //TVAR_wbt_pBaseCcms[90] // D65
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFCF);  //TVAR_wbt_pBaseCcms[91]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFEB);  //TVAR_wbt_pBaseCcms[92]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEFA);  //TVAR_wbt_pBaseCcms[93]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01A4);  //TVAR_wbt_pBaseCcms[94]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF37);  //TVAR_wbt_pBaseCcms[95]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //TVAR_wbt_pBaseCcms[96]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFDB);  //TVAR_wbt_pBaseCcms[97]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01CF);  //TVAR_wbt_pBaseCcms[98]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00DC);  //TVAR_wbt_pBaseCcms[99]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00C5);  //TVAR_wbt_pBaseCcms[100]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF13);  //TVAR_wbt_pBaseCcms[101]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C1);  //TVAR_wbt_pBaseCcms[102]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF57);  //TVAR_wbt_pBaseCcms[103]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017C);  //TVAR_wbt_pBaseCcms[104]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF25);  //TVAR_wbt_pBaseCcms[105]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01A5);  //TVAR_wbt_pBaseCcms[106]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017C);  //TVAR_wbt_pBaseCcms[107]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01FD);  //TVAR_wbt_pOutdoorCcm[0]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFBB);  //TVAR_wbt_pOutdoorCcm[1]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFF2);  //TVAR_wbt_pOutdoorCcm[2]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFE90);  //TVAR_wbt_pOutdoorCcm[3]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x013F);  //TVAR_wbt_pOutdoorCcm[4]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF1B);  //TVAR_wbt_pOutdoorCcm[5]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFB9);  //TVAR_wbt_pOutdoorCcm[6]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC6);  //TVAR_wbt_pOutdoorCcm[7]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0245);  //TVAR_wbt_pOutdoorCcm[8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D6);  //TVAR_wbt_pOutdoorCcm[9]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00E0);  //TVAR_wbt_pOutdoorCcm[10]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF63);  //TVAR_wbt_pOutdoorCcm[11]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01CE);  //TVAR_wbt_pOutdoorCcm[12]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF83);  //TVAR_wbt_pOutdoorCcm[13]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0195);  //TVAR_wbt_pOutdoorCcm[14]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFEF3);  //TVAR_wbt_pOutdoorCcm[15]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0126);  //TVAR_wbt_pOutdoorCcm[16]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0162);  //TVAR_wbt_pOutdoorCcm[17]
S5K5EAYX_write_cmos_sensor(0x002A, 0x12F4);  //0x200012F4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                                     seti 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);  //           SARR_usGammaLutRGBIndoor_0__1_ 0010 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //           SARR_usGammaLutRGBIndoor_0__2_ 0022 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //           SARR_usGammaLutRGBIndoor_0__3_ 0048 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0066);  //           SARR_usGammaLutRGBIndoor_0__4_ 0102 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D5);  //           SARR_usGammaLutRGBIndoor_0__5_ 0213 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0138);  //           SARR_usGammaLutRGBIndoor_0__6_ 0312 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0163);  //           SARR_usGammaLutRGBIndoor_0__7_ 0355 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0189);  //           SARR_usGammaLutRGBIndoor_0__8_ 0393 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //           SARR_usGammaLutRGBIndoor_0__9_ 0454 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F8);  //          SARR_usGammaLutRGBIndoor_0__10_ 0504 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  //          SARR_usGammaLutRGBIndoor_0__11_ 0546 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x023D);  //          SARR_usGammaLutRGBIndoor_0__12_ 0573 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x026E);  //          SARR_usGammaLutRGBIndoor_0__13_ 0622 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02AA);  //          SARR_usGammaLutRGBIndoor_0__14_ 0668 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x030E);  //          SARR_usGammaLutRGBIndoor_0__15_ 0748 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x036A);  //          SARR_usGammaLutRGBIndoor_0__16_ 0813 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03B4);  //          SARR_usGammaLutRGBIndoor_0__17_ 0878 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E8);  //          SARR_usGammaLutRGBIndoor_0__18_ 0946 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03FF);  //          SARR_usGammaLutRGBIndoor_0__19_ 1023 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //           SARR_usGammaLutRGBIndoor_1__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);  //           SARR_usGammaLutRGBIndoor_1__1_ 0010 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //           SARR_usGammaLutRGBIndoor_1__2_ 0022 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //           SARR_usGammaLutRGBIndoor_1__3_ 0048 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0066);  //           SARR_usGammaLutRGBIndoor_1__4_ 0102 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D5);  //           SARR_usGammaLutRGBIndoor_1__5_ 0213 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0138);  //           SARR_usGammaLutRGBIndoor_1__6_ 0312 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0163);  //           SARR_usGammaLutRGBIndoor_1__7_ 0355 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0189);  //           SARR_usGammaLutRGBIndoor_1__8_ 0393 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //           SARR_usGammaLutRGBIndoor_1__9_ 0454 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F8);  //          SARR_usGammaLutRGBIndoor_1__10_ 0504 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  //          SARR_usGammaLutRGBIndoor_1__11_ 0546 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x023D);  //          SARR_usGammaLutRGBIndoor_1__12_ 0573 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x026E);  //          SARR_usGammaLutRGBIndoor_1__13_ 0622 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02AA);  //          SARR_usGammaLutRGBIndoor_1__14_ 0668 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x030E);  //          SARR_usGammaLutRGBIndoor_1__15_ 0748 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x036A);  //          SARR_usGammaLutRGBIndoor_1__16_ 0813 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03B4);  //          SARR_usGammaLutRGBIndoor_1__17_ 0878 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E8);  //          SARR_usGammaLutRGBIndoor_1__18_ 0946 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03FF);  //          SARR_usGammaLutRGBIndoor_1__19_ 1023 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //           SARR_usGammaLutRGBIndoor_2__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);  //           SARR_usGammaLutRGBIndoor_2__1_ 0010 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //           SARR_usGammaLutRGBIndoor_2__2_ 0022 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //           SARR_usGammaLutRGBIndoor_2__3_ 0048 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0066);  //           SARR_usGammaLutRGBIndoor_2__4_ 0102 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D5);  //           SARR_usGammaLutRGBIndoor_2__5_ 0213 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0138);  //           SARR_usGammaLutRGBIndoor_2__6_ 0312 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0163);  //           SARR_usGammaLutRGBIndoor_2__7_ 0355 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0189);  //           SARR_usGammaLutRGBIndoor_2__8_ 0393 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //           SARR_usGammaLutRGBIndoor_2__9_ 0454 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F8);  //          SARR_usGammaLutRGBIndoor_2__10_ 0504 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  //          SARR_usGammaLutRGBIndoor_2__11_ 0546 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x023D);  //          SARR_usGammaLutRGBIndoor_2__12_ 0573 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x026E);  //          SARR_usGammaLutRGBIndoor_2__13_ 0622 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02AA);  //          SARR_usGammaLutRGBIndoor_2__14_ 0668 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x030E);  //          SARR_usGammaLutRGBIndoor_2__15_ 0748 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x036A);  //          SARR_usGammaLutRGBIndoor_2__16_ 0813 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03B4);  //          SARR_usGammaLutRGBIndoor_2__17_ 0878 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E8);  //          SARR_usGammaLutRGBIndoor_2__18_ 0946 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03FF);  //          SARR_usGammaLutRGBIndoor_2__19_ 1023 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          SARR_usGammaLutRGBOutdoor_0__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);  //          SARR_usGammaLutRGBOutdoor_0__1_ 0010 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //          SARR_usGammaLutRGBOutdoor_0__2_ 0022 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //          SARR_usGammaLutRGBOutdoor_0__3_ 0048 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0066);  //          SARR_usGammaLutRGBOutdoor_0__4_ 0102 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D5);  //          SARR_usGammaLutRGBOutdoor_0__5_ 0213 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0138);  //          SARR_usGammaLutRGBOutdoor_0__6_ 0312 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0163);  //          SARR_usGammaLutRGBOutdoor_0__7_ 0355 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0189);  //          SARR_usGammaLutRGBOutdoor_0__8_ 0393 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //          SARR_usGammaLutRGBOutdoor_0__9_ 0454 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F8);  //         SARR_usGammaLutRGBOutdoor_0__10_ 0504 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  //         SARR_usGammaLutRGBOutdoor_0__11_ 0546 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x023D);  //         SARR_usGammaLutRGBOutdoor_0__12_ 0573 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x026E);  //         SARR_usGammaLutRGBOutdoor_0__13_ 0622 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02AA);  //         SARR_usGammaLutRGBOutdoor_0__14_ 0668 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x030E);  //         SARR_usGammaLutRGBOutdoor_0__15_ 0748 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x036A);  //         SARR_usGammaLutRGBOutdoor_0__16_ 0813 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03B4);  //         SARR_usGammaLutRGBOutdoor_0__17_ 0878 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03E8);  //         SARR_usGammaLutRGBOutdoor_0__18_ 0946 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03FF);  //         SARR_usGammaLutRGBOutdoor_0__19_ 1023 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          SARR_usGammaLutRGBOutdoor_1__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);  //          SARR_usGammaLutRGBOutdoor_1__1_ 0010 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //          SARR_usGammaLutRGBOutdoor_1__2_ 0022 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //          SARR_usGammaLutRGBOutdoor_1__3_ 0048 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0066);  //          SARR_usGammaLutRGBOutdoor_1__4_ 0102 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D5);  //          SARR_usGammaLutRGBOutdoor_1__5_ 0213 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0138);  //          SARR_usGammaLutRGBOutdoor_1__6_ 0312 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0163);  //          SARR_usGammaLutRGBOutdoor_1__7_ 0355 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0189);  //          SARR_usGammaLutRGBOutdoor_1__8_ 0393 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //          SARR_usGammaLutRGBOutdoor_1__9_ 0454 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F8);  //         SARR_usGammaLutRGBOutdoor_1__10_ 0504 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  //         SARR_usGammaLutRGBOutdoor_1__11_ 0546 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x023D);  //         SARR_usGammaLutRGBOutdoor_1__12_ 0573 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x026E);  //         SARR_usGammaLutRGBOutdoor_1__13_ 0622 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x029C);  //         SARR_usGammaLutRGBOutdoor_1__14_ 0668 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02EC);  //         SARR_usGammaLutRGBOutdoor_1__15_ 0748 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x032D);  //         SARR_usGammaLutRGBOutdoor_1__16_ 0813 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x036E);  //         SARR_usGammaLutRGBOutdoor_1__17_ 0878 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03B2);  //         SARR_usGammaLutRGBOutdoor_1__18_ 0946 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03FF);  //         SARR_usGammaLutRGBOutdoor_1__19_ 1023 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          SARR_usGammaLutRGBOutdoor_2__0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000A);  //          SARR_usGammaLutRGBOutdoor_2__1_ 0010 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0016);  //          SARR_usGammaLutRGBOutdoor_2__2_ 0022 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //          SARR_usGammaLutRGBOutdoor_2__3_ 0048 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0066);  //          SARR_usGammaLutRGBOutdoor_2__4_ 0102 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D5);  //          SARR_usGammaLutRGBOutdoor_2__5_ 0213 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0138);  //          SARR_usGammaLutRGBOutdoor_2__6_ 0312 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0163);  //          SARR_usGammaLutRGBOutdoor_2__7_ 0355 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0189);  //          SARR_usGammaLutRGBOutdoor_2__8_ 0393 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01C6);  //          SARR_usGammaLutRGBOutdoor_2__9_ 0454 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F8);  //         SARR_usGammaLutRGBOutdoor_2__10_ 0504 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0222);  //         SARR_usGammaLutRGBOutdoor_2__11_ 0546 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x023D);  //         SARR_usGammaLutRGBOutdoor_2__12_ 0573 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x026E);  //         SARR_usGammaLutRGBOutdoor_2__13_ 0622 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x029C);  //         SARR_usGammaLutRGBOutdoor_2__14_ 0668 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x02EC);  //         SARR_usGammaLutRGBOutdoor_2__15_ 0748 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x032D);  //         SARR_usGammaLutRGBOutdoor_2__16_ 0813 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x036E);  //         SARR_usGammaLutRGBOutdoor_2__17_ 0878 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03B2);  //         SARR_usGammaLutRGBOutdoor_2__18_ 0946 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03FF);  //         SARR_usGammaLutRGBOutdoor_2__19_ 1023 
S5K5EAYX_write_cmos_sensor(0x002A, 0x14FC);  //0x200014FC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003F);  //                  afit_uNoiseIndInDoor_0_ 0063 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0083);  //                  afit_uNoiseIndInDoor_1_ 0131 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x012F);  //                  afit_uNoiseIndInDoor_2_ 0303 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01F0);  //                  afit_uNoiseIndInDoor_3_ 0496 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0255);  //                  afit_uNoiseIndInDoor_4_ 0597 
S5K5EAYX_write_cmos_sensor(0x002A, 0x14F0);  //0x200014F0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                         afit_bUseNB_Afit 0000  on/off AFIT by NB option
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0014);  //                    SARR_uNormBrInDoor_0_ 0020 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00D2);  //                    SARR_uNormBrInDoor_1_ 0210 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0384);  //                    SARR_uNormBrInDoor_2_ 0900 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x07D0);  //                    SARR_uNormBrInDoor_3_ 2000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1388);  //                    SARR_uNormBrInDoor_4_ 5000 
S5K5EAYX_write_cmos_sensor(0x002A, 0x152E);  //0x2000152E 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0070);  //                           afit_usGamutTh 0112 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                    afit_usNeargrayOffset 0005 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0180);  //                 afit_NIContrastAFITValue 0384 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0196);  //                        afit_NIContrastTh 0406 
S5K5EAYX_write_cmos_sensor(0x002A, 0x1538);  //0x20001538 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_0__0_ 0000 0x20001538  AFIT16_BRIGHTNESS
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_0__1_ 0000 0x2000153A  AFIT16_CONTRAST
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_0__2_ 0000 0x2000153C  AFIT16_SATURATION
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_0__3_ 0000 0x2000153E  AFIT16_SHARP_BLUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_0__4_ 0000 0x20001540  AFIT16_GLAMOUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0064);  //                       AfitBaseVals_0__5_ 0100 0x20001542  AFIT16_EE_iFlatBoundary
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                       AfitBaseVals_0__6_ 0032 0x20001544  AFIT16_Yuvemix_mNegRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                       AfitBaseVals_0__7_ 0080 0x20001546  AFIT16_Yuvemix_mNegRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                       AfitBaseVals_0__8_ 0128 0x20001548  AFIT16_Yuvemix_mNegRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0008);  //                       AfitBaseVals_0__9_ 0008 0x2000154A  AFIT16_Yuvemix_mPosRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                      AfitBaseVals_0__10_ 0032 0x2000154C  AFIT16_Yuvemix_mPosRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                      AfitBaseVals_0__11_ 0080 0x2000154E  AFIT16_Yuvemix_mPosRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F18);  //                      AfitBaseVals_0__12_ 12056 0x20001550  AFIT8_Dspcl_edge_low [7:0] AFIT8_Dspcl_edge_high [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x001A);  //                      AfitBaseVals_0__13_ 0026 0x20001552  AFIT8_Dspcl_repl_thresh [7:0] AFIT8_Dspcl_iConnectedThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C24);  //                      AfitBaseVals_0__14_ 15396 0x20001554  AFIT8_Dspcl_iPlainLevel [7:0] AFIT8_Dspcl_iSatThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C01);  //                      AfitBaseVals_0__15_ 3073 0x20001556  AFIT8_Dspcl_iPlainReference_H [7:0] AFIT8_Dspcl_iVarianceMultThresh_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_0__16_ 4108 0x20001558  AFIT8_Dspcl_iVariancePlainMax_H [7:0] AFIT8_Dspcl_iVarianceLimitMax_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_0__17_ 0257 0x2000155A  AFIT8_Dspcl_nClustLevel_C [7:0] AFIT8_Dspcl_iPlainReference_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_0__18_ 3084 0x2000155C  AFIT8_Dspcl_iVarianceMultThresh_C [7:0] AFIT8_Dspcl_iVariancePlainMax_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3010);  //                      AfitBaseVals_0__19_ 12304 0x2000155E  AFIT8_Dspcl_iVarianceLimitMax_C [7:0] AFIT8_EE_iShVLowRegion [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_0__20_ 4108 0x20001560  AFIT8_EE_iFSmagPosPwrLow [7:0] AFIT8_EE_iFSmagPosPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_0__21_ 4108 0x20001562  AFIT8_EE_iFSmagNegPwrLow [7:0] AFIT8_EE_iFSmagNegPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_0__22_ 0000 0x20001564  AFIT8_EE_iFSThLow [7:0] AFIT8_EE_iFSThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C08);  //                      AfitBaseVals_0__23_ 3080 0x20001566  AFIT8_EE_iXformTh_High [7:0] AFIT8_EE_iXformTh_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_0__24_ 3084 0x20001568  AFIT8_EE_iVLowFSmagPower [7:0] AFIT8_EE_iVLowiXformTh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0610);  //                      AfitBaseVals_0__25_ 1552 0x2000156A  AFIT8_EE_iReduceNoiseRatio [7:0] AFIT8_EE_iFlatSpan [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1008);  //                      AfitBaseVals_0__26_ 4104 0x2000156C  AFIT8_EE_iMSharpenLow [7:0] AFIT8_EE_iMSharpenHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x105A);  //                      AfitBaseVals_0__27_ 4186 0x2000156E  AFIT8_EE_iFlatMean [7:0] AFIT8_EE_iFlatOffset [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_0__28_ 0000 0x20001570  AFIT8_EE_iMShThLow [7:0] AFIT8_EE_iMShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_0__29_ 1030 0x20001572  AFIT8_EE_iMShDirThLow [7:0] AFIT8_EE_iMShDirThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0608);  //                      AfitBaseVals_0__30_ 1544 0x20001574  AFIT8_EE_iMShVLowPwr [7:0] AFIT8_EE_iMShVLowThrld [15:8]
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_0__31_ 6156 0x20001576  AFIT8_EE_iWSharpenPosLow [7:0] AFIT8_EE_iWSharpenPosHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_0__32_ 6156 0x20001578  AFIT8_EE_iWSharpenNegLow [7:0] AFIT8_EE_iWSharpenNegHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  //                      AfitBaseVals_0__33_ 0516 0x2000157A  AFIT8_EE_iWShThLow [7:0] AFIT8_EE_iWShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x040C);  //                      AfitBaseVals_0__34_ 1036 0x2000157C  AFIT8_EE_iWShVLowPwr [7:0] AFIT8_EE_iWShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A00);  //                      AfitBaseVals_0__35_ 23040 0x2000157E  AFIT8_EE_iReduceNegative [7:0] AFIT8_EE_iRadialLimitSh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4003);  //                      AfitBaseVals_0__36_ 16387 0x20001580  AFIT8_EE_iRadialPowerSh [7:0] AFIT8_Bdns_iDispTH_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0440);  //                      AfitBaseVals_0__37_ 1088 0x20001582  AFIT8_Bdns_iDispTH_H [7:0] AFIT8_Bdns_iDispLimit_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A04);  //                      AfitBaseVals_0__38_ 2564 0x20001584  AFIT8_Bdns_iDispLimit_H [7:0] AFIT8_Bdns_iDispTH4HF [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_0__39_ 0257 0x20001586  AFIT8_Bdns_iDispLimit4HF_L [7:0] AFIT8_Bdns_iDispLimit4HF_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7B7F);  //                      AfitBaseVals_0__40_ 10300 0x20001588  AFIT8_Bdns_iDenoiseTH_G_L [7:0] AFIT8_Bdns_iDenoiseTH_G_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7F7F);  //                      AfitBaseVals_0__41_ 10300 0x2000158A  AFIT8_Bdns_iDenoiseTH_NG_L [7:0] AFIT8_Bdns_iDenoiseTH_NG_H [15:8]
//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0601);  //                      AfitBaseVals_0__42_ 1537 0x2000158C  AFIT8_Bdns_iDistSigmaMin [7:0] AFIT8_Bdns_iDistSigmaMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C3C);  //                      AfitBaseVals_0__43_ 15420 0x2000158E  AFIT8_Bdns_iDenoiseTH_Add_Plain [7:0] AFIT8_Bdns_iDenoiseTH_Add_Direc [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5004);  //                      AfitBaseVals_0__44_ 20484 0x20001590  AFIT8_Bdns_iDirConfidenceMin [7:0] AFIT8_Bdns_iDirConfidenceMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7808);  //                      AfitBaseVals_0__45_ 30728 0x20001592  AFIT8_Bdns_iPatternTH_MIN [7:0] AFIT8_Bdns_iPatternTH_MAX [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C00);  //                      AfitBaseVals_0__46_ 15360 0x20001594  AFIT8_Bdns_iNRTune [7:0] AFIT8_Bdns_iLowMaxSlopeAllowed [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A3C);  //                      AfitBaseVals_0__47_ 23100 0x20001596  AFIT8_Bdns_iHighMaxSlopeAllowed [7:0] AFIT8_Bdns_iRadialLimitNR [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C04);  //                      AfitBaseVals_0__48_ 3076 0x20001598  AFIT8_Bdns_iRadialPowerNR [7:0] AFIT8_Dmsc_iEnhThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F08);  //                      AfitBaseVals_0__49_ 3848 0x2000159A  AFIT8_Dmsc_iDesatThresh [7:0] AFIT8_Dmsc_iDemBlurLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x050F);  //                      AfitBaseVals_0__50_ 1295 0x2000159C  AFIT8_Dmsc_iDemBlurHigh [7:0] AFIT8_Dmsc_iDemBlurRange [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8006);  //                      AfitBaseVals_0__51_ 32774 0x2000159E  AFIT8_Dmsc_iDecisionThresh [7:0] AFIT8_Dmsc_iCentGrad [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                      AfitBaseVals_0__52_ 0032 0x200015A0  AFIT8_Dmsc_iMonochrom [7:0] AFIT8_Dmsc_iGRDenoiseVal [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_0__53_ 0000 0x200015A2  AFIT8_Dmsc_iGBDenoiseVal [7:0] AFIT8_Dmsc_iEdgeDesatThrLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1800);  //                      AfitBaseVals_0__54_ 6144 0x200015A4  AFIT8_Dmsc_iEdgeDesatThrHigh [7:0] AFIT8_Dmsc_iEdgeDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  //                      AfitBaseVals_0__55_ 0000 0x200015A6  AFIT8_Dmsc_iEdgeDesatLimit [7:0] AFIT8_Dmsc_iNearGrayDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE119);  //                      AfitBaseVals_0__56_ 57625 0x200015A8  AFIT8_Postdmsc_iLowBright [7:0] AFIT8_Postdmsc_iHighBright [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7D0D);  //                      AfitBaseVals_0__57_ 32013 0x200015AA  AFIT8_Postdmsc_iLowSat [7:0] AFIT8_Postdmsc_iHighSat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E10);  //                      AfitBaseVals_0__58_ 7696 0x200015AC  AFIT8_Postdmsc_iBCoeff [7:0] AFIT8_Postdmsc_iGCoeff [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C09);  //                      AfitBaseVals_0__59_ 7177 0x200015AE  AFIT8_Postdmsc_iWideMult [7:0] AFIT8_Postdmsc_iTune [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //                      AfitBaseVals_0__60_ 2560 0x200015B0  AFIT8_Postdmsc_NoisePower_Low [7:0] AFIT8_Postdmsc_NoisePower_High [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //                      AfitBaseVals_0__61_ 2560 0x200015B2  AFIT8_Postdmsc_NoisePower_VLow [7:0] AFIT8_Postdmsc_NoiseLimit_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A0A);  //                      AfitBaseVals_0__62_ 2570 0x200015B4  AFIT8_Postdmsc_NoiseLimit_High [7:0] AFIT8_Postdmsc_iSkinNS [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0308);  //                      AfitBaseVals_0__63_ 0776 0x200015B6  AFIT8_Postdmsc_iReduceNS_EdgeTh [7:0] AFIT8_Postdmsc_iReduceNS_Slope [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_0__64_ 1792 0x200015B8  AFIT8_Yuvemix_mNegSlopes_0 [7:0] AFIT8_Yuvemix_mNegSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_0__65_ 1286 0x200015BA  AFIT8_Yuvemix_mNegSlopes_2 [7:0] AFIT8_Yuvemix_mNegSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_0__66_ 1792 0x200015BC  AFIT8_Yuvemix_mPosSlopes_0 [7:0] AFIT8_Yuvemix_mPosSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_0__67_ 1286 0x200015BE  AFIT8_Yuvemix_mPosSlopes_2 [7:0] AFIT8_Yuvemix_mPosSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1414);  //                      AfitBaseVals_0__68_ 5140 0x200015C0  AFIT8_Yuviirnr_iYThreshL [7:0] AFIT8_Yuviirnr_iYThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1212);  //                      AfitBaseVals_0__69_ 4626 0x200015C2  AFIT8_Yuviirnr_iYNRStrengthL [7:0] AFIT8_Yuviirnr_iYNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x080C);  //                      AfitBaseVals_0__70_ 2060 0x200015C4  AFIT8_Yuviirnr_iUVThreshL [7:0] AFIT8_Yuviirnr_iUVThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C10);  //                      AfitBaseVals_0__71_ 3088 0x200015C6  AFIT8_Yuviirnr_iDiffThreshL_UV [7:0] AFIT8_Yuviirnr_iDiffThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_0__72_ 1030 0x200015C8  AFIT8_Yuviirnr_iMaxThreshL_UV [7:0] AFIT8_Yuviirnr_iMaxThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1F1F);  //                      AfitBaseVals_0__73_ 7967 0x200015CA  AFIT8_Yuviirnr_iUVNRStrengthL [7:0] AFIT8_Yuviirnr_iUVNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8080);  //                      AfitBaseVals_0__74_ 32896 0x200015CC  AFIT8_byr_gras_iShadingPower [7:0] AFIT8_RGBGamma2_iLinearity [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_0__75_ 32768 0x200015CE  AFIT8_RGBGamma2_iDarkReduce [7:0] AFIT8_ccm_oscar_iSaturation [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_0__76_ 32768 0x200015D0  AFIT8_RGB2YUV_iYOffset [7:0] AFIT8_RGB2YUV_iRGBGain [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2001);  //                      AfitBaseVals_0__77_ 8193 0x200015D2  AFIT8_Dspcl_nClustLevel_H [7:0] AFIT8_EE_iLowSharpPower [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0140);  //                      AfitBaseVals_0__78_ 0320 0x200015D4  AFIT8_EE_iHighSharpPower [7:0] AFIT8_Dspcl_nClustLevel_H_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4020);  //                      AfitBaseVals_0__79_ 16416 0x200015D6  AFIT8_EE_iLowSharpPower_Bin [7:0] AFIT8_EE_iHighSharpPower_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_1__0_ 0000 0x200015D8  AFIT16_BRIGHTNESS
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_1__1_ 0000 0x200015DA  AFIT16_CONTRAST
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_1__2_ 0000 0x200015DC  AFIT16_SATURATION
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_1__3_ 0000 0x200015DE  AFIT16_SHARP_BLUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_1__4_ 0000 0x200015E0  AFIT16_GLAMOUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                       AfitBaseVals_1__5_ 0080 0x200015E2  AFIT16_EE_iFlatBoundary
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                       AfitBaseVals_1__6_ 0032 0x200015E4  AFIT16_Yuvemix_mNegRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                       AfitBaseVals_1__7_ 0080 0x200015E6  AFIT16_Yuvemix_mNegRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                       AfitBaseVals_1__8_ 0128 0x200015E8  AFIT16_Yuvemix_mNegRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0008);  //                       AfitBaseVals_1__9_ 0008 0x200015EA  AFIT16_Yuvemix_mPosRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                      AfitBaseVals_1__10_ 0032 0x200015EC  AFIT16_Yuvemix_mPosRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                      AfitBaseVals_1__11_ 0080 0x200015EE  AFIT16_Yuvemix_mPosRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F18);  //                      AfitBaseVals_1__12_ 12056 0x200015F0  AFIT8_Dspcl_edge_low [7:0] AFIT8_Dspcl_edge_high [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x001A);  //                      AfitBaseVals_1__13_ 0026 0x200015F2  AFIT8_Dspcl_repl_thresh [7:0] AFIT8_Dspcl_iConnectedThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C24);  //                      AfitBaseVals_1__14_ 15396 0x200015F4  AFIT8_Dspcl_iPlainLevel [7:0] AFIT8_Dspcl_iSatThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C01);  //                      AfitBaseVals_1__15_ 3073 0x200015F6  AFIT8_Dspcl_iPlainReference_H [7:0] AFIT8_Dspcl_iVarianceMultThresh_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_1__16_ 4108 0x200015F8  AFIT8_Dspcl_iVariancePlainMax_H [7:0] AFIT8_Dspcl_iVarianceLimitMax_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_1__17_ 0257 0x200015FA  AFIT8_Dspcl_nClustLevel_C [7:0] AFIT8_Dspcl_iPlainReference_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_1__18_ 3084 0x200015FC  AFIT8_Dspcl_iVarianceMultThresh_C [7:0] AFIT8_Dspcl_iVariancePlainMax_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3010);  //                      AfitBaseVals_1__19_ 12304 0x200015FE  AFIT8_Dspcl_iVarianceLimitMax_C [7:0] AFIT8_EE_iShVLowRegion [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_1__20_ 4108 0x20001600  AFIT8_EE_iFSmagPosPwrLow [7:0] AFIT8_EE_iFSmagPosPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1410);  //                      AfitBaseVals_1__21_ 5136 0x20001602  AFIT8_EE_iFSmagNegPwrLow [7:0] AFIT8_EE_iFSmagNegPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_1__22_ 0000 0x20001604  AFIT8_EE_iFSThLow [7:0] AFIT8_EE_iFSThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A04);  //                      AfitBaseVals_1__23_ 2564 0x20001606  AFIT8_EE_iXformTh_High [7:0] AFIT8_EE_iXformTh_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A0C);  //                      AfitBaseVals_1__24_ 2572 0x20001608  AFIT8_EE_iVLowFSmagPower [7:0] AFIT8_EE_iVLowiXformTh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0510);  //                      AfitBaseVals_1__25_ 1296 0x2000160A  AFIT8_EE_iReduceNoiseRatio [7:0] AFIT8_EE_iFlatSpan [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1408);  //                      AfitBaseVals_1__26_ 5128 0x2000160C  AFIT8_EE_iMSharpenLow [7:0] AFIT8_EE_iMSharpenHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x183C);  //                      AfitBaseVals_1__27_ 6204 0x2000160E  AFIT8_EE_iFlatMean [7:0] AFIT8_EE_iFlatOffset [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_1__28_ 0000 0x20001610  AFIT8_EE_iMShThLow [7:0] AFIT8_EE_iMShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_1__29_ 1030 0x20001612  AFIT8_EE_iMShDirThLow [7:0] AFIT8_EE_iMShDirThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0608);  //                      AfitBaseVals_1__30_ 1544 0x20001614  AFIT8_EE_iMShVLowPwr [7:0] AFIT8_EE_iMShVLowThrld [15:8]
//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_1__31_ 5132 0x20001616  AFIT8_EE_iWSharpenPosLow [7:0] AFIT8_EE_iWSharpenPosHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_1__32_ 6156 0x20001618  AFIT8_EE_iWSharpenNegLow [7:0] AFIT8_EE_iWSharpenNegHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  //                      AfitBaseVals_1__33_ 0516 0x2000161A  AFIT8_EE_iWShThLow [7:0] AFIT8_EE_iWShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x040C);  //                      AfitBaseVals_1__34_ 1036 0x2000161C  AFIT8_EE_iWShVLowPwr [7:0] AFIT8_EE_iWShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A00);  //                      AfitBaseVals_1__35_ 23040 0x2000161E  AFIT8_EE_iReduceNegative [7:0] AFIT8_EE_iRadialLimitSh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4003);  //                      AfitBaseVals_1__36_ 16387 0x20001620  AFIT8_EE_iRadialPowerSh [7:0] AFIT8_Bdns_iDispTH_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0440);  //                      AfitBaseVals_1__37_ 1088 0x20001622  AFIT8_Bdns_iDispTH_H [7:0] AFIT8_Bdns_iDispLimit_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A04);  //                      AfitBaseVals_1__38_ 2564 0x20001624  AFIT8_Bdns_iDispLimit_H [7:0] AFIT8_Bdns_iDispTH4HF [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_1__39_ 0257 0x20001626  AFIT8_Bdns_iDispLimit4HF_L [7:0] AFIT8_Bdns_iDispLimit4HF_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6265);  //                      AfitBaseVals_1__40_ 7720 0x20001628  AFIT8_Bdns_iDenoiseTH_G_L [7:0] AFIT8_Bdns_iDenoiseTH_G_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x666C);  //                      AfitBaseVals_1__41_ 7720 0x2000162A  AFIT8_Bdns_iDenoiseTH_NG_L [7:0] AFIT8_Bdns_iDenoiseTH_NG_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  //                      AfitBaseVals_1__42_ 1280 0x2000162C  AFIT8_Bdns_iDistSigmaMin [7:0] AFIT8_Bdns_iDistSigmaMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C3C);  //                      AfitBaseVals_1__43_ 15420 0x2000162E  AFIT8_Bdns_iDenoiseTH_Add_Plain [7:0] AFIT8_Bdns_iDenoiseTH_Add_Direc [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3204);  //                      AfitBaseVals_1__44_ 12804 0x20001630  AFIT8_Bdns_iDirConfidenceMin [7:0] AFIT8_Bdns_iDirConfidenceMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5008);  //                      AfitBaseVals_1__45_ 20488 0x20001632  AFIT8_Bdns_iPatternTH_MIN [7:0] AFIT8_Bdns_iPatternTH_MAX [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);  //                      AfitBaseVals_1__46_ 8192 0x20001634  AFIT8_Bdns_iNRTune [7:0] AFIT8_Bdns_iLowMaxSlopeAllowed [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A20);  //                      AfitBaseVals_1__47_ 23072 0x20001636  AFIT8_Bdns_iHighMaxSlopeAllowed [7:0] AFIT8_Bdns_iRadialLimitNR [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C04);  //                      AfitBaseVals_1__48_ 3076 0x20001638  AFIT8_Bdns_iRadialPowerNR [7:0] AFIT8_Dmsc_iEnhThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F08);  //                      AfitBaseVals_1__49_ 3848 0x2000163A  AFIT8_Dmsc_iDesatThresh [7:0] AFIT8_Dmsc_iDemBlurLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x050F);  //                      AfitBaseVals_1__50_ 1295 0x2000163C  AFIT8_Dmsc_iDemBlurHigh [7:0] AFIT8_Dmsc_iDemBlurRange [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8006);  //                      AfitBaseVals_1__51_ 32774 0x2000163E  AFIT8_Dmsc_iDecisionThresh [7:0] AFIT8_Dmsc_iCentGrad [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                      AfitBaseVals_1__52_ 0032 0x20001640  AFIT8_Dmsc_iMonochrom [7:0] AFIT8_Dmsc_iGRDenoiseVal [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_1__53_ 0000 0x20001642  AFIT8_Dmsc_iGBDenoiseVal [7:0] AFIT8_Dmsc_iEdgeDesatThrLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1800);  //                      AfitBaseVals_1__54_ 6144 0x20001644  AFIT8_Dmsc_iEdgeDesatThrHigh [7:0] AFIT8_Dmsc_iEdgeDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);  //                      AfitBaseVals_1__55_ 0000 0x20001646  AFIT8_Dmsc_iEdgeDesatLimit [7:0] AFIT8_Dmsc_iNearGrayDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE119);  //                      AfitBaseVals_1__56_ 57625 0x20001648  AFIT8_Postdmsc_iLowBright [7:0] AFIT8_Postdmsc_iHighBright [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7D0D);  //                      AfitBaseVals_1__57_ 32013 0x2000164A  AFIT8_Postdmsc_iLowSat [7:0] AFIT8_Postdmsc_iHighSat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E10);  //                      AfitBaseVals_1__58_ 7696 0x2000164C  AFIT8_Postdmsc_iBCoeff [7:0] AFIT8_Postdmsc_iGCoeff [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C0B);  //                      AfitBaseVals_1__59_ 7179 0x2000164E  AFIT8_Postdmsc_iWideMult [7:0] AFIT8_Postdmsc_iTune [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A02);  //                      AfitBaseVals_1__60_ 2562 0x20001650  AFIT8_Postdmsc_NoisePower_Low [7:0] AFIT8_Postdmsc_NoisePower_High [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C04);  //                      AfitBaseVals_1__61_ 3076 0x20001652  AFIT8_Postdmsc_NoisePower_VLow [7:0] AFIT8_Postdmsc_NoiseLimit_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A0C);  //                      AfitBaseVals_1__62_ 2572 0x20001654  AFIT8_Postdmsc_NoiseLimit_High [7:0] AFIT8_Postdmsc_iSkinNS [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0308);  //                      AfitBaseVals_1__63_ 0776 0x20001656  AFIT8_Postdmsc_iReduceNS_EdgeTh [7:0] AFIT8_Postdmsc_iReduceNS_Slope [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_1__64_ 1792 0x20001658  AFIT8_Yuvemix_mNegSlopes_0 [7:0] AFIT8_Yuvemix_mNegSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_1__65_ 1286 0x2000165A  AFIT8_Yuvemix_mNegSlopes_2 [7:0] AFIT8_Yuvemix_mNegSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_1__66_ 1792 0x2000165C  AFIT8_Yuvemix_mPosSlopes_0 [7:0] AFIT8_Yuvemix_mPosSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_1__67_ 1286 0x2000165E  AFIT8_Yuvemix_mPosSlopes_2 [7:0] AFIT8_Yuvemix_mPosSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1414);  //                      AfitBaseVals_1__68_ 5140 0x20001660  AFIT8_Yuviirnr_iYThreshL [7:0] AFIT8_Yuviirnr_iYThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1212);  //                      AfitBaseVals_1__69_ 4626 0x20001662  AFIT8_Yuviirnr_iYNRStrengthL [7:0] AFIT8_Yuviirnr_iYNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x080C);  //                      AfitBaseVals_1__70_ 2060 0x20001664  AFIT8_Yuviirnr_iUVThreshL [7:0] AFIT8_Yuviirnr_iUVThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C10);  //                      AfitBaseVals_1__71_ 3088 0x20001666  AFIT8_Yuviirnr_iDiffThreshL_UV [7:0] AFIT8_Yuviirnr_iDiffThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_1__72_ 1030 0x20001668  AFIT8_Yuviirnr_iMaxThreshL_UV [7:0] AFIT8_Yuviirnr_iMaxThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1F1F);  //                      AfitBaseVals_1__73_ 7967 0x2000166A  AFIT8_Yuviirnr_iUVNRStrengthL [7:0] AFIT8_Yuviirnr_iUVNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8080);  //                      AfitBaseVals_1__74_ 32896 0x2000166C  AFIT8_byr_gras_iShadingPower [7:0] AFIT8_RGBGamma2_iLinearity [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_1__75_ 32768 0x2000166E  AFIT8_RGBGamma2_iDarkReduce [7:0] AFIT8_ccm_oscar_iSaturation [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_1__76_ 32768 0x20001670  AFIT8_RGB2YUV_iYOffset [7:0] AFIT8_RGB2YUV_iRGBGain [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3001);  //                      AfitBaseVals_1__77_ 12289 0x20001672  AFIT8_Dspcl_nClustLevel_H [7:0] AFIT8_EE_iLowSharpPower [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0140);  //                      AfitBaseVals_1__78_ 0320 0x20001674  AFIT8_EE_iHighSharpPower [7:0] AFIT8_Dspcl_nClustLevel_H_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4030);  //                      AfitBaseVals_1__79_ 16432 0x20001676  AFIT8_EE_iLowSharpPower_Bin [7:0] AFIT8_EE_iHighSharpPower_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_2__0_ 0000 0x20001678  AFIT16_BRIGHTNESS
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_2__1_ 0000 0x2000167A  AFIT16_CONTRAST
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_2__2_ 0000 0x2000167C  AFIT16_SATURATION
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_2__3_ 0000 0x2000167E  AFIT16_SHARP_BLUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_2__4_ 0000 0x20001680  AFIT16_GLAMOUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                       AfitBaseVals_2__5_ 0080 0x20001682  AFIT16_EE_iFlatBoundary
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                       AfitBaseVals_2__6_ 0048 0x20001684  AFIT16_Yuvemix_mNegRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0060);  //                       AfitBaseVals_2__7_ 0096 0x20001686  AFIT16_Yuvemix_mNegRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                       AfitBaseVals_2__8_ 0128 0x20001688  AFIT16_Yuvemix_mNegRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000C);  //                       AfitBaseVals_2__9_ 0012 0x2000168A  AFIT16_Yuvemix_mPosRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                      AfitBaseVals_2__10_ 0048 0x2000168C  AFIT16_Yuvemix_mPosRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                      AfitBaseVals_2__11_ 0128 0x2000168E  AFIT16_Yuvemix_mPosRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F18);  //                      AfitBaseVals_2__12_ 12056 0x20001690  AFIT8_Dspcl_edge_low [7:0] AFIT8_Dspcl_edge_high [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0420);  //                      AfitBaseVals_2__13_ 1056 0x20001692  AFIT8_Dspcl_repl_thresh [7:0] AFIT8_Dspcl_iConnectedThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C24);  //                      AfitBaseVals_2__14_ 15396 0x20001694  AFIT8_Dspcl_iPlainLevel [7:0] AFIT8_Dspcl_iSatThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C01);  //                      AfitBaseVals_2__15_ 3073 0x20001696  AFIT8_Dspcl_iPlainReference_H [7:0] AFIT8_Dspcl_iVarianceMultThresh_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_2__16_ 4108 0x20001698  AFIT8_Dspcl_iVariancePlainMax_H [7:0] AFIT8_Dspcl_iVarianceLimitMax_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_2__17_ 0257 0x2000169A  AFIT8_Dspcl_nClustLevel_C [7:0] AFIT8_Dspcl_iPlainReference_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_2__18_ 3084 0x2000169C  AFIT8_Dspcl_iVarianceMultThresh_C [7:0] AFIT8_Dspcl_iVariancePlainMax_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3010);  //                      AfitBaseVals_2__19_ 12304 0x2000169E  AFIT8_Dspcl_iVarianceLimitMax_C [7:0] AFIT8_EE_iShVLowRegion [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_2__20_ 4108 0x200016A0  AFIT8_EE_iFSmagPosPwrLow [7:0] AFIT8_EE_iFSmagPosPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1410);  //                      AfitBaseVals_2__21_ 5136 0x200016A2  AFIT8_EE_iFSmagNegPwrLow [7:0] AFIT8_EE_iFSmagNegPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_2__22_ 0000 0x200016A4  AFIT8_EE_iFSThLow [7:0] AFIT8_EE_iFSThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0602);  //                      AfitBaseVals_2__23_ 1538 0x200016A6  AFIT8_EE_iXformTh_High [7:0] AFIT8_EE_iXformTh_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x060C);  //                      AfitBaseVals_2__24_ 1548 0x200016A8  AFIT8_EE_iVLowFSmagPower [7:0] AFIT8_EE_iVLowiXformTh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0510);  //                      AfitBaseVals_2__25_ 1296 0x200016AA  AFIT8_EE_iReduceNoiseRatio [7:0] AFIT8_EE_iFlatSpan [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1008);  //                      AfitBaseVals_2__26_ 4104 0x200016AC  AFIT8_EE_iMSharpenLow [7:0] AFIT8_EE_iMSharpenHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_2__27_ 0257 0x200016AE  AFIT8_EE_iFlatMean [7:0] AFIT8_EE_iFlatOffset [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_2__28_ 0000 0x200016B0  AFIT8_EE_iMShThLow [7:0] AFIT8_EE_iMShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0408);  //                      AfitBaseVals_2__29_ 1032 0x200016B2  AFIT8_EE_iMShDirThLow [7:0] AFIT8_EE_iMShDirThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0808);  //                      AfitBaseVals_2__30_ 2056 0x200016B4  AFIT8_EE_iMShVLowPwr [7:0] AFIT8_EE_iMShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_2__31_ 4108 0x200016B6  AFIT8_EE_iWSharpenPosLow [7:0] AFIT8_EE_iWSharpenPosHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x140C);  //                      AfitBaseVals_2__32_ 5132 0x200016B8  AFIT8_EE_iWSharpenNegLow [7:0] AFIT8_EE_iWSharpenNegHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  //                      AfitBaseVals_2__33_ 0516 0x200016BA  AFIT8_EE_iWShThLow [7:0] AFIT8_EE_iWShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x040C);  //                      AfitBaseVals_2__34_ 1036 0x200016BC  AFIT8_EE_iWShVLowPwr [7:0] AFIT8_EE_iWShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A00);  //                      AfitBaseVals_2__35_ 23040 0x200016BE  AFIT8_EE_iReduceNegative [7:0] AFIT8_EE_iRadialLimitSh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4002);  //                      AfitBaseVals_2__36_ 16386 0x200016C0  AFIT8_EE_iRadialPowerSh [7:0] AFIT8_Bdns_iDispTH_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0440);  //                      AfitBaseVals_2__37_ 1088 0x200016C2  AFIT8_Bdns_iDispTH_H [7:0] AFIT8_Bdns_iDispLimit_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A04);  //                      AfitBaseVals_2__38_ 2564 0x200016C4  AFIT8_Bdns_iDispLimit_H [7:0] AFIT8_Bdns_iDispTH4HF [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_2__39_ 0257 0x200016C6  AFIT8_Bdns_iDispLimit4HF_L [7:0] AFIT8_Bdns_iDispLimit4HF_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1824);  //                      AfitBaseVals_2__40_ 6180 0x200016C8  AFIT8_Bdns_iDenoiseTH_G_L [7:0] AFIT8_Bdns_iDenoiseTH_G_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1824);  //                      AfitBaseVals_2__41_ 6180 0x200016CA  AFIT8_Bdns_iDenoiseTH_NG_L [7:0] AFIT8_Bdns_iDenoiseTH_NG_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  //                      AfitBaseVals_2__42_ 1280 0x200016CC  AFIT8_Bdns_iDistSigmaMin [7:0] AFIT8_Bdns_iDistSigmaMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C3C);  //                      AfitBaseVals_2__43_ 15420 0x200016CE  AFIT8_Bdns_iDenoiseTH_Add_Plain [7:0] AFIT8_Bdns_iDenoiseTH_Add_Direc [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E04);  //                      AfitBaseVals_2__44_ 7684 0x200016D0  AFIT8_Bdns_iDirConfidenceMin [7:0] AFIT8_Bdns_iDirConfidenceMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2808);  //                      AfitBaseVals_2__45_ 10248 0x200016D2  AFIT8_Bdns_iPatternTH_MIN [7:0] AFIT8_Bdns_iPatternTH_MAX [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);  //                      AfitBaseVals_2__46_ 8192 0x200016D4  AFIT8_Bdns_iNRTune [7:0] AFIT8_Bdns_iLowMaxSlopeAllowed [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A20);  //                      AfitBaseVals_2__47_ 23072 0x200016D6  AFIT8_Bdns_iHighMaxSlopeAllowed [7:0] AFIT8_Bdns_iRadialLimitNR [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C02);  //                      AfitBaseVals_2__48_ 3074 0x200016D8  AFIT8_Bdns_iRadialPowerNR [7:0] AFIT8_Dmsc_iEnhThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F08);  //                      AfitBaseVals_2__49_ 3848 0x200016DA  AFIT8_Dmsc_iDesatThresh [7:0] AFIT8_Dmsc_iDemBlurLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x050F);  //                      AfitBaseVals_2__50_ 1295 0x200016DC  AFIT8_Dmsc_iDemBlurHigh [7:0] AFIT8_Dmsc_iDemBlurRange [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8006);  //                      AfitBaseVals_2__51_ 32774 0x200016DE  AFIT8_Dmsc_iDecisionThresh [7:0] AFIT8_Dmsc_iCentGrad [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                      AfitBaseVals_2__52_ 0032 0x200016E0  AFIT8_Dmsc_iMonochrom [7:0] AFIT8_Dmsc_iGRDenoiseVal [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_2__53_ 0000 0x200016E2  AFIT8_Dmsc_iGBDenoiseVal [7:0] AFIT8_Dmsc_iEdgeDesatThrLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1800);  //                      AfitBaseVals_2__54_ 6144 0x200016E4  AFIT8_Dmsc_iEdgeDesatThrHigh [7:0] AFIT8_Dmsc_iEdgeDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);  //                      AfitBaseVals_2__55_ 0000 0x200016E6  AFIT8_Dmsc_iEdgeDesatLimit [7:0] AFIT8_Dmsc_iNearGrayDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE119);  //                      AfitBaseVals_2__56_ 57625 0x200016E8  AFIT8_Postdmsc_iLowBright [7:0] AFIT8_Postdmsc_iHighBright [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7D0D);  //                      AfitBaseVals_2__57_ 32013 0x200016EA  AFIT8_Postdmsc_iLowSat [7:0] AFIT8_Postdmsc_iHighSat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E10);  //                      AfitBaseVals_2__58_ 7696 0x200016EC  AFIT8_Postdmsc_iBCoeff [7:0] AFIT8_Postdmsc_iGCoeff [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C0B);  //                      AfitBaseVals_2__59_ 7179 0x200016EE  AFIT8_Postdmsc_iWideMult [7:0] AFIT8_Postdmsc_iTune [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C04);  //                      AfitBaseVals_2__60_ 3076 0x200016F0  AFIT8_Postdmsc_NoisePower_Low [7:0] AFIT8_Postdmsc_NoisePower_High [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1004);  //                      AfitBaseVals_2__61_ 4100 0x200016F2  AFIT8_Postdmsc_NoisePower_VLow [7:0] AFIT8_Postdmsc_NoiseLimit_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A10);  //                      AfitBaseVals_2__62_ 2576 0x200016F4  AFIT8_Postdmsc_NoiseLimit_High [7:0] AFIT8_Postdmsc_iSkinNS [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0308);  //                      AfitBaseVals_2__63_ 0776 0x200016F6  AFIT8_Postdmsc_iReduceNS_EdgeTh [7:0] AFIT8_Postdmsc_iReduceNS_Slope [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_2__64_ 1792 0x200016F8  AFIT8_Yuvemix_mNegSlopes_0 [7:0] AFIT8_Yuvemix_mNegSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_2__65_ 1286 0x200016FA  AFIT8_Yuvemix_mNegSlopes_2 [7:0] AFIT8_Yuvemix_mNegSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_2__66_ 1792 0x200016FC  AFIT8_Yuvemix_mPosSlopes_0 [7:0] AFIT8_Yuvemix_mPosSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_2__67_ 1286 0x200016FE  AFIT8_Yuvemix_mPosSlopes_2 [7:0] AFIT8_Yuvemix_mPosSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1414);  //                      AfitBaseVals_2__68_ 5140 0x20001700  AFIT8_Yuviirnr_iYThreshL [7:0] AFIT8_Yuviirnr_iYThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_2__69_ 3084 0x20001702  AFIT8_Yuviirnr_iYNRStrengthL [7:0] AFIT8_Yuviirnr_iYNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x080A);  //                      AfitBaseVals_2__70_ 2058 0x20001704  AFIT8_Yuviirnr_iUVThreshL [7:0] AFIT8_Yuviirnr_iUVThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C10);  //                      AfitBaseVals_2__71_ 3088 0x20001706  AFIT8_Yuviirnr_iDiffThreshL_UV [7:0] AFIT8_Yuviirnr_iDiffThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_2__72_ 1030 0x20001708  AFIT8_Yuviirnr_iMaxThreshL_UV [7:0] AFIT8_Yuviirnr_iMaxThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C1F);  //                      AfitBaseVals_2__73_ 7199 0x2000170A  AFIT8_Yuviirnr_iUVNRStrengthL [7:0] AFIT8_Yuviirnr_iUVNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8080);  //                      AfitBaseVals_2__74_ 32896 0x2000170C  AFIT8_byr_gras_iShadingPower [7:0] AFIT8_RGBGamma2_iLinearity [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_2__75_ 32768 0x2000170E  AFIT8_RGBGamma2_iDarkReduce [7:0] AFIT8_ccm_oscar_iSaturation [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_2__76_ 32768 0x20001710  AFIT8_RGB2YUV_iYOffset [7:0] AFIT8_RGB2YUV_iRGBGain [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x6D01);  //                      AfitBaseVals_2__77_ 13825 0x20001712  AFIT8_Dspcl_nClustLevel_H [7:0] AFIT8_EE_iLowSharpPower [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x017D);  //                      AfitBaseVals_2__78_ 0326 0x20001714  AFIT8_EE_iHighSharpPower [7:0] AFIT8_Dspcl_nClustLevel_H_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7B76);  //                      AfitBaseVals_2__79_ 17974 0x20001716  AFIT8_EE_iLowSharpPower_Bin [7:0] AFIT8_EE_iHighSharpPower_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_3__0_ 0000 0x20001718  AFIT16_BRIGHTNESS
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_3__1_ 0000 0x2000171A  AFIT16_CONTRAST
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_3__2_ 0000 0x2000171C  AFIT16_SATURATION
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_3__3_ 0000 0x2000171E  AFIT16_SHARP_BLUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_3__4_ 0000 0x20001720  AFIT16_GLAMOUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050);  //                       AfitBaseVals_3__5_ 0080 0x20001722  AFIT16_EE_iFlatBoundary
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                       AfitBaseVals_3__6_ 0048 0x20001724  AFIT16_Yuvemix_mNegRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0060);  //                       AfitBaseVals_3__7_ 0096 0x20001726  AFIT16_Yuvemix_mNegRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                       AfitBaseVals_3__8_ 0128 0x20001728  AFIT16_Yuvemix_mNegRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000C);  //                       AfitBaseVals_3__9_ 0012 0x2000172A  AFIT16_Yuvemix_mPosRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                      AfitBaseVals_3__10_ 0048 0x2000172C  AFIT16_Yuvemix_mPosRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                      AfitBaseVals_3__11_ 0128 0x2000172E  AFIT16_Yuvemix_mPosRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F18);  //                      AfitBaseVals_3__12_ 12056 0x20001730  AFIT8_Dspcl_edge_low [7:0] AFIT8_Dspcl_edge_high [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0420);  //                      AfitBaseVals_3__13_ 1056 0x20001732  AFIT8_Dspcl_repl_thresh [7:0] AFIT8_Dspcl_iConnectedThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C20);  //                      AfitBaseVals_3__14_ 15392 0x20001734  AFIT8_Dspcl_iPlainLevel [7:0] AFIT8_Dspcl_iSatThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C01);  //                      AfitBaseVals_3__15_ 3073 0x20001736  AFIT8_Dspcl_iPlainReference_H [7:0] AFIT8_Dspcl_iVarianceMultThresh_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_3__16_ 4108 0x20001738  AFIT8_Dspcl_iVariancePlainMax_H [7:0] AFIT8_Dspcl_iVarianceLimitMax_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_3__17_ 0257 0x2000173A  AFIT8_Dspcl_nClustLevel_C [7:0] AFIT8_Dspcl_iPlainReference_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_3__18_ 3084 0x2000173C  AFIT8_Dspcl_iVarianceMultThresh_C [7:0] AFIT8_Dspcl_iVariancePlainMax_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3010);  //                      AfitBaseVals_3__19_ 12304 0x2000173E  AFIT8_Dspcl_iVarianceLimitMax_C [7:0] AFIT8_EE_iShVLowRegion [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_3__20_ 4108 0x20001740  AFIT8_EE_iFSmagPosPwrLow [7:0] AFIT8_EE_iFSmagPosPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1810);  //                      AfitBaseVals_3__21_ 6160 0x20001742  AFIT8_EE_iFSmagNegPwrLow [7:0] AFIT8_EE_iFSmagNegPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_3__22_ 0000 0x20001744  AFIT8_EE_iFSThLow [7:0] AFIT8_EE_iFSThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0602);  //                      AfitBaseVals_3__23_ 1538 0x20001746  AFIT8_EE_iXformTh_High [7:0] AFIT8_EE_iXformTh_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x060C);  //                      AfitBaseVals_3__24_ 1548 0x20001748  AFIT8_EE_iVLowFSmagPower [7:0] AFIT8_EE_iVLowiXformTh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0510);  //                      AfitBaseVals_3__25_ 1296 0x2000174A  AFIT8_EE_iReduceNoiseRatio [7:0] AFIT8_EE_iFlatSpan [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1008);  //                      AfitBaseVals_3__26_ 4104 0x2000174C  AFIT8_EE_iMSharpenLow [7:0] AFIT8_EE_iMSharpenHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_3__27_ 0257 0x2000174E  AFIT8_EE_iFlatMean [7:0] AFIT8_EE_iFlatOffset [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_3__28_ 0000 0x20001750  AFIT8_EE_iMShThLow [7:0] AFIT8_EE_iMShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_3__29_ 1030 0x20001752  AFIT8_EE_iMShDirThLow [7:0] AFIT8_EE_iMShDirThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0608);  //                      AfitBaseVals_3__30_ 1544 0x20001754  AFIT8_EE_iMShVLowPwr [7:0] AFIT8_EE_iMShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x140C);  //                      AfitBaseVals_3__31_ 5132 0x20001756  AFIT8_EE_iWSharpenPosLow [7:0] AFIT8_EE_iWSharpenPosHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x180C);  //                      AfitBaseVals_3__32_ 6156 0x20001758  AFIT8_EE_iWSharpenNegLow [7:0] AFIT8_EE_iWSharpenNegHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0204);  //                      AfitBaseVals_3__33_ 0516 0x2000175A  AFIT8_EE_iWShThLow [7:0] AFIT8_EE_iWShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x040C);  //                      AfitBaseVals_3__34_ 1036 0x2000175C  AFIT8_EE_iWShVLowPwr [7:0] AFIT8_EE_iWShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A00);  //                      AfitBaseVals_3__35_ 23040 0x2000175E  AFIT8_EE_iReduceNegative [7:0] AFIT8_EE_iRadialLimitSh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4002);  //                      AfitBaseVals_3__36_ 16386 0x20001760  AFIT8_EE_iRadialPowerSh [7:0] AFIT8_Bdns_iDispTH_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0440);  //                      AfitBaseVals_3__37_ 1088 0x20001762  AFIT8_Bdns_iDispTH_H [7:0] AFIT8_Bdns_iDispLimit_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A04);  //                      AfitBaseVals_3__38_ 2564 0x20001764  AFIT8_Bdns_iDispLimit_H [7:0] AFIT8_Bdns_iDispTH4HF [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_3__39_ 0257 0x20001766  AFIT8_Bdns_iDispLimit4HF_L [7:0] AFIT8_Bdns_iDispLimit4HF_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1420);  //                      AfitBaseVals_3__40_ 5152 0x20001768  AFIT8_Bdns_iDenoiseTH_G_L [7:0] AFIT8_Bdns_iDenoiseTH_G_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1420);  //                      AfitBaseVals_3__41_ 5152 0x2000176A  AFIT8_Bdns_iDenoiseTH_NG_L [7:0] AFIT8_Bdns_iDenoiseTH_NG_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  //                      AfitBaseVals_3__42_ 1280 0x2000176C  AFIT8_Bdns_iDistSigmaMin [7:0] AFIT8_Bdns_iDistSigmaMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C3C);  //                      AfitBaseVals_3__43_ 15420 0x2000176E  AFIT8_Bdns_iDenoiseTH_Add_Plain [7:0] AFIT8_Bdns_iDenoiseTH_Add_Direc [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E04);  //                      AfitBaseVals_3__44_ 7684 0x20001770  AFIT8_Bdns_iDirConfidenceMin [7:0] AFIT8_Bdns_iDirConfidenceMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2808);  //                      AfitBaseVals_3__45_ 10248 0x20001772  AFIT8_Bdns_iPatternTH_MIN [7:0] AFIT8_Bdns_iPatternTH_MAX [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2000);  //                      AfitBaseVals_3__46_ 8192 0x20001774  AFIT8_Bdns_iNRTune [7:0] AFIT8_Bdns_iLowMaxSlopeAllowed [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A20);  //                      AfitBaseVals_3__47_ 23072 0x20001776  AFIT8_Bdns_iHighMaxSlopeAllowed [7:0] AFIT8_Bdns_iRadialLimitNR [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C02);  //                      AfitBaseVals_3__48_ 3074 0x20001778  AFIT8_Bdns_iRadialPowerNR [7:0] AFIT8_Dmsc_iEnhThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0F08);  //                      AfitBaseVals_3__49_ 3848 0x2000177A  AFIT8_Dmsc_iDesatThresh [7:0] AFIT8_Dmsc_iDemBlurLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x050F);  //                      AfitBaseVals_3__50_ 1295 0x2000177C  AFIT8_Dmsc_iDemBlurHigh [7:0] AFIT8_Dmsc_iDemBlurRange [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8006);  //                      AfitBaseVals_3__51_ 32774 0x2000177E  AFIT8_Dmsc_iDecisionThresh [7:0] AFIT8_Dmsc_iCentGrad [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                      AfitBaseVals_3__52_ 0032 0x20001780  AFIT8_Dmsc_iMonochrom [7:0] AFIT8_Dmsc_iGRDenoiseVal [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_3__53_ 0000 0x20001782  AFIT8_Dmsc_iGBDenoiseVal [7:0] AFIT8_Dmsc_iEdgeDesatThrLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1800);  //                      AfitBaseVals_3__54_ 6144 0x20001784  AFIT8_Dmsc_iEdgeDesatThrHigh [7:0] AFIT8_Dmsc_iEdgeDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);  //                      AfitBaseVals_3__55_ 0000 0x20001786  AFIT8_Dmsc_iEdgeDesatLimit [7:0] AFIT8_Dmsc_iNearGrayDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE119);  //                      AfitBaseVals_3__56_ 57625 0x20001788  AFIT8_Postdmsc_iLowBright [7:0] AFIT8_Postdmsc_iHighBright [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7D0D);  //                      AfitBaseVals_3__57_ 32013 0x2000178A  AFIT8_Postdmsc_iLowSat [7:0] AFIT8_Postdmsc_iHighSat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E10);  //                      AfitBaseVals_3__58_ 7696 0x2000178C  AFIT8_Postdmsc_iBCoeff [7:0] AFIT8_Postdmsc_iGCoeff [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C0B);  //                      AfitBaseVals_3__59_ 7179 0x2000178E  AFIT8_Postdmsc_iWideMult [7:0] AFIT8_Postdmsc_iTune [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C08);  //                      AfitBaseVals_3__60_ 3080 0x20001790  AFIT8_Postdmsc_NoisePower_Low [7:0] AFIT8_Postdmsc_NoisePower_High [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1406);  //                      AfitBaseVals_3__61_ 5126 0x20001792  AFIT8_Postdmsc_NoisePower_VLow [7:0] AFIT8_Postdmsc_NoiseLimit_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A14);  //                      AfitBaseVals_3__62_ 2580 0x20001794  AFIT8_Postdmsc_NoiseLimit_High [7:0] AFIT8_Postdmsc_iSkinNS [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0308);  //                      AfitBaseVals_3__63_ 0776 0x20001796  AFIT8_Postdmsc_iReduceNS_EdgeTh [7:0] AFIT8_Postdmsc_iReduceNS_Slope [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_3__64_ 1792 0x20001798  AFIT8_Yuvemix_mNegSlopes_0 [7:0] AFIT8_Yuvemix_mNegSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_3__65_ 1286 0x2000179A  AFIT8_Yuvemix_mNegSlopes_2 [7:0] AFIT8_Yuvemix_mNegSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_3__66_ 1792 0x2000179C  AFIT8_Yuvemix_mPosSlopes_0 [7:0] AFIT8_Yuvemix_mPosSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_3__67_ 1286 0x2000179E  AFIT8_Yuvemix_mPosSlopes_2 [7:0] AFIT8_Yuvemix_mPosSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1414);  //                      AfitBaseVals_3__68_ 5140 0x200017A0  AFIT8_Yuviirnr_iYThreshL [7:0] AFIT8_Yuviirnr_iYThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_3__69_ 3084 0x200017A2  AFIT8_Yuviirnr_iYNRStrengthL [7:0] AFIT8_Yuviirnr_iYNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x080A);  //                      AfitBaseVals_3__70_ 2058 0x200017A4  AFIT8_Yuviirnr_iUVThreshL [7:0] AFIT8_Yuviirnr_iUVThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C10);  //                      AfitBaseVals_3__71_ 3088 0x200017A6  AFIT8_Yuviirnr_iDiffThreshL_UV [7:0] AFIT8_Yuviirnr_iDiffThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_3__72_ 1030 0x200017A8  AFIT8_Yuviirnr_iMaxThreshL_UV [7:0] AFIT8_Yuviirnr_iMaxThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C1F);  //                      AfitBaseVals_3__73_ 7199 0x200017AA  AFIT8_Yuviirnr_iUVNRStrengthL [7:0] AFIT8_Yuviirnr_iUVNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8080);  //                      AfitBaseVals_3__74_ 32896 0x200017AC  AFIT8_byr_gras_iShadingPower [7:0] AFIT8_RGBGamma2_iLinearity [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_3__75_ 32768 0x200017AE  AFIT8_RGBGamma2_iDarkReduce [7:0] AFIT8_ccm_oscar_iSaturation [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_3__76_ 32768 0x200017B0  AFIT8_RGB2YUV_iYOffset [7:0] AFIT8_RGB2YUV_iRGBGain [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7A01);  //                      AfitBaseVals_3__77_ 14849 0x200017B2  AFIT8_Dspcl_nClustLevel_H [7:0] AFIT8_EE_iLowSharpPower [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0192);  //                      AfitBaseVals_3__78_ 0326 0x200017B4  AFIT8_EE_iHighSharpPower [7:0] AFIT8_Dspcl_nClustLevel_H_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8F7A);  //                      AfitBaseVals_3__79_ 17978 0x200017B6  AFIT8_EE_iLowSharpPower_Bin [7:0] AFIT8_EE_iHighSharpPower_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_4__0_ 0000 0x200017B8  AFIT16_BRIGHTNESS
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_4__1_ 0000 0x200017BA  AFIT16_CONTRAST
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_4__2_ 0000 0x200017BC  AFIT16_SATURATION
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_4__3_ 0000 0x200017BE  AFIT16_SHARP_BLUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                       AfitBaseVals_4__4_ 0000 0x200017C0  AFIT16_GLAMOUR
S5K5EAYX_write_cmos_sensor(0x0F12, 0x003C);  //                       AfitBaseVals_4__5_ 0060 0x200017C2  AFIT16_EE_iFlatBoundary
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                       AfitBaseVals_4__6_ 0048 0x200017C4  AFIT16_Yuvemix_mNegRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0060);  //                       AfitBaseVals_4__7_ 0096 0x200017C6  AFIT16_Yuvemix_mNegRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                       AfitBaseVals_4__8_ 0128 0x200017C8  AFIT16_Yuvemix_mNegRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x000C);  //                       AfitBaseVals_4__9_ 0012 0x200017CA  AFIT16_Yuvemix_mPosRanges_0
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0030);  //                      AfitBaseVals_4__10_ 0048 0x200017CC  AFIT16_Yuvemix_mPosRanges_1
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0080);  //                      AfitBaseVals_4__11_ 0128 0x200017CE  AFIT16_Yuvemix_mPosRanges_2
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2F18);  //                      AfitBaseVals_4__12_ 12056 0x200017D0  AFIT8_Dspcl_edge_low [7:0] AFIT8_Dspcl_edge_high [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0420);  //                      AfitBaseVals_4__13_ 1056 0x200017D2  AFIT8_Dspcl_repl_thresh [7:0] AFIT8_Dspcl_iConnectedThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C20);  //                      AfitBaseVals_4__14_ 15392 0x200017D4  AFIT8_Dspcl_iPlainLevel [7:0] AFIT8_Dspcl_iSatThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C01);  //                      AfitBaseVals_4__15_ 3073 0x200017D6  AFIT8_Dspcl_iPlainReference_H [7:0] AFIT8_Dspcl_iVarianceMultThresh_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x100C);  //                      AfitBaseVals_4__16_ 4108 0x200017D8  AFIT8_Dspcl_iVariancePlainMax_H [7:0] AFIT8_Dspcl_iVarianceLimitMax_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_4__17_ 0257 0x200017DA  AFIT8_Dspcl_nClustLevel_C [7:0] AFIT8_Dspcl_iPlainReference_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C0C);  //                      AfitBaseVals_4__18_ 3084 0x200017DC  AFIT8_Dspcl_iVarianceMultThresh_C [7:0] AFIT8_Dspcl_iVariancePlainMax_C [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3010);  //                      AfitBaseVals_4__19_ 12304 0x200017DE  AFIT8_Dspcl_iVarianceLimitMax_C [7:0] AFIT8_EE_iShVLowRegion [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x180C);  //                      AfitBaseVals_4__20_ 6156 0x200017E0  AFIT8_EE_iFSmagPosPwrLow [7:0] AFIT8_EE_iFSmagPosPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1810);  //                      AfitBaseVals_4__21_ 6160 0x200017E2  AFIT8_EE_iFSmagNegPwrLow [7:0] AFIT8_EE_iFSmagNegPwrHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_4__22_ 0000 0x200017E4  AFIT8_EE_iFSThLow [7:0] AFIT8_EE_iFSThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0402);  //                      AfitBaseVals_4__23_ 1026 0x200017E6  AFIT8_EE_iXformTh_High [7:0] AFIT8_EE_iXformTh_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x040C);  //                      AfitBaseVals_4__24_ 1036 0x200017E8  AFIT8_EE_iVLowFSmagPower [7:0] AFIT8_EE_iVLowiXformTh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x050C);  //                      AfitBaseVals_4__25_ 1292 0x200017EA  AFIT8_EE_iReduceNoiseRatio [7:0] AFIT8_EE_iFlatSpan [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x140C);  //                      AfitBaseVals_4__26_ 5132 0x200017EC  AFIT8_EE_iMSharpenLow [7:0] AFIT8_EE_iMSharpenHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0101);  //                      AfitBaseVals_4__27_ 0257 0x200017EE  AFIT8_EE_iFlatMean [7:0] AFIT8_EE_iFlatOffset [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_4__28_ 0000 0x200017F0  AFIT8_EE_iMShThLow [7:0] AFIT8_EE_iMShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0205);  //                      AfitBaseVals_4__29_ 0517 0x200017F2  AFIT8_EE_iMShDirThLow [7:0] AFIT8_EE_iMShDirThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x050C);  //                      AfitBaseVals_4__30_ 1292 0x200017F4  AFIT8_EE_iMShVLowPwr [7:0] AFIT8_EE_iMShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1810);  //                      AfitBaseVals_4__31_ 6160 0x200017F6  AFIT8_EE_iWSharpenPosLow [7:0] AFIT8_EE_iWSharpenPosHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2010);  //                      AfitBaseVals_4__32_ 8208 0x200017F8  AFIT8_EE_iWSharpenNegLow [7:0] AFIT8_EE_iWSharpenNegHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0102);  //                      AfitBaseVals_4__33_ 0258 0x200017FA  AFIT8_EE_iWShThLow [7:0] AFIT8_EE_iWShThHigh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0210);  //                      AfitBaseVals_4__34_ 0528 0x200017FC  AFIT8_EE_iWShVLowPwr [7:0] AFIT8_EE_iWShVLowThrld [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A00);  //                      AfitBaseVals_4__35_ 23040 0x200017FE  AFIT8_EE_iReduceNegative [7:0] AFIT8_EE_iRadialLimitSh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x4001);  //                      AfitBaseVals_4__36_ 16385 0x20001800  AFIT8_EE_iRadialPowerSh [7:0] AFIT8_Bdns_iDispTH_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0240);  //                      AfitBaseVals_4__37_ 0576 0x20001802  AFIT8_Bdns_iDispTH_H [7:0] AFIT8_Bdns_iDispLimit_L [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0402);  //                      AfitBaseVals_4__38_ 1026 0x20001804  AFIT8_Bdns_iDispLimit_H [7:0] AFIT8_Bdns_iDispTH4HF [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_4__39_ 0000 0x20001806  AFIT8_Bdns_iDispLimit4HF_L [7:0] AFIT8_Bdns_iDispLimit4HF_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C18);  //                      AfitBaseVals_4__40_ 3096 0x20001808  AFIT8_Bdns_iDenoiseTH_G_L [7:0] AFIT8_Bdns_iDenoiseTH_G_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C18);  //                      AfitBaseVals_4__41_ 3096 0x2000180A  AFIT8_Bdns_iDenoiseTH_NG_L [7:0] AFIT8_Bdns_iDenoiseTH_NG_H [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //                      AfitBaseVals_4__42_ 0768 0x2000180C  AFIT8_Bdns_iDistSigmaMin [7:0] AFIT8_Bdns_iDistSigmaMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3C32);  //                      AfitBaseVals_4__43_ 15410 0x2000180E  AFIT8_Bdns_iDenoiseTH_Add_Plain [7:0] AFIT8_Bdns_iDenoiseTH_Add_Direc [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2802);  //                      AfitBaseVals_4__44_ 10242 0x20001810  AFIT8_Bdns_iDirConfidenceMin [7:0] AFIT8_Bdns_iDirConfidenceMax [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x2806);  //                      AfitBaseVals_4__45_ 10246 0x20001812  AFIT8_Bdns_iPatternTH_MIN [7:0] AFIT8_Bdns_iPatternTH_MAX [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1000);  //                      AfitBaseVals_4__46_ 4096 0x20001814  AFIT8_Bdns_iNRTune [7:0] AFIT8_Bdns_iLowMaxSlopeAllowed [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5A10);  //                      AfitBaseVals_4__47_ 23056 0x20001816  AFIT8_Bdns_iHighMaxSlopeAllowed [7:0] AFIT8_Bdns_iRadialLimitNR [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0801);  //                      AfitBaseVals_4__48_ 2049 0x20001818  AFIT8_Bdns_iRadialPowerNR [7:0] AFIT8_Dmsc_iEnhThresh [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0808);  //                      AfitBaseVals_4__49_ 2056 0x2000181A  AFIT8_Dmsc_iDesatThresh [7:0] AFIT8_Dmsc_iDemBlurLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0508);  //                      AfitBaseVals_4__50_ 1288 0x2000181C  AFIT8_Dmsc_iDemBlurHigh [7:0] AFIT8_Dmsc_iDemBlurRange [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8006);  //                      AfitBaseVals_4__51_ 32774 0x2000181E  AFIT8_Dmsc_iDecisionThresh [7:0] AFIT8_Dmsc_iCentGrad [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0020);  //                      AfitBaseVals_4__52_ 0032 0x20001820  AFIT8_Dmsc_iMonochrom [7:0] AFIT8_Dmsc_iGRDenoiseVal [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                      AfitBaseVals_4__53_ 0000 0x20001822  AFIT8_Dmsc_iGBDenoiseVal [7:0] AFIT8_Dmsc_iEdgeDesatThrLow [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0C00);  //                      AfitBaseVals_4__54_ 3072 0x20001824  AFIT8_Dmsc_iEdgeDesatThrHigh [7:0] AFIT8_Dmsc_iEdgeDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400);  //                      AfitBaseVals_4__55_ 0000 0x20001826  AFIT8_Dmsc_iEdgeDesatLimit [7:0] AFIT8_Dmsc_iNearGrayDesat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0xE119);  //                      AfitBaseVals_4__56_ 57625 0x20001828  AFIT8_Postdmsc_iLowBright [7:0] AFIT8_Postdmsc_iHighBright [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7D0D);  //                      AfitBaseVals_4__57_ 32013 0x2000182A  AFIT8_Postdmsc_iLowSat [7:0] AFIT8_Postdmsc_iHighSat [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1E10);  //                      AfitBaseVals_4__58_ 7696 0x2000182C  AFIT8_Postdmsc_iBCoeff [7:0] AFIT8_Postdmsc_iGCoeff [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C0B);  //                      AfitBaseVals_4__59_ 7179 0x2000182E  AFIT8_Postdmsc_iWideMult [7:0] AFIT8_Postdmsc_iTune [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1008);  //                      AfitBaseVals_4__60_ 4104 0x20001830  AFIT8_Postdmsc_NoisePower_Low [7:0] AFIT8_Postdmsc_NoisePower_High [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1408);  //                      AfitBaseVals_4__61_ 5128 0x20001832  AFIT8_Postdmsc_NoisePower_VLow [7:0] AFIT8_Postdmsc_NoiseLimit_Low [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A14);  //                      AfitBaseVals_4__62_ 2580 0x20001834  AFIT8_Postdmsc_NoiseLimit_High [7:0] AFIT8_Postdmsc_iSkinNS [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0308);  //                      AfitBaseVals_4__63_ 0776 0x20001836  AFIT8_Postdmsc_iReduceNS_EdgeTh [7:0] AFIT8_Postdmsc_iReduceNS_Slope [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_4__64_ 1792 0x20001838  AFIT8_Yuvemix_mNegSlopes_0 [7:0] AFIT8_Yuvemix_mNegSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_4__65_ 1286 0x2000183A  AFIT8_Yuvemix_mNegSlopes_2 [7:0] AFIT8_Yuvemix_mNegSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0700);  //                      AfitBaseVals_4__66_ 1792 0x2000183C  AFIT8_Yuvemix_mPosSlopes_0 [7:0] AFIT8_Yuvemix_mPosSlopes_1 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0506);  //                      AfitBaseVals_4__67_ 1286 0x2000183E  AFIT8_Yuvemix_mPosSlopes_2 [7:0] AFIT8_Yuvemix_mPosSlopes_3 [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1010);  //                      AfitBaseVals_4__68_ 4112 0x20001840  AFIT8_Yuviirnr_iYThreshL [7:0] AFIT8_Yuviirnr_iYThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0808);  //                      AfitBaseVals_4__69_ 2056 0x20001842  AFIT8_Yuviirnr_iYNRStrengthL [7:0] AFIT8_Yuviirnr_iYNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x080A);  //                      AfitBaseVals_4__70_ 2058 0x20001844  AFIT8_Yuviirnr_iUVThreshL [7:0] AFIT8_Yuviirnr_iUVThreshH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x080C);  //                      AfitBaseVals_4__71_ 2060 0x20001846  AFIT8_Yuviirnr_iDiffThreshL_UV [7:0] AFIT8_Yuviirnr_iDiffThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0406);  //                      AfitBaseVals_4__72_ 1030 0x20001848  AFIT8_Yuviirnr_iMaxThreshL_UV [7:0] AFIT8_Yuviirnr_iMaxThreshH_UV [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x1C1F);  //                      AfitBaseVals_4__73_ 7199 0x2000184A  AFIT8_Yuviirnr_iUVNRStrengthL [7:0] AFIT8_Yuviirnr_iUVNRStrengthH [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8080);  //                      AfitBaseVals_4__74_ 32896 0x2000184C  AFIT8_byr_gras_iShadingPower [7:0] AFIT8_RGBGamma2_iLinearity [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_4__75_ 32768 0x2000184E  AFIT8_RGBGamma2_iDarkReduce [7:0] AFIT8_ccm_oscar_iSaturation [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8000);  //                      AfitBaseVals_4__76_ 32768 0x20001850  AFIT8_RGB2YUV_iYOffset [7:0] AFIT8_RGB2YUV_iRGBGain [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x7A01);  //                      AfitBaseVals_4__77_ 16385 0x20001852  AFIT8_Dspcl_nClustLevel_H [7:0] AFIT8_EE_iLowSharpPower [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0192);  //                      AfitBaseVals_4__78_ 0396 0x20001854  AFIT8_EE_iHighSharpPower [7:0] AFIT8_Dspcl_nClustLevel_H_Bin [15:8]
S5K5EAYX_write_cmos_sensor(0x0F12, 0x8C7D);  //                      AfitBaseVals_4__79_ 35904 0x20001856  AFIT8_EE_iLowSharpPower_Bin [7:0] AFIT8_EE_iHighSharpPower_Bin [15:8]
//>2014/5/6-37370-joubert.she.
S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFEE);  ////[0]CAFITB_Dspcl_bypass
S5K5EAYX_write_cmos_sensor(0x0F12, 0x3376);  ////[0]CAFITB_EE_bReduceNegative
S5K5EAYX_write_cmos_sensor(0x0F12, 0xBC3F);  ////[0]CAFITB_Dmsc_bEnhThresh
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0337);  ////[0]CAFITB_Postdmsc_bNoiseLimit
S5K5EAYX_write_cmos_sensor(0x002A, 0x01E4);  //0x200001E4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x5DC0);  //                      REG_TC_IPRM_CM_Init 24000 / 24Mhz
S5K5EAYX_write_cmos_sensor(0x002A, 0x0200);  //0x20000200 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                 REG_TC_IPRM_wNumOfClocks 0002 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0006);  //            REG_TC_IPRM_pllSets_0__vco_wP 0006 /
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0086);  //            REG_TC_IPRM_pllSets_0__vco_wM 0134 /67MHz
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //            REG_TC_IPRM_pllSets_0__vco_wS 0000 /
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //          REG_TC_IPRM_pllSets_0__div_wDbr 0005 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //          REG_TC_IPRM_pllSets_0__div_wSys 0004 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0214);  //0x20000214 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //       REG_TC_IPRM_pllSets_0__wNumOfLanes 0002 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0006);  //            REG_TC_IPRM_pllSets_1__vco_wP 0006 /
S5K5EAYX_write_cmos_sensor(0x0F12, 0x00A2);  //            REG_TC_IPRM_pllSets_1__vco_wM 0162 /81MHz
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //            REG_TC_IPRM_pllSets_1__vco_wS 0000 /
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0006);  //          REG_TC_IPRM_pllSets_1__div_wDbr 0006 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0004);  //          REG_TC_IPRM_pllSets_1__div_wSys 0004 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0228);  //0x20000228 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //       REG_TC_IPRM_pllSets_1__wNumOfLanes 0002 
S5K5EAYX_write_cmos_sensor(0x002A, 0x04B4);  //0x200004B4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x005F);  //                 REG_TC_BRC_usPrevQuality 0095 REG_TC_BRC_usPrevQuality
S5K5EAYX_write_cmos_sensor(0x0F12, 0x005F);  //              REG_TC_BRC_usCaptureQuality 0095 REG_TC_BRC_usCaptureQuality
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //               REG_TC_THUMB_Thumb_bActive 0001 REG_TC_THUMB_Thumb_bActive
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0280);  //                REG_TC_THUMB_Thumb_uWidth 0640 REG_TC_THUMB_Thumb_uWidth
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E0);  //               REG_TC_THUMB_Thumb_uHeight 0480 REG_TC_THUMB_Thumb_uHeight
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                REG_TC_THUMB_Thumb_Format 0005 REG_TC_THUMB_Thumb_Format
S5K5EAYX_write_cmos_sensor(0x002A, 0x12C8);  //0x200012C8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0054);  //                          jpeg_ManualMBCV 0084 jpeg_ManualMBCV
S5K5EAYX_write_cmos_sensor(0x002A, 0x0D24);  //0x20000D24 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x001C);  //                     senHal_bExtraAddLine 0028 senHal_bExtraAddLine
S5K5EAYX_write_cmos_sensor(0x002A, 0x02BE);  //0x200002BE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //               REG_TC_GP_bBypassScalerJpg 0001 REG_TC_GP_bBypassScalerJpg
S5K5EAYX_write_cmos_sensor(0x002A, 0x02C4);  //0x200002C4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_TC_GP_bUse1FrameCaptureMode 0000 REG_TC_GP_bUse1FrameCaptureMode
S5K5EAYX_write_cmos_sensor(0x002A, 0x028A);  //0x2000028A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //              REG_TC_GP_PrevReqInputWidth 2560 REG_TC_GP_PrevReqInputWidth
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //             REG_TC_GP_PrevReqInputHeight 1920 REG_TC_GP_PrevReqInputHeight
S5K5EAYX_write_cmos_sensor(0x002A, 0x0292);  //0x20000292 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //               REG_TC_GP_CapReqInputWidth 2560 REG_TC_GP_CapReqInputWidth
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //              REG_TC_GP_CapReqInputHeight 1920 REG_TC_GP_CapReqInputHeight
S5K5EAYX_write_cmos_sensor(0x002A, 0x029C);  //0x2000029C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //              REG_TC_GP_bUseReqInputInPre 0001 REG_TC_GP_bUseReqInputInPre
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //              REG_TC_GP_bUseReqInputInCap 0001 REG_TC_GP_bUseReqInputInCap
S5K5EAYX_write_cmos_sensor(0x002A, 0x04D0);  //0x200004D0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //       REG_TC_PZOOM_PrevZoomReqInputWidth 2560 REG_TC_PZOOM_PrevZoomReqInputWidth
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //      REG_TC_PZOOM_PrevZoomReqInputHeight 1920 REG_TC_PZOOM_PrevZoomReqInputHeight
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //    REG_TC_PZOOM_PrevZoomReqInputWidthOfs 0000 REG_TC_PZOOM_PrevZoomReqInputWidthOfs
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //   REG_TC_PZOOM_PrevZoomReqInputHeightOfs 0000 REG_TC_PZOOM_PrevZoomReqInputHeightOfs
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //        REG_TC_PZOOM_CapZoomReqInputWidth 2560 REG_TC_PZOOM_CapZoomReqInputWidth
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //       REG_TC_PZOOM_CapZoomReqInputHeight 1920 REG_TC_PZOOM_CapZoomReqInputHeight
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //     REG_TC_PZOOM_CapZoomReqInputWidthOfs 0000 REG_TC_PZOOM_CapZoomReqInputWidthOfs
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //    REG_TC_PZOOM_CapZoomReqInputHeightOfs 0000 REG_TC_PZOOM_CapZoomReqInputHeightOfs
S5K5EAYX_write_cmos_sensor(0x002A, 0x02E2);  //0x200002E2  REG_PrevConfigControls_0_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  //                REG_PrevConfigControls_0_ 1280 REG_0TC_PCFG_usWidth	1280
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03C0);  //                    REG_0TC_PCFG_usHeight 0960 REG_0TC_PCFG_usHeight	960
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                      REG_0TC_PCFG_Format 0005 REG_0TC_PCFG_Format		YUV
S5K5EAYX_write_cmos_sensor(0x002A, 0x02EC);  //0x200002EC 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //              REG_0TC_PCFG_OutClkPerPix88 0256 REG_0TC_PCFG_OutClkPerPix88			
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //                      REG_0TC_PCFG_uBpp88 0768 REG_0TC_PCFG_uBpp88							
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                     REG_0TC_PCFG_PVIMask 0064 REG_0TC_PCFG_PVIMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                     REG_0TC_PCFG_OIFMask 0000 REG_0TC_PCFG_OIFMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E0);  //            REG_0TC_PCFG_usJpegPacketSize 0480 REG_0TC_PCFG_usJpegPacketSize		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_0TC_PCFG_usJpegTotalPackets 0000 REG_0TC_PCFG_usJpegTotalPackets	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                   REG_0TC_PCFG_uClockInd 0000 REG_0TC_PCFG_uClockInd					 67
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                REG_0TC_PCFG_usFrTimeType 0000 0: Dynamic, 1:Not Accurate, 2: Fixed
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //           REG_0TC_PCFG_FrRateQualityType 0000 0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x029A);  //       REG_0TC_PCFG_usMaxFrTimeMsecMult10 1000 REG_0TC_PCFG_usMaxFrTimeMsecMult10 10fps
//>2014/5/2-37049-joubert.she,
S5K5EAYX_write_cmos_sensor(0x0F12, 0x014D);  //       REG_0TC_PCFG_usMinFrTimeMsecMult10 0333 REG_0TC_PCFG_usMinFrTimeMsecMult10 30fps
S5K5EAYX_write_cmos_sensor(0x002A, 0x030C);  //0x2000030C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                 REG_0TC_PCFG_uPrevMirror 0003 REG_0TC_PCFG_uPrevMirror
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //              REG_0TC_PCFG_uCaptureMirror 0003 REG_0TC_PCFG_uCaptureMirror
S5K5EAYX_write_cmos_sensor(0x002A, 0x0312);  //0x20000312 REG_PrevConfigControls_1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  //                REG_PrevConfigControls_1_ 1280 REG_1TC_PCFG_usWidth	1280
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03C0);  //                    REG_1TC_PCFG_usHeight 0960 REG_1TC_PCFG_usHeight	960
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                      REG_1TC_PCFG_Format 0005 REG_1TC_PCFG_Format		JPEG
S5K5EAYX_write_cmos_sensor(0x002A, 0x031C);  //0x2000031C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //              REG_1TC_PCFG_OutClkPerPix88 0256 REG_1TC_PCFG_OutClkPerPix88			
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //                      REG_1TC_PCFG_uBpp88 0768 REG_1TC_PCFG_uBpp88							
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                     REG_1TC_PCFG_PVIMask 0064 REG_1TC_PCFG_PVIMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                     REG_1TC_PCFG_OIFMask 0000 REG_1TC_PCFG_OIFMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E0);  //            REG_1TC_PCFG_usJpegPacketSize 0480 REG_1TC_PCFG_usJpegPacketSize		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_1TC_PCFG_usJpegTotalPackets 0000 REG_1TC_PCFG_usJpegTotalPackets 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                   REG_1TC_PCFG_uClockInd 0000 REG_1TC_PCFG_uClockInd 67
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                REG_1TC_PCFG_usFrTimeType 0000 0: Dynamic, 1:Not Accurate, 2: Fixed
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //           REG_1TC_PCFG_FrRateQualityType 0001 0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x07D0);  //       REG_1TC_PCFG_usMaxFrTimeMsecMult10 2000 5fps
S5K5EAYX_write_cmos_sensor(0x0F12, 0x014D);  //       REG_1TC_PCFG_usMinFrTimeMsecMult10 0333 30fps
S5K5EAYX_write_cmos_sensor(0x002A, 0x033C);  //0x2000033C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //                 REG_1TC_PCFG_uPrevMirror 0003 REG_1TC_PCFG_uPrevMirror		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //              REG_1TC_PCFG_uCaptureMirror 0003 REG_1TC_PCFG_uCaptureMirror 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0342);  //0x20000342 REG_PrevConfigControls_2_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0500);  //                REG_PrevConfigControls_2_ 1280 REG_2TC_PCFG_usWidth	1280
S5K5EAYX_write_cmos_sensor(0x0F12, 0x03C0);  //                    REG_2TC_PCFG_usHeight 0960 REG_2TC_PCFG_usHeight	960
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                      REG_2TC_PCFG_Format 0005 REG_2TC_PCFG_Format		YUV
S5K5EAYX_write_cmos_sensor(0x002A, 0x034C);  //0x2000034C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //              REG_2TC_PCFG_OutClkPerPix88 0256 REG_2TC_PCFG_OutClkPerPix88			
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //                      REG_2TC_PCFG_uBpp88 0768 REG_2TC_PCFG_uBpp88							
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                     REG_2TC_PCFG_PVIMask 0064 REG_2TC_PCFG_PVIMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                     REG_2TC_PCFG_OIFMask 0000 REG_2TC_PCFG_OIFMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E0);  //            REG_2TC_PCFG_usJpegPacketSize 0480 REG_2TC_PCFG_usJpegPacketSize		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_2TC_PCFG_usJpegTotalPackets 0000 REG_2TC_PCFG_usJpegTotalPackets	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                   REG_2TC_PCFG_uClockInd 0000 REG_2TC_PCFG_uClockInd					41
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //                REG_2TC_PCFG_usFrTimeType 0002 0: Dynamic, 1:Not Accurate, 2: Fixed
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //           REG_2TC_PCFG_FrRateQualityType 0000 0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x014D);  //       REG_2TC_PCFG_usMaxFrTimeMsecMult10 0333 REG_2TC_PCFG_usMaxFrTimeMsecMult10 30fps
S5K5EAYX_write_cmos_sensor(0x0F12, 0x014D);  //       REG_2TC_PCFG_usMinFrTimeMsecMult10 0333 REG_2TC_PCFG_usMinFrTimeMsecMult10 30fps
S5K5EAYX_write_cmos_sensor(0x002A, 0x036C);  //0x2000036C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //                 REG_2TC_PCFG_uPrevMirror 0003 REG_2TC_PCFG_uPrevMirror		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //              REG_2TC_PCFG_uCaptureMirror 0003 REG_2TC_PCFG_uCaptureMirror 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0372);  //0x20000372 REG_PrevConfigControls_3_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //                REG_PrevConfigControls_3_ 2560 REG_3TC_PCFG_usWidth	2560
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //                    REG_3TC_PCFG_usHeight 1920 REG_3TC_PCFG_usHeight	1920
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);  //                      REG_3TC_PCFG_Format 0007 REG_3TC_PCFG_Format		RAW
S5K5EAYX_write_cmos_sensor(0x002A, 0x037C);  //0x2000037C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //              REG_3TC_PCFG_OutClkPerPix88 0256 REG_3TC_PCFG_OutClkPerPix88			
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //                      REG_3TC_PCFG_uBpp88 0768 REG_3TC_PCFG_uBpp88							
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                     REG_3TC_PCFG_PVIMask 0064 REG_3TC_PCFG_PVIMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                     REG_3TC_PCFG_OIFMask 0000 REG_3TC_PCFG_OIFMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x01E0);  //            REG_3TC_PCFG_usJpegPacketSize 0480 REG_3TC_PCFG_usJpegPacketSize		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_3TC_PCFG_usJpegTotalPackets 0000 REG_3TC_PCFG_usJpegTotalPackets	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //                   REG_3TC_PCFG_uClockInd 0003 REG_3TC_PCFG_uClockInd					
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                REG_3TC_PCFG_usFrTimeType 0001 REG_3TC_PCFG_usFrTimeType					0: Dynamic, 1:Not Accurate, 2: Fixed
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //           REG_3TC_PCFG_FrRateQualityType 0002 REG_3TC_PCFG_FrRateQualityType		0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0535);  //       REG_3TC_PCFG_usMaxFrTimeMsecMult10 1333 REG_3TC_PCFG_usMaxFrTimeMsecMult107.5fps
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0535);  //       REG_3TC_PCFG_usMinFrTimeMsecMult10 1333 REG_3TC_PCFG_usMinFrTimeMsecMult107.5fps
S5K5EAYX_write_cmos_sensor(0x002A, 0x039C);  //0x2000039C 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //                 REG_3TC_PCFG_uPrevMirror 0003 REG_3TC_PCFG_uPrevMirror		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0003);  //              REG_3TC_PCFG_uCaptureMirror 0003 REG_3TC_PCFG_uCaptureMirror 
S5K5EAYX_write_cmos_sensor(0x002A, 0x03D2);  //0x200003D2 REG_CapConfigControls
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                 REG_CapConfigControls_0_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //                         REG_0TC_CCFG_Cfg 2560 REG_0TC_CCFG_usWidth	2560
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //                    REG_0TC_CCFG_usHeight 1920 REG_0TC_CCFG_usHeight	1920
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                      REG_0TC_CCFG_Format 0005 REG_0TC_CCFG_Format		JPEG
S5K5EAYX_write_cmos_sensor(0x002A, 0x03DE);  //0x200003DE 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //              REG_0TC_CCFG_OutClkPerPix88 0256 REG_0TC_CCFG_OutClkPerPix88			
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //                      REG_0TC_CCFG_uBpp88 0768 REG_0TC_CCFG_uBpp88							
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                     REG_0TC_CCFG_PVIMask 0064 REG_0TC_CCFG_PVIMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                     REG_0TC_CCFG_OIFMask 0064 REG_0TC_CCFG_OIFMask						  //JPEG8
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0810);  //            REG_0TC_CCFG_usJpegPacketSize 2064 REG_0TC_CCFG_usJpegPacketSize		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_0TC_CCFG_usJpegTotalPackets 0000 REG_0TC_CCFG_usJpegTotalPackets	 //SPOOF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                   REG_0TC_CCFG_uClockInd 0001 REG_0TC_CCFG_uClockInd					
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                REG_0TC_CCFG_usFrTimeType 0001 REG_0TC_CCFG_usFrTimeType					0: Dynamic, 1:Not Accurate, 2: Fixed
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //           REG_0TC_CCFG_FrRateQualityType 0002 REG_0TC_CCFG_FrRateQualityType		0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0535);  //       REG_0TC_CCFG_usMaxFrTimeMsecMult10 1000 REG_0TC_CCFG_usMaxFrTimeMsecMult10 10fps
S5K5EAYX_write_cmos_sensor(0x0F12, 0x029A);  //       REG_0TC_CCFG_usMinFrTimeMsecMult10 0666 REG_0TC_CCFG_usMinFrTimeMsecMult10 15fps
S5K5EAYX_write_cmos_sensor(0x002A, 0x03FE);  //0x200003FE  REG_CapConfigControls_1_
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                 REG_CapConfigControls_1_ 0000 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //                         REG_1TC_CCFG_Cfg 2560  REG_1TC_CCFG_usWidth	  2560*1920
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //                    REG_1TC_CCFG_usHeight 1920  REG_1TC_CCFG_usHeight	
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //                      REG_1TC_CCFG_Format 0005  REG_1TC_CCFG_Format		
S5K5EAYX_write_cmos_sensor(0x002A, 0x040A);  //0x2000040A 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //              REG_1TC_CCFG_OutClkPerPix88 0256 REG_1TC_CCFG_OutClkPerPix88			
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //                      REG_1TC_CCFG_uBpp88 0768 REG_1TC_CCFG_uBpp88							
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //                     REG_1TC_CCFG_PVIMask 0064 REG_1TC_CCFG_PVIMask						
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                     REG_1TC_CCFG_OIFMask 0064 REG_1TC_CCFG_OIFMask						  //JPEG8
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0810);  //            REG_1TC_CCFG_usJpegPacketSize 2064 REG_1TC_CCFG_usJpegPacketSize		
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //          REG_1TC_CCFG_usJpegTotalPackets 0000 REG_1TC_CCFG_usJpegTotalPackets	 //SPOOF
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                   REG_1TC_CCFG_uClockInd 0001 REG_1TC_CCFG_uClockInd					
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                REG_1TC_CCFG_usFrTimeType 0001 REG_1TC_CCFG_usFrTimeType					0: Dynamic, 1:Not Accurate, 2: Fixed
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //           REG_1TC_CCFG_FrRateQualityType 0002 REG_1TC_CCFG_FrRateQualityType		0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)
S5K5EAYX_write_cmos_sensor(0x0F12, 0x07D0);  //       REG_1TC_CCFG_usMaxFrTimeMsecMult10 2000 REG_1TC_CCFG_usMaxFrTimeMsecMult10 7.5fps
S5K5EAYX_write_cmos_sensor(0x0F12, 0x029A);  //       REG_1TC_CCFG_usMinFrTimeMsecMult10 0666 REG_1TC_CCFG_usMinFrTimeMsecMult10 15fps
S5K5EAYX_write_cmos_sensor(0x002A, 0x02A0);  //0x200002A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //               REG_TC_GP_ActivePrevConfig 0000 REG_TC_GP_ActivePrevConfig
S5K5EAYX_write_cmos_sensor(0x002A, 0x02A8);  //0x200002A8 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //                REG_TC_GP_ActiveCapConfig 0000 REG_TC_GP_ActiveCapConfig
S5K5EAYX_write_cmos_sensor(0x002A, 0x0266);  //0x20000266 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //            REG_TC_IPRM_InitParamsUpdated 0001 REG_TC_IPRM_InitParamsUpdated

// s5k5eayx_init_reg3 ----------------------
S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);  
S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);  //Page : 2000 
S5K5EAYX_write_cmos_sensor(0x002A, 0x02A0);  //0x200002A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //               REG_TC_GP_ActivePrevConfig 0000 REG_TC_GP_ActivePrevConfig
S5K5EAYX_write_cmos_sensor(0x002A, 0x02A4);  //0x200002A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //            REG_TC_GP_PrevOpenAfterChange 0001 REG_TC_GP_PrevOpenAfterChange
S5K5EAYX_write_cmos_sensor(0x002A, 0x02A2);  //0x200002A2 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //              REG_TC_GP_PrevConfigChanged 0001 REG_TC_GP_PrevConfigChanged
S5K5EAYX_write_cmos_sensor(0x002A, 0x0288);  //0x20000288 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                  REG_TC_GP_NewConfigSync 0001 
S5K5EAYX_write_cmos_sensor(0x002A, 0x02A0);  //0x200002A0 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //               REG_TC_GP_ActivePrevConfig 0000 
S5K5EAYX_write_cmos_sensor(0x002A, 0x02A4);  //0x200002A4 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //            REG_TC_GP_PrevOpenAfterChange 0001 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0288);  //0x20000288 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                  REG_TC_GP_NewConfigSync 0001 
S5K5EAYX_write_cmos_sensor(0x002A, 0x0278);  //0x20000278 
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //                  REG_TC_GP_EnablePreview 0001 REG_TC_GP_EnablePreview
S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //           REG_TC_GP_EnablePreviewChanged 0001 REG_TC_GP_EnablePreviewChanged
////////////////////////////////////////////////////////////////
// Automatically written by Setfile Rule Check function.
// Date: 2013-06-27 12:37:03 
//WRITE #REG_TC_GP_InvokeReadOTPData 0001
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////
// Automatically written by Setfile Rule Check function.
// Date: 2014-03-13 10:34:24 
//sFCFC2000
//s02C20001
////////////////////////////////////////////////////////////////  
    
}


/*************************************************************************
* FUNCTION
*    S5K5EAYXReadShutter
*
* DESCRIPTION
*    This function Read Shutter from sensor
*
* PARAMETERS
*    Shutter: integration time
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint32 S5K5EAYXReadShutter(void)
{
	kal_uint32 Shutter_lowIndex=0,Shutter_highIndex=0,Shutter=0;
	
	S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002E,0x2340);
	Shutter_lowIndex=S5K5EAYX_read_cmos_sensor(0x0F12);

	S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002E,0x2342);
	Shutter_highIndex=S5K5EAYX_read_cmos_sensor(0x0F12);

	Shutter = (Shutter_highIndex<<4)|Shutter_lowIndex;
    //SENSORDB("[5EA]: S5K5EAYXReadShutter shutter-org =%d \n",Shutter);

    //shutter valud divide 400 to get exposure time(ms), just get a ratio here;
	Shutter=Shutter/S5K5EA_READ_SHUTTER_RATIO;
	if(Shutter <1)
		Shutter=1;
		
    SENSORDB("[5EA]: S5K5EAYXReadShutter shutter =%d \n",Shutter);

	return Shutter;
}

/*************************************************************************
* FUNCTION
*    S5K5EAYXReadGain
*
* DESCRIPTION
*    This function get gain from sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    Gain: base on 0x40
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint32 S5K5EAYXReadGain(void)
{
	kal_uint32 Reg=0 ;

	S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002E,0x2144);
	Reg=S5K5EAYX_read_cmos_sensor(0x0F12);	
    //SENSORDB("[5EA]:S5K5EAYXReadGain gain-org =%d \n",Reg);

    //sensor gain base is 256;
	Reg=Reg/2;
	if(Reg<1)
	{
		Reg=1;
	}
    SENSORDB("[5EA]: S5K5EAYXReadGain gain =%d \n",Reg);

	return Reg; 
}



static kal_uint32 S5K5EAYXReadAwb_RGain(void)
{

	kal_uint32 RGain=0 ;
	
	S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002E,0x214c);
	RGain=S5K5EAYX_read_cmos_sensor(0x0F12);	
	//SENSORDB("[5EA]: S5K5EAYXReadAwb_RGain RGain-org =%d \n",RGain);

    //sensor gain base is 1024;
    RGain=RGain/8;

	if(RGain<1)
		RGain=1;
	
	SENSORDB("[5EA]: S5K5EAYXReadAwb_RGain RGain =%d \n",RGain);
	return RGain;
}

static kal_uint32 S5K5EAYXReadAwb_BGain(void)
{

	kal_uint32 BGain=0 ;
	
	S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002E,0x2150);
	BGain=S5K5EAYX_read_cmos_sensor(0x0F12);	
	//SENSORDB("[5EA]: S5K5EAYXReadAwb_RGain BGain-org =%d \n",BGain);

    //sensor gain base is 1024;
    BGain=BGain/8;

	if(BGain<1)
		BGain=1;
	
	SENSORDB("[5EA]: S5K5EAYXReadAwb_RGain BGain =%d \n",BGain);
	return BGain;
}



/*************************************************************************
* FUNCTION
*    S5K5EAYXGetEvAwbRef
*
* DESCRIPTION
*    This function get sensor Ev/Awb (EV05/EV13) for auto scene detect
*
* PARAMETERS
*    Ref
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void S5K5EAYXGetEvAwbRef(PSENSOR_AE_AWB_REF_STRUCT Ref)//checked in lab
{
    Ref->SensorAERef.AeRefLV05Shutter = 9000; 
    Ref->SensorAERef.AeRefLV05Gain = 732; /*  128 base */
    Ref->SensorAERef.AeRefLV13Shutter = 238;
    Ref->SensorAERef.AeRefLV13Gain = 128; /*  128 base */
    Ref->SensorAwbGainRef.AwbRefD65Rgain = 172; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefD65Bgain = 142; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFRgain = 159; /* 128 base */
    Ref->SensorAwbGainRef.AwbRefCWFBgain = 256;/* 128 base */
}
/*************************************************************************
* FUNCTION
*    S5K5EAYXGetCurAeAwbInfo
*
* DESCRIPTION
*    This function get sensor cur Ae/Awb for auto scene detect
*
* PARAMETERS
*    Info
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void S5K5EAYXGetCurAeAwbInfo(PSENSOR_AE_AWB_CUR_STRUCT Info)
{
    Info->SensorAECur.AeCurShutter = S5K5EAYX_ReadShutter();//S5K5EAYXReadShutter();
    Info->SensorAECur.AeCurGain = S5K5EAYX_ReadGain();//S5K5EAYXReadGain(); /* 128 base */

	Info->SensorAwbGainCur.AwbCurRgain=S5K5EAYXReadAwb_RGain; /* 128 base */
    Info->SensorAwbGainCur.AwbCurBgain = S5K5EAYXReadAwb_BGain; /* 128 base */
}


static void S5K5EAYX_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{ 	
    //<2014/5/8-37593-joubert.she, [5503][DRV] modify max-num-focus-areas parameter
    *pFeatureReturnPara32 = 0;//1   
	//>2014/5/8-37593-joubert.she
    SENSORDB("[5EA]: S5K5EAYX_Get_AF_Max_Num_Focus_Areas: *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}

static void S5K5EAYX_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{ 	
    
    *pFeatureReturnPara32 = 0;    
    SENSORDB("[5EA]: S5K5EAYX_Get_AE_Max_Num_Metering_Areas: *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);	
}


void S5K5EAYX_CaptureCfg0_FixFps(UINT16 FixFps)
{
	UINT16 ExposureTime =0;

	if(FixFps<5||FixFps>15)
		return;

    ExposureTime = 10000/FixFps;
	
	SENSORDB("[5EA]:S5K5EAYX_CaptureCfg0_FixFps\r\n");
	//Capture config[0] (org :15~7.5fps)==>fixfps
	S5K5EAYX_write_cmos_sensor(0x002A, 0x03D2);  //REG_CapConfigControls

	S5K5EAYX_write_cmos_sensor(0x0F12, S5K5EAYX_CAP_AEAWB_OPEN);  //AE AWB open	for zsd		

	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //REG_0TC_CCFG_usWidth 2560																			  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //REG_0TC_CCFG_usHeight	1920		 
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //REG_0TC_CCFG_Format	JPEG	
	
	S5K5EAYX_write_cmos_sensor(0x002A, 0x03DE); 																										  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //REG_0TC_CCFG_OutClkPerPix88																				  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //REG_0TC_CCFG_uBpp88			
	
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);   //REG_0TC_PCFG_PVIMask	
	
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //REG_0TC_CCFG_OIFMask 					  //JPEG8																
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0810);  //REG_0TC_CCFG_usJpegPacketSize																			  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //REG_0TC_CCFG_usJpegTotalPackets	 //SPOOF															  

	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //REG_0TC_CCFG_uClockInd																							

	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //REG_0TC_CCFG_usFrTimeType					0: Dynamic, 1:Not Accurate, 2: Fixed								
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //REG_0TC_CCFG_FrRateQualityType		0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)	  

	S5K5EAYX_write_cmos_sensor(0x0F12, ExposureTime);  //REG_0TC_CCFG_usMaxFrTimeMsecMult10 10fps 															  
	S5K5EAYX_write_cmos_sensor(0x0F12, ExposureTime);  //REG_0TC_CCFG_usMinFrTimeMsecMult10 15fps 															  
}


void S5K5EAYX_CaptureCfg0_FixFps_Restore(void)
{
	SENSORDB("[5EA]:S5K5EAYX_CaptureCfg0_FixFps_Restore\r\n");
	//Capture config[0] (org :15~7.5fps)
	S5K5EAYX_write_cmos_sensor(0x002A, 0x03D2);  //REG_CapConfigControls

	S5K5EAYX_write_cmos_sensor(0x0F12, S5K5EAYX_CAP_AEAWB_OPEN);  //AE AWB unlock for zsd	

	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A00);  //REG_0TC_CCFG_usWidth 2560																			  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0780);  //REG_0TC_CCFG_usHeight	1920		 
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0005);  //REG_0TC_CCFG_Format	JPEG	
	
	S5K5EAYX_write_cmos_sensor(0x002A, 0x03DE); 																										  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);  //REG_0TC_CCFG_OutClkPerPix88																				  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0300);  //REG_0TC_CCFG_uBpp88			
	
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);   //REG_0TC_PCFG_PVIMask	
	
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);  //REG_0TC_CCFG_OIFMask 					  //JPEG8																
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0810);  //REG_0TC_CCFG_usJpegPacketSize																			  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //REG_0TC_CCFG_usJpegTotalPackets	 //SPOOF															  

	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //REG_0TC_CCFG_uClockInd																							

	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);  //REG_0TC_CCFG_usFrTimeType					0: Dynamic, 1:Not Accurate, 2: Fixed								
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0002);  //REG_0TC_CCFG_FrRateQualityType		0: Dynamic 1: BEST FrameRate(Binning), 2: BEST QUALITY(No Binning)	  

	S5K5EAYX_write_cmos_sensor(0x0F12, 0x0535);  //REG_0TC_CCFG_usMaxFrTimeMsecMult10 10fps 															  
	S5K5EAYX_write_cmos_sensor(0x0F12, 0x029A);  //REG_0TC_CCFG_usMinFrTimeMsecMult10 15fps 															  
}


void S5K5EAYX_CaptureCfg_Close_AEAWB(bool status)
{
	   //Capture config[0] 15~7.5fps
	S5K5EAYX_write_cmos_sensor(0x002A, 0x03D2);  //REG_CapConfigControls
	if(status == S5K5EAYX_CAP_AEAWB_CLOSE)
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //AE AWB close		
	else
	    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //AE AWB open			

	//Capture config[1] 5~15fps
	S5K5EAYX_write_cmos_sensor(0x002A, 0x03FE); 	// REG_CapConfigControls_1_ 																		 
	if(status == S5K5EAYX_CAP_AEAWB_CLOSE)
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //AE AWB close		
	else
	    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);  //AE AWB open			

}


static void S5K5EAYX_Mode_Config(kal_bool NightModeEnable)
{
	SENSORDB("[5EA]:S5K5EAYX_Mode_Config: nightmode=%d\r\n", NightModeEnable);
    kal_uint8 ConfigIndex =0,readout_index=0;

	spin_lock(&s5k5eayx_drv_lock);
	S5K5EAYX_CurrentStatus.iNightMode = NightModeEnable;
	spin_unlock(&s5k5eayx_drv_lock);

    if((s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_PREVIEW)||
		(s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_VIDEO))
	{
	    if(NightModeEnable)
	    {
	       if(S5K5EAYX_video_mode == KAL_FALSE)
		   	 ConfigIndex = 1;//night  preview mode 5~30fps
		   else
			 ConfigIndex = 3;//night  video  mode 15fps

		}
		else
		{
			if(S5K5EAYX_video_mode == KAL_FALSE)
			  ConfigIndex = 0;//normal  preview mode 10~30fps
			else
			  ConfigIndex = 2;//normal  video mode 30fps
		}
		SENSORDB("[5EA]:S5K5EAYX_Mode_Config(preview or video): ConfigIndex=%d\r\n", ConfigIndex);

		S5K5EAYX_write_cmos_sensor(0xFCFC,0xd000);
		S5K5EAYX_write_cmos_sensor(0x0028,0x2000);

	    S5K5EAYX_write_cmos_sensor(0x002A, 0x02A0);   
		//night :5-30fps Index 1
	    //normal :10~30fps Index 0
	    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000+ConfigIndex);  //REG_TC_GP_ActivePrevConfig 
		S5K5EAYX_write_cmos_sensor(0x002A, 0x02A2); 									
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);    
	    S5K5EAYX_write_cmos_sensor(0x002A, 0x02A4);                                     
	    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);   //REG_TC_GP_PrevOpenAfterChange   
	    S5K5EAYX_write_cmos_sensor(0x002A, 0x0288);                                     
	    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);   //REG_TC_GP_NewConfigSync      
	    
	    S5K5EAYX_write_cmos_sensor(0x002A, 0x0278);                                     
	    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);   //REG_TC_GP_EnablePreview         
	    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);   //REG_TC_GP_EnablePreviewChanged
		mdelay(50);
	}
	else if((s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_CAPTURE)||
		(s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_ZSD))
	{
	    if(NightModeEnable)
			ConfigIndex = 1; //7.5~15 FPS
		else
			ConfigIndex = 0; //10~15 FPS
		
		SENSORDB(" [5EA]:S5K5EAYX_Mode_Config(capture or ZSD): ConfigIndex=%d\r\n", ConfigIndex);
		
		S5K5EAYX_write_cmos_sensor(0xFCFC, 0xd000);
		S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
		
		S5K5EAYX_write_cmos_sensor(0x002a, 0x02A8);
		S5K5EAYX_write_cmos_sensor(0x0f12, 0x0000+ConfigIndex);	
		
		S5K5EAYX_write_cmos_sensor(0x002A, 0x02AA);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
		
		S5K5EAYX_write_cmos_sensor(0x002A, 0x0288);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
		
		S5K5EAYX_write_cmos_sensor(0x002A, 0x027C);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
		S5K5EAYX_write_cmos_sensor(0x002A, 0x027E);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); 
		//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
        if(S5K5EAYX_preflash == KAL_TRUE)
            return;
		//>2014/5/2-37049-joubert.she, 
		mdelay(50);
	}
}


kal_bool S5K5EAYX_CheckHDRstatus(void)
{
    if((S5K5EAYX_CurrentStatus.iSceneMode == SCENE_MODE_HDR)
		&&(s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_CAPTURE))
    {
		return KAL_TRUE;
	}
	else 
		return KAL_FALSE;
}

void S5K5EAYXHdr_Part1(void)
{
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
	//read org shutter/gain
    kal_uint32 shutter=0,gain=0;

    //shutter= S5K5EAYX_ReadShutter();
    S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,0x2340);
    shutter  = S5K5EAYX_read_cmos_sensor(0x0F12);
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,0x2342);
    shutter |= (S5K5EAYX_read_cmos_sensor(0x0F12) << 16);
    // reg is in terms of 1000/400 us
    shutter = shutter * 5 / 2; //us
    gain= S5K5EAYX_ReadGain();//S5K5EAYXReadGain()*2;


    spin_lock(&s5k5eayx_drv_lock);
    S5K5EAYX_CurrentStatus.iShutter= shutter;
    S5K5EAYX_CurrentStatus.iGain= gain;
    spin_unlock(&s5k5eayx_drv_lock);
    
     SENSORDB("[5EA]: S5K5EAYXHdr_Part1:S5K5EAYX_CurrentStatus.iShutter = %d,S5K5EAYX_CurrentStatus.iGain = %d\r\n",  
	 	       S5K5EAYX_CurrentStatus.iShutter,S5K5EAYX_CurrentStatus.iGain);	
//>2014/5/2-37049-joubert.she, 
}

void S5K5EAYXHdr_Part2(UINT16 para)
{

	kal_uint32 HDRShutter =0,HDRGain=0;

    //multiply 100 could get exposure time(ms) for writing to register;
    //use a ratio here ,corresponding to ReadShutter();
	HDRShutter = S5K5EAYX_CurrentStatus.iShutter*S5K5EA_SET_SHUTTER_RATIO;
	HDRGain = S5K5EAYX_CurrentStatus.iGain*S5K5EA_SET_GAIN_RATIO;

	switch (para)
	{
	   case AE_EV_COMP_20:
       case AE_EV_COMP_10:
		   HDRGain = HDRGain<<1;
           HDRShutter = HDRShutter<<1;
           SENSORDB("[5EA]:[S5K5EAYX] HDR AE+20\n");
		 	break;
	   case AE_EV_COMP_00:
           SENSORDB("[5EA]:[S5K5EAYX] HDR AE00\n");
		 	break;
	   case AE_EV_COMP_n10:
	   case AE_EV_COMP_n20:
		   HDRGain = HDRGain >> 1;
           HDRShutter = HDRShutter >> 1;
           SENSORDB("[5EA]:[S5K5EAYX] HDR AE-20\n");
		 break;
	   default:
		 break; 
	}

    //shutter is limited by fps,200ms*100=>0x4E20
    //Total 10x gain, 10*256=>0x0A00
	HDRShutter = (HDRShutter<0x4E20)? HDRShutter:0x4E20;
	HDRGain = (HDRGain<0x0A00)? HDRGain:0x0A00;

	SENSORDB("[5EA]: S5K5EAYXHdr_Part2:HDRShutter = %d,HDRGain = %d\r\n",HDRShutter,HDRGain);   

	//Write shutter
    S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0779);
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x04E8);
	S5K5EAYX_write_cmos_sensor(0x0F12,HDRShutter & 0x0000ffff);
	S5K5EAYX_write_cmos_sensor(0x002A,0x04EA);
	S5K5EAYX_write_cmos_sensor(0x0F12,HDRShutter & 0xffff0000);
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x04EC);
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);
	
    //Write Gain
    S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0779);
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x0504);
	S5K5EAYX_write_cmos_sensor(0x0F12,HDRGain);
	S5K5EAYX_write_cmos_sensor(0x002A,0x0506);
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);

	spin_lock(&s5k5eayx_drv_lock);
	S5K5EAYX_CurrentStatus.HDRFixed_AE_Done=KAL_TRUE;
	spin_unlock(&s5k5eayx_drv_lock);

    //debug mode
	//SENSORDB(" [5EA]:part2 S5K5EAYXHdr_Part2 debug:HDRShutter = %d,HDRGain = %d\r\n",HDRShutter,HDRGain);   
    //mdelay(500);
	//S5K5EAYXReadShutter();
	//S5K5EAYXReadGain();
}


void S5K5EAYXHdr_Part3()
{
	SENSORDB("[5EA]:S5K5EAYXHdr_Part3:\r\n");   
    S5K5EAYX_CurrentStatus.HDRFixed_AE_Done=KAL_FALSE;

    //open auto AE
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);
	S5K5EAYX_write_cmos_sensor(0x0F12,0x077F);
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
	S5K5EAYX_get_exposure_gain();
/*        
	spin_lock(&s5k5eayx_drv_lock);
	S5K5EAYX_CurrentStatus.iShutter= 0;
	S5K5EAYX_CurrentStatus.iGain= 0;
	spin_unlock(&s5k5eayx_drv_lock);
*/	
//>2014/5/2-37049-joubert.she
}



/*************************************************************************
* FUNCTION
*	S5K5EAOpen
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
 
UINT32 S5K5EAYXOpen(void)
{
	kal_uint16 sensor_id=0;
  
    SENSORDB("[5EA]:S5K5EAYXOpen :\r\n");
    
    S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,0x0000);//id register
	sensor_id = S5K5EAYX_read_cmos_sensor(0x0F12);
	SENSORDB("[5EA]:Read Sensor ID = %x\n", sensor_id); 
	
    if (sensor_id != S5K5EAYX_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;

	S5K5EAYX_InitialPara();
    S5K5EAYX_Init_Setting();

	return ERROR_NONE;
}	/* S5K5EAYXOpen() */


UINT32 S5K5EAYXGetSensorID(UINT32 *sensorID)
{
	int  retry = 2; 
    SENSORDB("[5EA]: S5K5EAYXGetSensorID \n");
	
	do {
	    S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);
	    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
	    S5K5EAYX_write_cmos_sensor(0x002E,0x0000);//id register
		*sensorID = S5K5EAYX_read_cmos_sensor(0x0F12);	
		
		if (*sensorID == S5K5EAYX_SENSOR_ID)
			break; 
		SENSORDB("[5EA]:Read Sensor ID Fail = %x\n", *sensorID); 
		retry--; 
	} while (retry > 0);

	if (*sensorID != S5K5EAYX_SENSOR_ID) {
		*sensorID = 0xFFFFFFFF; 
		return ERROR_SENSOR_CONNECT_FAIL;
	}
    
	return ERROR_NONE;
}	/* S5K5EAYXGetSensorID() */


/*************************************************************************
* FUNCTION
*	S5K5EAYXClose
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K5EAYXClose(void)
{

	return ERROR_NONE;
}	


static void S5K5EAYX_HVMirror(kal_uint8 image_mirror)
{
/********************************************************
Preview:Mirror: 0x02d0 bit[0],Flip :    0x02d0 bit[1]
Capture:Mirror: 0x02d2 bit[0],Flip :    0x02d2 bit[1]
*********************************************************/

    SENSORDB("[5EA]:[Enter]:Mirror = image_mirror %d \r\n",image_mirror);
	S5K5EAYX_write_cmos_sensor(0xFCFC,0xd000);
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);

        //<20140520-39313-PengXu,[5503][Camera]Fix issue for panoramic mode when landscape
        #if  defined( ARIMA_LO1_HW )
            image_mirror=IMAGE_HV_MIRROR;
        #else
            image_mirror=IMAGE_NORMAL;
        #endif
	switch (image_mirror) {
	//switch (0) {
        //>20140520-39313-PengXu
	case IMAGE_HV_MIRROR:
		SENSORDB("case 3 mirror +flip");
			S5K5EAYX_write_cmos_sensor(0x002A,	0x030C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_0TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_0TC_PCFG_uCaptureMirror


			S5K5EAYX_write_cmos_sensor(0x002A,	0x033C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_1TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_1TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x036C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_2TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_2TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x039C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_3TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_3TC_PCFG_uCaptureMirror

			S5K5EAYX_write_cmos_sensor(0x002A,	0x03CC);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_4TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0000);	//#REG_4TC_PCFG_uCaptureMirror

            break;
		case IMAGE_H_MIRROR:
			S5K5EAYX_write_cmos_sensor(0x002A,	0x030C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_0TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_0TC_PCFG_uCaptureMirror


			S5K5EAYX_write_cmos_sensor(0x002A,	0x033C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_1TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_1TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x036C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_2TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_2TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x039C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_3TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_3TC_PCFG_uCaptureMirror

			S5K5EAYX_write_cmos_sensor(0x002A,	0x03CC);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_4TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0001);	//#REG_4TC_PCFG_uCaptureMirror
            break;
		case IMAGE_V_MIRROR:
			S5K5EAYX_write_cmos_sensor(0x002A,	0x030C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_0TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_0TC_PCFG_uCaptureMirror


			S5K5EAYX_write_cmos_sensor(0x002A,	0x033C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_1TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_1TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x036C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_2TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_2TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x039C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_3TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_3TC_PCFG_uCaptureMirror

			S5K5EAYX_write_cmos_sensor(0x002A,	0x03CC);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_4TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0002);	//#REG_4TC_PCFG_uCaptureMirror
            break;
		case IMAGE_NORMAL:
			SENSORDB("case 0 normal");
			S5K5EAYX_write_cmos_sensor(0x002A,	0x030C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_0TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_0TC_PCFG_uCaptureMirror


			S5K5EAYX_write_cmos_sensor(0x002A,	0x033C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_1TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_1TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x036C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_2TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_2TC_PCFG_uCaptureMirror
			
			S5K5EAYX_write_cmos_sensor(0x002A,	0x039C);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_3TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_3TC_PCFG_uCaptureMirror

			S5K5EAYX_write_cmos_sensor(0x002A,	0x03CC);
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_4TC_PCFG_uPrevMirror
			S5K5EAYX_write_cmos_sensor(0x0F12,	0x0003);	//#REG_4TC_PCFG_uCaptureMirror
            break;
	}
       
    S5K5EAYX_write_cmos_sensor(0x002A, 0x027A);                                     
    S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);   //REG_TC_GP_EnablePreviewChanged
    
}
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
static void S5K5EAYX_ReadFlashData(void)
{
    kal_uint32 i;
    
    SENSORDB("[5EA]S5K5EAYX_ReadFlashData\n ");	
    //<2014/5/5-37322-joubert.she, [Lo1/Lo2][DRV] update flashlight brightness driver from norman.
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000); 
    S5K5EAYX_write_cmos_sensor(0x002E,0x2500); 
    for(i=0x2500;i<0x2566;i+=2)
	//>2014/5/5-37322-joubert.she
        SENSORDB("[5EA] [0x%04x,0x%04x]\n",i,S5K5EAYX_read_cmos_sensor(0x0F12) );

}
//>2014/5/2-37049-joubert.she, 
/*************************************************************************
* FUNCTION
*	S5K5EAYXPreview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static UINT32 S5K5EAYXPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{	
     
    SENSORDB("[5EA]:S5K5EAYX preview func:\n ");	

    if(S5K5EAYX_A_COLOR_TEMP == KAL_TRUE )
    {
        spin_lock(&s5k5eayx_drv_lock);
        S5K5EAYX_A_COLOR_TEMP = KAL_FALSE;
        spin_unlock(&s5k5eayx_drv_lock);
        S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x0c42); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 
        
        S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x1208); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 

    }
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
    if(S5K5EAYX_preflash==KAL_TRUE)
    {
        S5K5EAYX_ReadFlashData();
        SENSORDB("[5EA]: s5k5eayx_Flash_End\n");
        S5K5EAYX_write_cmos_sensor(0x0028,0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x0854); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0002); //Fast AE off
        S5K5EAYX_write_cmos_sensor(0x002a,0x0C44); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 
        S5K5EAYX_write_cmos_sensor(0x002a,0x04C0); 
        S5K5EAYX_write_cmos_sensor(0x0F12,0x0000); 
        spin_lock(&s5k5eayx_drv_lock);
        S5K5EAYX_preflash=KAL_FALSE;
        spin_unlock(&s5k5eayx_drv_lock);
	//<2014/6/12-39517-joubert.she, [Lo1/Lo2][DRV] update main camera driver 
        S5K5EAYX_write_cmos_sensor(0x0028, 0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002A, 0x0C52);
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0050); //0020: for preflash case , 0100: for main case
      //>2014/6/12-39517-joubert.she 
    }
//>2014/5/2-37049-joubert.she
    spin_lock(&s5k5eayx_drv_lock);
    S5K5EAYX_video_mode = KAL_FALSE;
    s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_PREVIEW;
    spin_unlock(&s5k5eayx_drv_lock);

    S5K5EAYX_Mode_Config(S5K5EAYX_CurrentStatus.iNightMode);
	S5K5EAYX_HVMirror(sensor_config_data->SensorImageMirror);

	image_window->ExposureWindowWidth = S5K5EAYX_IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight = S5K5EAYX_IMAGE_SENSOR_PV_HEIGHT;
	
	memcpy(&S5K5EAYXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    
    return ERROR_NONE; 
}	/* S5K5EAYXPreview */



static UINT32 S5K5EAYXVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{	
     
	SENSORDB("[5EA]:S5K5EAYX video func:\n ");	

	spin_lock(&s5k5eayx_drv_lock);
	S5K5EAYX_video_mode = KAL_TRUE;
	s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_VIDEO;
	spin_unlock(&s5k5eayx_drv_lock);
	
    S5K5EAYX_Mode_Config(S5K5EAYX_CurrentStatus.iNightMode);
	S5K5EAYX_HVMirror(sensor_config_data->SensorImageMirror);

	image_window->ExposureWindowWidth = S5K5EAYX_IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight = S5K5EAYX_IMAGE_SENSOR_PV_HEIGHT;
	
	memcpy(&S5K5EAYXSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    
    return ERROR_NONE; 
}	/* S5K5EAYXVideo */

void TestByNorman(void)
{
    kal_uint16 Status_3A=0,Status_3A1=0;
    kal_uint32 i;
    
    for(i=0x0000;i<0xffff;i+=2)
        {
    S5K5EAYX_write_cmos_sensor(0xFCFC,0xd000);
    S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002E,i);
    Status_3A=S5K5EAYX_read_cmos_sensor(0x0F12); //Index number of active capture configuration //Normal capture// 
    SENSORDB("norman 0x%04x=0x%04x\n",i,Status_3A);

}
}

UINT32 S5K5EAYXCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

    SENSORDB("[5EA]:S5K5EAYXCapture func:\n ");	

    spin_lock(&s5k5eayx_drv_lock);
    S5K5EAYX_video_mode = KAL_FALSE;
    spin_unlock(&s5k5eayx_drv_lock);
		

    if((s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_PREVIEW)
        ||(s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_VIDEO))
    {
        spin_lock(&s5k5eayx_drv_lock);
        s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_CAPTURE;//for mode config
        spin_unlock(&s5k5eayx_drv_lock);
        //<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
        if(S5K5EAYX_preflash == KAL_FALSE)
        {
            S5K5EAYX_CaptureCfg_Close_AEAWB(S5K5EAYX_CAP_AEAWB_CLOSE);
            S5K5EAYX_Mode_Config(S5K5EAYX_CurrentStatus.iNightMode);
        }
    }
    S5K5EAYX_HVMirror(sensor_config_data->SensorImageMirror);

    image_window->GrabStartX = S5K5EAYX_FULL_X_START;
    image_window->GrabStartY = S5K5EAYX_FULL_Y_START;
    image_window->ExposureWindowWidth = S5K5EAYX_IMAGE_SENSOR_FULL_WIDTH ;
    image_window->ExposureWindowHeight = S5K5EAYX_IMAGE_SENSOR_FULL_HEIGHT;

    spin_lock(&s5k5eayx_drv_lock);
    s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_CAPTURE;//for zsd HDR
    spin_unlock(&s5k5eayx_drv_lock);

    if(S5K5EAYX_CheckHDRstatus())//capture & ZSD mode could get in
    {
        S5K5EAYXHdr_Part1();
    }else
        S5K5EAYX_get_exposure_gain();
    //>2014/5/2-37049-joubert.she,
    return ERROR_NONE; 
}


UINT32 S5K5EAYXZsd(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

		SENSORDB("[5EA]: S5K5EAYXZsd func:\n ");	  
		
		spin_lock(&s5k5eayx_drv_lock);
		S5K5EAYX_video_mode = KAL_FALSE;
		s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_ZSD;
		spin_unlock(&s5k5eayx_drv_lock);

		S5K5EAYX_CaptureCfg_Close_AEAWB(S5K5EAYX_CAP_AEAWB_OPEN);
		S5K5EAYX_Mode_Config(S5K5EAYX_CurrentStatus.iNightMode);

		S5K5EAYX_HVMirror(sensor_config_data->SensorImageMirror);

		image_window->GrabStartX = S5K5EAYX_FULL_X_START;
		image_window->GrabStartY = S5K5EAYX_FULL_Y_START;
		image_window->ExposureWindowWidth = S5K5EAYX_IMAGE_SENSOR_FULL_WIDTH;
		image_window->ExposureWindowHeight = S5K5EAYX_IMAGE_SENSOR_FULL_HEIGHT;
		
		return ERROR_NONE; 
}



UINT32 S5K5EAYXGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("[5EA]:S5K5EAYX get Resolution func\n");
	
	pSensorResolution->SensorFullWidth=S5K5EAYX_IMAGE_SENSOR_FULL_WIDTH;  
	pSensorResolution->SensorFullHeight=S5K5EAYX_IMAGE_SENSOR_FULL_HEIGHT;
	
	pSensorResolution->SensorPreviewWidth=S5K5EAYX_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=S5K5EAYX_IMAGE_SENSOR_PV_HEIGHT;
	
	pSensorResolution->SensorVideoWidth=S5K5EAYX_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight=S5K5EAYX_IMAGE_SENSOR_VIDEO_HEIGHT;
	
	return ERROR_NONE;
}	

UINT32 S5K5EAYXGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    	SENSORDB("[5EA]: S5K5EAYXGetInfo func\n");
   
        pSensorInfo->SensorCameraPreviewFrameRate=30;
        pSensorInfo->SensorVideoFrameRate=30;
        pSensorInfo->SensorStillCaptureFrameRate=15;
        pSensorInfo->SensorWebCamCaptureFrameRate=15;
        pSensorInfo->SensorResetActiveHigh=FALSE;
        pSensorInfo->SensorResetDelayCount=4;
        
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;

        pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
        pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
        pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
        pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
        pSensorInfo->SensorInterruptDelayLines = 1; 
		
		pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
    if(S5K5EAYX_preflash == KAL_FALSE)
        pSensorInfo->CaptureDelayFrame = 2; 
    else
        pSensorInfo->CaptureDelayFrame = 0; 
//>2014/5/2-37049-joubert.she, 
        pSensorInfo->PreviewDelayFrame = 2; 
        pSensorInfo->VideoDelayFrame = 2; 
        pSensorInfo->SensorMasterClockSwitch = 0; 
        pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;   

		pSensorInfo->YUVAwbDelayFrame = 2; 
		pSensorInfo->YUVEffectDelayFrame = 2;   
	
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			
			pSensorInfo->SensorGrabStartX = S5K5EAYX_PV_X_START; 
			pSensorInfo->SensorGrabStartY = S5K5EAYX_PV_Y_START;  
			
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = S5K5EAYX_FULL_X_START; 
            pSensorInfo->SensorGrabStartY = S5K5EAYX_FULL_Y_START;			
		    
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;			
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
			
		break;
		default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount=3;
            pSensorInfo->SensorDataLatchCount=2;
            pSensorInfo->SensorGrabStartX = S5K5EAYX_PV_X_START; 
            pSensorInfo->SensorGrabStartY = S5K5EAYX_PV_Y_START;  			
		break;
	}
	
	return ERROR_NONE;
}	


/*************************************************************************
* FUNCTION
*	S5K5EAYX_set_param_effect
*
* DESCRIPTION
*	effect setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL S5K5EAYX_set_param_effect(UINT16 para)
{

   SENSORDB("[5EA]:S5K5EAYX set_param_effect func:para = %d,MEFFECT_OFF =%d\n",para,MEFFECT_OFF);
   switch (para)
	{
		case MEFFECT_OFF:
		{
			S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					 
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					 
			S5K5EAYX_write_cmos_sensor(0x002A,0x0276);					 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);	//REG_TC_GP_SpecialEffects 	
        }
	        break;
		case MEFFECT_NEGATIVE:
			S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					 
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					 
			S5K5EAYX_write_cmos_sensor(0x002A,0x0276);					 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0002);	//REG_TC_GP_SpecialEffects 	
			break;
		case MEFFECT_SEPIA:
		{
			S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					 
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					 
			S5K5EAYX_write_cmos_sensor(0x002A,0x0276);					 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0004);	//REG_TC_GP_SpecialEffects 	
        }	
			break;  
		case MEFFECT_SEPIABLUE:
		{
			S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					 
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					 
			S5K5EAYX_write_cmos_sensor(0x002A,0x0276);					 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0007);	//REG_TC_GP_SpecialEffects 	
	    }     
			break;        
		case MEFFECT_SEPIAGREEN:		
		{
			S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					 
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					 
			S5K5EAYX_write_cmos_sensor(0x002A,0x0276);					 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0008);	//REG_TC_GP_SpecialEffects 	
	    }     
			break;        
		case MEFFECT_MONO:			
		{
			S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					 
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					 
			S5K5EAYX_write_cmos_sensor(0x002A,0x0276);					 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);	//REG_TC_GP_SpecialEffects 	
        }
			break;

		default:
			return KAL_FALSE;
	}

	return KAL_TRUE;

} /* S5K5EAYX_set_param_effect */

UINT32 S5K5EAYXControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

   spin_lock(&s5k5eayx_drv_lock);
   S5K5EAYXCurrentScenarioId = ScenarioId;
   spin_unlock(&s5k5eayx_drv_lock);
   
   SENSORDB("[5EA]: S5K5EAYXControl:ScenarioId = %d \n",S5K5EAYXCurrentScenarioId);
   
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:// 0
			 S5K5EAYXPreview(pImageWindow, pSensorConfigData);
			 break;

		case MSDK_SCENARIO_ID_VIDEO_PREVIEW: // 2
			 S5K5EAYXVideo(pImageWindow, pSensorConfigData);
			 break;
			 
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:// 1
			 S5K5EAYXCapture(pImageWindow, pSensorConfigData);
			 break;			
		case MSDK_SCENARIO_ID_CAMERA_ZSD:// 4
			 S5K5EAYXZsd(pImageWindow, pSensorConfigData);
			 break;
		default:
		     break; 
	}

	return ERROR_NONE;
}	/* S5K5EAYXControl() */


/*************************************************************************
* FUNCTION
*	S5K5EAYX_set_param_wb
*
* DESCRIPTION
*	wb setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL S5K5EAYX_set_param_wb(UINT16 para)
{
	
    //This sensor need more time to balance AWB, 
    //we suggest higher fps or drop some frame to avoid garbage color when preview initial
	kal_uint16 Status_3A=0;
//TestByNorman();
	SENSORDB("[5EA]:S5K5EAYX set_param_wb func:para = %d,AWB_MODE_AUTO =%d\n",para,AWB_MODE_AUTO);
	while(Status_3A==0)
	{
		S5K5EAYX_write_cmos_sensor(0xFCFC,0xd000);
		S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
		S5K5EAYX_write_cmos_sensor(0x002E,0x051C);
		Status_3A=S5K5EAYX_read_cmos_sensor(0x0F12); //Index number of active capture configuration //Normal capture// 
		mdelay(10);
	}

    spin_lock(&s5k5eayx_drv_lock);
    S5K5EAYX_CurrentStatus.iWB = para;
    spin_unlock(&s5k5eayx_drv_lock);
	switch (para)
	{            
		case AWB_MODE_AUTO:
			{
			Status_3A = (Status_3A | 0x8); // Enable AWB
			S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x051C);
			S5K5EAYX_write_cmos_sensor(0x0F12, Status_3A);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x04F6); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
            }   
		    break;
		case AWB_MODE_OFF:
	         {
			 Status_3A = (Status_3A & 0xFFF7); // Disable AWB
			 S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
			 S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			 S5K5EAYX_write_cmos_sensor(0x002A, 0x051C);
			 S5K5EAYX_write_cmos_sensor(0x0F12, Status_3A);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x04F6); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
		     }
			 break;
			 //<2014/6/12-39517-joubert.she, [Lo1/Lo2][DRV] update main camera driver 
		case AWB_MODE_CLOUDY_DAYLIGHT:
			{
			Status_3A = (Status_3A & 0xFFF7); // Disable AWB
			S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x051C);
			S5K5EAYX_write_cmos_sensor(0x0F12, Status_3A);
			//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0777);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x04F6); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0565); //Reg_sf_user_Rgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_RgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); //Reg_sf_user_Ggain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_GgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0620); //Reg_sf_user_Bgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_BgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RGBgainChanged
	        }			   
		    break;
		case AWB_MODE_DAYLIGHT:
		    {
			Status_3A = (Status_3A & 0xFFF7); // Disable AWB
			S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x051C);
			S5K5EAYX_write_cmos_sensor(0x0F12, Status_3A);
			//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0777);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x04F6); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0507); //Reg_sf_user_Rgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_RgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); //Reg_sf_user_Ggain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_GgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x05A4); //Reg_sf_user_Bgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_BgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RGBgainChanged
            }      
		    break;
		case AWB_MODE_INCANDESCENT:	
		    {
			Status_3A = (Status_3A & 0xFFF7); // Disable AWB
			S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x051C);
			S5K5EAYX_write_cmos_sensor(0x0F12, Status_3A);
			//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0777);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x04F6); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0457); //Reg_sf_user_Rgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_RgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); //Reg_sf_user_Ggain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_GgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x08B2); //Reg_sf_user_Bgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_BgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RGBgainChanged
            }		
		    break;  
		case AWB_MODE_FLUORESCENT:
		    {
			Status_3A = (Status_3A & 0xFFF7); // Disable AWB
			S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x051C);
			S5K5EAYX_write_cmos_sensor(0x0F12, Status_3A);
			//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0777);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x04F6); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x044A); //Reg_sf_user_Rgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_RgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); //Reg_sf_user_Ggain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_GgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0802); //Reg_sf_user_Bgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_BgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RGBgainChanged
            }	
		    break;  
			//>2014/6/12-39517-joubert.she
		case AWB_MODE_TUNGSTEN:
		    {
			Status_3A = (Status_3A & 0xFFF7); // Disable AWB
			S5K5EAYX_write_cmos_sensor(0xFCFC, 0xD000);
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x051C);
			S5K5EAYX_write_cmos_sensor(0x0F12, Status_3A);
			//S5K5EAYX_write_cmos_sensor(0x0F12, 0x0777);
			S5K5EAYX_write_cmos_sensor(0x002A, 0x04F6); 
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0400); //Reg_sf_user_Rgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_RgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0485); //Reg_sf_user_Ggain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_GgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0A68); //Reg_sf_user_Bgain
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); //Reg_sf_user_BgainChanged
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); //Reg_sf_user_RGBgainChanged
            }	
		    break;  			    
		default:
			return FALSE;
	}
        //SENSORDB("Status_3A = 0x%x\n",Status_3A);
	return TRUE;
} /* S5K5EAYX_set_param_wb */


/*************************************************************************
* FUNCTION
*	S5K5EAYX_set_param_banding
*
* DESCRIPTION
*	banding setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL S5K5EAYX_set_param_banding(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX set_param_banding func:para = %d,AE_FLICKER_MODE_50HZ=%d\n",para,AE_FLICKER_MODE_50HZ);
	kal_uint16 Status_3A=0;
	while(Status_3A==0)
	{
		S5K5EAYX_write_cmos_sensor(0xFCFC,0xd000);
		S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
		S5K5EAYX_write_cmos_sensor(0x002E,0x051C);
		Status_3A=S5K5EAYX_read_cmos_sensor(0x0F12); //Index number of active capture configuration //Normal capture// 
		mdelay(10);
	}
	switch (para)
	{
		case AE_FLICKER_MODE_AUTO:
		default:
	   		 {
			Status_3A = (Status_3A | 0x0020); // enable auto-flicker
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);   
			S5K5EAYX_write_cmos_sensor(0x002a, 0x051C);   
			S5K5EAYX_write_cmos_sensor(0x0f12, Status_3A);   
	    	}
			break;
		case AE_FLICKER_MODE_60HZ:
	   		 {
			Status_3A = (Status_3A & 0xFFDF); // disable auto-flicker
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);   
			S5K5EAYX_write_cmos_sensor(0x002a, 0x051C);   
			S5K5EAYX_write_cmos_sensor(0x0f12, Status_3A);   
			S5K5EAYX_write_cmos_sensor(0x002a, 0x0512);   
			S5K5EAYX_write_cmos_sensor(0x0f12, 0x0002);   
			S5K5EAYX_write_cmos_sensor(0x0f12, 0x0001); 
	    	}
			break;

		case AE_FLICKER_MODE_50HZ:
				{
				Status_3A = (Status_3A & 0xFFDF); // disable auto-flicker
				S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);   
				S5K5EAYX_write_cmos_sensor(0x002a, 0x051C);   
				S5K5EAYX_write_cmos_sensor(0x0f12, Status_3A);	 
				//S5K5EAYX_write_cmos_sensor(0x0f12, 0x075f);	
				S5K5EAYX_write_cmos_sensor(0x002a, 0x0512);   
				S5K5EAYX_write_cmos_sensor(0x0f12, 0x0001);   
				S5K5EAYX_write_cmos_sensor(0x0f12, 0x0001);
				}
				break;
	}
	return KAL_TRUE;
} /* S5K5EAYX_set_param_banding */


/*************************************************************************
* FUNCTION
*	S5K5EAYX_set_param_exposure
*
* DESCRIPTION
*	exposure setting.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
BOOL S5K5EAYX_set_param_exposure(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX set_param_exposure func:para = %d\n",para);

    if(S5K5EAYX_CheckHDRstatus())
    {
		S5K5EAYXHdr_Part2(para);
        return true;
	}

	
	switch (para)
	{
       case AE_EV_COMP_n20://-2
		   S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					
		   S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					
		   S5K5EAYX_write_cmos_sensor(0x002A,0x0274);					
		   S5K5EAYX_write_cmos_sensor(0x0F12,0x0040);  	
           break;
	   case AE_EV_COMP_n10://-1
		   S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					
		   S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					
		   S5K5EAYX_write_cmos_sensor(0x002A,0x0274);					
		   S5K5EAYX_write_cmos_sensor(0x0F12,0x0080);  	
		   break;	 
	   case AE_EV_COMP_00:	// +0 EV
		   S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					
		   S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					
		   S5K5EAYX_write_cmos_sensor(0x002A,0x0274);					
		   S5K5EAYX_write_cmos_sensor(0x0F12,0x0100);  	
		   break;	 
	   case AE_EV_COMP_10://+1
		  S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					
		  S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					
		  S5K5EAYX_write_cmos_sensor(0x002A,0x0274);					
		  S5K5EAYX_write_cmos_sensor(0x0F12,0x0170);  	
		   break;
	   case AE_EV_COMP_20://+2
		   S5K5EAYX_write_cmos_sensor(0xFCFC,0xD000);					
		   S5K5EAYX_write_cmos_sensor(0x0028,0x2000);					
		   S5K5EAYX_write_cmos_sensor(0x002A,0x0274);					
		   S5K5EAYX_write_cmos_sensor(0x0F12,0x0200);  	
		   break;	 
	   default:
		   return FALSE;

	}
	return TRUE;
	
} 


#if 1//scene mode
void S5K5EAYX_SceneMode_SetDefault(void)
{
	SENSORDB("[5EA]:S5K5EAYX_SceneMode_SetDefault\n");
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);//								
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);//								
	S5K5EAYX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits	
																				
	S5K5EAYX_write_cmos_sensor(0x002A,0x02C8);//								
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0005);//  //REG_TC_AF_AfCmd 			
	S5K5EAYX_write_cmos_sensor(0x002A,0x050C);//								
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0002);//  //REG_SF_USER_IsoType 		
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0100);//  //REG_SF_USER_IsoVal			  
	S5K5EAYX_write_cmos_sensor(0x002A,0x026A);//								
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserBrightness		
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserContrast 		  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSaturation		
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSharpBlur	
	
	//S5K5EAYX_write_cmos_sensor(0x002A,0x0276);//								
	//S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_GP_SpecialEffects;don't open this ,MTK
	//<2014/5/6-37370-joubert.she, [Lo1/Lo2][DRV] update main camera driver from norman.
	S5K5EAYX_write_cmos_sensor(0x002A,0x24BC);//								
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_0_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_1_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_2_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_3_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_4_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_5_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_6_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_7_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_8_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_9_	 0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_10_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_11_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_12_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0202);//  //Mon_AE_WeightTbl_16_13_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0202);//  //Mon_AE_WeightTbl_16_14_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_15_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0201);//  //Mon_AE_WeightTbl_16_16_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0403);//  //Mon_AE_WeightTbl_16_17_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0304);//  //Mon_AE_WeightTbl_16_18_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0102);//  //Mon_AE_WeightTbl_16_19_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0201);//  //Mon_AE_WeightTbl_16_20_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0303);//  //Mon_AE_WeightTbl_16_21_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0303);//  //Mon_AE_WeightTbl_16_22_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0102);//  //Mon_AE_WeightTbl_16_23_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0201);//  //Mon_AE_WeightTbl_16_24_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0202);//  //Mon_AE_WeightTbl_16_25_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0202);//  //Mon_AE_WeightTbl_16_26_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0102);//  //Mon_AE_WeightTbl_16_27_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_28_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_29_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_30_  0101	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);//  //Mon_AE_WeightTbl_16_31_  0101	  
}


void S5K5EAYX_SceneMode_PORTRAIT(void)
{
	SENSORDB("[5EA]:S5K5EAYX_SceneMode_PORTRAIT\n");
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);// 
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x077F);// //REG_TC_DBG_AutoAlgEnBits
	S5K5EAYX_write_cmos_sensor(0x002A,0x02C8);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0005);// //REG_TC_AF_AfCmd	  
	S5K5EAYX_write_cmos_sensor(0x002A,0x050C);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0002);// //REG_SF_USER_IsoType	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0100);// //REG_SF_USER_IsoVal		  
	S5K5EAYX_write_cmos_sensor(0x002A,0x026A);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_UserBrightness
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_UserContrast
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_UserSaturation
	S5K5EAYX_write_cmos_sensor(0x0F12,0xFF41);// //REG_TC_UserSharpBlur   
	
	//S5K5EAYX_write_cmos_sensor(0x002A,0x0276);//
	//S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// ////REG_TC_UserSharpBlur;
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x24BC);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_0_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_1_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_2_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_3_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_4_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_5_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_6_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_7_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_8_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_9_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_10_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_11_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_12_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0f01);// //Mon_AE_WeightTbl_16_13_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x010f);// //Mon_AE_WeightTbl_16_14_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_15_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_16_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0f01);// //Mon_AE_WeightTbl_16_17_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x010f);// //Mon_AE_WeightTbl_16_18_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_19_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_20_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_21_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_22_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_23_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_24_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_25_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_26_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_27_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_28_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_29_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_30_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_31_	
}


void S5K5EAYX_SceneMode_LANDSCAPE(void)
{
	SENSORDB("[5EA]:S5K5EAYX_SceneMode_LANDSCAPE\n");
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);//
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);//
	S5K5EAYX_write_cmos_sensor(0x0F12,0x077F);// //REG_TC_DBG_AutoAlgEnBits   
	S5K5EAYX_write_cmos_sensor(0x002A,0x02C8);//  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0005);// // REG_TC_AF_AfCmd   
	S5K5EAYX_write_cmos_sensor(0x002A,0x050C);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0002);// //REG_SF_USER_IsoType	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0100);// //REG_SF_USER_IsoVal 
	S5K5EAYX_write_cmos_sensor(0x002A,0x026A);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_UserBrightness  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0040);// //REG_TC_UserContrast		
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0040);// //REG_TC_UserSaturation  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0040);// //REG_TC_UserSharpBlur   
	
	//S5K5EAYX_write_cmos_sensor(0x002A,0x0276);// 
	//S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_GP_SpecialEffects   
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x24BC);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_0_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_1_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_2_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_3_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_4_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_5_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_6_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_7_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_8_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_9_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_10_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_11_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_12_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_13_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_14_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_15_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_16_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_17_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_18_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_19_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_20_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_21_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_22_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_23_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_24_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_25_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_26_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_27_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_28_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_29_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_30_   
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_31_   
	
}

void S5K5EAYX_SceneMode_SUNSET(void)
{
	SENSORDB("[5EA]:S5K5EAYX_SceneMode_SUNSET\n");
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);// 
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0777);// //REG_TC_DBG_AutoAlgEnBits
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x02C8);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0005);// //REG_TC_AF_AfCmd

	S5K5EAYX_write_cmos_sensor(0x002A,0x04F6);
	S5K5EAYX_write_cmos_sensor(0x0F12,0x05E0);//REG_SF_USER_Rgain
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);//REG_SF_USER_RgainChanged
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0400);//REG_SF_USER_Ggain
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);//REG_SF_USER_GgainChanged
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0530); //REG_SF_USER_Bgain
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);//REG_SF_USER_BgainChanged 
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x050C);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0002);// //REG_SF_USER_IsoType
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0100);// //REG_SF_USER_IsoVal
	S5K5EAYX_write_cmos_sensor(0x002A,0x026A);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_UserBrightness
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_UserContrast
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_UserSaturation
	S5K5EAYX_write_cmos_sensor(0x0F12,0xFF41);// //REG_TC_UserSharpBlur
	
	//S5K5EAYX_write_cmos_sensor(0x002A,0x0276);//
	//S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);// //REG_TC_GP_SpecialEffects
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x24BC);// 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_0_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_1_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_2_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_3_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_4_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_5_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_6_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_7_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_8_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_9_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_10_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_11_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_12_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0f01);// //Mon_AE_WeightTbl_16_13_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x010f);// //Mon_AE_WeightTbl_16_14_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_15_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_16_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0f01);// //Mon_AE_WeightTbl_16_17_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x010f);// //Mon_AE_WeightTbl_16_18_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_19_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_20_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_21_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_22_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_23_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_24_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_25_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_26_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_27_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_28_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_29_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_30_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// //Mon_AE_WeightTbl_16_31_	 
}

void S5K5EAYX_SceneMode_SPORTS(void)
{
	SENSORDB("[5EA]:S5K5EAYX_SceneMode_SPORTS\n");
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);//
	S5K5EAYX_write_cmos_sensor(0x002A,0x051C);//
	S5K5EAYX_write_cmos_sensor(0x0F12,0x077F);//  //REG_TC_DBG_AutoAlgEnBits
	S5K5EAYX_write_cmos_sensor(0x02CA,0x02CA);//  //
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //
	S5K5EAYX_write_cmos_sensor(0x002A,0x02C8);//  //
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0004);//  //REG_TC_AF_AfCmd
	S5K5EAYX_write_cmos_sensor(0x002A,0x050C);//  //
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0002);//  //REG_SF_USER_IsoType
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0400);//  //REG_SF_USER_IsoVal
	S5K5EAYX_write_cmos_sensor(0x002A,0x026A);//  //
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserBrightness
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserContrast
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSaturation
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_UserSharpBlur
	
	//S5K5EAYX_write_cmos_sensor(0x002A,0x0276);//  //
	//S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);//  //REG_TC_GP_SpecialEffects
	
	S5K5EAYX_write_cmos_sensor(0x002A,0x24BC);//  //
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_0_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_1_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_2_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_3_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_4_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_5_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_6_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_7_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_8_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_9_	  
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_10_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_11_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_12_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0f01);// // Mon_AE_WeightTbl_16_13_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x010f);// // Mon_AE_WeightTbl_16_14_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_15_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_16_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0f01);// // Mon_AE_WeightTbl_16_17_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x010f);// // Mon_AE_WeightTbl_16_18_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_19_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_20_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_21_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_22_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_23_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_24_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_25_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_26_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_27_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_28_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_29_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_30_ 
	S5K5EAYX_write_cmos_sensor(0x0F12,0x0101);// // Mon_AE_WeightTbl_16_31_ 
}
#endif

void S5K5EAYX_set_scene_mode(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_scene_mode func:para = %d\n",para);
	//SENSORDB(" [5EA]:S5K5EAYX S5K5EAYX_set_scene_mode func:SCENE_MODE_NIGHTSCENE = %d\n",SCENE_MODE_NIGHTSCENE);
	//SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_scene_mode func:SCENE_MODE_OFF = %d\n",SCENE_MODE_OFF);
	//SENSORDB(" [5EA]:S5K5EAYX S5K5EAYX_set_scene_mode func:SCENE_MODE_HDR \n",SCENE_MODE_HDR);

	S5K5EAYX_CurrentStatus.iSceneMode = para;

	S5K5EAYX_SceneMode_SetDefault();
	S5K5EAYX_CaptureCfg0_FixFps_Restore();
	
    switch (para)
    { 

		case SCENE_MODE_NIGHTSCENE:
             S5K5EAYX_Mode_Config(S5K5EZYX_NightMode_On); 
			 break;
        case SCENE_MODE_PORTRAIT:
			 S5K5EAYX_SceneMode_PORTRAIT();
             S5K5EAYX_Mode_Config(S5K5EZYX_NightMode_Off); 
             break;
        case SCENE_MODE_LANDSCAPE:
			 S5K5EAYX_SceneMode_LANDSCAPE();
             S5K5EAYX_Mode_Config(S5K5EZYX_NightMode_Off); 
             break;
        case SCENE_MODE_SUNSET:
			 S5K5EAYX_SceneMode_SUNSET();
             S5K5EAYX_Mode_Config(S5K5EZYX_NightMode_Off); 
             break;
        case SCENE_MODE_SPORTS://control fps
        	 S5K5EAYX_SceneMode_SPORTS();
			 
			 if((s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_PREVIEW)
			 	||(s5k5eayx_sensor_mode == S5K5EAYX_SENSORMODE_VIDEO))
			 {
			     //active video normal mode ,30fps
				 S5K5EAYX_write_cmos_sensor(0x002A,0x02A0);// //
				 S5K5EAYX_write_cmos_sensor(0x0F12,0x0002);//  //REG_TC_GP_ActivePrevConfig    
				 S5K5EAYX_write_cmos_sensor(0x0F12,0x0001);//  //REG_TC_GP_PrevConfigChanged   
			 }
			 else
			 {
				S5K5EAYX_CaptureCfg0_FixFps(15);

				//activeCaptureConfig 0
				 S5K5EAYX_write_cmos_sensor(0xFCFC, 0xd000);
				 S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
				 
				 S5K5EAYX_write_cmos_sensor(0x002a, 0x02A8);
				 S5K5EAYX_write_cmos_sensor(0x0f12, 0x0000+0); 
				 
				 S5K5EAYX_write_cmos_sensor(0x002A, 0x02AA);
				 S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
				 
				 S5K5EAYX_write_cmos_sensor(0x002A, 0x0288);
				 S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
				 
				 S5K5EAYX_write_cmos_sensor(0x002A, 0x027C);
				 S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
				 S5K5EAYX_write_cmos_sensor(0x002A, 0x027E);
				 S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); 
			 }
			 
             break;
        case SCENE_MODE_HDR:
			if(S5K5EAYX_CurrentStatus.HDRFixed_AE_Done==KAL_TRUE)
			{
				S5K5EAYXHdr_Part3();
			}
			break;
        case SCENE_MODE_OFF://AUTO
        default:
			S5K5EAYX_Mode_Config(S5K5EZYX_NightMode_Off); 
			break;
    }

	return;
}


UINT32 S5K5EAYXYUVSetVideoMode(UINT16 u2FrameRate)
{
	
	SENSORDB("[5EA]:S5K5EAYX Set Video Mode:FrameRate= %d\n",u2FrameRate);
    //set fps in FUNC S5K5EAYX_Mode_Config(xxxx);
		
    return ERROR_NONE;
}

static void S5K5EAYX_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
	UINT32 state = 0;

	S5K5EAYX_write_cmos_sensor(0xFCFC,0xd000);
	S5K5EAYX_write_cmos_sensor(0x002C,0x2000);
	S5K5EAYX_write_cmos_sensor(0x002E,0x1F92);
	state = S5K5EAYX_read_cmos_sensor(0x0F12);

    if(0x00 == state)
    {
        *pFeatureReturnPara32 = SENSOR_AF_IDLE;
	
    }
    else if(0x01 == state)//focusing
    {
        *pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
		
    }
    else if(0x02 == state)//success
    {
        *pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
			
    }
    else
    {
        *pFeatureReturnPara32 = SENSOR_AF_ERROR;
			   
    }
   //SENSORDB("[5EA]:S5K5EAYX_Get_AF_Status = 0x%x\n", state); 

}	

static void S5K5EAYX_Get_AF_Inf(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}


static void S5K5EAYX_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 255;
}

static void S5K5EAYX_Single_Focus(void)
{
   
    S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002a,0x02C8);
    S5K5EAYX_write_cmos_sensor(0x0f12,0x0005); // single AF 

    SENSORDB("[5EA]:S5K5EAYX_Single_Focus\r\n");
    return;
}	
static void S5K5EAYX_Constant_Focus(void)
{

	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
    S5K5EAYX_write_cmos_sensor(0x002a,0x02C8);
    S5K5EAYX_write_cmos_sensor(0x0f12,0x0006);
    SENSORDB("[5EA]:S5K5EAYX_Constant_Focus \r\n");
    return;
}

 static void S5K5EAYX_Cancel_Focus(void)
 {
	S5K5EAYX_write_cmos_sensor(0x0FCFC,0xD000);	
	S5K5EAYX_write_cmos_sensor(0x0028,0x2000);

	S5K5EAYX_write_cmos_sensor(0x002a,0x02C8);
	S5K5EAYX_write_cmos_sensor(0x0f12,0x0001);
	
 	
    SENSORDB("[5EA]:S5K5EAYX_Cancel_Focus \r\n");
 	return;
 }

void S5K5EAYX_set_saturation(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_saturation func:para = %d\n",para);

    switch (para)
    {
        case ISP_SAT_HIGH:
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_saturation func:para = ISP_SAT_HIGH\n");
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026E);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0070);
             break;
			 
        case ISP_SAT_LOW:
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_saturation func:para = ISP_SAT_LOW\n");
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026E);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF81);
             break;
			 
        case ISP_SAT_MIDDLE:
        default:
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_saturation func:para = ISP_SAT_MIDDLE\n");
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026E);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
             break;
    }
     return;
}


void S5K5EAYX_set_contrast(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_contrast func:para = %d\n",para);

	switch (para)
	{
		case ISP_CONTRAST_LOW:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026C);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0xFF81);
			 break;
			 
		case ISP_CONTRAST_HIGH:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026C);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x007F);
			 break;
			 
		case ISP_CONTRAST_MIDDLE:
		default:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026C);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
			 break;
	}
	
	return;
}


void S5K5EAYX_set_brightness(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_brightness func:para = %d\n",para);
    switch (para)
    {
        case ISP_BRIGHT_LOW:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026A);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0xFFC0);
             break;
        case ISP_BRIGHT_HIGH:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026A);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0040);
             break;
        case ISP_BRIGHT_MIDDLE:
        default:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x026A);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
             break;
    }

    return;
}

void S5K5EAYX_set_iso(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_iso func:para = %d\n",para);
    switch (para)
    {
        case AE_ISO_100:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x050C);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0100);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x098E);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);
             break;
        case AE_ISO_200:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x050C);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0340);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x098E);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);
             break;
        case AE_ISO_400:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x050C);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0710);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x098E);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);
             break;
        default:
        case AE_ISO_AUTO:
			S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x050C);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
			S5K5EAYX_write_cmos_sensor(0x002a, 0x098E);
			S5K5EAYX_write_cmos_sensor(0x0F12, 0x0200);
             break;
    }
    return;
}

//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
void S5K5EAYX_AE_Lock_Enable(UINT16 iPara)
{
    SENSORDB("[5EA] S5K5EAYX_AE_Lock_Enable Function \n");
    if(iPara == AE_MODE_OFF)//turn off AE
    {
        SENSORDB("[5EA] S5K5EAYX AE Lock\n");
        S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
        S5K5EAYX_write_cmos_sensor(0x002a, 0x21CE);
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
    }
    else//turn on AE
    {
        SENSORDB("[5EA] S5K5EAYX AE unLock\n");
        S5K5EAYX_write_cmos_sensor(0x0028, 0x2000);
        S5K5EAYX_write_cmos_sensor(0x002a, 0x21CE);
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
    }
    ReadNB();
}

void S5K5EAYX_AWB_Lock_Enable(UINT16 iPara)
{
    SENSORDB("[5EA] S5K5EAYX_AWB_Lock_Enable Function \n");
    if(iPara == AWB_MODE_OFF)//turn off AWB
    {
        SENSORDB("[5EA] S5K5EAYX AWB Lock\n");
        S5K5EAYX_write_cmos_sensor(0x0028, 0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002A, 0x21D6); 
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000); 
    }
    else//turn on AWB
    {
        SENSORDB("[5EA] S5K5EAYX AWB unLock\n");
        S5K5EAYX_write_cmos_sensor(0x0028, 0x2000); 
        S5K5EAYX_write_cmos_sensor(0x002A, 0x21D6); 
        S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001); 
    }
}
//>2014/5/2-37049-joubert.she,
void S5K5EAYX_set_hue(UINT16 para)
{
	SENSORDB("[5EA]:S5K5EAYX S5K5EAYX_set_hue func:para = %d,ISP_HUE_MIDDLE=%d\n",para,ISP_HUE_MIDDLE);
	switch (para)
	{
		case ISP_HUE_LOW:
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A,0x21DC);  //Hue1
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);

			S5K5EAYX_write_cmos_sensor(0x002A,0x2206); 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x01BB); 
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF35);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x00BB);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFFB8);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0122);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFE7D);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF70);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x007C);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0188);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x014E);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x003B);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF7A);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x00A3);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF7D);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x01C8);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF99);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x021E);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0103);			 
			break;
		case ISP_HUE_HIGH:
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A,0x21DC);	//Hue3
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0000);
			
			S5K5EAYX_write_cmos_sensor(0x002A,0x2206); 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x01E8); 
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0061);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF62);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFE6D);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0112);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFFD7);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x007C);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF70);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0188);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0072);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0133);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF5F);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x01C8);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFF7D);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x00A2);
			S5K5EAYX_write_cmos_sensor(0x0F12,0xFFB3);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x00CF);
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0238);
			 break;
		case ISP_HUE_MIDDLE:
		default:
			S5K5EAYX_write_cmos_sensor(0x0028,0x2000);
			S5K5EAYX_write_cmos_sensor(0x002A,0x21DC); //REG_TC_UserSaturation
			S5K5EAYX_write_cmos_sensor(0x0F12,0x0001); //Hue 2
			 break;

	}
	return;
}


UINT32 S5K5EAYXYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
        
    SENSORDB("[5EA]:S5K5EAYXYUVSensorSetting func:cmd = %d\n",iCmd);

	switch (iCmd) {

	case FID_COLOR_EFFECT:
           S5K5EAYX_set_param_effect(iPara);
	     break;

	case FID_AE_EV:	    	    
           S5K5EAYX_set_param_exposure(iPara);
	     break;

	case FID_SCENE_MODE:
		S5K5EAYX_set_scene_mode(iPara);
	     break; 	    
		 
	case FID_AWB_MODE:
           S5K5EAYX_set_param_wb(iPara);
	     break;
		 
	case FID_AE_FLICKER:	    	    	    
           S5K5EAYX_set_param_banding(iPara);
	     break;

    case FID_ISP_SAT:
        S5K5EAYX_set_saturation(iPara);
		break;

	case FID_ISP_CONTRAST:
		S5K5EAYX_set_contrast(iPara);
		break;
		 
	 case FID_ISP_BRIGHT:
		 S5K5EAYX_set_brightness(iPara);
		 break;

	case FID_AE_ISO:
		S5K5EAYX_set_iso(iPara);
		break;

	case FID_AE_SCENE_MODE:
        S5K5EAYX_AE_Lock_Enable(iPara);
		break;

	case FID_ISP_HUE:
		S5K5EAYX_set_hue(iPara);
		 break;
		 
	case FID_ZOOM_FACTOR:
			spin_lock(&s5k5eayx_drv_lock);
			zoom_factor = iPara; 
			spin_unlock(&s5k5eayx_drv_lock);
		break; 

	default:
	     break;
	}
	return TRUE;
}

void S5K5EAYX_Get_AEAWB_lock_info(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
    *pAElockRet32 = 1;
    *pAWBlockRet32 = 1;
    SENSORDB("GetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}


void S5K5EAYX_GetDelayInfo(UINT32 delayAddr)
{
    SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;
    pDelayInfo->InitDelay = 3;
    pDelayInfo->EffectDelay = 2;
    pDelayInfo->AwbDelay = 4;
    pDelayInfo->AFSwitchDelayFrame = 50;
}



void S5K5EA_GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 26;
    pExifInfo->AEISOSpeed = S5K5EAYX_CurrentStatus.iGain;
    pExifInfo->AWBMode = S5K5EAYX_CurrentStatus.iWB;
    pExifInfo->CapExposureTime = S5K5EAYX_CurrentStatus.iShutter;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = S5K5EAYX_CurrentStatus.iGain;
}	

UINT32 S5K5EAYX_SetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("[5EA]:[S5K5EAYX_SetTestPatternMode] Test pattern enable:%d\n", bEnable);

	if(bEnable) 
	{
		S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);
		S5K5EAYX_write_cmos_sensor(0x002a, 0x3100);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
	}
	else        
	{
		S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);
		S5K5EAYX_write_cmos_sensor(0x002a, 0x3100);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
	}
    return ERROR_NONE;

}

UINT32 S5K5EAYX_SetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate)
{
    kal_uint32 pclk;
    kal_int16 dummyLine;
    kal_uint16 lineLength,frameHeight;
#if 0
    SENSORDB("S5K5EAYX_SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
             pclk = S5K5EAYX_PREVIEW_PCLK;
             lineLength = S5K5EAYX_PV_PERIOD_PIXEL_NUMS;
             frameHeight = (10 * pclk)/frameRate/lineLength;
             dummyLine = frameHeight - S5K5EAYX_PV_PERIOD_LINE_NUMS;
             //s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_PREVIEW;
            //S5K5EAYX_SetDummy(0, dummyLine);
             break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             pclk = S5K5EAYX_VIDEO_PCLK;
             lineLength = S5K5EAYX_VIDEO_PERIOD_PIXEL_NUMS;
             frameHeight = (10 * pclk)/frameRate/lineLength;
             dummyLine = frameHeight - S5K5EAYX_VIDEO_PERIOD_LINE_NUMS;
             //s5k5eayx_sensor_mode = S5K5EAYX_SENSORMODE_VIDEO;
            // S5K5EAYX_SetDummy(0, dummyLine);
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             pclk = S5K5EAYX_CAPTURE_PCLK;
             lineLength = S5K5EAYX_CAP_PERIOD_PIXEL_NUMS;
             frameHeight = (10 * pclk)/frameRate/lineLength;
             dummyLine = frameHeight - S5K5EAYX_CAP_PERIOD_LINE_NUMS;
             //s5k5eayx_sensor_mode= S5K5EAYX_SENSORMODE_CAPTURE;
             //S5K5EAYX_SetDummy(0, dummyLine);
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
             break;
        default:
             break;
    }
#endif
  return ERROR_NONE;
}


UINT32 S5K5EAYX_GetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
    switch (scenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
             *pframeRate = 300;
             break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        case MSDK_SCENARIO_ID_CAMERA_ZSD:
             *pframeRate = 150;
             break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
             *pframeRate = 300;
             break;
        default:
          break;
    }

  return ERROR_NONE;
}


void S5K5EAYX_AutoTestCmd(UINT32 *cmd,UINT32 *para)
{
	SENSORDB("[S5K5EAYX]enter S5K5EAYX_AutoTestCmd function:\n ");
	switch(*cmd)
	{
		case YUV_AUTOTEST_SET_SHADDING:
			SENSORDB("YUV_AUTOTEST_SET_SHADDING:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAMMA:
			SENSORDB("YUV_AUTOTEST_SET_GAMMA:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_AE:
			SENSORDB("YUV_AUTOTEST_SET_AE:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_SHUTTER:
			SENSORDB("YUV_AUTOTEST_SET_SHUTTER:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_SET_GAIN:
			SENSORDB("YUV_AUTOTEST_SET_GAIN:para=%d\n",*para);
			break;
		case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			*para=8228;
			break;
		default:
			SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
			break;	
	}
	SENSORDB("[S5K5EAYX]exit OV5645MIPI_AutoTestCmd function:\n ");
#if 0 //factory mode funtion back up
		//Gamma enable
		S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);
		S5K5EAYX_write_cmos_sensor(0x002a, 0x6700);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0000);
	
		//Gamma disable
		S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);
		S5K5EAYX_write_cmos_sensor(0x002a, 0x6700);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);
		//Shading enable
		S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);
		S5K5EAYX_write_cmos_sensor(0x002a, 0x3400); 
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0006);
	
		//Shading disable
		S5K5EAYX_write_cmos_sensor(0x0028, 0xD000);
		S5K5EAYX_write_cmos_sensor(0x002a, 0x3400);
		S5K5EAYX_write_cmos_sensor(0x0F12, 0x0007);
#endif
	
}


void S5K5EAYX_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          S5K5EAYX_AE_Lock_Enable(AE_MODE_OFF);
      break;
	  
      case SENSOR_3A_AE_UNLOCK:
          S5K5EAYX_AE_Lock_Enable(AE_MODE_AUTO);
      break;
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
      case SENSOR_3A_AWB_LOCK:
          S5K5EAYX_AWB_Lock_Enable(AWB_MODE_OFF);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          S5K5EAYX_AWB_Lock_Enable(AWB_MODE_AUTO);
      break;
//>2014/5/2-37049-joubert.she, 
      default:
      	break;
   }
   return;
}


static UINT32 S5K5EAYX_FOCUS_Move_to(UINT32 a_u2MovePosition)
{
//
}

static void S5K5EAYX_AF_Set_Window(UINT32 zone_addr)
{
	  UINT32 FD_XS = 4;
	  UINT32 FD_YS = 3;  
	  UINT32 x0, y0, x1, y1;
	  UINT32* zone = (UINT32*)zone_addr;
	  UINT32 width_ratio, height_ratio, start_x, start_y;  
	  static UINT32 LastPosition=400;
  
	  x0 = *zone;
	  y0 = *(zone + 1);
	  x1 = *(zone + 2);
	  y1 = *(zone + 3); 	  
	  FD_XS = *(zone + 4);
	  FD_YS = *(zone + 5);
  
  	SENSORDB(" [5EA]:[S5K5EAYX]enter S5K5EAYX_AF_Set_Window function:\n ");

    //Do CAF
	if(x0==x1)
		return;

	//check if we really need to update AF window
	if((LastPosition<x0 && LastPosition+S5K5EA_FAF_TOLERANCE>x0)||
		(LastPosition>x0 && x0+S5K5EA_FAF_TOLERANCE>LastPosition))
	{
		LastPosition = x0;
		return;
	}

	LastPosition = x0;
    if(s5k5eayx_sensor_mode != S5K5EAYX_SENSORMODE_ZSD)
	{
		//width_ratio =  1024*(x1 - x0)/FD_XS;
		//height_ratio = 1024*(y1 - y0)/FD_YS;
        width_ratio = S5K5EA_PRV_RATIO_WIDTH;
		height_ratio = S5K5EA_PRV_RATIO_HEIGHT;

		start_x = x0*S5K5EA_PRE_AFRangeWidth/FD_XS;
		start_y = y0*S5K5EA_PRE_AFRangeHeight/FD_YS;
		
        if(start_x+S5K5EA_PRV_RATIO_WIDTH > S5K5EA_PRE_AFRangeWidth)
			start_x = S5K5EA_PRE_AFRangeWidth-S5K5EA_PRV_RATIO_WIDTH;
		if(start_y+S5K5EA_PRV_RATIO_HEIGHT>S5K5EA_PRE_AFRangeHeight)
			start_y = S5K5EA_PRE_AFRangeHeight-S5K5EA_PRV_RATIO_HEIGHT;
	}
	else
	{
	      //ZSD
		  //width_ratio =  2048*(x1 - x0)/FD_XS;
		  //height_ratio = 2048*(y1 - y0)/FD_YS;
		  width_ratio = S5K5EA_CAP_RATIO_WIDTH;
		  height_ratio = S5K5EA_CAP_RATIO_HEIGHT;

		  start_x = x0*S5K5EA_CAP_AFRangeWidth/FD_XS;
		  start_y = y0*S5K5EA_CAP_AFRangeHeight/FD_YS;

		if(start_x+S5K5EA_CAP_RATIO_WIDTH > S5K5EA_CAP_AFRangeWidth)
			start_x = S5K5EA_CAP_AFRangeWidth-S5K5EA_CAP_RATIO_WIDTH;
		if(start_y+S5K5EA_CAP_RATIO_HEIGHT>S5K5EA_CAP_AFRangeHeight)
			start_y = S5K5EA_CAP_AFRangeHeight-S5K5EA_CAP_RATIO_HEIGHT;

	}
  
	  SENSORDB("[5EA]:af x0=%d,y0=%d,x1=%d,y1=%d,FD_XS=%d,FD_YS=%d\n",
					 x0, y0, x1, y1, FD_XS, FD_YS);		 
	  SENSORDB("[5EA]:af start_x=%d,start_y=%d,width_ratio=%d,height_ratio=%d\n",
					 start_x, start_y, width_ratio, height_ratio);	
  
	  S5K5EAYX_write_cmos_sensor(0x002A,		 0x02D0);
	  S5K5EAYX_write_cmos_sensor(0x0F12,	  start_x); 		//#REG_TC_AF_FstWinStartX
	  S5K5EAYX_write_cmos_sensor(0x0F12,	  start_y); 		//#REG_TC_AF_FstWinStartY
	  S5K5EAYX_write_cmos_sensor(0x0F12,   width_ratio);		//#REG_TC_AF_FstWinSizeX
	  S5K5EAYX_write_cmos_sensor(0x0F12, height_ratio); 		//#REG_TC_AF_FstWinSizeY
	  S5K5EAYX_write_cmos_sensor(0x0F12,	  start_x); 		//#REG_TC_AF_ScndWinStartX
	  S5K5EAYX_write_cmos_sensor(0x0F12,	  start_y); 		//#REG_TC_AF_ScndWinStartY
	  S5K5EAYX_write_cmos_sensor(0x0F12,   width_ratio);		//#REG_TC_AF_ScndWinSizeX
	  S5K5EAYX_write_cmos_sensor(0x0F12, height_ratio); 		//#REG_TC_AF_ScndWinSizeY
	  S5K5EAYX_write_cmos_sensor(0x0F12, 0x0001);		  //#REG_TC_AF_WinSizesUpdated
      msleep(150);//need 1  fps time to update 
}


void S5K5EAYX_AE_Set_Window(UINT32 zone_addr)
{
//don't support
}

UINT32 S5K5EAYXFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 u2Temp = 0; 
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

   // SENSORDB(" [5EA]:S5K5EAYXFeatureControl,FeatureId=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=S5K5EAYX_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=S5K5EAYX_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		     break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(S5K5EAYXCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=S5K5EAYX_CAP_PERIOD_PIXEL_NUMS;
					*pFeatureReturnPara16=S5K5EAYX_CAP_PERIOD_LINE_NUMS;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=S5K5EAYX_VIDEO_PERIOD_PIXEL_NUMS;
					*pFeatureReturnPara16=S5K5EAYX_VIDEO_PERIOD_LINE_NUMS;
					*pFeatureParaLen=4;
					break;
			}
		     break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ://3003
			switch(S5K5EAYXCurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = S5K5EAYX_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = S5K5EAYX_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = S5K5EAYX_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = S5K5EAYX_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;
		case SENSOR_FEATURE_SET_ESHUTTER:
                    S5K5EAYX_SetShutter(*pFeatureData16);
		     break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			 S5K5EAYX_Mode_Config((BOOL) *pFeatureData16);
		     break;
		case SENSOR_FEATURE_SET_GAIN:
                    S5K5EAYX_SetGain(*pFeatureData16);
			 break; 
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		     break;
		case SENSOR_FEATURE_SET_REGISTER:
			 S5K5EAYX_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		     break;
		case SENSOR_FEATURE_GET_REGISTER:
			 pSensorRegData->RegData = S5K5EAYX_read_cmos_sensor(pSensorRegData->RegAddr);
			 break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			 memcpy(pSensorConfigData, &S5K5EAYXSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			 *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
		     break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
		     break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
	         *pFeatureReturnPara32++=0;
			 *pFeatureParaLen=4;
		     break; 
		
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		     break;

        case SENSOR_FEATURE_SET_TEST_PATTERN:
             S5K5EAYX_SetTestPatternMode((BOOL)*pFeatureData16);
             break;
			 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
             S5K5EAYXGetSensorID(pFeatureReturnPara32); 
            break;     

		case SENSOR_FEATURE_SET_YUV_CMD:
			 S5K5EAYXYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		     break;	
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		     S5K5EAYXYUVSetVideoMode(*pFeatureData16);
		     break; 

		case SENSOR_FEATURE_GET_EV_AWB_REF:
		    S5K5EAYXGetEvAwbRef(*pFeatureData32);
		    break;
		
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
		    S5K5EAYXGetCurAeAwbInfo(*pFeatureData32);
		    break;

		case SENSOR_FEATURE_AUTOTEST_CMD:
			SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
			S5K5EAYX_AutoTestCmd(*pFeatureData32,*(pFeatureData32+1));
			break;

		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            S5K5EAYX_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;

		case SENSOR_FEATURE_INITIALIZE_AF:   //3029   
			SENSORDB(" SENSOR_FEATURE_INITIALIZE_AF \n");
			break; 
			
		case SENSOR_FEATURE_MOVE_FOCUS_LENS://3031
			SENSORDB(" SENSOR_FEATURE_MOVE_FOCUS_LENS \n");
			S5K5EAYX_FOCUS_Move_to(*pFeatureData16);
			break;
			
        case SENSOR_FEATURE_GET_AF_STATUS://3032              
			SENSORDB(" SENSOR_FEATURE_GET_AF_STATUS \n");
            S5K5EAYX_Get_AF_Status(pFeatureReturnPara32);               
            *pFeatureParaLen=4;
            break;
			
		case SENSOR_FEATURE_GET_AF_INF: //3033
			SENSORDB(" SENSOR_FEATURE_GET_AF_INF \n");
			S5K5EAYX_Get_AF_Inf( pFeatureReturnPara32); 				  
			*pFeatureParaLen=4; 		   
			break;
			
		case SENSOR_FEATURE_GET_AF_MACRO: //3034
			SENSORDB(" SENSOR_FEATURE_GET_AF_MACRO \n");
			S5K5EAYX_Get_AF_Macro(pFeatureReturnPara32);			   
			*pFeatureParaLen=4; 		   
			break;	
			
		case SENSOR_FEATURE_CONSTANT_AF://3030
			SENSORDB(" SENSOR_FEATURE_CONSTANT_AF \n");
			S5K5EAYX_Constant_Focus(); 
			break;	
			
		case SENSOR_FEATURE_SET_AF_WINDOW://3041
			SENSORDB(" SENSOR_FEATURE_SET_AF_WINDOW \n");
			S5K5EAYX_AF_Set_Window(*pFeatureData32);
			 break;
			 
        case SENSOR_FEATURE_SINGLE_FOCUS_MODE://3039
			SENSORDB(" SENSOR_FEATURE_SINGLE_FOCUS_MODE \n");
            S5K5EAYX_Single_Focus(); 
            break;	
			
        case SENSOR_FEATURE_CANCEL_AF://3040
			SENSORDB(" SENSOR_FEATURE_CANCEL_AF \n");
            S5K5EAYX_Cancel_Focus();
            break;

        case SENSOR_FEATURE_SET_AE_WINDOW:
			 S5K5EAYX_AE_Set_Window(*pFeatureData32);
			 break;
			
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
            S5K5EAYX_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;        
        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
            S5K5EAYX_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);            
            *pFeatureParaLen=4;
            break;

		case SENSOR_FEATURE_GET_EXIF_INFO:
			S5K5EA_GetExifInfo(*pFeatureData32);
			break;
			
        case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
             S5K5EAYX_Get_AEAWB_lock_info((*pFeatureData32),*(pFeatureData32+1));

        case SENSOR_FEATURE_GET_DELAY_INFO:
             S5K5EAYX_GetDelayInfo(*pFeatureData32);
             break;

        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
             S5K5EAYX_SetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
             break;

        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
             S5K5EAYX_GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
             break;

		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=S5K5EAYX_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;

        /**********************Strobe Ctrl Start *******************************/
        case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
             S5K5EAYX_GetAEFlashlightInfo(*pFeatureData32);
             SENSORDB("[5EA] F_GET_AE_FLASHLIGHT_INFO: Not Support\n");
             break;

        case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
             S5K5EAYX_FlashTriggerCheck(pFeatureData32);
             SENSORDB("[5EA] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", pFeatureData32);
             break;

        case SENSOR_FEATURE_SET_FLASHLIGHT:
             SENSORDB("S5K45EAYX SENSOR_FEATURE_SET_FLASHLIGHT\n");
             break;
        /**********************Strobe Ctrl End *******************************/

		default:
			 break;			
	}

	return ERROR_NONE;
}	/* S5K5EAYXFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncS5K5EAYX=
{
	S5K5EAYXOpen,            
	S5K5EAYXGetInfo,          
	S5K5EAYXGetResolution,    
	S5K5EAYXFeatureControl,   
	S5K5EAYXControl,         
	S5K5EAYXClose           
};

UINT32 S5K5EAYX_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncS5K5EAYX;

	return ERROR_NONE;
}	/* SensorInit() */


