/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
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
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
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
#include <asm/atomic.h>
//#include <mach/mt6516_pll.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ov2675mipiyuv_Sensor.h"
#include "ov2675mipiyuv_Camera_Sensor_para.h"
#include "ov2675mipiyuv_CameraCustomized.h"

#define OV2675MIPIYUV_DEBUG
#ifdef OV2675MIPIYUV_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#define __SLT_DRV_OV3660_BURST_SHOTS__	//Debug: Brightless became lower and lower in burst shot mode
#ifdef __SLT_DRV_OV3660_BURST_SHOTS__	//wujinyou, 2011.11.21
static kal_uint8 preview_init_flag = 0;
#endif


static DEFINE_SPINLOCK(ov2675_drv_lock);

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
#define OV2675_MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,OV2675_MIPI_WRITE_ID)
#define OV2675_MIPI_write_cmos_sensor_2(addr, para, bytes) iWriteReg((u16) addr , (u32) para ,bytes,OV2675_MIPI_WRITE_ID)
kal_uint16 OV2675_MIPI_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV2675_MIPI_WRITE_ID);
    return get_byte;
}


/*******************************************************************************
* // Adapter for Winmo typedef 
********************************************************************************/

#define Sleep(ms) mdelay(ms)
#define RETAILMSG(x,...)
#define TEXT


/*******************************************************************************
* // End Adapter for Winmo typedef 
********************************************************************************/


#define	OV2675_MIPI_LIMIT_EXPOSURE_LINES				(1253)
#define	OV2675_MIPI_VIDEO_NORMALMODE_30FRAME_RATE       (30)
#define	OV2675_MIPI_VIDEO_NORMALMODE_FRAME_RATE         (15)
#define	OV2675_MIPI_VIDEO_NIGHTMODE_FRAME_RATE          (7.5)
#define BANDING50_30HZ
/* Global Valuable */

static kal_uint32 zoom_factor = 0; 
static kal_uint8 OV2675_MIPI_exposure_line_h = 0, OV2675_MIPI_exposure_line_l = 0,OV2675_MIPI_extra_exposure_line_h = 0, OV2675_MIPI_extra_exposure_line_l = 0;

static kal_bool OV2675_MIPI_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool OV2675_MIPI_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool OV2675_MIPI_sensor_cap_state = KAL_FALSE; //Preview or Capture
static kal_bool OV2675_MIPI_sensor_zsd_mode = KAL_FALSE; //Preview or Capture

static kal_uint16 OV2675_MIPI_dummy_pixels=0, OV2675_MIPI_dummy_lines=0;

static kal_uint16 OV2675_MIPI_exposure_lines=0, OV2675_MIPI_extra_exposure_lines = 0;


static kal_int8 OV2675_MIPI_DELAY_AFTER_PREVIEW = -1;

static kal_uint8 OV2675_MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;  //Wonder add

/****** OVT 6-18******/
static kal_uint16 OV2675_MIPI_Capture_Max_Gain16= 6*16;
static kal_uint16 OV2675_MIPI_Capture_Gain16=0 ;    
static kal_uint16 OV2675_MIPI_Capture_Shutter=0;
static kal_uint16 OV2675_MIPI_Capture_Extra_Lines=0;

static kal_uint16  OV2675_MIPI_PV_Dummy_Pixels =0,OV2675_MIPI_Capture_Dummy_Pixels =0, OV2675_MIPI_Capture_Dummy_Lines =0;
static kal_uint16  OV2675_MIPI_PV_Gain16 = 0;
static kal_uint16  OV2675_MIPI_PV_Shutter = 0;
static kal_uint16  OV2675_MIPI_PV_Extra_Lines = 0;
static kal_uint8 ModeChange=0;
static kal_uint8 firstmode=1;


kal_uint16 OV2675_MIPI_iOV2675_MIPI_Mode=0;
kal_uint32 OV2675_MIPI_capture_pclk_in_M=520,OV2675_MIPI_preview_pclk_in_M=390,OV2675_MIPI_PV_dummy_pixels=0,OV2675_MIPI_PV_dummy_lines=0,OV2675_MIPI_isp_master_clock=0;

static kal_uint32  OV2675_MIPI_sensor_pclk=390;
static kal_bool OV2675_MIPI_AWB_ENABLE = KAL_TRUE; 
static kal_bool OV2675_MIPI_AE_ENABLE = KAL_TRUE; 

static kal_uint32 Capture_Shutter = 0; 
static kal_uint32 Capture_Gain = 0; 

UINT8 OV2675_MIPI_PixelClockDivider=0;

//	camera_para.SENSOR.reg	SensorReg
MSDK_SENSOR_CONFIG_STRUCT OV2675SensorConfigData;
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
} OV2675_Status;
OV2675_Status OV2675_CurrentStatus;


void OV2675_MIPI_set_dummy(kal_uint16 pixels, kal_uint16 lines)
{

}    /* OV2675_MIPI_set_dummy */
kal_uint16 OV2675_MIPI_read_OV2675_MIPI_gain(void)
{
    kal_uint8 temp_reg1 , temp_reg2;
    kal_uint16 sensor_gain,iso;

    temp_reg1 = OV2675_MIPI_read_cmos_sensor(0x350A);		//Gain[8]
    temp_reg2 = OV2675_MIPI_read_cmos_sensor(0x350B);		//Gain[0~7]
    sensor_gain = ((temp_reg1 & 0x01)<< 8) | temp_reg2 ;      
    
    if(sensor_gain < 0x10) // 1x~2x gain
        iso=AE_ISO_100; 
    else if(sensor_gain < 0x30)// 2X~4x gain
        iso=AE_ISO_200;
    else if(sensor_gain < 0x70)// 4x~8x gain
        iso=AE_ISO_400;
    else if(sensor_gain < 0xF0)// 8x~16x gain
        iso=AE_ISO_800;
    else 
        iso=AE_ISO_1600;
      
    printk("OV2675_MIPI_read_OV2675_MIPI_gain sensor_gain=%x,iso=%d\n",sensor_gain,iso);
    return iso;
}  /* OV7695_MIPI_read_OV7695_MIPI_gain */
//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
kal_uint32 OV2675_MIPI_read_shutter(void)
{
    kal_uint16 temp_reg1, temp_reg2, temp_reg3;
    kal_uint32 temp_reg, extra_exp_lines,exp;

    temp_reg1 = OV2675_MIPI_read_cmos_sensor(0x3500);    // b3~b0 Expo[b15~b12]
    temp_reg2 = OV2675_MIPI_read_cmos_sensor(0x3501);    // b7~b0 Expo[b11~b4]
    temp_reg3 = OV2675_MIPI_read_cmos_sensor(0x3502);    // b7~b4 Expo[b3~b0]
    temp_reg = ((temp_reg3 & 0x00F0)>>4) | ((temp_reg2 & 0x00FF) << 4) | ((temp_reg1&0x000F)<<12);

    temp_reg1 = OV2675_MIPI_read_cmos_sensor(0x3A5F);    // EXVTS[b7~b0]
    temp_reg2 = OV2675_MIPI_read_cmos_sensor(0x3A5E);    // EXVTS[b15~b8]
    extra_exp_lines = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

    spin_lock(&ov2675_drv_lock);
    OV2675_MIPI_PV_Shutter = temp_reg ;
    OV2675_MIPI_PV_Extra_Lines = extra_exp_lines;
    spin_unlock(&ov2675_drv_lock);
    
    //printk("OV2675_MIPI_PV_Shutter=%d,OV2675_MIPI_PV_Extra_Lines=%d\n",OV2675_MIPI_PV_Shutter,OV2675_MIPI_PV_Extra_Lines);
    exp=(temp_reg )*746/12;

    return exp;
}    /* OV7695_MIPI_read_shutter */
void OV2675_MIPI_write_OV2675_MIPI_gain(kal_uint16 gain)
{    
}  /* OV2675_MIPI_write_OV2675_MIPI_gain */

static void OV2675_MIPI_write_shutter(kal_uint16 shutter)
{
}    /* OV2675_MIPI_write_shutter */


void OV2675_MIPI_Computer_AECAGC(kal_uint16 preview_clk_in_M, kal_uint16 capture_clk_in_M)
{
}

void OV2675_MIPI_set_isp_driving_current(kal_uint8 current)
{
}


static void OV2675_MIPI_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 temp_AE_reg = 0;

    if (AE_enable == KAL_TRUE)
    {
        // turn on AEC/AGC
		SENSORDB("HHL[OV2675]OV2675_MIPI_set_AE_mode mode  KAL_TRUE\n");
    }
    else
    {
        // turn off AEC/AGC
		SENSORDB("HHL[OV2675]OV2675_MIPI_set_AE_mode mode  KAL_FALSE\n");
    }
}


static void OV2675_MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 temp_AWB_reg = 0;

    //return ;
//<2014/5/7-37561-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 front camera driver from norman.
    temp_AWB_reg = OV2675_MIPI_read_cmos_sensor(0x5200);	
    if (AWB_enable == KAL_TRUE)
    {
        //enable Auto WB
        OV2675_MIPI_write_cmos_sensor(0x5200, temp_AWB_reg & ~0x20);
    }
    else
    {
        //turn off AWB
        OV2675_MIPI_write_cmos_sensor(0x5200, temp_AWB_reg | 0x20);
    }
//>2014/5/7-37561-joubert.she
}


BOOL OV2675_MIPI_set_param_banding(UINT16 para)
{
    kal_uint8 banding;
    banding = OV2675_MIPI_read_cmos_sensor(0x5002);	
    
    switch (para)
    {
    default:
    case AE_FLICKER_MODE_50HZ:
		                spin_lock(&ov2675_drv_lock);		
		                OV2675_MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;		
		                spin_unlock(&ov2675_drv_lock);
										
		                OV2675_MIPI_write_cmos_sensor(0x5002,banding | 0x02);    /* enable banding and 50 Hz */			//norman@20140310	
                    break;    
    case AE_FLICKER_MODE_60HZ:
		                spin_lock(&ov2675_drv_lock);
		                OV2675_MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;	
		                spin_unlock(&ov2675_drv_lock);	
	 
                    OV2675_MIPI_write_cmos_sensor(0x5002,(banding & ~0x02));    /* enable banding and 60 Hz */     //norman@20140310       
                    break;
//            default:
//                    return FALSE;
    }
    return TRUE;
} /* OV2675_MIPI_set_param_banding */

/*************************************************************************
* FUNCTION
*	OV2675_MIPI_night_mode
*
* DESCRIPTION
*	This function night mode of OV2675.
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
void OV2675_MIPI_night_mode(kal_bool enable)
{
	kal_uint8 temp;
	// <----Nicholas 20140409 NightMode
	temp= OV2675_MIPI_read_cmos_sensor(0x3A00);
	
	if(enable){
			OV2675_MIPI_write_cmos_sensor(0x3A00,(temp | 0x04));
			//;@@ Enable NightMode (30fps~5fps)
			OV2675_MIPI_write_cmos_sensor(0x3a00,0x7c);//;78; [2] Enable night mode; [5] Enable banding option
		        OV2675_MIPI_write_cmos_sensor(0x382a,0x08);//;00; [3] Enable Auto_VTS (for ov7695 only)
		        OV2675_MIPI_write_cmos_sensor(0x3a05,0x30);//; 30; [6] 1: insert by frame; 0: insert by band
		        OV2675_MIPI_write_cmos_sensor(0x3a17,0x03);//;    [1:0] Gain threshold of night mode
		          //;   2'b00: Night mode gain threshold as 00
		          //;   2'b01: Night mode gain threshold as 10
		          //;   2'b10: Night mode gain threshold as 30
		          //;   2'b11: Night mode gain threshold as 70
		        OV2675_MIPI_write_cmos_sensor(0x3a02,0x0c);//;02; 60Hz Max Expo ; ( 1/5 / 1/120 * 0x86 )
		        OV2675_MIPI_write_cmos_sensor(0x3a03,0x90);//;14; 60Hz Max Expo
		        OV2675_MIPI_write_cmos_sensor(0x3a14,0x0c);//;02; 50Hz Max Expo ; ( 1/5 / 1/100 * 0xA0 )
		        OV2675_MIPI_write_cmos_sensor(0x3a15,0x80);//;14; 50Hz Max Expo 
		        OV2675_MIPI_write_cmos_sensor(0x3a0d,0x04);//;0c;04; 60Hz Max Band Setp "in one frame". No need to be increased for AEC v1.1
		        OV2675_MIPI_write_cmos_sensor(0x3a0e,0x03);//;0a;03; 50Hz Max Band Setp "in one frame". No need to be increased for AEC v1.1
		        OV2675_MIPI_write_cmos_sensor(0x3a21,0x72);//;    [6:4] maximum allowable dummy frame   	
		}
	else{
			OV2675_MIPI_write_cmos_sensor(0x3A00,(temp & ~0x04));
			//;@@ Enable NormalMode (30fps~10fps) 
		        OV2675_MIPI_write_cmos_sensor(0x3a00,0x7c);//;78; [2] Enable night mode; [5] Enable banding option
		        OV2675_MIPI_write_cmos_sensor(0x382a,0x08);//;00; [3] Enable Auto_VTS (for ov7695 only)
		        OV2675_MIPI_write_cmos_sensor(0x3a05,0x30);//; 30; [6] 1: insert by frame; 0: insert by band
		        OV2675_MIPI_write_cmos_sensor(0x3a17,0x03);//;    [1:0] Gain threshold of night mode
		          //;   2'b00: Night mode gain threshold as 00
		          //;   2'b01: Night mode gain threshold as 10
		          //;   2'b10: Night mode gain threshold as 30
		          //;   2'b11: Night mode gain threshold as 70
		        OV2675_MIPI_write_cmos_sensor(0x3a02,0x06);//;02; 60Hz Max Expo ; 0x86*4*3-4
		        OV2675_MIPI_write_cmos_sensor(0x3a03,0x44);//;14; 60Hz Max Expo
		        OV2675_MIPI_write_cmos_sensor(0x3a14,0x06);//;02; 50Hz Max Expo ; 0xa0*3*3+0xa0
		        OV2675_MIPI_write_cmos_sensor(0x3a15,0x40);//;14; 50Hz Max Expo 
		        OV2675_MIPI_write_cmos_sensor(0x3a0d,0x04);//;0c;04; 60Hz Max Band Setp "in one frame". No need to be increased for AEC v1.1
		        OV2675_MIPI_write_cmos_sensor(0x3a0e,0x03);//;0a;03; 50Hz Max Band Setp "in one frame". No need to be increased for AEC v1.1
		        OV2675_MIPI_write_cmos_sensor(0x3a21,0x72);//;    [6:4] maximum allowable dummy frame   
		}

}	/* OV2675_MIPI_night_mode */

void OV2675_MIPI_night_ZSD_mode(kal_bool enable)
{
    return;
}
void OV2675_MIPI_Initialize_Setting(void)
{
        OV2675_MIPI_write_cmos_sensor(0x0103,0x01);
        Sleep(10);
        OV2675_MIPI_write_cmos_sensor(0x3620,0x2f);
        OV2675_MIPI_write_cmos_sensor(0x3623,0x12);
        OV2675_MIPI_write_cmos_sensor(0x3718,0x88);
        OV2675_MIPI_write_cmos_sensor(0x3703,0x80);
        OV2675_MIPI_write_cmos_sensor(0x3712,0x40);
        OV2675_MIPI_write_cmos_sensor(0x3706,0x40);
        OV2675_MIPI_write_cmos_sensor(0x3631,0x44);
        OV2675_MIPI_write_cmos_sensor(0x3632,0x05);
        OV2675_MIPI_write_cmos_sensor(0x3013,0xd0);
        OV2675_MIPI_write_cmos_sensor(0x3705,0x1d);
        OV2675_MIPI_write_cmos_sensor(0x3713,0x0e);
        OV2675_MIPI_write_cmos_sensor(0x3012,0x0a);
        OV2675_MIPI_write_cmos_sensor(0x3717,0x18);
        OV2675_MIPI_write_cmos_sensor(0x3621,0x47);
        OV2675_MIPI_write_cmos_sensor(0x0309,0x24);
        OV2675_MIPI_write_cmos_sensor(0x3820,0x90);
        OV2675_MIPI_write_cmos_sensor(0x4803,0x08);
        OV2675_MIPI_write_cmos_sensor(0x0101,0x01);
        OV2675_MIPI_write_cmos_sensor(0x5100,0x01);
        //
        OV2675_MIPI_write_cmos_sensor(0x4500,0x24);


        //gamma OK
        OV2675_MIPI_write_cmos_sensor(0x5301,0x05);
        OV2675_MIPI_write_cmos_sensor(0x5302,0x0c);
        OV2675_MIPI_write_cmos_sensor(0x5303,0x1c);
        OV2675_MIPI_write_cmos_sensor(0x5304,0x2a);
        OV2675_MIPI_write_cmos_sensor(0x5305,0x39);
        OV2675_MIPI_write_cmos_sensor(0x5306,0x45);
        OV2675_MIPI_write_cmos_sensor(0x5307,0x52);
        OV2675_MIPI_write_cmos_sensor(0x5308,0x5d);
        OV2675_MIPI_write_cmos_sensor(0x5309,0x68);

        OV2675_MIPI_write_cmos_sensor(0x530a,0x7f); 
        OV2675_MIPI_write_cmos_sensor(0x530b,0x91);  
        OV2675_MIPI_write_cmos_sensor(0x530c,0xa5);  
        OV2675_MIPI_write_cmos_sensor(0x530d,0xc6);  
        OV2675_MIPI_write_cmos_sensor(0x530e,0xde); 
        OV2675_MIPI_write_cmos_sensor(0x530f,0xef);  
        OV2675_MIPI_write_cmos_sensor(0x5310,0x16);
        //
        OV2675_MIPI_write_cmos_sensor(0x520a,0xf4);
        OV2675_MIPI_write_cmos_sensor(0x520b,0xf4);
        OV2675_MIPI_write_cmos_sensor(0x520c,0xf4);
        //
        OV2675_MIPI_write_cmos_sensor(0x5504,0x08);
        OV2675_MIPI_write_cmos_sensor(0x5505,0x48);
        OV2675_MIPI_write_cmos_sensor(0x5506,0x07);
        OV2675_MIPI_write_cmos_sensor(0x5507,0x0b);
        //
        OV2675_MIPI_write_cmos_sensor(0x3a18,0x01);
        OV2675_MIPI_write_cmos_sensor(0x3a19,0x00);
        OV2675_MIPI_write_cmos_sensor(0x3503,0x03);
        OV2675_MIPI_write_cmos_sensor(0x3500,0x00);
        OV2675_MIPI_write_cmos_sensor(0x3501,0x21);
        OV2675_MIPI_write_cmos_sensor(0x3502,0x00);
        OV2675_MIPI_write_cmos_sensor(0x350a,0x00);
        OV2675_MIPI_write_cmos_sensor(0x350b,0x00);
        OV2675_MIPI_write_cmos_sensor(0x4008,0x02);
        OV2675_MIPI_write_cmos_sensor(0x4009,0x09);
        OV2675_MIPI_write_cmos_sensor(0x3002,0x09);
        OV2675_MIPI_write_cmos_sensor(0x3024,0x00);
        OV2675_MIPI_write_cmos_sensor(0x3503,0x00);
        //


        //ISP
        OV2675_MIPI_write_cmos_sensor(0x0101,0x01); //mirror_on

        OV2675_MIPI_write_cmos_sensor(0x5002,0x48);
        OV2675_MIPI_write_cmos_sensor(0x5910,0x00);
        OV2675_MIPI_write_cmos_sensor(0x3a0f,0x58);
        OV2675_MIPI_write_cmos_sensor(0x3a10,0x50);
        OV2675_MIPI_write_cmos_sensor(0x3a1b,0x5a);
        OV2675_MIPI_write_cmos_sensor(0x3a1e,0x4e);
        OV2675_MIPI_write_cmos_sensor(0x3a11,0xa0);
        OV2675_MIPI_write_cmos_sensor(0x3a1f,0x28);
        OV2675_MIPI_write_cmos_sensor(0x3a18,0x00);
        OV2675_MIPI_write_cmos_sensor(0x3a19,0xe0);//f8; max gain 15.5x
        OV2675_MIPI_write_cmos_sensor(0x3503,0x00);//aec/agc
        OV2675_MIPI_write_cmos_sensor(0x3a0d,0x04);//60Hz Max Band Step

        OV2675_MIPI_write_cmos_sensor(0x5000,0xff);//lcd,gma,awb,awbg,bc,wc,lenc,isp
        OV2675_MIPI_write_cmos_sensor(0x5001,0x3f);//avg, blc,sde,uv_avg,cmx, cip


        //lens
        //R
        OV2675_MIPI_write_cmos_sensor(0x5100,0x01);
        OV2675_MIPI_write_cmos_sensor(0x5101,0x48);
        OV2675_MIPI_write_cmos_sensor(0x5102,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5103,0xf8);
        OV2675_MIPI_write_cmos_sensor(0x5104,0x04);
        OV2675_MIPI_write_cmos_sensor(0x5105,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5106,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5107,0x00);
        //G
        OV2675_MIPI_write_cmos_sensor(0x5108,0x01);
        OV2675_MIPI_write_cmos_sensor(0x5109,0x48);
        OV2675_MIPI_write_cmos_sensor(0x510a,0x00);
        OV2675_MIPI_write_cmos_sensor(0x510b,0xf8);
        OV2675_MIPI_write_cmos_sensor(0x510c,0x03);
        OV2675_MIPI_write_cmos_sensor(0x510d,0x00);
        OV2675_MIPI_write_cmos_sensor(0x510e,0x00);
        OV2675_MIPI_write_cmos_sensor(0x510f,0x00);
        //B
        OV2675_MIPI_write_cmos_sensor(0x5110,0x01);
        OV2675_MIPI_write_cmos_sensor(0x5111,0x48);
        OV2675_MIPI_write_cmos_sensor(0x5112,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5113,0xf8);
        OV2675_MIPI_write_cmos_sensor(0x5114,0x03);
        OV2675_MIPI_write_cmos_sensor(0x5115,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5116,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5117,0x00);
        //;AEC/AGC

        //;D5060

        //;Window Setup

        //awb OK
        OV2675_MIPI_write_cmos_sensor(0x520a,0xf4);
        OV2675_MIPI_write_cmos_sensor(0x520b,0xf4);
        OV2675_MIPI_write_cmos_sensor(0x520c,0xb4); //94,f4
        OV2675_MIPI_write_cmos_sensor(0x5004,0x45);
        OV2675_MIPI_write_cmos_sensor(0x5006,0x41); 

        //Gamma
        OV2675_MIPI_write_cmos_sensor(0x5301,0x05);
        OV2675_MIPI_write_cmos_sensor(0x5302,0x0c);
        OV2675_MIPI_write_cmos_sensor(0x5303,0x1c);
        OV2675_MIPI_write_cmos_sensor(0x5304,0x2a);
        OV2675_MIPI_write_cmos_sensor(0x5305,0x39);
        OV2675_MIPI_write_cmos_sensor(0x5306,0x45);
        OV2675_MIPI_write_cmos_sensor(0x5307,0x53);
        OV2675_MIPI_write_cmos_sensor(0x5308,0x5d);
        OV2675_MIPI_write_cmos_sensor(0x5309,0x68);

        OV2675_MIPI_write_cmos_sensor(0x530a,0x7f); 
        OV2675_MIPI_write_cmos_sensor(0x530b,0x91);  
        OV2675_MIPI_write_cmos_sensor(0x530c,0xa5);  
        OV2675_MIPI_write_cmos_sensor(0x530d,0xc6);  
        OV2675_MIPI_write_cmos_sensor(0x530e,0xde); 
        OV2675_MIPI_write_cmos_sensor(0x530f,0xef);  
        OV2675_MIPI_write_cmos_sensor(0x5310,0x16);

        //Sharpness/De-noise
        OV2675_MIPI_write_cmos_sensor(0x5003,0x80);
        OV2675_MIPI_write_cmos_sensor(0x5500,0x08);
        OV2675_MIPI_write_cmos_sensor(0x5501,0x1a); //48, detect gain
        OV2675_MIPI_write_cmos_sensor(0x5502,0x38);
        OV2675_MIPI_write_cmos_sensor(0x5503,0x14);
        OV2675_MIPI_write_cmos_sensor(0x5504,0x08);
        OV2675_MIPI_write_cmos_sensor(0x5505,0x48);
        OV2675_MIPI_write_cmos_sensor(0x5506,0x02); //0a,02
        OV2675_MIPI_write_cmos_sensor(0x5507,0x10); //66,15,12,16
        OV2675_MIPI_write_cmos_sensor(0x5508,0x2d);
        OV2675_MIPI_write_cmos_sensor(0x5509,0x08);
        OV2675_MIPI_write_cmos_sensor(0x550a,0x48);
        OV2675_MIPI_write_cmos_sensor(0x550b,0x06);
        OV2675_MIPI_write_cmos_sensor(0x550c,0x04);
        OV2675_MIPI_write_cmos_sensor(0x550d,0x01); //00,01

        //SDE, for saturation 120% under D65
        OV2675_MIPI_write_cmos_sensor(0x5800,0x02);
        OV2675_MIPI_write_cmos_sensor(0x5803,0x2e);
        OV2675_MIPI_write_cmos_sensor(0x5804,0x20);

        //cmx QE
        OV2675_MIPI_write_cmos_sensor(0x5600,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5601 ,0x2e); 
        OV2675_MIPI_write_cmos_sensor(0x5602 ,0x60);
        OV2675_MIPI_write_cmos_sensor(0x5603,0x06);
        
        
        OV2675_MIPI_write_cmos_sensor(0x560a,0x01);
        OV2675_MIPI_write_cmos_sensor(0x560b,0x9c);

        //
        OV2675_MIPI_write_cmos_sensor(0x3811,0x07);
        OV2675_MIPI_write_cmos_sensor(0x3813,0x06);


        OV2675_MIPI_write_cmos_sensor(0x3630,0x79);


        //;@@ 0 0 OVM7695 IQ fine tune in arima office
        OV2675_MIPI_write_cmos_sensor(0x5000,0xff);
        //R
        OV2675_MIPI_write_cmos_sensor(0x5100,0x01);
        OV2675_MIPI_write_cmos_sensor(0x5101,0x48);
        OV2675_MIPI_write_cmos_sensor(0x5102,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5103,0xf8);
        OV2675_MIPI_write_cmos_sensor(0x5104,0x02);//  ;R_A1 [6:0]       //0x04
        OV2675_MIPI_write_cmos_sensor(0x5105,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5106,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5107,0x00);
        //G
        OV2675_MIPI_write_cmos_sensor(0x5108,0x01);
        OV2675_MIPI_write_cmos_sensor(0x5109,0x48);
        OV2675_MIPI_write_cmos_sensor(0x510A,0x00);//  ;[1:0] G_Y[9:8]
        OV2675_MIPI_write_cmos_sensor(0x510B,0xf8);//  ;G_Y [7:0]
        OV2675_MIPI_write_cmos_sensor(0x510C,0x02);//;03  ;G_A1 [6:0]    //0x02
        OV2675_MIPI_write_cmos_sensor(0x510D,0x00);//  ;G_A2[3:0]	
        OV2675_MIPI_write_cmos_sensor(0x510E,0x01);//;00  ;G_B1 [7:0]        
        OV2675_MIPI_write_cmos_sensor(0x510F,0x00);//  ;G_B2 [3:0]	
        //B
        OV2675_MIPI_write_cmos_sensor(0x5110,0x01);
        OV2675_MIPI_write_cmos_sensor(0x5111,0x48);
        OV2675_MIPI_write_cmos_sensor(0x5112,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5113,0xf8);
        OV2675_MIPI_write_cmos_sensor(0x5114,0x02);
        OV2675_MIPI_write_cmos_sensor(0x5115,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5116,0x00);
        OV2675_MIPI_write_cmos_sensor(0x5117,0x00);


        //;@@ -0.7EV to meet 128
        OV2675_MIPI_write_cmos_sensor(0x3a0f,0x48);
        OV2675_MIPI_write_cmos_sensor(0x3a10,0x40);
        OV2675_MIPI_write_cmos_sensor(0x3a11,0x90);
        OV2675_MIPI_write_cmos_sensor(0x3a1b,0x4A);
        OV2675_MIPI_write_cmos_sensor(0x3a1e,0x3E);
        OV2675_MIPI_write_cmos_sensor(0x3a1f,0x18);

        //;adding on 20140127-tom
        //;@@ ov ori ctx(19.)

        OV2675_MIPI_write_cmos_sensor(0x5604,0x1c);
        OV2675_MIPI_write_cmos_sensor(0x5605,0x65);
        OV2675_MIPI_write_cmos_sensor(0x5606,0x81);
        OV2675_MIPI_write_cmos_sensor(0x5607,0x9f);
        OV2675_MIPI_write_cmos_sensor(0x5608,0x8a);
        OV2675_MIPI_write_cmos_sensor(0x5609,0x15);
        
        //----Nicholas 20140415 sat
        //@@ r0.9x CTX-arima(depends 20140414 final setting)
        
        OV2675_MIPI_write_cmos_sensor(0x5604,0x1c);
        OV2675_MIPI_write_cmos_sensor(0x5605,0x65);
        OV2675_MIPI_write_cmos_sensor(0x5606,0x81);
        OV2675_MIPI_write_cmos_sensor(0x5607,0x9f);
        OV2675_MIPI_write_cmos_sensor(0x5608,0x8a);
        OV2675_MIPI_write_cmos_sensor(0x5609,0x15);
        
        
        //@@ r0.8x CTX
        /*
        OV2675_MIPI_write_cmos_sensor(0x5604,0x16);
        OV2675_MIPI_write_cmos_sensor(0x5605,0x52);
        OV2675_MIPI_write_cmos_sensor(0x5606,0x68);
        OV2675_MIPI_write_cmos_sensor(0x5607,0x81);
        OV2675_MIPI_write_cmos_sensor(0x5608,0x70);
        OV2675_MIPI_write_cmos_sensor(0x5609,0x11);
        */
        //@@ r0.7x CTX
        /*
        OV2675_MIPI_write_cmos_sensor(0x5604,0x14);
        OV2675_MIPI_write_cmos_sensor(0x5605,0x4a);
        OV2675_MIPI_write_cmos_sensor(0x5606,0x5e);
        OV2675_MIPI_write_cmos_sensor(0x5607,0x74);
        OV2675_MIPI_write_cmos_sensor(0x5608,0x65);
        OV2675_MIPI_write_cmos_sensor(0x5609,0x0f);
        */
        //@@ r0.6x CTX 
        /*
        OV2675_MIPI_write_cmos_sensor(0x5604,0x12);
        OV2675_MIPI_write_cmos_sensor(0x5605,0x43);
        OV2675_MIPI_write_cmos_sensor(0x5606,0x55);
        OV2675_MIPI_write_cmos_sensor(0x5607,0x68);
        OV2675_MIPI_write_cmos_sensor(0x5608,0x5b);
        OV2675_MIPI_write_cmos_sensor(0x5609,0x0d);
        */
        //----sat end
        

        OV2675_MIPI_write_cmos_sensor(0x5602,0x60);
        //

        //;color sat
        OV2675_MIPI_write_cmos_sensor(0x5803,0x2c);//;29; main
        OV2675_MIPI_write_cmos_sensor(0x5804,0x23);//; second

        OV2675_MIPI_write_cmos_sensor(0x5601,0x30);//;2c
        OV2675_MIPI_write_cmos_sensor(0x5602,0x60);//
        OV2675_MIPI_write_cmos_sensor(0x5104,0x04);//;02,04
        OV2675_MIPI_write_cmos_sensor(0x510C,0x00);//;02
        OV2675_MIPI_write_cmos_sensor(0x5114,0x00);//;02
        OV2675_MIPI_write_cmos_sensor(0x5105,0x01);//;00
        OV2675_MIPI_write_cmos_sensor(0x510d,0x01);//;00
        OV2675_MIPI_write_cmos_sensor(0x5115,0x01);//;00
        OV2675_MIPI_write_cmos_sensor(0x5803,0x2c);//;26
        OV2675_MIPI_write_cmos_sensor(0x5804,0x24);//;20


//;@@ Whole Image Center More weight
        OV2675_MIPI_write_cmos_sensor(0x5908,0x22);   //62  
        OV2675_MIPI_write_cmos_sensor(0x5909,0x22);   //26
        OV2675_MIPI_write_cmos_sensor(0x590a,0xe2);   //e6
        OV2675_MIPI_write_cmos_sensor(0x590b,0x2e);   //6e
        OV2675_MIPI_write_cmos_sensor(0x590c,0xe2);   //ea
        OV2675_MIPI_write_cmos_sensor(0x590d,0x2e);   //ae
        OV2675_MIPI_write_cmos_sensor(0x590e,0x22);   //a6
        OV2675_MIPI_write_cmos_sensor(0x590f,0x22);   //6a
 
//;@@ BLC
        OV2675_MIPI_write_cmos_sensor(0x4003,0x08);

//;@@ edge
        //OV2675_MIPI_write_cmos_sensor(0x5502,0x2c);//;38;20;18 ;sharp mt offset1
        //OV2675_MIPI_write_cmos_sensor(0x5503,0x0e);//;14;08;04 ;sharp mt offset2
        // 1
        //OV2675_MIPI_write_cmos_sensor(0x5502,0x26);//;38;20;18 ;sharp mt offset1
        //OV2675_MIPI_write_cmos_sensor(0x5503,0x0b);//;14;08;04 ;sharp mt offset2
        // 2
        //OV2675_MIPI_write_cmos_sensor(0x5502,0x20);//;38;20;18 ;sharp mt offset1
        //OV2675_MIPI_write_cmos_sensor(0x5503,0x08);//;14;08;04 ;sharp mt offset2
        // 3
        OV2675_MIPI_write_cmos_sensor(0x5502,0x2c);//;1c,38;20;18 ;sharp mt offset1
        OV2675_MIPI_write_cmos_sensor(0x5503,0x04);//;06,14;08;04 ;sharp mt offset2
        //old
        //OV2675_MIPI_write_cmos_sensor(0x5502,0x18);//;38;20;18 ;sharp mt offset1
        //OV2675_MIPI_write_cmos_sensor(0x5503,0x04);//;14;08;04 ;sharp mt offset2

//@@ dark level more black gamma-arima
/*
        OV2675_MIPI_write_cmos_sensor(0x5310,0x16);  
        OV2675_MIPI_write_cmos_sensor(0x5301,0x1 ); 
        OV2675_MIPI_write_cmos_sensor(0x5302,0x2 ); 
        OV2675_MIPI_write_cmos_sensor(0x5303,0xe ); 
        OV2675_MIPI_write_cmos_sensor(0x5304,0x19);  
        OV2675_MIPI_write_cmos_sensor(0x5305,0x29);  
        OV2675_MIPI_write_cmos_sensor(0x5306,0x3a);  
        OV2675_MIPI_write_cmos_sensor(0x5307,0x4d);  
        OV2675_MIPI_write_cmos_sensor(0x5308,0x5a);  
        OV2675_MIPI_write_cmos_sensor(0x5309,0x67);  
        OV2675_MIPI_write_cmos_sensor(0x530a,0x7f);  
        OV2675_MIPI_write_cmos_sensor(0x530b,0x91);  
        OV2675_MIPI_write_cmos_sensor(0x530c,0xa5);  
        OV2675_MIPI_write_cmos_sensor(0x530d,0xc6);  
        OV2675_MIPI_write_cmos_sensor(0x530e,0xde);  
        OV2675_MIPI_write_cmos_sensor(0x530f,0xef); 
*/        
/*
//gamma=>middle level high
        OV2675_MIPI_write_cmos_sensor(0x5301,0x07);  
        OV2675_MIPI_write_cmos_sensor(0x5302,0x0c);  
        OV2675_MIPI_write_cmos_sensor(0x5303,0x18);  
        OV2675_MIPI_write_cmos_sensor(0x5304,0x23);  
        OV2675_MIPI_write_cmos_sensor(0x5305,0x33);  
        OV2675_MIPI_write_cmos_sensor(0x5306,0x40);  
        OV2675_MIPI_write_cmos_sensor(0x5307,0x51);  
        OV2675_MIPI_write_cmos_sensor(0x5308,0x5c);  
        OV2675_MIPI_write_cmos_sensor(0x5309,0x67);  
        OV2675_MIPI_write_cmos_sensor(0x530a,0x7f);  
        OV2675_MIPI_write_cmos_sensor(0x530b,0x91);  
        OV2675_MIPI_write_cmos_sensor(0x530c,0xa5);  
        OV2675_MIPI_write_cmos_sensor(0x530d,0xc6);  
        OV2675_MIPI_write_cmos_sensor(0x530e,0xde);  
        OV2675_MIPI_write_cmos_sensor(0x530f,0xef);  
*/

//gamma=>middle level low
/*
        OV2675_MIPI_write_cmos_sensor(0x5310,0x1a);   
        OV2675_MIPI_write_cmos_sensor(0x5301,0x01);   
        OV2675_MIPI_write_cmos_sensor(0x5302,0x02);   
        OV2675_MIPI_write_cmos_sensor(0x5303,0x0e);   
        OV2675_MIPI_write_cmos_sensor(0x5304,0x19);   
        OV2675_MIPI_write_cmos_sensor(0x5305,0x29);   
        OV2675_MIPI_write_cmos_sensor(0x5306,0x3a);   
        OV2675_MIPI_write_cmos_sensor(0x5307,0x4d);   
        OV2675_MIPI_write_cmos_sensor(0x5308,0x57);   
        OV2675_MIPI_write_cmos_sensor(0x5309,0x61);   
        OV2675_MIPI_write_cmos_sensor(0x530a,0x76);   
        OV2675_MIPI_write_cmos_sensor(0x530b,0x84);   
        OV2675_MIPI_write_cmos_sensor(0x530c,0x9a);   
        OV2675_MIPI_write_cmos_sensor(0x530d,0xbd);  
        OV2675_MIPI_write_cmos_sensor(0x530e,0xd8);  
        OV2675_MIPI_write_cmos_sensor(0x530f,0xec);
*/
//middle gamma ----Nicholas 20140414
/*
        OV2675_MIPI_write_cmos_sensor(0x5310,0x16);   
        OV2675_MIPI_write_cmos_sensor(0x5301,0x02);   
        OV2675_MIPI_write_cmos_sensor(0x5302,0x05);   
        OV2675_MIPI_write_cmos_sensor(0x5303,0x0f);   
        OV2675_MIPI_write_cmos_sensor(0x5304,0x1b);   
        OV2675_MIPI_write_cmos_sensor(0x5305,0x2d);   
        OV2675_MIPI_write_cmos_sensor(0x5306,0x3c);   
        OV2675_MIPI_write_cmos_sensor(0x5307,0x50);   
        OV2675_MIPI_write_cmos_sensor(0x5308,0x5b);   
        OV2675_MIPI_write_cmos_sensor(0x5309,0x67);   
        OV2675_MIPI_write_cmos_sensor(0x530a,0x7f);   
        OV2675_MIPI_write_cmos_sensor(0x530b,0x91);   
        OV2675_MIPI_write_cmos_sensor(0x530c,0xa5);   
        OV2675_MIPI_write_cmos_sensor(0x530d,0xc6);  
        OV2675_MIPI_write_cmos_sensor(0x530e,0xde);  
        OV2675_MIPI_write_cmos_sensor(0x530f,0xef);
 */
 //middle++ ----Nicholas 20140414

        OV2675_MIPI_write_cmos_sensor(0x5310,0x28);   //16
        OV2675_MIPI_write_cmos_sensor(0x5301,0x05);   
        OV2675_MIPI_write_cmos_sensor(0x5302,0x09);   
        OV2675_MIPI_write_cmos_sensor(0x5303,0x15);   
        OV2675_MIPI_write_cmos_sensor(0x5304,0x20);   
        OV2675_MIPI_write_cmos_sensor(0x5305,0x31);   
        OV2675_MIPI_write_cmos_sensor(0x5306,0x3f);   
        OV2675_MIPI_write_cmos_sensor(0x5307,0x50);   //51  
        OV2675_MIPI_write_cmos_sensor(0x5308,0x5b);   //5c
        OV2675_MIPI_write_cmos_sensor(0x5309,0x65);   //67
        OV2675_MIPI_write_cmos_sensor(0x530a,0x77);   //7f
        OV2675_MIPI_write_cmos_sensor(0x530b,0x83);   //91
        OV2675_MIPI_write_cmos_sensor(0x530c,0x91);   //a5
        OV2675_MIPI_write_cmos_sensor(0x530d,0xad);   //c6
        OV2675_MIPI_write_cmos_sensor(0x530e,0xcb);   //de
        OV2675_MIPI_write_cmos_sensor(0x530f,0xe2);   //ef

        //for init AWB in D50
        OV2675_MIPI_write_cmos_sensor(0x5200,0x20);//  MWB
        OV2675_MIPI_write_cmos_sensor(0x5204,0x05);// R-gain (could try tuning to target lighting)
        OV2675_MIPI_write_cmos_sensor(0x5208,0x05);// B-gain (could try tuning to target lighting) 
        
        OV2675_MIPI_write_cmos_sensor(0x0100,0x01);
        //mipi

        Sleep(100);
        OV2675_MIPI_write_cmos_sensor(0x5200,0x00);//  AWB
}


/*************************************************************************
* FUNCTION
*	OV2675_MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
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
static kal_uint32 OV2675_MIPI_GetSensorID(kal_uint32 *sensorID)

{
	volatile signed char i;
		kal_uint32 sensor_id=0;
		kal_uint8 temp_sccb_addr = 0;
		//s_move to here from CISModulePowerOn()

		OV2675_MIPI_write_cmos_sensor(0x0103,0x01);// Reset sensor
		
			mDELAY(10);
		
		
			//	Read sensor ID to adjust I2C is OK?
			for(i=0;i<3;i++)
			{
				sensor_id = (OV2675_MIPI_read_cmos_sensor(0x300A) << 8) | OV2675_MIPI_read_cmos_sensor(0x300B);
				printk("++++OV2675_MIPI_GetSensorID,read id = 0x%x\n", sensor_id);
				if(sensor_id != OV2675_SENSOR_ID)
				{
					*sensorID =0xFFFFFFFF;
					return ERROR_SENSOR_CONNECT_FAIL;
				}
			}
			    *sensorID = sensor_id;
			printk("++++OV2675_MIPI_GetSensorID OK!!!\n");
			RETAILMSG(1, (TEXT("OV2675 Sensor Read ID OK \r\n")));
		
    return ERROR_NONE;    
}   

void OV2675_MIPI_get_exposure_gain(void)
{
  kal_uint32  Shutter_tmp,Gain_tmp;

    Shutter_tmp = OV2675_MIPI_read_shutter() ;
    Gain_tmp = OV2675_MIPI_read_OV2675_MIPI_gain() ;
    
    spin_lock(&ov2675_drv_lock);
	//<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
    OV2675_CurrentStatus.iShutter= Shutter_tmp;
	//>2014/5/2-37049-joubert.she
    spin_unlock(&ov2675_drv_lock);

    spin_lock(&ov2675_drv_lock);
    OV2675_CurrentStatus.iGain= Gain_tmp;
    spin_unlock(&ov2675_drv_lock);
}

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	OV2675Open
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
UINT32 OV2675Open(void)
{
	volatile signed char i;
	kal_uint16 sensor_id=0;

	zoom_factor = 0; 
	//OV2675_MIPI_write_cmos_sensor(0x3012,0x80);// Reset sensor
    //Sleep(10);

	printk("+++++ KYLE ++++ OV2675Open START\n");
	//  Read sensor ID to adjust I2C is OK?
	for(i=0;i<3;i++)
	{
		sensor_id = (OV2675_MIPI_read_cmos_sensor(0x300A) << 8) | OV2675_MIPI_read_cmos_sensor(0x300B);
		printk("++++OV2675Open,read id = 0x%x\n", sensor_id);
		
		if(sensor_id != OV2675_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}
	}
	
	RETAILMSG(1, (TEXT("OV2675 Sensor Read ID OK \r\n")));
		printk("+++++ KYLE ++++ OV2675 Sensor Read ID OK \n");
//init MIPI
		//OV2675_MIPI_write_cmos_sensor(0x0103,0x01);
  		 Sleep(10);

		  //mipi
        OV2675_MIPI_Initialize_Setting();
        printk("+++++ KYLE ++++ OV2675Open END\n");
		return ERROR_NONE;
}	/* OV2675Open() */

/*************************************************************************
* FUNCTION
*	OV2675Close
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
UINT32 OV2675Close(void)
{
//	CISModulePowerOn(FALSE);

	return ERROR_NONE;
}	/* OV2675Close() */


/*************************************************************************
* FUNCTION
*	OV2675Preview
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
UINT32 OV2675Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	kal_uint8 iTemp, temp_AE_reg, temp_AWB_reg;
  kal_uint16 iDummyPixels = 0, iDummyLines = 0, iStartX = 0, iStartY = 0;
   
	//spin_lock(&ov2675_drv_lock);
	//OV2675_MIPI_sensor_cap_state = KAL_FALSE;
	//spin_unlock(&ov2675_drv_lock);  


	//OV2675_MIPI_sensor_zsd_mode=KAL_FALSE;

//	Sleep(50); //caosq add

    printk("++++%s\n", __FUNCTION__);	//
    
    //OV2675_MIPI_set_AE_mode(KAL_FALSE);

    //4  <1> preview config sequence
printk("+++++ KYLE ++++ OV2675Preview START\n");
    
	  //spin_lock(&ov2675_drv_lock);
	  //     OV2675_MIPI_sensor_pclk=390;
	  //spin_unlock(&ov2675_drv_lock);


    //===preview setting end===
    /* ==Camera Preview, MT6235 use 36MHz PCLK, 30fps 60Hz, 25fps in 50Hz== */
    /* after set exposure line, there should be delay for 2~4 frame time, then enable AEC */
    //Use preview_ae_stable_frame to drop frame   

    // turn on AEC/AGC
    // OV2675_MIPI_set_AE_mode(KAL_TRUE); 
    //temp_AE_reg = OV2675_MIPI_read_cmos_sensor(0x3013);
    //OV2675_MIPI_write_cmos_sensor(0x3013, temp_AE_reg|0x05);

    //enable Auto WB
    //OV2675_MIPI_set_AWB_mode(KAL_TRUE); 
    //temp_AWB_reg = OV2675_MIPI_read_cmos_sensor(0x3324);
    //OV2675_MIPI_write_cmos_sensor(0x3324, temp_AWB_reg&~0x40);
   
	  //spin_lock(&ov2675_drv_lock);
       //OV2675_MIPI_gPVmode = KAL_TRUE;
	  //spin_unlock(&ov2675_drv_lock);


        RETAILMSG(1,(TEXT("Camera preview\r\n")));
       printk("++++OV2675 driver: Camera preview \n");
        //sensor_config_data->frame_rate == 30
        //ISP_PREVIEW_MODE
        //4  <2> if preview of capture PICTURE

        /* preview: 30 fps with 36M PCLK */
	//spin_lock(&ov2675_drv_lock);
        //OV2675_MIPI_VEDIO_encode_mode = KAL_FALSE;
	//spin_unlock(&ov2675_drv_lock);

        iDummyPixels = 0; 
        iDummyLines = 0; 

	//spin_lock(&ov2675_drv_lock);
        //OV2675_MIPI_iOV2675_MIPI_Mode = OV2675_MIPI_MODE_PREVIEW;
	//spin_unlock(&ov2675_drv_lock);
        // Set for dynamic sensor delay. 2009-09-09
        //image_window->wait_stable_frame = 3;	


          iStartX = 1;
          iStartY = 1;
 

    //4 <6> set dummy
	//spin_lock(&ov2675_drv_lock);
    //OV2675_MIPI_PV_Dummy_Pixels = iDummyPixels;
	//spin_unlock(&ov2675_drv_lock);
   // OV2675_MIPI_set_dummy(iDummyPixels, iDummyLines);


    //4 <7> set shutter
    image_window->GrabStartX = iStartX;
    image_window->GrabStartY = iStartY;
    image_window->ExposureWindowWidth = OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH - iStartX -2;
    image_window->ExposureWindowHeight = OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT- iStartY -2;

	//spin_lock(&ov2675_drv_lock);
    //OV2675_MIPI_DELAY_AFTER_PREVIEW = 1;
	//spin_unlock(&ov2675_drv_lock);

	// copy sensor_config_data
	memcpy(&OV2675SensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
printk("+++++ KYLE ++++ OV2675Preview END\n");
  	return ERROR_NONE;
}	/* OV2675Preview() */

UINT32 OV2675ZSD(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	// 1600x1200	  
	 spin_lock(&ov2675_drv_lock);
	OV2675_MIPI_sensor_zsd_mode=KAL_TRUE;
	OV2675_MIPI_VEDIO_encode_mode = KAL_FALSE;
	OV2675_MIPI_sensor_cap_state = KAL_FALSE;
	spin_unlock(&ov2675_drv_lock);	
	SENSORDB("HHL[OV2675ZSD]enterOV2675ZSD\n");

	return ERROR_NONE;
}	/* OV2675Capture() */

UINT32 OV2675Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
        volatile kal_uint32 shutter = OV2675_MIPI_exposure_lines, temp_reg;
        kal_uint8 temp_AE_reg, temp;
        kal_uint16 AE_setting_delay = 0;

        spin_lock(&ov2675_drv_lock);
        OV2675_MIPI_sensor_cap_state = KAL_TRUE;

        OV2675_MIPI_sensor_zsd_mode=KAL_FALSE;
        spin_unlock(&ov2675_drv_lock);  

        printk("++++%s\n", __FUNCTION__);	//

        // turn off AEC/AGC
		//<2014/5/7-37561-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 front camera driver from norman.
        //OV2675_MIPI_set_AE_mode(KAL_FALSE);

        //OV2675_MIPI_set_AWB_mode(KAL_FALSE); 
		//>2014/5/7-37561-joubert.she

        spin_lock(&ov2675_drv_lock);
        OV2675_MIPI_DELAY_AFTER_PREVIEW = 2;
        spin_unlock(&ov2675_drv_lock);
        return ERROR_NONE;
}	/* OV2675Capture() */

UINT32 OV2675GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{ 
	pSensorResolution->SensorFullWidth=OV2675_MIPI_IMAGE_SENSOR_FULL_WIDTH;  //modify by yanxu
	pSensorResolution->SensorFullHeight=OV2675_MIPI_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=OV2675_MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorVideoHeight=OV2675_MIPI_IMAGE_SENSOR_PV_HEIGHT;
	

	return ERROR_NONE;
}	/* OV2675GetResolution() */
void OV2675GetDelayInfo(UINT32 delayAddr)
{
	SENSOR_DELAY_INFO_STRUCT* pDelayInfo = (SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay = 3;
	pDelayInfo->EffectDelay = 2;
	pDelayInfo->AwbDelay = 2;
}

void OV2675_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
		  SENSORDB("SENSOR_3A_AE_LOCK \n,");
          OV2675_MIPI_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
		  SENSORDB("SENSOR_3A_AE_UNLOCK \n,");
          OV2675_MIPI_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
	  	  //<2014/04/24-kylechang. CQ:BU2SC00144536 Front camera preview screen will display green during Counting Down Shot.
		  SENSORDB("+++KYLE+++DISable SENSOR_3A_AWB_LOCK \n,");
		  //OV2675_MIPI_set_AWB_mode(KAL_FALSE);
		  //>2014/04/24-kylechang
      break;

      case SENSOR_3A_AWB_UNLOCK:
		  SENSORDB("SENSOR_3A_AE_LOCK \n,");
		  //<2014/5/2-37049-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 camera driver from norman.
		  //OV2675_MIPI_set_AWB_mode(KAL_TRUE);
		  //>2014/5/2-37049-joubert.she
      break;
      default:
      	break;
   }
   return;
}

void OV2675GetAEAWBLock(UINT32 *pAElockRet32,UINT32 *pAWBlockRet32)
{
	*pAElockRet32 = 1;
	*pAWBlockRet32 = 1;
	SENSORDB("S5K8AAYX_MIPIGetAEAWBLock,AE=%d ,AWB=%d\n,",*pAElockRet32,*pAWBlockRet32);
}
void OV2675AutoTestCmd(UINT32 *cmd, UINT32 *para)
{
	switch(*cmd){
		case YUV_AUTOTEST_SET_SHADDING:
			SENSORDB("YUV_AUTOTEST_SET_SHADDING:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_GAMMA:
			SENSORDB("YUV_AUTOTEST_SET_GAMMA:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_AE:
			SENSORDB("YUV_AUTOTEST_SET_AE:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_SHUTTER:
			SENSORDB("YUV_AUTOTEST_SET_SHUTTER:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_SET_GAIN:
			SENSORDB("YUV_AUTOTEST_SET_GAIN:para = %d\n",*para);
		break;
		case YUV_AUTOTEST_GET_SHUTTER_RANGE:
			*para = 8228;
		break;
		default:	
			SENSORDB("YUV AUTOTEST NOT SUPPORT CMD:%d\n",*cmd);
		break;
	}
}

UINT32 OV2675GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1; 
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	/*??? */
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;    //SENSOR_INTERFACE_TYPE_PARALLEL

	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 3; 
	pSensorInfo->VideoDelayFrame = 4; 		
	pSensorInfo->YUVAwbDelayFrame=2;
	pSensorInfo->YUVEffectDelayFrame=2;
	pSensorInfo->SensorMasterClockSwitch = 0; 
  pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;  	

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
		//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=24;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 0; 
			pSensorInfo->SensorGrabStartY = 0; 
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		 pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
		     break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			pSensorInfo->SensorClockFreq=24;
			   pSensorInfo->SensorClockDividCount=	3;
			   pSensorInfo->SensorClockRisingCount= 0;
			   pSensorInfo->SensorClockFallingCount= 2;
			   pSensorInfo->SensorPixelClockCount= 3;
			   pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 0; 
			pSensorInfo->SensorGrabStartY = 0; 
		 
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		 pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		 pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
				 break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = 0; 
			pSensorInfo->SensorGrabStartY = 0; 
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE; 		
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
		    pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
		    pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
			pSensorInfo->SensorHightSampling = 0;	// 0 is default 1x 
			pSensorInfo->SensorPacketECCOrder = 1;
	       break;
	}
	spin_lock(&ov2675_drv_lock);
	OV2675_MIPI_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	spin_unlock(&ov2675_drv_lock);
	memcpy(pSensorConfigData, &OV2675SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	return ERROR_NONE;
}	/* OV2675GetInfo() */


UINT32 OV2675Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
        switch (ScenarioId)
        {
            case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                //case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
                OV2675Preview(pImageWindow, pSensorConfigData);
                break;
            case MSDK_SCENARIO_ID_CAMERA_ZSD:
                OV2675Preview(pImageWindow, pSensorConfigData);
                break;
            case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            //case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
                OV2675Preview(pImageWindow, pSensorConfigData);
                break;
            default:
                break; 
        }
        return TRUE;
}	/* OV2675Control() */

/* [TC] YUV sensor */	

BOOL OV2675_MIPI_set_param_wb(UINT16 para)
{
    kal_uint8  temp_reg,temp_AE_reg;

    spin_lock(&ov2675_drv_lock);
    OV2675_CurrentStatus.iWB = para;
    spin_unlock(&ov2675_drv_lock);
	//<2014/5/7-37561-joubert.she, [Lo1/Lo2][DRV] update Lo1/Lo2 front camera driver from norman.
    switch (para)
    {
        case AWB_MODE_OFF:
            spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_AWB_ENABLE = KAL_FALSE; 
            spin_unlock(&ov2675_drv_lock);
            //OV2675_MIPI_set_AWB_mode(OV2675_MIPI_AWB_ENABLE);
            //break;                     
        case AWB_MODE_AUTO:
            //<2014/04/24-kylechang. Re-Config register while change to AWB mode by Norman
            if(OV2675_MIPI_AWB_ENABLE == KAL_FALSE)
            {
                OV2675_MIPI_write_cmos_sensor(0x5200, 0x20);
                OV2675_MIPI_write_cmos_sensor(0x5204, 0x05);  //R Gain
                OV2675_MIPI_write_cmos_sensor(0x5205, 0x00);	
                OV2675_MIPI_write_cmos_sensor(0x5206, 0x04);	//G Gain
                OV2675_MIPI_write_cmos_sensor(0x5207, 0x00);              
                OV2675_MIPI_write_cmos_sensor(0x5208, 0x05);	//B Gain
                OV2675_MIPI_write_cmos_sensor(0x5209, 0x00);
                Sleep(100);
            }
            OV2675_MIPI_write_cmos_sensor(0x5200, 0x00);
            spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_AWB_ENABLE = KAL_TRUE; 
            spin_unlock(&ov2675_drv_lock);
            //>2014/04/24-kylechang

            break;

        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
            spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_AWB_ENABLE = KAL_FALSE; 
            spin_unlock(&ov2675_drv_lock);
            OV2675_MIPI_write_cmos_sensor(0x5200, 0x20);
            OV2675_MIPI_write_cmos_sensor(0x5204, 0x06);  //R Gain
            OV2675_MIPI_write_cmos_sensor(0x5205, 0x0b);	
            OV2675_MIPI_write_cmos_sensor(0x5206, 0x04);	//G Gain
            OV2675_MIPI_write_cmos_sensor(0x5207, 0x00);              
            OV2675_MIPI_write_cmos_sensor(0x5208, 0x05);	//B Gain
            OV2675_MIPI_write_cmos_sensor(0x5209, 0x60);
            break;

        case AWB_MODE_DAYLIGHT: //sunny
            spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_AWB_ENABLE = KAL_FALSE; 
            spin_unlock(&ov2675_drv_lock);
            OV2675_MIPI_write_cmos_sensor(0x5200, 0x20);
            OV2675_MIPI_write_cmos_sensor(0x5204, 0x04);  //R Gain
            OV2675_MIPI_write_cmos_sensor(0x5205, 0xa2);	
            OV2675_MIPI_write_cmos_sensor(0x5206, 0x04);	//G Gain
            OV2675_MIPI_write_cmos_sensor(0x5207, 0x00);              
            OV2675_MIPI_write_cmos_sensor(0x5208, 0x05);	//B Gain
            OV2675_MIPI_write_cmos_sensor(0x5209, 0x52);
            break;


        case AWB_MODE_TUNGSTEN: //home
        case AWB_MODE_INCANDESCENT: //office
            spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_AWB_ENABLE = KAL_FALSE; 
            spin_unlock(&ov2675_drv_lock);
            OV2675_MIPI_write_cmos_sensor(0x5200, 0x20);
            OV2675_MIPI_write_cmos_sensor(0x5204, 0x04);  //R Gain
            OV2675_MIPI_write_cmos_sensor(0x5205, 0x00);	
            OV2675_MIPI_write_cmos_sensor(0x5206, 0x05);	//G Gain
            OV2675_MIPI_write_cmos_sensor(0x5207, 0x0D);              
            OV2675_MIPI_write_cmos_sensor(0x5208, 0x0B);	//B Gain
            OV2675_MIPI_write_cmos_sensor(0x5209, 0xE8);
            break;

        case AWB_MODE_FLUORESCENT:
            spin_lock(&ov2675_drv_lock);
            OV2675_MIPI_AWB_ENABLE = KAL_FALSE; 
            spin_unlock(&ov2675_drv_lock);
            OV2675_MIPI_write_cmos_sensor(0x5200, 0x20);
            OV2675_MIPI_write_cmos_sensor(0x5204, 0x04);  //R Gain
            OV2675_MIPI_write_cmos_sensor(0x5205, 0xa0);	
            OV2675_MIPI_write_cmos_sensor(0x5206, 0x04);	//G Gain
            OV2675_MIPI_write_cmos_sensor(0x5207, 0x00);              
            OV2675_MIPI_write_cmos_sensor(0x5208, 0x07);	//B Gain
            OV2675_MIPI_write_cmos_sensor(0x5209, 0x8e);
            break;
#if WINMO_USE
        case AWB_MODE_MANUAL:
            // TODO
            break;
#endif 

        default:
            return FALSE;
    }
//>2014/5/7-37561-joubert.she.
    return TRUE;
} /* OV2675_MIPI_set_param_wb */

BOOL OV2675_MIPI_set_param_effect(UINT16 para)
{
   BOOL  ret = TRUE;
   //UINT8  temp_reg;
   //temp_reg=OV2675_MIPI_read_cmos_sensor(0x3391);
    switch (para)
    {
        case MEFFECT_OFF: 
             break;

        case MEFFECT_SEPIA:
              break;

        case MEFFECT_NEGATIVE:
             break;

        case MEFFECT_SEPIAGREEN:
             break;

        case MEFFECT_SEPIABLUE:
             break;
             
		    case MEFFECT_MONO: //B&W
 			      break;
 			      
#if WINMO_USE
        case CAM_EFFECT_ENC_GRAYINV:
        case CAM_EFFECT_ENC_COPPERCARVING:
        case CAM_EFFECT_ENC_BLUECARVING:
        case CAM_EFFECT_ENC_EMBOSSMENT:
        case CAM_EFFECT_ENC_SKETCH:
        case CAM_EFFECT_ENC_BLACKBOARD:
        case CAM_EFFECT_ENC_WHITEBOARD:
        case CAM_EFFECT_ENC_JEAN:
        case CAM_EFFECT_ENC_OIL:
#endif 
        default:
            ret = FALSE;
    }

    return ret;

} /* OV2675_MIPI_set_param_effect */

BOOL OV2675_MIPI_set_param_exposure(UINT16 para)
{
 #if 1
 kal_uint8  temp_reg;

    temp_reg=OV2675_MIPI_read_cmos_sensor(0x3391);

    switch (para)
    {
        case AE_EV_COMP_n20:

            OV2675_MIPI_write_cmos_sensor(0x3a11, 0x70);
 
            OV2675_MIPI_write_cmos_sensor(0x3a1b, 0x2A);
            OV2675_MIPI_write_cmos_sensor(0x3a0f, 0x28);
            OV2675_MIPI_write_cmos_sensor(0x3a10, 0x20);
            OV2675_MIPI_write_cmos_sensor(0x3a1e, 0x1E);
 
            OV2675_MIPI_write_cmos_sensor(0x3a1f, 0x00);            
        	  break;

       /* case AE_EVs_COMP_n10:    
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x49);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x28);/* for difference */
          //  break;

        case AE_EV_COMP_n10:
            OV2675_MIPI_write_cmos_sensor(0x3a11, 0x80);
            OV2675_MIPI_write_cmos_sensor(0x3a1b, 0x3A);
            OV2675_MIPI_write_cmos_sensor(0x3a0f, 0x38);
            OV2675_MIPI_write_cmos_sensor(0x3a10, 0x30);
            OV2675_MIPI_write_cmos_sensor(0x3a1e, 0x2E);
            OV2675_MIPI_write_cmos_sensor(0x3a1f, 0x08);            
             break;
          
       /* case AE_EV_COMP_n03:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x49);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x10);
            break;*/

        case AE_EV_COMP_00:
            OV2675_MIPI_write_cmos_sensor(0x3a11, 0x90);
            OV2675_MIPI_write_cmos_sensor(0x3a1b, 0x4A);
            OV2675_MIPI_write_cmos_sensor(0x3a0f, 0x48);
            OV2675_MIPI_write_cmos_sensor(0x3a10, 0x40);
            OV2675_MIPI_write_cmos_sensor(0x3a1e, 0x3E);
            OV2675_MIPI_write_cmos_sensor(0x3a1f, 0x18);
            break;          
/*                          
        case AE_EV_COMP_03: 
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x41);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x10);
            break;
*/
        case AE_EV_COMP_10:
            OV2675_MIPI_write_cmos_sensor(0x3a0f, 0x58);
            OV2675_MIPI_write_cmos_sensor(0x3a10, 0x50);
            OV2675_MIPI_write_cmos_sensor(0x3a11, 0xa0);
            OV2675_MIPI_write_cmos_sensor(0x3a1b, 0x5a);
            OV2675_MIPI_write_cmos_sensor(0x3a1e, 0x4e);
            OV2675_MIPI_write_cmos_sensor(0x3a1f, 0x28);          
            break;
/*
        case AE_EV_COMP_10:
            OV2675_MIPI_write_cmos_sensor(0x3391, temp_reg|0x4);
            OV2675_MIPI_write_cmos_sensor(0x3390, 0x41);
            OV2675_MIPI_write_cmos_sensor(0x339a, 0x28);/* for difference */
         /*   break;*/

        case AE_EV_COMP_20:
            OV2675_MIPI_write_cmos_sensor(0x3a0f, 0x70);
            OV2675_MIPI_write_cmos_sensor(0x3a10, 0x68);
            OV2675_MIPI_write_cmos_sensor(0x3a11, 0xB8);
            OV2675_MIPI_write_cmos_sensor(0x3a1b, 0x72);
            OV2675_MIPI_write_cmos_sensor(0x3a1e, 0x66);
            OV2675_MIPI_write_cmos_sensor(0x3a1f, 0x40);
            break;

        default:
            return FALSE;
    }
#endif

    return TRUE;
} /* OV2675_MIPI_set_param_exposure */
#if 1
void OV2675_MIPI_SetBrightness(UINT16 para)
{
    //spin_lock(&s5k4ecgx_mipi_rw_lock);
    //S5K4ECGX_write_cmos_sensor(0xFCFC ,0xD000);  
    //S5K4ECGX_write_cmos_sensor(0x0028,0x7000);
    //S5K4ECGX_write_cmos_sensor(0x002A ,0x0230);  
 
    switch (para)
    {
        case ISP_BRIGHT_LOW:
			SENSORDB("OV2675_MIPI_SetBrightness:ISP_BRIGHT_LOW\n");
             break; 
             
        case ISP_BRIGHT_HIGH:
			SENSORDB("OV2675_MIPI_SetBrightness:ISP_BRIGHT_HIGH\n");
             break; 
             
        case ISP_BRIGHT_MIDDLE:
			SENSORDB("OV2675_MIPI_SetBrightness:ISP_BRIGHT_MIDDLE\n");
			       break; 

        default:
             break; 
    }
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);
    return;
}
#endif


void OV2675_MIPI_SetContrast(UINT16 para)
{
    //spin_lock(&s5k4ecgx_mipi_rw_lock);
 
    switch (para)
    {
        case ISP_CONTRAST_LOW:
             break; 
        case ISP_CONTRAST_HIGH:
             break; 
        case ISP_CONTRAST_MIDDLE:
             break; 
        default:
             break; 
    }
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);	
    return;
}



void OV2675_MIPI_SetSetIso(UINT16 para)
{
    //spin_lock(&s5k4ecgx_mipi_rw_lock);
    switch (para)
    {
        case AE_ISO_100:
             //ISO 100
            break; 
        case AE_ISO_200:
             //ISO 200
            break; 
        default:
        case AE_ISO_AUTO:
             //ISO Auto
            
			// OV2675_MIPI_write_cmos_sensor(0x3015, 0x01);  
             break; 
    }	
    //spin_unlock(&s5k4ecgx_mipi_rw_lock);	
}


void OV2675_MIPI_SetSaturation(UINT16 para)
{
    switch (para)
    {
        case ISP_SAT_HIGH:
             break; 
        case ISP_SAT_LOW:
             break; 
        case ISP_SAT_MIDDLE:
             break; 
        default:
             break; 
    }	
}

void OV2675_GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    OV2675_MIPI_get_exposure_gain();
    pExifInfo->FNumber = 27;
    pExifInfo->AEISOSpeed = OV2675_CurrentStatus.iGain;
    pExifInfo->AWBMode = OV2675_CurrentStatus.iWB;
    pExifInfo->CapExposureTime = OV2675_CurrentStatus.iShutter;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = OV2675_CurrentStatus.iGain;
}	

UINT32 OV2675MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
//   if( OV2675_MIPI_sensor_cap_state == KAL_TRUE)
//	   return TRUE;

	switch (iCmd) {
	case FID_SCENE_MODE:	
//	    printk("Set Scene Mode:%d\n", iPara); 
	    if (iPara == SCENE_MODE_OFF||iPara == SCENE_MODE_NORMAL)
	    {
	        if(OV2675_MIPI_sensor_zsd_mode==KAL_TRUE)
	        OV2675_MIPI_night_ZSD_mode(0); 
			else
				
	        OV2675_MIPI_night_mode(0); 
			

	    }
	    else if (iPara == SCENE_MODE_NIGHTSCENE)
	    {
	    
		if(OV2675_MIPI_sensor_zsd_mode==KAL_TRUE)
	        OV2675_MIPI_night_ZSD_mode(1); 
		else
               OV2675_MIPI_night_mode(1); 		
			
	    }	    
	    break; 	    
	case FID_AWB_MODE:
//	    printk("Set AWB Mode:%d\n", iPara); 	    
           OV2675_MIPI_set_param_wb(iPara);
	break;
	case FID_COLOR_EFFECT:
//	    printk("Set Color Effect:%d\n", iPara); 	    	    
           OV2675_MIPI_set_param_effect(iPara);
	break;
	case FID_AE_EV:
#if WINMO_USE	    
	case ISP_FEATURE_EXPOSURE:
#endif 	    
//           printk("Set EV:%d\n", iPara); 	    	    
           OV2675_MIPI_set_param_exposure(iPara);
	break;
	case FID_AE_FLICKER:
//           printk("Set Flicker:%d\n", iPara); 	    	    	    
           OV2675_MIPI_set_param_banding(iPara);
	break;
    case FID_AE_SCENE_MODE: 
            if (iPara == AE_MODE_OFF) {
		spin_lock(&ov2675_drv_lock);
               // OV2675_MIPI_AE_ENABLE = KAL_FALSE; 
                OV2675_MIPI_AE_ENABLE = KAL_TRUE;
		spin_unlock(&ov2675_drv_lock);
            }
            else {
		spin_lock(&ov2675_drv_lock);
                OV2675_MIPI_AE_ENABLE = KAL_TRUE; 
		spin_unlock(&ov2675_drv_lock);
	    }
            OV2675_MIPI_set_AE_mode(OV2675_MIPI_AE_ENABLE);
            break; 
	case FID_ZOOM_FACTOR:
		spin_lock(&ov2675_drv_lock);
		zoom_factor = iPara; 
		spin_unlock(&ov2675_drv_lock);
        break; 
		//case FID_AE_SCENE_MODE: 
		//	OV2675_MIPI__set_AE_mode(iPara);
		//	break; 
			case FID_ISP_CONTRAST:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_CONTRAST:%d\n",iPara);
				OV2675_MIPI_SetContrast(iPara);
				break;
			case FID_ISP_BRIGHT:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_BRIGHT:%d\n",iPara);
				OV2675_MIPI_SetBrightness(iPara);
				break;
			case FID_ISP_SAT:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_ISP_SAT:%d\n",iPara);
				OV2675_MIPI_SetSaturation(iPara);
				break;
			/*case FID_AE_ISO:
				SENSORDB("S5K4ECGX_MIPISensorSetting func:FID_AE_ISO:%d\n",iPara);
				OV2675_MIPI_SetSetIso(iPara);
				break;*/
	default:
	break;
	}
	return TRUE;
}   /* OV2675MIPIYUVSensorSetting */

UINT32 OV2675MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
    kal_uint8 iTemp;
    /* to fix VSYNC, to fix frame rate */
    //printk("Set YUV Video Mode \n");  
    iTemp = OV2675_MIPI_read_cmos_sensor(0x3014);
    OV2675_MIPI_write_cmos_sensor(0x3014, iTemp&0xf7); //Disable night mode

    if (u2FrameRate == 30)
    {
     }
    else if (u2FrameRate == 15)       
    {
     }
    else 
    {
        printk("Wrong frame rate setting \n");
    }

	spin_lock(&ov2675_drv_lock);
	OV2675_MIPI_VEDIO_encode_mode = KAL_TRUE;
	spin_unlock(&ov2675_drv_lock);
        
    return TRUE;
}

UINT32 OV2675MIPIYUVSetSoftwarePWDNMode(kal_bool bEnable)
{
    SENSORDB("[OV2675MIPIYUVSetSoftwarePWDNMode] Software Power down enable:%d\n", bEnable);
    /*if(bEnable) {   // enable software power down mode   
	     OV2675_MIPI_write_cmos_sensor(0x3086, 0x01);
    } else {
       OV2675_MIPI_write_cmos_sensor(0x3086, 0x00);  
    }*/
    return TRUE;
}
/*************************************************************************
* FUNCTION
*OV2675MIPIClose
*
* DESCRIPTION
* This OV2675SetMaxFramerateByScenario is to turn off sensor module power.
*
* PARAMETERS
* None
*
* RETURNS
* None
*
* GLOBALS AFFECTED
*
*************************************************************************/
  UINT32 OV2675MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	SENSORDB("OV2675SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	/*switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = 134200000;
			lineLength = IMX111MIPI_PV_LINE_LENGTH_PIXELS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - IMX111MIPI_PV_FRAME_LENGTH_LINES;
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	*/
	return ERROR_NONE;
}
  /*************************************************************************
  * FUNCTION
  * OV2675GetDefaultFramerateByScenario
  *
  * DESCRIPTION
  * This function is to turn off sensor module power.
  * RETURNS
  * None
  *
  * GLOBALS AFFECTED
  *
  *************************************************************************/
UINT32 OV2675MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 220;
			break;		//hhl 2-28
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

  	}

UINT32 OV2675FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

#if WINMO_USE	
	PMSDK_FEATURE_INFO_STRUCT pSensorFeatureInfo=(PMSDK_FEATURE_INFO_STRUCT) pFeaturePara;
#endif 

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=OV2675_MIPI_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=OV2675_MIPI_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:
			*pFeatureReturnPara16++=OV2675_MIPI_PV_PERIOD_PIXEL_NUMS+OV2675_MIPI_PV_dummy_pixels;
			*pFeatureReturnPara16=OV2675_MIPI_PV_PERIOD_LINE_NUMS+OV2675_MIPI_PV_dummy_lines;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			*pFeatureReturnPara32 = OV2675_MIPI_sensor_pclk/10;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			OV2675_MIPI_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&ov2675_drv_lock);
			OV2675_MIPI_isp_master_clock=*pFeatureData32;
			spin_unlock(&ov2675_drv_lock);
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			OV2675_MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = OV2675_MIPI_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &OV2675SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			 OV2675_MIPI_GetSensorID(pFeatureData32);
			 break;
		case SENSOR_FEATURE_SET_YUV_CMD:
//		       printk("OV2675 MIPI YUV sensor Setting:%d, %d \n", *pFeatureData32,  *(pFeatureData32+1));
			OV2675MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		       OV2675MIPIYUVSetVideoMode(*pFeatureData16);
		       break; 
	        case SENSOR_FEATURE_SET_SOFTWARE_PWDN:
	            OV2675MIPIYUVSetSoftwarePWDNMode((BOOL)*pFeatureData16);        	        	
	            break;
			//	case SENSOR_CMD_SET_VIDEO_FRAME_RATE:
			//		break;
				case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
					OV2675GetAEAWBLock((*pFeatureData32),*(pFeatureData32+1));
					break;
					case SENSOR_FEATURE_SET_YUV_3A_CMD:
					   OV2675_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
					   break;
				case SENSOR_FEATURE_GET_DELAY_INFO:
					SENSORDB("SENSOR_FEATURE_GET_DELAY_INFO\n");
					OV2675GetDelayInfo(*pFeatureData32);
					break;
				case SENSOR_FEATURE_AUTOTEST_CMD:
					SENSORDB("SENSOR_FEATURE_AUTOTEST_CMD\n");
					OV2675AutoTestCmd((*pFeatureData32),*(pFeatureData32+1));
					break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			OV2675_GetExifInfo(*pFeatureData32);
			break;
		default:
			break;			
	}
	return ERROR_NONE;
}	/* OV2675FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncOV2675=
{
	OV2675Open,
	OV2675GetInfo,
	OV2675GetResolution,
	OV2675FeatureControl,
	OV2675Control,
	OV2675Close
};

UINT32 OV2675_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncOV2675;

	return ERROR_NONE;
}	/* SensorInit() */



