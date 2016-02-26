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
#ifndef _ILI9488C_HVGA_DSI_VDO_C_
#define _ILI9488C_HVGA_DSI_VDO_C_
/*****************************************************************************
** LCM model
**    DSBJ D0305FA00_V01 3.47" 320RGBx480 262K-color TFT-LCD
**    ILITEK ILI9488C a-Si TFT LCD Single chip driver.
**  Fosc:
**  Bit rate:
**  Frame rate:
******************************************************************************/
#if defined( BUILD_LK )
    #include  <platform/mt_gpio.h>
    #include  <platform/mt_typedefs.h>
  //#include  <string.h>
#elif defined( BUILD_UBOOT )
    #include  <asm/arch/mt_typedefs.h>
    #include  <asm/arch/mt_gpio.h>
    #include  <linux/string.h>
#else
    #include  <linux/kernel.h>
    #include  <mach/mt_typedefs.h>
    #include  <mach/mt_gpio.h>
    #include  <linux/string.h>
#endif /** End.. (BUILD_LK) **/
  //#include  <cust_gpio_boot.h>
    #include  "lcm_drv.h"

/*****************************************************************************
** Local Macro-defined Variables
******************************************************************************/
  #ifndef TRUE
    #define   TRUE          1
  #endif

  #ifndef FALSE
    #define   FALSE         0
  #endif

  /*=======================================================================
  **  For information and debug
  **=======================================================================*/
  #if defined( LCM_MSG_ENABLE )
    #undef    LCM_MSG_ENABLE
  #endif
  //#define   LCM_MSG_ENABLE

  #if defined( LCM_DEBUG_ENABLE )
    #undef    LCM_DEBUG_ENABLE
  #endif
  //#define   LCM_DEBUG_ENABLE

  #if defined( BUILD_UBOOT )
    #ifndef KERN_INFO
      #define   KERN_INFO
    #endif
  #endif

  #if defined( LCM_PRINT )
    #undef    LCM_PRINT
  #endif
  #if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    #define   LCM_PRINT                 printf
  #else
    #define   LCM_PRINT                 printk
  #endif /** End.. (BUILD_LK) **/

  #if defined( LCM_MSG_ENABLE )
    #define   LCM_MSG(srt,arg...)       LCM_PRINT(srt,##arg)
  #else
    #define   LCM_MSG(srt,arg...)       {/*Do Nothing*/}
  #endif

  #if defined( LCM_DEBUG_ENABLE )
    #define   LCM_DBG(srt,arg...)       LCM_PRINT(srt,##arg)
  #else
    #define   LCM_DBG(srt,arg...)       {/*Do Nothing*/}
  #endif

  /*=======================================================================
  ** Local Constants
  **=======================================================================*/
  #if defined( LCM_DSI_CMD_MODE )
    #undef    LCM_DSI_CMD_MODE
  #endif
  //#define   LCM_DSI_CMD_MODE

  #if defined( LCM_TE_VSYNC_MODE )
    #undef    LCM_TE_VSYNC_MODE
  #endif
  //#define   LCM_TE_VSYNC_MODE

  #if defined( LCM_ID_PIN_ENBLE )
    #undef    LCM_ID_PIN_ENBLE
  #endif
//#if defined( ARIMA_LO2_HW )
  #if 1 //defined( LO2_EP0 ) || defined( LO2_EP1 )
    #define   LCM_ID_PIN_ENBLE
  #endif
//#endif

  #if defined( LCM_PWM_BL_PIN_ENBLE )
    #undef    LCM_PWM_BL_PIN_ENBLE
  #endif
    #define   LCM_PWM_BL_PIN_ENBLE

    #define   FRAME_WIDTH               (320)
    #define   FRAME_HEIGHT              (480)

    #define   REGFLAG_DELAY             0xFE
    #define   REGFLAG_END_OF_TABLE      0xFF  /* END OF REGISTERS MARKER */

#if defined( LCM_ID_PIN_ENBLE )
  #if !defined( GPIO_LCM_ID_PIN )
    #define   GPIO_LCM_ID_PIN           GPIO58
  #endif
  #if !defined( GPIO_LCM_ID_PIN_M_GPIO )
    #define   GPIO_LCM_ID_PIN_M_GPIO    GPIO_MODE_00
  #endif
  #if defined( GPIO_LCM_ID_PIN_MODE )
    #undef    GPIO_LCM_ID_PIN_MODE
  #endif
    #define   GPIO_LCM_ID_PIN_MODE      GPIO_LCM_ID_PIN_M_GPIO

  /** For LCM panel resource selection **/
    #define   LCM_MAIN_SOURCE           1   /* For main source panel */
    #define   LCM_SECOND_SOURCE         0   /* For second source panel */

  #if defined( LCM_PANEL_RESOURCE )
    #undef    LCM_PANEL_RESOURCE
  #endif
    #define   LCM_PANEL_RESOURCE        LCM_SECOND_SOURCE
#endif /* End.. (LCM_ID_PIN_ENBLE) */

#if defined( LCM_PWM_BL_PIN_ENBLE )
  #if !defined( GPIO_PWM_BL_PIN )
    #define   GPIO_PWM_BL_PIN           GPIO22
  #endif
  #if !defined( GPIO_PWM_BL_PIN_M_PWM )
    #define   GPIO_PWM_BL_PIN_M_PWM     GPIO_MODE_02//GPIO22_MODE// /* PWM_BL mode */
  #endif
  #if defined( GPIO_PWM_BL_PIN_MODE )
    #undef    GPIO_PWM_BL_PIN_MODE
  #endif
    #define   GPIO_PWM_BL_PIN_MODE      GPIO_PWM_BL_PIN_M_PWM
#endif /* End.. (LCM_PWM_BL_PIN_ENBLE) */

  /*=========================================================================
  ** DSI Video Mode
  **=========================================================================
  ** VRR: 320x480, 60Hz
  **-------------------------------------------------------------------------
  ** 
  **=========================================================================*/
    #define   LCM_HSYNC_NUM             (10)  /** HSPW: PCLK. Shall be larger than 3 **/
    #define   LCM_HBP_NUM               (200) /** HBPD: PCLK. 5~78 **/
    #define   LCM_HFP_NUM               (255) /** HFPD: PCLK. 5~88 **/

    #define   LCM_VSYNC_NUM             (8)
    #define   LCM_VBP_NUM               (16)  /** VBPD. **/
    #define   LCM_VFP_NUM               (8)   /** VFPD. **/

    #define   LCM_LINE_BYTE             ((FRAME_WIDTH+LCM_HSYNC_NUM+LCM_HBP_NUM+LCM_HFP_NUM)*3)

  /*======================================================================
  ** ref freq = 13 or 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
  ** pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
  **======================================================================*/
    #define   LCM_PWM_CLK               (23706)

  /*=======================================================================
  ** READID1~3 (DAh~DCh): Read ID1~3
  **-----------------------------------------------------------------------
  ** ID1: The LCD module's manufacturer. Defined by customer. Default: 0x00
  ** ID2: The LCD module/driver version. Defined by customer. Default: 0x00
  ** ID3: The LCD module/driver. Defined by customer. Default: 0x00
  **=======================================================================*/
    #define   LCM_ID_ILI9488C           0x9488

    #define   LCM_ID0                   (0x79)
    #define   LCM_ID1                   (0x00)
    #define   LCM_ID2                   (0x00)
    #define   LCM_ID3                   (0x00)

/*****************************************************************************
** Local Variables
******************************************************************************/
    static  LCM_UTIL_FUNCS    lcm_util = { 0 };

/*****************************************************************************
** Local Functions
******************************************************************************/
    #define   SET_RESET_PIN(v)          (lcm_util.set_reset_pin((v)))

    #define   UDELAY(n)                 (lcm_util.udelay(n))
    #define   MDELAY(n)                 (lcm_util.mdelay(n))

    #define   dsi_set_cmdq(pdata, queue_size, force_update) \
                                        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
    #define   dsi_set_cmdq_V2(cmd, count, ppara, force_update)  \
                                        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
    #define   dsi_set_cmdq_V3(para_tbl,size,force_update)   \
                                        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
    #define   wrtie_cmd(cmd)            lcm_util.dsi_write_cmd(cmd)
    #define   write_regs(addr, pdata, byte_nums)  \
                                        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
    #define   read_reg                  lcm_util.dsi_read_reg()
    #define   read_reg_v1(cmd)          lcm_util.dsi_dcs_read_lcm_reg(cmd)
    #define   read_reg_v2(cmd, buffer, buffer_size) \
                                        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

    #define   SET_GPIO_MODE(v,m)        lcm_util.set_gpio_mode(v,m)
    #define   SET_GPIO_DIR_OUT(v)       lcm_util.set_gpio_dir(v,GPIO_DIR_OUT)
    #define   SET_GPIO_OUT(v,m)         lcm_util.set_gpio_out(v,GPIO_DIR_OUT)
    #define   SET_GPIO_DIR_IN(v)        lcm_util.set_gpio_dir(v,GPIO_DIR_IN)
    #define   SET_GPIO_PULL_ENABLE(v)   lcm_util.set_gpio_pull_enable(v,GPIO_PULL_ENABLE)

/*****************************************************************************
** External Functions
******************************************************************************/
#if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    extern void   DSI_clk_HS_mode(BOOL enter);
#endif

/*****************************************************************************
**
******************************************************************************/
#if 0
  typedef static struct LCM_setting_table
  {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
  } LCM_SET_TAB;
#endif

/*****************************************************************************
**  Note :
******************************************************************************
** Data ID will depends on the following rule.
**----------------------------------------------------------------------------
** count of parameters > 1  => Data ID = 0x39
** count of parameters = 1  => Data ID = 0x15
** count of parameters = 0  => Data ID = 0x05
**============================================================================
** Structure Format :
**----------------------------------------------------------------------------
** {DCS command, count of parameters, {parameter list}}
** {REGFLAG_DELAY, milliseconds of time, {}},
** ...
**
**============================================================================
** Setting ending by predefined flag
**----------------------------------------------------------------------------
** {REGFLAG_END_OF_TABLE, 0x00, {}}
******************************************************************************/
static LCM_setting_table_V3  lcm_initialization_setting[] =
{
#if 0
    { 0x15, 0xED, 1, { 0x20 }},
    { 0x39, 0xC0, 2, { 0x17, 0x15 }},
    { 0x15, 0xC1, 1, { 0x44 }},
    { 0x39, 0xC5, 3, { 0x01, 0x0F, 0x81 }},
    { 0x15, 0x36, 1, { 0x08 }},
    { 0x15, 0x3A, 1, { 0x66 }},
    { 0x15, 0xB1, 1, { 0xA0 }},
    { 0x15, 0xB4, 1, { 0x02 }},
    { 0x39, 0xB6, 2, { 0xB2, 0x22 }},
    { 0x05, 0x11, 0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {}},
    { 0x05, 0x29,  0, {}},
#else
  #if 0
    { 0x39, 0xBC,  2, { 0x10, 0x00 }},
    { 0x39, 0xBF, 16, { 0xE0, 0x00, 0x03, 0x0C, 0x09, 0x17, 0x09, 0x3E, 0x89, 0x49,
                        0x08, 0x0D, 0x0A, 0x13, 0x15, 0x0F }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
    { 0x39, 0xBC,  2, { 0x10, 0x00}},
    { 0x39, 0xBF, 16, { 0xE1, 0x00, 0x11, 0x15, 0x03, 0x0F, 0x05, 0x2D, 0x34, 0x41,
                        0x02, 0x0B, 0x0A, 0x33, 0x37, 0x0F }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
    { 0x39, 0xBC,  2, { 0x03, 0x00 }},
    { 0x39, 0xBF,  3, { 0xC0, 0x17, 0x15 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 1, {}},
    { 0x39, 0xBC,  2, { 0x02, 0x00 }},
    { 0x39, 0xBF,  2, { 0xC1, 0x41 }},
    { 0x39, 0xBC,  2, { 0x04, 0x00 }},
    { 0x39, 0xBF,  4, { 0xC5, 0x00, 0x12,0x80 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 1, {}},
    { 0x39, 0xBC,  2, { 0x02, 0x00 }},
    { 0x39, 0xBF,  2, { 0x36, 0x00 }}, //{ 0xBF, 2, { 0x36, 0x48 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 5, {}},
    { 0x39, 0xBC,  2, { 0x02, 0x00 }},
    { 0x39, 0xBF,  2, { 0x3A, 0x66 }},
    { 0x39, 0xBC,  2, { 0x02, 0x00 }},
    { 0x39, 0xBF,  2, { 0xB0, 0x00 }},
    { 0x39, 0xBC,  2, { 0x02, 0x00 }},
    { 0x39, 0xBF,  2, { 0xB1, 0xA0 }},
    { 0x39, 0xBC,  2, { 0x02, 0x00 }},
    { 0x39, 0xBF,  2, { 0xB4, 0x02 }},
    { 0x39, 0xBC,  2, { 0x03, 0x00 }},
    { 0x39, 0xBF,  3, { 0xB6, 0x02, 0x42 }},//{0xBF, 3, {0xB6, 0x02, 0x02}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 1, {}},
    { 0x39, 0xBC,  2, { 0x02, 0x00 }},
    { 0x39, 0xBF,  2, { 0xE9, 0x00 }},
    { 0x39, 0xBC,  2, { 0x05, 0x00 }},
    { 0x39, 0xBF,  5, { 0xF7, 0xA9, 0x51, 0x2C,0x82 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
    { 0x39, 0xBC,  2, { 0x01, 0x00 }},
    { 0x15, 0xBF,  1, { 0x11 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {}},
    { 0x39, 0xBC,  2, { 0x01, 0x00 }},
    { 0x15, 0xBF,  1, { 0x29 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {}},
  #else
  /* PGAMCTRL: Positive Gamma Control */
    { 0x39, 0xE0, 15, { 0x00, 0x03, 0x0C, 0x09, 0x17, 0x09, 0x3E, 0x89, 0x49, 0x08,
                        0x0D, 0x0A, 0x13, 0x15, 0x0F }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* NGAMCTRL: Negative Gamma Control */
    { 0x39, 0xE1, 15, { 0x00, 0x11, 0x15, 0x03, 0x0F, 0x05, 0x2D, 0x34, 0x41, 0x02,
                        0x0B, 0x0A, 0x33, 0x37, 0x0F }},
    {  REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* Power Control 1 */
    { 0x39, 0xC0,  2,  { 0x17, 0x15 }},
  /* Power Control 2 */
    { 0x15, 0xC1,  1,  { 0x41 }},
  /* VCOM Control */
    { 0x39, 0xC5,  3,  { 0x00, 0x12,0x80 }},
  /* VCOM Control */
    { 0x15, 0x36,  1,  { 0x48 }},
  /* Interface Pixel Format */
    { 0x15, 0x3A,  1,  { 0x66 }}, /* 18 Bits/Pixel */
  /* Interface Mode Control */
    { 0x15, 0xB0,  1,  { 0x00 }}, /* SDA and SDO pins, VSPL: Low level, HSPL: Low level, DPL: Rise time, EPL: High enable. */
  /* Frame Rate Control */
    { 0x15, 0xB1,  1,  { 0xA0 }}, /* Fosc */
  /* Display Inversion Control */
    { 0x15, 0xB4,  1,  { 0x02 }}, /* 2 dot inversion */
  /* Blanking Porch Control */
  //{ 0x39, 0xB5,  4,  { LCM_VFP_NUM, LCM_VBP_NUM, LCM_HFP_NUM, LCM_HBP_NUM }},
  /* Display Function Control */
    { 0x39, 0xB6,  2,  { 0x02, 0x02 }}, /* DE mode, S1->S960, G1->G480, SM */
  /* Set image Function */
    { 0x15, 0xE9,  1,  { 0x00 }}, /* Disable 24-bits date bus */
  /* Adjust Control 5 */
    { 0x39, 0xF7,  4,  { 0xA9, 0x51, 0x2C, 0x82 }}, /* DSI 18bit option: stream packet RGB 666 */
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /****************************************************************************
  ** Note:
  **===========================================================================
  ** Strongly recommend not to set Sleep out / Display On here. That will cause
  ** messed frame to be shown as later the backlight is on.
  ******************************************************************************/
  /* Sleep out */
    { 0x05, 0x11,  0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {}},
  /* Display on */
    { 0x05, 0x29,  0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  #endif
#endif
  /* Setting ending by predefined flag */
  /*{ REGFLAG_END_OF_TABLE, 0x00, {}} */
};

#if defined( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3 lcm_set_window[] =
{
    { 0x39, 0x2A,  4, { 0x00, 0x00, (FRAME_WIDTH >>8), (FRAME_WIDTH &0xFF)}},
    { 0x39, 0x2B,  4, { 0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif /* End.. (LCM_DSI_CMD_MODE) */

static LCM_setting_table_V3 lcm_sleep_out_setting[] =
{
    { 0x05, 0x00, 0, {}},
  /* Sleep Out */
    { 0x05, 0x11, 0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {}},
  /* Display ON */
    { 0x05, 0x29, 0, {}},
  //{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static LCM_setting_table_V3 lcm_sleep_in_setting[] =
{
  /* Display off sequence */
    { 0x05, 0x28, 0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 120, {}},
    /* Sleep Mode On */
    { 0x05, 0x10, 0, {}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if defined( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3   lcm_backlight_set[] = {
  /* WRDISBV: Write display brightness */
    { 0x15, 0x51,  1, { 0xFF }},
  /* End */
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif /* End.. (LCM_DSI_CMD_MODE) */

/******************************************************************************
** LCM Driver Implementations
** Local Functions
*******************************************************************************/
/******************************************************************************
** push_table
*******************************************************************************/
static void push_table(LCM_setting_table_V3 *table, unsigned int count, unsigned char force_update )
{
    dsi_set_cmdq_V3( table, count, force_update );
} /* End.. push_table() */

/******************************************************************************
** lcm_set_util_funcs
*******************************************************************************/
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

/******************************************************************************
** lcm_get_params
*******************************************************************************/
static void lcm_get_params(LCM_PARAMS *params)
{
    memset( params, 0, sizeof( LCM_PARAMS ));
    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

#if defined( LCM_DSI_CMD_MODE )
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
  /**** DSI ****/
  /* Command mode setting */
    params->dsi.LANE_NUM        = LCM_ONE_LANE;
  /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB; //LCM_COLOR_ORDER_BGR;//
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
  /* Highly depends on LCD driver capability.
  ** Not support in MT6573 */
    params->dsi.packet_size = 256;
  /* Video mode setting */
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.vertical_sync_active    = LCM_VSYNC_NUM;  /* Line */
    params->dsi.vertical_backporch      = LCM_VBP_NUM;
    params->dsi.vertical_frontporch     = LCM_VFP_NUM;
    params->dsi.vertical_active_line    = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active  = LCM_HSYNC_NUM;  /* PCLK */
    params->dsi.horizontal_backporch    = LCM_HBP_NUM;
    params->dsi.horizontal_frontporch   = LCM_HFP_NUM;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

  /*===========================================================================
  ** Bit rate calculation
  **---------------------------------------------------------------------------
  ** fref=26MHz, fvco=fref*(div1+1) (div1=0~63, fvco=500MHZ~1GHz)
  ** div2=0~15: fout=fvco/(2*div2) (Mbps)
  **---------------------------------------------------------------------------
  ** cycle_time = (8*1000*(div2*2))/(26*(div1+1))
  ** ui = (1000*(div2*2))/(26*(div1+1)) + 1
  **===========================================================================*/
  #if 1
  /** The best PLL_CLOCK is setting the multiple of 26. **/
  //params->dsi.pll_select  = 1;  /* 0: MIPI_PLL; 1: LVDS_PLL */
    params->dsi.PLL_CLOCK = 234;  //LCM_DSI_6589_PLL_CLOCK_201_5;
  #else
    params->dsi.pll_div1  = 24;
    params->dsi.pll_div2  = 1;
  #endif

  /**************************************************************************
  ** ESD or noise interference recovery For video mode LCM only.
  ***************************************************************************/
#if !defined( LCM_DSI_CMD_MODE )
  /* Send TE packet to LCM in a period of n frames and check the response. */
    params->dsi.lcm_int_te_monitor    = FALSE;
    params->dsi.lcm_int_te_period     = 1;  /* Unit : frames */
  /* Need longer FP for more opportunity to do int. TE monitor applicably. */
    if( params->dsi.lcm_int_te_monitor )
      params->dsi.vertical_frontporch *= 2;
  /* Monitor external TE (or named VSYNC) from LCM once per 2 sec.
  ** (LCM VSYNC must be wired to baseband TE pin.) */
    params->dsi.lcm_ext_te_monitor    = FALSE;
  /* Non-continuous clock */
    params->dsi.noncont_clock         = FALSE;
    params->dsi.noncont_clock_period  = 2; /* Unit : frames */
#endif
}

/***********************************************************************
** lcm_compare_id
************************************************************************/
static unsigned int lcm_compare_id(void)
{
unsigned char buffer[3];
unsigned int  id = 0, vRet = 0;
unsigned int  array[16];

#if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
  #if defined( LCM_ID_PIN_ENBLE )
    SET_GPIO_MODE( GPIO_LCM_ID_PIN, GPIO_LCM_ID_PIN_MODE );
    SET_GPIO_DIR_IN( GPIO_LCM_ID_PIN );
  #endif /* End.. (LCM_ID_PIN_ENBLE) */
  #if defined( LCM_PWM_BL_PIN_ENBLE )
    SET_GPIO_MODE( GPIO_PWM_BL_PIN, GPIO_PWM_BL_PIN_MODE );
    SET_GPIO_DIR_OUT( GPIO_PWM_BL_PIN );
  //SET_GPIO_OUT( GPIO_PWM_BL_PIN, GPIO_OUT_ONE );
  #endif /* End.. (LCM_PWM_BL_PIN_ENBLE) */
#endif
  /* NOTE: Should reset LCM firstly */
    SET_RESET_PIN( 1 );
    SET_RESET_PIN( 0 );
    MDELAY(1 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 ); /* Must over 6 ms */

    array[0] = 0x00033700;  /* return byte number 3 */
    dsi_set_cmdq( &array, 1, 1 );
    MDELAY(10);

    read_reg_v2( 0xD3, buffer, 3 );
    id = ( buffer[1] << 8 ) | buffer[2];

#if defined( BUILD_UBOOT ) || defined( BUILD_LK )
    printf( "%s, buffer[0] = 0x%08X\n", __func__, buffer[0] );
    printf( "%s, buffer[1] = 0x%08X\n", __func__, buffer[1] );
    printf( "%s, buffer[2] = 0x%08X\n", __func__, buffer[2] );

    printf( "%s, id = 0x%08X\n", __func__, id );
#endif

    if( LCM_ID_ILI9488C == id )
    {
    #if defined( LCM_MSG_ENABLE )
      LCM_MSG( "[ILI9488C_DSI] %s -- Check ID OK.\n", __func__ );
    #endif
      vRet = 1;
    }
    else
    {
    #if defined( LCM_ID_PIN_ENBLE )
      if( LCM_PANEL_RESOURCE == mt_get_gpio_in( GPIO_LCM_ID_PIN ))
      {
      #if defined( LCM_MSG_ENABLE )
        LCM_MSG( "[ILI9488C_DSI] %s -- LCM_ID GPIO OK.\n", __func__ );
      #endif
        vRet = 1;
      }
    #endif /* End.. (LCM_ID_PIN_ENBLE) */
    }

    return  vRet;
}

/***********************************************************************
** lcm_init
************************************************************************/
static void lcm_init(void)
{
char  buffer[3];
int   array[4];

  //lcm_compare_id();

    SET_RESET_PIN( 1 );
    MDELAY( 1 );
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY( 120 );

    push_table( lcm_initialization_setting, sizeof( lcm_initialization_setting ) / sizeof( lcm_initialization_setting[0] ), 1 );
}

/***********************************************************************
** lcm_suspend
************************************************************************/
static void lcm_suspend(void)
{
    push_table( lcm_sleep_in_setting, sizeof( lcm_sleep_in_setting ) / sizeof( lcm_sleep_in_setting[0] ), 1 );
}

/***********************************************************************
** lcm_resume
************************************************************************/
static void lcm_resume(void)
{
    push_table( lcm_sleep_out_setting, sizeof( lcm_sleep_out_setting ) / sizeof( lcm_sleep_out_setting[0] ), 1 );
}

#if defined( LCM_DSI_CMD_MODE )
/***********************************************************************
** lcm_update
************************************************************************/
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
unsigned int  x0 = x;
unsigned int  y0 = y;
unsigned int  x1 = x0 + width - 1;
unsigned int  y1 = y0 + height - 1;

unsigned char x0_MSB = ((x0>>8)&0xFF);
unsigned char x0_LSB = (x0&0xFF);
unsigned char x1_MSB = ((x1>>8)&0xFF);
unsigned char x1_LSB = (x1&0xFF);
unsigned char y0_MSB = ((y0>>8)&0xFF);
unsigned char y0_LSB = (y0&0xFF);
unsigned char y1_MSB = ((y1>>8)&0xFF);
unsigned char y1_LSB = (y1&0xFF);

unsigned int data_array[16];

  data_array[0] = 0x00053902;
  data_array[1] = (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2A;
  data_array[2] = (x1_LSB);
  data_array[3] = 0x00053902;
  data_array[4] = (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2B;
  data_array[5] = (y1_LSB);
  data_array[6] = 0x002C3909;

  dsi_set_cmdq( &data_array, 7, 0 );
}

/***********************************************************************
** lcm_set_backlight
************************************************************************/
static void lcm_set_backlight(unsigned int level)
{
unsigned int  default_level = 145;
unsigned int  mapped_level  = 0;

  /* For LGE backlight IC mapping table */
    if( level > 255 )
      level = 255;

    if( level > 0 )
      mapped_level = default_level + ( level )*( 255 - default_level ) / ( 255 );
    else
      mapped_level = 0;

  /* Refresh value of backlight level. */
    lcm_backlight_set[0].para_list[0] = (unsigned char)mapped_level;
    push_table( lcm_backlight_set, sizeof( lcm_backlight_set ) / sizeof( lcm_backlight_set[0] ), 1 );
} /* End.. lcm_set_backlight() */
#endif /* End.. (LCM_DSI_CMD_MODE) */

/***********************************************************************
** lcm_update
************************************************************************/
LCM_DRIVER ili9488c_hvga_dsi_vdo_drv =
{
    .name     = "ili9488c_hvga_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#if defined( LCM_DSI_CMD_MODE )
    .set_backlight  = lcm_setbacklight,
    .update         = lcm_update,
#endif
};

/*****************************************************************************
** End
******************************************************************************/
#undef _ILI9488C_HVGA_DSI_VDO_C_
#endif /** End.. !(_ILI9488C_HVGA_DSI_VDO_C_) **/
