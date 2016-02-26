/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of ARIMA Inc. (C) 2014
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
#ifndef _HX8357C_HVGA_DSI_VDO_C_
#define _HX8357C_HVGA_DSI_VDO_C_
/*****************************************************************************
** LCM model
**    Truly CSHMDS9960QR-B 3.47" 320RGBx480 dot, 16.7M-color TFT-LCD
**    Himax HX8357-C DS Temporary driver IC.
**
**  Fosc: 10MHz
**  Bit rate: 225Mbps/18BPP, 299Mbps/24BPP.
**  Frame rate: 60Hz
******************************************************************************/
#if defined( BUILD_LK )
    #include  <platform/mt_gpio.h>
    #include  <platform/mt_typedefs.h>
    #include  <string.h>
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

    #define   CSHMDS9960QR_B
  //#define   TFT3P3078_V1_E
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
    #define   LCM_TE_VSYNC_MODE

  #if defined( LCM_ID_PIN_ENABLE )
    #undef    LCM_ID_PIN_ENABLE
  #endif
//#if defined( ARIMA_LO2_HW )
  #if 1 //defined( LO2_EP0 ) || defined( LO2_EP1 )
    #define   LCM_ID_PIN_ENABLE
  #endif
//#endif

  #if defined( LCM_PWM_BL_PIN_ENBLE )
    #undef    LCM_PWM_BL_PIN_ENBLE
  #endif
    #define   LCM_PWM_BL_PIN_ENBLE

  #if defined( LCM_PIXEL_24B_888 )
    #undef    LCM_PIXEL_24B_888
  #endif
  //<2014/04/17-Yuting Shih. Modified from 18BPP to 24BPP for ESD.
    #define   LCM_PIXEL_24B_888
  //>2014/04/17-Yuting Shih.

  #if defined( LCM_PIXEL_RGB )
    #undef    LCM_PIXEL_RGB
  #endif
    #define   LCM_PIXEL_RGB

    #define   FRAME_WIDTH               (320)
    #define   FRAME_HEIGHT              (480)

    #define   REGFLAG_DELAY             0xFFFE
    #define   REGFLAG_END_OF_TABLE      0xFFFA  /* END OF REGISTERS MARKER */

#if defined( LCM_ID_PIN_ENABLE )
  #if !defined( GPIO_LCM_ID_PIN )
    #define   GPIO_LCM_ID_PIN           GPIO58  /* Must be modified. */
  #endif
  #if !defined( GPIO_LCM_ID_PIN_M_GPIO )
    #define   GPIO_LCM_ID_PIN_M_GPIO    GPIO_MODE_00
  #endif
  #if defined( GPIO_LCM_ID_PIN_MODE )
    #undef    GPIO_LCM_ID_PIN_MODE
  #endif
    #define   GPIO_LCM_ID_PIN_MODE      GPIO_LCM_ID_PIN_M_GPIO
#endif /* End.. (LCM_ID_PIN_ENABLE) */
  /** For LCM panel resource selection **/
    #define   LCM_MAIN_SOURCE           1   /* For main source panel */
    #define   LCM_SECOND_SOURCE         0   /* For second source panel */

  #if defined( LCM_PANEL_RESOURCE )
    #undef    LCM_PANEL_RESOURCE
  #endif
    #define   LCM_PANEL_RESOURCE        LCM_MAIN_SOURCE

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

    #define   LCM_MEM_MY                0 //(0x1<<7)  /* 1: Decrease in vertical, 0: Increase in vertical */
  #if( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    #define   LCM_MEM_MX                0 //(0x1<<6)  /* 1: Decrease in horizon, 0: Increase in horizon */
  #else
    #define   LCM_MEM_MX                (0x1<<6)      /* 1: Decrease in horizon, 0: Increase in horizon */
  #endif
    #define   LCM_MEM_MV                0 //(0x1<<5)  /* */
    #define   LCM_MEM_ML                0 //(0x1<<4)
  #if defined( LCM_PIXEL_RGB )
    #define   LCM_MEM_RGB               0         /* RGB:RGB color filter panel */
  #else
    #define   LCM_MEM_RGB               (0x1<<3)  /* RGB:BGR color filter panel */
  #endif
    #define   LCM_MEM_MH                0 //(0x1<<2)

    #define   LCM_MEM_OPERATE           (LCM_MEM_MY|LCM_MEM_MX|LCM_MEM_MV|LCM_MEM_ML|LCM_MEM_MH)
    #define   LCM_MEMORY_ACCESS         (LCM_MEM_OPERATE|LCM_MEM_RGB)

  /*=========================================================================
  ** DSI Video Mode
  **=========================================================================
  ** VRR: 320x480, 60Hz
  **-------------------------------------------------------------------------
  ** 326 <= HS cycle(HP)  (DOTCLK)
  ** 2 <= HS low pulse width(HS) (DOTCLK)
  ** 2 <= Horizontal back porch(HBP) (DOTCLK)
  ** 2 <= Horizontal front porch(HFP) (DOTCLK)
  ** 6 <= Horizontal blanking period(HS+HBP+HFP) (DOTCLK)
  **
  ** 486 <= VS cycle(VP)  (HS cycle)
  ** 2 <= VP low pulse width(VS) (HS)
  ** 2 <= Vertical back porch(VBP) (HS)
  ** 2 <= Vertical front porch(VFP) (HS)
  ** 6 <= Vertical blanking period(VS+VBP+VFP) (HS)
  **=========================================================================*/
    #define   LCM_EVENT_PULSE_MODE_N
    #define   LCM_BURST_PULSE_MODE_N
    #define   LCM_VIDEO_PULSE_MODE

    #define   LCM_RF_ADJUST

#if defined( LCM_RF_ADJUST )
  #if 0 //( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    #define   LCM_HSYNC_NUM             (10)   /** HSPW: PCLK. **/
    #define   LCM_HBP_NUM               (12)   /** HBPD: PCLK. **/
    #define   LCM_HFP_NUM               (10)   /** HFPD: PCLK. **/

    #define   LCM_VSYNC_NUM             (5)   /** VSPW. **/
    #define   LCM_VBP_NUM               (5)   /** VBPD. **/
    #define   LCM_VFP_NUM               (5)   /** VFPD. **/
  #else
    #define   LCM_HSYNC_NUM             (10)  //(4)   /** HSPW: PCLK. **/
    #define   LCM_HBP_NUM               (12)  //(12)  /** HBPD: PCLK. **/
    #define   LCM_HFP_NUM               (10)  //(12)  /** HFPD: PCLK. **/

    #define   LCM_VSYNC_NUM             (5)   //(4)   /** VSPW. **/
    #define   LCM_VBP_NUM               (5)   //(6)   /** VBPD. **/
    #define   LCM_VFP_NUM               (5)   //(6)   /** VFPD. **/
  #endif
#else
    #define   LCM_HSYNC_NUM             (4)   /** HSPW: PCLK. **/
    #define   LCM_HBP_NUM               (48)  /** HBPD: PCLK. **/
    #define   LCM_HFP_NUM               (48)  /** HFPD: PCLK. **/

    #define   LCM_VSYNC_NUM             (4)   /** VSPW. **/
    #define   LCM_VBP_NUM               (6)   /** VBPD. **/
    #define   LCM_VFP_NUM               (6)   /** VFPD. **/
#endif

    #define   LCM_LINE_BYTE             ((FRAME_WIDTH+LCM_HSYNC_NUM+LCM_HBP_NUM+LCM_HFP_NUM)*3)

  /*======================================================================
  ** ref freq = 13 or 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
  ** pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
  **======================================================================*/
  #if defined( LCM_DSI_CMD_MODE )
    #define   LCM_PWM_CLK               (23706)
  #endif

  /*=======================================================================
  ** GETICID (D0h): IC ID Read Command Data
  **-----------------------------------------------------------------------
  ** ID1: The HX8357-C ID = 0x90
  **  - 1st parameter: Dummy byte.
  **  - 2nd parameter: Read HX8357-C ID = 0x90.
  **=======================================================================*/
    #define   LCM_ID0                   (0x00)
    #define   LCM_ID1                   (0x90)

/*****************************************************************************
** Local Variables
******************************************************************************/
  #if 1 //defined( LCM_DSI_CMD_MODE )
    static unsigned int lcm_esd_test = FALSE;   /* Only for ESD test */
  #endif
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
**  Note :
******************************************************************************
** Data ID will depends on the following rule.
**----------------------------------------------------------------------------
** count of parameters > 1  => Data ID = 0x39
** count of parameters = 1  => Data ID = 0x15 ==> HX8357E non-support
** count of parameters = 0  => Data ID = 0x05 ==> HX8357E non-support
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
static LCM_setting_table_V3   lcm_initial_set[] = {
  /* SETEXTC: Set extension command */
    { 0x39, 0xB9,  3, { 0xFF, 0x83, 0x57 }}, /* Enable */
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 5, {}},
  /* SETPANEL: set panel characteristic */
    { 0x15, 0xCC,  1, { 0x09 }},
  /* ? */
  //{ 0x05, 0xBC,  0, { }},
  /* SETCOM: set VCOM voltage related register */
#if defined( LCM_RF_ADJUST )
  #if 0 //( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    { 0x15, 0xB6,  1, { 0x30 }},
  #else
  //{ 0x15, 0xB6,  1, { 0x14 }},  /* -1.46, VCOM OTP not programmed */
  /* 3rd. For water ripples at 2014/04/23 */
    { 0x15, 0xB6,  1, { 0x30 }},
  #endif
#else
    { 0x15, 0xB6,  1, { 0x34 }},  /* -1.46, VCOM OTP not programmed */
#endif
  /* SETRGB: set RGB interface */
    { 0x39, 0xB3,  4, { 0x43, 0x00, 0x06, 0x06 }},
  /* SETOSC: Set internal oscillator */
  //{ 0x15, 0xB0, 1, { 0x66 }}, /* 100% x 10MHz, 60Hz */
  //{ REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 100, {}},
  /* SETPOWER: set power control */
#if defined( LCM_RF_ADJUST )
  #if 0 //( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    { 0x39, 0xB1,  6, { 0x00, 0x15, 0x1C, 0x1C, 0x83, 0xCB }},
  #else
  //{ 0x39, 0xB1,  6, { 0x00, 0x15, 0x1C, 0x1C, 0x83, 0xAA }},
  //{ 0x39, 0xB1,  6, { 0x00, 0x15, 0x1C, 0x1C, 0x83, 0xCB }},
  /* For 24Bit pixel and water ripples when camera preview at 2014/05/29. */
    { 0x39, 0xB1,  6, { 0x00, 0x15, 0x1C, 0x1C, 0x86, 0xCB }},
  #endif
#else
    { 0x39, 0xB1,  6, { 0x00, 0x15, 0x1C, 0x1C, 0x83, 0xAA }},
#endif
  /* SETSTBA: */
    { 0x39, 0xC0,  6, { 0x24, 0x24, 0x01, 0x3C, 0x1E, 0x08 }},
  /* SETCYC: set display cycle register */
#if defined( LCM_RF_ADJUST )
  #if 0 //( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    { 0x39, 0xB4,  7, { 0x02, 0x40, 0x00, 0x2A, 0x2A, 0x0D, 0x4F }},
  #else
  //{ 0x39, 0xB4,  7, { 0x01, 0x40, 0x00, 0x2A, 0x2A, 0x0D, 0x4F }},
    { 0x39, 0xB4,  7, { 0x02, 0x40, 0x00, 0x2A, 0x2A, 0x0D, 0x4F }},
  #endif
#else
    { 0x39, 0xB4,  7, { 0x01, 0x40, 0x00, 0x2A, 0x2A, 0x0D, 0x4F }},
#endif
  /* SETMIPI: */
    { 0x39, 0xBA, 16, { 0x00, 0x56, 0xD4, 0x00,
                        0x0A, 0x00, 0x10, 0x32, 0x6E, 0x04, 0x05, 0x9A, 0x14, 0x19,
                        0x10, 0x40 }},
  /* Memory access control */
  //<2014/3/5--jtao.lee
    { 0x15, 0x36, 1, { LCM_MEMORY_ACCESS }},
  //{ 0x15, 0x36, 1, { 0xC0 }},
  //>2014/3/5--jtao.lee
  /* SETGamma: set gamma curve */
#if 0
    { 0x39, 0xE0, 67, { 0x00, 0x00,
                        0x00, 0x00, 0x13, 0x00, 0x1A, 0x00, 0x29, 0x00, 0x2D, 0x00,
                        0x41, 0x00, 0x49, 0x00, 0x52, 0x00, 0x48, 0x00, 0x41, 0x00,
                        0x3C, 0x00, 0x33, 0x00, 0x30, 0x00, 0x1C, 0x00, 0x19, 0x00,
                        0x03, 0x01,
                        0x00, 0x00, 0x13, 0x00, 0x1A, 0x00, 0x29, 0x00, 0x2D, 0x00,
                        0x41, 0x00, 0x49, 0x00, 0x52, 0x00, 0x48, 0x00, 0x41, 0x00,
                        0x3C, 0x00, 0x33, 0x00, 0x31, 0x00, 0x1C, 0x00, 0x19, 0x00,
                        0x03, 0x00, 0x00 }},
#else /* For Gamma 2.2 */
  #if 0 //( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    { 0x39, 0xE0, 67, { 0x00, 0x00,
                        0x00, 0x00, 0x03, 0x00, 0x0A, 0x00, 0x0B, 0x00, 0x18, 0x00,
                        0x3B, 0x00, 0x48, 0x00, 0x52, 0x00, 0x46, 0x00, 0x3E, 0x00,
                        0x39, 0x00, 0x2D, 0x00, 0x29, 0x00, 0x21, 0x00, 0x20, 0x00,
                        0x00, 0x01,
                        0x00, 0x00, 0x03, 0x00, 0x0A, 0x00, 0x0B, 0x00, 0x18, 0x00,
                        0x3B, 0x00, 0x48, 0x00, 0x52, 0x00, 0x46, 0x00, 0x3E, 0x00,
                        0x39, 0x00, 0x2D, 0x00, 0x29, 0x00, 0x21, 0x00, 0x20, 0x00,
                        0x00, 0x00, 0x00 }},
  #else
  /* 3rd. Adjust at 2014/04/23 */
  /* 2014/04/30 */
    { 0x39, 0xE0, 67, { 0x00, 0x00,
                        0x00, 0x00, 0x02, 0x00, 0x08, 0x00, 0x09, 0x00, 0x13, 0x00,
                        0x33, 0x00, 0x43, 0x00, 0x4F, 0x00, 0x47, 0x00, 0x3E, 0x00,
                        0x37, 0x00, 0x2B, 0x00, 0x27, 0x00, 0x21, 0x00, 0x21, 0x00,
                        0x00, 0x01,
                        0x00, 0x00, 0x02, 0x00, 0x08, 0x00, 0x09, 0x00, 0x13, 0x00,
                        0x33, 0x00, 0x43, 0x00, 0x4F, 0x00, 0x47, 0x00, 0x3E, 0x00,
                        0x37, 0x00, 0x2B, 0x00, 0x27, 0x00, 0x21, 0x00, 0x21, 0x00,
                        0x00, 0x00, 0x00 }},
  #endif
#endif

  /* Interface pixel format */
#if defined( LCM_PIXEL_24B_888 )
//<2014/05/27-Yuting Shih. Add 0xE9 setting for water ripples when camera pre-view.
  //#if defined( LCM_DSI_CMD_MODE )
  ///* SETIMAGE: set image function */
  //  { 0x15, 0xE9,  1, { 0x30 }}, /* For command_mode */
  //#else
  //  { 0x15, 0xE9,  1, { 0x20 }}, /* For video mode */
  //#endif
//>2014/05/27-Yuting Shih.
    { 0x15, 0x3A,  1, { 0x77 }}, /* Set pixel format, 24 Bit/Pixel */
#else
  /* SETIMAGE: set image function */
  //{ 0x15, 0xE9,  1, { 0x00 }},
  //{ 0x15, 0x3A,  1, { 0x66 }}, /* Set pixel format, 18 Bit/Pixel */
#endif
#if defined( LCM_TE_VSYNC_MODE )
  /* SETTE: Set internal TE function */
  //{ 0x39, 0xB7,  3, { 0x00, 0x00, 0x00 }},
  /* Set Tearing ON */
    { 0x15, 0x35,  1, { 0x00 }},
#endif /** End.. (LCM_TE_VSYNC_MODE) **/
  /* SETEXTC: Enable external Command */
  //{ 0xB9, 3, { 0xFF, 0x83, 0x47 }}, /* Disable SETEXTC command */
  //{ REGFLAG_DELAY, 5, {}},
  /****************************************************************************
  ** Note:
  **===========================================================================
  ** Strongly recommend not to set Sleep out / Display On here. That will cause
  ** messed frame to be shown as later the backlight is on.
  ******************************************************************************/
#if defined( BUILD_LK )|| defined( BUILD_UBOOT )
  /* SLPOUT: Exit sleep mode, sleep out */
    { 0x05, 0x11,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* DISPON: Set display on, display on */
    { 0x05, 0x29,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 20, {}},
#endif /* End.. (BUILD_LK)||... */
  /* End: Setting ending by pre-defined flag */
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

static LCM_setting_table_V3   lcm_sleep_out_setting[] = {
  /* SLPOUT: Exit sleep mode, sleep out */
    { 0x05, 0x11,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* DISPON: Set display on, display on */
    { 0x05, 0x29,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 20, {}},
  /* End */
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

static LCM_setting_table_V3   lcm_sleep_in_setting[] = {
  /* DISPOFF: Set display off, Display off sequence */
    { 0x05, 0x28,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 20, {}},
  /* SLPIN: Enter sleep mode, Sleep Mode On */
    { 0x05, 0x10,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* End */
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if defined( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3   lcm_set_window[] = {
    { 0x39, 0x2A,  4,  { 0x00, 0x00, (FRAME_WIDTH >>8), (FRAME_WIDTH &0xFF)}},
    { 0x39, 0x2B,  4,  { 0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

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
//<2014/05/30-Yuting Shih. Add for water ripples when camera preview.
/******************************************************************************
** lcm_set_for_camera_on
*******************************************************************************/
void lcm_set_for_camera_on(void)
{
unsigned int  data_array[16] = { 0 };

    LCM_MSG("[HX8357C_DSI]%s ...\n", __func__ );

    data_array[0] = 0x20E91500;
    dsi_set_cmdq( &data_array, 1, 1 );
} /* End.. push_table() */

/******************************************************************************
** lcm_set_for_camera_off
*******************************************************************************/
void lcm_set_for_camera_off(void)
{
unsigned int  data_array[16] = { 0 };

    LCM_MSG("[HX8357C_DSI]%s ...\n", __func__ );

    data_array[0] = 0x00E91500;
    dsi_set_cmdq( &data_array, 1, 1 );
} /* End.. lcm_set_for_camera_off() */
//>2014/05/30-Yuting Shih.

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
    memcpy( &lcm_util, util, sizeof( LCM_UTIL_FUNCS ));
} /* End.. lcm_set_util_funcs() */

/******************************************************************************
** lcm_get_params
*******************************************************************************/
static void lcm_get_params(LCM_PARAMS *params)
{
    memset( params, 0x00, sizeof( LCM_PARAMS ));

    params->type    = LCM_TYPE_DSI;
    params->width   = FRAME_WIDTH;
    params->height  = FRAME_HEIGHT;
#if 0
    params->physical_width  = FRAME_WIDTH;
    params->physical_height = FRAME_HEIGHT;
#endif

  /* Enable tearing-free */
#if defined( LCM_TE_VSYNC_MODE )
    params->dbi.te_mode           = LCM_DBI_TE_MODE_VSYNC_ONLY;
#else
    params->dbi.te_mode           = LCM_DBI_TE_MODE_DISABLED;
#endif /** End.. (LCM_TE_VSYNC_MODE) **/
    params->dbi.te_edge_polarity  = LCM_POLARITY_RISING;

#if defined( LCM_DSI_CMD_MODE )
    params->dsi.mode  = CMD_MODE;
#else
  #if defined( LCM_EVENT_PULSE_MODE )
    params->dsi.mode  = SYNC_EVENT_VDO_MODE;
  #elif defined( LCM_BURST_PULSE_MODE )
    params->dsi.mode  = BURST_VDO_MODE;
  #else
    params->dsi.mode  = SYNC_PULSE_VDO_MODE;
  #endif
#endif

  /**** DSI ****/
#if defined( LCM_DSI_CMD_MODE )
    params->dsi.word_count  = FRAME_WIDTH * 3; /* DSI CMD mode need set these two bellow params, different to 6577 */
    params->dsi.vertical_active_line  = FRAME_HEIGHT;
    params->dsi.compatibility_for_nvk = 0;    /* this parameter would be set to 1 if DriverIC is NTK's and when force
                                              ** match DSI clock for NTK's */
#endif
  /* Command mode setting */
    params->dsi.LANE_NUM        = LCM_ONE_LANE;
  /* The following defined the format for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
#if defined( LCM_PIXEL_24B_888 )
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    params->dsi.PS                      = LCM_PACKED_PS_24BIT_RGB888;
#else
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB666;
    params->dsi.PS                      = LCM_PACKED_PS_18BIT_RGB666;
#endif

  /* Highly depends on LCD driver capability. */
  /* Not support in MT6573 */
    params->dsi.packet_size = 256;

  /* Video mode setting */
    params->dsi.intermediat_buffer_num  = 2;

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
  /** The best PLL_CLOCK is setting the multiple of 26. **/
  //params->dsi.pll_select  = 1;    /* 0: MIPI_PLL; 1: LVDS_PLL */
#if defined( LCM_PIXEL_24B_888 )
#if defined( LCM_RF_ADJUST )
  #if 0 //( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    params->dsi.PLL_CLOCK   = 111;  //165;
  #else
    params->dsi.PLL_CLOCK   = 111;  //126;  //165;  //130;
  #endif
#else
    params->dsi.PLL_CLOCK   = 156;  //312;  //LCM_DSI_6589_PLL_CLOCK_214_5;
#endif
#else
#if defined( LCM_RF_ADJUST )
  #if 0 //( LCM_PANEL_RESOURCE == LCM_MAIN_SOURCE )
    params->dsi.PLL_CLOCK   = 111;
  #else
    params->dsi.PLL_CLOCK   = 111;  //104;  //165;
  #endif
#else
    params->dsi.PLL_CLOCK   = 130;  //234;  /* this value must be in MTK suggested table */
#endif
#endif
  //<2014/05/06-Yuting Shih. Add for register read when ESD check.
  // 100(or 80)ns > TPLX > 50ns
    params->dsi.LPX         = 3;  /* TLPX: The total time of transmit a byte data.
                                  ** (165Mhz) 2:36ns, 4:72ns
                                  ** (126MHz) 4: ESD check error sometimes
                                  ** (104MHz) 4: ESD check error always, 6: Fail
                                  ** (111MHz) 2: 48~53ns, 3: 74~80ns.*/
  //>2014/05/06-Yuting Shih.

  /**************************************************************************
  ** ESD or noise interference recovery For video mode LCM only.
  ***************************************************************************/
#if 0 //!defined( LCM_DSI_CMD_MODE )
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

//<2014/03/11-Yuting Shih. Disable the SSC feature support.
    params->dsi.ssc_range   = 4;  /* SSC range control: 1 ~ 8(Max), default: 4% */
    params->dsi.ssc_disable = 1;  /* SSC disable control. 0: Enable, 1: Disable */
//>2014/03/11-Yuting Shih.
} /* End.. lcm_get_params() */

/***********************************************************************
** lcm_init
************************************************************************/
static void lcm_init( void )
{
#if defined( LCM_DEBUG_ENABLE )
    LCM_DBG( "[HX8357C_DSI] %s.\n", __func__ );
#endif

    SET_RESET_PIN( 1 );
    MDELAY( 5 );
    SET_RESET_PIN( 0 );
    MDELAY( 15 );
    SET_RESET_PIN( 1 );
    MDELAY( 50 );  /* 120 ms */

#if 0 //defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
#endif
    push_table( lcm_initial_set, sizeof( lcm_initial_set ) / sizeof( lcm_initial_set[0] ), 1 );
} /* End.. lcm_init() */

/***********************************************************************
** lcm_suspend
************************************************************************/
static void lcm_suspend( void )
{
#if defined( LCM_DEBUG_ENABLE )
    LCM_DBG( "[HX8357C_DSI] %s.\n", __func__ );
#endif
    push_table( lcm_sleep_in_setting, sizeof( lcm_sleep_in_setting ) / sizeof( lcm_sleep_in_setting[0] ), 1 );

    SET_RESET_PIN( 1 );
    MDELAY( 5 );
    SET_RESET_PIN( 0 );
    MDELAY( 5 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 );
} /* End.. lcm_suspend() */

/***********************************************************************
** lcm_resume
************************************************************************/
static void lcm_resume( void )
{
#if defined( LCM_DEBUG_ENABLE )
    LCM_DBG( "[HX8357C_DSI] %s.\n", __func__ );
#endif

#if !defined( BUILD_LK ) && !defined( BUILD_UBOOT )
    lcm_init();
#endif
    push_table( lcm_sleep_out_setting, sizeof( lcm_sleep_out_setting ) / sizeof( lcm_sleep_out_setting[0] ), 1 );
} /* End.. lcm_resume() */

/***********************************************************************
** lcm_compare_id
************************************************************************/
static unsigned int lcm_compare_id( void )
{
unsigned int  data_array[16] = { 0 };
unsigned int  id[4] = { 0 }, vRet = 0;
unsigned char buffer[4] = { 0 };

#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8357C_DSI] %s...\n", __func__ );
#endif

#if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
  #if defined( LCM_ID_PIN_ENABLE )
    SET_GPIO_MODE( GPIO_LCM_ID_PIN, GPIO_LCM_ID_PIN_MODE );
    SET_GPIO_DIR_IN( GPIO_LCM_ID_PIN );
  #endif /* End.. (LCM_ID_PIN_ENABLE) */
  #if defined( LCM_PWM_BL_PIN_ENBLE )
    SET_GPIO_MODE( GPIO_PWM_BL_PIN, GPIO_PWM_BL_PIN_MODE );
    SET_GPIO_DIR_OUT( GPIO_PWM_BL_PIN );
  //SET_GPIO_OUT( GPIO_PWM_BL_PIN, GPIO_OUT_ONE );
  #endif /* End.. (LCM_PWM_BL_PIN_ENBLE) */
#endif
  /* NOTE: Should reset LCM firstly */
    SET_RESET_PIN( 1 );
    MDELAY( 5 );
    SET_RESET_PIN( 0 );
    MDELAY( 5 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 );

#if 0
  /** Sleep out **/
    data_array[0] = 0x00110500;
    dsi_set_cmdq( &data_array, 1, 1 );
    MDELAY( 120 );
#endif

  /* SETEXTC: Enable external Command */
    data_array[0] = 0x00043902;
    data_array[1] = 0x5783FFB9;
    dsi_set_cmdq( &data_array, 2, 1 );
    MDELAY( 5 );

  /* Set Maximum Return Packet Size: Read ID return 2 bytes, vender and version id */
    data_array[0] = 0x00023700;
    dsi_set_cmdq( &data_array, 1, 1 );
    MDELAY( 1 );

/***************************************************************
** GETICID: IC ID Read
****************************************************************/
    read_reg_v2( 0xD0, (unsigned char*)&buffer[0], 1 );
    id[0] = buffer[0]; /* Dummy */
    id[1] = buffer[1]; /* HIMAX_ID */

#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8357C_DSI]%s -- ID is 0x%02X\n", __func__, id[1] );
#endif

    if( LCM_ID1 == id[1] )
    {
    #if defined( LCM_DEBUG_ENABLE )
      LCM_DBG( "[HX8357C_DSI]%s -- Check ID OK.\n", __func__ );
    #endif
      vRet = 1;
    }
    else
    {
    #if defined( LCM_ID_PIN_ENABLE )
      if( LCM_PANEL_RESOURCE == mt_get_gpio_in( GPIO_LCM_ID_PIN ))
      {
      #if defined( LCM_DEBUG_ENABLE )
        LCM_DBG( "[HX8357C_DSI]%s -- LCM_ID GPIO OK.\n", __func__ );
      #endif
        vRet = 1;
      }
    #endif /* End.. (LCM_ID_PIN_ENABLE) */
    }

    return  vRet;
} /* End.. lcm_compare_id() */

#if defined( LCM_DSI_CMD_MODE )
/***********************************************************************
** lcm_set_backlight
************************************************************************/
static void lcm_set_backlight(unsigned int level)
{
unsigned int  default_level = 145;
unsigned int  mapped_level  = 0;

#if defined( LCM_DEBUG_ENABLE )
    LCM_DBG( "[HX8357C_DSI] %s.\n", __func__ );
#endif

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

/***********************************************************************
** lcm_update
************************************************************************/
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
unsigned int x0 = x;
unsigned int y0 = y;
unsigned int x1 = x0 + width - 1;
unsigned int y1 = y0 + height - 1;

unsigned char x0_MSB = ((x0>>8)&0xFF);
unsigned char x0_LSB = (x0&0xFF);
unsigned char x1_MSB = ((x1>>8)&0xFF);
unsigned char x1_LSB = (x1&0xFF);
unsigned char y0_MSB = ((y0>>8)&0xFF);
unsigned char y0_LSB = (y0&0xFF);
unsigned char y1_MSB = ((y1>>8)&0xFF);
unsigned char y1_LSB = (y1&0xFF);

unsigned int  data_array[16] = { 0x00 };

#if defined( LCM_DEBUG_ENABLE )
    LCM_DBG( "[HX8357C_DSI] %s.\n", __func__ );
#endif

    data_array[0] = 0x00053902;
    data_array[1] = (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2A;
    data_array[2] = (x1_LSB);
    data_array[3] = 0x00053902;
    data_array[4] = (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2B;
    data_array[5] = (y1_LSB);
    data_array[6] = 0x002C3909;

    dsi_set_cmdq( &data_array, 7, 0 );
} /* End.. lcm_update() */

/***********************************************************************
** lcm_setpwm
************************************************************************/
static void lcm_setpwm(unsigned int divider)
{
  // TBD
} /* End.. lcm_setpwm() */

/***********************************************************************
** lcm_getpwm
************************************************************************/
static unsigned int lcm_getpwm(unsigned int divider)
{
unsigned int pwm_clk = 0;

    pwm_clk = LCM_PWM_CLK / ( 1 << divider );

    return  pwm_clk;
} /* End.. lcm_getpwm() */
#endif /* End.. (LCM_DSI_CMD_MODE) */

#if 1 // defined( LCM_DSI_CMD_MODE )
/******************************************************************************
** lcm_esd_check
**=============================================================================
** NOTE:
**  The function cannot include timer waiting/delay.
*******************************************************************************/
static unsigned int lcm_esd_check(void)
{
unsigned int  array[16] = { 0 };
unsigned char buffer[8] = { 0 };
unsigned char reg09;
//unsigned char reg0A, reg0D;
unsigned char fResult = 0;

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[HX8357C] %s +++\n", __func__ );
  //LCM_PRINT("HX8357C] %s +++\n", __func__ );
#endif

    if( lcm_esd_test )
    {
      lcm_esd_test = FALSE;
      return  TRUE;
    }

#if 0 //defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
#endif

  //<2014/04/17-Yuting Shih. Add/Merged with Nanjing team.
  /* SETEXTC: Enable external Command */
    array[0]=0x00043902;
    array[1]=0x5783FFB9;
    dsi_set_cmdq( &array, 2, 1 );

    array[0]=0x00113902;
    array[1]=0xD45600BA;
    array[2]=0x10000A00;
    array[3]=0x05046E32;
    array[4]=0x1019149A;
    array[5]=0x00000040;
    dsi_set_cmdq( &array, 6, 1 );
  //>2014/04/17-Yuting Shih.
  /*=============================================
  ** Set Maximum Return Size.
  **=============================================*/
    array[0] = 0x00063700;
    dsi_set_cmdq( &array, 1, 1 );
    //MDELAY( 1 );  /* Add waiting for ready. */
  /*=============================================
  ** Read 09h
  **=============================================*/
    read_reg_v2( 0x09, buffer, 2 );
    reg09 = buffer[0];  //buffer[1];
  /*=============================================
  ** Read 0Ah and 0Dh
  **=============================================*/
  //read_reg_v2( 0x0A, buffer, 2 );
  //reg0A = buffer[1];
  //read_reg_v2( 0x0D, buffer, 2 );
  //reg0D = buffer[1];

#if defined( LCM_MSG_ENABLE )
  //LCM_MSG("[HX8357C] Check reg 0Ah = 0x%02X, 0Dh = 0x%02X\n", reg0A, reg0D );
    LCM_MSG("[HX8357C] Check reg 09h = 0x%02X\n", reg09 );
  //LCM_PRINT("[HX8357C] Check reg 09h = 0x%02X\n", reg09 );
#endif
  /*=============================================
  ** Judge Readout & Error Report
  **=============================================*/
  //if(( reg0A != 0x9C ) || ( reg0D != 0x00 ))
    if(( reg09 != 0xE0 ) && ( reg09 != 0x80 )) /* DV1: 0xE0, DV0:0x80 */
    {
    #if defined( LCM_DEBUG_ENABLE )
      LCM_DBG("[HX8357C] ESD check Failed.\n" );
    //LCM_PRINT("[HX8357C] ESD check Failed.\n" );
    #endif
      fResult = 1;
    }
    else
    {
    #if defined( LCM_DEBUG_ENABLE )
      LCM_DBG("[HX8357C] Normal.\n");
    #endif
      fResult = 0;
    }

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[HX8357C] %s ---\n", __func__ );
#endif

    if( fResult )
      return  TRUE;
    else
      return  FALSE;
} /* End.. lcm_esd_check() */

/******************************************************************************
** lcm_esd_recover
*******************************************************************************/
static unsigned int lcm_esd_recover(void)
{
static int recount = 0;

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[HX8357C] %s...\n", __func__ );
  //LCM_PRINT("[HX8357C] %s...\n", __func__ );
#endif

  #if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    lcm_init();
  #else
    lcm_resume();
  #endif
    recount ++;

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[HX8357C] %s: recover recount = %d\n", __func__, recount );
#endif

    return TRUE;
} /* End.. lcm_esd_recover() */
#endif

/***********************************************************************
**
************************************************************************/
LCM_DRIVER  hx8357c_hvga_dsi_vdo_lcm_drv =
{
    .name           = "hx8357c_hvga_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
  #if defined( LCM_DSI_CMD_MODE )
    .set_backlight  = lcm_set_backlight,
    .update         = lcm_update,
    .set_pwm        = lcm_setpwm,
    .get_pwm        = lcm_getpwm,
  #endif
    .esd_check      = lcm_esd_check,
    .esd_recover    = lcm_esd_recover,
};

/*****************************************************************************
** End
******************************************************************************/
#undef _HX8357C_HVGA_DSI_VDO_C_
#endif /** End.. !(_HX8357C_HVGA_DSI_VDO_C_) **/
