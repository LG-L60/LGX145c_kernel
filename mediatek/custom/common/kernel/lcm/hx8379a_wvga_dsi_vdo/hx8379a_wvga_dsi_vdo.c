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
#ifndef _HX8379A_WVGA_DSI_VDO_C_
#define _HX8379A_WVGA_DSI_VDO_C_
/*****************************************************************************
** LCM model
**  Truly TFT3P3040QR-A-E CSHMDS9378 4.46" 480RGBx854 TFT-LCD
**  Truly TFT3P3080QR-C-E CSHMDS9378 4.45" 480RGBx854 TFT-LCD
**  Truly TFT3P3078-V1-E CSHMDS9378 3.97" 480RGBx854 TFT-LCD
**  Himax HX8379-A DS driver IC.
**  Fosc:
**  Bit rate:
**  Frame rate:
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

    #define   TFT3P3078_V1_E
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
//#if defined( ARIMA_LO1_HW )
  #if 0 //defined( LO1_EP1 )
    #define   LCM_ID_PIN_ENBLE
  #endif
//#endif

  #if defined( LCM_PWM_BL_PIN_ENBLE )
    #undef    LCM_PWM_BL_PIN_ENBLE
  #endif
  //#define   LCM_PWM_BL_PIN_ENBLE

    #define   FRAME_WIDTH               (480)
    #define   FRAME_HEIGHT              (800)

    #define   REGFLAG_DELAY             0xFFFE
    #define   REGFLAG_END_OF_TABLE      0xFFFA  /* END OF REGISTERS MARKER */

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
    #define   LCM_PANEL_RESOURCE        LCM_MAIN_SOURCE
#endif /* End.. (LCM_ID_PIN_ENBLE) */

#if defined( LCM_PWM_BL_PIN_ENBLE )
  #if !defined( GPIO_PWM_BL_PIN )
    #define   GPIO_PWM_BL_PIN           GPIO145
  #endif
  #if !defined( GPIO_PWM_BL_PIN_M_PWM )
    #define   GPIO_PWM_BL_PIN_M_PWM     GPIO_MODE_02//GPIO145_MODE// /* PWM_BL mode */
  #endif
  #if defined( GPIO_PWM_BL_PIN_MODE )
    #undef    GPIO_PWM_BL_PIN_MODE
  #endif
    #define   GPIO_PWM_BL_PIN_MODE      GPIO_PWM_BL_PIN_M_PWM
#endif /* End.. (LCM_PWM_BL_PIN_ENBLE) */

  /*=========================================================================
  ** DPI IF
  **=========================================================================
  ** PCLK: 480x854, 21.6~34.3 MHz
  ** PCLK: 480x800, 20.3~32.2 MHz
  ** VRR: 50~70 Hz
  **-------------------------------------------------------------------------
  ** 504 <= HS cycle(HP) <= 568
  ** 5 <= HS low pulse width(HS)  <= 78 (PCLK)
  ** 5 <= Horizontal back porch(HBP) <= 78 (PCLK)
  ** 5 <= Horizontal front porch(HFP) <= 78 (PCLK)
  ** 19 <= Horizontal data start point(HS+HBP) <= 83
  ** 24 <= Horizontal blanking period(HS+HBP+HFP) <= 88
  **
  ** 806 <= VS cycle (LINE) (480x800)
  ** 860 <= VS cycle (LINE) (480x856)
  ** 2 <= CS low pulse width(VS) (LINE)
  ** 2 <= Vertical back porch(VBP) (LINE)
  ** 2 <= Vertical front porch(VFP) (LINE)
  ** 4 <= Vertical data start point(VS+VBP) (LINE)
  ** 6 <= Vertical blanking period(VS+VBP+VFP) (LINE)
  **=========================================================================
  ** DSI Video Mode
  **=========================================================================
  ** VRR: 480x854, 60Hz
  **-------------------------------------------------------------------------
  ** 504 <= HS cycle(HP) <= 568
  ** 5 <= HS low pulse width(HS)  <= 78 (PCLK)
  ** 5 <= Horizontal back porch(HBP) <= 78 (PCLK)
  ** 5 <= Horizontal front porch(HFP) <= 78 (PCLK)
  ** 19 <= Horizontal data start point(HS+HBP) <= 83
  ** 24 <= Horizontal blanking period(HS+HBP+HFP) <= 88
  **=========================================================================*/
    #define   LCM_EVENT_PULSE_MODE_N
    #define   LCM_BURST_PULSE_MODE_N
    #define   LCM_VIDEO_PULSE_MODE

  #if defined( LCM_EVENT_PULSE_MODE ) || defined( LCM_BURST_PULSE_MODE )
    #define   LCM_HSYNC_NUM             (37)  /** HSPW: PCLK. Shall be larger than 3 **/
    #define   LCM_HBP_NUM               (35)  /** HBPD: PCLK. 5~78 **/
    #define   LCM_HFP_NUM               (35)  /** HFPD: PCLK. 5~88 **/

    #define   LCM_VSYNC_NUM             (2)
    #define   LCM_VBP_NUM               (10)  /** VBPD. **/
    #define   LCM_VFP_NUM               (5)   /** VFPD. **/
  #elif defined( LCM_VIDEO_PULSE_MODE )
    #define   LCM_HSYNC_NUM             (37)  /** HSPW: PCLK. Shall be larger than 3 **/
    #define   LCM_HBP_NUM               (35)  /** HBPD: PCLK. 5~78 **/
    #define   LCM_HFP_NUM               (35)  /** HFPD: PCLK. 5~88 **/

    #define   LCM_VSYNC_NUM             (2)
    #define   LCM_VBP_NUM               (11)  /** VBPD. **/
    #define   LCM_VFP_NUM               (5)   /** VFPD. **/
  #else
    #define   LCM_HSYNC_NUM             (30)  /** HSPW: PCLK. Shall be larger than 3 **/
    #define   LCM_HBP_NUM               (30)  /** HBPD: PCLK. 5~78 **/
    #define   LCM_HFP_NUM               (30)  /** HFPD: PCLK. 5~88 **/

    #define   LCM_VSYNC_NUM             (4)   /** VSPW. Shall be larger than 3 **/
    #define   LCM_VBP_NUM               (11)  /** VBPD. **/
    #define   LCM_VFP_NUM               (14)  /** VFPD. **/
  #endif

    #define   LCM_LINE_BYTE             ((FRAME_WIDTH+LCM_HSYNC_NUM+LCM_HBP_NUM+LCM_HFP_NUM)*3)

  /*======================================================================
  ** ref freq = 13 or 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
  ** pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
  **======================================================================*/
    #define   LCM_PWM_CLK               (23706)

  /*=======================================================================
  ** GETHXID (F4h): Get the Himax IC ID
  **-----------------------------------------------------------------------
  ** ID0: The HX8379A ID = 0x79
  **=======================================================================
  ** READID1~3 (DAh~DCh): Read ID1~3
  **-----------------------------------------------------------------------
  ** ID1: The LCD module's manufacturer. Defined by customer. Default: 0x00
  ** ID2: The LCD module/driver version. Defined by customer. Default: 0x00
  ** ID3: The LCD module/driver. Defined by customer. Default: 0x00
  **=======================================================================*/
    #define   LCM_ID0                   (0x79)
    #define   LCM_ID1                   (0x00)
    #define   LCM_ID2                   (0x00)
    #define   LCM_ID3                   (0x00)

/*****************************************************************************
** Local Variables
******************************************************************************/
  //unsigned int  lcm_esd_test = FALSE; /* Only for ESD test */

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
static LCM_setting_table_V3   lcm_initial_set[] = {
  /* SETEXTC: Set extension command */
    { 0x39, 0xB9,  3, { 0xFF, 0x83, 0x79 }}, /* Enable */
  /* SET MIPI */
    { 0x39, 0xBA,  2, { 0x51, 0x93 }},
  /* SETPOWER: Set power */
    { 0x39, 0xB1, 19, { 0x00, 0x50, 0x14, 0xEA, 0xD0, 0x08, 0x11, 0x10, 0x70, 0x27,
                        0x2F, 0x9A, 0x1A, 0x22, 0x0B, 0x4A, 0xF1, 0x00, 0xE6 }},
  /* SETDISP: Set dislay related register */
    { 0x39, 0xB2, 13, { 0x00, 0x00, 0x3C, 0x0B, 0x03, 0x19, 0x22, 0x00, 0xFF, 0x0B,
                        0x03, 0x19, 0x20 }},
  /* SETCYC: Set panel driving timing */
    { 0x39, 0xB4, 31, { 0x80, 0x08, 0x00, 0x32, 0x10, 0x01, 0x32, 0x13, 0x72, 0x32,
                        0x10, 0x08, 0x33, 0x08, 0x28, 0x05, 0x37, 0x08, 0x3C, 0x0C,
                        0x3E, 0x3E, 0x0C, 0x00, 0x40, 0x08, 0x28, 0x08, 0x30, 0x30,
                        0x04 }},
  /* SETPANEL */
    { 0x15, 0xCC,  1, { 0x02 }},  // 0x0E
  /* SETGIP */
    { 0x39, 0xD5, 47, { 0x00, 0x00, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x03, 0x00, 0x99,
                        0x88, 0xAA, 0xBB, 0x23, 0x01, 0x67, 0x45, 0x01, 0x23, 0x88,
                        0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x99, 0xBB, 0xAA, 0x54,
                        0x76, 0x10, 0x32, 0x32, 0x10, 0x88, 0x88, 0x88, 0x88, 0x88,
                        0x88, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 }},
  /* SETGAMMA: Set gamma 2.2 curve related setting */
    { 0x39, 0xE0, 35, { 0x79, 0x00, 0x03, 0x0F, 0x24, 0x25, 0x3F, 0x31, 0x47, 0x08,
                        0x10, 0x11, 0x15, 0x17, 0x15, 0x15, 0x12, 0x17, 0x00, 0x03,
                        0x0F, 0x24, 0x25, 0x3F, 0x31, 0x47, 0x08, 0x10, 0x11, 0x15,
                        0x17, 0x15, 0x15, 0x12, 0x17 }},
  /* SETVCOM: Set VCOM voltage */
    { 0x39, 0xB6,  4, { 0x00, 0xB5, 0x00, 0xB5 }},
  /* Set Address mode */
  //{ 0x15, 0x36,  1, { 0x00 }},  /* SS normal, GS normal, REV normal, BGR. */
  /* Set Pixel mode  */
  //{ 0x15, 0x3A,  1, { 0x70 }},  /* 24 Bit/Pixel */
#if defined( LCM_TE_VSYNC_MODE )
  /* SETTE: Set internal TE function */
  //{ 0x39, 0xB7,  3, { 0x00, 0x00, 0x00 }},
  /* Set Tearing ON */
    { 0x15, 0x35,  1, { 0x00 }},
#endif /** End.. (LCM_TE_VSYNC_MODE) **/
  /****************************************************************************
  ** Note:
  **===========================================================================
  ** Strongly recommend not to set Sleep out / Display On here. That will cause
  ** messed frame to be shown as later the backlight is on.
  ******************************************************************************/
#if defined( BUILD_LK )|| defined( BUILD_UBOOT )
  /* SLPOUT: Exit sleep mode, sleep out */
    { 0x05, 0x11, 0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 150, {}},
  /* DISPON: Set display on, display on */
    { 0x05, 0x29, 0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 20, {}},
#endif /* End.. (BUILD_LK)||... */
  /* End: Setting ending by pre-defined flag */
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if defined( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3   lcm_set_window[] = {
    { 0x39, 0x2A, 4,  { 0x00, 0x00, (FRAME_WIDTH >>8), (FRAME_WIDTH &0xFF)}},
    { 0x39, 0x2B, 4,  { 0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif /* End.. (LCM_DSI_CMD_MODE) */

static LCM_setting_table_V3   lcm_sleep_out_setting[] = {
  /* SLPOUT: Exit sleep mode, sleep out */
    { 0x05, 0x11, 0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 150, {}},
  /* DISPON: Set display on, display on */
    { 0x05, 0x29, 0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 20, {}},
  /* End */
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

static LCM_setting_table_V3   lcm_sleep_in_setting[] = {
  /* DISPOFF: Set display off, Display off sequence */
    { 0x05, 0x28, 0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 20, {}},
  /* SLPIN: Enter sleep mode, Sleep Mode On */
    { 0x05, 0x10, 0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* End */
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
    params->dsi.LANE_NUM        = LCM_TWO_LANE;
  /* The following defined the format for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

  /* Highly depends on LCD driver capability. */
  /* Not support in MT6573 */
    params->dsi.packet_size = 256;

  /* Video mode setting */
    params->dsi.intermediat_buffer_num  = 2;

    params->dsi.PS          = LCM_PACKED_PS_24BIT_RGB888;

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
  //params->dsi.pll_select  = 1;    /* 0: MIPI_PLL; 1: LVDS_PLL */
    params->dsi.PLL_CLOCK   = 182;  //192;  //LCM_DSI_6589_PLL_CLOCK_214_5; /* this value must be in MTK suggested table */

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
} /* End.. lcm_get_params() */

/***********************************************************************
** lcm_init
************************************************************************/
static void lcm_init( void )
{
#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8379A_DSI] %s.\n", __func__ );
#endif
    SET_RESET_PIN( 1 );
    MDELAY( 1 );
    SET_RESET_PIN( 0 );
    MDELAY( 5 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 );  /* 120 ms */

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
#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8379A_DSI] %s.\n", __func__ );
#endif
    push_table( lcm_sleep_in_setting, sizeof( lcm_sleep_in_setting ) / sizeof( lcm_sleep_in_setting[0] ), 1 );

    SET_RESET_PIN( 1 );
    MDELAY( 1 );
    SET_RESET_PIN( 0 );
    MDELAY( 5 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 );
} /* End.. lcm_suspend() */

/***********************************************************************
** lcm_suspend
************************************************************************/
static void lcm_resume( void )
{
#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8379A_DSI] %s.\n", __func__ );
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
    LCM_MSG( "[HX8379A_DSI] %s...\n", __func__ );
#endif

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
    MDELAY( 1 );
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

  /* SETEXTC: Set ectension command */
    data_array[0] = 0x00043902;
    data_array[1] = 0x7983FFB9; /* Enable */
    dsi_set_cmdq( &data_array, 2, 1 );
    MDELAY( 5 );

  /* Set Maximum Return Packet Size: Read ID return 1 bytes, vender and version id */
    data_array[0] = 0x00013700;
    dsi_set_cmdq( &data_array, 1, 1 );
    MDELAY( 1 );

/***************************************************************
** GETHXID: Read F4h
****************************************************************/
    read_reg_v2( 0xF4, (unsigned char*)&buffer[0], 1 ); /* GETHXID: 0x79 */
    id[0] = buffer[0]; /* HIMAX_ID: we only need ID */
/***************************************************************
** READID1~3 (DAh~DCh): Read ID1~3
****************************************************************/
#if 0
  /* Set Maximum Return Packet Size: Read ID return 2 bytes, vender and version id */
    data_array[0] = 0x00023700;
    dsi_set_cmdq( &data_array, 1, 1 );
    MDELAY( 1 );
    read_reg_v2( 0xDA, (unsigned char*)&buffer[0], 2 );
    id[1] = buffer[1];
    read_reg_v2( 0xDB, (unsigned char*)&buffer[0], 2 );
    id[2] = buffer[1];
    read_reg_v2( 0xDC, (unsigned char*)&buffer[0], 2 );
    id[3] = buffer[1];
#endif

#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8379A_DSI]%s -- ID is 0x%02X\n", __func__, id[0] );
  #if 0
    LCM_MSG( ",0x%02X,0x%02X,0x%02X\n", id[1], id[2], id[3] );
  #endif
#endif
  //if(( LCM_ID0 == id[0] ) && ( LCM_ID1 == id[1] ) && ( LCM_ID2 == id[2] ))
    if( LCM_ID0 == id[0] )
    {
    #if defined( LCM_MSG_ENABLE )
      LCM_MSG( "[HX8379A_DSI]%s -- Check ID OK.\n", __func__ );
    #endif
      vRet = 1;
    }
    else
    {
    #if defined( LCM_ID_PIN_ENBLE )
      if( LCM_PANEL_RESOURCE == mt_get_gpio_in( GPIO_LCM_ID_PIN ))
      {
      #if defined( LCM_MSG_ENABLE )
        LCM_MSG( "[HX8379A_DSI]%s -- LCM_ID GPIO OK.\n", __func__ );
      #endif
        vRet = 1;
      }
    #endif /* End.. (LCM_ID_PIN_ENBLE) */
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
    LCM_MSG( "[HX8379A_DSI]%s enter...\n", __func__ );
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
    LCM_MSG( "[HX8379A_DSI]%s enter...\n", __func__ );
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

/***********************************************************************
**
************************************************************************/
LCM_DRIVER  hx8379a_wvga_dsi_vdo_lcm_drv =
{
    .name             = "hx8379a_wvga_dsi_vdo",
    .set_util_funcs   = lcm_set_util_funcs,
    .get_params       = lcm_get_params,
    .init             = lcm_init,
    .suspend          = lcm_suspend,
    .resume           = lcm_resume,
    .compare_id       = lcm_compare_id,
  #if defined( LCM_DSI_CMD_MODE )
    .set_backlight    = lcm_set_backlight,
    .update           = lcm_update,
    .set_pwm          = lcm_setpwm,
    .get_pwm          = lcm_getpwm,
  #endif
};

/*****************************************************************************
** End
******************************************************************************/
#undef _HX8379A_WVGA_DSI_VDO_C_
#endif /** End.. !(_HX8379A_WVGA_DSI_VDO_C_) **/
