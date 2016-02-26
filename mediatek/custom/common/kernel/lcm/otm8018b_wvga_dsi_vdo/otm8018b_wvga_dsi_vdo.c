#ifndef _OTM8018B_WVGA_DSI_VDO_C_
#define _OTM8018B_WVGA_DSI_VDO_C_
/*****************************************************************************
** LCM model
**    Truly TFT480800-82-E 4.3" 480RGBx800 16.7M-color TFT-LCD
**    ORISE OTM8018B 1440-channel 8-bit source driver and 864 gate driver
**    with system-on-chip for color amorphous TFT-LCDs.
**  Fosc:
**  Bit rate: 330.624Mbps/Lane (166MHz)
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

  //#define   TFT480800_82
    #define   TFT3P3253_E
  /*=======================================================================
  ** Local Constants
  **=======================================================================*/
  #if defined( LCM_DSI_CMD_MODE )
    #undef    LCM_DSI_CMD_MODE
  #endif
  //#define   LCM_DSI_CMD_MODE          0

  #if defined( LCM_TE_VSYNC_MODE )
    #undef    LCM_TE_VSYNC_MODE
  #endif
    #define   LCM_TE_VSYNC_MODE

  #if defined( LCM_ID_PIN_ENABLE )
    #undef    LCM_ID_PIN_ENABLE
  #endif
#if defined( ARIMA_LO1_HW )
  #if 0 //defined( LO1_EP1 )
    #define   LCM_ID_PIN_ENABLE
  #endif
#endif

  #if defined( LCM_PWM_BL_PIN_ENBLE )
    #undef    LCM_PWM_BL_PIN_ENBLE
  #endif
    #define   LCM_PWM_BL_PIN_ENBLE

    #define   FRAME_WIDTH               (480)
    #define   FRAME_HEIGHT              (800)

    #define   REGFLAG_DELAY             0xFFE
    #define   REGFLAG_END_OF_TABLE      0xFFF   /* END OF REGISTERS MARKER */

  /*=======================================================================
  ** RDDDBSTR (A1h): Read DDB Start
  **  - Supplier ID: 0x018B
  **  - Display module ID: 0x8009
  **-----------------------------------------------------------------------
  ** Parameter 1: MSB byte of supplier ID
  ** Parameter 2: LSB byte of supplier ID
  ** Parameter 3: MSB byte of display module ID
  ** Parameter 4: LSB byte of display module ID
  ** Parameter 5: 0xFF
  **=======================================================================*/
    #define   LCM_SUPPLIER_ID           0x018B
    #define   LCM_ID_OTM8018B           0x8009

#if defined( LCM_ID_PIN_ENABLE )
  #if !defined( GPIO_LCM_ID_PIN )
  #if defined( LO1_EP1 )
    #define   GPIO_LCM_ID_PIN           GPIO142
  #else
    #define   GPIO_LCM_ID_PIN           GPIO58
  #endif
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
  ** DSI Video Mode
  **=========================================================================
  ** VRR: 480x800, 60Hz
  ** PCLK: 28.75MHz
  ** Data Rate: 345Mbps/2-lane
  **-------------------------------------------------------------------------
  ** 532 <= HS cycle(HP)  (PCLK, Type:574)
  ** 4 <= HS low pulse width(HS) (PCLK, Type:4)
  ** 4 <= Horizontal back porch(HBP) (PCLK, Type:44)
  ** 4 <= Horizontal front porch(HFP) (PCLK, Type:46)
  ** 52 <= Horizontal blanking period(HS+HBP+HFP) (PCLK, Type:94)
  **
  ** VS cycle(VP)  (LINE, Type: 832)
  ** 1 <= VP low pulse width(VS) (LINE, Type:1)
  ** 4 <= Vertical back porch(VBP) (LINE, Type:15)
  ** 4 <= Vertical front porch(VFP) (LINE, Type:16)
  ** Vertical blanking period(VS+VBP+VFP) (LINE, Type:32)
  **=========================================================================*/
    #define   LCM_RF_ADJUST_N
#if 0
    #define   LCM_HSYNC_NUM             (6)   /** HSPW: PCLK. **/
    #define   LCM_HBP_NUM               (37)  /** HBPD: PCLK. **/
    #define   LCM_HFP_NUM               (37)  /** HFPD: PCLK. **/

    #define   LCM_VSYNC_NUM             (4)
    #define   LCM_VBP_NUM               (8)   /** VBPD. **/
    #define   LCM_VFP_NUM               (8)   /** VFPD. **/
#else
  #if defined( LCM_RF_ADJUST )
  /* PLL clock: 166 - 175 */
  //#define   LCM_HSYNC_NUM             (4)   /** HSPW: PCLK. **/
  //#define   LCM_HBP_NUM               (32)  /** HBPD: PCLK. **/
  //#define   LCM_HFP_NUM               (36)  /** HFPD: PCLK. **/
  /* PLL clock: 176 - 185 */
  //#define   LCM_HSYNC_NUM             (4)   /** HSPW: PCLK. **/
  //#define   LCM_HBP_NUM               (44)  /** HBPD: PCLK. **/
  //#define   LCM_HFP_NUM               (46)  /** HFPD: PCLK. **/
  /* PLL clock: 186 - 215 */
    #define   LCM_HSYNC_NUM             (16)  /** HSPW: PCLK. **/
    #define   LCM_HBP_NUM               (56)  /** HBPD: PCLK. **/
    #define   LCM_HFP_NUM               (60)  /** HFPD: PCLK. **/

  /* PLL clock: 166 - 200 */
  //#define   LCM_VSYNC_NUM             (1)
  //#define   LCM_VBP_NUM               (16)   /** VBPD. **/
  //#define   LCM_VFP_NUM               (15)   /** VFPD. **/
  /* PLL clock: 202-215 */
    #define   LCM_VSYNC_NUM             (2)
    #define   LCM_VBP_NUM               (20)   /** VBPD. **/
    #define   LCM_VFP_NUM               (18)   /** VFPD. **/
  #else
    #define   LCM_HSYNC_NUM             (4)   /** HSPW: PCLK. **/
    #define   LCM_HBP_NUM               (44)  /** HBPD: PCLK. **/
    #define   LCM_HFP_NUM               (46)  /** HFPD: PCLK. **/

    #define   LCM_VSYNC_NUM             (1)
    #define   LCM_VBP_NUM               (16)   /** VBPD. **/
    #define   LCM_VFP_NUM               (15)   /** VFPD. **/
  #endif
#endif

    #define   LCM_LINE_BYTE             ((FRAME_WIDTH+LCM_HSYNC_NUM+LCM_HBP_NUM+LCM_HFP_NUM)*3)

/*****************************************************************************
** Local Variables
******************************************************************************/
  #if 1 //defined( LCM_DSI_CMD_MODE )
    static unsigned int lcm_esd_test = FALSE;   /* only for ESD test */
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
    #define   SET_GPIO_DIR_IN(v)        lcm_util.set_gpio_dir(v,GPIO_DIR_IN)
    #define   SET_GPIO_OUT(v,m)         lcm_util.set_gpio_out(v,GPIO_DIR_OUT)
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
    unsigned      cmd;
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
static LCM_setting_table_V3 lcm_initialization_setting[] = {
/***** MIPI IF Enable access command 2 register. *****/
  /* CMD_ENA1: Enable EXTC (CMD2 Enable) */
#if defined( TFT3P3253_E )
    { 0x15, 0x00,  1, { 0x00 }},
#endif
    { 0x39, 0xFF,  3, { 0x80, 0x09, 0x01 }},
  /** CMD_ENA2: ORISE CMD Enable **/
    { 0x15, 0x00,  1, { 0x80 }},        /* Shift address */
    { 0x39, 0xFF,  2, { 0x80, 0x09 }},  /* Enable Orise mode */
#if defined( TFT480800_82 )
  /* ?, CMD 0xFF, Parameter 0x03: 0xFF03 */
    { 0x15, 0x00,  1, { 0x03 }},
    { 0x15, 0xFF,  1, { 0x01 }},
  /* P_DRV_M: Panel Driving Mode */
    { 0x15, 0x00,  1, { 0xB4 }},
    { 0x15, 0xC0,  1, { 0x10 }},  /* 1+2 Dot Inversion */
  /* ? */
    { 0x15, 0x00,  1, { 0x82 }},
    { 0x15, 0xC5,  1, { 0xA3 }},
  /* ? */
    { 0x15, 0x00,  1, { 0x90 }},
    { 0x39, 0xC5,  2, { 0x96, 0x76 }},
  /* GVDDSET: GVDD/NGVDD */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x39, 0xD8,  2, { 0x75, 0x73 }},  /* GVDD:4.575V, NGVDD:4.55V */
  /* VCOMDC(D900h): VCOM voltage setting */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x15, 0xD9,  1, { 0x5E }},  /* -1.4625V */
  /* OSC_ADJ: Oscillator Adjustment for Normal mode */
    { 0x15, 0x00,  1, { 0x81 }},
    { 0x15, 0xC1,  1, { 0x66 }},  /* OSC Adj = 65Hz, 31.15MHz */
  /* ? */
    { 0x15, 0x00,  1, { 0xA1 }},
    { 0x15, 0xC1,  1, { 0x08 }},
  /* ? */
    { 0x15, 0x00,  1, { 0x89 }},
    { 0x15, 0xC4,  1, { 0x08 }},
  /* PTSP3~4: Panel Timing Setting Paramater 3~4 */
    { 0x15, 0x00,  1, { 0xA2 }},
    { 0x39, 0xC0,  3, { 0x1B, 0x00, 0x02 }},  /* pl_width: 28 mclk, pcg_dri: 1 mclk, pcg_nop: 3 mclk */
  /* SD_PCH_CTRL: Source Driver Precharge Control. 1st Parameter */
  //{ 0x15, 0x00,  1, { 0x80 }},
  //{ 0x15, 0xC4,  1, { 0x30 }},  /* SD_PT: GND */
  /* SD_PCH_CTRL: Source Driver Precharge Control. 2nd Parameter */
    { 0x15, 0x00,  1, { 0x81 }},
    { 0x15, 0xC4,  1, { 0x83 }},  /* sd_sap: 0.86uA */
  /* PWR_CTRL2: Power Control Setting 2 for Normal Mode. 2nd Parameter */
    { 0x15, 0x00,  1, { 0x92 }},
    { 0x15, 0xC5,  1, { 0x01 }},  /* PUMP4 VGH: 11.5V, PUMP5 VGL: -7.5V */
  /* PWR_CTRL3: Power Control Setting 3 for DC Voltage Setting. 2nd Parameter */
    { 0x15, 0x00,  1, { 0xB1 }},
    { 0x15, 0xC5,  1, { 0xA9 }},  /* TE output voltage level, LEDON/LEDPWM output voltage level, VCOM hold pulse width x */
  #if 0
  /* PAD_PARA1: IOPAD Parameter 1, HW_EXPEN[2] must equal 1 */
    { 0x15, 0x00,  1, { 0x90 }},
    { 0x15, 0xB3,  1, { 0x03 }},  /* 480x800 */
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* PAD_PARA2: IOPAD Parameter 2, HW_EXPEN[1] must equal 1 */
    { 0x15, 0x00,  1, { 0x91 }},
  //{ 0x15, 0xB3,  1, { 0xC0 }},  /* HW_SMX: memory R/W right to left, HW_SMY: memory R/W down to up */
    { 0x15, 0xB3,  1, { 0x00 }},  /* HW_SMX: memory R/W left to right, HW_SMY: memory R/W up to down */
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* PAD_PARA3: IOPAD Parameter 3 */
    { 0x15, 0x00,  1, { 0x92 }},
    { 0x15, 0xB3,  1, { 0x45 }},
  /* TSP1: TCON Setting Parameter 1 */
  //{ 0x15, 0x00,  1, { 0x80 }},
  //{ 0x39, 0xC0,  5, { 0x00, 0x58, 0x00, 0x14, 0x16 }},
  /* ? */
  //{ 0x15, 0x00,  1, { 0xC1 }},
  //{ 0x15, 0xF5,  1, { 0xC0 }},
  #endif
  /* ? */
    { 0x15, 0x00,  1, { 0x90 }},
    { 0x39, 0xC0,  6, { 0x00, 0x44, 0x00, 0x00, 0x00, 0x03 }},
  /* ? */
    { 0x15, 0x00,  1, { 0xA6 }},
    { 0x39, 0xC1,  3, { 0x00, 0x00, 0x00 }},
  /* GOAVST: GOA VST Setting */
    { 0x15, 0x00,  1, { 0x80 }},
    { 0x39, 0xCE, 12, { 0x87, 0x03, 0x00, 0x85, 0x03, 0x00, 0x86, 0x03, 0x00, 0x84,
                        0x03, 0x00 }},
  /* GOACLKA1~2: GOA CLKA1~2 Setting */
    { 0x15, 0x00,  1, { 0xA0 }},
    { 0x39, 0xCE, 14, { 0x38, 0x03, 0x03, 0x58, 0x00, 0x00, 0x00, 0x38, 0x02, 0x03,
                        0x59, 0x00, 0x00, 0x00 }},
  /* GOACLKA3~4: GOA CLKA3~4 Setting */
    { 0x15, 0x00,  1, { 0xB0 }},
    { 0x39, 0xCE, 14, { 0x38, 0x01, 0x03, 0x5A, 0x00, 0x00, 0x00, 0x38, 0x00, 0x03,
                        0x5B, 0x00, 0x00, 0x00 }},
  /* GOACLKB1~2: GOA CLKB1~2 Setting */
    { 0x15, 0x00,  1, { 0xC0 }},
    { 0x39, 0xCE, 14, { 0x30, 0x00, 0x03, 0x5C, 0x00, 0x00, 0x00, 0x30, 0x01, 0x03,
                        0x5D, 0x00, 0x00, 0x00 }},
  /* GOACLKB3~4: GOA CLKB3~4 Setting */
    { 0x15, 0x00,  1, { 0xD0 }},
    { 0x39, 0xCE, 14, { 0x30, 0x02, 0x03, 0x5E, 0x00, 0x00, 0x00, 0x30, 0x03, 0x03,
                        0x5F, 0x00, 0x00, 0x00 }},
  /* GOATGOPT: GOA Signal Toggle Option Setting. */
    { 0x15, 0x00,  1, { 0xC7 }},
    { 0x15, 0xCF,  1, { 0x00 }},
  /* ? */
    { 0x15, 0x00,  1, { 0xC9 }},
    { 0x15, 0xCF,  1, { 0x00 }},
  /* ? */
    { 0x15, 0x00,  1, { 0xC4 }},
    { 0x39, 0xCB,  6, { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 }},
  /* ? */
    { 0x15, 0x00,  1, { 0xD9 }},
    { 0x39, 0xCB,  6, { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 }},
  /* ? */
    { 0x15, 0x00,  1, { 0x84 }},
    { 0x39, 0xCC,  6, { 0x0C, 0x0A, 0x10, 0x0E, 0x03, 0x04 }},
  /* ? */
    { 0x15, 0x00,  1, { 0x9E }},
    { 0x15, 0xCC,  1, { 0x0B }},
  /* ? */
    { 0x15, 0x00,  1, { 0xA0 }},
    { 0x39, 0xCC,  5, { 0x09, 0x0F, 0x0D, 0x01, 0x02}},
  /* ? */
    { 0x15, 0x00,  1, { 0xB4}},
    { 0x39, 0xCC,  5, { 0x0D, 0x09, 0x0B, 0x02, 0x01 }},
  /* ? */
    { 0x15, 0x00,  1, { 0xCE }},
    { 0x15, 0xCC,  1, { 0x0E }},
  /* */
    { 0x15, 0x00,  1, { 0xD0 }},
    { 0x39, 0xCC,  5, { 0x10, 0x0A, 0x0C, 0x04, 0x03 }},
  /* */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x15, 0x3A,  1, { 0x77 }},

#elif defined( TFT3P3253_E )
  /* ?, CMD 0xFF, Parameter 0x03: 0xFF03 */
  //{ 0x15, 0x00,  1, { 0x03 }},
  //{ 0x15, 0xFF,  1, { 0x01 }},
/**********************************************************/
  /* SD_PCH_CTRL: Source Driver Precharge Control. 1st Parameter **/
    { 0x15, 0x00,  1, { 0x80 }},
    { 0x15, 0xC4,  1, { 0x30 }},  /* SD_PT: GND */
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* ? */
    { 0x15, 0x00,  1, { 0x8A }},
    { 0x15, 0xC4,  1, { 0x40 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
/**********************************************************/
  /* P_DRV_M: Panel Driving Mode */
  //<2014/04/09-Yuting Shih. Modified to reduce power consumption
  #if 0
    { 0x15, 0x00,  1, { 0xB4 }},
    { 0x15, 0xC0,  1, { 0x10 }},  /* 1+2 Dot Inversion */
  #else /* To reduce power consumption. */
    { 0x15, 0x00,  1, { 0xB4 }},
    { 0x15, 0xC0,  1, { 0x50 }},  /* Column Inversion */
  #endif
  //>2014/04/09-Yuting Shih.
  /* ? */
    { 0x15, 0x00,  1, { 0x89 }},
    { 0x15, 0xC4,  1, { 0x08 }},
 /*********************************************************/
  /* PTSP4: Panel Timing Setting Parameter 4 */
    { 0x15, 0x00,  1, { 0xA3 }},
    { 0x15, 0xC0,  1, { 0x00 }},
  /* PWR_CTRL_1 */
    { 0x15, 0x00,  1, { 0x82 }},
    { 0x15, 0xC5,  1, { 0xA3 }},
  /* PWR_CTRL_2 */
  //<2014/04/16-Yuting Shih. Modified/Add to reduce power consumption
  #if 0
    { 0x15, 0x00,  1, { 0x90 }},
    { 0x39, 0xC5,  2, { 0xD6, 0x87 }},
  #else
    { 0x15, 0x00,  1, { 0x90 }},
    { 0x39, 0xC5,  2, { 0x96, 0xEC }},
  #endif
  /*  */  /* Add */
    { 0x15, 0x00,  1, { 0x96 }},
    { 0x15, 0xC5,  1, { 0x30 }},
  //>2014/04/16-Yuting Shih.
  /*  */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x39, 0xD8,  2, { 0x74, 0x72 }},
  /*  */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x15, 0xD9,  1, { 0x58 }},
  /* OSC_ADJ: Oscillator Adjustment for Normal mode */
    { 0x15, 0x00,  1, { 0x81 }},
    { 0x15, 0xC1,  1, { 0x66 }},  /* OSC Adj = 65Hz, 31.15MHz */
  /*  */
    { 0x15, 0x00,  1, { 0xA1 }},
    { 0x15, 0xC1,  1, { 0x08 }},
  /*  */
    { 0x15, 0x00,  1, { 0x81 }},
    { 0x15, 0xC4,  1, { 0x83 }},
  /*  */
  //<2014/04/16-Yuting Shih. Modified to reduce power consumption
  #if 0
    { 0x15, 0x00,  1, { 0x92 }},
    { 0x15, 0xC5,  1, { 0x01 }},
  #else
    { 0x15, 0x00,  1, { 0x92 }},
    { 0x15, 0xC5,  1, { 0x06 }},
  #endif
  //>2014/04/16-Yuting Shih.
  /*  */
    { 0x15, 0x00,  1, { 0xB1 }},
    { 0x15, 0xC5,  1, { 0xA9 }},
  /*  */
    { 0x15, 0x00,  1, { 0x80 }},
    { 0x15, 0xC4,  1, { 0x30 }},

    { 0x15, 0x00,  1, { 0x90 }},
    { 0x39, 0xC0,  6, { 0x00, 0x44, 0x00, 0x00, 0x00, 0x03 }},
  /*  */
    { 0x15, 0x00,  1, { 0x80 }},
    { 0x39, 0xCE, 12, { 0x87, 0x03, 0x00, 0x85, 0x03, 0x00, 0x86, 0x03, 0x00, 0x84,
                        0x03, 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xA0 }},
    { 0x39, 0xCE, 14, { 0x38, 0x03, 0x03, 0x20, 0x00, 0x00, 0x00, 0x38, 0x02, 0x03,
                        0x21, 0x00, 0x00, 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xB0 }},
    { 0x39, 0xCE, 14, { 0x38, 0x01, 0x03, 0x22, 0x00, 0x00, 0x00, 0x38, 0x00, 0x03,
                        0x23, 0x00, 0x00, 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xC0 }},
    { 0x39, 0xCE, 14, { 0x30, 0x00, 0x03, 0x24, 0x00, 0x00, 0x00, 0x30, 0x01, 0x03,
                        0x25, 0x00, 0x00, 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xD0 }},
    { 0x39, 0xCE, 14, { 0x30, 0x02, 0x03, 0x26, 0x00, 0x00, 0x00, 0x30, 0x03, 0x03,
                        0x27, 0x00, 0x00, 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xC7 }},
    { 0x15, 0xCF,  1, { 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xC9 }},
    { 0x15, 0xCF,  1, { 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xC4 }},
    { 0x39, 0xCB,  6, { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 }},
  /*  */
    { 0x15, 0x00,  1, { 0xD9 }},
    { 0x39, 0xCB,  6, { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 }},
  /*  */
    { 0x15, 0x00,  1, { 0x84 }},
    { 0x39, 0xCC,  6, { 0x0C, 0x0A, 0x10, 0x0E, 0x03, 0x04 }},
  /*  */
    { 0x15, 0x00,  1, { 0x9E }},
    { 0x15, 0xCC,  1, { 0x0B }},
  /*  */
    { 0x15, 0x00,  1, { 0xA0 }},
    { 0x39, 0xCC,  5, { 0x09, 0x0F, 0x0D, 0x01, 0x02 }},
  /*  */
    { 0x15, 0x00,  1, { 0xB4 }},
    { 0x39, 0xCC,  6, { 0x0D, 0x0F, 0x09, 0x0B, 0x02, 0x01 }},
  /*  */
    { 0x15, 0x00,  1, { 0xCE }},
    { 0x15, 0xCC,  1, { 0x0E }},
  /*  */
    { 0x15, 0x00,  1, { 0xD0 }},
    { 0x39, 0xCC,  5, { 0x10, 0x0A, 0x0C, 0x04, 0x03 }},
#if 0 /* For Gamma 2.2 re-adjust, 1st */
  /*  */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x39, 0xE1, 16, { 0x09, 0x0A, 0x0E, 0x0D, 0x07, 0x18, 0x0D, 0x0D, 0x01, 0x04,
                        0x05, 0x06, 0x0E, 0x25, 0x22, 0x05 }},
  /*  */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x39, 0xE2, 16, { 0x09, 0x0A, 0x0E, 0x0D, 0x07, 0x18, 0x0D, 0x0D, 0x01, 0x04,
                        0x05, 0x06, 0x0E, 0x25, 0x22, 0x05 }},
#else
/* 7th adjustion: 2014/04/10 */
  /*  */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x39, 0xE1, 16, { 0x01, 0x06, 0x03, 0x00, 0x00, 0x1A, 0x0D, 0x0F, 0x00, 0x03,
                        0x02, 0x07, 0x0E, 0x20, 0x22, 0x09 }},
  /*  */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x39, 0xE2, 16, { 0x01, 0x06, 0x03, 0x00, 0x00, 0x1A, 0x0D, 0x0F, 0x00, 0x03,
                        0x02, 0x07, 0x0E, 0x20, 0x22, 0x09 }},
#endif
/**********************************************************/
  /*  */
    { 0x15, 0x00,  1, { 0xA0 }},
    { 0x15, 0xC1,  1, { 0xEA }},
  /*  */
    { 0x15, 0x00,  1, { 0xA6 }},
    { 0x39, 0xC1,  3, { 0x01, 0x00, 0x00 }},
  /*  */
    { 0x15, 0x00,  1, { 0xC6 }},
    { 0x15, 0xB0,  1, { 0x03 }},
  /*  */
    { 0x15, 0x00,  1, { 0x81 }},
    { 0x15, 0xC5,  1, { 0x66 }},  /* Auto pump 2x */
  /*  */
    { 0x15, 0x00,  1, { 0xB6 }},
    { 0x15, 0xF5,  1, { 0x06 }},  /* VGL_REG timing setting. */
  /*  */
    { 0x15, 0x00,  1, { 0x8B }},
    { 0x15, 0xB0,  1, { 0x40 }},  /* ULPM */
  /*  */
    { 0x15, 0x00,  1, { 0xB1 }},
    { 0x15, 0xB0,  1, { 0x80 }},  /* MIPI RC delay setting */
  /*  */
  /* 7th Gamma adjustion */
    { 0x15, 0x00,  1, { 0x80 }},
    { 0x15, 0xD6,  1, { 0x38 }},  /* Color saturation 1.3 */
/**********************************************************/
  /* CMD_ENA1: CMD2 disable */
    { 0x15, 0x00,  1, { 0x00 }},
    { 0x15, 0xFF,  1, { 0xFF, 0xFF, 0xFF }},
#endif /* End.. (TFT480800_82).. */

#if defined( LCM_TE_VSYNC_MODE )
  /* TEON: Tearing Effect Line ON */
  //{ 0x15, 0x00,  1, { 0x00 }},
    { 0x15, 0x35,  1, { 0x00 }},
#endif
  /****************************************************************************
  ** Note:
  **===========================================================================
  ** Strongly recommend not to set Sleep out / Display On here. That will cause
  ** messed frame to be shown as later the backlight is on.
  ******************************************************************************/
#if defined( BUILD_LK )|| defined( BUILD_UBOOT )
  /* Sleep Out */
    { 0x05, 0x11,  0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* Display ON */
    { 0x05, 0x29,  0, {}},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 15, {}},
  #if defined( TFT480800_82 )
    { 0x05, 0x2C,  0, {}},
  #endif
#endif
  /* Setting ending by predefined flag */
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if defined( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3 lcm_set_window[] = {
  { 0x39, 0x2A,  4, { 0x00, 0x00, (FRAME_WIDTH >>8), (FRAME_WIDTH &0xFF)}},
  { 0x39, 0x2B,  4, { 0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
};
#endif /* End.. (LCM_DSI_CMD_MODE) */

#if 1
static LCM_setting_table_V3 lcm_sleep_out_setting[] = {
  /* Sleep Out */
    { 0x05, 0x11,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* Display ON */
    { 0x05, 0x29,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 15, {}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

static LCM_setting_table_V3 lcm_sleep_in_setting[] = {
  /* Display off sequence */
    { 0x05, 0x28, 0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 15, {}},
  /* Sleep Mode On */
    { 0x05, 0x10, 0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

#if defined( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3 lcm_backlight_set[] = {
    { 0x15, 0x51, 1, { 0xFF }},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif /* End.. (LCM_DSI_CMD_MODE) */

/******************************************************************************
** push_table
*******************************************************************************/
#if 0
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
unsigned int  i;

    for( i = 0; i < count; i++ )
    {
    unsigned  cmd;

      cmd = table[i].cmd;
      switch( cmd )
      {
        case REGFLAG_DELAY:
        {
          MDELAY( table[i].count );
        } break;

        case REGFLAG_END_OF_TABLE:
        {
        } break;

        default:
        {
          dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        } break;
      }
    }
} /* End.. push_table() */

#else
static void push_table(LCM_setting_table_V3 *table, unsigned int count, unsigned char force_update)
{
    dsi_set_cmdq_V3( table, count, force_update );
} /* End.. push_table() */

#endif


/******************************************************************************
** LCM Driver Implementations
** Local Functions
*******************************************************************************/
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
} /* End.. lcm_set_util_funcs() */

/******************************************************************************
** lcm_get_params
*******************************************************************************/
static void lcm_get_params(LCM_PARAMS *params)
{
    memset( params, 0, sizeof( LCM_PARAMS ));

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
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
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif /* End.. (LCM_DSI_CMD_MODE) */

  /**** DSI ****/
#if defined( LCM_DSI_CMD_MODE )
    params->dsi.word_count  = FRAME_WIDTH * 3; /* DSI CMD mode need set these two bellow params, different to 6577 */
    params->dsi.vertical_active_line  = FRAME_HEIGHT;
    params->dsi.compatibility_for_nvk = 0;    /* this parameter would be set to 1 if DriverIC is NTK's and when force
                                              ** match DSI clock for NTK's */
#endif
  /* Command mode setting */
    params->dsi.LANE_NUM        = LCM_TWO_LANE;
  /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

  /* Highly depends on LCD driver capability.
  ** Not support in MT6573 */
    params->dsi.packet_size = 256;

  /* Video mode setting */
    params->dsi.intermediat_buffer_num = 2;
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
  //params->dsi.pll_select  = 1;    /* 0: MIPI_PLL; 1: LVDS_PLL */
  #if defined( LCM_RF_ADJUST )
    params->dsi.PLL_CLOCK   = 208;  /* this value must be in MTK suggested table */
  #else
    params->dsi.PLL_CLOCK   = 182;  //LCM_DSI_6589_PLL_CLOCK_175_5; /* this value must be in MTK suggested table */
  #endif
#else
    params->dsi.pll_div1    = 29;   // fref=26MHz, fvco=fref*(div1+1) (div1=0~63, fvco=500MHZ~1GHz)
    params->dsi.pll_div2    = 1;    // div2=0~15: fout=fvo/(2*div2)
#endif
  //<2014/05/06-Yuting Shih. Add for register read when ESD check.
  //params->dsi.LPX         = 4;  /* TLPX. 4:72ns, 2:36ns */
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
    params->dsi.noncont_clock         = TRUE;
    params->dsi.noncont_clock_period  = 2; /* Unit : frames */
#endif /* End.. !(LCM_DSI_CMD_MODE) */

//<2014/03/11-Yuting Shih. Disable the SSC feature support.
    params->dsi.ssc_range   = 4;  /* SSC range control: 1 ~ 8(Max), default: 4% */
    params->dsi.ssc_disable = 1;  /* SSC disable control. 0: Enable, 1: Disable */
//>2014/03/11-Yuting Shih.
} /* End.. lcm_get_params() */

/******************************************************************************
** lcm_init
*******************************************************************************/
static void lcm_init(void)
{
#if defined( LCM_DEBUG_ENABLE )
    LCM_DBG( "[OTM8018B_DSI] %s.\n", __func__ );
#endif

    SET_RESET_PIN( 1 );
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY( 50 );

#if 0 //defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
#endif
    push_table( lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(lcm_initialization_setting[0]), 1 );
} /* End.. lcm_init() */

/******************************************************************************
** lcm_suspend
*******************************************************************************/
static void lcm_suspend(void)
{
#if 1
    push_table( lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(lcm_sleep_in_setting[0]), 1 );
#endif

    SET_RESET_PIN( 1 );
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY(20);
} /* End.. lcm_suspend() */

/******************************************************************************
** lcm_resume
*******************************************************************************/
static void lcm_resume(void)
{
    lcm_init();
#if 1
    push_table( lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(lcm_sleep_out_setting[0]), 1 );
#endif
} /* End.. lcm_resume() */

#if defined( LCM_DSI_CMD_MODE )
/******************************************************************************
** lcm_update
*******************************************************************************/
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
unsigned int  x0 = x;
unsigned int  y0 = y;
unsigned int  x1 = x0 + width - 1;
unsigned int  y1 = y0 + height - 1;

unsigned char x0_MSB = ((x0>>8)&0xFF);
unsigned char x0_LSB = ( x0&0xFF);
unsigned char x1_MSB = ((x1>>8)&0xFF);
unsigned char x1_LSB = ( x1&0xFF);
unsigned char y0_MSB = ((y0>>8)&0xFF);
unsigned char y0_LSB = ( y0&0xFF);
unsigned char y1_MSB = ((y1>>8)&0xFF);
unsigned char y1_LSB = ( y1&0xFF);

unsigned int data_array[16];

    data_array[0] = 0x00053902;
    data_array[1] = (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2A;
    data_array[2] = (x1_LSB);
    data_array[3] = 0x00053902;
    data_array[4] = (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2B;
    data_array[5] = (y1_LSB);
    data_array[6] = 0x002C3909;

    dsi_set_cmdq( &data_array, 7, 0 );
} /* End.. lcm_update() */

/******************************************************************************
** lcm_update
*******************************************************************************/
static void lcm_setbacklight(unsigned int level)
{
unsigned int  default_level = 145;
unsigned int  mapped_level  = 0;

  /* For LGE backlight IC mapping table */
    if( level > 255 )
      level = 255;

    if( level > 0 )
    {
      mapped_level = default_level + ( level )*( 255 - default_level ) / ( 255 );
    }
    else
    {
      mapped_level = 0;
    }

  /* Refresh value of backlight level. */
    lcm_backlight_set[0].para_list[0] = (unsigned char)mapped_level;
    push_table( lcm_backlight_set, sizeof( lcm_backlight_set ) / sizeof( lcm_backlight_set[0] ), 1 );
} /* End.. lcm_setbacklight() */
#endif /* End.. (LCM_DSI_CMD_MODE) */

/******************************************************************************
** lcm_compare_id
*******************************************************************************/
static unsigned int lcm_compare_id(void)
{
int   array[4];
int   supp_id = 0, disp_id = 0, vRet = 0;
char  buffer[16] = { 0 };
char  id_high = 0;
char  id_low  = 0;
#if defined( LCM_ID_PIN_ENABLE )
int   gpio_id = LCM_PANEL_RESOURCE;
#endif

#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[OTM8018B] %s...\n", __func__ );
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
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY( 200 );

    array[0] = 0x00063700;
    dsi_set_cmdq( &array, 1, 1 );
    MDELAY( 10 );

    read_reg_v2( 0xA1, buffer, 5 );

  #if defined( LCM_DEBUG_ENABLE )
  /* 0x01, 0x8B, 0x80, 0x09, 0xFF */
    LCM_DBG("[OTM8018B]%s -- ID get: 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",
          __func__, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
  #endif
  /* Supplier ID */
    id_high = buffer[0];  /* MSB */
    id_low  = buffer[1];  /* LSB */
    supp_id = ( id_high << 8 ) | id_low;
  /* Display module ID */
    id_high = buffer[2];  /* MSB */
    id_low  = buffer[3];  /* LSB */
    disp_id = ( id_high << 8 ) | id_low;

#if defined( LCM_ID_PIN_ENABLE )
    gpio_id = mt_get_gpio_in( GPIO_LCM_ID_PIN );
  #if defined( LCM_DEBUG_ENABLE )
    LCM_DBG("[OTM8018B] LCM ID PIN: %d\n", gpio_id );
  #endif
#endif /* End.. (LCM_ID_PIN_ENABLE) */

    if( LCM_ID_OTM8018B == disp_id )
    {
    #if defined( LCM_ID_PIN_ENABLE )
      if( LCM_PANEL_RESOURCE == gpio_id )
    #endif
      {
      #if defined( LCM_DEBUG_ENABLE )
        LCM_DBG("[OTM8018B] Check ID = 0x%04X 0x%04X OK.\n", supp_id, disp_id );
      #endif
        vRet = 1;
      }
    }
    else
    {
    #if defined( LCM_ID_PIN_ENABLE )
      if( LCM_PANEL_RESOURCE == gpio_id )
      {
      #if defined( LCM_DEBUG_ENABLE )
        LCM_DBG("[OTM8018B] LCM_ID GPIO OK.\n");
      #endif
        vRet = 1;
      }
    #endif /* End.. (LCM_ID_PIN_ENABLE) */
    }

    return  vRet;
}

#if 1 //defined( LCM_DSI_CMD_MODE )
/******************************************************************************
** lcm_esd_check
**=============================================================================
** NOTE:
**  The function cannot include timer waiting/delay.
*******************************************************************************/
static unsigned int lcm_esd_check(void)
{
static int  count = 0;
static int  err_count = 0;
static int  uncount = 0;
unsigned char fResult;
unsigned char buffer[4] = { 0 }, reg0A, reg0D;
unsigned int  array[16] = { 0 };

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[OTM8018B] %s +++\n", __func__ );
#endif
    if( lcm_esd_test )
    {
      lcm_esd_test = FALSE;
      return  TRUE;
    }

#if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
#endif

  /*=============================================
  ** Set Maximum Return Size
  **=============================================*/
    array[0] = 0x00023700;
    dsi_set_cmdq( &array, 1, 1 );
    //MDELAY( 1 );  /* Add waiting for ready. */
  /*=============================================
  ** Read 0Ah and 0Dh
  **=============================================*/
    read_reg_v2( 0x0A, buffer, 1 );
    reg0A = buffer[0];
    read_reg_v2( 0x0D, buffer, 1 );
    reg0D = buffer[0];

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[OTM8018B] Read register 0Ah = 0x%02X, 0Dh = 0x%02X\n", reg0A, reg0D );
  //LCM_PRINT("[OTM8018B] Read register 0Ah = 0x%02X, 0Dh = 0x%02X\n", reg0A, reg0D );
#endif
  /*=============================================
  ** Judge Readout & Error Report
  **=============================================*/
    if(( reg0A != 0x9C ) || ( reg0D != 0x00 ))
    {
      err_count = 0;
      uncount++;
  #if defined( LCM_DEBUG_ENABLE )
      LCM_DBG("[OTM8018B] failed, err_count = %d, unnormal uncount = %d\n", err_count, uncount );
    //LCM_PRINT("[OTM8018B] ESD check failed, err_count = %d, unnormal uncount = %d\n", err_count, uncount );
  #endif
      fResult = 1;
    }
    else
    {
      count++;
  #if defined( LCM_DEBUG_ENABLE )
      LCM_DBG("[OTM8018B] normal count = %d\n", count );
  #endif
      fResult = 0;
    }

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[OTM8018B] %s ---\n", __func__ );
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
      LCM_MSG("[OTM8018B] %s +++\n", __func__ );
    //LCM_PRINT("[OTM8018B] %s +++\n", __func__ );
#endif

  #if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    lcm_init();
  #else
    lcm_resume();
  #endif
    recount ++;

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[OTM8018B] %s: recover recount = %d\n", __func__, recount );
#endif

    return TRUE;
} /* End.. lcm_esd_recover() */
#endif

/******************************************************************************
** lcm_compare_id
*******************************************************************************/
LCM_DRIVER otm8018b_wvga_dsi_vdo_lcm_drv =
{
    .name           = "otm8018b_wvga_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#if defined( LCM_DSI_CMD_MODE )
    .update         = lcm_update,
    .set_backlight  = lcm_setbacklight,
#endif
    .esd_check      = lcm_esd_check,    /* Only for command mode, no use in video mode */
    .esd_recover    = lcm_esd_recover,  /* Only for command mode, no use in video mode */
};

/*****************************************************************************
** End
******************************************************************************/
#undef _OTM8018B_WVGA_DSI_VDO_C_
#endif /** End.. !(_OTM8018B_WVGA_DSI_VDO_C_) **/

