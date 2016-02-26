#ifndef _HX8357C_WVGA_DSI_VDO_C_
#define _HX8357C_WVGA_DSI_VDO_C_
/*****************************************************************************
** LCM model
**
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

    #define   REGFLAG_DELAY             0XFE
    #define   REGFLAG_END_OF_TABLE      0xFF  /* END OF REGISTERS MARKER */

    #define   LCM_ID_HX8389B            0x79

    #define   LCM_HSYNC_NUM             (8)   /** HSPW: PCLK. **/
    #define   LCM_HBP_NUM               (55)  /** HBPD: PCLK. **/
    #define   LCM_HFP_NUM               (105) /** HFPD: PCLK. **/

    #define   LCM_VSYNC_NUM             (1)   /** VSPW. **/
    #define   LCM_VBP_NUM               (15)  /** VBPD. **/
    #define   LCM_VFP_NUM               (20)  /** VFPD. **/

    #define   LCM_LINE_BYTE             ((FRAME_WIDTH+LCM_HSYNC_NUM+LCM_HBP_NUM+LCM_HFP_NUM)*3)

#if defined( LCM_ID_PIN_ENBLE )
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
#if 0 //defined( BUILD_LK ) || defined( BUILD_UBOOT )
    extern void   DSI_clk_HS_mode(BOOL enter);
#endif

#if 0
static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
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
static LCM_setting_table_V3 lcm_initialization_setting[] = {

  /* must use 0x39 for init setting for all register. */
    { 0x39, 0xB9,  3, { 0xFF, 0x83, 0x79 }},
  //{ 0xB9,  3, { 0xFF, 0x83, 0x89 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
    { 0x39, 0xBA,  2, { 0x51, 0x93 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
    { 0x39, 0xB1, 19, { 0x00, 0x50, 0x44, 0xE8, 0x94, 0x08, 0x11, 0x12, 0x72, 0x28,
                        0x30, 0x9A, 0x1A, 0x42, 0x08, 0x76, 0xF1, 0x00, 0xE6 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
#if( FRAME_HEIGHT == 854 )
    { 0x39, 0xB2, 13, { 0x00, 0x00, 0xFE, 0x07, 0x0C, 0x19, 0x22, 0x00, 0xFF, 0x07,
                        0x02, 0x19, 0x20 }},
#elif( FRAME_HEIGHT == 800 )
    { 0x39, 0xB2, 13, { 0x00, 0x00, 0x3C, 0x07, 0x0C, 0x19, 0x22, 0x00, 0xFF, 0x07,
                        0x02, 0x19, 0x20 }},
#else /* Default: 864 */
    { 0x39, 0xB2, 13, { 0x00, 0x00, 0x44, 0x07, 0x0C, 0x19, 0x22, 0x00, 0xFF, 0x07,
                        0x02, 0x19, 0x20 }},
#endif
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
    { 0x39, 0xB4, 31, { 0x82, 0x00, 0x00, 0x32, 0x10, 0x03, 0x32, 0x13, 0x29, 0x32,
                        0x10, 0x08, 0x35, 0x01, 0x28, 0x0E, 0x37, 0x00, 0x34, 0x08,
                        0x3E, 0x3E, 0x08, 0x00, 0x40, 0x08, 0x28, 0x08, 0x30, 0x30,
                        0x04 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
    { 0x15, 0xCC,  1, { 0x02 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
    { 0x39, 0xD5, 47, { 0x00, 0x00, 0x0A, 0x00, 0x01, 0x05, 0x00, 0x00, 0x18, 0x88,
                        0x99, 0x88, 0x01, 0x45, 0x88, 0x88, 0x01, 0x45, 0x23, 0x67,
                        0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x99, 0x54, 0x10,
                        0x88, 0x88, 0x76, 0x32, 0x54, 0x10, 0x88, 0x88, 0x88, 0x88,
                        0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
    { 0x39, 0xE0, 35, { 0x79, 0x00, 0x16, 0x22, 0x28, 0x2A, 0x3F, 0x3B, 0x4C, 0x09,
                        0x10, 0x10, 0x15, 0x17, 0x15, 0x16, 0x13, 0x18, 0x00, 0x16,
                        0x22, 0x29, 0x29, 0x3F, 0x3B, 0x4C, 0x09, 0x10, 0x11, 0x14,
                        0x17, 0x16, 0x16, 0x12, 0x17 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
    { 0x39, 0xB6,  4, { 0x00, 0x83, 0x00, 0x83 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
   /* */
    { 0x39, 0xB5,  3, { 0x00, 0x32, 0x32 }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
  //{ 0x15, 0x3a,  1, { 0x60 }},
  //{ REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
#if defined( LCM_TE_VSYNC_MODE )
  //{ 0x15, 0x35,  1, { 0x01 }},//TE on
    { 0x15, 0x35,  1, { 0x00 }},//TE on
#endif
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* */
#if defined( BUILD_LK ) || defined( BUILD_UBOOT )
  /* Sleep Out */
    { 0x05, 0x11,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* Display ON */
    { 0x05, 0x29,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
#endif
  /* Setting ending by predefined flag */
  //{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static LCM_setting_table_V3 lcm_sleep_out_setting[] = {
  /* Sleep Out */
    { 0x05, 0x11,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  /* Display ON */
    { 0x05, 0x29,  0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};


static LCM_setting_table_V3 lcm_sleep_in_setting[] = {
  /* Display off sequence */
    { 0x05, 0x28, 0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 10, {}},
  /* Sleep Mode On */
    { 0x05, 0x10, 0, { }},
    { REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 125, {}},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if defined( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3 lcm_set_window[] = {
    { 0x39, 0x2A,  4,  { 0x00, 0x00, (FRAME_WIDTH >>8), (FRAME_WIDTH &0xFF)}},
    { 0x39, 0x2B,  4,  { 0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
  //{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static LCM_setting_table_V3 lcm_backlight_set[] = {
    { 0x15, 0x51, 1, { 0xFF }},
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
** lcm_set_util_funcs
*******************************************************************************/
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy( &lcm_util, util, sizeof( LCM_UTIL_FUNCS ));
}

/******************************************************************************
** lcm_get_params
*******************************************************************************/
static void lcm_get_params(LCM_PARAMS *params)
{
    memset( params, 0x00, sizeof( LCM_PARAMS ));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
#if defined( LCM_TE_VSYNC_MODE )
    params->dbi.te_mode         = LCM_DBI_TE_MODE_VSYNC_ONLY;
#else
    params->dbi.te_mode         = LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
#endif
    params->dbi.te_edge_polarity  = LCM_POLARITY_RISING;

#if defined( LCM_DSI_CMD_MODE )
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

  /*** DSI ***/
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
    params->dsi.PS  = LCM_PACKED_PS_24BIT_RGB888;

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
    params->dsi.PLL_CLOCK   = 208;  //LCM_DSI_6589_PLL_CLOCK_175_5; /* this value must be in MTK suggested table */
  #else
    params->dsi.pll_div1    = 34;   // fref=26MHz, fvco=fref*(div1+1) (div1=0~63, fvco=500MHZ~1GHz)
    params->dsi.pll_div2    = 1;    // div2=0~15: fout=fvo/(2*div2)
  #endif

  //params->dsi.ssc_range = 4;    /* SSC range control: 1 ~ 8(Max), default: 4% */
  //params->dsi.ssc_disable = 0;  /* SSC disable control. 0: Enable */
}

/***********************************************************************
** lcm_init
************************************************************************/
static void lcm_init(void)
{
#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8389B] %s.\n", __func__ );
#endif

    SET_RESET_PIN( 1 );
    SET_RESET_PIN( 0 );
    MDELAY( 1 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 );//Must over 6 ms,SPEC request

#if 0 //defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
#endif

    push_table( lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(lcm_initialization_setting[0]), 1 );
}

/***********************************************************************
** lcm_suspend
************************************************************************/
static void lcm_suspend(void)
{
  SET_RESET_PIN( 0 );
  MDELAY( 1 );
  SET_RESET_PIN( 1 );

  push_table( lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(lcm_sleep_in_setting[0]), 1);
}

/***********************************************************************
** lcm_resume
************************************************************************/
static void lcm_resume(void)
{
#if !defined( BUILD_LK ) && !defined( BUILD_UBOOT )
  lcm_init();
#endif
  push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(lcm_sleep_out_setting[0]), 1);
}

#if defined( LCM_DSI_CMD_MODE )
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
    LCM_DBG( "[HX8389B] %s.\n", __func__ );
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

/******************************************************************************
** lcm_setbacklight
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

/***********************************************************************
** lcm_compare_id
************************************************************************/
static unsigned int lcm_compare_id(void)
{
unsigned int  id = 0, vRet = 0;
unsigned int  array[16];
unsigned char buffer[2];

#if defined( LCM_MSG_ENABLE )
    LCM_MSG( "[HX8389B] %s...\n", __func__ );
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
    SET_RESET_PIN( 0 );
    MDELAY( 1 );
    SET_RESET_PIN( 1 );
    MDELAY( 10 ); /* Must over 6 ms */

    array[0]=0x00043902;
    array[1]=0x7983FFB9; /* page enable */
    dsi_set_cmdq( &array, 2, 1);
    MDELAY( 10 );

  /* Set Maximum Return Packet Size: Read ID return 2 bytes, vender and version id */
    array[0] = 0x00023700;/* return byte number */
    dsi_set_cmdq( &array, 1, 1 );
    MDELAY( 10 );

    read_reg_v2( 0xF4, buffer, 2 );
    id = buffer[0];

#if defined( LCM_MSG_ENABLE )
    LCM_MSG("[HX8389B]%s.. id = 0x%08x\n", __func__, id);
#endif

    if( LCM_ID_HX8389B == id )
    {
    #if defined( LCM_MSG_ENABLE )
      LCM_MSG( "[HX8389B]%s -- Check ID OK.\n", __func__ );
    #endif
      vRet = 1;
    }
    else
    {
    #if defined( LCM_ID_PIN_ENBLE )
      if( LCM_PANEL_RESOURCE == mt_get_gpio_in( GPIO_LCM_ID_PIN ))
      {
      #if defined( LCM_MSG_ENABLE )
        LCM_MSG( "[HX8389B]%s -- LCM_ID GPIO OK.\n", __func__ );
      #endif
        vRet = 1;
      }
    #endif /* End.. (LCM_ID_PIN_ENBLE) */
    }

  return  vRet;
}

/***********************************************************************
**
************************************************************************/
LCM_DRIVER hx8389b_wvga_dsi_vdo_drv =
{
    .name           = "hx8389b_wvga_dsi_vdo",
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
#undef _HX8357C_WVGA_DSI_VDO_C_
#endif /** End.. !(_HX8357C_WVGA_DSI_VDO_C_) **/
