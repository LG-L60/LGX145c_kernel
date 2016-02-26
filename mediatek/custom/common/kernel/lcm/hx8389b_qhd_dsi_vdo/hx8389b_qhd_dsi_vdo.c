#ifdef BUILD_LK
    #include  <platform/mt_typedefs.h>
#define LCM_PRINT printf
#else
#include <linux/string.h>

#if defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
    #include  <asm/arch/mt_typedefs.h>
	#define LCM_PRINT printf
	#ifndef KERN_INFO
		#define KERN_INFO
	#endif
#else
	#include <linux/kernel.h>
	#include <mach/mt_gpio.h>
    #include  <mach/mt_typedefs.h>
	#define LCM_PRINT printk
#endif
#endif
#if 1
#define LCM_DBG(fmt, arg...) \
	LCM_PRINT("[HX83891B] %s (line:%d) :" fmt "\r\n", __func__, __LINE__, ## arg)
#else
#define LCM_DBG(fmt, arg...) do {} while (0)
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)//(480)//(540)
#define FRAME_HEIGHT 										(800)//(854)//(960)

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID_HX8389B  0x79  //0x89

  #if defined( LCM_TE_VSYNC_MODE )
    #undef    LCM_TE_VSYNC_MODE
  #endif
    #define   LCM_TE_VSYNC_MODE

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    
       
/*****************************************************************************
** External Functions
******************************************************************************/
#if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    extern void   DSI_clk_HS_mode(BOOL enter);
#endif

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#ifdef _HX8389_
static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/


	//must use 0x39 for init setting for all register.

	{0XB9, 3, {0XFF,0X83,0X89}},
	{REGFLAG_DELAY, 10, {}},

#if 0//CLOSE IC ESD Protect enhance
	{0XBA, 17, {0X41,0X83,0X00,0X16,
				0XA4,0X00,0X18,0XFF,
				0X0F,0X21,0X03,0X21,
				0X23,0X25,0X20,0X02, 
		  		0X31}}, 
	{REGFLAG_DELAY, 10, {}},
#else //open IC ESD  Protect enhance 
{0XBA, 17, {0X41,0X83,0X00,0X16,
			0XA4,0X00,0X18,0XFF,
			0X0F,0X21,0X03,0X21,
			0X23,0X25,0X20,0X02, 
			0X35}}, 
{REGFLAG_DELAY, 10, {}},

#endif

	{0XDE, 2, {0X05,0X58}},
	{REGFLAG_DELAY, 10, {}},

	{0XB1, 19, {0X00,0X00,0X04,0XD9,
				0XCF,0X10,0X11,0XAC,
				0X0C,0X1D,0X25,0X1D,
				0X1D,0X42,0X01,0X58,
		  		0XF7,0X20,0X80}},
	{REGFLAG_DELAY, 10, {}},


	{0XB2, 5,   {0X00,0X00,0X78,0X03,
				 0X02}},
	{REGFLAG_DELAY, 10, {}},
	
	{0XB4, 31, {0X82,0X04,0X00,0X32,
				0X10,0X00,0X32,0X10,
				0X00,0X00,0X00,0X00,
				0X17,0X0A,0X40,0X01,
				0X13,0X0A,0X40,0X14,
				0X46,0X50,0X0A,0X0A,
				0X3C,0X0A,0X3C,0X14,
		  		0X46,0X50,0X0A}}, 
	{REGFLAG_DELAY, 10, {}},
	
	{0XD5, 48, {0X00,0X00,0X00,0X00,
				0X01,0X00,0X00,0X00,
				0X20,0X00,0X99,0X88,
				0X88,0X88,0X88,0X88,
				0X88,0X88,0X88,0X01,
				0X88,0X23,0X01,0X88,
				0X88,0X88,0X88,0X88,
				0X88,0X88,0X99,0X88,
				0X88,0X88,0X88,0X88,
				0X88,0X88,0X32,0X88,
				0X10,0X10,0X88,0X88,
				0X88,0X88,0X88,0X88}}, 
	{REGFLAG_DELAY, 10, {}},
	

	{0XB6, 4,   {0X00,0X8A,0X00,0X8A}},
	{REGFLAG_DELAY, 10, {}},

	{0XCC, 1,   {0X02}},
	{REGFLAG_DELAY, 10, {}},


	{0X35, 1,   {0X00}},//TE on
	{REGFLAG_DELAY, 10, {}},

	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#else
static struct LCM_setting_table lcm_initialization_setting[] = {

	//must use 0x39 for init setting for all register.

#if 0 //truly
	{0XB9, 3, {0XFF,0X83,0X79}},
	{REGFLAG_DELAY, 10, {}},

{0XBA, 3, {0x51, 0x93}}, 
{REGFLAG_DELAY, 10, {}},



	{0XB1, 20, {0x00,0x50,0x44,
	0xE8,0x80,0x08,0x11,
	0x11,0x71,0x2F,0x37,
	0xBF,0x3F,0x42,0x0B,
	0x6E,0xF1,0x00,0xE6,
	0xE6}},
	{REGFLAG_DELAY, 10, {}},


	{0XB2, 13,   {0x00,0x00,0xFE,
	0x08,0x0C,0x19,0x22,
	0x00,0xFF,0x08,0x0C,
	0x19,0x20}},
	{REGFLAG_DELAY, 10, {}},
	
	{0XB4, 31, {0x80,0x08,0x00,
	0x32,0x10,0x05,0x32,  
	0x13,0x66,0x32,0x10,
	0x08,0x37,0x01,0x20,
	0x07,0x37,0x08,0x28,
	0x08,0x30,0x30,0x04,
	0x00,0x40,0x08,0x28,
	0x08,0x30,0x30,0x04}}, 
	{REGFLAG_DELAY, 10, {}},
	
	{0XD5, 47, {0x00,0x00,0x0A,
	0x00,0x01,0x05,0x00,
	0x0A,0x00,0x88,0x88,
	0x89,0x88,0x23,0x01,
	0x67,0x45,0x88,0x01,
	0x88,0x45,0x88,0x88,
	0x88,0x88,0x88,0x88,
	0x88,0x88,0x54,0x76,
	0x10,0x32,0x88,0x54,
	0x88,0x10,0x88,0x88,
	0x88,0x88,0x00,0x00,
	0x00,0x00,0x00,0x00}}, 
	{REGFLAG_DELAY, 10, {}},

	{0XE0, 11, {0x79,0x07,0x12,
	0x15,0x3F,0x3F,0x3F,
	0x1F,0x3C,0x08,0x0D,
	0x0E,0x11,0x12,0x10,
	0x12,0x0C,0x1F,0x07,
	0x12,0x15,0x3F,0x3F,
	0x3F,0x1F,0x3C,0x08,
	0x0D,0x0E,0x11,0x12,
	0x10,0x12,0x0C,0x1F}}, 
	{REGFLAG_DELAY, 10, {}},

	{0XCC, 1,   {0X0A}},
	{REGFLAG_DELAY, 10, {}},
	
	

	{0XB6, 4,   {0x00,0xA0,0x00,	0xA0}},
	{REGFLAG_DELAY, 10, {}},

	{0X51, 1,   {0XFF}},
	{REGFLAG_DELAY, 10, {}},

	{0X53, 1,   {0X24}},
	{REGFLAG_DELAY, 10, {}},

	{0X55, 1,   {0X01}},
	{REGFLAG_DELAY, 10, {}},

	{0X5E, 1,   {0X00}},
	{REGFLAG_DELAY, 10, {}},

	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
#endif

	{0xB9,	3,	{0xFF,0x83,0x79}},
//{0XB9, 3, {0XFF,0X83,0X89}},
{REGFLAG_DELAY, 10, {}},
	{0xBA,	2,	{0x51, 0x93}},
{REGFLAG_DELAY, 10, {}},
	{0XB1,   19,	{0x00, 0x50, 0x44, 0xE8, 0x94, 0x08, 0x11, 0x12, 0x72, 0x28, 
			0x30, 0x9a, 0x1a, 0x42, 0x08, 0x76, 0xf1, 0x00, 0xe6}},
	{REGFLAG_DELAY, 10, {}},		
#if( FRAME_HEIGHT == 854 )
	{0xB2, 13, 	{0x00,0x00,0xFE,0x07,0x0C,0x19,0x22,0x00,0xFF,0x07,
			0x02,0x19,0x20}},
#elif( FRAME_HEIGHT == 800 )
	{0xB2, 13, 	{0x00,0x00,0x3C,0x07,0x0C,0x19,0x22,0x00,0xFF,0x07,
			0x02,0x19,0x20}},
#else /* Default: 864 */
	{0xB2, 13, 	{0x00,0x00,0x44,0x07,0x0C,0x19,0x22,0x00,0xFF,0x07,
			0x02,0x19,0x20}},
#endif
	{REGFLAG_DELAY, 10, {}},
	{0xB4,  31,  	{0x82,0x00,0x00,0x32,0x10,0x03,0x32,0x13,0x29,0x32,
			0x10,0x08,0x35,0x01,0x28,0x0E,0x37,0x00,0x34,0x08,
			0x3E,0x3E,0x08,0x00,0x40,0x08,0x28,0x08,0x30,0x30,
			0x04}},
	{REGFLAG_DELAY, 10, {}},
	{0xCC,	1,	{0x02}}, 
	{REGFLAG_DELAY, 10, {}},
	{0xD5,   47,    {0x00,0x00,0x0A,0x00,0x01,0x05,0x00,0x00,0x18,0x88,
			0x99,0x88,0x01,0x45,0x88,0x88,0x01,0x45,0x23,0x67,
			0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x99,0x54,0x10,
			0x88,0x88,0x76,0x32,0x54,0x10,0x88,0x88,0x88,0x88,
			0x88,0x00,0x00,0x00,0x00,0x00,0x00}},
	{REGFLAG_DELAY, 10, {}},
	{0xE0,     35,	{0x79,0x00,0x16,0x22,0x28,0x2A,0x3F,0x3B,0x4C,0x09,
			0x10,0x10,0x15,0x17,0x15,0x16,0x13,0x18,0x00,0x16,
			0x22,0x29,0x29,0x3F,0x3B,0x4C,0x09,0x10,0x11,0x14,
			0x17,0x16,0x16,0x12,0x17}},
	{REGFLAG_DELAY, 10, {}},
	{0xB6,	4,	{0x00,0x83,0x00,0x83}},
	{REGFLAG_DELAY, 10, {}},
	{0xB5,	3,	{0x00,0x32,0x32}},
	{REGFLAG_DELAY, 10, {}},
	//{0x3a,	1,	{0x60}}, 
	//{REGFLAG_DELAY, 10, {}},

#if defined( LCM_TE_VSYNC_MODE )
//{0X35, 1,   {0X01}},//TE on
	{0x35, 1,   {0x00}},//TE on
#endif
	{REGFLAG_DELAY, 10, {}},
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};

#endif

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if ( LCM_DSI_CMD_MODE )
static LCM_setting_table_V3 lcm_backlight_set[] = {
    { 0x15, 0x51, 1, { 0xFF }},
  //{ REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif /* End.. (LCM_DSI_CMD_MODE) */

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
#if defined( LCM_TE_VSYNC_MODE )
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
#else
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
#endif
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

//#if (LCM_DSI_CMD_MODE)
//		params->dsi.mode   = CMD_MODE;
//#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
//#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 1;   //5;
		params->dsi.vertical_backporch					= 15;  //5;
		params->dsi.vertical_frontporch					= 20;  //5;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 8;   //20;
		params->dsi.horizontal_backporch				= 55;  //46;
		params->dsi.horizontal_frontporch				= 105; //21;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//params->dsi.pll_div1=34;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		//params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
		params->dsi.PLL_CLOCK = 208; //david

    //params->dsi.ssc_range = 4;    /* SSC range control: 1 ~ 8(Max), default: 4% */
    //params->dsi.ssc_disable = 0;  /* SSC disable control. 0: Enable */

}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);//Must over 6 ms,SPEC request

#if defined( BUILD_LK ) || defined( BUILD_UBOOT )
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
#endif

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	SET_RESET_PIN(0);	
	MDELAY(1);	
	SET_RESET_PIN(1);

	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_resume(void)
{

	lcm_init();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}


#if ( LCM_DSI_CMD_MODE )
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

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}

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
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);//Must over 6 ms

	array[0]=0x00043902;
	array[1]=0x7983FFB9;// page enable
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(10);

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; 
	
#if defined(BUILD_LK) || defined(BUILD_UBOOT)
	printf("[HX8389B]%s, id = 0x%08x\n", __func__, id);
#endif

	return (LCM_ID_HX8389B == id)?1:0;

}




LCM_DRIVER hx8389b_qhd_dsi_vdo_drv = 
{
    .name			= "hx8389b_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
    .update         = lcm_update,
#endif
};

