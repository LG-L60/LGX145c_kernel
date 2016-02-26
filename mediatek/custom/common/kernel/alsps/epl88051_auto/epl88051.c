/* drivers/hwmon/mt6516/amit/epl88051.c - EPL88051 PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/** VERSION: 1.04**/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/hwmsen_helper.h>
#include "epl8800.h"
#include <linux/input/mt.h>


#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

//add for fix resume issue
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
//add for fix resume issue end


/******************************************************************************
 * extern functions
*******************************************************************************/
/*
#ifdef MT6575
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif


#ifdef MT6589
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

#ifdef MT6582
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt_eint_set_sens(kal_uint8 eintno, kal_bool sens);
//extern void mt_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
//                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
//                                     kal_bool auto_umask);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

#endif
*/
/*-------------------------MT6516&MT6575 define-------------------------------*/

#define POWER_NONE_MACRO MT65XX_POWER_NONE


/******************************************************************************
 *  configuration
 ******************************************************************************/
//<2014/04/15 ShermanWei
static int PS_INTT 				= EPL_INTT_PS_110;///EPL_INTT_PS_90;
//>2014/04/15 ShermanWei

//<2014/03/25 ShermanWei
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
//<2014/8/4-42212-LindaGuo, [5503][Common][Psenser]Fix Psenser vendor register error will cause 2.5cm fail .
//<2014/07/30 Ices
#define PS_DELAY 				300//200//300///100///40
//>2014/07/30 Ices
//>2014/8/4-42212-LindaGuo
//>2014/07/14-Yuting Shih.
//>2014/03/25
//<2014/04/15 ShermanWei
#define PS_ENABLE_DELAY 			100
//>2014/04/15 ShermanWei
/******************************************************************************
*******************************************************************************/

#define TXBYTES 				2
#define RXBYTES					2

#define PACKAGE_SIZE 			RXBYTES //8
#define I2C_RETRY_COUNT 		2

#define EPL_DEV_NAME   		 "EPL88051"

typedef struct _epl_ps_factory
{
    bool cal_file_exist;
    bool cal_finished;
    char s1[16];
    char s2[16];
//<2014/03/25 ShermanWei
    char s3[16];
    u16 cal_ctalk;
    u16 cal_h;
    u16 cal_l;
//>2014/03/25 ShermanWei
};

typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
//<2014/06/26 ShermanWei,near state ReDynamicCal
    bool dynaKagain;
//>2014/06/26 ShermanWei
//<2014/03/25 ShermanWei
    bool buffer_ready;
    bool dynaK;
    u16 show_screen;
    u16	cal_delta;
//>2014/03/25 ShermanWei
    u16 ps_state;
    u16 ps_raw;
    struct _epl_ps_factory ps_factory;

} epl_raw_data;

//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
char high[8];
char low[8];
int high_length =0;
int low_length =0;
char ctalk[8];
int ctalk_length =0;
//2014/07/31-42093-RichardLi

/*----------------------------------------------------------------------------*/
#define APS_TAG                 	  	"[PS] "
#define APS_FUN(f)              	  	printk(KERN_INFO APS_TAG "%s \r\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(KERN_ERR  APS_TAG "%s %d : " fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk(KERN_INFO APS_TAG "[%s] " fmt, EPL_DEV_NAME, ##args)
#define APS_DBG(fmt, args...)    	    printk(KERN_INFO "[%s] " fmt, EPL_DEV_NAME, ##args)

/*----------------------------------------------------------------------------*/
static struct i2c_client *epl_i2c_client = NULL;


/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl_i2c_id[] = {{EPL_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_EPL= { I2C_BOARD_INFO(EPL_DEV_NAME, (0x82>>1))};

/*----------------------------------------------------------------------------*/
static int epl_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl_i2c_remove(struct i2c_client *client);
static int epl_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);


/*----------------------------------------------------------------------------*/
static int epl_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl_i2c_resume(struct i2c_client *client);

static void epl_eint_func(void);
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);


extern struct  alsps_hw *epl88051_get_cust_alsps_hw(void);
static int epl_local_init(void);
static int epl_remove(void);
static struct sensor_init_info epl88051_init_info =
{
    .name  = "epl88051",
    .init      = epl_local_init,
    .uninit    = epl_remove,
};

static int epl88051_init_flag = -1;
//<2014/06/26 ShermanWei
static DEFINE_MUTEX(epl88051_mutex);
/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_PS     	= 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct epl_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
    struct delayed_work  polling_work;

    /*i2c address group*/
    struct epl_i2c_addr  addr;

    /*misc*/
    atomic_t    	trace;
    atomic_t    	ps_suspend;
    u16             ps_delay;

    /*data*/
    ulong       	enable;         	/*record HAL enalbe status*/
    ulong      	pending_intr;   	/*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};



/*----------------------------------------------------------------------------*/
static struct i2c_driver epl_i2c_driver =
{
    .probe     	= epl_i2c_probe,
    .remove     = epl_i2c_remove,
    .detect     	= epl_i2c_detect,
    .suspend    = epl_i2c_suspend,
    .resume     = epl_i2c_resume,
    .id_table   	= epl_i2c_id,
    .driver = {
        //.owner          = THIS_MODULE,
        .name           = EPL_DEV_NAME,
    },
};

static struct epl_priv *epl_obj = NULL;
////static struct platform_driver epl_alsps_driver;
static struct wake_lock g_ps_wlock;
static epl_raw_data	gRawData;


/*
//====================I2C write operation===============//
//regaddr: ELAN epl Register Address.
//bytecount: How many bytes to be written to epl register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      epl_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
*/
static int epl_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = (regaddr<<3) | bytecount ;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n",ret);
        mdelay(10);
    }


    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}




/*
//====================I2C read operation===============//
*/
static int epl_I2C_Read(struct i2c_client *client)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i =0;
    int retry;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_recv(client, buffer, RXBYTES);

        if (ret == RXBYTES)
            break;

        APS_ERR("i2c read error,RXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    for(i=0; i<PACKAGE_SIZE; i++)
        gRawData.raw_bytes[i] = buffer[i];

    return ret;
}


static int elan_calibration_atoi(char* s)
{
    int num=0,flag=0;
    int i=0;
    for(i=0; i<=strlen(s); i++)
    {
        if(s[i] >= '0' && s[i] <= '9')
            num = num * 10 + s[i] -'0';
        else if(s[0] == '-' && i==0)
            flag =1;
        else
            break;
    }
    if(flag == 1)
        num = num * -1;
    return num;
}

static int elan_calibaration_read(struct epl_priv *epl_data)
{
	struct file *fp_h;
	struct file *fp_l;
//<2014/03/25 ShermanWei
	struct file *fp_ct;
//>2014/03/25 ShermanWei
	struct i2c_client *client = epl_data->client;
	mm_segment_t fs;
	loff_t pos;
	APS_LOG("[ELAN] %s,cal_file_exist=%d\n", __func__,gRawData.ps_factory.cal_file_exist);

	if(gRawData.ps_factory.cal_file_exist == 1)
	{
		///fp_h = filp_open("/data/ps/h-threshold.dat", O_RDWR, S_IRUSR/*0777*/);
		fp_h = filp_open("/protect_f/h-threshold.dat", O_RDONLY, 0755);
		if (IS_ERR(fp_h))
		{
			APS_ERR("[ELAN]create file_h error\n");
			gRawData.ps_factory.cal_file_exist = 0;
		}

		///fp_l = filp_open("/data/ps/l-threshold.dat", O_RDWR, S_IRUSR/*0777*/);
///		fp_l = filp_open("/protect_f/l-threshold.dat", O_RDONLY, 0755);
///		if (IS_ERR(fp_l))
///		{
///			APS_ERR("[ELAN]create file_l error\n");
///			gRawData.ps_factory.cal_file_exist = 0;
///		}
//<2014/03/25 ShermanWei
		fp_ct = filp_open("/protect_f/cross-talk.dat", O_RDONLY, 0755);
		if (IS_ERR(fp_ct))
		{
			APS_ERR("[ELAN]create file_ct error\n");
			gRawData.ps_factory.cal_file_exist = 0;
		}
//>2014/03/25 ShermanWei
	}

	if(gRawData.ps_factory.cal_file_exist == 1)
	{
        int read_ret = 0;
		fs = get_fs();
		set_fs(KERNEL_DS);
		pos = 0;

		read_ret = vfs_read(fp_h, gRawData.ps_factory.s1, sizeof(gRawData.ps_factory.s1), &pos);
		gRawData.ps_factory.s1[read_ret] = '\0';
        APS_LOG("[ELAN] gRawData.ps_factory.s1=%s, sizeof(gRawData.ps_factory.s1)=%d \r\n", gRawData.ps_factory.s1, read_ret);
///		pos = 0;

///		read_ret = vfs_read(fp_l, gRawData.ps_factory.s2, sizeof(gRawData.ps_factory.s2), &pos);
///		gRawData.ps_factory.s2[read_ret] = '\0';
///        APS_LOG("[ELAN] gRawData.ps_factory.s2=%s, sizeof(gRawData.ps_factory.s2)=%d \r\n", gRawData.ps_factory.s2, read_ret);
//<2014/03/25 ShermanWei
		pos = 0;
		read_ret = vfs_read(fp_ct, gRawData.ps_factory.s3, sizeof(gRawData.ps_factory.s3), &pos);
		gRawData.ps_factory.s3[read_ret] = '\0';
        APS_LOG("[ELAN] gRawData.ps_factory.s3=%s, sizeof(gRawData.ps_factory.s3)=%d \r\n", gRawData.ps_factory.s3, read_ret);
		filp_close(fp_ct, NULL);
//>2014/03/25 ShermanWei
		filp_close(fp_h, NULL);
///		filp_close(fp_l, NULL);
		set_fs(fs);

		gRawData.ps_factory.cal_h = elan_calibration_atoi(gRawData.ps_factory.s1);
//<2014/04/17 ShermanWei ,change l-threshold rule = crosstalk + 0.5(delta)
		/////gRawData.ps_factory.cal_l = elan_calibration_atoi(gRawData.ps_factory.s2);
//<2014/03/25 ShermanWei
		gRawData.ps_factory.cal_ctalk = elan_calibration_atoi(gRawData.ps_factory.s3);
		gRawData.cal_delta = gRawData.ps_factory.cal_h - gRawData.ps_factory.cal_ctalk;
//>2014/03/25 ShermanWei
		gRawData.ps_factory.cal_l = gRawData.ps_factory.cal_ctalk + (gRawData.cal_delta/2);
//>2014/04/17 ShermanWei ,change l-threshold rule
//<2014/04/28 ShermanWei ,change H-threshold rule
///		epl_data->hw->ps_threshold_high = gRawData.ps_factory.cal_h;
		epl_data->hw->ps_threshold_high = gRawData.ps_factory.cal_ctalk + (gRawData.cal_delta * 9 / 10);
//>2014/04/28 ShermanWei ,change H-threshold rule
		epl_data->hw->ps_threshold_low = gRawData.ps_factory.cal_l;

		APS_LOG("[ELAN] shiftR3bit read cal_h:%d ,cal_l:%d,cal_ctalk:%d,gRawData.cal_delta:%d \n", 					gRawData.ps_factory.cal_h,gRawData.ps_factory.cal_l,gRawData.ps_factory.cal_ctalk,gRawData.cal_delta);
		//<2014/03/25 ShermanWei
//<2014/05/05 ShermanWei for MiddleGain
#if 0///defined ARIMA_PCBA_RELEASE //<2014/06/26 ShermanWei
		set_psensor_intr_threshold((epl_data->hw ->ps_threshold_low)<<3,(epl_data->hw ->ps_threshold_high)<<3);
#endif
//>2014/05/05 ShermanWei
		//>2014/03/25 ShermanWei
		//<2014/08/04-samhuang, add delta check
		if( ( gRawData.cal_delta < 20) || ( gRawData.cal_delta > 150) )
		{
			gRawData.ps_factory.cal_ctalk = 100;
			#if defined(ARIMA_LO1_HW)
				gRawData.cal_delta = 100;
			#elif defined(ARIMA_LO2_HW)
				gRawData.cal_delta = 50;
			#endif
			APS_LOG("Wrong delta reset to default : cal_ctalk:%d, cal_delta:%d \n", gRawData.ps_factory.cal_ctalk,gRawData.cal_delta);
		}
		//>2014/08/04-samhuang
	}
	////gRawData.ps_factory.cal_finished = 1;
//<2014/04/17 ShermanWei
	else
	{
		APS_LOG("[ELAN] default thres_h:%d ,thres_l:%d \n",epl_data->hw ->ps_threshold_high, epl_data->hw ->ps_threshold_low);
		//<2014/06/26 ShermanWei,near state ReDynamicCal
		gRawData.ps_factory.cal_ctalk = 100;
		//>2014/06/26 ShermanWei
		//<2014/04/18 ShermanWei
		#if defined(ARIMA_LO1_HW)
		gRawData.cal_delta = 100;
		#elif defined(ARIMA_LO2_HW)
//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
//<2014/07/26 ShermanWei Modify Patch2
		gRawData.cal_delta = 50;///100
//<2014/07/26 ShermanWei
//2014/07/31-42093-RichardLi
		#endif
		//<2014/04/18 ShermanWei
//<2014/05/05 ShermanWei for MiddleGain
#if 0///defined ARIMA_PCBA_RELEASE //<2014/06/26 ShermanWei
		set_psensor_intr_threshold((epl_data->hw ->ps_threshold_low)<<3,(epl_data->hw ->ps_threshold_high)<<3);
#endif
//>2014/05/05 ShermanWei
	}
//>2014/04/17 ShermanWei
	return 0;
}

//<2014/03/25 ShermanWei

static int elan_calibaration_write(struct epl_priv *epl_data, char* h_data, int h_length, char* l_data, int l_length)
{
    struct file *fp_h;
	struct file *fp_l;
	struct i2c_client *client = epl_data->client;

	mm_segment_t fs;
	loff_t pos;
	int read_ret = 0;

	APS_LOG("[ELAN] %s\n", __func__);

	///fp_h = filp_open("/data/ps/h-threshold.dat", O_CREAT|O_EXCL|O_RDWR, 0777);
	fp_h = filp_open("/protect_f/h-threshold.dat", O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
	if (IS_ERR(fp_h))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}

	///fp_l = filp_open("/data/ps/l-threshold.dat", O_CREAT|O_EXCL|O_RDWR, 0777);
///	fp_l = filp_open("/protect_f/l-threshold.dat", O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
///	if (IS_ERR(fp_l))
///	{
///		APS_ERR("[ELAN]create file_l error\n");
///		return -1;
///	}

    fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
APS_ERR("[ELAN]h_length=%d, l_length=%d \n",h_length,l_length);
	vfs_write(fp_h, h_data, h_length, &pos);
///	pos = 0;
///	vfs_write(fp_l, l_data, l_length, &pos);

    filp_close(fp_h, NULL);
///	filp_close(fp_l, NULL);
	set_fs(fs);
//<2014/05/06 ShermanWei for without poweroff after fail to read Cali file & then ReCali
gRawData.ps_factory.cal_file_exist = 1;
//<2014/05/06 ShermanWei
	return 0;
}


static int elan_crosstalk_write(struct epl_priv *epl_data, char* ct_data, int ct_length)
{
	struct file *fp_ct;
	struct i2c_client *client = epl_data->client;

	mm_segment_t fs;
	loff_t pos;
	int read_ret = 0;

	APS_LOG("[ELAN] %s\n", __func__);

	fp_ct = filp_open("/protect_f/cross-talk.dat", O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
	if (IS_ERR(fp_ct))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
APS_ERR("[ELAN]ct_length=%d \n",ct_length);
	vfs_write(fp_ct, ct_data, ct_length, &pos);
	filp_close(fp_ct, NULL);
	set_fs(fs);
//<2014/05/06 ShermanWei for without poweroff after fail to read Cali file & then ReCali
gRawData.ps_factory.cal_file_exist = 1;
//<2014/05/06 ShermanWei
	return 0;
}
//>2014/03/25 ShermanWei


#if 1 //ices add by 20140711
static int epl_cali_enable(struct epl_priv *epl_data, int enable)
{

    int ret = 0;


    uint8_t regdata;
    struct i2c_client *client = epl_data->client;

    APS_LOG("[ELAN epl88051] %s enable = %d\n", __func__, enable);

    if(enable)
    {
        regdata = EPL_INT_DISABLE;
        ret = epl_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);

        regdata =  (0 << 4) | EPL_10BIT_ADC | EPL_M_GAIN | EPL_C_SENSING_MODE;;
        ret = epl_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02, regdata);

        regdata = 11 | EPL_SENSING_16_TIME; //100us
        ret = epl_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02, regdata);

        ret = epl_I2C_Write(client,REG_10,W_SINGLE_BYTE,0X02, EPL_GO_MID);
        ret = epl_I2C_Write(client,REG_11,W_SINGLE_BYTE,0x02, EPL_GO_LOW);

        ret = epl_I2C_Write(client,REG_28,W_SINGLE_BYTE,0x02, EPL_DOC_ON_DISABLED);

        ret = epl_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02, EPL_C_RESET);
        ret = epl_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02, EPL_C_START_RUN);
        msleep(105);

    }


    if(ret<0)
    {
        APS_ERR("[ELAN epl8865 error]%s: als_enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}
#endif

static int epl_psensor_enable(struct epl_priv *epl_data, int enable)
{
    int ret = 0;
    int ps_state;
    uint8_t regdata;
    struct i2c_client *client = epl_data->client;

    APS_LOG("[%s]: enable = %d\n", __func__, enable);
//<2014/07/08 ShermanWei
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
    ret = epl_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, EPL_INT_DISABLE | EPL_INT_CH1); // | EPL_PST_8_TIME);
//>2014/07/14-Yuting Shih.
//>2014/07/08 ShermanWei

    if(enable)
    {
//<2014/05/05 ShermanWei for MiddleGain
        regdata =  EPL_PS_MODE |EPL_10BIT_ADC | EPL_M_GAIN;///EPL_L_GAIN ;
//>2014/05/05 ShermanWei
		//<2014/03/25 ShermanWei
		///regdata	=regdata | (epl_data->hw->polling_mode_ps==0? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
        regdata	=regdata | EPL_C_SENSING_MODE;
		//>2014/03/25 ShermanWei
        ret = epl_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

	    regdata = EPL_ILED_LOW_NONDIVIDED | EPL_DRIVE_100MA | EPL_IR_MODE_CURRENT;
        epl_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,regdata);
//<2014/04/15 ShermanWei
        regdata = regdata = PS_INTT|EPL_SENSING_16_TIME; /*PS_INTT|EPL_SENSING_8_TIME;*/
//>2014/04/15 ShermanWei
        ret = epl_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
//<2014/03/25 ShermanWei
//<2014/8/4-42212-LindaGuo, [5503][Common][Psenser]Fix Psenser vendor register error will cause 2.5cm fail .
//<2014/07/30 Ices
////	ret = epl_I2C_Write(client,REG_28,W_SINGLE_BYTE,0x02, EPL_DOC_ON_ENABLED);
//>2014/07/30 Ices
//>2014/8/4-42212-LindaGuo
/*        if(gRawData.ps_factory.cal_finished == 0 &&  gRawData.ps_factory.cal_file_exist ==1)
		    ret=elan_calibaration_read(epl_data);
        APS_LOG("[%s]: cal_finished = %d\, cal_file_exist = %d\n", __func__, gRawData.ps_factory.cal_finished , gRawData.ps_factory.cal_file_exist);
*/
//>2014/03/25 ShermanWei
//<2014/04/17 ShermanWei
///        set_psensor_intr_threshold(epl_data->hw ->ps_threshold_low,epl_data->hw ->ps_threshold_high);
//>2014/04/17 ShermanWei
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
        set_psensor_intr_threshold((epl_data->hw->ps_threshold_low)<<3,(epl_data->hw->ps_threshold_high)<<3); //ices add by 20140712
//>2014/07/14-Yuting Shih.
        ret = epl_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);
        ret = epl_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

//<2014/04/15 ShermanWei
        ///msleep(epl_data->ps_delay);
        msleep(PS_ENABLE_DELAY);
//>2014/04/15
        if(epl_data->hw->polling_mode_ps==0)
        {
            epl_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
            epl_I2C_Read(client);
            gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

            epl_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
            epl_I2C_Read(client);
            ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);

            if(gRawData.ps_state != ps_state)
            {
                regdata =   EPL_INT_FRAME_ENABLE | EPL_INT_CH1;
                ret = epl_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);
            }
            else
            {
                regdata =  EPL_INT_ACTIVE_LOW | EPL_INT_CH1;
                ret = epl_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);
            }
        }

    }
    else
    {
        //sherman? disable command??
        ret = epl_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,EPL_S_SENSING_MODE);
	ret = epl_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,0x2);
    }

    if(ret<0)
    {
        APS_ERR("%s: ps enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    int ret = 0;
    struct epl_priv *epld = epl_obj;
    struct i2c_client *client = epld->client;

    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    ///APS_LOG("%s\n", __func__);

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb   = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb    = (uint8_t) (low_thd & 0x00ff);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n",__func__, low_thd, high_thd);

    epl_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
    epl_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
    epl_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
    epl_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02,low_msb);

    return ret;
}



/*----------------------------------------------------------------------------*/
static void epl_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    APS_LOG("chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    APS_LOG("chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    APS_LOG("chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    APS_LOG("chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    APS_LOG("chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    APS_LOG("chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    APS_LOG("chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    APS_LOG("chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    APS_LOG("chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    APS_LOG("chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    APS_LOG("chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    APS_LOG("chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

}


/*----------------------------------------------------------------------------*/
int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");

    epl_i2c_client=client;

    APS_LOG(" I2C Addr==[0x%x],line=%d\n",epl_i2c_client->addr,__LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_get_addr(struct alsps_hw *hw, struct epl_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_power(struct alsps_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;

    //APS_LOG("power %s\n", on ? "on" : "off");
///#ifndef MT6582
    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, EPL_DEV_NAME))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, EPL_DEV_NAME))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
///#endif
}



/*----------------------------------------------------------------------------*/
static int epl_check_intr(struct i2c_client *client)
{
    struct epl_priv *obj = i2c_get_clientdata(client);
    int mode;

    APS_LOG("int pin = %d\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN));

    //if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
    //   return 0;

    epl_I2C_Write(obj->client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl_I2C_Read(obj->client);
    mode =(gRawData.raw_bytes[0]>>3)&7;
    APS_LOG("mode %d\n", mode);

    if(mode==0x01)// PS
    {
        set_bit(CMC_BIT_PS, &obj->pending_intr);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &obj->pending_intr);
    }

    APS_LOG("check intr: 0x%08X\n", obj->pending_intr);

    return 0;

}
#if 1 //ices add by 20140711
int cali_flag = 0;
#endif
/*----------------------------------------------------------------------------*/
long epl_read_ps(struct i2c_client *client, u16 *data)
{
    struct epl_priv *obj = i2c_get_clientdata(client);
    uint8_t setting;
	//<2014/8/7-42315-LindaGuo , [5503][Common][Psenser]Fix the Psenser glare issue .
	u16 tmp_state;
	u16 tmp_raw;
	//>2014/8/7-42315-LindaGuo
//<2014/07/04 ShermanWei,strong light bug
	u8 con_sat = 0;
	u16 env_ch0 = 0;
#if 1 //ices add by 20140711
    u16 clai_env_ch0 = 0;
#endif
//>2014/07/04 ShermanWei
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }
///#if 1//ices add by 20140711
if(cali_flag == 0)
{
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
    epl_cali_enable(obj, 1);
//>2014/07/14-Yuting Shih.
    mutex_lock(&epl88051_mutex);
    epl_I2C_Write(obj->client,REG_14,R_TWO_BYTE,0x01,0x00);
    epl_I2C_Read(obj->client);
    clai_env_ch0 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    APS_LOG("[%s]:clai_env_ch0=%d \r\n", __func__, clai_env_ch0);
	//<2014/8/7-42315-LindaGuo , [5503][Common][Psenser]Fix the Psenser glare issue .
    if(clai_env_ch0 < 30000)
    {
        cali_flag =1;
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
        gRawData.dynaK = 1;
        gRawData.buffer_ready = 1;  //ices add by 20140807
        APS_LOG("[%s]: gRawData.buffer_ready=%d \r\n ", gRawData.buffer_ready); //ices add by 20140807
//>2014/07/14-Yuting Shih.
        epl_psensor_enable(obj, 1);
    }
    else
    {
        gRawData.buffer_ready = 0;  //ices add by 20140807
        APS_LOG("[%s]: gRawData.buffer_ready=%d \r\n ", gRawData.buffer_ready); //ices add by 20140807
    }
	//>2014/8/7-42315-LindaGuo


    mutex_unlock(&epl88051_mutex);
}
else
///#endif
{
//<2014/06/26 ShermanWei,
mutex_lock(&epl88051_mutex);
//<2014/07/04 ShermanWei,strong light bug
    epl_I2C_Write(obj->client,REG_14,R_TWO_BYTE,0x01,0x00);
    epl_I2C_Read(obj->client);
    env_ch0 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
//>2014/07/04 ShermanWei
    epl_I2C_Write(obj->client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl_I2C_Read(obj->client);
    setting = gRawData.raw_bytes[0];
//<2014/07/04 ShermanWei,strong light bug
    con_sat = (gRawData.raw_bytes[0] >> 1) & 0x01;
//>2014/07/04 ShermanWei
    if(((setting>>3)&7)!=0x01)
    {
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
        cali_flag =1;
//>2014/07/14-Yuting Shih.
        APS_ERR("read ps data in wrong mode\n");
//<2014/03/25 ShermanWei
	gRawData.buffer_ready = 0;
//<2014/06/26 ShermanWei,
mutex_unlock(&epl88051_mutex);
	return 0;
//>2014/03/25 ShermanWei
    }

	//<2014/8/7-42315-LindaGuo , [5503][Common][Psenser]Fix the Psenser glare issue .
	tmp_state= !((gRawData.raw_bytes[0]&0x04)>>2);
	//<2014/8/7-42315-LindaGuo

    epl_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
    epl_I2C_Read(obj->client);
    gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    //<2014/05/05 ShermanWei for MiddleGain
	//<2014/8/7-42315-LindaGuo , [5503][Common][Psenser]Fix the Psenser glare issue .
    tmp_raw = gRawData.ps_raw >> 3;
	//<2014/8/7-42315-LindaGuo
    //>2014/05/05 ShermanWei
    /// *data = gRawData.ps_raw ;
//<2014/06/26 ShermanWei,
mutex_unlock(&epl88051_mutex);
//<2014/03/25 ShermanWei
#if 0///defined ARIMA_PCBA_RELEASE
    APS_LOG("read ps raw data = %d\n", gRawData.ps_raw);
    APS_LOG("read ps binary data = %d\n", gRawData.ps_state);
#else
///Dynamic K
///add delta to set threshold
	APS_LOG("ps rawdata:%d,ps_state=%d,dynaK=%d,dynaKagain=%d,cal_delta=%d,con_sat=%d,env_ch0=%d \n", tmp_raw, tmp_state, gRawData.dynaK, gRawData.dynaKagain, gRawData.cal_delta, con_sat, env_ch0);
#if 1   //ices add by 20140709
    //<2014/07/04 ShermanWei,strong light bug
    if ( (con_sat == 1) || (env_ch0 > 30000) )
    {
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
        cali_flag = 0;
        return 0;
//>2014/07/14-Yuting Shih.
	//<2014/8/7-42315-LindaGuo , [5503][Common][Psenser]Fix the Psenser glare issue .
    }
	else{
	// 0807 renato add
		gRawData.ps_state=tmp_state;
		gRawData.ps_raw=tmp_raw;
	}
	//>2014/8/7-42315-LindaGuo
    //>2014/07/04 ShermanWei,strong light bug
    //<2014/06/26 ShermanWei,near state ReDynamicCal
    if ( (gRawData.dynaKagain == 1) && (gRawData.ps_raw < (gRawData.ps_factory.cal_ctalk + 130)) )
    {
	gRawData.dynaKagain = 0;
	gRawData.dynaK = 1;
    }
#else
    if ( (con_sat == 1) || (env_ch0 > 30000) || (gRawData.ps_raw > (gRawData.ps_factory.cal_ctalk+2000)) )
    {
        gRawData.dynaKagain = 0;
	    gRawData.dynaK = 0;
        APS_LOG("[%s]:Recover default H/L threshold:Low=%d,High=%d \r\n", __func__ , epl_obj->hw->ps_threshold_low<<3, epl_obj->hw->ps_threshold_high<<3);
	    set_psensor_intr_threshold((epl_obj->hw->ps_threshold_low<<3),(epl_obj->hw->ps_threshold_high<<3));    //set default H/L threshold
    }
    else if( (gRawData.dynaKagain == 1) && (gRawData.ps_raw < (gRawData.ps_factory.cal_ctalk + 130)) )
    {
        gRawData.dynaKagain = 0;
	    gRawData.dynaK = 1;
    }
#endif
    //>2014/06/26 ShermanWei
	if (gRawData.dynaK == 1)
	{
	    u16 dyna_threshold;
	    u16 dyna_threshold_low;
	//<2014/07/14-Yuting Shih. Merged with ELAN patch.
	    //gRawData.ps_state =1;//ices add by 20140712
	//>2014/07/14-Yuting Shih.
		//<2014/04/18 ShermanWei
//<2014/05/05 ShermanWei for MiddleGain
//	    if (gRawData.cal_delta < 50) {
//		#if defined(ARIMA_LO1_HW)
//		gRawData.cal_delta = 100;
//		#elif defined(ARIMA_LO2_HW)
//		gRawData.cal_delta = 100;
//		#endif
//	    }
//>2014/05/05 ShermanWei
		//<2014/04/18 ShermanWei

//2014/07/26-41913-RichardLi,[5502][Commom][Psensor] update Psensor driver to fix Lo2 can not turn on screen
//<2014/07/26 ShermanWei Remove Patch1
//	    if (gRawData.ps_raw == 0)///for abnormal read 0 while enable
//    	    {
//		msleep(2);
//		epl_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
//    		epl_I2C_Read(obj->client);
//    		gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
//	    	//<2014/05/05 ShermanWei for MiddleGain
//	    	gRawData.ps_raw = gRawData.ps_raw >> 3;
//	    	//>2014/05/05 ShermanWei
//		APS_LOG("Again ++ read ps raw data= %d \n", gRawData.ps_raw);
//		//<2014/07/01 ShermanWei, for strong light
//		if (gRawData.ps_raw == 0)
//		{
//		//<2014/07/14-Yuting Shih. Merged with ELAN patch.
//		    cali_flag = 0;
//		    return 0;
//		//<2014/07/14-Yuting Shih. Merged with ELAN patch.
//       }
//		//>2014/07/01 ShermanWei,
//    	    }
//2014/07/26-41913-RichardLi
	    //<2014/06/26 ShermanWei,near state ReDynamicCal
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
#if 0 //ices add by 20140712
	    if ( gRawData.ps_raw > (gRawData.ps_factory.cal_ctalk + 1300) )
	    gRawData.dynaKagain = 1;
#else
        //if ( gRawData.ps_raw > (gRawData.ps_factory.cal_ctalk + 1300) )
        if ( gRawData.ps_raw > (gRawData.ps_factory.cal_ctalk + gRawData.cal_delta*9) )
        {

            gRawData.dynaKagain = 1;
            cali_flag = 0;
            APS_LOG("[%s]:Recover default H/L threshold and re-cali \r\n", __func__);
            return 0;
        }

#endif
//>2014/07/14-Yuting Shih.
	    //>2014/06/26 ShermanWei
//<2014/04/28 ShermanWei ,change H-threshold rule
	    dyna_threshold = (9 * gRawData.cal_delta / 10) + gRawData.ps_raw;
//<2014/04/28 ShermanWei ,change H-threshold rule
	    dyna_threshold_low = (gRawData.cal_delta/2) + gRawData.ps_raw;
	    ///set_psensor_intr_threshold( (dyna_threshold*8)/10, dyna_threshold );
//<2014/05/05 ShermanWei for MiddleGain
	    set_psensor_intr_threshold( dyna_threshold_low<<3, dyna_threshold<<3 );
//<2014/05/05 ShermanWei
	    gRawData.dynaK = 0;
	}

#endif
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
    //2014/08/06-42311-RichardLi,[5503][Common][Psensor] Add ELAN 0806 patch to fix India test can not turn on LCD issue
	//<2014/8/7-42315-LindaGuo , [5503][Common][Psenser]Fix the Psenser glare issue .
	//gRawData.buffer_ready = 1;    //ices del by 20140807
	//>2014/8/7-42315-LindaGuo
	//2014/08/06-42311-RichardLi
}
    //2014/08/06-42311-RichardLi,[5503][Common][Psensor] Add ELAN 0806 patch to fix India test can not turn on LCD issue
    //gRawData.buffer_ready = 1;
	//2014/08/06-42311-RichardLi

//>2014/07/14-Yuting Shih.
//>2014/03/25 ShermanWei
    return 0;
}

void epl_restart_polling(void)
{
    struct epl_priv *obj = epl_obj;
    cancel_delayed_work(&obj->polling_work);
//<2014/03/25 ShermanWei
    ///schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(50));
    schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(1));
//>2014/03/25 ShermanWei
}


void epl_polling_work(struct work_struct *work)
{
    struct epl_priv *obj = epl_obj;
    struct i2c_client *client = obj->client;

    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_LOG("ps: %d  \n", enable_ps);

    cancel_delayed_work(&obj->polling_work);

    if(enable_ps==true && obj->hw->polling_mode_ps==1)
    {
        //<2014/03/25 ShermanWei
        ///schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(2*obj->ps_delay+30));
        schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(obj->ps_delay));
        //>2014/03/25 ShermanWei
    }


    if(enable_ps)
    {
        /////epl_psensor_enable(obj, 1);
        if(obj->hw->polling_mode_ps==1)
        {
            epl_read_ps(client, &gRawData.ps_raw);
        }
    }


    if(enable_ps==false)
    {
        APS_LOG("disable sensor\n");
        cancel_delayed_work(&obj->polling_work);
		//sherman? REG_10???
        ////epl_I2C_Write(client,REG_10,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
        epl_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,EPL_S_SENSING_MODE);
	epl_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,0x2);
    }

}


/*----------------------------------------------------------------------------*/
void epl_eint_func(void)
{
/*
    struct epl_priv *obj = epl_obj;

    // APS_LOG(" interrupt fuc\n");

    if(!obj)
    {
        return;
    }

#ifdef MT6582
    mt_eint_mask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif

    schedule_delayed_work(&obj->eint_work, 0);
*/
}



/*----------------------------------------------------------------------------*/
static void epl_eint_work(struct work_struct *work)
{
/*
    struct epl_priv *epld = epl_obj;
    int err;
    hwm_sensor_data sensor_data;

    if(test_bit(CMC_BIT_PS, &epld->enable))
    {
        APS_LOG("xxxxx eint work\n");
        if((err = epl_check_intr(epld->client)))
        {
            APS_ERR("check intrs: %d\n", err);
        }

        if(epld->pending_intr)
        {
            epl_I2C_Write(epld->client,REG_13,R_SINGLE_BYTE,0x01,0);
            epl_I2C_Read(epld->client);
            gRawData.ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);
            APS_LOG("ps state = %d\n", gRawData.ps_state);

            epl_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
            epl_I2C_Read(epld->client);
            gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
            APS_LOG("ps raw_data = %d\n", gRawData.ps_raw);

            sensor_data.values[0] = gRawData.ps_state;
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

            //let up layer to know
            if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
            {
                APS_ERR("get interrupt data failed\n");
                APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
            }
        }

        epl_I2C_Write(epld->client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW | EPL_INT_CH1);
        epl_I2C_Write(epld->client,REG_8,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
    }


exit:

#ifdef MT6575
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

#ifdef MT6589
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

#ifdef MT6582
    mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif
*/
}



/*----------------------------------------------------------------------------*/
int epl_setup_eint(struct i2c_client *client)
{
    struct epl_priv *obj = i2c_get_clientdata(client);

    APS_LOG("epl_setup_eint\n");

///    epl_obj = obj;
    /*configure to GPIO function, external interrupt*/
/*
    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#ifdef  MT6575
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_EDGE_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, epl_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

#ifdef  MT6589
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_EDGE_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, epl_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

#ifdef MT6582
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif
*/
    return 0;
}




/*----------------------------------------------------------------------------*/
static int epl_init_client(struct i2c_client *client)
{
    struct epl_priv *obj = i2c_get_clientdata(client);
    int err=0;

    /*  interrupt mode */


    APS_FUN();
/*
    if(obj->hw->polling_mode_ps == 0)
    {
#ifdef MT6582
        mt_eint_mask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif

        if((err = epl_setup_eint(client)))
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl interrupt setup\n");
    }
*/

    if((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }

	//<2014/03/25 ShermanWei
	/*
    if((err = epl_check_intr(client)))
    {
        APS_ERR("check/clear intr: %d\n", err);
        return err;
    }*/
    //>2014/03/25 ShermanWei


    /*  interrupt mode */
//if(obj->hw->polling_mode_ps == 0)
    //     mt65xx_eint_unmask(CUST_EINT_ALS_NUM);

    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_show_reg(struct device_driver *ddri, char *buf)
{
    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    ssize_t len = 0;
    struct i2c_client *client = epl_obj->client;

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

    return len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl_priv *epld = epl_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;

    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "ps enable is %d \r\n",test_bit(CMC_BIT_PS, &epld->enable));
    len += snprintf(buf+len, PAGE_SIZE-len, "ps int time is %d \r\n", PS_INTT);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps delay is %d \r\n", epld->ps_delay);

    if(epld->hw->polling_mode_ps==0 && enable_ps == true){
        msleep(epld->ps_delay);
        /*read channel 1 raw data*/
        epl_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
        epl_I2C_Read(epld->client);
        gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "ps_state=%d  \r\n", gRawData.ps_state);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps_raw=%d  \r\n", gRawData.ps_raw);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps threshold is %d / %d \r\n",epld->hw->ps_threshold_low, epld->hw->ps_threshold_high);

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_show_ps_cal_raw(struct device_driver *ddri, char *buf)
{
    APS_FUN();
    long *tmp = (long*)buf;
    struct epl_priv *epld = epl_obj;
    u16 ch1=0;
    u32 ch1_all=0;
//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
//<2014/03/25 ShermanWei
    ///int count =5, i;
    int count =3, i;
    ///int high_int = 12345;
    //char high[8];
    //int high_length =0;
///    char* low = "2456";
    //char low[8];
    ///int low_int = 86;
    //int low_length =0;
//>2014/03/25 ShermanWei
//2014/07/31-42093-RichardLi
    ssize_t len = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
APS_LOG("[%s]: enable_ps=%d\n", __FUNCTION__, enable_ps);
    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    if(enable_ps == false){
        set_bit(CMC_BIT_PS, &epld->enable);
//<2014/03/25 ShermanWei
        ///epl_restart_polling();
        epl_psensor_enable(epld, 1);
        ////msleep(2);
//>2014/03/25 ShermanWei
    }

//<2014/06/26 ShermanWei,
mutex_lock(&epl88051_mutex);
    for(i=0; i<count; i++)
    {
//<2014/03/25 ShermanWei
        ///msleep(epld->ps_delay);
        ///if(epld->hw->polling_mode_ps==0 && enable_ps == true){
            epl_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
            epl_I2C_Read(epld->client);
            gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
	    //<2014/05/05 ShermanWei for MiddleGain
	    gRawData.ps_raw = gRawData.ps_raw >> 3;
	    //>2014/05/05 ShermanWei
        ///}
		ch1_all = ch1_all+ gRawData.ps_raw;
//>2014/03/25 ShermanWei
    }
//<2014/06/26 ShermanWei,
mutex_unlock(&epl88051_mutex);
    ch1 = (u16)ch1_all/count;
	APS_LOG("[%s]: ch1=%d\n", __FUNCTION__, ch1);
	epl_psensor_enable(epld, 0);
	clear_bit(CMC_BIT_PS, &epld->enable);
//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
//<2014/03/25 ShermanWei
	memset(high, '\0', sizeof(high));
	memset(low, '\0', sizeof(low));
	high_length = 0;
	low_length = 0;
	high_length = sprintf(high, "%d",  ch1/*high_int*/);
	low_length = sprintf(low, "%d",  (ch1*9)/10/*low_int*/);
	//////elan_calibaration_write(epld, high, high_length, low, low_length);
//>2014/03/25 ShermanWei
//2014/07/31-42093-RichardLi
	if (gRawData.show_screen == 1)
	{
	len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ch1);
	return len;
	}
	else
	{
	tmp[0] = ch1;
	return 2;
	}
}


//<2014/03/25 ShermanWei
/*----------------------------------------------------------------------------*/
static ssize_t epl_show_ps_crosstalk_raw(struct device_driver *ddri, char *buf)
{
    APS_FUN();
    long *tmp = (long*)buf;
    struct epl_priv *epld = epl_obj;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =3, i;
	//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
    //char ctalk[8];
    //int ctalk_length =0;
	//2014/07/31-42093-RichardLi

    ssize_t len = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
APS_LOG("[%s]: enable_ps=%d\n", __FUNCTION__, enable_ps);
    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    if(enable_ps == false){
        set_bit(CMC_BIT_PS, &epld->enable);
        epl_psensor_enable(epld, 1);
        ////msleep(2);
    }

//<2014/06/26 ShermanWei,
mutex_lock(&epl88051_mutex);
    for(i=0; i<count; i++)
    {
        epl_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
        epl_I2C_Read(epld->client);
        gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
	//<2014/05/05 ShermanWei for MiddleGain
	gRawData.ps_raw = gRawData.ps_raw >> 3;
	//>2014/05/05 ShermanWei
	ch1_all = ch1_all+ gRawData.ps_raw;
    }
//<2014/06/26 ShermanWei,
mutex_unlock(&epl88051_mutex);
    ch1 = (u16)ch1_all/count;
	APS_LOG("[%s]: ch1=%d\n", __FUNCTION__, ch1);
	epl_psensor_enable(epld, 0);
	clear_bit(CMC_BIT_PS, &epld->enable);
	memset(ctalk, '\0', sizeof(ctalk));
	//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
	ctalk_length = 0;
	//2014/07/31-42093-RichardLi
	ctalk_length = sprintf(ctalk, "%d",  ch1);
    //2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
	//////elan_crosstalk_write(epld, ctalk, ctalk_length);
	//2014/07/31-42093-RichardLi
	if (gRawData.show_screen == 1)
	{
	len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ch1);
	return len;
	}
	else
	{
	tmp[0] = ch1;
	return 2;
	}
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_show_ps_rawdata(struct device_driver *ddri, char *buf)
{
    APS_FUN();
	long *tmp = (long*)buf;
    struct epl_priv *epld = epl_obj;
    u16 ch1=0;
    u32 ch1_all=0;

    int count =3, i;

    ssize_t len = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
APS_LOG("[%s]: enable_ps=%d\n", __FUNCTION__, enable_ps);
    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    if(enable_ps == false){
        set_bit(CMC_BIT_PS, &epld->enable);
	epl_psensor_enable(epld, 1);
	////msleep(2);
//<2014/06/26 ShermanWei,
/////    }

//<2014/06/26 ShermanWei,
mutex_lock(&epl88051_mutex);
    for(i=0; i<count; i++)
    {
		epl_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
		epl_I2C_Read(epld->client);
		gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
		//<2014/05/05 ShermanWei for MiddleGain
		gRawData.ps_raw = gRawData.ps_raw >> 3;
		//>2014/05/05 ShermanWei
		ch1_all = ch1_all+ gRawData.ps_raw;
    }
//<2014/06/26 ShermanWei,
mutex_unlock(&epl88051_mutex);
    ch1 = (u16)ch1_all/count;
	APS_LOG("[%s]: ch1=%d\n", __FUNCTION__, ch1);
	epl_psensor_enable(epld, 0);
	clear_bit(CMC_BIT_PS, &epld->enable);
	APS_LOG("[%s]: gRawData.show_screen=%d\n", __FUNCTION__, gRawData.show_screen);
//<2014/06/26 ShermanWei,
} else {
ch1 = gRawData.ps_raw;///1;//<2014/07/10 ShermanWei,
APS_LOG("[%s]: GetPsensor ch1=%d\n", __FUNCTION__, ch1);
}
	if (gRawData.show_screen == 1)
	{
	len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ch1);
	return len;
	}
	else
	{
	tmp[0] = ch1;
	return 2;
	}
}



//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
static ssize_t epl_show_ps_savedata(struct device_driver *ddri, char *buf)
{
    struct epl_priv *epld = epl_obj;

    APS_LOG("[%s]: \n", __FUNCTION__);

    elan_calibaration_write(epld, high, high_length, low, low_length);
    elan_crosstalk_write(epld, ctalk, ctalk_length);

}
//2014/07/31-42093-RichardLi

/*----------------------------------------------------------------------------*/
static ssize_t epl_store_ps_screen_show(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d", &gRawData.show_screen);

    return count;
}

static ssize_t epl_show_ps_screen_show(struct device_driver *ddri, const char *buf, size_t count)
{
    ssize_t len = 0;
    struct epl_priv *epld = epl_obj;

    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "ps_screen_show=%d \r\n", gRawData.show_screen);
    return len;
}
//>2014/03/25 ShermanWei


/*----------------------------------------------------------------------------*/

static ssize_t epl_store_ps_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d", &PS_INTT);
    APS_LOG("ps int time is %d\n", PS_INTT);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_store_ps_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d", &epl_obj->hw->ps_threshold_low, &epl_obj->hw->ps_threshold_high);
    gRawData.ps_factory.cal_l = epl_obj->hw->ps_threshold_low;
    gRawData.ps_factory.cal_h = epl_obj->hw->ps_threshold_high;
    set_psensor_intr_threshold(epl_obj->hw->ps_threshold_low,epl_obj->hw->ps_threshold_high);

    return count;
}

static ssize_t epl_show_ps_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
    ssize_t len = 0;
    struct epl_priv *epld = epl_obj;

    if(!epl_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "ps_threshold_low=%d \r\n", epl_obj->hw->ps_threshold_low);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps_threshold_high=%d \r\n", epl_obj->hw->ps_threshold_high);

    gRawData.ps_factory.cal_l = epl_obj->hw->ps_threshold_low;
    gRawData.ps_factory.cal_h = epl_obj->hw->ps_threshold_high;

    epl_restart_polling();

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_store_ps_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_priv *obj = epl_obj;
    struct i2c_client *client = obj->client;
    struct hwmsen_object obj_ps;

    obj_ps.self = obj;
    sscanf(buf, "%d",&obj->hw->polling_mode_ps);

    if(obj->hw->polling_mode_ps==0)
    {
        obj_ps.polling = 0;
        epl_setup_eint(client);
    }
    else
    {
        obj_ps.polling = 1;
    }
    return count;
}

static ssize_t epl_show_ps_delay(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_priv *epld = epl_obj;
    ssize_t len = 0;

    if(!epld)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "epld->ps_delay=%d \r\n", epld->ps_delay);

    return len;
}

static ssize_t epl_store_ps_delay(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_priv *epld = epl_obj;
    ssize_t len = 0;

    if(!epld)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d",&epld->ps_delay);
    epl_restart_polling();

    return count;
}

//<2014/08/01-samhuang, add for CT become to high
static ssize_t epl_all_calling_end(struct device_driver *ddri, char *buf)
{
    struct epl_priv *epld = epl_obj;

    APS_LOG("[%s]: \n", __FUNCTION__);

}
//>2014/08/01-samhuang
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,					S_IWUSR  | S_IRUGO, epl_show_status,  	  		NULL);
static DRIVER_ATTR(elan_reg,    				S_IWUSR  | S_IRUGO, epl_show_reg,   			NULL);
static DRIVER_ATTR(ps_cal_raw, 				    S_IWUSR  | S_IRUGO, epl_show_ps_cal_raw, 	  	NULL);
static DRIVER_ATTR(ps_crosstalk_raw, 				    S_IWUSR  | S_IRUGO, epl_show_ps_crosstalk_raw, 	  	NULL);
static DRIVER_ATTR(ps_rawdata, 				    S_IWUSR  | S_IRUGO, epl_show_ps_rawdata, 	  	NULL);
//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
static DRIVER_ATTR(ps_savedata, 				    S_IWUSR  | S_IRUGO, epl_show_ps_savedata, 	  	NULL);
//2014/07/31-42093-RichardLi
static DRIVER_ATTR(ps_screen_show,     			S_IWUSR  | S_IRUGO, epl_show_ps_screen_show, epl_store_ps_screen_show);
static DRIVER_ATTR(ps_int_time,     			S_IWUSR  | S_IRUGO, NULL,   					epl_store_ps_int_time);
static DRIVER_ATTR(ps_threshold,     			S_IWUSR  | S_IRUGO, epl_show_ps_threshold, epl_store_ps_threshold);
static DRIVER_ATTR(ps_polling_mode,			    S_IWUSR  | S_IRUGO, NULL, epl_store_ps_polling_mode);
static DRIVER_ATTR(ps_delay,			        S_IWUSR  | S_IRUGO, epl_show_ps_delay, epl_store_ps_delay);

/*----------------------------------------------------------------------------*/
static struct device_attribute * epl_attr_list[] =
{
//    &driver_attr_elan_status,
//    &driver_attr_elan_reg,
    &driver_attr_ps_cal_raw,
    &driver_attr_ps_crosstalk_raw,
    &driver_attr_ps_rawdata,
	//2014/07/31-42093-RichardLi,[5503][Common][Psensor] Modify Psensor cal data save condition from Sherman
    &driver_attr_ps_savedata,
	//2014/07/31-42093-RichardLi
    &driver_attr_ps_screen_show,
//    &driver_attr_ps_int_time,
//    &driver_attr_ps_threshold,
//    &driver_attr_ps_polling_mode,
//    &driver_attr_ps_delay,
};

/*----------------------------------------------------------------------------*/
static int epl_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl_attr_list)/sizeof(epl_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, epl_attr_list[idx]))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl_attr_list)/sizeof(epl_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl_attr_list[idx]);
    }

    return err;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl_open(struct inode *inode, struct file *file)
{
    file->private_data = epl_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long epl_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;

    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            if(enable)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
            }
            else
            {
                clear_bit(CMC_BIT_PS, &obj->enable);
            }
            epl_restart_polling();
            break;


        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_DATA:
            dat = gRawData.ps_state;

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_RAW_DATA:
            dat = gRawData.ps_raw;

            APS_LOG("ioctl ps raw value = %d \n", dat);
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
        case ALSPS_GET_PS_THRESHOLD_HIGH:
            dat = obj->hw ->ps_threshold_high;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_THRESHOLD_LOW:
            dat = obj->hw ->ps_threshold_low;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl_fops =
{
    .owner = THIS_MODULE,
    .open = epl_open,
    .release = epl_release,
    .unlocked_ioctl = epl_unlocked_ioctl,
};


/*----------------------------------------------------------------------------*/
static struct miscdevice epl_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl_fops,
};


//<2014/03/25 ShermanWei,wakelock while psensor on
void epl_psensor_lock(int enable_pflag){

	APS_LOG("[%s], enable_pflag=%d\n",__func__,enable_pflag);

	if(enable_pflag){
		wake_lock(&g_ps_wlock);
	}
	else{
		wake_unlock(&g_ps_wlock);
	}

}
//>2014/03/25 ShermanWei,


/*----------------------------------------------------------------------------*/
static int epl_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct epl_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();
//<2014/03/25 ShermanWei
/*
    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }

        atomic_set(&obj->ps_suspend, 1);

        if(test_bit(CMC_BIT_PS,  &obj->enable) && obj->hw->polling_mode_ps==0)
            epl_restart_polling();

        epl_power(obj->hw, 0);
    }
*/
//>2014/03/25
    return 0;

}



/*----------------------------------------------------------------------------*/
static int epl_i2c_resume(struct i2c_client *client)
{
    struct epl_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();
//<2014/03/25 ShermanWei
/*
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    epl_power(obj->hw, 1);

    msleep(50);

    atomic_set(&obj->ps_suspend, 0);

    if(err = epl_init_client(client))
    {
        APS_ERR("initialize client fail!!\n");
        return err;
    }

    if(obj->hw->polling_mode_ps == 0)
        epl_setup_eint(client);


    if(test_bit(CMC_BIT_PS,  &obj->enable))
        epl_restart_polling();
*/
//>2014/03/25
    return 0;
}



/*----------------------------------------------------------------------------*/
static void epl_early_suspend(struct early_suspend *h)
{
	APS_FUN();
}



/*----------------------------------------------------------------------------*/
static void epl_late_resume(struct early_suspend *h)
{
	APS_FUN();
}


/*----------------------------------------------------------------------------*/
int epl_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
                       void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct epl_priv *obj = (struct epl_priv *)self;

    APS_LOG("epl_ps_operate command = %x\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            break;


        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                APS_LOG("ps enable = %d\n", value);
		//<2014/03/25 ShermanWei
		gRawData.buffer_ready = 0;
		//>2014/03/25 ShermanWei
                if(value)
                {
                    /*
                    if(obj->hw->polling_mode_ps==0)
                        gRawData.ps_state = 2;
                    */
                    set_bit(CMC_BIT_PS, &obj->enable);
		    //<2014/06/26 ShermanWei,near state ReDynamicCal
		    gRawData.dynaKagain = 0;
		    //>2014/06/26 ShermanWei
                    //<2014/03/25 ShermanWei
		    gRawData.dynaK =1;
#if 0//ices add by 20140711
		    epl_psensor_enable(obj, 1);
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
		    elan_calibaration_read(obj);
//>2014/07/14-Yuting Shih.
#else
//<2014/07/14-Yuting Shih. Merged with ELAN patch.
            elan_calibaration_read(obj);
		    //epl_cali_enable(obj, 1);
		    cali_flag = 0;
//>2014/07/14-Yuting Shih.
#endif
                    epl_restart_polling();
                    //>2014/03/25
		    //<2014/03/25 ShermanWei,
		    epl_psensor_lock(1);
		    //>2014/03/25 ShermanWei,
                }
                else
                {
                    clear_bit(CMC_BIT_PS, &obj->enable);
		    //<2014/03/25 ShermanWei,
		    epl_psensor_lock(0);
		    epl_restart_polling();
		    //>2014/03/25 ShermanWei,
                }
                ////epl_restart_polling();
            }

            break;



        case SENSOR_GET_DATA:
            APS_LOG(" get ps data !!!!!!\n");
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {

                APS_LOG("---SENSOR_GET_DATA:buffer_ready=%d,ps_state=%d,ps_raw=%d---\n\n",gRawData.buffer_ready,gRawData.ps_state,gRawData.ps_raw);
				APS_LOG("report cali_flag %d\n", cali_flag); //0807
//<2014/06/26 ShermanWei,
mutex_lock(&epl88051_mutex);
                sensor_data = (hwm_sensor_data *)buff_out;
		if (gRawData.buffer_ready)
		{
		APS_LOG("report state %d\n", gRawData.ps_state); //0807
	                sensor_data->values[0] = gRawData.ps_state;
	                sensor_data->values[1] = gRawData.ps_raw;
		}
		else
		{
		APS_LOG("report state far \n");//0807
			sensor_data->values[0] = 1;
			sensor_data->values[1] = 0;
		}
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
//<2014/06/26 ShermanWei,
mutex_unlock(&epl88051_mutex);
            }
            break;


        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;



    }

    return err;

}

/*----------------------------------------------------------------------------*/

static int epl_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, EPL_DEV_NAME);
    return 0;
}


/*----------------------------------------------------------------------------*/
static int epl_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl_priv *obj;
    struct hwmsen_object obj_ps;
    int err = 0;
    APS_FUN();

    epl_dumpReg(client);

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl_obj = obj;
    obj->hw = epl88051_get_cust_alsps_hw();

    epl_get_addr(obj->hw, &obj->addr);

///    INIT_DELAYED_WORK(&obj->eint_work, epl_eint_work);
    INIT_DELAYED_WORK(&obj->polling_work, epl_polling_work);

    obj->client = client;

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->ps_suspend, 0);

    obj->enable = 0;
    obj->pending_intr = 0;
    obj->ps_delay = PS_DELAY;

    epl_i2c_client = client;

    epl_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
    epl_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, EPL_INT_DISABLE);

    if(err = epl_init_client(client))
    {
        goto exit_init_failed;
    }


    if(err = misc_register(&epl_device))
    {
        APS_ERR("epl_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    ////if(err = epl_create_attr(&epl_alsps_driver.driver))
	if( err = epl_create_attr(&(epl88051_init_info.platform_diver_addr->driver))  )
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_ps.self = epl_obj;

    if( obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_ps.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }


    obj_ps.sensor_operate = epl_ps_operate;



    if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    gRawData.ps_factory.cal_file_exist = 1;
    gRawData.ps_factory.cal_finished = 0;
    gRawData.show_screen = 1;

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    obj->early_drv.suspend  = epl_early_suspend,
    obj->early_drv.resume   = epl_late_resume,
    register_early_suspend(&obj->early_drv);
#endif
	//<2014/03/25 ShermanWei,wakelock while psensor on
	wake_lock_init(&g_ps_wlock, WAKE_LOCK_SUSPEND, "ps_wakelock");
	//>2014/03/25 ShermanWei,wakelock while psensor on
/*
    if(obj->hw->polling_mode_ps ==0)
        epl_setup_eint(client);
*/
    epl88051_init_flag = 0;
    APS_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&epl_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
//	exit_kfree:
    kfree(obj);
exit:
    epl_i2c_client = NULL;

    epl88051_init_flag = -1;
    APS_ERR("%s: err = %d\n", __func__, err);
    return err;



}



/*----------------------------------------------------------------------------*/
static int epl_i2c_remove(struct i2c_client *client)
{
    int err;

    if(err = epl_delete_attr(&epl_i2c_driver.driver))
    {
        APS_ERR("epl_delete_attr fail: %d\n", err);
    }

    if(err = misc_deregister(&epl_device))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}



/*----------------------------------------------------------------------------*/
static int epl_probe(struct platform_device *pdev)
{
    struct alsps_hw *hw = epl88051_get_cust_alsps_hw();

    epl_power(hw, 1);

    if(i2c_add_driver(&epl_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }
    return 0;
}



/*----------------------------------------------------------------------------*/
static int epl_local_init(void)
{
    struct alsps_hw *hw = epl88051_get_cust_alsps_hw();

    APS_FUN();
    epl_power(hw, 1);

    if(i2c_add_driver(&epl_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }
    if (-1 == epl88051_init_flag)
    return -1;
    else
    return 0;
}


/*----------------------------------------------------------------------------*/
////static int epl_remove(struct platform_device *pdev)
static int epl_remove(void)
{
    struct alsps_hw *hw = epl88051_get_cust_alsps_hw();
    APS_FUN();
    epl_power(hw, 0);

    APS_ERR("EPL remove \n");
    i2c_del_driver(&epl_i2c_driver);
    return 0;
}



/*----------------------------------------------------------------------------*/
/*
static struct platform_driver epl_alsps_driver =
{
    .probe      = epl_probe,
    .remove     = epl_remove,
    .driver     = {
        .name  = "als_ps",
        //.owner = THIS_MODULE,
    }

};
*/
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int __init epl88051_init(void)
{
    struct alsps_hw *hw = epl88051_get_cust_alsps_hw();
    APS_FUN();
    i2c_register_board_info(hw->i2c_num, &i2c_EPL, 1);
    ////if(platform_driver_register(&epl_alsps_driver))
    if(hwmsen_alsps_sensor_add(&epl88051_init_info))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }

    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit epl88051_exit(void)
{
    APS_FUN();
    ////platform_driver_unregister(&epl_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(epl88051_init);
module_exit(epl88051_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL88051 PS driver");
MODULE_LICENSE("GPL");

