//<2014/04/01-tedwu, Add for battery ID checking feature.

#include <mach/charging.h>
//#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
//#include <cust_gpio_usage.h>
//#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
//#include <mach/mt_sleep.h>
//#include <mach/mt_boot.h>
//#include <mach/system.h>
//#include <cust_charging.h>
#include <cust_bat_id.h>

//<2012/7/9-tedwu, check MOTO battery ID
#if defined(CHK_BATTERY_ID)
#define BATTERY_CHECK_MAX_NUM		5
static spinlock_t g_BatteryID_SpinLock;
static kal_bool Is_battery_valid = KAL_FALSE;
static int Battery_ID_read_count = 0;
static char rom_data[8];
static char memory_data[32*4];
static char Family_Code, Max_Voltage, page2_CheckSum, page3_CheckSum, page4_Str[32+1];
static int  Custom_ID;
static const char page4_Str_MOTO[] = "COPR2012MOTOROLA E.P CHARGE ONLY";
kal_uint8 battery_pack_id = BATTERY_PACK_ERROR;
//>2012/7/9-tedwu

void bat_id_init_spinLock(void)
{
	spin_lock_init(&g_BatteryID_SpinLock);
}

//<2012/12/26-tedwu, for power consumption and avoid ESD issue on 1-wire pin?.
void bat_id_init_pin(void)
{
#if 1  //For just high/low checking
    mt_set_gpio_mode(GPIO_BATTERY_DATA_PIN, GPIO_BATTERY_DATA_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_select(GPIO_BATTERY_DATA_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(GPIO_BATTERY_DATA_PIN, GPIO_PULL_ENABLE);
#else //For reading EEPROM data
    mt_set_gpio_mode(GPIO_BATTERY_DATA_PIN, GPIO_BATTERY_DATA_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_OUT);
	mt_set_gpio_pull_select(GPIO_BATTERY_DATA_PIN, GPIO_PULL_UP);
	mt_set_gpio_pull_enable(GPIO_BATTERY_DATA_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, GPIO_OUT_ONE);
    udelay(550);
#endif
}

void bat_id_deinit_pin(void)
{
	mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_BATTERY_DATA_PIN, GPIO_PULL_DISABLE);
}
//>2012/12/26-tedwu

void bat_id_init_variable(void)
{
	Battery_ID_read_count = 0;
	Is_battery_valid = KAL_FALSE;
}

//<2012/7/9-tedwu, check MOTO battery ID
static int OneWire_Reset_Bus(void)
{
	int presence_pulse;
	
	mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, GPIO_OUT_ZERO);
	udelay(550);
	mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, GPIO_OUT_ONE);
	udelay(60);
	mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_IN);
	udelay(40);
	presence_pulse = mt_get_gpio_in(GPIO_BATTERY_DATA_PIN);
	mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_OUT);
	udelay(400);
	
	//battery_xlog_printk(BAT_LOG_CRTI, "Presence Pulse: %d\n", presence_pulse);
	
	return presence_pulse;
}

static void OneWire_Write_Byte(char cmd)
{
	int idx;
/*
#if defined(CHK_BATTERY_ID__WITH_INTERNAL_PULL_UP_R)
	mt_set_gpio_pull_enable(GPIO_BATTERY_DATA_PIN, GPIO_PULL_DISABLE);
#endif	
*/
	//mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_OUT);
		
	for(idx=0; idx<8; idx++)
	{
		mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, 0);
		udelay(3);
		
		if((cmd >> idx) & 0x01)
		{
			mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, 1);
			udelay(60);
		}
		else
		{
			mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, 0);
			udelay(60);
		}			
		
		mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, 1);
		udelay(10);	// Trec
	}
}	

static char OneWire_Read_Byte(void)
{
	int idx = 0;
	char retbyte = 0;
	unsigned long flags;
	
	//mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, 1);  //to be deleted?
/*
#if defined(CHK_BATTERY_ID__WITH_INTERNAL_PULL_UP_R)
	mt_set_gpio_pull_enable(GPIO_BATTERY_DATA_PIN, GPIO_PULL_DISABLE);
#endif	
*/
	//mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_OUT);  //to be deleted?

	spin_lock_irqsave(&g_BatteryID_SpinLock, flags);
	
	for(idx=0; idx<8; idx++)
	{
		mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, 0);
		
		//This delay has to remove, because Tsu has to be less than 1us for DS2502
		//udelay(1 /*3*/);
/*		
#if defined(CHK_BATTERY_ID__WITH_INTERNAL_PULL_UP_R)
		mt_set_gpio_pull_select(GPIO_BATTERY_DATA_PIN, GPIO_PULL_UP);
		mt_set_gpio_pull_enable(GPIO_BATTERY_DATA_PIN, GPIO_PULL_ENABLE);
#endif
*/
		mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_IN);
		//udelay(15);
		udelay(18);
		
		if(mt_get_gpio_in(GPIO_BATTERY_DATA_PIN) & 0x01)
		{
			retbyte |= (0x01 << idx);
		}	
		
		udelay(60);
		
		mt_set_gpio_out(GPIO_BATTERY_DATA_PIN, 1);
/*
#if defined(CHK_BATTERY_ID__WITH_INTERNAL_PULL_UP_R)
		mt_set_gpio_pull_enable(GPIO_BATTERY_DATA_PIN, GPIO_PULL_DISABLE);
#endif		
*/
		mt_set_gpio_dir(GPIO_BATTERY_DATA_PIN, GPIO_DIR_OUT);

		udelay(10);	// Trec
	}

	spin_unlock_irqrestore(&g_BatteryID_SpinLock, flags);
	
	return retbyte;
}
	
void Read_Battery_Info(void)
{
	char read_rom = 0x33, skip_rom = 0xcc, read_memory = 0xF0;
	int idx = 0, page_idx;
		
  battery_xlog_printk(BAT_LOG_CRTI, "Read_Battery_Info Start ---\n");

//<2012/12/26-tedwu, for power consumption and avoid ESD issue on 1-wire pin?.
#if defined(ARIMA_LO1_HW)
	bat_id_init_pin();
#endif
//>2012/12/26-tedwu
	if(OneWire_Reset_Bus() == 0)
	{	
		// Read ROM
		OneWire_Write_Byte(read_rom);
		
		for(idx=0; idx<8; idx++)
		{	
			rom_data[idx] = OneWire_Read_Byte();
		}	

		for(idx=0; idx<(sizeof(rom_data)); idx++)
			battery_xlog_printk(BAT_LOG_CRTI, "0x%X, ", rom_data[idx]);
		
		battery_xlog_printk(BAT_LOG_CRTI, "\n=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n");
		
		// Read Memory		
		for(page_idx=0; page_idx<4; page_idx++)
		//for(page_idx=3; page_idx<4; page_idx++)
		{		
			OneWire_Reset_Bus();
			OneWire_Write_Byte(skip_rom);
			OneWire_Write_Byte(read_memory);
			OneWire_Write_Byte(0x00 + (0x20 * page_idx));
			OneWire_Write_Byte(0x00);						
			
			OneWire_Read_Byte();	// CRC
			
			for(idx=0; idx<32; idx++)
			{
				memory_data[idx + (0x20 * page_idx)] = OneWire_Read_Byte();
			}	
		}

		// parsing data		
		{
			Family_Code = rom_data[0];
			Custom_ID = (int)(((rom_data[6]<<8)|(rom_data[5]&0xF0))>>4);
			Max_Voltage = memory_data[2];
			page2_CheckSum = memory_data[0x20];
			page3_CheckSum = memory_data[0x40];
			memset(page4_Str, 0x00, sizeof(page4_Str));
			strncpy(page4_Str, &(memory_data[0x60]), 32);
			
			battery_xlog_printk(BAT_LOG_CRTI, "Family_Code = %x, Custom_ID = %x, Max_Voltage = %x, page2_CheckSum = %x, page3_CheckSum = %x\n", \
							Family_Code, Custom_ID, Max_Voltage, page2_CheckSum, page3_CheckSum);
			battery_xlog_printk(BAT_LOG_CRTI, "page4_Str = %s\n", page4_Str);					
		}		
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY:bq24158] presence pulse = high\n");
	}		
	
//<2012/12/26-tedwu, for power consumption and avoid ESD issue on 1-wire pin?.
#if defined(ARIMA_LO1_HW)
	bat_id_deinit_pin();
#endif
//>2012/12/26-tedwu
  battery_xlog_printk(BAT_LOG_CRTI, "Read_Battery_Info End ---\n");
}

extern kal_bool upmu_is_chr_det(void);

kal_bool Is_MOTO_Battery(void)
{
	kal_bool isValid = KAL_TRUE;
	//char Family_Code, page2_CheckSum, page3_CheckSum;
	//char page4_Str_MOTO[] = "COPR2011MOTOROLA E.P CHARGE ONLY";
	//char page4_Str[32+1];
	//int  Custom_ID;

	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY:bq24158] Is_MOTO_Battery, Battery_ID_read_count = %d\n", Battery_ID_read_count);		
	if(upmu_is_chr_det() == KAL_TRUE)
	{
		//Family_Code = rom_data[0];
		//Custom_ID = (int)(((rom_data[6]<<8)|(rom_data[5]&0xF0))>>4);
		//page2_CheckSum = memory_data[0x20];
		//page3_CheckSum = memory_data[0x40];
		//memset(page4_Str, 0x00, sizeof(page4_Str));
		//strncpy(page4_Str, &(memory_data[0x60]), 32);
		//strncpy(page4_Str, &(memory_data[0x68]), 24);
		
		//battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY:bq24158] Is_MOTO_Battery\n");
		//battery_xlog_printk(BAT_LOG_CRTI, "Family_Code = %x, Custom_ID = %x, page2_CheckSum = %x, page3_CheckSum = %x\n", \
						//Family_Code, Custom_ID, page2_CheckSum, page3_CheckSum);
		//battery_xlog_printk(BAT_LOG_CRTI, "page4_Str = %s\n", page4_Str);					
		
		if((Battery_ID_read_count <= BATTERY_CHECK_MAX_NUM) && (Is_battery_valid == KAL_FALSE))
		{
           #if defined(BATTERY_PACK_HW4X)
			if((Family_Code == 0x89) && (Max_Voltage == 0x9E) && 
				(((page2_CheckSum == 0xD3) && (page3_CheckSum == 0xDC)) /*|| 
				 ((page2_CheckSum == 0x66) && (page3_CheckSum == 0x83)) ||
				 ((page2_CheckSum == 0x75) && (page3_CheckSum == 0xde)) ||
				 ((page2_CheckSum == 0x75) && (page3_CheckSum == 0x90)) */)
				&& (Custom_ID == 0x500) && (strncmp(&(page4_Str_MOTO[8]), &(page4_Str[8]), 24) == 0))
					#elif defined(BATTERY_PACK_EG30)
			if((Family_Code == 0x89) && (Max_Voltage == 0x9E) && 
				(((page2_CheckSum == 0xd2) && (page3_CheckSum == 0x71)) ||
				 ((page2_CheckSum == 0xd4) && (page3_CheckSum == 0x5a)))
				&& (Custom_ID == 0x500) && (strncmp(&(page4_Str_MOTO[8]), &(page4_Str[8]), 24) == 0))
            #else
			if((strncmp(&(page4_Str_MOTO[0]), &(page4_Str[0]), 4) == 0)
					&& (strncmp(&(page4_Str_MOTO[8]), &(page4_Str[8]), 24) == 0))
					#endif
			{
				//isValid = KAL_TRUE;
				Is_battery_valid = KAL_TRUE;
			}		
			else
			{
				Read_Battery_Info();
				Battery_ID_read_count ++;
			}
		}
		else
		{
			isValid = Is_battery_valid;
		}

		battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY:bq24158] Is_MOTO_Battery %d, %d page4_Str = %s\n",Is_battery_valid, isValid, page4_Str);
	}	
		
	return isValid;			
}

kal_bool Is_Invalid_Battery(void)
{
	if((Is_battery_valid == KAL_FALSE) && (Battery_ID_read_count > BATTERY_CHECK_MAX_NUM))
			return KAL_FALSE;
	
	return KAL_TRUE;	
}	

int Get_MOTO_Battery_Vender(void)
{
	if((page2_CheckSum == 0xd4) && (page3_CheckSum == 0x5a))
		return 0x01;		
	else
		return 0x00;	
}
#endif //if defined(CHK_BATTERY_ID)
//>2012/7/9-tedwu
//>2014/04/01-tedwu
