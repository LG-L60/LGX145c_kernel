/* 
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
/*
 * Definitions for CM36671 als/ps sensor chip.
 */
#ifndef __CM36671_H__
#define __CM36671_H__

#include <linux/ioctl.h>

/*cm36671 als/ps sensor register related macro*/
#define CM36671_REG_ALS_CONF 		0X00
#define CM36671_REG_ALS_THDH 		0X01
#define CM36671_REG_ALS_THDL 		0X02
#define CM36671_REG_PS_CONF1_2		0X03
#define CM36671_REG_PS_CONF3_MS		0X04
#define CM36671_REG_PS_CANC			0X05
#define CM36671_REG_PS_THD			0X06
#define CM36671_REG_PS_DATA			0X08
#define CM36671_REG_ALS_DATA		0X09
#define CM36671_REG_INT_FLAG		0X0B
#define CM36671_REG_ID_MODE			0X0C

/*CM36671 related driver tag macro*/
#define CM36671_SUCCESS				 		 0
#define CM36671_ERR_I2C						-1
#define CM36671_ERR_STATUS					-3
#define CM36671_ERR_SETUP_FAILURE			-4
#define CM36671_ERR_GETGSENSORDATA			-5
#define CM36671_ERR_IDENTIFICATION			-6


#endif

