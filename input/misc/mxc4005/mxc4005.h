/*
 * Copyright (C) 2014 MEMSIC, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for pointer accelerometer sensor chip.
 */
#ifndef __MXC400X_H__
#define __MXC400X_H__


#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>

#define	MXC400X_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define	MXC400X_IOCTL_SET_DELAY		     _IOW(MXC400X_IOCTL_BASE, 0x00, int)
#define	MXC400X_IOCTL_GET_DELAY		     _IOR(MXC400X_IOCTL_BASE, 0x01, int)
#define	MXC400X_IOCTL_SET_ENABLE		 _IOW(MXC400X_IOCTL_BASE, 0x02, int)
#define	MXC400X_IOCTL_GET_ENABLE		 _IOR(MXC400X_IOCTL_BASE, 0x03, int)
#define	MXC400X_IOCTL_GET_TEMP		     _IOR(MXC400X_IOCTL_BASE, 0x04, int)
#define	MXC400X_IOCTL_GET_LAYOUT       _IOR(MXC400X_IOCTL_BASE, 0x26, int)
#define MXC400X_IOCTL_READ_REG		_IOR(MXC400X_IOCTL_BASE, 0x19, int)
#define MXC400X_IOCTL_WRITE_REG		_IOW(MXC400X_IOCTL_BASE, 0x1A, unsigned char[2])
#define	GSENSOR_IOCTL_GET_XYZ       _IOR(MXC400X_IOCTL_BASE, 0x22, int[3])
#define	MXC400X_IOCTL_GET_CHIP_ID        _IOR(MXC400X_IOCTL_BASE, 0x25, char[32])
//#define POINTER_ACC_IOCTL_SET_APARMS	_IOW(MXC400X_IOCTL_BASE, 0x20, int)
#define POINTER_ACC_IOCTL_SET_APARMS	_IOW(MXC400X_IOCTL_BASE, 0x46, int[3])

/*
#define 	MXC400X_IOC_READ_REG				_IOWR(MXC400X_IOCTL_BASE, 0x15, unsigned char)
#define 	MXC400X_IOC_WRITE_REG			_IOW(MXC400X_IOCTL_BASE, 0x16, unsigned char[2])
#define 	MXC400X_IOC_READ_REGS			_IOWR(MXC400X_IOCTL_BASE, 0x17, unsigned char[10])
#define 	MXC400X_IOC_WRITE_REGS			_IOW(MXC400X_IOCTL_BASE, 0x18, unsigned char[10])
*/

/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/
#define MXC400X_DEV_NAME		"mxc400x"
#define MXC400X_INPUT_NAME		"accelerometer" 
#define MXC400X_I2C_ADDR     	0x15	//for MXC4005XC
#define MXC400X_I2C_NAME        MXC400X_DEV_NAME
/*CHIP ID is 0x02*/
#define WHOAMI_MXC400X	0x02	
//////chip id check 20190129 mayun
#define MXC400X_ID_1			0x02
#define MXC400X_ID_2			0x03
//#define WHOAMI_MXC622X	0x05
/*	chip id register	*/
#define WHO_AM_I		0x0E
//#define WHO_AM_I_MXC622X 0x08
/* MXC400X register address */
// x 0304 y  0506 z 0708
#define MXC400X_REG_X		0x03
#define MXC400X_REG_Y		0x05
#define MXC400X_REG_Z		0x07
#define MXC400X_REG_TEMP       0x09
#define MXC400X_REG_CTRL		0x0D

#define MXC400X_CTRL_PWRON		0x00	/* power on */
#define MXC400X_CTRL_PWRDN		0x01	/* power donw */

#define MXC400X_RANGE_2G   0x00
#define MXC400X_RANGE_4G   0x20
#define MXC400X_RANGE_8G   0x40



#define MXC400X_DATA_LENGTH       6

/* I2C static allocate option */
//#define I2C_BUS_NUM_STATIC_ALLOC
//#define I2C_STATIC_BUS_NUM        (2)	// Need to be modified according to actual setting

//add for acc cali
#define CALIBRATION_DATA_LENGTH 30 
#define CALIB_PATH_MAX_LENG 100
#define CALIBRATION_NODE_ACC    "/mnt/vendor/productinfo/sensor_calibration_data/acc"
 
#ifdef	__KERNEL__
struct mxc400x_platform_data {
	int poll_interval;
	int min_interval;
	unsigned axis_map_x;
	unsigned axis_map_y;
	unsigned axis_map_z;

	unsigned negate_x;
	unsigned negate_y;
	unsigned negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif	/* __KERNEL__ */
//add for read nv
struct acc_factory_cali_data{
	uint32_t x;
	uint32_t y;
	uint32_t z;
};
#endif /* __MXC400X_H__ */

