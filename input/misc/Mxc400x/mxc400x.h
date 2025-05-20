/*
 * Copyright (C) 2016 MEMSIC, Inc.
 *
 * Initial Code:
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
 * Definitions for pointer accelorometer sensor chip.
 */
#ifndef __MXC400X_H__
#define __MXC400X_H__

//#include 	<linux/i2c/gsensor_multi_drv.h>

#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>

#define	MXC400X_ACC_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define	MXC400X_ACC_IOCTL_SET_DELAY		     _IOW(MXC400X_ACC_IOCTL_BASE, 0x00, int)
#define	MXC400X_ACC_IOCTL_GET_DELAY		     _IOR(MXC400X_ACC_IOCTL_BASE, 0x01, int)
#define	MXC400X_ACC_IOCTL_SET_ENABLE		 _IOW(MXC400X_ACC_IOCTL_BASE, 0x02, int)
#define	MXC400X_ACC_IOCTL_GET_ENABLE		 _IOR(MXC400X_ACC_IOCTL_BASE, 0x03, int)
#define	MXC400X_ACC_IOCTL_GET_TEMP		     _IOR(MXC400X_ACC_IOCTL_BASE, 0x04, int)
#define	MXC400X_ACC_IOCTL_GET_COOR_XYZ       _IOW(MXC400X_ACC_IOCTL_BASE, 0x22, int[3])
#define	MXC400X_ACC_IOCTL_GET_CHIP_ID        _IOR(MXC400X_ACC_IOCTL_BASE, 0x25, char[32])
#define	MXC400X_ACC_IOCTL_GET_LAYOUT        _IOR(MXC400X_ACC_IOCTL_BASE, 0x26, int)
#define MXC400X_ACC_IOC_SET_AFLAG			_IOW(MXC400X_ACC_IOCTL_BASE, 0x10, short)
#define MXC400X_ACC_IOC_GET_AFLAG			_IOR(MXC400X_ACC_IOCTL_BASE, 0x11, short)
#define MXC400X_ACC_IOC_SET_MFLAG			_IOW(MXC400X_ACC_IOCTL_BASE, 0x12, short)
#define MXC400X_ACC_IOC_GET_MFLAG			_IOR(MXC400X_ACC_IOCTL_BASE, 0x13, short)
#define MXC400X_ACC_IOC_SET_OFLAG			_IOW(MXC400X_ACC_IOCTL_BASE, 0x14, short)
#define MXC400X_ACC_IOC_GET_OFLAG			_IOR(MXC400X_ACC_IOCTL_BASE, 0x15, short)
#define MXC400X_ACC_IOCTL_READ_REG          _IOR(MXC400X_ACC_IOCTL_BASE, 0x19, int)
#define MXC400X_ACC_IOCTL_WRITE_REG          _IOW(MXC400X_ACC_IOCTL_BASE, 0x1A, unsigned char[2])
#define MXC400X_ACC_IOC_SET_APARMS			_IOW(MXC400X_ACC_IOCTL_BASE, 0x20, int)

#define MXC400X_ACC_IOC_SET_YPR				_IOW(MXC400X_ACC_IOCTL_BASE, 0x30, int[3])

#define MXC400X_ACC_IOC_GET_FAC_XYZ			_IOR(MXC400X_ACC_IOCTL_BASE, 0x35, int[3])

/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/
#define MXC400X_ACC_DEV_NAME		"mxc400x"
#define MXC400X_ACC_INPUT_NAME		"accelerometer" 
#define MXC400X_ACC_I2C_ADDR     	0x15		//mxc4005
#define MXC400X_ACC_I2C_NAME        MXC400X_ACC_DEV_NAME
/*CHIP ID is 0x01*/
#define MXC400X_ID_1        0x02
#define MXC400X_ID_2        0x03
/*	chip id register	*/
#define MXC400X_REG_ID      0x0E

/* MXC400X register address */
#define MXC400X_REG_FAC         0x0E
#define MXC400X_REG_FSRE        0x19
#define MXC400X_REG_CTRL		0x0D
#define MXC400X_REG_TEMP        0x09

#define MXC400X_PASSWORD        0x93
#define MXC400X_RANGE_8G        0x5B    /*full scale range +/-8g*/
#define MXC400X_CTRL_PWRON		0x40	/* power on */
#define MXC400X_CTRL_PWRDN		0x01	/* power donw */



// x 0304 y  0506 z 0708
#define MXC400X_REG_X		0x03
#define MXC400X_REG_Y		0x05
#define MXC400X_REG_Z		0x07

#define MXC400X_DATA_LENGTH       6

#define I2C_BUS_NUM_STATIC_ALLOC

struct mxc400x_acc_platform_data {
	int poll_interval;
	int min_interval;
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
};


#endif /* __MXC400X_H__ */

