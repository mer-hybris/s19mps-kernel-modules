/******************** (C) COPYRIGHT 2016 MEMSIC ********************
 *
 * File Name          : mxc400x.c
 * Description        : MXC400X accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, MEMSIC SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH MEMSIC PARTS.
 *

 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include 	<linux/module.h>
#include	<linux/input.h>
#include	<linux/input-polldev.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>
#include  	<linux/slab.h>

#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include  "mxc400x.h"
#include "prj/prj_config.h"


#define USE_HRTIMER

#ifdef USE_HRTIMER
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/sched/rt.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#define     POLL_INTERVAL_DEFAULT 20
#endif

#define MXC400X_DEBUG_ON          		0
#define MXC400X_DEBUG_FUNC_ON     		0
/* Log define */
#define MXC400X_INFO(fmt, arg...)      	pr_warn("<<-MXC400X_INFO->> "fmt"\n", ##arg)
#define MXC400X_ERR(fmt, arg...)        pr_err("<<-MXC400X_ERROR->> "fmt"\n", ##arg)
#define MXC400X_DEBUG(fmt, arg...)		do {\
						if (MXC400X_DEBUG_ON)\
							pr_warn("<<-MXC400X DEBUG->> [%d]"fmt"\n", __LINE__, ##arg);\
					} while (0)
#define MXC400X_DEBUG_FUNC()		do {\
						if (MXC400X_DEBUG_FUNC_ON)\
							pr_debug("<<-MXC400X FUNC->> Func:%s@Line:%d\n", __func__, __LINE__);\
					} while (0)

#define DEVICE_INFO         	"MXC, mxc400x"
#define DEVICE_INFO_LEN     	32
#define	MAX_INTERVAL		1200
#define MAX_AXIS_REMAP_TAB_SZ   8
#define I2C_RETRIES             5
#define I2C_RETRY_DELAY         5
#define DRIVER_VERSION     	"V80.83.00_r"

/* end RESUME STATE INDICES */

static int poll_interval = 20;
static int min_interval = 10;

//qzhu note:
//position is design this value to 0-7
static int sensor_dir = CONFIG_MXC400X_POSITION;

struct mxc400x_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 *      *      * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 *      *      * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 *      *      * src is 2, sign_* is (-)1 */
	int src_x;
	int src_y;
	int src_z;

	int sign_x;
	int sign_y;
	int sign_z;
};

static struct mxc400x_sensor_axis_remap
mxc400x_axis_remap_tab[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};


struct mxc400x_acc_data {
	struct i2c_client *client;
	struct mxc400x_acc_platform_data *pdata;

	struct mutex lock;
#ifdef USE_HRTIMER
	struct hrtimer work_timer;
	struct completion report_complete;
	struct task_struct *thread;
	bool hrtimer_running;
#else
	struct delayed_work input_work;
#endif

	struct input_dev *input_dev;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct mxc400x_acc_data *mxc400x_acc_misc_data;
struct i2c_client      *mxc400x_i2c_client;

static DECLARE_WAIT_QUEUE_HEAD(open_wq);


static int mxc400x_z = 1023;
static atomic_t	a_flag;


static int mxc400x_acc_i2c_read(struct mxc400x_acc_data *acc, u8 * buf, int len)
{
	int err , retry = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if(err != 2)
		{
			msleep_interruptible(I2C_RETRY_DELAY);
		}
	} while (err != 2 && retry++ < I2C_RETRIES);

	if (err != 2)
	{
		MXC400X_ERR("read transfer error\n");
		err = -EIO;
	} 
	else
	{
		err = 0;
	}

	
	return err;
}

static int mxc400x_acc_i2c_write(struct mxc400x_acc_data *acc, u8 * buf, int len)
{
	int err , retry = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1, .buf = buf,
		},
	};
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if(err != 1)
		{
			msleep_interruptible(I2C_RETRY_DELAY);
		}
	} while (err != 1 && retry++ < I2C_RETRIES);

	if (err != 1)
	{
		MXC400X_ERR("write transfer error\n");
		err = -EIO;
	} 
	else
	{
		err = 0;
	}

	return err;
}

static int mxc400x_check_device(struct mxc400x_acc_data *acc)
{
	u8 buf[7] = {0};
	int err ;
	MXC400X_DEBUG_FUNC();

	buf[0] = MXC400X_REG_ID;
	err = mxc400x_acc_i2c_read(acc, buf, 1);
	if (err < 0)
	{
	    MXC400X_ERR("write transfer error %s\n",__func__);
	    return -1;
	}

	if (((buf[0] & 0x3F) != MXC400X_ID_1) && ((buf[0] & 0x3F) != MXC400X_ID_2))
	{
		  MXC400X_ERR("mxc400x check device failed\n");
		  return -1;
	}
	MXC400X_INFO("%s: check device successfully\n", MXC400X_ACC_DEV_NAME);
	return 0;

}

static void mxc400x_acc_device_power_off(struct mxc400x_acc_data *acc)
{
	int err=-1,i = 0;
	u8 buf[2];

	MXC400X_DEBUG_FUNC();

	buf[0] = MXC400X_REG_CTRL;
	buf[1] = MXC400X_CTRL_PWRDN;
	while(i++ < 3)
	{
		err = mxc400x_acc_i2c_write(acc, buf, 1);
		if(!err)
		{
			break;
		}
	}
	if (err < 0)
	{
		MXC400X_ERR("soft power off failed: %d\n", err);
	}
}

static int mxc400x_acc_device_power_on(struct mxc400x_acc_data *acc)
{
	int err = -1,i = 0;
	u8 buf[2];

	MXC400X_DEBUG_FUNC();

	buf[0] = MXC400X_REG_CTRL;
	buf[1] = MXC400X_CTRL_PWRON;
	while(i++ < 3)
	{
		err = mxc400x_acc_i2c_write(acc, buf, 1);
		if(!err)
		{
			break;
		}
	}
	if (err < 0)
	{
		MXC400X_ERR("soft power on failed: %d\n", err);
	}
	msleep(300);
	return 0;
}

static int mxc400x_acc_register_read(struct mxc400x_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = mxc400x_acc_i2c_read(acc, buf, 1);
	return err;
}


static int mxc400x_remap_xyz(int *xyz, int layout)
{
	int temp[3] = {0};
	struct mxc400x_sensor_axis_remap *map_tab;

	map_tab = &mxc400x_axis_remap_tab[layout];

	temp[0] = xyz[0];
	temp[1] = xyz[1];
	temp[2] = xyz[2];
	xyz[0] = temp[map_tab->src_x] * map_tab->sign_x;
	xyz[1] = temp[map_tab->src_y] * map_tab->sign_y;
	xyz[2] = temp[map_tab->src_z] * map_tab->sign_z;

	return 0;
}

static int mxc400x_acc_get_acceleration_data(struct mxc400x_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware x, y,z */
	unsigned char acc_data[MXC400X_DATA_LENGTH] = {0, 0, 0, 0, 0, 0};
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	acc_data[0] = MXC400X_REG_X;
	err = mxc400x_acc_i2c_read(acc, acc_data, MXC400X_DATA_LENGTH);

	if (err < 0)
	{
		MXC400X_ERR("%s I2C read xy error %d\n", MXC400X_ACC_DEV_NAME, err);
		return err;
	}

	hw_d[0] = (signed short)(acc_data[0] << 8 | acc_data[1]) >> 4;
	hw_d[1] = (signed short)(acc_data[2] << 8 | acc_data[3]) >> 4;
	hw_d[2] = (signed short)(acc_data[4] << 8 | acc_data[5]) >> 4;

	mxc400x_remap_xyz(hw_d, sensor_dir);

	xyz[0] = hw_d[0] * 4;
	xyz[1] = hw_d[1] * 4;
	xyz[2] = hw_d[2] * 4;

	MXC400X_DEBUG("%s read x=%d, y=%d, z=%d\n",MXC400X_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	MXC400X_DEBUG("%s poll interval %d\n", MXC400X_ACC_DEV_NAME, poll_interval);

	return err;
}


static void mxc400x_acc_report_values(struct mxc400x_acc_data *acc, int *xyz)
{
	static int last_x = 0;
	static int last_y = 0;
	static int last_z = 0;

      if ((last_x == xyz[0]) && (last_y == xyz[1]) && last_z == xyz[2])
	  	xyz[1] = xyz[1] + 1;
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);

	last_x = xyz[0];
	last_y = xyz[1];
	last_z = xyz[2];
	input_sync(acc->input_dev);
}

static int mxc400x_acc_mod_delay(struct mxc400x_acc_data *acc, int delay)
{
	ktime_t poll_delay;
	if (atomic_read(&acc->enabled))
	{
		if (hrtimer_try_to_cancel(&acc->work_timer) < 0)
			return -1;
      
		if (poll_interval > 0)
		{
			poll_delay = ktime_set(0, poll_interval * NSEC_PER_MSEC);
		}
		else
		{
			poll_delay = ktime_set(0, POLL_INTERVAL_DEFAULT * NSEC_PER_MSEC);
		}
		hrtimer_start(&acc->work_timer, poll_delay, HRTIMER_MODE_REL);
	}
	//printk("ctsmemsic set delay =%d\n", delay);
	return 0;
}

static int mxc400x_acc_enable(struct mxc400x_acc_data *acc)
{
	int err;
#ifdef USE_HRTIMER
	ktime_t poll_delay;
#endif
	MXC400X_DEBUG_FUNC();
	if (!atomic_read(&acc->enabled))
	{
		err = mxc400x_acc_device_power_on(acc);

		if (err < 0)
		{
			atomic_set(&acc->enabled, 0);
			return err;
		}
		atomic_set(&acc->enabled, 1);
#ifdef USE_HRTIMER
		acc->hrtimer_running = true;
		
		if (hrtimer_try_to_cancel(&acc->work_timer) < 0)
			return -1;
      
		if (poll_interval > 0)
		{
			poll_delay = ktime_set(0, poll_interval * NSEC_PER_MSEC);
		}
		else
		{
			poll_delay = ktime_set(0, POLL_INTERVAL_DEFAULT * NSEC_PER_MSEC);
		}
		hrtimer_start(&acc->work_timer, poll_delay, HRTIMER_MODE_REL);
		//printk("ctsmemsic set enable\n");

#else
		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
					poll_interval));
#endif

	}

	return 0;
}

static int mxc400x_acc_disable(struct mxc400x_acc_data *acc)
{
	if (atomic_read(&acc->enabled))
	{
#ifdef USE_HRTIMER
		if (acc->hrtimer_running)
		{
			acc->hrtimer_running = false;
			hrtimer_cancel(&acc->work_timer);
		}
#else
		cancel_delayed_work_sync(&acc->input_work);
#endif
		mxc400x_acc_device_power_off(acc);
		atomic_set(&acc->enabled, 0);
	}

	return 0;
}

static int mxc400x_acc_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = mxc400x_acc_misc_data;

	return 0;
}

static int fac_xyz[3] = {0,0,0};

static long mxc400x_acc_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	
	int err;
	int interval;
    int xyz[3] = {0,0,0};
	u8  write_reg[2] = {0, 0};
	int reg;
	int parms;
	int flag;
	struct mxc400x_acc_data *acc = file->private_data;


	switch (cmd)
	{
		case MXC400X_ACC_IOCTL_GET_DELAY:
			interval = poll_interval;
			if (copy_to_user(argp, &interval, sizeof(interval)))
			{
				return -EFAULT;
			}
			break;

		case MXC400X_ACC_IOCTL_SET_DELAY:
			if (copy_from_user(&interval, argp, sizeof(interval)))
			{
				return -EFAULT;
			}
			if (interval < 0 || interval > 1000)
			{
				return -EINVAL;
			}
			if(interval > MAX_INTERVAL)
			{
				interval = MAX_INTERVAL;
			}
			poll_interval = max(interval,min_interval);
			mxc400x_acc_mod_delay(acc, poll_interval);
			break;

		case MXC400X_ACC_IOCTL_SET_ENABLE:
			if (copy_from_user(&interval, argp, sizeof(interval)))
			{
				return -EFAULT;
			}
			if (interval > 1)
			{
				return -EINVAL;
			}
			if (interval)
			{
				err = mxc400x_acc_enable(acc);
			}
			else
			{
				err = mxc400x_acc_disable(acc);
			}
			return err;
			break;

		case MXC400X_ACC_IOCTL_GET_ENABLE:
			interval = atomic_read(&acc->enabled);
			if (copy_to_user(argp, &interval, sizeof(interval)))
			{
				return -EINVAL;
			}
			break;
		case MXC400X_ACC_IOCTL_GET_COOR_XYZ:
			err = mxc400x_acc_get_acceleration_data(acc, xyz);
			if (err < 0)
			{
				return err;
			}

			if (copy_to_user(argp, xyz, sizeof(xyz)))
			{
				MXC400X_ERR(" %s %d error in copy_to_user \n",
						__func__, __LINE__);
				return -EINVAL;
			}
			break;
		case MXC400X_ACC_IOCTL_GET_TEMP:
			{
				u8  tempture = 0;

				err = mxc400x_acc_register_read(acc, &tempture,MXC400X_REG_TEMP);
				if (err < 0)
				{
					MXC400X_ERR("%s, error read register MXC400X_REG_TEMP\n", __func__);
					return err;
				}

				if (copy_to_user(argp, &tempture, sizeof(tempture)))
				{
					MXC400X_ERR(" %s %d error in copy_to_user \n",
							__func__, __LINE__);
					return -EINVAL;
				}
			}
			break;
		case MXC400X_ACC_IOCTL_GET_CHIP_ID:
			{
				u8 devid = 0;
				u8 devinfo[DEVICE_INFO_LEN] = {0};
				err = mxc400x_acc_register_read(acc, &devid, MXC400X_REG_ID);
				if (err < 0)
				{
					MXC400X_ERR("%s, error read register MXC400X_REG_ID\n", __func__);
					return -EAGAIN;
				}

				sprintf(devinfo, "%s, %#x", DEVICE_INFO, devid);

				if (copy_to_user(argp, &devinfo, sizeof(devinfo)))
				{
					MXC400X_ERR("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
					return -EINVAL;
				}
			}
			break;

		case MXC400X_ACC_IOCTL_GET_LAYOUT:
			{
				if (copy_to_user(argp, &sensor_dir,
							sizeof(sensor_dir)))
				{
					MXC400X_ERR("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
					return -EINVAL;
				}
			}
			break;
		case MXC400X_ACC_IOCTL_READ_REG:
			{
				u8 content = 0;
				if (copy_from_user(&reg, argp, sizeof(reg)))
				{
					return -EFAULT;
				}
				err = mxc400x_acc_register_read(acc, &content, reg);
				if (err < 0)
				{
					MXC400X_ERR("%s, error read register %x\n", __func__, reg);
					return -EAGAIN;
				}
				if (copy_to_user(argp, &content, sizeof(content)))
				{
					MXC400X_ERR("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
					return -EINVAL;
				}

			}
			break;
		case MXC400X_ACC_IOCTL_WRITE_REG:
			{
				if (copy_from_user(write_reg, argp, sizeof(write_reg)))
				{
					return -EFAULT;
				}
				err = mxc400x_acc_i2c_write(acc, write_reg, 1);
				if (err < 0)
				{
					MXC400X_ERR("%s, error write register %x\n", __func__, reg);
					return -EAGAIN;
				}
			}
			break;

		case MXC400X_ACC_IOC_SET_AFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag)))
			{
				return -EFAULT;
			}
			if (flag < 0 || flag > 1)
			{
				return -EINVAL;
			}
			atomic_set(&a_flag, flag);
			break;
		case MXC400X_ACC_IOC_GET_AFLAG:
			flag = atomic_read(&a_flag);
			if (copy_to_user(argp, &flag, sizeof(flag)))
			{
				return -EFAULT;
			}
			break;
		case MXC400X_ACC_IOC_SET_APARMS:
			if (copy_from_user(&parms, argp, sizeof(parms)))
			{
				return -EFAULT;
			}
			mxc400x_z = parms;
			break;
		default:
			MXC400X_ERR("%s no cmd error\n", __func__);
			return -EINVAL;
	}
	
	return 0;
}

static const struct file_operations mxc400x_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = mxc400x_acc_misc_open,
	.unlocked_ioctl = mxc400x_acc_misc_ioctl,
};

static struct miscdevice mxc400x_acc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MXC400X_ACC_DEV_NAME,
	.fops = &mxc400x_acc_misc_fops,
};

#ifdef USE_HRTIMER
static void mxc400x_acc_input_work_func(struct mxc400x_acc_data *acc)
{
	int xyz[4] = { 0 };
	int err;

	err = mxc400x_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
	{
		MXC400X_ERR("get_acceleration_data failed\n");
	}
	else
	{
		mxc400x_acc_report_values(acc, xyz);
		fac_xyz[0] = xyz[0];
		fac_xyz[1] = xyz[1];
		fac_xyz[2] = xyz[2];
	}
}

static enum hrtimer_restart mxc400x_acc_work(struct hrtimer *timer)
{
	struct mxc400x_acc_data *acc;
	ktime_t poll_delay;
	acc = container_of((struct hrtimer *)timer,
			struct mxc400x_acc_data, work_timer);

	complete(&acc->report_complete);
	if (poll_interval > 0)
	{
		poll_delay = ktime_set(0, poll_interval * NSEC_PER_MSEC);
	}
	else
	{
		poll_delay = ktime_set(0, POLL_INTERVAL_DEFAULT * NSEC_PER_MSEC);
	}
	acc->hrtimer_running = true;
	hrtimer_forward_now(&acc->work_timer, poll_delay);

	return HRTIMER_RESTART;
}

static int report_event(void *data)
{
	struct mxc400x_acc_data *acc = data;

	while(1)
	{
		/* wait for report event */
		wait_for_completion(&acc->report_complete);
		mutex_lock(&acc->lock);
		if (atomic_read(&acc->enabled) <= 0)
		{
			mutex_unlock(&acc->lock);
			continue;
		}
		mxc400x_acc_input_work_func(acc);
		mutex_unlock(&acc->lock);
	}
	return 0;
}
#else //use_hrtimer
static void mxc400x_acc_input_work_func(struct work_struct *work)
{
	struct mxc400x_acc_data *acc;

	int xyz[3] = { 0 };
	int err;
	static int proximity_state = 0;
	static int pre_proximity_state = 0;

	acc = container_of((struct delayed_work *)work,struct mxc400x_acc_data,	input_work);

	err = mxc400x_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
	{
		MXC400X_ERR("get_acceleration_data failed\n");
	}
	else
	{
		mxc400x_acc_report_values(acc, xyz);
		fac_xyz[0] = xyz[0];
		fac_xyz[1] = xyz[1];
		fac_xyz[2] = xyz[2];
//		printk("memsic x = %d y = %d z = %d\n", xyz[0], xyz[1], xyz[2]);
	}

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
				poll_interval));
}
#endif

static int mxc400x_acc_validate_pdata(struct mxc400x_acc_data *acc)
{
	poll_interval = max(poll_interval,min_interval);

	/* Enforce minimum polling interval */
	if (poll_interval < min_interval)
	{
		MXC400X_ERR("minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static ssize_t mxc_accel_get_layout(struct device *dev, struct device_attribute *attr, char *buf)
{
	//struct i2c_client *client = to_i2c_client(dev);

	return sprintf(buf, "%d\n", sensor_dir);
}

static ssize_t mxc_accel_set_layout(struct device *dev, struct device_attribute *attr, 	const char *buf, size_t count)
{

	if(1 == sscanf(buf, "%d", &sensor_dir))
	{
		MXC400X_INFO("set sensor_dir to %d\n", sensor_dir);
	}
	else
	{
		MXC400X_INFO("invalid format = '%s'\n", buf);
	}

	return count;
}


static DEVICE_ATTR(layout, S_IRUGO|S_IWUSR, mxc_accel_get_layout, mxc_accel_set_layout);

static struct attribute *mxc400x_accel_attributes[] = {
	&dev_attr_layout.attr,
	NULL
};

static struct attribute_group mxc400x_accel_attribute_group = {
	.attrs = mxc400x_accel_attributes
};


static int mxc400x_acc_input_init(struct mxc400x_acc_data *acc)
{
	int err;
#ifdef USE_HRTIMER
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };
#endif
#ifdef USE_HRTIMER
		hrtimer_init(&acc->work_timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
		acc->work_timer.function = mxc400x_acc_work;
		acc->hrtimer_running = false;
		init_completion(&acc->report_complete);
		acc->thread = kthread_run(report_event, acc, "sensor_report_event");
		if (IS_ERR(acc->thread))
		{
			MXC400X_ERR("unable to create report_event thread\n");
			return -EINVAL;
		}
		sched_setscheduler_nocheck(acc->thread, SCHED_FIFO, &param);
#else
	INIT_DELAYED_WORK(&acc->input_work, mxc400x_acc_input_work_func);
#endif

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev)
	{
		err = -ENOMEM;
		MXC400X_ERR("input device allocate failed\n");
		goto err0;
	}

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -32768, 32768, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Y, -32768, 32768, 0, 0);
	input_set_abs_params(acc->input_dev, ABS_Z, -32768, 32768, 0, 0);

	acc->input_dev->name = "accelerometer";

	err = input_register_device(acc->input_dev);
	if (err)
	{
		MXC400X_ERR("unable to register input polled device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static struct kobject *acc_type_ctrl_kobj = NULL;

static ssize_t get_type(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", "mxc400x");
}

static DEVICE_ATTR(type, 0664, get_type, NULL);

static struct attribute *type_attrs[] = {
  &dev_attr_type.attr,
  NULL,
};

static struct attribute_group acc_type_attr_group = {
  .attrs = type_attrs,
};

static int acc_type_init(void)
{
  acc_type_ctrl_kobj = kobject_create_and_add("accelerometer", NULL);
  if (!acc_type_ctrl_kobj){
      printk("Create acc_type_ctrl_kobj failed!\n");
      return -ENOMEM;
  }
  return sysfs_create_group(acc_type_ctrl_kobj, &acc_type_attr_group);
}

static int mxc400x_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct mxc400x_acc_data *acc;
	int err = -1;

	MXC400X_DEBUG_FUNC();
	MXC400X_INFO("Driver version=%s\n",DRIVER_VERSION);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		MXC400X_ERR("client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	acc = kzalloc(sizeof(struct mxc400x_acc_data), GFP_KERNEL);
	if (acc == NULL)
	{
		err = -ENOMEM;
		MXC400X_ERR("failed to allocate memory for module data:""%d\n", err);
		goto exit_alloc_data_failed;
	}

	mutex_init(&acc->lock);

	acc->client = client;
	mxc400x_i2c_client = client;
	i2c_set_clientdata(client, acc);

	err = mxc400x_check_device(acc);
	if (err < 0)
	{
		  MXC400X_ERR("check deviced failed\n");
		  goto exit_kfree_acc;
	}
	
	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL)
	{
		err = -ENOMEM;
		MXC400X_ERR("failed to allocate memory for pdata: %d\n",err);
		goto exit_kfree_acc;
	}

	err = mxc400x_acc_validate_pdata(acc);
	if (err < 0)
	{
		MXC400X_ERR("failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	i2c_set_clientdata(client, acc);

	err = mxc400x_acc_device_power_on(acc);
	if (err < 0)
	{
		MXC400X_ERR("power on failed: %d\n", err);
		goto exit_kfree_pdata;
	}

	atomic_set(&acc->enabled, 1);

	err = mxc400x_acc_input_init(acc);
	if (err < 0)
	{
		MXC400X_ERR("input init failed\n");
		goto exit_kfree_pdata;
	}

	mxc400x_acc_misc_data = acc;

	err = misc_register(&mxc400x_acc_misc_device);
	if (err < 0)
	{
		MXC400X_ERR("misc MXC400X_ACC_DEV_NAME register failed\n");
		goto exit_kfree_pdata;
	}

	mxc400x_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	acc->on_before_suspend = 0;

	err = sysfs_create_group(&client->dev.kobj, &mxc400x_accel_attribute_group);

  	acc_type_init();
	MXC400X_INFO("%s: probed\n", MXC400X_ACC_DEV_NAME);

	return 0;


exit_kfree_pdata:
	kfree(acc->pdata);
	i2c_set_clientdata(client, NULL);
	mxc400x_acc_misc_data = NULL;
exit_kfree_acc:
  kfree(acc);		
exit_alloc_data_failed:
exit_check_functionality_failed:
	MXC400X_ERR("%s: Driver Init failed\n", MXC400X_ACC_DEV_NAME);
	return err;
}

static int  mxc400x_acc_remove(struct i2c_client *client)
{
	struct mxc400x_acc_data *acc = i2c_get_clientdata(client);
#ifdef USE_HRTIMER
	hrtimer_cancel(&acc->work_timer);
	kthread_stop(acc->thread);
#endif
	misc_deregister(&mxc400x_acc_misc_device);
	sysfs_remove_group(&client->dev.kobj, &mxc400x_accel_attribute_group);
	sysfs_remove_group(acc_type_ctrl_kobj, &acc_type_attr_group);
	
	mxc400x_acc_device_power_off(acc);
	if (acc->pdata->exit)
	{
		acc->pdata->exit();
	}
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

static int mxc400x_acc_resume(struct device *dev)
{
	struct mxc400x_acc_data *acc = i2c_get_clientdata(mxc400x_i2c_client);
	MXC400X_DEBUG_FUNC();
	if (acc != NULL && acc->on_before_suspend)
	{
		acc->on_before_suspend = 0;
		return mxc400x_acc_enable(acc);
	}

	return 0;
}

static int mxc400x_acc_suspend(struct device *dev)
{
	struct mxc400x_acc_data *acc = i2c_get_clientdata(mxc400x_i2c_client);

	MXC400X_DEBUG_FUNC();

	if (acc != NULL)
	{
		if (atomic_read(&acc->enabled))
		{
			acc->on_before_suspend = 1;
			return mxc400x_acc_disable(acc);
		}
	}
	return 0;
}

static const struct dev_pm_ops mxc400x_pm_ops = {
	.suspend = mxc400x_acc_suspend,
	.resume = mxc400x_acc_resume,
};

static struct of_device_id mxc400x_of_match_table[] = {
  { .compatible = "memsicp,mxc400x_acc", },
	{ },
};

MODULE_DEVICE_TABLE(of, mxc400x_of_match_table);

static struct i2c_driver mxc400x_acc_driver = {
	.driver = {
		.name = MXC400X_ACC_I2C_NAME,
		.pm = &mxc400x_pm_ops,
		.of_match_table = mxc400x_of_match_table,
	},
	.probe = mxc400x_acc_probe,
	.remove = mxc400x_acc_remove,
};


static int __init mxc400x_acc_init(void)
{
	printk( "%s called\n", __func__);
  return i2c_add_driver(&mxc400x_acc_driver);
}

static void __exit mxc400x_acc_exit(void)
{
	i2c_del_driver(&mxc400x_acc_driver);
}

late_initcall(mxc400x_acc_init);
module_exit(mxc400x_acc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC400X I2C driver");
MODULE_AUTHOR("hcpeng@memsic.cn");
