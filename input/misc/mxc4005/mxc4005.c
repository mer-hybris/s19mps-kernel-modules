/******************** (C) COPYRIGHT 2014 MEMSIC INC ********************
 *
 * File Name          : mxc400x_acc.c
 * Description        : Memsic MXC400X accelerometer sensor API
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
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *

 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include  	<linux/module.h>
#include	<linux/input.h>
#include	<linux/input-polldev.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>
#include  	<linux/slab.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include 	<linux/of_device.h>
#include 	<linux/of_address.h>
#include 	<linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include        <linux/earlysuspend.h>
#endif
//#include 	<transsion/ps_order.h>
typedef ssize_t (*TRANSSION_SENSOR_R)(void *, char *);
typedef ssize_t (*TRANSSION_SENSOR_W)(void *, const char *, size_t);
extern int transsion_sensor_add(char *name, TRANSSION_SENSOR_R read, TRANSSION_SENSOR_W write, void *data);
extern int transsion_sensor_report(void *name, signed short x, signed short y, signed short z);
#include "mxc4005.h"

#ifdef CONFIG_TRANSSION_HWINFO
//#include <transsion/hwinfo.h>
#endif

#define	I2C_RETRY_DELAY		10
#define	I2C_RETRIES		10

#define DEVICE_INFO         "Memsic, MXC400X"
#define DEVICE_INFO_LEN     32

//zhl add sumdge
struct acc_factory_cali_data factory_acc_cali;

/* end RESUME STATE INDICES */
extern int transsion_sensor_report(void *name, signed short x, signed short y, signed short z);
//#define DEBUG    0
//#define MXC400X_DEBUG  1


#define	MAX_INTERVAL	50
static int user_enable = 0;

struct mxc400x_data {
        atomic_t enabled;                /* attribute value */
        atomic_t delay;                 /* attribute value */
	struct i2c_client *client;
	struct mxc400x_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;

#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif
};


/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct mxc400x_data *mxc400x_misc_data;
struct i2c_client      *mxc400x_i2c_client;

//static int pointer_z = 0;

static int mxc400x_i2c_read(struct mxc400x_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf, },
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf, },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	
	return err;
}

static int mxc400x_i2c_write(struct mxc400x_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = { { .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1, .buf = buf, }, };
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int mxc400x_hw_init(struct mxc400x_data *acc)
{
	int err = -1;
	u8 buf[7] = {0};

	printk("jiapeng mxc400x_hw_init start \n");

	printk(KERN_INFO "%s: jiapeng hw init start\n", MXC400X_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = mxc400x_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;
/////////////////////chip id check 20190129 mayun
	if(((buf[0] & 0x3F)!= MXC400X_ID_1) && ((buf[0] & 0x3F) != MXC400X_ID_2)){
	//if ((buf[0] & 0x3F) != WHOAMI_MXC400X) {
		err = -1; /* choose the right coded error */
		goto error_unknown_device;
	}

	acc->hw_initialized = 1;
	printk(KERN_INFO "%s: hw init done\n", MXC400X_DEV_NAME);
	return 0;

error_firstread:
	acc->hw_working = 0;
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_MXC400X, buf[0]);
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void mxc400x_device_power_off(struct mxc400x_data *acc)
{
	int err;
	u8 buf[2] = { MXC400X_REG_CTRL, MXC400X_CTRL_PWRDN };

	err = mxc400x_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);
}
int read_acc_nv_data(void)
{
	int err = 0;
	struct file *pfile = NULL;
	char file_path[CALIB_PATH_MAX_LENG];
	char raw_cali_data[CALIBRATION_DATA_LENGTH] =  {0};
	int cal_file_size = CALIBRATION_DATA_LENGTH;
	int j = 0;

	snprintf(file_path, sizeof(file_path), "%s", CALIBRATION_NODE_ACC);
        printk("zhl file_path=%s\n",file_path);

	pfile = filp_open(file_path, O_RDONLY, 0664);
        if (IS_ERR(pfile)) {
               err = (int)PTR_ERR(pfile);
               printk("zhl open file %s ret=%d\n", file_path, err);
               return err;
        }
	/*
	cal_file_size = get_file_size(pfile);
	if (cal_file_size != CALIBRATION_DATA_LENGTH) {
        LOG_INFO("zhl Unable to get file size:%s\n", file_path);
        filp_close(pfile, NULL);
        return err;
        }
	LOG_INFO( "zhl cal_file_size=%d\n", cal_file_size);
	*/
	if (kernel_read(pfile, raw_cali_data,cal_file_size, &pfile->f_pos) != cal_file_size) {
               printk("Error: file read failed\n");
               filp_close(pfile, NULL);
               return err;
        } else {
               filp_close(pfile, NULL);
        }
	for (j = 0; j < cal_file_size; j++)
               printk("raw_cali_data[%d]=%d\n", j, raw_cali_data[j]);
	memcpy(&factory_acc_cali, raw_cali_data, sizeof(factory_acc_cali));
	printk("zhl add acc x=%d",factory_acc_cali.x);
	printk("zhl add acc y=%d",factory_acc_cali.y);
	printk("zhl add acc z=%d",factory_acc_cali.z);
	return 0;
}
static int mxc400x_device_power_on(struct mxc400x_data *acc)
{
    
	int err = -1;
	u8 buf[2] = {MXC400X_REG_CTRL, (MXC400X_RANGE_8G | MXC400X_CTRL_PWRON)};
	//+/-8G
	printk("jiapeng mxc400x_device_power_on start \n");

	err = mxc400x_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft change to +/8g failed: %d\n", err);


	return 0;
}

static int mxc400x_register_read(struct mxc400x_data *acc, u8 *buf,
		u8 reg_address)
{
	int err = -1;
	buf[0] = (reg_address);
	err = mxc400x_i2c_read(acc, buf, 1);
	return err;
}

/* */

static int mxc400x_get_acceleration_data_fact(struct mxc400x_data *acc,
		int *xyz)
{
	//mutex_lock(&acc->lock);
	int err = -1;




	/* Data bytes from hardware x, y,z */
	unsigned char acc_data[MXC400X_DATA_LENGTH] = {0, 0, 0, 0, 0, 0};
	printk("%s entre success\n", __func__);
   
	acc_data[0] = MXC400X_REG_X;
	err = mxc400x_i2c_read(acc, acc_data, MXC400X_DATA_LENGTH);
	#ifdef MXC400X_DEBUG
	printk("acc[0] = %x, acc[1] = %x, acc[2] = %x, acc[3] = %x, acc[4] = %x, acc[5] = %x\n", acc_data[0], acc_data[1], acc_data[2], 																								acc_data[3], acc_data[4], acc_data[5]);
  	#endif					
	if (err < 0)
    {
        #ifdef MXC400X_DEBUG
        printk(KERN_INFO "%s I2C read xy error %d\n", MXC400X_DEV_NAME, err);
        #endif
		return err;
    }
#if 0 // for debug
	printk("poll_interval = %d\n",acc->pdata->poll_interval);
	printk("min_interval = %d\n",acc->pdata->min_interval);
	printk("axis_map_x = %d\n",acc->pdata->axis_map_x);
	printk("axis_map_y = %d\n",acc->pdata->axis_map_y);
	printk("axis_map_z = %d\n",acc->pdata->axis_map_z);
	printk("negate_x = %d\n",acc->pdata->negate_x);
	printk("negate_y = %d\n",acc->pdata->negate_y);
	printk("negate_z = %d\n",acc->pdata->negate_z);
#endif
#if 1
	if(acc->pdata->negate_x)
		xyz[acc->pdata->axis_map_x] = -((signed short)(acc_data[0] << 8 | acc_data[1]) >> 4);
	else
		xyz[acc->pdata->axis_map_x] = (signed short)(acc_data[0] << 8 | acc_data[1]) >> 4;
	if(acc->pdata->negate_y)
		xyz[acc->pdata->axis_map_y] = -((signed short)(acc_data[2] << 8 | acc_data[3]) >> 4);
	else
		xyz[acc->pdata->axis_map_y] = (signed short)(acc_data[2] << 8 | acc_data[3]) >> 4;
	if(acc->pdata->negate_z)	
		xyz[acc->pdata->axis_map_z] = -((signed short)(acc_data[4] << 8 | acc_data[5]) >> 4);
	else
		xyz[acc->pdata->axis_map_z] = (signed short)(acc_data[4] << 8 | acc_data[5]) >> 4;
#else
   	xyz[1] = (signed short)(acc_data[0] << 8 | acc_data[1]) >> 4;
	xyz[0] = (signed short)(acc_data[2] << 8 | acc_data[3]) >> 4;
	xyz[2] = (signed short)(acc_data[4] << 8 | acc_data[5]) >> 4;
#endif
	

	#ifdef MXC400X_DEBUG
	printk(KERN_INFO "%s read x=%d, y=%d, z=%d \n",MXC400X_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	#endif
        
	
	//mutex_unlock(&acc->lock);
	return err;
	
}

/* */

static int mxc400x_get_acceleration_data(struct mxc400x_data *acc,
		int *xyz)
{
	//mutex_lock(&acc->lock);
	int err = -1;
    unsigned char acc_data[MXC400X_DATA_LENGTH] = {0, 0, 0, 0, 0, 0};
/*
// ++read the tempture

	       u8  tempture = 0; 
		
		err = mxc400x_register_read(acc, &tempture,MXC400X_REG_TEMP);                
		if (err < 0)  
		{
			printk("%s, error read register MXC400X_REG_TEMP\n", __func__);
			
			return err;
		} 

            xyz[3]=tempture;
          
//--end read the tempture
*/
	acc_data[0] = MXC400X_REG_X;
	err = mxc400x_i2c_read(acc, acc_data, MXC400X_DATA_LENGTH);
	#ifdef MXC400X_DEBUG
	printk("acc[0] = %x, acc[1] = %x, acc[2] = %x, acc[3] = %x, acc[4] = %x, acc[5] = %x\n", acc_data[0], acc_data[1], acc_data[2], 																								acc_data[3], acc_data[4], acc_data[5]);
  	#endif					
	if (err < 0)
    {
        #ifdef MXC400X_DEBUG
        printk(KERN_INFO "%s I2C read xy error %d\n", MXC400X_DEV_NAME, err);
        #endif
		return err;
    }
#if 0 // for debug
	printk("poll_interval = %d\n",acc->pdata->poll_interval);
	printk("min_interval = %d\n",acc->pdata->min_interval);
	printk("axis_map_x = %d\n",acc->pdata->axis_map_x);
	printk("axis_map_y = %d\n",acc->pdata->axis_map_y);
	printk("axis_map_z = %d\n",acc->pdata->axis_map_z);
	printk("negate_x = %d\n",acc->pdata->negate_x);
	printk("negate_y = %d\n",acc->pdata->negate_y);
	printk("negate_z = %d\n",acc->pdata->negate_z);
#endif
#if 1
	if(acc->pdata->negate_x)
		xyz[acc->pdata->axis_map_x] = -((signed short)(acc_data[0] << 8 | acc_data[1]) >> 2);
	else
		xyz[acc->pdata->axis_map_x] = (signed short)(acc_data[0] << 8 | acc_data[1]) >> 2;
	if(acc->pdata->negate_y)
		xyz[acc->pdata->axis_map_y] = -((signed short)(acc_data[2] << 8 | acc_data[3]) >> 2);
	else
		xyz[acc->pdata->axis_map_y] = (signed short)(acc_data[2] << 8 | acc_data[3]) >> 2;
	if(acc->pdata->negate_z)	
		xyz[acc->pdata->axis_map_z] = -((signed short)(acc_data[4] << 8 | acc_data[5]) >> 2);
	else
		xyz[acc->pdata->axis_map_z] = (signed short)(acc_data[4] << 8 | acc_data[5]) >> 2;
#else
   	xyz[0] = (signed short)(acc_data[0] << 8 | acc_data[1]) >> 4;
	xyz[1] = (signed short)(acc_data[2] << 8 | acc_data[3]) >> 4;
	xyz[2] = (signed short)(acc_data[4] << 8 | acc_data[5]) >> 4;
#endif
	#ifdef MXC400X_DEBUG
	printk(KERN_INFO "%s read x=%d, y=%d, z=%d tempture=%d\n",MXC400X_DEV_NAME, xyz[0], xyz[1], xyz[2],xyz[3]);
	#endif
        
	
	//mutex_unlock(&acc->lock);
	return err;
	
}


static int mxc400x_enable(struct mxc400x_data *acc)
{
	int err;
	printk("mxc400x_enable 1\n");
	//zhl add for read nv
	read_acc_nv_data();
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = mxc400x_device_power_on(acc);
		
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}

		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
				max(acc->pdata->poll_interval,300)));
	}

	return 0;
}

static int mxc400x_disable(struct mxc400x_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		mxc400x_device_power_off(acc);
	}
	printk("mxc400x_enable 0\n");
	return 0;
}

static int mxc400x_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = mxc400x_misc_data;

	return 0;
}

static long mxc400x_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	
	int err;
	int interval;
   	int xyz[3] = {0,0,0};
	int parms[3]={0};
	struct mxc400x_data *acc = file->private_data;
	unsigned char reg_addr;
	unsigned char reg_value;
	unsigned char data[16] = {0};
	int mxc400x_dir;

	//printk(KERN_INFO "%s: %s call with cmd 0x%x and arg 0x%x\n",
	//		MXC400X_DEV_NAME, __func__, cmd, (unsigned int)arg);
	mutex_lock(&acc->lock);
	switch (cmd) {
	case MXC400X_IOCTL_GET_DELAY:
		interval = acc->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		break;

	case MXC400X_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		if (interval < 0 || interval > 1000){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
		if(interval > MAX_INTERVAL)
			interval = MAX_INTERVAL;
		acc->pdata->poll_interval = max(interval,
				acc->pdata->min_interval);
		break;

	case MXC400X_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		if (interval > 1){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
		if (interval)
			err = mxc400x_enable(acc);
		else
			err = mxc400x_disable(acc);
		
		mutex_unlock(&acc->lock);
		return err;
		break;

	case MXC400X_IOCTL_GET_ENABLE:
		interval = atomic_read(&acc->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval))){
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}	
		break;
	case GSENSOR_IOCTL_GET_XYZ:    

		err = mxc400x_get_acceleration_data_fact(acc, xyz);                
		if (err < 0){
			mutex_unlock(&acc->lock);                    
			return err;
		}	
               
		if (copy_to_user(argp, xyz, sizeof(xyz))) {			
			printk(KERN_ERR " %s %d error in copy_to_user \n",					 
				__func__, __LINE__);
			mutex_unlock(&acc->lock);			
			return -EINVAL;                
		} 
              
		break;
	case MXC400X_IOCTL_GET_TEMP:   

	{	
		u8  tempture = 0; 
		printk("%s MXC400X_IOCTL_GET_TEMP,mxc400x_register_read\n", __func__);
		err = mxc400x_register_read(acc, &tempture,MXC400X_REG_TEMP);                
		if (err < 0)  
		{
			printk("%s, error read register MXC400X_REG_TEMP\n", __func__);
			mutex_unlock(&acc->lock);
			return err;
		}   
		     	
		if (copy_to_user(argp, &tempture, sizeof(tempture))) {			
			printk(KERN_ERR " %s %d error in copy_to_user \n",					 
				__func__, __LINE__);
			mutex_unlock(&acc->lock);			
			return -EINVAL;                
		}    
	}            
	
	break;
	case MXC400X_IOCTL_GET_CHIP_ID:
	{
		u8 devid = 0;
		err = mxc400x_register_read(acc, &devid, WHO_AM_I);
		if (err < 0) {
			printk("%s, error read register WHO_AM_I\n", __func__);
			mutex_unlock(&acc->lock);
			return -EAGAIN;
		}
		
		if (copy_to_user(argp, &devid, sizeof(devid))) {
			printk("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
			mutex_unlock(&acc->lock);
			return -EINVAL;
		}
	}
            break;

	case MXC400X_IOCTL_GET_LAYOUT:
	    {
             if(acc->pdata->negate_z){mxc400x_dir = 4;}
             else{mxc400x_dir = 0;}
				if (copy_to_user(argp, &(mxc400x_dir),
							sizeof(mxc400x_dir)))
				{
					printk("%s error in copy_to_user(IOCTL_GET_CHIP_ID)\n", __func__);
					return -EINVAL;
				}
         }
			break;
	case MXC400X_IOCTL_READ_REG:
		if (copy_from_user(&reg_addr, argp, sizeof(reg_value)))
			return -EFAULT;
		data[0] = reg_addr;
		if (mxc400x_i2c_read(acc, data, 1) < 0) {
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		printk("<7>mxc400x Read register No. 0x%02x\n", data[0]);
		reg_value = data[0];
		if (copy_to_user(argp, &reg_value, sizeof(reg_value))) {
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}		
		break;       
 	case MXC400X_IOCTL_WRITE_REG:
		if (copy_from_user(&data, argp, sizeof(data)))
			return -EFAULT;
		if (mxc400x_i2c_write(acc, data, 2) < 0) {
			mutex_unlock(&acc->lock);
			return -EFAULT;
		}
		printk("<7>mxc400x Write '0x%02x' to  register No. 0x%02x\n", data[1], data[0]);
		break;   

	case POINTER_ACC_IOCTL_SET_APARMS:

		if (copy_from_user(parms, argp, sizeof(parms)))
{
			mutex_unlock(&acc->lock);
			return -EFAULT;
}		
		//printk("input hal data %d\n",pointer_z);
		transsion_sensor_report(acc->input_dev, (signed short)parms[0], (signed short)parms[1], (signed short)parms[2]);
		

		break;

	default:
	    printk("%s no cmd error\n", __func__);
	    mutex_unlock(&acc->lock);
		return -EINVAL;
	}
	
    mutex_unlock(&acc->lock);
	return 0;
}

static const struct file_operations mxc400x_misc_fops = {
		.owner = THIS_MODULE,
		.open = mxc400x_misc_open,
		.unlocked_ioctl = mxc400x_misc_ioctl,
};

static struct miscdevice mxc400x_misc_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = MXC400X_DEV_NAME,
		.fops = &mxc400x_misc_fops,
};

/***********************************************jiapeng add****************************************************/

static ssize_t  mxc400x_enable_show(void *dev, char *buf)
{
        struct mxc400x_data *mxc400x = dev;

        return sprintf(buf, "%d\n", atomic_read(&mxc400x->enabled));
}

static ssize_t  mxc400x_enable_store(void *dev,const char *buf, size_t count)
{
        unsigned int enable = simple_strtoul(buf, NULL, 10);

	struct mxc400x_data *mxc400x = dev;

        if (enable)
		mxc400x_enable(mxc400x);
        else
                mxc400x_disable(mxc400x);
	user_enable = enable;

        return count;
}

static ssize_t  mxc400x_delay_show(void *dev, char *buf)
{
	struct mxc400x_data *mxc400x = dev;

	return sprintf(buf, "%d\n", atomic_read(&mxc400x->delay));
}

static ssize_t  mxc400x_delay_store(void *dev,
                    const char *buf, size_t count)
{
    unsigned long data=0;
    int error=0;
	struct mxc400x_data *mxc400x = dev;

    error = kstrtoul(buf, 10, &data);
    if (error)
        return error;

    if(data <= 10) {
        data = 10;
    }

    atomic_set(&mxc400x->delay, (unsigned int) data);

    return count;
}

static int mxc400x_chip_info_is_ok = 0;

static ssize_t  mxc400x_chip_info_show(void *dev,
                                       char *buf)
{
	if(1 == mxc400x_chip_info_is_ok) {
        	return snprintf(buf, PAGE_SIZE, "Mxc400x");		
	}

        return snprintf(buf, PAGE_SIZE, "unknown\n");
}

/***************************************jiapeng add sys end************************************************/
//zhl add for read nv
static ssize_t  mxc400x_acc_nv_cali_show(void *dev,char *buf)
{
	read_acc_nv_data();
	return sprintf(buf, "%d %d %d\n",factory_acc_cali.x,factory_acc_cali.y,factory_acc_cali.z);
	
}
static void mxc400x_acc_input_work_func(struct work_struct *work)
{
	struct mxc400x_data *acc;

	int xyz[4] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			struct mxc400x_data,	input_work);

	mutex_lock(&acc->lock);

	err = mxc400x_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
	{

		transsion_sensor_report(acc->input_dev, (signed short)xyz[0], (signed short)xyz[1], (signed short)xyz[2]);
	
		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
				            atomic_read(&acc->delay)));
		
	}


	mutex_unlock(&acc->lock);
}
static int mxc400x_validate_pdata(struct mxc400x_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int mxc400x_acc_input_init(struct mxc400x_data *acc)
{
	int res = 0;
	INIT_DELAYED_WORK(&acc->input_work, mxc400x_acc_input_work_func);
	pr_info("ptaccelerator driver: init\n");
// pr_err("%s: ============OK chen ==============\n", __FUNCTION__);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		res = -ENOMEM;
		pr_err("%s: failed to allocate input device\n", __FUNCTION__);
		goto out;
	}
	// {printk("%s,  ptacc_data_device = input_allocate_device();if (!ptacc_data_device) 00  =====OK chen !!!!! \n", __func__);}

	input_set_drvdata(acc->input_dev, acc);
	set_bit(EV_ABS, acc->input_dev->evbit);

	/* 1024 == 1g, range -8g ~ +8g */
	/* ptacceleration x-axis */
	input_set_abs_params(acc->input_dev, ABS_X, 
		-1024*8, 1024*8, 0, 0);
	/* ptacceleration y-axis */
	input_set_abs_params(acc->input_dev, ABS_Y, 
		-1024*8, 1024*8, 0, 0);
	/* ptacceleration z-axis */
	input_set_abs_params(acc->input_dev, ABS_Z, 
		-1024*8, 1024*8, 0, 0);
	/* ptacceleration status, 0 ~ 3 */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, 
		-256, 256, 0, 0);


	acc->input_dev->name = "accelerometer";
	res = input_register_device(acc->input_dev);
	if (res) {
		pr_err("%s: unable to register input device: %s\n",
			__FUNCTION__, acc->input_dev->name);
		printk("jiapeng input_register_device failed \n");
		goto out_free_input;
	}
	// {printk("%s,  res = input_register_device(ptacc_data_device); if (res) 11  =====OK chen !!!!! \n", __func__);}

//	res = misc_register(&ptacc_ctrl_device);
//	if (res) {
//		pr_err("%s: ptacc_ctrl_device register failed\n", __FUNCTION__);
//		goto out_free_input;
//	}
//	else 
	// {printk("%s,  res = misc_register(&ptacc_ctrl_device);if (res)  22 =====OK chen !!!!! \n", __func__);}

//	res = device_create_file(ptacc_ctrl_device.this_device, &dev_attr_ptacc_ctrl);
//	if (res) {
//		pr_err("%s: device_create_file failed\n", __FUNCTION__);
//		goto out_deregister_misc;
//	}
// {printk("%s, res = device_create_file(ptacc_ctrl_device.this_device, &dev_attr_ptacc_ctrl); if (res) 33 =====OK chen !!!!! \n", __func__);}

	return 0;

//out_deregister_misc:
//	misc_deregister(&ptacc_ctrl_device);
out_free_input:
	input_free_device(acc->input_dev);
out:
	return res;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxc400x_early_suspend (struct early_suspend* es);
static void mxc400x_early_resume (struct early_suspend* es);
#endif

#ifdef CONFIG_OF
static struct mxc400x_platform_data *mxc400x_acc_parse_dt(struct device *dev)
{
	struct mxc400x_platform_data *pdata;
	//struct device_node *np = dev->of_node;
	int ret;
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct mxc622x_acc_platform_data");
		return NULL;
	}
	ret = of_property_read_u32(dev->of_node, "poll_interval", &pdata->poll_interval);
	//printk("poll_interval = %d\n",pdata->poll_interval);
	if(ret){
		dev_err(dev, "fail to get poll_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(dev->of_node, "min_interval", &pdata->min_interval);
	//printk("min_interval = %d\n",pdata->min_interval);
	if(ret){
		dev_err(dev, "fail to get min_interval\n");
		goto fail;
	}
	ret = of_property_read_u32(dev->of_node, "axis_map_x", &pdata->axis_map_x);
	//printk("axis_map_x = %d\n",pdata->axis_map_x);
	if(ret || pdata->axis_map_x>2){
		dev_err(dev, "fail to get axis_map_x\n");
		goto fail;
	}
	ret = of_property_read_u32(dev->of_node, "axis_map_y", &pdata->axis_map_y);
	//printk("axis_map_y = %d\n",pdata->axis_map_y);
	if(ret || pdata->axis_map_y>2){
		dev_err(dev, "fail to get axis_map_y\n");
		goto fail;
	}
	ret = of_property_read_u32(dev->of_node, "axis_map_z", &pdata->axis_map_z);
	//printk("axis_map_z = %d\n",pdata->axis_map_z);
	if(ret || pdata->axis_map_z>2){
		dev_err(dev, "fail to get axis_map_z\n");
		goto fail;
	}
	ret = of_property_read_u32(dev->of_node, "negate_x", &pdata->negate_x);
	//printk("negate_x = %d\n",pdata->negate_x);
	if(ret){
		dev_err(dev, "fail to get negate_x\n");
		goto fail;
	}
	ret = of_property_read_u32(dev->of_node, "negate_y", &pdata->negate_y);
	//printk("negate_y = %d\n",pdata->negate_y);
	if(ret){
		dev_err(dev, "fail to get negate_y\n");
		goto fail;
	}
	ret = of_property_read_u32(dev->of_node, "negate_z", &pdata->negate_z);
	//printk("negate_z = %d\n",pdata->negate_z);
	if(ret){
		dev_err(dev, "fail to get negate_z\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif
static int mxc400x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct mxc400x_data *acc;
	struct mxc400x_platform_data *pdata = client->dev.platform_data;
	int err = -1;
	int tempvalue;
	u8 buf[2];
	struct device_node *np = client->dev.of_node;

	printk("%s: jiapeng probe start.\n", MXC400X_DEV_NAME);


#ifdef CONFIG_OF

	if (np && !pdata){
		pdata = mxc400x_acc_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		if(!pdata){
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif
	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	mxc400x_chip_info_is_ok = 0;

/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	printk(KERN_INFO "chip id: %d !\n",	tempvalue);
/////////////////////chip id check 20190129 mayun
      if(((tempvalue & 0x003F)== MXC400X_ID_1) || ((tempvalue & 0x003F) == MXC400X_ID_2))
	//if ((tempvalue & 0x00FF) == WHOAMI_MXC400X) 
	{
		//jiapeng add
		mxc400x_chip_info_is_ok = 1;
		//jiapeng add end		
		printk(KERN_INFO "%s I2C driver registered!\n",
							MXC400X_DEV_NAME);
	} 
	else 
	{
	//			acc->client = NULL;
				printk(KERN_INFO "I2C driver not registered!"
						" Device unknown 0x%x\n", tempvalue);
				err = -ENODEV;
//				goto err_mutexunlockfreedata;
				goto exit_check_functionality_failed;
			
	}


	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE |
					I2C_FUNC_SMBUS_BYTE_DATA |
					I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}


	if (!i2c_check_functionality(client->adapter,
						I2C_FUNC_SMBUS_I2C_BLOCK)){
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	/*
	 * OK. From now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */

	acc = kzalloc(sizeof(struct mxc400x_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_alloc_data_failed;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	mxc400x_i2c_client = client;
	i2c_set_clientdata(client, acc);

	
	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n",
				err);
		goto exit_kfree_pdata;
	}

	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));

	err = mxc400x_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	i2c_set_clientdata(client, acc);


	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err2;
		}
	}

	err = mxc400x_device_power_on(acc);
	msleep(300);
	if (!acc->hw_initialized) {
		err = mxc400x_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			mxc400x_device_power_off(acc);
			goto err2;
		}
	}
	printk("mxc400x probe power on!\n");
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		printk("mxc400x probe power on fail!\n");
		goto err2;
	}

	atomic_set(&acc->enabled, 1);

	atomic_set(&acc->delay, 40);

	err = mxc400x_acc_input_init(acc);
	printk("mxc400x probe input init!\n");
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		printk("mxc400x probe input init fail!\n");
		goto err_power_off;
	}

	//err = sysfs_create_group(&acc->input_dev->dev.kobj, &mxc400x_attribute_group);
	//if(err < 0) {
	//	printk("jiapeng mxc400x probe sysfs_create_group fail!\n");
	//}

	mxc400x_misc_data = acc;

	err = misc_register(&mxc400x_misc_device);
	printk("mxc400x probe misc_register!\n");
	if (err < 0) {
		dev_err(&client->dev,
				"misc MXC400X_DEV_NAME register failed\n");

		//sysfs_remove_group(&acc->input_dev->dev.kobj, &mxc400x_attribute_group);

		printk("mxc400x probe misc_register fail!\n");
		goto exit_kfree_pdata;
	}

	//mxc400x_device_power_off(acc);

	buf[0] = MXC400X_REG_CTRL;
	buf[1] = MXC400X_CTRL_PWRDN;
	err = mxc400x_i2c_write(acc, buf, 1);
	
	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);
	user_enable = 0;

 #ifdef CONFIG_HAS_EARLYSUSPEND
    acc->early_suspend.suspend = mxc400x_early_suspend;
    acc->early_suspend.resume  = mxc400x_early_resume;
    acc->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    register_early_suspend(&acc->early_suspend);
#endif

#ifdef CONFIG_TRANSSION_HWINFO
        transsion_hwinfo_add("gsensor_vendor", "MX");
        transsion_hwinfo_add("gsensor_ic", "MXC4005XC");
#endif

	transsion_sensor_add("enable",		mxc400x_enable_show,		mxc400x_enable_store,		acc);
	transsion_sensor_add("delay", 		mxc400x_delay_show, 		mxc400x_delay_store,		acc);
	transsion_sensor_add("chip_info", 	mxc400x_chip_info_show, 	NULL,				acc);
	transsion_sensor_add("acc_nv", 	    mxc400x_acc_nv_cali_show, 	NULL,				acc);
	mutex_unlock(&acc->lock);

	dev_info(&client->dev, "%s: probed ok\n", MXC400X_DEV_NAME);

	return 0;


err2:
	if (acc->pdata->exit) acc->pdata->exit();
exit_kfree_pdata:
	kfree(acc->pdata);
err_power_off:
	mxc400x_device_power_off(acc);
//err_mutexunlockfreedata:
 	mutex_unlock(&acc->lock);
	i2c_set_clientdata(client, NULL);
	mxc400x_misc_data = NULL;
exit_alloc_data_failed:
exit_check_functionality_failed:
	printk(KERN_ERR "%s: Driver Init failed\n", MXC400X_DEV_NAME);
	//return err;
exit_alloc_platform_data_failed:
	kfree(acc);
	return err;
}

static int mxc400x_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct mxc400x_data *acc = i2c_get_clientdata(client);

    	misc_deregister(&mxc400x_misc_device);
    	mxc400x_device_power_off(acc);

	//	sysfs_remove_group(&acc->input_dev->dev.kobj, &mxc400x_attribute_group);

    	if (acc->pdata->exit)
    		acc->pdata->exit();
    	kfree(acc->pdata);
    	kfree(acc);

	return 0;
}

static int mxc400x_resume(struct device *dev)
{
	struct mxc400x_data *acc = dev_get_drvdata(dev);
#ifdef MXC400X_DEBUG
    printk("%s.\n", __func__);
#endif

    if (acc && user_enable) {
	return mxc400x_enable(acc);
    }
    return 0;
}

static int mxc400x_suspend(struct device *dev)
{
	struct mxc400x_data *acc = dev_get_drvdata(dev);
#ifdef MXC400X_DEBUG
    printk("%s.\n", __func__);
#endif
    if (acc) {
            return mxc400x_disable(acc);
    }
    return 0;
}

static const struct dev_pm_ops mxc400x_pm_ops = {
    .suspend = mxc400x_suspend,
    .resume = mxc400x_resume,
};

static const struct i2c_device_id mxc400x_id[]
				= { { MXC400X_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, mxc400x_id);

static const struct of_device_id mxc400x_acc_of_match[] = {
       { .compatible = "MXC,mxc400x_acc", },
       { }
};
MODULE_DEVICE_TABLE(of, mxc400x_acc_of_match);
static struct i2c_driver mxc400x_driver = {
	.driver = {
			.name = MXC400X_I2C_NAME,
			.of_match_table = mxc400x_acc_of_match,
                        .pm = &mxc400x_pm_ops,
		  },
	.probe = mxc400x_probe,
	.remove = mxc400x_remove,
	.id_table = mxc400x_id,
};


static int __init mxc400x_init(void)
{
        int  ret = 0;

    	printk(KERN_INFO "%s accelerometer driver: init\n",
						MXC400X_I2C_NAME);
						
        ret = i2c_add_driver(&mxc400x_driver);
        printk(KERN_INFO "%s add driver: %d\n",
						MXC400X_I2C_NAME,ret);
        return ret;
}

static void __exit mxc400x_exit(void)
{
	printk(KERN_INFO "%s accelerometer driver exit\n", MXC400X_DEV_NAME);

	i2c_del_driver(&mxc400x_driver);
	return;
}

module_init(mxc400x_init);
module_exit(mxc400x_exit);

MODULE_DESCRIPTION("MEMSIC MXC400X Accelerometer Sensor Driver");
MODULE_LICENSE("GPL");
