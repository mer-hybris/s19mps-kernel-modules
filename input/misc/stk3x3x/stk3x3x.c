/*
 *  stk3x3x.c - Linux kernel modules for sensortek stk301x, stk321x, stk331x 
 *  , and stk3410 proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2013 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/errno.h>
// #include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include   <linux/fs.h>
#include <linux/miscdevice.h> 

#include <linux/uaccess.h>
#include <linux/of_gpio.h>


#include "prj/prj_config.h"

#include "stk3x3x_pls.h"

#define DRIVER_VERSION  "3.8.3 20150401"

/* Driver Settings */
//#define CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD

#ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#define STK_ALS_CHANGE_THD	10	/* The threshold to trigger ALS interrupt, unit: lux */	
#endif	/* #ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD */

#define STK_INT_PS_MODE			1	/* 1, 2, or 3	*/
//#define STK_POLL_PS
#define STK_POLL_ALS		/* ALS interrupt is valid only when STK_PS_INT_MODE = 1	or 4*/
#define STK_TUNE0
//#define CALI_PS_EVERY_TIME
//#define STK_DEBUG_PRINTF

#define STK_ALS_FIR
//#define STK_CHK_REG


/* Define Register Map */
#define STK_STATE_REG 			0x00
#define STK_PSCTRL_REG 			0x01
#define STK_ALSCTRL_REG 			0x02
#define STK_LEDCTRL_REG 			0x03
#define STK_INT_REG 				0x04
#define STK_WAIT_REG 			0x05
#define STK_THDH1_PS_REG 		0x06
#define STK_THDH2_PS_REG 		0x07
#define STK_THDL1_PS_REG 		0x08
#define STK_THDL2_PS_REG 		0x09
#define STK_THDH1_ALS_REG 		0x0A
#define STK_THDH2_ALS_REG 		0x0B
#define STK_THDL1_ALS_REG 		0x0C
#define STK_THDL2_ALS_REG 		0x0D
#define STK_FLAG_REG 			0x10
#define STK_DATA1_PS_REG	 	0x11
#define STK_DATA2_PS_REG 		0x12
#define STK_DATA1_ALS_REG 		0x13
#define STK_DATA2_ALS_REG 		0x14
#define STK_DATA1_R_REG					0x15
#define STK_DATA2_R_REG					0x16
#define STK_DATA1_G_REG				    0x17
#define STK_DATA2_G_REG					0x18
#define STK_DATA1_B_REG					0x19
#define STK_DATA2_B_REG					0x1A
#define STK_DATA1_C_REG					0x1B
#define STK_DATA2_C_REG					0x1C
#define STK_DATA1_OFFSET_REG			0x1D
#define STK_DATA2_OFFSET_REG			0x1E
#define STK_DATA_CTIR1_REG				0x20
#define STK_DATA_CTIR2_REG				0x21
#define STK_DATA_CTIR3_REG				0x22
#define STK_DATA_CTIR4_REG				0x23
#define STK_PDT_ID_REG					0x3E
#define STK_RSRVD_REG					0x3F
#define STK_ALSCTRL2_REG				0x4E
#define STK_INTELLI_WAIT_PS_REG			0x4F
#define STK_SW_RESET_REG				0x80
#define STK_INTCTRL2_REG				0xA4


/* Define state reg */
#define STK_STATE_EN_WAIT_SHIFT  	2
#define STK_STATE_EN_ALS_SHIFT  	1
#define STK_STATE_EN_PS_SHIFT  		0
#define STK_STATE_EN_INTELL_PRST_SHIFT	3

#define STK_STATE_EN_CT_AUTOK_MASK         0x10
#define STK_STATE_EN_INTELL_PRST_MASK      0x08
#define STK_STATE_EN_WAIT_MASK	0x04
#define STK_STATE_EN_ALS_MASK	0x02
#define STK_STATE_EN_PS_MASK	0x01

/* Define PS ctrl reg */
#define STK_PS_PRS_SHIFT  		6
#define STK_PS_GAIN_SHIFT  		4
#define STK_PS_IT_SHIFT  			0

#define STK_PS_PRS_MASK			0xC0
#define STK_PS_GAIN_MASK			0x30
#define STK_PS_IT_MASK			0x0F

/* Define ALS ctrl reg */
#define STK_ALS_PRS_SHIFT  		6
#define STK_ALS_GAIN_SHIFT  		4
#define STK_ALS_IT_SHIFT  			0

#define STK_ALS_PRS_MASK		0xC0
#define STK_ALS_GAIN_MASK		0x30
#define STK_ALS_IT_MASK			0x0F
	
/* Define LED ctrl reg */
#define STK_EN_CTIR_SHIFT           0
#define STK_EN_CTIRFC_SHIFT         1
#define STK_LED_IRDR_SHIFT  		6

#define STK_EN_CTIR_MASK        0x01
#define STK_EN_CTIRFC_MASK      0x02
#define STK_LED_IRDR_MASK		0xC0

/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT  		7
#define STK_INT_INVALID_PS_SHIFT    5
#define STK_INT_ALS_SHIFT  			3
#define STK_INT_PS_SHIFT  			0

#define STK_INT_CTRL_MASK			0x80
#define STK_INT_INVALID_PS_MASK		0x20
#define STK_INT_ALS_MASK			0x08
#define STK_INT_PS_MASK				0x07

#define STK_INT_PS_MODE1 			0x01
#define STK_INT_ALS					0x08
#define STK_INT_PS                  0x01

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT  			7
#define STK_FLG_PSDR_SHIFT  			6
#define STK_FLG_ALSINT_SHIFT  			5
#define STK_FLG_PSINT_SHIFT  			4
#define STK_FLG_ALSSAT_SHIFT  			2
#define STK_FLG_INVALID_PSINT_SHIFT  	1
#define STK_FLG_NF_SHIFT  				0

#define STK_FLG_ALSDR_MASK				0x80
#define STK_FLG_PSDR_MASK				0x40
#define STK_FLG_ALSINT_MASK				0x20
#define STK_FLG_PSINT_MASK				0x10
#define STK_FLG_ALSSAT_MASK				0x04
#define STK_FLG_INVALID_PSINT_MASK		0x02
#define STK_FLG_NF_MASK					0x01

/* Define ALS CTRL-2 reg */
#define STK_ALSC_GAIN_SHIFT         0x04
#define STK_ALSC_GAIN_MASK          0x30

/* Define INT-2 reg */
#define STK_INT_ALS_DR_SHIFT        0x01
#define STK_INT_PS_DR_SHIFT         0x00
#define STK_INT_ALS_DR_MASK         0x02
#define STK_INT_PS_DR_MASK          0x01

	
/* misc define */
#define MIN_ALS_POLL_DELAY_NS	60000000


#ifdef STK_TUNE0
	#define STK_MAX_MIN_DIFF	300
	#define STK_LT_N_CT	100
	#define STK_HT_N_CT	150
#endif	/* #ifdef STK_TUNE0 */

#define STK_IRC_MAX_ALS_CODE		20000
#define STK_IRC_MIN_ALS_CODE		25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		1	
#define STK_IRC_ALS_NUMERA		10	//5
#define STK_IRC_ALS_CORREC		117	//850

#define DEVICE_NAME		"stk3x3x_pls"
#define DEVICE_ADDRESS 0x47
#define ALS_NAME "lightsensor-level"
#define PS_NAME "proximity"

#define STK3331_PID			0x53
#define STK3332_PID			0x52
#define STK3335_PID			0x51
#define STK3337_PID			0x57
#define STK3338_PID			0x58

#define STK3X3X_PLS_MISC_DEVICE 		"stk3x3x_pls" /* for misc_device */
static int stk3x3x_pls_misc_opened=0; /* for misc_device */
static struct stk3x3x_data *stk3x3x_data_misc = NULL; /* for misc_device */



#ifdef STK_ALS_FIR
#define STK_FIR_LEN	8
#define MAX_FIR_LEN 32
struct data_filter {
    u16 raw[MAX_FIR_LEN];
    int sum;
    int number;
    int idx;
};
#endif



struct stk3x3x_data {
	struct i2c_client *client;

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
	int32_t irq;
	struct work_struct stk_work;
	struct workqueue_struct *stk_wq;	
#endif	

	uint16_t ir_code;
	uint16_t als_correct_factor;
	uint8_t alsctrl_reg;
	uint8_t psctrl_reg;
	uint8_t ledctrl_reg;
	uint8_t state_reg;
	int		int_pin;
	uint8_t wait_reg;
	uint8_t int_reg;
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;

#ifdef CALI_PS_EVERY_TIME	
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif
  
  struct input_dev *input;
	struct mutex io_lock;
	//struct input_dev *ps_input_dev;
	int32_t ps_distance_last;
	bool ps_enabled;
	bool re_enable_ps;
	struct wakeup_source *ps_wakelock;

#ifdef STK_POLL_PS		
	struct hrtimer ps_timer;	
	struct work_struct stk_ps_work;
	struct workqueue_struct *stk_ps_wq;
	struct wakeup_source *ps_nosuspend_wl;		
#endif

	//struct input_dev *als_input_dev;
	int32_t als_lux_last;
	uint32_t als_transmittance;	
	bool als_enabled;
	bool re_enable_als;
	ktime_t ps_poll_delay;
	ktime_t als_poll_delay;

#ifdef STK_POLL_ALS		
	struct work_struct stk_als_work;
	struct hrtimer als_timer;	
	struct workqueue_struct *stk_als_wq;
#endif
	
	bool first_boot;
	
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *stk_ps_tune0_wq;
	struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;
	int stk_max_min_diff;
	int stk_lt_n_ct;
	int stk_ht_n_ct;
#endif

#ifdef STK_ALS_FIR
	struct data_filter      fir;
	atomic_t                firlength;	
#endif

	atomic_t	recv_reg;



	uint8_t pid;
	uint32_t als_code_last;
	uint32_t  c_code_last;
};

#if( !defined(CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD))
static uint32_t lux_threshold_table[] =
{
	3,
	10,
	40,
	65,
	145,
	300,
	550,
	930,
	1250,
	1700,
};

#define LUX_THD_TABLE_SIZE (sizeof(lux_threshold_table)/sizeof(uint32_t)+1)
static uint16_t code_threshold_table[LUX_THD_TABLE_SIZE+1];
#endif 	

static int32_t stk3x3x_enable_ps(struct stk3x3x_data *ps_data, uint8_t enable, uint8_t validate_reg);
static int32_t stk3x3x_enable_als(struct stk3x3x_data *ps_data, uint8_t enable);
static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l);
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h);
static int32_t stk3x3x_set_als_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l);
static int32_t stk3x3x_set_als_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h);

#ifdef STK_TUNE0
static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *ps_data);
#endif
#ifdef STK_CHK_REG
static int stk3x3x_validate_n_handle(struct i2c_client *client);
#endif
static int stk_ps_val(struct stk3x3x_data *ps_data);


//wangyang add the lightsensor name----  down----
static ssize_t lsensor_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        return sprintf(buf,"%s",DEVICE_NAME);
}
static struct kobj_attribute lsensor_name_attr =
{
    .attr =
    {    
        .name = "lsensor_name", 
        .mode = S_IRUGO,
    },   
    .show = &lsensor_name_show,
};
static struct attribute *properties_attrs[] =
{
    &lsensor_name_attr.attr,

    NULL 
};


static struct attribute_group properties_attr_group =
{
    .attrs = properties_attrs,
};

static struct kobject *properties_kobj;
static void lsensor_name_create(void)
{
    int ret; 
    properties_kobj = kobject_create_and_add("lsensor_properties", NULL);
    if (properties_kobj)
    {    
        ret = sysfs_create_group(properties_kobj, &properties_attr_group);
    }    

    if (!properties_kobj || ret) 
    {    
        pr_err("failed to create lsensor_properties\n");
    }

}
//static DEVICE_ATTR(lsensor_properties, 0666, lsensor_name_show, NULL);
//wangyang -------------up

static int stk3x3x_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	uint8_t retry;	
	int err;
	struct i2c_msg msgs[] = 
	{
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &command,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = values,
		},
	};
	
	for (retry = 0; retry < 5; retry++) 
	{
		err = i2c_transfer(client->adapter, msgs, 2);
		if (err == 2)
			break;
		else
			mdelay(5);
	}
	
	if (retry >= 5) 
	{
		printk(KERN_ERR "%s: i2c read fail, err=%d\n", __func__, err);
		return -EIO;
	} 
	return 0;		
}

static int stk3x3x_i2c_write_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
{
	int retry;
	int err;	
	unsigned char data[11];
	struct i2c_msg msg;
	int index;

    if (!client)
		return -EINVAL;
    else if (length >= 10) 
	{        
        printk(KERN_ERR "%s:length %d exceeds 10\n", __func__, length);
        return -EINVAL;
    }   	
	
	data[0] = command;
	for (index=1;index<=length;index++)
		data[index] = values[index-1];	
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = length+1;
	msg.buf = data;
	
	for (retry = 0; retry < 5; retry++) 
	{
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		else
			mdelay(5);
	}
	
	if (retry >= 5) 
	{
		printk(KERN_ERR "%s: i2c write fail, err=%d\n", __func__, err);		
		return -EIO;
	}
	return 0;
}

static int stk3x3x_i2c_smbus_read_byte_data(struct i2c_client *client, unsigned char command)
{
	unsigned char value;
	int err;
	err = stk3x3x_i2c_read_data(client, command, 1, &value);
	if(err < 0)
		return err;
	return value;
}

static int stk3x3x_i2c_smbus_write_byte_data(struct i2c_client *client, unsigned char command, unsigned char value)
{
	int err;
	err = stk3x3x_i2c_write_data(client, command, 1, &value);
	return err;
}

uint32_t stk_alscode2lux(struct stk3x3x_data *ps_data, uint32_t alscode)
{
	alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));   
    alscode<<=3; 
    alscode/=ps_data->als_transmittance;
	return alscode;
}

uint32_t stk_lux2alscode(struct stk3x3x_data *ps_data, uint32_t lux)
{
    lux*=ps_data->als_transmittance;
    lux/=1100;
    if (unlikely(lux>=(1<<16)))
        lux = (1<<16) -1;
    return lux;
}

#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
static void stk_init_code_threshold_table(struct stk3x3x_data *ps_data)
{
    uint32_t i,j;
    uint32_t alscode;

    code_threshold_table[0] = 0;
#ifdef STK_DEBUG_PRINTF	
    printk(KERN_INFO "alscode[0]=%d\n",0);
#endif	
    for (i=1,j=0;i<LUX_THD_TABLE_SIZE;i++,j++)
    {
        alscode = stk_lux2alscode(ps_data, lux_threshold_table[j]);
        printk(KERN_INFO "alscode[%d]=%d\n",i,alscode);
        code_threshold_table[i] = (uint16_t)(alscode);
    }
    code_threshold_table[i] = 0xffff;
    printk(KERN_INFO "alscode[%d]=%d\n",i,alscode);
}

static uint32_t stk_get_lux_interval_index(uint16_t alscode)
{
    uint32_t i;
    for (i=1;i<=LUX_THD_TABLE_SIZE;i++)
    {
        if ((alscode>=code_threshold_table[i-1])&&(alscode<code_threshold_table[i]))
        {
            return i;
        }
    }
    return LUX_THD_TABLE_SIZE;
}
#else
void stk_als_set_new_thd(struct stk3x3x_data *ps_data, uint16_t alscode)
{
    int32_t high_thd,low_thd;
    high_thd = alscode + stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);
    low_thd = alscode - stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);
    if (high_thd >= (1<<16))
        high_thd = (1<<16) -1;
    if (low_thd <0)
        low_thd = 0;
    stk3x3x_set_als_thd_h(ps_data, (uint16_t)high_thd);
    stk3x3x_set_als_thd_l(ps_data, (uint16_t)low_thd);
}
#endif // CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD


static void stk3x3x_proc_plat_data(struct stk3x3x_data *ps_data, struct stk3x3x_platform_data *plat_data)
{
	uint8_t w_reg;
	
	ps_data->state_reg = plat_data->state_reg;
	ps_data->psctrl_reg = 0x31;//plat_data->psctrl_reg;
#ifdef STK_POLL_PS
	ps_data->psctrl_reg &= 0x3F;
#endif
	ps_data->alsctrl_reg = 0x31;//plat_data->alsctrl_reg;
	//ps_data->ledctrl_reg = plat_data->ledctrl_reg;
	
	ps_data->wait_reg = plat_data->wait_reg;	
	if(ps_data->wait_reg < 2)
	{
		printk(KERN_WARNING "%s: wait_reg should be larger than 2, force to write 2\n", __func__);
		ps_data->wait_reg = 2;
	}
	else if (ps_data->wait_reg > 0xFF)
	{
		printk(KERN_WARNING "%s: wait_reg should be less than 0xFF, force to write 0xFF\n", __func__);
		ps_data->wait_reg = 0xFF;		
	}
//#ifndef STK_TUNE0		
	if(ps_data->ps_thd_h == 0 && ps_data->ps_thd_l == 0)
	{
	ps_data->ps_thd_h = plat_data->ps_thd_h;
	ps_data->ps_thd_l = plat_data->ps_thd_l;		
	}
//#endif	
#ifdef CALI_PS_EVERY_TIME
	ps_data->ps_high_thd_boot = plat_data->ps_thd_h;
	ps_data->ps_low_thd_boot = plat_data->ps_thd_l;	
#endif	

	w_reg = 0;
#ifndef STK_POLL_PS	
	w_reg |= STK_INT_PS_MODE;	
#else
	w_reg |= 0x01;		
#endif	

#if (!defined(STK_POLL_ALS) && (STK_INT_PS_MODE != 0x02) && (STK_INT_PS_MODE != 0x03))
	w_reg |= STK_INT_ALS;
#endif	
	ps_data->int_reg = w_reg;
	return;
}

static int32_t stk3x3x_init_all_reg(struct stk3x3x_data *ps_data)
{
	int32_t ret;
	
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, ps_data->state_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, ps_data->psctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, ps_data->alsctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, ps_data->ledctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
#ifdef STK_TUNE0	
	ps_data->psa = 0x0;
	ps_data->psi = 0xFFFF;	
#endif	
//#else
	stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);	
//#endif	

    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	

	return 0;
}

static int32_t stk3x3x_check_pid(struct stk3x3x_data *ps_data)
{
	unsigned char value[3];
	int err;
	
	
	err = stk3x3x_i2c_read_data(ps_data->client, STK_PDT_ID_REG, 2, &value[0]);
	if(err < 0) {
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
	err = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, 0xE0);
	if(err < 0)
		return err;
	value[2] = err;

	printk(KERN_INFO "%s: PID=0x%x, RID=0x%x, 0x90=0x%x\n", __func__, value[0], value[1], value[2]);
	ps_data->pid = value[0];
	
	if(value[0] == STK3338_PID)
		ps_data->ledctrl_reg =0x40;
	else
		ps_data->ledctrl_reg =0xA0; //0x60;
	return 0;
}


static int32_t stk3x3x_software_reset(struct stk3x3x_data *ps_data)
{
    int32_t r;
    uint8_t w_reg;
	
    w_reg = 0x7F;
    r = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_WAIT_REG,w_reg);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: write i2c error, ret=%d\n", __func__, r);
        return r;
    }
    r = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_WAIT_REG);
    if (w_reg != r)
    {
        printk(KERN_ERR "%s: software reset: read-back value is not the same\n", __func__);
        return -1;
    }
	
    r = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_SW_RESET_REG,0);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
        return r;
    }
	usleep_range(13000, 15000);
    return 0;
}


static int32_t stk3x3x_set_als_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDL1_ALS_REG, 2, val);
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;		
}
static int32_t stk3x3x_set_als_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDH1_ALS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;	
}

static int32_t stk3x3x_set_ps_thd_l(struct stk3x3x_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDL1_PS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;	
}
static int32_t stk3x3x_set_ps_thd_h(struct stk3x3x_data *ps_data, uint16_t thd_h)
{	
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_THDH1_PS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;
}

static uint32_t stk3x3x_get_ps_reading(struct stk3x3x_data *ps_data)
{	
	unsigned char value[2];
	int err;
	err = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_PS_REG, 2, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
	return ((value[0]<<8) | value[1]);	
}


static int32_t stk3x3x_set_flag(struct stk3x3x_data *ps_data, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	int ret;
	
	w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK );
	w_flag &= (~clr);
	//printk(KERN_INFO "%s: org_flag_reg=0x%x, w_flag = 0x%x\n", __func__, org_flag_reg, w_flag);		
	
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_FLAG_REG, w_flag);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3x3x_get_flag(struct stk3x3x_data *ps_data)
{	
	int ret;
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_FLAG_REG);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3x3x_set_state(struct stk3x3x_data *ps_data, uint8_t state)
{
	int ret;		
    ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client,STK_STATE_REG, state);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3x3x_get_state(struct stk3x3x_data *ps_data)
{	
	int ret;
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}



static int32_t stk3x3x_enable_ps(struct stk3x3x_data *ps_data, uint8_t enable, uint8_t validate_reg)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;	
	uint32_t reading;
	int32_t near_far_state;		

#ifdef STK_CHK_REG
	if(validate_reg)
	{
		ret = stk3x3x_validate_n_handle(ps_data->client);
		if(ret < 0)	
			printk(KERN_ERR "stk3x3x_validate_n_handle fail: %d\n", ret); 
	}			
#endif /* #ifdef STK_CHK_REG */	

	curr_ps_enable = ps_data->ps_enabled?1:0;	
	if(curr_ps_enable == enable)
		return 0;
	
#ifdef STK_TUNE0
	if (!(ps_data->psi_set) && !enable)
	{

		hrtimer_cancel(&ps_data->ps_tune0_timer);					
		cancel_work_sync(&ps_data->stk_ps_tune0_work);

	}
#endif		
	if(ps_data->first_boot == true)
	{		
		ps_data->first_boot = false;
	}

	ret = stk3x3x_get_state(ps_data);
	if(ret < 0)
		return ret;
	w_state_reg = ret;
	
	
	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK ); 
	if(enable)	
	{
		w_state_reg |= STK_STATE_EN_PS_MASK;	
		if(!(ps_data->als_enabled))
			w_state_reg |= STK_STATE_EN_WAIT_MASK;			
	}	
	ret = stk3x3x_set_state(ps_data, w_state_reg);
	if(ret < 0)
		return ret;	

    if(enable)
	{
#ifdef STK_TUNE0
	#ifdef CALI_PS_EVERY_TIME
		ps_data->psi_set = 0;
		ps_data->psa = 0;
		ps_data->psi = 0xFFFF;
		ps_data->ps_thd_h = ps_data->ps_high_thd_boot;
		ps_data->ps_thd_l = ps_data->ps_low_thd_boot;
		hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);					
	#else

		if (!(ps_data->psi_set))
			hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);			
	#endif	/* #ifdef CALI_PS_EVERY_TIME */
		stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);			
#endif			
		printk(KERN_INFO "%s: HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h,  ps_data->ps_thd_l);				
#ifdef STK_POLL_PS		
		hrtimer_start(&ps_data->ps_timer, ps_data->ps_poll_delay, HRTIMER_MODE_REL);	
		ps_data->ps_distance_last = -1;	
#endif		
#ifndef STK_POLL_PS
	#ifndef STK_POLL_ALS		
		if(!(ps_data->als_enabled))
	#endif	/* #ifndef STK_POLL_ALS	*/
			enable_irq(ps_data->irq);
#endif	/* #ifndef STK_POLL_PS */						
		ps_data->ps_enabled = true;
#ifdef STK_CHK_REG		
		if(!validate_reg)		
		{
			ps_data->ps_distance_last = 1;
			input_report_abs(ps_data->input, ABS_DISTANCE, 1);
			input_sync(ps_data->input);
			__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ); 
			reading = stk3x3x_get_ps_reading(ps_data);
			printk(KERN_INFO "%s: force report ps input event=1, ps code = %d\n",__func__, reading);				
		}
		else
#endif /* #ifdef STK_CHK_REG */
		{
			usleep_range(4000, 5000);

/*			input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, 1);
			input_sync(ps_data->ps_input_dev);
			msleep(50);
			//printk("\n\n ## 1 ##\n\n");
			input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, 0);
			input_sync(ps_data->ps_input_dev);
			msleep(50);
			//printk("\n\n ## 2 ##\n\n");
			input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, 1);
			input_sync(ps_data->ps_input_dev);
			msleep(50);
			//printk("\n\n ## 3 ##\n\n");	*/
			
			ret = stk3x3x_get_flag(ps_data);
			if (ret < 0)
				return ret;
			
/*			near_far_state = ret & STK_FLG_NF_MASK;					
			ps_data->ps_distance_last = near_far_state;
			input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, near_far_state);
			input_sync(ps_data->ps_input_dev);
			msleep(50);
			//printk("\n\n ## 4 near_far_state is %d ##\n\n", near_far_state);	*/

			near_far_state = ret & STK_FLG_NF_MASK;					
			ps_data->ps_distance_last = near_far_state;
			input_report_abs(ps_data->input, ABS_DISTANCE, near_far_state);
			input_sync(ps_data->input);
			__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ);  
			reading = stk3x3x_get_ps_reading(ps_data);
			printk(KERN_INFO "%s: ps input event=%d, ps code = %d\n",__func__, near_far_state, reading);	
		}
	}
	else
	{		
#ifdef STK_POLL_PS
		hrtimer_cancel(&ps_data->ps_timer);
		cancel_work_sync(&ps_data->stk_ps_work);
#else		
#ifndef STK_POLL_ALS
		if(!(ps_data->als_enabled))	
#endif				
			disable_irq(ps_data->irq);
#endif
		ps_data->ps_enabled = false;			
	}
	return ret;
}

static int32_t stk3x3x_enable_als(struct stk3x3x_data *ps_data, uint8_t enable)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_als_enable = (ps_data->als_enabled)?1:0;

	
	if(curr_als_enable == enable)
		return 0;
	
#ifndef STK_POLL_ALS			
	
    if (enable)
	{				
        stk3x3x_set_als_thd_h(ps_data, 0x0000);
        stk3x3x_set_als_thd_l(ps_data, 0xFFFF);		
	}
#endif	

	ret = stk3x3x_get_state(ps_data);
	if(ret < 0)
		return ret;
	
	w_state_reg = (uint8_t)(ret & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK))); 
	if(enable)	
		w_state_reg |= STK_STATE_EN_ALS_MASK;	
	else if (ps_data->ps_enabled)		
		w_state_reg |= STK_STATE_EN_WAIT_MASK;	

	ret = stk3x3x_set_state(ps_data, w_state_reg);
	if(ret < 0)
		return ret;	

    if (enable)
    {						
		ps_data->als_enabled = true;
#ifdef STK_POLL_ALS			
		hrtimer_start(&ps_data->als_timer, ps_data->als_poll_delay, HRTIMER_MODE_REL);		
#else
#ifndef STK_POLL_PS
		if(!(ps_data->ps_enabled))
#endif		
			enable_irq(ps_data->irq);
#endif		

    }
	else
	{
		ps_data->als_enabled = false;
#ifdef STK_POLL_ALS			
		hrtimer_cancel(&ps_data->als_timer);
		cancel_work_sync(&ps_data->stk_als_work);
#else
#ifndef STK_POLL_PS
		if(!(ps_data->ps_enabled))	
#endif		
			disable_irq(ps_data->irq);		
#endif		
	}
    return ret;
}

static int32_t stk3x3x_get_als_reading(struct stk3x3x_data *ps_data)
{
    int32_t als_data;
#ifdef STK_ALS_FIR
	int index;   
	int firlen = atomic_read(&ps_data->firlength);   
#endif	
	unsigned char value[2];
	int ret;
	
	ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	

	ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_R_REG, 2, &value[0]);
	als_data = (value[0]<<8) | value[1];
	ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_G_REG, 2, &value[0]);
	als_data += (value[0]<<8) | value[1];
	ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_B_REG, 2, &value[0]);
	als_data += (value[0]<<8) | value[1];
	
	if (ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}

	ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_C_REG, 2, &value[0]);
	if (ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	ps_data->c_code_last = (value[0] << 8) | value[1];

	ps_data->als_code_last = als_data;
	
#ifdef STK_ALS_FIR
	if(ps_data->fir.number < firlen)
	{                
		ps_data->fir.raw[ps_data->fir.number] = als_data;
		ps_data->fir.sum += als_data;
		ps_data->fir.number++;
		ps_data->fir.idx++;
	}
	else
	{
		index = ps_data->fir.idx % firlen;
		ps_data->fir.sum -= ps_data->fir.raw[index];
		ps_data->fir.raw[index] = als_data;
		ps_data->fir.sum += als_data;
		ps_data->fir.idx++;
		als_data = ps_data->fir.sum/firlen;
	}	
#endif	
	
	return als_data;
}


#ifdef STK_CHK_REG
static int stk3x3x_chk_reg_valid(struct stk3x3x_data *ps_data) 
{
	unsigned char value[9];
	int err;
	/*
	uint8_t cnt;
		
	for(cnt=0;cnt<9;cnt++)
	{
		value[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (cnt+1));
		if(value[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, value[cnt]);	
			return value[cnt];
		}
	}	
	*/
	err = stk3x3x_i2c_read_data(ps_data->client, STK_PSCTRL_REG, 9, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}	

	if(value[0] != ps_data->psctrl_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x01=0x%2x\n", __func__, value[0]);
		return 0xFF;
	}	
	if(value[1] != ps_data->alsctrl_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x02=0x%2x\n", __func__, value[1]);
		return 0xFF;
	}
	if(value[2] != ps_data->ledctrl_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x03=0x%2x\n", __func__, value[2]);
		return 0xFF;
	}	
	if(value[3] != ps_data->int_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x04=0x%2x\n", __func__, value[3]);
		return 0xFF;
	}
	if(value[4] != ps_data->wait_reg)
	{
		printk(KERN_ERR "%s: invalid reg 0x05=0x%2x\n", __func__, value[4]);
		return 0xFF;
	}		
	if(value[5] != ((ps_data->ps_thd_h & 0xFF00) >> 8))
	{
		printk(KERN_ERR "%s: invalid reg 0x06=0x%2x\n", __func__, value[5]);
		return 0xFF;
	}		
	if(value[6] != (ps_data->ps_thd_h & 0x00FF))
	{
		printk(KERN_ERR "%s: invalid reg 0x07=0x%2x\n", __func__, value[6]);
		return 0xFF;
	}		
	if(value[7] != ((ps_data->ps_thd_l & 0xFF00) >> 8))
	{
		printk(KERN_ERR "%s: invalid reg 0x08=0x%2x\n", __func__, value[7]);
		return 0xFF;
	}		
	if(value[8] != (ps_data->ps_thd_l & 0x00FF))
	{
		printk(KERN_ERR "%s: invalid reg 0x09=0x%2x\n", __func__, value[8]);
		return 0xFF;
	}		
	
	return 0;
}

static int stk3x3x_validate_n_handle(struct i2c_client *client) 
{
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);
	int err;
	
	err = stk3x3x_chk_reg_valid(ps_data);
	if(err < 0)
	{
		printk(KERN_ERR "stk3x3x_chk_reg_valid fail: %d\n", err);        
		return err;
	}
	
	if(err == 0xFF)
	{		
		printk(KERN_ERR "%s: Re-init chip\n", __func__);				
		err = stk3x3x_software_reset(ps_data); 
		if(err < 0)
			return err;			
		err = stk3x3x_init_all_reg(ps_data);
		if(err < 0)
			return err;			
		
		//ps_data->psa = 0;
		//ps_data->psi = 0xFFFF;		
		stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);		
#ifdef STK_ALS_FIR
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));  
#endif		
		return 0xFF;
	}
	return 0;
}
#endif /* #ifdef STK_CHK_REG */
static ssize_t stk_als_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);		
    int32_t reading;
#ifdef STK_POLL_ALS	
	reading = ps_data->als_code_last;
#else
    reading = stk3x3x_get_als_reading(ps_data);
#endif	
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}


static ssize_t stk_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    int32_t ret;
	
	ret = stk3x3x_get_state(ps_data);
	if(ret < 0)
		return ret;		
    ret = (ret & STK_STATE_EN_ALS_MASK)?1:0;
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static ssize_t stk_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data = dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}	
    printk(KERN_INFO "%s: Enable ALS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
    stk3x3x_enable_als(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t stk_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data = dev_get_drvdata(dev);
    int32_t als_reading;
	uint32_t als_lux;
    als_reading = stk3x3x_get_als_reading(ps_data);    
	als_lux = stk_alscode2lux(ps_data, als_reading);
    return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t stk_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
    ps_data->als_lux_last = value;
	input_report_abs(ps_data->input, ABS_MISC, value);
	input_sync(ps_data->input);
	printk(KERN_INFO "%s: als input event %ld lux\n",__func__, value);	

    return size;
}


static ssize_t stk_als_transmittance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
    int32_t transmittance;
    transmittance = ps_data->als_transmittance;
    return scnprintf(buf, PAGE_SIZE, "%d\n", transmittance);
}


static ssize_t stk_als_transmittance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    ps_data->als_transmittance = value;
    return size;
}

static ssize_t stk_als_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	int64_t delay;
	mutex_lock(&ps_data->io_lock);
	delay = ktime_to_ns(ps_data->als_poll_delay);
	mutex_unlock(&ps_data->io_lock);
	return scnprintf(buf, PAGE_SIZE, "%lld\n", delay);
}


static ssize_t stk_als_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint64_t value = 0;
	int ret;	
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	ret = kstrtoull(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoull failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}	
#ifdef STK_DEBUG_PRINTF		
	printk(KERN_INFO "%s: set als poll delay=%lld\n", __func__, value);
#endif	
	if(value < MIN_ALS_POLL_DELAY_NS)	
	{
		printk(KERN_ERR "%s: delay is too small\n", __func__);
		value = MIN_ALS_POLL_DELAY_NS;
	}
	mutex_lock(&ps_data->io_lock);
	if(value != ktime_to_ns(ps_data->als_poll_delay))
		ps_data->als_poll_delay = ns_to_ktime(value);	
#ifdef STK_ALS_FIR		
	ps_data->fir.number = 0;
	ps_data->fir.idx = 0;
	ps_data->fir.sum = 0;
#endif	
	mutex_unlock(&ps_data->io_lock);
	return size;
}



#ifdef STK_ALS_FIR
static ssize_t stk_als_firlen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	int len = atomic_read(&ps_data->firlength);
	
	printk(KERN_INFO "%s: len = %2d, idx = %2d\n", __func__, len, ps_data->fir.idx);			
	printk(KERN_INFO "%s: sum = %5d, ave = %5d\n", __func__, ps_data->fir.sum, ps_data->fir.sum/len);
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", len);		
}


static ssize_t stk_als_firlen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint64_t value = 0;	
	int ret;	
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	ret = kstrtoull(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoull failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}	
	
	if(value > MAX_FIR_LEN)
	{
		printk(KERN_ERR "%s: firlen exceed maximum filter length\n", __func__);
	}
	else if (value < 1)
	{
		atomic_set(&ps_data->firlength, 1);
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	}
	else
	{ 
		atomic_set(&ps_data->firlength, value);
		memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));
	}	
	return size;	
}
#endif  /* #ifdef STK_ALS_FIR */


static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
    uint32_t reading;
    reading = stk3x3x_get_ps_reading(ps_data);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);

	ret = stk3x3x_get_state(ps_data);
	if(ret < 0)
		return ret;	
    ret = (ret & STK_STATE_EN_PS_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
    printk(KERN_INFO "%s: Enable PS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
    stk3x3x_enable_ps(ps_data, en, 1);
    mutex_unlock(&ps_data->io_lock);
    return size;
}



static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    int32_t word_data;		
	unsigned char value[2];
	int ret;
	
	ret = stk3x3x_i2c_read_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];					
		
	return scnprintf(buf, PAGE_SIZE, "%d\n", word_data);
}
 
static ssize_t stk_ps_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long offset = 0;
	int ret;
	unsigned char val[2];
	
	ret = kstrtoul(buf, 10, &offset);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	if(offset > 65535)
	{
		printk(KERN_ERR "%s: invalid value, offset=%ld\n", __func__, offset);
		return -EINVAL;
	}
	
	val[0] = (offset & 0xFF00) >> 8;
	val[1] = offset & 0x00FF;
	ret = stk3x3x_i2c_write_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, val);	
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	
	return size;
}


static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);
    int32_t dist=1, ret;

    //ret = stk3x3x_get_flag(ps_data);
    ret = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (16));
	if(ret < 0) {
		return ret;
	}
    //dist = (ret & STK_FLG_NF_MASK)?1:0;	
	dist = ret&0x01;
    ps_data->ps_distance_last = dist;
	input_report_abs(ps_data->input, ABS_DISTANCE, dist);
	input_sync(ps_data->input);
	__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ); 
	printk(KERN_INFO "%s: ps input event %d cm\n",__func__, dist);		
    return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
    ps_data->ps_distance_last = value;
	input_report_abs(ps_data->input, ABS_DISTANCE, value);
	input_sync(ps_data->input);
	__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ); 
	printk(KERN_INFO "%s: ps input event %ld cm\n",__func__, value);	
    return size;
}


static ssize_t stk_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
    ps_thd_l1_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL1_PS_REG);
    if(ps_thd_l1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l1_reg);		
		return -EINVAL;		
	}
    ps_thd_l2_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL2_PS_REG);
    if(ps_thd_l2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l2_reg);		
		return -EINVAL;		
	}
	ps_thd_l1_reg = ps_thd_l1_reg<<8 | ps_thd_l2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_l1_reg);
}


static ssize_t stk_ps_code_thd_l_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    stk3x3x_set_ps_thd_l(ps_data, value);
    return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
    ps_thd_h1_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH1_PS_REG);
    if(ps_thd_h1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h1_reg);		
		return -EINVAL;		
	}
    ps_thd_h2_reg = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH2_PS_REG);
    if(ps_thd_h2_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h2_reg);		
		return -EINVAL;		
	}
	ps_thd_h1_reg = ps_thd_h1_reg<<8 | ps_thd_h2_reg;
    return scnprintf(buf, PAGE_SIZE, "%d\n", ps_thd_h1_reg);
}


static ssize_t stk_ps_code_thd_h_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    stk3x3x_set_ps_thd_h(ps_data, value);
    return size;
}


static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[0x22];
	uint8_t cnt;
	int len = 0;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	
	for(cnt=0;cnt<0x20;cnt++)
	{
		ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
			len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);	
	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk(KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);

	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, 0xE0);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk(KERN_INFO "reg[0xE0]=0x%2X\n", ps_reg[cnt]);	
	len += scnprintf(buf+len, PAGE_SIZE-len, "[3E]%2X,[3F]%2X,[E0]%2X\n", ps_reg[cnt-2], ps_reg[cnt-1], ps_reg[cnt]);
	return len;
/*
    return scnprintf(buf, PAGE_SIZE, "[0]%2X [1]%2X [2]%2X [3]%2X [4]%2X [5]%2X [6/7 HTHD]%2X,%2X [8/9 LTHD]%2X, %2X [A]%2X [B]%2X [C]%2X [D]%2X [E/F Aoff]%2X,%2X,[10]%2X [11/12 PS]%2X,%2X [13]%2X [14]%2X [15/16 Foff]%2X,%2X [17]%2X [18]%2X [3E]%2X [3F]%2X\n", 	
		ps_reg[0], ps_reg[1], ps_reg[2], ps_reg[3], ps_reg[4], ps_reg[5], ps_reg[6], ps_reg[7], ps_reg[8], 
		ps_reg[9], ps_reg[10], ps_reg[11], ps_reg[12], ps_reg[13], ps_reg[14], ps_reg[15], ps_reg[16], ps_reg[17], 
		ps_reg[18], ps_reg[19], ps_reg[20], ps_reg[21], ps_reg[22], ps_reg[23], ps_reg[24], ps_reg[25], ps_reg[26]);
		*/
}

static ssize_t stk_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[27];
	uint8_t cnt;
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	for(cnt=0;cnt<25;cnt++)
	{
		ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			printk(KERN_INFO "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);	
	cnt++;
	ps_reg[cnt] = stk3x3x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);		

    return scnprintf(buf, PAGE_SIZE, "[PS=%2X] [ALS=%2X] [WAIT=0x%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n", 
		ps_reg[0]&0x01,(ps_reg[0]&0x02)>>1,((ps_reg[0]&0x04)>>2)*ps_reg[5]*6,(ps_reg[0]&0x20)>>5,
		(ps_reg[0]&0x40)>>6,ps_reg[16]&0x01,(ps_reg[16]&0x04)>>2,(ps_reg[16]&0x10)>>4,(ps_reg[16]&0x20)>>5);		
}

static ssize_t stk_recv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ps_data->recv_reg));     		
}


static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
	int ret;
	int32_t recv_data;	
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	
	if((ret = kstrtoul(buf, 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	recv_data = stk3x3x_i2c_smbus_read_byte_data(ps_data->client,value);
//	printk("%s: reg 0x%x=0x%x\n", __func__, (int)value, recv_data);
	atomic_set(&ps_data->recv_reg, recv_data);
	return size;
}


static ssize_t stk_send_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}


static ssize_t stk_send_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int addr, cmd;
	int32_t ret, i;
	char *token[10];
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");
	if((ret = kstrtoul(token[0], 16, (unsigned long *)&(addr))) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	if((ret = kstrtoul(token[1], 16, (unsigned long *)&(cmd))) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	printk(KERN_INFO "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);		

	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, (unsigned char)addr, (unsigned char)cmd);
	if (0 != ret)
	{	
		printk(KERN_ERR "%s: stk3x3x_i2c_smbus_write_byte_data fail\n", __func__);
		return ret;
	}
	
	return size;
}

#ifdef STK_TUNE0

static ssize_t stk_ps_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	int32_t word_data;	
	unsigned char value[2];
	int ret;
	
	ret = stk3x3x_i2c_read_data(ps_data->client, 0x20, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];	
	
	ret = stk3x3x_i2c_read_data(ps_data->client, 0x22, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	word_data += ((value[0]<<8) | value[1]);		

	printk("%s: psi_set=%d, psa=%d,psi=%d, word_data=%d\n", __func__, 
		ps_data->psi_set, ps_data->psa, ps_data->psi, word_data);	
#ifdef CALI_PS_EVERY_TIME
	printk("%s: boot HT=%d, LT=%d\n", __func__, ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
#endif	
	return 0;
}

static ssize_t stk_ps_maxdiff_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
    unsigned long value = 0;
	int ret;
	
	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	ps_data->stk_max_min_diff = (int) value;
	return size;
}


static ssize_t stk_ps_maxdiff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_max_min_diff);     		
}

static ssize_t stk_ps_ltnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
    unsigned long value = 0;
	int ret;
	
	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	ps_data->stk_lt_n_ct = (int) value;
	return size;
}

static ssize_t stk_ps_ltnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_lt_n_ct);     		
}

static ssize_t stk_ps_htnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
    unsigned long value = 0;
	int ret;
	
	if((ret = kstrtoul(buf, 10, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	ps_data->stk_ht_n_ct = (int) value;
	return size;
}

static ssize_t stk_ps_htnct_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3x3x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_ht_n_ct);     		
}
#endif	/* #ifdef STK_TUNE0 */

static struct device_attribute als_enable_attribute = __ATTR(asl_enable,0644,stk_als_enable_show,stk_als_enable_store);
static struct device_attribute als_lux_attribute = __ATTR(asl_lux,0644,stk_als_lux_show,stk_als_lux_store);
static struct device_attribute als_code_attribute = __ATTR(asl_code, 0644, stk_als_code_show, NULL);
static struct device_attribute als_transmittance_attribute = __ATTR(asl_transmittance,0644,stk_als_transmittance_show,stk_als_transmittance_store);
static struct device_attribute als_poll_delay_attribute = __ATTR(asl_delay,0644,stk_als_delay_show,stk_als_delay_store);
//static struct device_attribute als_ir_code_attribute = __ATTR(asl_ircode,0644,stk_als_ir_code_show,NULL);
#ifdef STK_ALS_FIR
static struct device_attribute als_firlen_attribute = __ATTR(asl_firlen,0644,stk_als_firlen_show,stk_als_firlen_store);
#endif
/*
static struct attribute *stk_als_attrs [] =
{
	&als_enable_attribute.attr,
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
	&als_poll_delay_attribute.attr,
	&als_ir_code_attribute.attr,
#ifdef STK_ALS_FIR
	&als_firlen_attribute.attr,
#endif	
    NULL
};

static struct attribute_group stk_als_attribute_group = {
	.name = "driver",
	.attrs = stk_als_attrs,
};
*/


static struct device_attribute ps_enable_attribute = __ATTR(enable,0644,stk_ps_enable_show,stk_ps_enable_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0644,stk_ps_distance_show, stk_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(offset,0644,stk_ps_offset_show, stk_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(code, 0644, stk_ps_code_show, NULL);
static struct device_attribute ps_code_thd_l_attribute = __ATTR(codethdl,0644,stk_ps_code_thd_l_show,stk_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(codethdh,0644,stk_ps_code_thd_h_show,stk_ps_code_thd_h_store);
static struct device_attribute ps_recv_attribute = __ATTR(recv,0644,stk_recv_show,stk_recv_store);
static struct device_attribute ps_send_attribute = __ATTR(send,0644,stk_send_show, stk_send_store);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0644, stk_all_reg_show, NULL);
static struct device_attribute status_attribute = __ATTR(status, 0644, stk_status_show, NULL);
#ifdef STK_TUNE0
static struct device_attribute ps_cali_attribute = __ATTR(cali,0644,stk_ps_cali_show, NULL);
static struct device_attribute ps_maxdiff_attribute = __ATTR(maxdiff,0644,stk_ps_maxdiff_show, stk_ps_maxdiff_store);
static struct device_attribute ps_ltnct_attribute = __ATTR(ltnct,0644,stk_ps_ltnct_show, stk_ps_ltnct_store);
static struct device_attribute ps_htnct_attribute = __ATTR(htnct,0644,stk_ps_htnct_show, stk_ps_htnct_store);
#endif

static struct attribute *stk_ps_attrs [] =
{
    &ps_enable_attribute.attr,

    &ps_distance_attribute.attr,
	&ps_offset_attribute.attr,
    &ps_code_attribute.attr,
	&ps_code_thd_l_attribute.attr,
	&ps_code_thd_h_attribute.attr,	
	&ps_recv_attribute.attr,
	&ps_send_attribute.attr,	
	&all_reg_attribute.attr,
	&status_attribute.attr,
#ifdef STK_TUNE0
	&ps_cali_attribute.attr,
	&ps_maxdiff_attribute.attr,
	&ps_ltnct_attribute.attr,
	&ps_htnct_attribute.attr,
#endif	

	&als_enable_attribute.attr,
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
	&als_poll_delay_attribute.attr,
	//&als_ir_code_attribute.attr,
#ifdef STK_ALS_FIR
	&als_firlen_attribute.attr,
#endif	

    NULL
};

static struct attribute_group stk_ps_attribute_group = {
	.name = "driver",	
	.attrs = stk_ps_attrs,
};

static int stk_ps_val(struct stk3x3x_data *ps_data)
{
	u8 ps_invalid_flag;
	u8 bgir_raw_data[4];
	int ret;


	ret = stk3x3x_i2c_read_data(ps_data->client, 0xA7, 1, &ps_invalid_flag);
	if (ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	
	ret = stk3x3x_i2c_read_data(ps_data->client, 0x34, 4, bgir_raw_data);
	if (ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	
	if (((ps_invalid_flag >> 5) & 0x1) || ((bgir_raw_data[0] & 0x7f) >= 100) ||
		((bgir_raw_data[1] & 0x7f) >= 100) || ((bgir_raw_data[2] & 0x7f) >= 100) || ((bgir_raw_data[3] & 0x7f) >= 100))
	{
		return -1;
	}
	
	return 0;
}	

#ifdef STK_TUNE0	

static int stk_ps_tune_zero_final(struct stk3x3x_data *ps_data)
{
	int ret;
	
	ps_data->tune_zero_init_proc = false;
	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	
	
	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, 0);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
	if(ps_data->data_count == -1)
	{
		printk(KERN_INFO "%s: exceed limit\n", __func__);
		hrtimer_cancel(&ps_data->ps_tune0_timer);	
		return 0;
	}
	
	ps_data->psa = ps_data->ps_stat_data[0];
	ps_data->psi = ps_data->ps_stat_data[2];							

#ifdef CALI_PS_EVERY_TIME
	ps_data->ps_high_thd_boot = ps_data->ps_stat_data[1] + ps_data->stk_ht_n_ct*3;
	ps_data->ps_low_thd_boot = ps_data->ps_stat_data[1] + ps_data->stk_lt_n_ct*3;
	ps_data->ps_thd_h = ps_data->ps_high_thd_boot ;
	ps_data->ps_thd_l = ps_data->ps_low_thd_boot ;		
#else						
	ps_data->ps_thd_h = ps_data->ps_stat_data[1] + ps_data->stk_ht_n_ct;
	ps_data->ps_thd_l = ps_data->ps_stat_data[1] + ps_data->stk_lt_n_ct;			
#endif
	stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);				
	printk(KERN_INFO "%s: set HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h,  ps_data->ps_thd_l);		
	hrtimer_cancel(&ps_data->ps_tune0_timer);					
	return 0;
}
	
static int32_t stk_tune_zero_get_ps_data(struct stk3x3x_data *ps_data)
{
	uint32_t ps_adc;
	int ret;
	
	ret = stk_ps_val(ps_data);	
	if(ret != 0)
	{		
		ps_data->data_count = -1;
		stk_ps_tune_zero_final(ps_data);
		return 0;
	}	
	
	ps_adc = stk3x3x_get_ps_reading(ps_data);
	printk(KERN_INFO "%s: ps_adc #%d=%d\n", __func__, ps_data->data_count, ps_adc);
	if(ps_adc < 0)		
		return ps_adc;		
	
	ps_data->ps_stat_data[1]  +=  ps_adc;			
	if(ps_adc > ps_data->ps_stat_data[0])
		ps_data->ps_stat_data[0] = ps_adc;
	if(ps_adc < ps_data->ps_stat_data[2])
		ps_data->ps_stat_data[2] = ps_adc;						
	ps_data->data_count++;	
	
	if(ps_data->data_count == 5)
	{
		ps_data->ps_stat_data[1]  /= ps_data->data_count;			
		stk_ps_tune_zero_final(ps_data);
	}		
	
	return 0;
}

static int stk_ps_tune_zero_init(struct stk3x3x_data *ps_data)
{
	int32_t ret = 0;
	uint8_t w_state_reg;	
	
	ps_data->psi_set = 0;	
	ps_data->ps_stat_data[0] = 0;
	ps_data->ps_stat_data[2] = 9999;
	ps_data->ps_stat_data[1] = 0;
	ps_data->data_count = 0;
	
	ps_data->tune_zero_init_proc = true;		
	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, 0);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}			
	
	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);			
	ret = stk3x3x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);		
	return 0;	
}

static int stk_ps_tune_zero_func_fae(struct stk3x3x_data *ps_data)
{
	int32_t word_data;
	int ret, diff;
	unsigned char value[2];
			
#ifdef CALI_PS_EVERY_TIME
	if(!(ps_data->ps_enabled))
#else
	if(ps_data->psi_set || !(ps_data->ps_enabled))
#endif
	{
		return 0;
	}	

	ret = stk3x3x_get_flag(ps_data);
	if(ret < 0)
		return ret;
	if(!(ret&STK_FLG_PSDR_MASK))
	{
		//printk(KERN_INFO "%s: ps data is not ready yet\n", __func__);
		return 0;
	}
	
	ret = stk_ps_val(ps_data);	
	if(ret == 0)
	{				
		ret = stk3x3x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
		if(ret < 0)
		{
			printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
			return ret;
		}
		word_data = (value[0]<<8) | value[1];						
		//printk(KERN_INFO "%s: word_data=%d\n", __func__, word_data);
		
		if(word_data == 0)
		{
			//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		
		if(word_data > ps_data->psa)
		{
			ps_data->psa = word_data;
			printk(KERN_INFO "%s: update psa: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);
		}
		if(word_data < ps_data->psi)
		{
			ps_data->psi = word_data;	
			printk(KERN_INFO "%s: update psi: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);	
		}	
	}	
	diff = ps_data->psa - ps_data->psi;
	if(diff > ps_data->stk_max_min_diff)
	{
		ps_data->psi_set = ps_data->psi;
		ps_data->ps_thd_h = ps_data->psi + ps_data->stk_ht_n_ct;
		ps_data->ps_thd_l = ps_data->psi + ps_data->stk_lt_n_ct;
		
		
#ifdef CALI_PS_EVERY_TIME
		if(ps_data->ps_thd_h > ps_data->ps_high_thd_boot)
		{
			ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
			ps_data->ps_low_thd_boot = ps_data->ps_thd_l;
			printk(KERN_INFO "%s: update boot HT=%d, LT=%d\n", __func__, ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
		}
#endif		
		
		stk3x3x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3x3x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: FAE tune0 psa-psi(%d) > STK_DIFF found\n", __func__, diff);
#endif					
		hrtimer_cancel(&ps_data->ps_tune0_timer);
	}
	
	return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_ps_tune0_work);		
	if(ps_data->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(ps_data);
	else
		stk_ps_tune_zero_func_fae(ps_data);
	return;
}	


static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *ps_data = container_of(timer, struct stk3x3x_data, ps_tune0_timer);
	queue_work(ps_data->stk_ps_tune0_wq, &ps_data->stk_ps_tune0_work);	
	hrtimer_forward_now(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
#endif

#ifdef STK_POLL_ALS
static enum hrtimer_restart stk_als_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *ps_data = container_of(timer, struct stk3x3x_data, als_timer);
	queue_work(ps_data->stk_als_wq, &ps_data->stk_als_work);	
	hrtimer_forward_now(&ps_data->als_timer, ps_data->als_poll_delay);
	return HRTIMER_RESTART;	
	
}

static void stk_als_poll_work_func(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_als_work);	
	int32_t reading, reading_lux, flag_reg;
	uint32_t raw_data_u32;
	flag_reg = stk3x3x_get_flag(ps_data);
	if(flag_reg < 0)
		return;
	
	if(!(flag_reg&STK_FLG_ALSDR_MASK))
	{
		//printk(KERN_INFO "%s: als is not ready\n", __func__);
		return;
	}
	
	reading = stk3x3x_get_als_reading(ps_data);
	if(reading < 0)		
		return;
	
#if 0

	if (ps_data->c_code_last != 0)
	{
		if(reading < STK_IRC_MAX_ALS_CODE && reading > STK_IRC_MIN_ALS_CODE &&
		ps_data->c_code_last > STK_IRC_MIN_IR_CODE)
		{	
			ps_data->als_correct_factor = 217;
			als_comperator_A = reading * (20); //D55
			if((ps_data->c_code_last*1000) > als_comperator_A)
				ps_data->als_correct_factor = STK_IRC_ALS_CORREC_A;
		}
	}
	
	ps_data->c_code_last = 0;
			
#endif	
	reading = reading * ps_data->als_correct_factor / 1000;
	
	reading_lux = stk_alscode2lux(ps_data, reading);
	if(reading_lux < 10)
	{
		reading_lux = 0;
		ps_data->als_lux_last = reading_lux;
		input_report_abs(ps_data->input, ABS_MISC, reading_lux);
		input_sync(ps_data->input);
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: als input event %d lux\n",__func__, reading_lux);		
#endif		
	}
	else 
#ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
	if(abs(ps_data->als_lux_last - reading_lux) >= STK_ALS_CHANGE_THD)
#endif	/* #ifdef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD */
	{
		ps_data->als_lux_last = reading_lux;
		input_report_abs(ps_data->input, ABS_MISC, reading_lux);
		input_sync(ps_data->input);
#ifdef STK_DEBUG_PRINTF				
		printk(KERN_INFO "%s: als input event %d lux\n",__func__, reading_lux);		
#endif		
	}
	return;
}
#endif /* #ifdef STK_POLL_ALS */


#ifdef STK_POLL_PS	
static enum hrtimer_restart stk_ps_timer_func(struct hrtimer *timer)
{
	struct stk3x3x_data *ps_data = container_of(timer, struct stk3x3x_data, ps_timer);
	queue_work(ps_data->stk_ps_wq, &ps_data->stk_ps_work);
	hrtimer_forward_now(&ps_data->ps_timer, ps_data->ps_poll_delay);
	return HRTIMER_RESTART;		
}

static void stk_ps_poll_work_func(struct work_struct *work)
{
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_ps_work);	
	uint32_t reading;
	int32_t near_far_state;
    uint8_t org_flag_reg;	

	
	if(ps_data->ps_enabled)
	{
#ifdef STK_TUNE0
		// if(!(ps_data->psi_set))
			// return;	
#endif	
		org_flag_reg = stk3x3x_get_flag(ps_data);
		if(org_flag_reg < 0)
			goto err_i2c_rw;		

		if(!(org_flag_reg&STK_FLG_PSDR_MASK))
		{
			//printk(KERN_INFO "%s: ps is not ready\n", __func__);
			return;
		}	
				
		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;	
		reading = stk3x3x_get_ps_reading(ps_data);
		if(ps_data->ps_distance_last != near_far_state)
		{
			ps_data->ps_distance_last = near_far_state;
			input_report_abs(ps_data->input, ABS_DISTANCE, near_far_state);
			input_sync(ps_data->input);
			__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ);	
#ifdef STK_DEBUG_PRINTF		
			printk(KERN_INFO "%s: ps input event %d cm, ps code = %d\n",__func__, near_far_state, reading);		
#endif		
		}
		// ret = stk3x3x_set_flag(ps_data, org_flag_reg, disable_flag);		
		// if(ret < 0)
			// goto err_i2c_rw;
		return;
	}
	
	
err_i2c_rw:
	msleep(30);	
	return;
}
#endif

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
static void stk_work_func(struct work_struct *work)
{
	uint32_t reading;
#if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02))
    int32_t ret;
    uint8_t disable_flag = 0;
    uint8_t org_flag_reg;
#endif  /* #if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02)) */

#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD	
	uint32_t nLuxIndex;	
#endif
	struct stk3x3x_data *ps_data = container_of(work, struct stk3x3x_data, stk_work);	
	int32_t near_far_state;
	int32_t als_comperator;
#ifdef STK_GES				
	uint8_t disable_flag2 = 0, org_flag2_reg;
#endif	
	
#if (STK_INT_PS_MODE	== 0x03)
	near_far_state = gpio_get_value(ps_data->int_pin);
#elif	(STK_INT_PS_MODE	== 0x02)
	near_far_state = !(gpio_get_value(ps_data->int_pin));
#endif	

#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))
	ps_data->ps_distance_last = near_far_state;
	input_report_abs(ps_data->input, ABS_DISTANCE, near_far_state);
	input_sync(ps_data->input);
	__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ);	 
	reading = stk3x3x_get_ps_reading(ps_data);
	#ifdef STK_DEBUG_PRINTF	
		printk(KERN_INFO "%s: ps input event %d cm, ps code = %d\n",__func__, near_far_state, reading);			
	#endif	
#else	/* mode 0x01*/	
	org_flag_reg = stk3x3x_get_flag(ps_data);
	if(org_flag_reg < 0)
		goto err_i2c_rw;	
	
    if (org_flag_reg & STK_FLG_ALSINT_MASK)
    {
			disable_flag |= STK_FLG_ALSINT_MASK;
			reading = stk3x3x_get_als_reading(ps_data);
		if(reading < 0)		
		{
			printk(KERN_ERR "%s: stk3x3x_get_als_reading fail, ret=%d", __func__, reading);
			goto err_i2c_rw;
		}			
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
        nLuxIndex = stk_get_lux_interval_index(reading);
        stk3x3x_set_als_thd_h(ps_data, code_threshold_table[nLuxIndex]);
        stk3x3x_set_als_thd_l(ps_data, code_threshold_table[nLuxIndex-1]);
#else
        stk_als_set_new_thd(ps_data, reading);
#endif //CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#if 0
		if(ps_data->ir_code)
		{
			if(reading < STK_IRC_MAX_ALS_CODE && reading > STK_IRC_MIN_ALS_CODE && 
			ps_data->ir_code > STK_IRC_MIN_IR_CODE)
			{
				als_comperator = reading * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
				if(ps_data->ir_code > als_comperator)
					ps_data->als_correct_factor = STK_IRC_ALS_CORREC;
				else
					ps_data->als_correct_factor = 1000;
			}
			printk(KERN_INFO "%s: als=%d, ir=%d, als_correct_factor=%d", __func__, reading, ps_data->ir_code, ps_data->als_correct_factor);
			ps_data->ir_code = 0;
		}	
#endif
		reading = reading * ps_data->als_correct_factor / 1000;

		ps_data->als_lux_last = stk_alscode2lux(ps_data, reading);
		input_report_abs(ps_data->input, ABS_MISC, ps_data->als_lux_last);
		input_sync(ps_data->input);
#ifdef STK_DEBUG_PRINTF		
		printk(KERN_INFO "%s: als input event %d lux\n",__func__, ps_data->als_lux_last);			
#endif		
    }
    if (org_flag_reg & STK_FLG_PSINT_MASK)
    {
			disable_flag |= STK_FLG_PSINT_MASK;
			near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;
			
			ps_data->ps_distance_last = near_far_state;
			input_report_abs(ps_data->input, ABS_DISTANCE, near_far_state);
			input_sync(ps_data->input);
			__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ);	 		
			reading = stk3x3x_get_ps_reading(ps_data);
			#ifdef STK_DEBUG_PRINTF		
				printk(KERN_INFO "%s: ps input event=%d, ps code = %d\n",__func__, near_far_state, reading);
			#endif			
		}
	
	if(disable_flag)	
	{	
		ret = stk3x3x_set_flag(ps_data, org_flag_reg, disable_flag);		
		if(ret < 0)
			goto err_i2c_rw;
	}
#endif	
	usleep_range(1000, 2000);
	//msleep(1);
    enable_irq(ps_data->irq);
	return;

err_i2c_rw:
	msleep(30);
	enable_irq(ps_data->irq);
	return;	
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stk3x3x_data *pData = data;
	disable_irq_nosync(irq);
	queue_work(pData->stk_wq,&pData->stk_work);
	//printk(KERN_INFO "%s: get irq\n", __FUNCTION__);
	return IRQ_HANDLED;
}
#endif	/*	#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))	*/

static int32_t stk3x3x_init_all_setting(struct i2c_client *client, struct stk3x3x_platform_data *plat_data)
{
	int32_t ret;
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);		
	
	ret = stk3x3x_software_reset(ps_data); 
	if(ret < 0)
		return ret;
	
	ret = stk3x3x_check_pid(ps_data);
	if(ret < 0)
		return ret;
	stk3x3x_proc_plat_data(ps_data, plat_data);
	ret = stk3x3x_init_all_reg(ps_data);
	if(ret < 0)
		return ret;	
		
	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;		
	ps_data->re_enable_als = false;
	ps_data->re_enable_ps = false;
	ps_data->ir_code = 0;
	ps_data->als_correct_factor = 1000;
	ps_data->first_boot = true;	
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
	stk_init_code_threshold_table(ps_data);
#endif			
#ifdef STK_TUNE0
	stk_ps_tune_zero_init(ps_data);
#endif	
#ifdef STK_ALS_FIR
	memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));  
	atomic_set(&ps_data->firlength, STK_FIR_LEN);	
#endif
	atomic_set(&ps_data->recv_reg, 0);  
	
	ps_data->ps_distance_last = 1;
	ps_data->als_code_last = 500;
    return 0;
}

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
static int stk3x3x_setup_irq(struct i2c_client *client)
{		
	int irq, err = -EIO;
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);


	irq = gpio_to_irq(ps_data->int_pin);
	
#ifdef STK_DEBUG_PRINTF	
	printk(KERN_INFO "%s: int pin #=%d, irq=%d\n",__func__, ps_data->int_pin, irq);	
#endif	
	if (irq <= 0)
	{
		printk(KERN_ERR "irq number is not specified, irq # = %d, int pin=%d\n",irq, ps_data->int_pin);
		return irq;
	}
	ps_data->irq = irq;	
	err = gpio_request(ps_data->int_pin,"stk-int");        
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, err);
		return err;
	}
	err = gpio_direction_input(ps_data->int_pin);
	if(err < 0)
	{
		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, err);
		return err;
	}		
	err = request_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_LOW|IRQF_NO_SUSPEND|IRQF_ONESHOT, DEVICE_NAME, ps_data);
	printk(KERN_INFO "%s: request_irq--IRQF_TRIGGER_LOW\n", __FUNCTION__);

	if (err < 0) 
	{
		printk(KERN_WARNING "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);		
		goto err_request_any_context_irq;
	}
	disable_irq(irq);
	return 0;
err_request_any_context_irq:	

	gpio_free(ps_data->int_pin);		

	return err;
}
#endif


static int stk3x3x_suspend(struct device *dev)
{
	return 0;	
}

static int stk3x3x_resume(struct device *dev)
{	
	return 0;	
}

static const struct dev_pm_ops stk3x3x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(stk3x3x_suspend, stk3x3x_resume)
};

static int stk3x3x_set_wq(struct stk3x3x_data *ps_data)
{
	uint8_t als_it = ps_data->alsctrl_reg & 0x0F;
	
#ifdef STK_POLL_ALS	
	ps_data->stk_als_wq = create_singlethread_workqueue("stk_als_wq");
	INIT_WORK(&ps_data->stk_als_work, stk_als_poll_work_func);
	hrtimer_init(&ps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	if(als_it == 0x8)
		ps_data->als_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);	
	else if(als_it == 0x9)
		ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	else if(als_it == 0xA)
		ps_data->als_poll_delay = ns_to_ktime(220 * NSEC_PER_MSEC);		
	else if(als_it == 0xB)
		ps_data->als_poll_delay = ns_to_ktime(440 * NSEC_PER_MSEC);		
	else if(als_it == 0xC)
		ps_data->als_poll_delay = ns_to_ktime(880 * NSEC_PER_MSEC);		
	else
	{
		ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);		
		printk(KERN_INFO "%s: unknown ALS_IT=%d, set als_poll_delay=110ms\n", __func__, als_it);
	}
	ps_data->als_timer.function = stk_als_timer_func;

#endif	

#ifdef STK_POLL_PS	
	ps_data->stk_ps_wq = create_singlethread_workqueue("stk_ps_wq");
	INIT_WORK(&ps_data->stk_ps_work, stk_ps_poll_work_func);
	hrtimer_init(&ps_data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	ps_data->ps_timer.function = stk_ps_timer_func;
#endif	

#ifdef STK_TUNE0
	ps_data->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&ps_data->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&ps_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	ps_data->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif

#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	ps_data->stk_wq = create_singlethread_workqueue("stk_wq");
	INIT_WORK(&ps_data->stk_work, stk_work_func);
#endif
	return 0;
}


static int stk3x3x_set_input_devices(struct stk3x3x_data *ps_data)
{
	int err;
/*
	ps_data->als_input_dev = input_allocate_device();
	if (ps_data->als_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate als device\n", __func__);
		err = -ENOMEM;
		return err;
	}
*/
	ps_data->input = input_allocate_device();
	if (ps_data->input==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);		
		err = -ENOMEM;
		return err;
	}
	//ps_data->als_input_dev->name = ALS_NAME;
	ps_data->input->name = PS_NAME;
	//set_bit(EV_ABS, ps_data->als_input_dev->evbit);
	set_bit(EV_ABS, ps_data->input->evbit);
	input_set_abs_params(ps_data->input, ABS_MISC, 0, stk_alscode2lux(ps_data, (1<<16)-1), 0, 0);
	input_set_abs_params(ps_data->input, ABS_DISTANCE, 0,1, 0, 0);
/*
	err = input_register_device(ps_data->als_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register als input device\n", __func__);		
		return err;
	}
*/
	err = input_register_device(ps_data->input);	
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps/als input device\n", __func__);	
		return err;
	}
/*
	err = sysfs_create_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
		return err;
	}
*/
	err = sysfs_create_group(&ps_data->input->dev.kobj, &stk_ps_attribute_group);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
		return err;
	}
	//input_set_drvdata(ps_data->als_input_dev, ps_data);
	input_set_drvdata(ps_data->input, ps_data);
	

	
	return 0;
}

//wangyang add
static struct stk3x3x_platform_data stk3x3x_pls_info={
.state_reg = 0x0,
/* disable all */
.psctrl_reg = 0x31, //0x71
/* ps_persistance=4, ps_gain=64X, PS_IT=0.391ms */
.alsctrl_reg = 0x31,
/* als_persistance=1, als_gain=64X, ALS_IT=50ms */
.ledctrl_reg = 0xFF,
/* 100mA IRDR, 64/64 LED duty */
.wait_reg = 0x20,
/* 50 ms */
.ps_thd_h =1700,
.ps_thd_l = 1500,
.transmittance = 500,
};

static int stk3x3x_pls_misc_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "%s\n", __func__);
	if (stk3x3x_pls_misc_opened)
		return -EBUSY;
	stk3x3x_pls_misc_opened = 1;
	return 0;
}

static int stk3x3x_pls_misc_release(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "%s", __func__);
	stk3x3x_pls_misc_opened = 0;
  mutex_lock(&stk3x3x_data_misc->io_lock);
  stk3x3x_enable_ps(stk3x3x_data_misc, 0, 1); /* 0: disable*/
  stk3x3x_enable_als(stk3x3x_data_misc, 0);   /* 0: disable*/
  mutex_unlock(&stk3x3x_data_misc->io_lock);
	
	return 0;
}

static long stk3x3x_pls_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;
	unsigned char data;
	
	printk(KERN_INFO "%s: cmd %d", __func__, _IOC_NR(cmd));

	//get ioctl parameter
	switch (cmd) {
	case LTR_IOCTL_SET_PFLAG:
	case LTR_IOCTL_SET_LFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag))) {
			return -EFAULT;
		}
		if (flag < 0 || flag > 1) {
			return -EINVAL;
		}
		printk(KERN_INFO "%s: set flag=%d", __func__, flag);
		break;
	default:
		break;
	} 

	//handle ioctl
	switch (cmd) {
	case LTR_IOCTL_GET_PFLAG:
		flag = stk3x3x_data_misc->ps_enabled? 1:0;
		break;
		
	case LTR_IOCTL_GET_LFLAG:
		flag = stk3x3x_data_misc->als_enabled? 1:0;
		break;

	case LTR_IOCTL_GET_DATA:
		break;
		
	case LTR_IOCTL_SET_PFLAG:
		//atomic_set(&p_flag, flag);	
		if(flag==1){
		  mutex_lock(&stk3x3x_data_misc->io_lock);
			stk3x3x_enable_ps(stk3x3x_data_misc, 1, 1); /* 0: enable*/
			mutex_unlock(&stk3x3x_data_misc->io_lock);
		}
		else if(flag==0) {
		  mutex_lock(&stk3x3x_data_misc->io_lock);
			stk3x3x_enable_ps(stk3x3x_data_misc, 0, 1); /* 0: disable*/
			mutex_unlock(&stk3x3x_data_misc->io_lock);
		}
		break;
		
	case LTR_IOCTL_SET_LFLAG:
		//atomic_set(&l_flag, flag);
		if(flag==1){
		  mutex_lock(&stk3x3x_data_misc->io_lock);
			stk3x3x_enable_als(stk3x3x_data_misc, 1);   /* 0: enable*/
			mutex_unlock(&stk3x3x_data_misc->io_lock);
		}
		else if(flag==0) {
		  mutex_lock(&stk3x3x_data_misc->io_lock);
			stk3x3x_enable_als(stk3x3x_data_misc, 0);   /* 0: disable*/
			mutex_unlock(&stk3x3x_data_misc->io_lock);
		}
		break;
		
	default:
		printk(KERN_INFO "%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		return -EINVAL;
	}

	//report ioctl
	switch (cmd) {
	case LTR_IOCTL_GET_PFLAG:
	case LTR_IOCTL_GET_LFLAG:
		if (copy_to_user(argp, &flag, sizeof(flag))) {
			return -EFAULT;
		}
		printk(KERN_INFO "%s: get flag=%d", __func__, flag);
		break;
		
	case LTR_IOCTL_GET_DATA:
		//DI_read_reg(this_client,AP3212C_PLS_REG_DATA,&data);
		if (copy_to_user(argp, &data, sizeof(data))) {
			return -EFAULT;
		}
		printk(KERN_INFO "%s: get data=%d", __func__, flag);
		break;
		
	default:
		break;
	}

	return 0;
	
}

static struct file_operations stk3x3x_pls_miscfops = {
	.owner				= THIS_MODULE,
	.open				= stk3x3x_pls_misc_open,
	.release			= stk3x3x_pls_misc_release,
	.unlocked_ioctl = stk3x3x_pls_misc_ioctl,	
};
static struct miscdevice stk3x3x_pls_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = STK3X3X_PLS_MISC_DEVICE,
	.fops = &stk3x3x_pls_miscfops,
};

static int stk3x3x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = -ENODEV ,  proximity_gpio_int = 0  ;
	struct stk3x3x_data *ps_data;
	struct stk3x3x_platform_data *plat_data;
	printk(KERN_INFO "%s: driver version = %s\n", __func__, DRIVER_VERSION);

	if (client->dev.of_node) {
		struct device_node *np = client->dev.of_node;
		#if defined(CONFIG_BOARD_B65)||defined(CONFIG_BOARD_B71)||defined(CONFIG_BOARD_B64)
		proximity_gpio_int = of_get_named_gpio(np, "irq_gpio_1", 0); 
		#elif defined(CONFIG_BOARD_C68)
		proximity_gpio_int = of_get_named_gpio(np, "irq_gpio_2", 0);
		#else
		proximity_gpio_int = of_get_named_gpio(np, "irq_gpio_3", 0);
		#endif

		if (proximity_gpio_int < 0) {
			printk("fail to get proximity irq\n");
			return err;
		}
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "%s: No Support for I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	ps_data = kzalloc(sizeof(struct stk3x3x_data),GFP_KERNEL);

	stk3x3x_data_misc = ps_data;

	if(!ps_data) {
		printk(KERN_ERR "%s: failed to allocate stk3x3x_data\n", __func__);
		return -ENOMEM;
	}
	ps_data->client = client;
	client->dev.platform_data=&stk3x3x_pls_info;
	i2c_set_clientdata(client,ps_data);
	
	err = stk3x3x_check_pid(ps_data);
	if(err < 0) {
		printk(KERN_INFO "%s: check_pid fail (%d)", __func__, err);
		kfree(ps_data);
		return err;	
	}
	
	mutex_init(&ps_data->io_lock);
	//wakeup_source_init(&ps_data->ps_wakelock,"stk_input_wakelock");
        ps_data->ps_wakelock  = wakeup_source_register(NULL, "stk_input_wakelock");
        if (!ps_data->ps_wakelock) {
                return -ENOMEM;
        }

#ifdef STK_POLL_PS			
	//wakeup_source_init(&ps_data->ps_nosuspend_wl,"stk_nosuspend_wakelock");
        ps_data->ps_nosuspend_wl  = wakeup_source_register(NULL, "stk_nosuspend_wakelock");
        if (!ps_data->ps_nosuspend_wl) {
                return -ENOMEM;
        }
#endif	


	if(client->dev.platform_data != NULL)		
	{

		plat_data = client->dev.platform_data;	
		ps_data->als_transmittance = plat_data->transmittance;			
		ps_data->int_pin = proximity_gpio_int ;		
		if(ps_data->als_transmittance == 0) {
			printk(KERN_ERR "%s: Please set als_transmittance in platform data\n", __func__);
			goto err_als_input_allocate;
		}
	} else {
		printk(KERN_ERR "%s: no stk3x3x platform data!\n", __func__);		
		goto err_als_input_allocate;
	}
	stk3x3x_set_wq(ps_data);	/*for poll: PS or ALS*/

	ps_data->stk_max_min_diff = STK_MAX_MIN_DIFF;
	ps_data->stk_lt_n_ct = STK_LT_N_CT;
	ps_data->stk_ht_n_ct = STK_HT_N_CT;
	err = stk3x3x_init_all_setting(client, plat_data);
	if(err < 0)
		goto err_init_all_setting;

	err = stk3x3x_set_input_devices(ps_data);
	if(err < 0)
		goto err_setup_input_device;
	
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	err = stk3x3x_setup_irq(client);
	if(err < 0)
		goto err_stk3x3x_setup_irq;
#endif
	
	device_init_wakeup(&client->dev, true);

	lsensor_name_create();//wangyang add 

  //misc_register device
  err = misc_register(&stk3x3x_pls_miscdevice);
  if (err) {
    printk(KERN_ERR "%s: stk3x3x_pls_miscdevice register failed\n", __func__);
		goto err_device_miscregister_failed;
	}

#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	enable_irq(ps_data->irq);
	printk("stk3x3x_probe: enable_irq--ps_data->irq\n");
#endif

	printk(KERN_INFO "%s: probe successfully\n", __func__);

	return 0;

err_device_miscregister_failed:
	//device_init_wakeup(&client->dev, false);
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
err_stk3x3x_setup_irq:
	free_irq(ps_data->irq, ps_data);

		gpio_free(ps_data->int_pin);	

#endif
err_setup_input_device:
	
	sysfs_remove_group(&ps_data->input->dev.kobj, &stk_ps_attribute_group);	
	//sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);	
	input_unregister_device(ps_data->input);		
	//input_unregister_device(ps_data->als_input_dev);	
	input_free_device(ps_data->input);	
	//input_free_device(ps_data->als_input_dev);
err_init_all_setting:	
#ifdef STK_POLL_ALS		
	hrtimer_try_to_cancel(&ps_data->als_timer);
	destroy_workqueue(ps_data->stk_als_wq);	
#endif	
#ifdef STK_TUNE0
	destroy_workqueue(ps_data->stk_ps_tune0_wq);	
#endif	
#ifdef STK_POLL_PS	
	hrtimer_try_to_cancel(&ps_data->ps_timer);	
	destroy_workqueue(ps_data->stk_ps_wq);	
#endif		
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	destroy_workqueue(ps_data->stk_wq);	
#endif		
err_als_input_allocate:
#ifdef STK_POLL_PS
    //wakeup_source_trash(&ps_data->ps_nosuspend_wl);	
      if (ps_data->ps_nosuspend_wl) {
                wakeup_source_unregister(ps_data->ps_nosuspend_wl);
                ps_data->ps_nosuspend_wl = NULL;
        }

#endif	
    //wakeup_source_trash(&ps_data->ps_wakelock);	
      if (ps_data->ps_wakelock) {
                wakeup_source_unregister(ps_data->ps_wakelock);
                ps_data->ps_wakelock = NULL;
        }
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
	
    return err;
}


static int stk3x3x_remove(struct i2c_client *client)
{
	struct stk3x3x_data *ps_data = i2c_get_clientdata(client);
	misc_deregister(&stk3x3x_pls_miscdevice);
	device_init_wakeup(&client->dev, false);
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	free_irq(ps_data->irq, ps_data);
		gpio_free(ps_data->int_pin);	
#endif	/* #if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS)) */	


	
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	destroy_workqueue(ps_data->stk_wq);	
#endif	
	sysfs_remove_group(&ps_data->input->dev.kobj, &stk_ps_attribute_group);	
	//sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);	
	input_unregister_device(ps_data->input);		
	//input_unregister_device(ps_data->als_input_dev);	
	input_free_device(ps_data->input);	
	//input_free_device(ps_data->als_input_dev);	
#ifdef STK_POLL_ALS		
	hrtimer_try_to_cancel(&ps_data->als_timer);	
	destroy_workqueue(ps_data->stk_als_wq);	
#endif	
#ifdef STK_TUNE0
	destroy_workqueue(ps_data->stk_ps_tune0_wq);	
#endif	
#ifdef STK_POLL_PS
	hrtimer_try_to_cancel(&ps_data->ps_timer);	
	destroy_workqueue(ps_data->stk_ps_wq);		
	//wakeup_source_trash(&ps_data->ps_nosuspend_wl);	
        if (ps_data->ps_nosuspend_wl) {
                wakeup_source_unregister(ps_data->ps_nosuspend_wl);
                ps_data->ps_nosuspend_wl = NULL;
        }

#endif	
	//wakeup_source_trash(&ps_data->ps_wakelock);	
        if (ps_data->ps_wakelock) {
                wakeup_source_unregister(ps_data->ps_wakelock);
                ps_data->ps_wakelock = NULL;
        }
    mutex_destroy(&ps_data->io_lock);
	kfree(ps_data);
	
    return 0;
}

static const struct of_device_id stk3x3x_match_table[] = {
		{.compatible = "stk3x3x,stk3x3x_pls",},
		{ },
};


static struct i2c_driver stk_ps_driver =
{
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,	
		.of_match_table = stk3x3x_match_table,		
	},
	.probe = stk3x3x_probe,
	.remove = stk3x3x_remove,
};





static int __init stk3x3x_init(void)
{
	int ret;
	printk("stk3x3x_init!\n");
	ret = i2c_add_driver(&stk_ps_driver);
	if(ret) {
		pr_err("%s: add i2c device error %d\n", __FUNCTION__, ret);
		i2c_del_driver(&stk_ps_driver);
		return ret;
	}
	printk("stk3x3x_init end!\n");
	return 0;
}

static void __exit stk3x3x_exit(void)
{
	printk("%s!\n", __FUNCTION__);
	i2c_del_driver(&stk_ps_driver);	
	//i2c_unregister_device(this_client);
}

late_initcall(stk3x3x_init);
module_exit(stk3x3x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3x3x Proximity Sensor driver");
MODULE_LICENSE("GPL");
