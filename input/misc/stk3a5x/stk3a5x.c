/*
 *  stk3a5x.c - Linux kernel modules for sensortek stk301x, stk321x, stk331x 
 *  , and stk3410 proximity/ambient light sensor
 *
 *  Copyright (C) 2012~2016 Lex Hsieh / sensortek <lex_hsieh@sensortek.com.tw>
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
//#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include   <linux/fs.h>   
#include  <asm/uaccess.h> 
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <linux/syscalls.h>		//for copy_from_user
#ifdef CONFIG_HAS_EARLYSUSPEND
//#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_PM_SLEEP
#if defined(CONFIG_ADF_SPRD)
#include <linux/notifier.h>
#include <video/adf_notifier.h>
#endif
#endif
#include <linux/miscdevice.h>
#include "prj/prj_config.h"
#define DRIVER_VERSION  "3.11.01"

/* Driver Settings */
#define CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
#define STK_ALS_CHANGE_THD	10	/* The threshold to trigger ALS interrupt, unit: lux */	
#define STK_INT_PS_MODE			1	/* 1, 2, or 3	*/
//#define STK_POLL_PS
#define STK_POLL_ALS		/* ALS interrupt is valid only when STK_INT_PS_MODE = 1	or 4*/
#define STK_TUNE0
//#define STK_TUNE1
#define CALI_PS_EVERY_TIME
#define STK_DEBUG_PRINTF
#define SPREADTRUM_PLATFORM
#define STK_ALS_FIR
// #define STK_IRS
//#define STK_CHK_REG
//#define STK_GES
//#define QUALCOMM_PLATFORM

//add arvin_liao at 20190417
#define STK_WAKE_UP_TIMEOUT
#ifdef STK_WAKE_UP_TIMEOUT
static struct wakeup_source *stk_psensor_timeout_wakelock;
#define STK_WAKELOCK_TIMEOUT_INT_MS 100
#define STK_WAKELOCK_INT_NAME "stk psensor timeout wakelock"
#endif

#ifdef QUALCOMM_PLATFORM
	#include <linux/sensors.h>
	#include <linux/regulator/consumer.h>	
	#define STK_QUALCOMM_POWER_CTRL
#endif

//#ifdef SPREADTRUM_PLATFORM
//	#include "stk3388.h"
//#else
	#include "stk3a5x.h"
//#endif

#ifdef STK_TUNE1
// If tune1 is enabled, ALS = polling mode and PS = interrupt mode
	#ifdef	STK_POLL_PS
		#undef STK_POLL_PS
	#endif
	#ifndef	STK_POLL_ALS
		#define 	STK_POLL_ALS
	#endif
#endif
#define CTTRACKING

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
#define STK_DATA1_OFFSET_REG 	0x1D
#define STK_DATA2_OFFSET_REG 	0x1E
#define STK_DATA1_IR_REG 		0x20
#define STK_DATA2_IR_REG 		0x21
#define STK_PDT_ID_REG 			0x3E
#define STK_RSRVD_REG 			0x3F
#define STK_SW_RESET_REG		0x80

#define STK_GSCTRL_REG			0x1A
#define STK_FLAG2_REG			0x1C

/* Define state reg */
#define STK_STATE_EN_IRS_SHIFT  	7
#define STK_STATE_EN_AK_SHIFT  	6
#define STK_STATE_EN_ASO_SHIFT  	5
#define STK_STATE_EN_IRO_SHIFT  	4
#define STK_STATE_EN_WAIT_SHIFT  	2
#define STK_STATE_EN_ALS_SHIFT  	1
#define STK_STATE_EN_PS_SHIFT  	0

#define STK_STATE_EN_IRS_MASK	0x80
#define STK_STATE_EN_AK_MASK	0x40
#define STK_STATE_EN_ASO_MASK	0x20
#define STK_STATE_EN_IRO_MASK	0x10
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
#define STK_LED_IRDR_SHIFT  		6
#define STK_LED_DT_SHIFT  		0

#define STK_LED_IRDR_MASK		0xC0
#define STK_LED_DT_MASK			0x3F
	
/* Define interrupt reg */
#define STK_INT_CTRL_SHIFT  		7
#define STK_INT_OUI_SHIFT  		4
#define STK_INT_ALS_SHIFT  		3
#define STK_INT_PS_SHIFT  			0

#define STK_INT_CTRL_MASK		0x80
#define STK_INT_OUI_MASK			0x10
#define STK_INT_ALS_MASK			0x08
#define STK_INT_PS_MASK			0x07

#define STK_INT_ALS				0x08

/* Define flag reg */
#define STK_FLG_ALSDR_SHIFT  		7
#define STK_FLG_PSDR_SHIFT  		6
#define STK_FLG_ALSINT_SHIFT  		5
#define STK_FLG_PSINT_SHIFT  		4
#define STK_FLG_OUI_SHIFT  		2
#define STK_FLG_IR_RDY_SHIFT  		1
#define STK_FLG_NF_SHIFT  		0

#define STK_FLG_ALSDR_MASK		0x80
#define STK_FLG_PSDR_MASK		0x40
#define STK_FLG_ALSINT_MASK		0x20
#define STK_FLG_PSINT_MASK		0x10
#define STK_FLG_OUI_MASK			0x04
#define STK_FLG_IR_RDY_MASK		0x02
#define STK_FLG_NF_MASK			0x01
	
/* Define flag2 reg */
#define STK_FLG2_INT_GS_SHIFT		6
#define STK_FLG2_GS10_SHIFT		5
#define STK_FLG2_GS01_SHIFT		4

#define STK_FLG2_INT_GS_MASK	0x40
#define STK_FLG2_GS10_MASK		0x20
#define STK_FLG2_GS01_MASK		0x10

/* misc define */
#define MIN_ALS_POLL_DELAY_NS	60000000

#ifdef STK_TUNE0
	#define STK_MAX_MIN_DIFF	100
	#define STK_LT_N_CT	230
	#define STK_HT_N_CT	270
#endif	/* #ifdef STK_TUNE0 */

#define STK_H_PS        2000
#define STK_H_LT        650
#define STK_H_HT        600

#ifdef STK_TUNE1
	#define STK_FIN_THD				(2000)
	#define STK_ALS_NEAR				(40)
	#define STK_ALS_FAR				(50)
	#define STK_PS_HIGHER_THDH		(STK_FIN_THD - 400)
	#define STK_PS_HIGHER_THDL		(STK_FIN_THD - 500)

	#define STK_PS_CT_LIMIT			STK_FIN_THD
	#define PS_NOISE					(35)
	#define STK_CT_UPDATE_DIFF		(40)

	#define STK_PS_VAL_BUF			(10)
	#define STK_INT_NEAR_SAMPLE_NO	(10)
	#define STK_TUNE1_INVALID_PS_LIMIT	(5)
#endif	

#define STK_IRC_MAX_ALS_CODE		20000
#define STK_IRC_MIN_ALS_CODE		25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		2		
#define STK_IRC_ALS_NUMERA		5
#define STK_IRC_ALS_CORREC		850

#define STK_IRS_IT_REDUCE			2
#define STK_ALS_READ_IRS_IT_REDUCE	5
#define STK_ALS_THRESHOLD			30

#define DEVICE_NAME		"stk3a5x_pls"
#ifdef QUALCOMM_PLATFORM
	#define ALS_NAME "alps_pxy"
#else
	#define ALS_NAME "proximity"
#endif
#define PS_NAME "proximity_null"

#ifdef STK_QUALCOMM_POWER_CTRL
	/* POWER SUPPLY VOLTAGE RANGE */
	#define STK3A5X_VDD_MIN_UV	2000000
	#define STK3A5X_VDD_MAX_UV	3300000
	#define STK3A5X_VIO_MIN_UV	1750000
	#define STK3A5X_VIO_MAX_UV	1950000
#endif


#define STK3310SA_PID		0x17
#define STK3311SA_PID		0x1E
#define STK3311WV_PID		0x1D
#define STK3311X_PID			0x12
#define STK33119_PID			0x11

#ifdef STK_DEBUG_PRINTF
#define PRINT_WARN(x...)  printk(KERN_EMERG "[STK3a5x] " x)
#define PRINT_ERR(format,x...)  printk(KERN_EMERG "[STK3a5x] func:%s line:%04d  info: " format, __func__, __LINE__, ## x)
#else
#define PRINT_WARN(x...)
#define PRINT_ERR(format,x...) 
#endif


#define STK3A5X_IRQ_GPIO_NUMBER             214	

#ifdef QUALCOMM_PLATFORM

static struct sensors_classdev sensors_light_cdev = {
	.name = "stk3a5x-light",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_LIGHT_HANDLE,
	.type = SENSOR_TYPE_LIGHT,
	.max_range = "6500",
	.resolution = "0.0625",
	.sensor_power = "0.09",
	.min_delay = 0,	/* us */
	.max_delay = 0,
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.flags = 2,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,	
};

static struct sensors_classdev sensors_proximity_cdev = {
	.name = "stk3a5x-proximity",
	.vendor = "Sensortek",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "1.0",
	.resolution = "1.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.max_delay = 0,	
	.delay_msec = 200,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.flags = 3,	
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
	.sensors_calibrate = NULL,
	.sensors_write_cal_params = NULL,
	.params = NULL,		
};
#endif

#ifdef SPREADTRUM_PLATFORM
static struct i2c_client *this_client;
static struct stk3a5x_platform_data stk3a5x_pfdata={
  .state_reg = 0x0,    
  .psctrl_reg = 0x33,   
  .alsctrl_reg = 0x11, 
  .ledctrl_reg = 0x60,  
  .wait_reg = 0x40,   
  .ps_thd_h = 6250, 
  .ps_thd_l = 6200, 
  .int_pin = STK3A5X_IRQ_GPIO_NUMBER, //GPIO_PROX_INT//sprd_3rdparty_gpio_pls_irq
  .transmittance = 1000,     
  .vcc_pin = 131,
  .en_pin = 137,
}; 
#endif

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

#ifdef STK_GES
union stk_ges_operation{
	uint8_t ops[4];
	struct {
		uint8_t rw_len_retry;
		uint8_t reg;
		uint8_t reg_value_retry_crit;
		uint8_t sleep_10ns;
	}action;
};

union stk_ges_operation stk_ges_op[10] =
{
	{.ops={0xc1, 0x24, 0, 0}},
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}},	
	{.ops={0, 0, 0, 0}}
};
#endif

#ifdef STK_TUNE1
enum ps_int_state
{
	STK_PSINT_NORM = 0,
	STK_PSINT_FIN_DET,
	STK_PSINT_CT_DET
};
#endif

struct stk3a5x_data {
	struct i2c_client *client;
	struct stk3a5x_platform_data *pdata;
#ifdef QUALCOMM_PLATFORM
	struct sensors_classdev als_cdev;
	struct sensors_classdev ps_cdev;
#endif
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
#ifdef CONFIG_HAS_EARLYSUSPEND
	//struct early_suspend stk_early_suspend;
#endif	
	uint16_t ps_thd_h;
	uint16_t ps_thd_l;
#ifdef CALI_PS_EVERY_TIME	
	uint16_t ps_high_thd_boot;
	uint16_t ps_low_thd_boot;
#endif	
	struct mutex io_lock;
	struct input_dev *ps_input_dev;
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
	struct input_dev *als_input_dev;
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

#ifdef STK_GES		
	struct input_dev *ges_input_dev;
	int ges_enabled;
	int re_enable_ges;	
	atomic_t gesture2;
#endif	
#ifdef STK_IRS
	int als_data_index;
#endif	
#ifdef STK_QUALCOMM_POWER_CTRL
	struct regulator *vdd;
	struct regulator *vio;
	bool power_enabled;
#endif	
	uint8_t pid;
	uint8_t	p_wv_r_bd_with_co;
	uint32_t als_code_last;
	bool als_en_hal;
#ifdef STK_TUNE1	
	enum ps_int_state stk_int_handle_state;
	int invalid_ps_cnt;
#endif
	uint32_t ps_code_last;
	uint8_t boot_cali;
	uint8_t	p_1x_r_bd_with_co;	
	uint8_t	p_19_r_bc;
	
#ifdef CTTRACKING
	bool ps_thd_update;
	uint16_t tracking_count;
#endif

	int		vcc_pin;
	int		en_pin;	
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

static int32_t stk3a5x_enable_ps(struct stk3a5x_data *ps_data, uint8_t enable);
static int32_t stk3a5x_enable_als(struct stk3a5x_data *ps_data, uint8_t enable);
static int32_t stk3a5x_set_ps_thd_l(struct stk3a5x_data *ps_data, uint16_t thd_l);
static int32_t stk3a5x_set_ps_thd_h(struct stk3a5x_data *ps_data, uint16_t thd_h);
static int32_t stk3a5x_set_als_thd_l(struct stk3a5x_data *ps_data, uint16_t thd_l);
static int32_t stk3a5x_set_als_thd_h(struct stk3a5x_data *ps_data, uint16_t thd_h);
static int32_t stk3a5x_get_ir_reading(struct stk3a5x_data *ps_data, int32_t als_it_reduce);
#ifdef STK_TUNE0
static int stk_ps_tune_zero_func_fae(struct stk3a5x_data *ps_data);
#endif
#ifdef STK_CHK_REG
static int stk3a5x_validate_n_handle(struct i2c_client *client);
#endif
static int stk_ps_val(struct stk3a5x_data *ps_data);
#ifdef STK_QUALCOMM_POWER_CTRL
static int stk3a5x_device_ctl(struct stk3a5x_data *ps_data, bool enable);
#endif

static int stk3a5x_i2c_read_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
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

static int stk3a5x_i2c_write_data(struct i2c_client *client, unsigned char command, int length, unsigned char *values)
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

static int stk3a5x_i2c_smbus_read_byte_data(struct i2c_client *client, unsigned char command)
{
	unsigned char value;
	int err;
	err = stk3a5x_i2c_read_data(client, command, 1, &value);
	if(err < 0)
		return err;
	return value;
}

static int stk3a5x_i2c_smbus_write_byte_data(struct i2c_client *client, unsigned char command, unsigned char value)
{
	int err;
	err = stk3a5x_i2c_write_data(client, command, 1, &value);
	return err;
}

uint32_t stk_alscode2lux(struct stk3a5x_data *ps_data, uint32_t alscode)
{
	alscode += ((alscode<<7)+(alscode<<3)+(alscode>>1));   
    alscode<<=3; 
    alscode/=ps_data->als_transmittance;
	return alscode;
}

uint32_t stk_lux2alscode(struct stk3a5x_data *ps_data, uint32_t lux)
{
    lux*=ps_data->als_transmittance;
    lux/=1100;
    if (unlikely(lux>=(1<<16)))
        lux = (1<<16) -1;
    return lux;
}

#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
static void stk_init_code_threshold_table(struct stk3a5x_data *ps_data)
{
    uint32_t i,j;
    uint32_t alscode;

    code_threshold_table[0] = 0;
#ifdef STK_DEBUG_PRINTF	
    PRINT_WARN( "alscode[0]=%d\n",0);
#endif	
    for (i=1,j=0;i<LUX_THD_TABLE_SIZE;i++,j++)
    {
        alscode = stk_lux2alscode(ps_data, lux_threshold_table[j]);
        PRINT_WARN( "alscode[%d]=%d\n",i,alscode);
        code_threshold_table[i] = (uint16_t)(alscode);
    }
    code_threshold_table[i] = 0xffff;
    PRINT_WARN( "alscode[%d]=%d\n",i,alscode);
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
void stk_als_set_new_thd(struct stk3a5x_data *ps_data, uint16_t alscode)
{
    int32_t high_thd,low_thd;
    high_thd = alscode + stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);
    low_thd = alscode - stk_lux2alscode(ps_data, STK_ALS_CHANGE_THD);
    if (high_thd >= (1<<16))
        high_thd = (1<<16) -1;
    if (low_thd <0)
        low_thd = 0;
    stk3a5x_set_als_thd_h(ps_data, (uint16_t)high_thd);
    stk3a5x_set_als_thd_l(ps_data, (uint16_t)low_thd);
}
#endif // CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD


static void stk3a5x_proc_plat_data(struct stk3a5x_data *ps_data, struct stk3a5x_platform_data *plat_data)
{
	uint8_t w_reg;
	
	ps_data->state_reg = plat_data->state_reg;
	ps_data->psctrl_reg = plat_data->psctrl_reg;
#ifdef STK_POLL_PS		
	ps_data->psctrl_reg &= 0x3F;
#endif		
	ps_data->alsctrl_reg = plat_data->alsctrl_reg;
	ps_data->ledctrl_reg = plat_data->ledctrl_reg;
	if(ps_data->pid == STK3310SA_PID || ps_data->pid == STK3311SA_PID)
		ps_data->ledctrl_reg &= 0x3F;
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
	#ifdef STK_TUNE1
	w_reg |= 0x07;
	#else
	w_reg |= STK_INT_PS_MODE;
	#endif
#else
	w_reg |= 0x01;		
#endif

#if (!defined(STK_POLL_ALS) && (STK_INT_PS_MODE != 0x02) && (STK_INT_PS_MODE != 0x03))
	w_reg |= STK_INT_ALS;
#endif
	ps_data->int_reg = w_reg;
	return;
}
static int32_t stk3a5x_reinit_all_reg(struct stk3a5x_data *ps_data)
{
	int32_t ret;
	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, ps_data->state_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, ps_data->psctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, ps_data->alsctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, ps_data->ledctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	


    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0x4E, 0x06);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xFA, 0x01);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xA0, 0x10);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xA1, 0x7F);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	

    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xF6, 0x82);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	//qzhu modify for AGCTRL
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xDB, 0x15);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}				
	stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);	
	stk3a5x_enable_ps(ps_data, 1);
	stk3a5x_enable_als(ps_data, 1);

	
	return 0;	
}

static int32_t stk3a5x_init_all_reg(struct stk3a5x_data *ps_data)
{
	int32_t ret;
	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, ps_data->state_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_PSCTRL_REG, ps_data->psctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, ps_data->alsctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }		
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_LEDCTRL_REG, ps_data->ledctrl_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return ret;
    }	
#ifdef STK_TUNE0	
	ps_data->psa = 0x0;
	ps_data->psi = 0xFFFF;	
#endif	
	stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);	

    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
	//qzhu modify for reduce light value ,default is 0x06
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0x4E, 0x00);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xFA, 0x01);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xA0, 0x10);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xA1, 0x7F);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	

    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xF6, 0x82);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}

	//qzhu modify for AGCTRL
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, 0xDB, 0x15);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}				

	return 0;	
}

static int stk3a5x_otp_read_byte_data(struct i2c_client *client, unsigned char command)
{
	unsigned char value;
	int err;
	
    err = stk3a5x_i2c_smbus_write_byte_data(client, 0x0, 0x2);
    if (err < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return err;
    }
	
	err = stk3a5x_i2c_smbus_write_byte_data(client, 0x90, command);
	if(err)
		return err;
	
	err = stk3a5x_i2c_smbus_write_byte_data(client, 0x92, 0x82);
	if(err)
		return err;
	
	usleep_range(2000, 4000);

	err = stk3a5x_i2c_smbus_read_byte_data(client, 0x91);
	if(err < 0)
		return err;
	value = err;
	printk("%s:read OTP 0x%x=0x%x", __func__, command, value);

    err = stk3a5x_i2c_smbus_write_byte_data(client, 0x0, 0x0);
    if (err < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
        return err;
    }	
	
	return value;	
}

static int32_t stk3a5x_check_pid(struct stk3a5x_data *ps_data)
{
	unsigned char value[3], pid_msb;
	int err;
	int otp25;
	
	ps_data->p_wv_r_bd_with_co = 0;
	ps_data->p_1x_r_bd_with_co = 0;
	ps_data->p_19_r_bc = 0;
	
	err = stk3a5x_i2c_read_data(ps_data->client, STK_PDT_ID_REG, 2, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
	err = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, 0xE0);
	if(err < 0)
		return err;
	value[2] = err;
	
	PRINT_WARN( "%s: PID=0x%x, RID=0x%x, 0x90=0x%x\n", __func__, value[0], value[1], value[2]);
	ps_data->pid = value[0];
	
	if(value[0] == STK3311WV_PID)
		ps_data->p_wv_r_bd_with_co |= 0b100;
	else if(value[0] == STK3311X_PID)
		ps_data->p_1x_r_bd_with_co |= 0b100;
	else if(value[0] == STK33119_PID)
		ps_data->p_19_r_bc |= 0b10;

	if(value[1] == 0xC3)
	{
		ps_data->p_wv_r_bd_with_co |= 0b010;
		ps_data->p_1x_r_bd_with_co |= 0b010;
	}
	else if(value[1] == 0xC2)
	{
		ps_data->p_19_r_bc |= 0b01;
	}
	
	err = stk3a5x_otp_read_byte_data(ps_data->client, 0x25);
	if(err < 0)
		return err;
	
	otp25 = err;
	if(otp25 & 0x80)
		ps_data->p_wv_r_bd_with_co |= 0b001;
	PRINT_WARN( "%s: p_wv_r_bd_with_co = 0x%x\n", __func__, ps_data->p_wv_r_bd_with_co);	

	if(otp25 & 0x40)
		ps_data->p_1x_r_bd_with_co |= 0b001;
	PRINT_WARN( "%s: p_1x_r_bd_with_co = 0x%x\n", __func__, ps_data->p_1x_r_bd_with_co);		
	
	PRINT_WARN( "%s: p_19_r_bc = 0x%x\n", __func__, ps_data->p_19_r_bc);		
	
	if(value[0] == 0)
	{
		printk(KERN_ERR "PID=0x0, please make sure the chip is stk3a5x!\n");
		return -2;			
	}
	
	pid_msb = value[0] & 0xF0;
	switch(pid_msb)
	{
	case 0x10:
	case 0x20:
	case 0x30:
	case 0x50:
		return 0;
	default:
		printk(KERN_ERR "%s: invalid PID(%#x)\n", __func__, value[0]);	
		return -1;
	}
	return 0;
}


static int32_t stk3a5x_software_reset(struct stk3a5x_data *ps_data)
{
    int32_t r;
    uint8_t w_reg;
	
    w_reg = 0x7F;
    r = stk3a5x_i2c_smbus_write_byte_data(ps_data->client,STK_WAIT_REG,w_reg);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: write i2c error, ret=%d\n", __func__, r);
        return r;
    }
    r = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_WAIT_REG);
    if (w_reg != r)
    {
        printk(KERN_ERR "%s: software reset: read-back value is not the same\n", __func__);
        return -1;
    }
	
    r = stk3a5x_i2c_smbus_write_byte_data(ps_data->client,STK_SW_RESET_REG,0);
    if (r<0)
    {
        printk(KERN_ERR "%s: software reset: read error after reset\n", __func__);
        return r;
    }
	usleep_range(30000, 50000);
    return 0;
}


static int32_t stk3a5x_set_als_thd_l(struct stk3a5x_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3a5x_i2c_write_data(ps_data->client, STK_THDL1_ALS_REG, 2, val);
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;		
}
static int32_t stk3a5x_set_als_thd_h(struct stk3a5x_data *ps_data, uint16_t thd_h)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3a5x_i2c_write_data(ps_data->client, STK_THDH1_ALS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;	
}

static int32_t stk3a5x_set_ps_thd_l(struct stk3a5x_data *ps_data, uint16_t thd_l)
{
	unsigned char val[2];
	int ret;
	val[0] = (thd_l & 0xFF00) >> 8;
	val[1] = thd_l & 0x00FF;
	ret = stk3a5x_i2c_write_data(ps_data->client, STK_THDL1_PS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;	
}
static int32_t stk3a5x_set_ps_thd_h(struct stk3a5x_data *ps_data, uint16_t thd_h)
{	
	unsigned char val[2];
	int ret;
	val[0] = (thd_h & 0xFF00) >> 8;
	val[1] = thd_h & 0x00FF;
	ret = stk3a5x_i2c_write_data(ps_data->client, STK_THDH1_PS_REG, 2, val);		
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);	
	return ret;
}

static uint32_t stk3a5x_get_ps_reading(struct stk3a5x_data *ps_data)
{	
	unsigned char value[2];
	int err;
	err = stk3a5x_i2c_read_data(ps_data->client, STK_DATA1_PS_REG, 2, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
	return ((value[0]<<8) | value[1]);	
}


static int32_t stk3a5x_set_flag(struct stk3a5x_data *ps_data, uint8_t org_flag_reg, uint8_t clr)
{
	uint8_t w_flag;
	int ret;
	
	w_flag = org_flag_reg | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
	w_flag &= (~clr);
	//PRINT_WARN( "%s: org_flag_reg=0x%x, w_flag = 0x%x\n", __func__, org_flag_reg, w_flag);		
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client,STK_FLAG_REG, w_flag);	
	if(ret < 0)
		printk(KERN_EMERG "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3a5x_get_flag(struct stk3a5x_data *ps_data)
{	
	int ret;
    ret = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_FLAG_REG);	
	if(ret < 0)
		printk(KERN_EMERG "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3a5x_set_state(struct stk3a5x_data *ps_data, uint8_t state)
{
	int ret;		
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client,STK_STATE_REG, state);	
	if(ret < 0)
		printk(KERN_EMERG "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3a5x_get_state(struct stk3a5x_data *ps_data)
{	
	int ret;
    ret = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);	
	if(ret < 0)
		printk(KERN_EMERG "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static void stk_ps_report(struct stk3a5x_data *ps_data, int nf)
{
#ifdef QUALCOMM_PLATFORM	
	ktime_t	timestamp = ktime_get_boottime();	
#endif
	
	ps_data->ps_distance_last = nf;
	mutex_lock(&ps_data->io_lock);
	input_report_abs(ps_data->als_input_dev, ABS_DISTANCE, nf);
#ifdef QUALCOMM_PLATFORM
	input_event(ps_data->als_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(ps_data->als_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
#endif			
	input_sync(ps_data->als_input_dev);
	mutex_unlock(&ps_data->io_lock);
	__pm_wakeup_event(ps_data->ps_wakelock, 3*HZ);
	printk(KERN_EMERG "%s: ps input event %d\n",__func__, nf);
}

static void stk_als_report(struct stk3a5x_data *ps_data, int als)
{
#ifdef QUALCOMM_PLATFORM		
	ktime_t	timestamp = ktime_get_boottime();
#endif

	ps_data->als_lux_last = als;
	mutex_lock(&ps_data->io_lock);
	input_report_abs(ps_data->als_input_dev, ABS_MISC, als);
#ifdef QUALCOMM_PLATFORM
	input_event(ps_data->als_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
	input_event(ps_data->als_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
#endif	
	input_sync(ps_data->als_input_dev);
	mutex_unlock(&ps_data->io_lock);	
#ifdef STK_DEBUG_PRINTF
	PRINT_WARN( "%s: als input event %d lux\n",__func__, als);
#endif
}

#ifdef STK_GES		
static int32_t stk3a5x_set_gsctrl(struct stk3a5x_data *ps_data, uint8_t state)
{
	int ret;		
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client,STK_GSCTRL_REG, state);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

static int32_t stk3a5x_get_gsctrl(struct stk3a5x_data *ps_data)
{	
	int ret;
    ret = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_GSCTRL_REG);	
	if(ret < 0)
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, ret);
	return ret;
}

#if 0
static uint32_t stk3a5x_get_ges_reading(struct stk3a5x_data *ps_data, unsigned int *ges0,
				unsigned int *ges1, unsigned int *ges2)
{
	int retry, len, no = 0;
	uint8_t reg_val_crit;
	int err;
	unsigned char value[10];
	
	while(stk_ges_op[no].ops[1] != 0)
	{
		retry = stk_ges_op[no].action.rw_len_retry & 0x0F;
		len = (stk_ges_op[no].action.rw_len_retry & 0x70) >> 4;
		reg_val_crit = stk_ges_op[no].action.reg_value_retry_crit;
		if(stk_ges_op[no].action.rw_len_retry & 0x80)
		{
			while(retry != 0)
			{
				err = stk3a5x_i2c_read_data(ps_data->client, 
								stk_ges_op[no].action.reg, len, value);								
				if(err < 0)
				{
					printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
					return err;
				}
				if(reg_val_crit)
				{
					if(value[0] & reg_val_crit)
						break;
				}
				if(stk_ges_op[no].action.sleep_10ns != 0)
					usleep_range(stk_ges_op[no].action.sleep_10ns*10, 
									stk_ges_op[no].action.sleep_10ns*10+300);
				retry--;
			}			
		}
		else
		{
			while(retry != 0)
			{
				err = stk3a5x_i2c_write_data(ps_data->client, stk_ges_op[no].action.reg, 
								len, &reg_val_crit);		
				if(err < 0)
				{
					printk(KERN_ERR "%s: fail, err=%d\n", __func__, err);			
					return err;
				}

				if(stk_ges_op[no].action.sleep_10ns != 0)
					usleep_range(stk_ges_op[no].action.sleep_10ns*10, 
									stk_ges_op[no].action.sleep_10ns*10+300);
				retry--;
			}			
		}
		
		if(stk_ges_op[no].action.reg == 0x24)
		{
			*ges0 = (value[0]<<8) | value[1];	
			*ges1 = (value[2]<<8) | value[3];
		}
		else if(stk_ges_op[no].action.reg == stk_ges_op[9].ops[0])
		{
			*ges2 = (value[0]<<8) | value[1];
		}
		
		no++;
	}
	return 0;
}

#else
static uint32_t stk3a5x_get_ges_reading(struct stk3a5x_data *ps_data, unsigned int *ges0,unsigned int *ges1,unsigned int *ges2)
{
	unsigned char value[4];
	int err, retry = 10;
	
	do {
		err = stk3a5x_get_flag(ps_data);	
		if(err < 0)
			return err;
		if(err & STK_FLG_PSDR_MASK)
			break;
		//PRINT_WARN( "ges: not ready, %d\n", retry);
		retry--;
		usleep_range(350, 1000);
	} while(retry > 0);
	err = stk3a5x_i2c_read_data(ps_data->client, STK_DATA1_PS_REG, 2, &value[0]);
	
	err = stk3a5x_i2c_read_data(ps_data->client, 0x24, 4, &value[0]);
	if(err < 0)
	{
		printk(KERN_ERR "%s: fail, ret=%d\n", __func__, err);
		return err;
	}
	*ges0 = (value[0]<<8) | value[1];	
	*ges1 = (value[2]<<8) | value[3];	
	//PRINT_WARN( "%s: ges=%d,%d\n",__func__, *ges0, *ges1);	
	return 0;
}
#endif

static int32_t stk3a5x_enable_ges(struct stk3a5x_data *ps_data, uint8_t enable, uint8_t mode)
{
    int32_t ret;
	uint8_t w_state_reg, gsctrl_reg;
	uint8_t org_mode = 0;
	
	if(ps_data->ps_enabled)
	{
		PRINT_WARN( "%s: since PS is enabled, ges is disabled\n", __func__);
		ps_data->re_enable_ges = enable;
		return 0;
	}	
		
	if(enable == ps_data->ges_enabled)
		return 0;	
		
	if(enable)
	{
#ifdef STK_QUALCOMM_POWER_CTRL	
		ret = stk3a5x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
#endif	
		if(ps_data->als_enabled) 
		{
			PRINT_WARN( "%s: force disable ALS\n", __func__);			
			stk3a5x_enable_als(ps_data, 0);
			ps_data->re_enable_als = true;
		}		
		
		ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, 0);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}			
		
		ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, 0);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}				
			
		w_state_reg = STK_STATE_EN_WAIT_MASK | STK_STATE_EN_PS_MASK; 
		ret = stk3a5x_set_state(ps_data, w_state_reg);
		if(ret < 0)
			return ret;
		ps_data->state_reg = w_state_reg;
		
		if(mode == 2)
		{
			ret = stk3a5x_get_gsctrl(ps_data);
			if(ret < 0)
				return ret;		
			gsctrl_reg = ret & 0xF3;
#ifdef STK_POLL_PS
			gsctrl_reg |= 0x04;
#else
			gsctrl_reg |= 0x0C;
#endif			
			ret = stk3a5x_set_gsctrl(ps_data, gsctrl_reg);
			if(ret < 0)
				return ret;			
#ifdef STK_POLL_PS
			hrtimer_start(&ps_data->ps_timer, ps_data->ps_poll_delay, HRTIMER_MODE_REL);	
#else
			enable_irq(ps_data->irq);
#endif		
		}
		ps_data->ges_enabled = mode;
	}
	else
	{
		org_mode = ps_data->ges_enabled;
		if(org_mode == 2)
		{
#ifdef STK_POLL_PS
			hrtimer_cancel(&ps_data->ps_timer);
			cancel_work_sync(&ps_data->stk_ps_work);
#else	
			disable_irq(ps_data->irq);
#endif		
		}
	
		ret = stk3a5x_set_state(ps_data, 0);
		if(ret < 0)
			return ret;		
		ps_data->state_reg = 0;
		
		ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_WAIT_REG, ps_data->wait_reg);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}	
		
		ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: write i2c error\n", __func__);
			return ret;
		}				
		if(org_mode == 2)
		{
			ret = stk3a5x_get_gsctrl(ps_data);
			if(ret < 0)
				return ret;		
			gsctrl_reg = ret & (~0x0C);
			ret = stk3a5x_set_gsctrl(ps_data, gsctrl_reg);		
			if(ret < 0)
				return ret;		
		}
		ps_data->ges_enabled = 0;
		if(ps_data->re_enable_als) 
		{
			PRINT_WARN( "%s: re-enable ALS\n", __func__);
			stk3a5x_enable_als(ps_data, 1);
			ps_data->re_enable_als = false;
		}
#ifdef STK_QUALCOMM_POWER_CTRL	
		ret = stk3a5x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
#endif			
	}	

	return 0;
}
#endif /* #ifdef STK_GES */

static int32_t stk3a5x_enable_ps(struct stk3a5x_data *ps_data, uint8_t enable)
{
    int32_t ret = 0;
	uint8_t w_state_reg;
	uint8_t curr_ps_enable;	
	uint32_t reading;
	int32_t near_far_state;		
	
#ifdef STK_QUALCOMM_POWER_CTRL		
	if (enable) {
		ret = stk3a5x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}	
#endif
	
#ifdef STK_CHK_REG
	if(validate_reg)
	{
		ret = stk3a5x_validate_n_handle(ps_data->client);
		if(ret < 0)	
			printk(KERN_EMERG "stk3a5x_validate_n_handle fail: %d\n", ret); 
	}			
#endif /* #ifdef STK_CHK_REG */	

#ifdef STK_GES		
	if(ps_data->ges_enabled && enable)
	{
		PRINT_WARN( "%s: force disable ges\n", __func__);
		stk3a5x_enable_ges(ps_data, 0, 1);
		ps_data->re_enable_ges = 1;
	}
#endif
	curr_ps_enable = ps_data->ps_enabled?1:0;	
	if(curr_ps_enable == enable)
		return 0;
	
#ifdef STK_TUNE0
#ifndef CTTRACKING
	if (!(ps_data->psi_set) && !enable)
	{
		hrtimer_cancel(&ps_data->ps_tune0_timer);					
		cancel_work_sync(&ps_data->stk_ps_tune0_work);
	}
#endif
#endif		
	if(ps_data->first_boot == true)
	{
		ps_data->first_boot = false;
	}
    if(enable)
	{
		stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);		
	}

	ret = stk3a5x_get_state(ps_data);
	if(ret < 0){
		printk(KERN_EMERG "stk3a5x_get_state:%d\n",ret);
		return ret;
	}
	w_state_reg = ret;
	
	
	w_state_reg &= ~(STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK | STK_STATE_EN_AK_MASK); 
	if(enable)	
	{
		w_state_reg |= STK_STATE_EN_PS_MASK;	
		if(!(ps_data->als_enabled))
			w_state_reg |= STK_STATE_EN_WAIT_MASK;			
	}	
	ret = stk3a5x_set_state(ps_data, w_state_reg);
	if(ret < 0){
		printk(KERN_EMERG "stk3a5x_set_state:%d %d\n",w_state_reg,ret);
		return ret;
	}
	ps_data->state_reg = w_state_reg;
		
    if(enable)
	{
#ifdef STK_TUNE0
	#ifdef CALI_PS_EVERY_TIME
		ps_data->psi_set = 0;
		ps_data->psa = 0;
		ps_data->psi = 0xFFFF;
		#if (!defined(STK_TUNE1) && !defined(QUALCOMM_PLATFORM))
		ps_data->ps_thd_h = ps_data->ps_high_thd_boot;
		ps_data->ps_thd_l = ps_data->ps_low_thd_boot;
#ifdef CTTRACKING
		ps_data->ps_thd_update = false;
		ps_data->tracking_count = 0;
		ps_data->data_count = 0;	
#endif
		#endif
		hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);					
	#else
		if (!(ps_data->psi_set))
			hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);			
	#endif	/* #ifdef CALI_PS_EVERY_TIME */
#endif

		{
			usleep_range(4000, 5000);
			reading = stk3a5x_get_ps_reading(ps_data);			
			if (reading < 0)
				return reading;	
#ifdef STK_TUNE1
			near_far_state = (reading >= ps_data->ps_thd_h)?0:1;
#else			
			ret = stk3a5x_get_flag(ps_data);
			if (ret < 0){
				printk(KERN_EMERG "stk3a5x_get_state:%d %d\n",w_state_reg,ret);
				return ret;
			}
			near_far_state = ret & STK_FLG_NF_MASK;					
#endif			
			stk_ps_report(ps_data, near_far_state);
			PRINT_WARN( "%s: ps input event=%d, ps=%d\n",__func__, near_far_state, reading);
		}
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
		
#ifdef STK_TUNE1
		if( reading <= STK_FIN_THD)
		{
			ps_data->ps_thd_h = reading + ps_data->stk_ht_n_ct;
			ps_data->ps_thd_l = reading + ps_data->stk_lt_n_ct;
		}
	#ifdef CALI_PS_EVERY_TIME		
		else if(ps_data->boot_cali == 1 && ps_data->ps_low_thd_boot < STK_PS_HIGHER_THDH )
		{
			ps_data->ps_thd_h = reading + ps_data->ps_high_thd_boot;
			ps_data->ps_thd_l = reading + ps_data->ps_low_thd_boot;
		}
	#endif
		else
		{
			ps_data->ps_thd_h = STK_PS_HIGHER_THDH;
			ps_data->ps_thd_l = STK_PS_HIGHER_THDL;
		}
		
		// if(ps_data->ps_thd_h > STK_PS_HIGHER_THDH)
		// {
			// ps_data->ps_thd_h = STK_PS_HIGHER_THDH; 
			// ps_data->ps_thd_l = STK_PS_HIGHER_THDL;
		// }
		stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3a5x_set_ps_thd_l(ps_data, 0);		
#else//STK_TUNE1
#ifdef CALI_PS_EVERY_TIME		
		if(ps_data->boot_cali == 1 && ps_data->ps_low_thd_boot < 1000 )
		{
			ps_data->ps_thd_h = ps_data->ps_high_thd_boot;
			ps_data->ps_thd_l = ps_data->ps_low_thd_boot;
		}
		else
#endif			
		// {
			// ps_data->ps_thd_h = 0xFFFF;
			// ps_data->ps_thd_l = 0;
		// }
		// stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		// stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
#endif	// #ifdef STK_TUNE1
		PRINT_WARN( "%s: HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
	}
	else//if enable
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
#ifdef STK_GES		
		if(ps_data->re_enable_ges)
		{
			PRINT_WARN( "%s: re-enable ges\n", __func__);		
			stk3a5x_enable_ges(ps_data, 1, 1);
			ps_data->re_enable_ges = 0;
		}
#endif		
#ifdef CTTRACKING
		hrtimer_cancel(&ps_data->ps_tune0_timer);
		cancel_work_sync(&ps_data->stk_ps_tune0_work);
#endif		
	}
	return ret;
}

static int32_t stk3a5x_enable_als(struct stk3a5x_data *ps_data, uint8_t enable)
{
    int32_t ret;
	uint8_t w_state_reg;
	uint8_t curr_als_enable = (ps_data->als_enabled)?1:0;

	enable = 1;
	
	printk( "%s enter\n", __func__);

#ifdef STK_GES			
	if(ps_data->ges_enabled)
	{
		PRINT_WARN( "%s: since ges is enabled, ALS is disabled\n", __func__);
		ps_data->re_enable_als = enable ? true : false;
		return 0;
	}	
#endif	/* #ifdef STK_GES */
	printk( "%s: curr_als_enable = %d , enable = %d\n", __func__, curr_als_enable, enable);	
	if(curr_als_enable == enable)
		return 0;
#ifdef STK_QUALCOMM_POWER_CTRL		
	if (enable) {
		ret = stk3a5x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
	}	
#endif	
#ifndef STK_POLL_ALS			
	#ifdef STK_IRS
	if(enable && !(ps_data->ps_enabled))
	{		
		ret = stk3a5x_get_ir_reading(ps_data, STK_IRS_IT_REDUCE	);
		if(ret > 0)
			ps_data->ir_code = ret;
	}		
	#endif
	
    if (enable)
	{				
        stk3a5x_set_als_thd_h(ps_data, 0x0000);
        stk3a5x_set_als_thd_l(ps_data, 0xFFFF);		
	}
#endif	

	ret = stk3a5x_get_state(ps_data);
	printk( "%s: stk3a5x_get_state ret = %d\n", __func__, ret);
	if(ret < 0)
		return ret;
	
	w_state_reg = (uint8_t)(ret & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK))); 
	if(enable)	
		w_state_reg |= STK_STATE_EN_ALS_MASK;	
	else if (ps_data->ps_enabled)		
		w_state_reg |= STK_STATE_EN_WAIT_MASK;	

	ret = stk3a5x_set_state(ps_data, w_state_reg);
	printk( "%s: stk3a5x_get_state ret = %d\n", __func__, ret);
	if(ret < 0)
		return ret;	
	ps_data->state_reg = w_state_reg;
	
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
#ifdef STK_IRS
		ps_data->als_data_index = 0;
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
#ifdef STK_QUALCOMM_POWER_CTRL		
		ret = stk3a5x_device_ctl(ps_data, enable);
		if (ret)
			return ret;
#endif			
	}
    return ret;
}


static int32_t stk3a5x_get_als_reading(struct stk3a5x_data *ps_data)
{
    int32_t als_data, ir_data = 0;
#ifdef STK_ALS_FIR
	int index;   
	int firlen = atomic_read(&ps_data->firlength);   
#endif	
	unsigned char value[2];
	int ret;
#ifdef STK_IRS				
	const int ir_enlarge = 1 << (STK_ALS_READ_IRS_IT_REDUCE - STK_IRS_IT_REDUCE);
#endif	

	ret = stk3a5x_i2c_read_data(ps_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	als_data = (value[0]<<8) | value[1];
	// printk("%s: raw als_data=%d\n", __func__, als_data);
	
	if(ps_data->p_1x_r_bd_with_co == 0x07 || ps_data->p_19_r_bc == 0x03)
	{
		als_data = als_data * 16 / 10;
		if(als_data > 65535)
			als_data = 65535;
	}
	
	if(ps_data->p_wv_r_bd_with_co & 0b010)
	{
		if(als_data < STK_ALS_THRESHOLD && ps_data->als_code_last > 10000)
		{
			ir_data = stk3a5x_get_ir_reading(ps_data, STK_ALS_READ_IRS_IT_REDUCE);
#ifdef STK_IRS				
			if(ir_data > 0)
				ps_data->ir_code = ir_data * ir_enlarge;
#endif			
			// PRINT_WARN( "%s: als_data=%d, als_code_last=%d,ir_data=%d\n", 
					// __func__, als_data, ps_data->als_code_last, ir_data);	
			if(ir_data > (STK_ALS_THRESHOLD*3))
			{
				als_data = ps_data->als_code_last;
			}
		}
#ifdef STK_IRS			
		else
		{
			ps_data->ir_code = 0;
		}
#endif		
	}
	
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


#if (defined(STK_IRS) && defined(STK_POLL_ALS))
static int stk_als_ir_skip_als(struct stk3a5x_data *ps_data)
{
	int ret;
	unsigned char value[2];	
	
	if(ps_data->als_data_index < 60000)
		ps_data->als_data_index++;
	else
		ps_data->als_data_index = 0;
	
	if(	ps_data->als_data_index % 10 == 1)
	{
		ret = stk3a5x_i2c_read_data(ps_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
		if(ret < 0)
		{
			printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
			return ret;
		}
		return 1;
	}
	return 0;
}

static void stk_als_ir_get_corr(struct stk3a5x_data *ps_data, int32_t als)
{
	int32_t als_comperator;
	
	if(ps_data->ir_code)
	{
		ps_data->als_correct_factor = 1000;
		if(als < STK_IRC_MAX_ALS_CODE && als > STK_IRC_MIN_ALS_CODE && 
			ps_data->ir_code > STK_IRC_MIN_IR_CODE)
		{
			als_comperator = als * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(ps_data->ir_code > als_comperator)
				ps_data->als_correct_factor = STK_IRC_ALS_CORREC;
		}
#ifdef STK_DEBUG_PRINTF				
		PRINT_WARN( "%s: als=%d, ir=%d, als_correct_factor=%d", __func__, 
						als, ps_data->ir_code, ps_data->als_correct_factor);
#endif		
		ps_data->ir_code = 0;
	}	
	return;
}

static int stk_als_ir_run(struct stk3a5x_data *ps_data)
{
	int ret;
	
	if(	ps_data->als_data_index % 10 == 0)
	{
		if(ps_data->ps_distance_last != 0 && ps_data->ir_code == 0)
		{
			ret = stk3a5x_get_ir_reading(ps_data, STK_IRS_IT_REDUCE);
			if(ret > 0)
				ps_data->ir_code = ret;
		}		
		return ret;
	}	
	return 0;
}
#endif	/* #if (defined(STK_IRS) && defined(STK_POLL_ALS)) */


static int32_t stk3a5x_set_irs_it_slp(struct stk3a5x_data *ps_data, uint16_t *slp_time, int32_t ials_it_reduce)
{
	uint8_t irs_alsctrl;
	int32_t ret;
		
	irs_alsctrl = (ps_data->alsctrl_reg & 0x0F) - ials_it_reduce;
	switch(irs_alsctrl)
	{
		case 2:
			*slp_time = 1;
			break;			
		case 3:
			*slp_time = 2;
			break;	
		case 4:
			*slp_time = 3;
			break;	
		case 5:
			*slp_time = 6;
			break;
		case 6:
			*slp_time = 12;
			break;
		case 7:
			*slp_time = 24;			
			break;
		case 8:
			*slp_time = 48;			
			break;
		case 9:
			*slp_time = 96;			
			break;				
		case 10:
			*slp_time = 192;
			break;				
		default:
			printk(KERN_ERR "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
			ret = -EINVAL;	
			return ret;
	}
	irs_alsctrl |= (ps_data->alsctrl_reg & 0xF0);
	ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, irs_alsctrl);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;		
	}		
	return 0;
}

static int32_t stk3a5x_get_ir_reading(struct stk3a5x_data *ps_data, int32_t als_it_reduce)
{
    int32_t word_data, ret;
	uint8_t w_reg, retry = 0;	
	uint16_t irs_slp_time = 100;
	unsigned char value[2];
	
	ret = stk3a5x_set_irs_it_slp(ps_data, &irs_slp_time, als_it_reduce);
	if(ret < 0)
		goto irs_err_i2c_rw;
	
	ret = stk3a5x_get_state(ps_data);
	if(ret < 0)
		goto irs_err_i2c_rw;
	
	w_reg = ret | STK_STATE_EN_IRS_MASK;		
	ret = stk3a5x_set_state(ps_data, w_reg);
	if(ret < 0)
		goto irs_err_i2c_rw;
	msleep(irs_slp_time);	
	
	do
	{
		usleep_range(3000, 4000);
		ret = stk3a5x_get_flag(ps_data);
		if (ret < 0)
			goto irs_err_i2c_rw;
		retry++;
	}while(retry < 10 && ((ret&STK_FLG_IR_RDY_MASK) == 0));
	
	if(retry == 10)
	{
		printk(KERN_ERR "%s: ir data is not ready for a long time\n", __func__);
		ret = -EINVAL;
		goto irs_err_i2c_rw;
	}

	ret = stk3a5x_set_flag(ps_data, ret, STK_FLG_IR_RDY_MASK);
    if (ret < 0)
		goto irs_err_i2c_rw;
	
	ret = stk3a5x_i2c_read_data(ps_data->client, STK_DATA1_IR_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		goto irs_err_i2c_rw;
	}
	word_data = ((value[0]<<8) | value[1]);	
	//PRINT_WARN( "%s: ir=%d\n", __func__, word_data);
	
	ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_ALSCTRL_REG, ps_data->alsctrl_reg );
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}
				
	return word_data;

irs_err_i2c_rw:		
	return ret;
}

#ifdef STK_CHK_REG
static int stk3a5x_chk_reg_valid(struct stk3a5x_data *ps_data) 
{
	unsigned char value[9];
	int err;
	/*
	uint8_t cnt;
		
	for(cnt=0;cnt<9;cnt++)
	{
		value[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, (cnt+1));
		if(value[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, value[cnt]);	
			return value[cnt];
		}
	}	
	*/
	err = stk3a5x_i2c_read_data(ps_data->client, STK_PSCTRL_REG, 9, &value[0]);
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
#ifdef STK_IRS	
	if((value[1] != ps_data->alsctrl_reg) && (value[1] != (ps_data->alsctrl_reg - STK_IRS_IT_REDUCE))
		&& (value[1] != (ps_data->alsctrl_reg - STK_ALS_READ_IRS_IT_REDUCE)))
#else
	if((value[1] != ps_data->alsctrl_reg) && (value[1] != (ps_data->alsctrl_reg - STK_ALS_READ_IRS_IT_REDUCE)))
#endif	
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

static int stk3a5x_validate_n_handle(struct i2c_client *client) 
{
	struct stk3a5x_data *ps_data = i2c_get_clientdata(client);
	int err;
	
	err = stk3a5x_chk_reg_valid(ps_data);
	if(err < 0)
	{
		printk(KERN_ERR "stk3a5x_chk_reg_valid fail: %d\n", err);        
		return err;
	}
	
	if(err == 0xFF)
	{		
		printk(KERN_ERR "%s: Re-init chip\n", __func__);				
		err = stk3a5x_software_reset(ps_data); 
		if(err < 0)
			return err;			
		err = stk3a5x_init_all_reg(ps_data);
		if(err < 0)
			return err;			
		
		//ps_data->psa = 0;
		//ps_data->psi = 0xFFFF;		
		stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);		
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);		
    int32_t reading;
#ifdef STK_POLL_ALS	
	reading = ps_data->als_code_last;
#else
	unsigned char value[2];	
	int ret;
	ret = stk3a5x_i2c_read_data(ps_data->client, STK_DATA1_ALS_REG, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	reading = (value[0]<<8) | value[1];	
#endif	
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

#ifdef QUALCOMM_PLATFORM
static int stk_als_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct stk3a5x_data *ps_data = container_of(sensors_cdev,
						struct stk3a5x_data, als_cdev);
	int err = 0;

	mutex_lock(&ps_data->io_lock);
	err = stk3a5x_enable_als(ps_data, enabled);
	mutex_unlock(&ps_data->io_lock);
	ps_data->als_en_hal = enabled?true:false;
	if (err < 0)
		return err;
	return 0;
}
#endif

static ssize_t stk_als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
    int32_t ret;
	
	ret = stk3a5x_get_state(ps_data);
	if(ret < 0)
		return ret;		
    ret = (ret & STK_STATE_EN_ALS_MASK)?1:0;
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static ssize_t stk_als_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data = dev_get_drvdata(dev);
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
	PRINT_WARN( "%s: Enable ALS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
	stk3a5x_enable_als(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
	ps_data->als_en_hal = en?true:false;
    return size;
}

static ssize_t stk_als_lux_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data = dev_get_drvdata(dev);
    int32_t als_reading;
	uint32_t als_lux;
    als_reading = stk3a5x_get_als_reading(ps_data);    
	als_lux = stk_alscode2lux(ps_data, als_reading);
    return scnprintf(buf, PAGE_SIZE, "%d lux\n", als_lux);
}

static ssize_t stk_als_lux_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	stk_als_report(ps_data, value);
    return size;
}


static ssize_t stk_als_transmittance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
    int32_t transmittance;
    transmittance = ps_data->als_transmittance;
    return scnprintf(buf, PAGE_SIZE, "%d\n", transmittance);
}


static ssize_t stk_als_transmittance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
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

static ssize_t stk_als_ir_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);		
    int32_t reading;
    reading = stk3a5x_get_ir_reading(ps_data, STK_IRS_IT_REDUCE);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);	
}

#ifdef STK_ALS_FIR
static ssize_t stk_als_firlen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
	int len = atomic_read(&ps_data->firlength);
	
	PRINT_WARN( "%s: len = %2d, idx = %2d\n", __func__, len, ps_data->fir.idx);			
	PRINT_WARN( "%s: sum = %5d, ave = %5d\n", __func__, ps_data->fir.sum, ps_data->fir.sum/len);
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", len);		
}


static ssize_t stk_als_firlen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    uint64_t value = 0;	
	int ret;	
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
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
#endif  

#ifdef STK_GES		
static ssize_t stk_ges_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
    int ret;
	unsigned int gest0 = 0, gest1 = 0, gest2 = 0;

	if(!ps_data->ges_enabled)
		return 0;
	
    ret = stk3a5x_get_ges_reading(ps_data, &gest0, &gest1, &gest2);
	if(ret < 0)
		return ret;
	else if(ret == 0xFFFF)
		atomic_set(&ps_data->gesture2, 0);
		
    return scnprintf(buf, PAGE_SIZE, "%5d,%5d,%5d\n", gest0, gest1, atomic_read(&ps_data->gesture2));
}

static ssize_t stk_ges_code_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t ges;
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 16, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
	
	switch(value)
	{
	case 3:
		//PRINT_WARN( "%s: ges input event, not detected\n",__func__);			
	case 0:
		return size;
	case 1:
		ges = KEY_PAGEUP;	
		atomic_set(&ps_data->gesture2, 0);
		PRINT_WARN( "%s: ges input event >>>\n",__func__);		
		break;
	case 2:
		ges = KEY_PAGEDOWN;	
		atomic_set(&ps_data->gesture2, 0);
		PRINT_WARN( "%s: ges input event <<<\n",__func__);		
		break;
	case 32:
		ges = KEY_VOLUMEDOWN;
		PRINT_WARN( "%s: ges input event near\n",__func__);		
		break;
	case 48:
		ges = KEY_VOLUMEUP;
		PRINT_WARN( "%s: ges input event far\n",__func__);		
		break;
	default:
		printk(KERN_ERR "%s, invalid value %d\n", __func__, (int)value);
		return -EINVAL;
	}
	
	input_report_key(ps_data->ges_input_dev, ges, 1);
	input_report_key(ps_data->ges_input_dev, ges, 0);
	input_sync(ps_data->ges_input_dev);
    return size;
}

static ssize_t stk_ges_poll_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0, ii = 0, jj = 0;
	
	while(stk_ges_op[ii].ops[0] != 0)
	{
		len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", ii);
		for(jj=0;jj<4;jj++)
			len += scnprintf(buf + len, PAGE_SIZE - len, "%x ", stk_ges_op[ii].ops[jj]);
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
		ii++;
	}
	return len;
}


static ssize_t stk_ges_poll_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int32_t ret, i = 0, index = 0;	
	char *token;
	unsigned long value = 0;
		
	while(buf != '\0')
	{
		token = strsep((char **)&buf, " ");
		if((ret = kstrtoul(token, 16, &value)) < 0)
		{
			printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
			return ret;	
		}
		
		if(i == 0)
		{
			if(value >= 10)
			{
				memset(stk_ges_op, 0, sizeof(stk_ges_op));				
				break;
			}
			else
				index = value;
		}
		else
		{
			stk_ges_op[index].ops[i-1] = value;
		}
		i++;
		if(i == 5)
			break;
	}
	if(i != 5)
	{
		printk(KERN_ERR "%s: invalid length(%d)\n", __func__, i);
		memset(&(stk_ges_op[index]), 0, sizeof(union stk_ges_operation));				
	}
	return size;
}
		
static ssize_t stk_ges_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
		
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=%d\n", __func__, ret);	
		return ret;	
	}	
    PRINT_WARN( "%s: Enable GES : %d\n", __func__, (int)value);	
	
	switch(value)
	{
	case 0:
		mutex_lock(&ps_data->io_lock);
		if(ps_data->ges_enabled == 1)
			stk3a5x_enable_ges(ps_data, 0, 1);
		else
			stk3a5x_enable_ges(ps_data, 0, 2);	
		mutex_unlock(&ps_data->io_lock);		
		break;
	case 1:
		mutex_lock(&ps_data->io_lock);
		stk3a5x_enable_ges(ps_data, 1, 1);
		mutex_unlock(&ps_data->io_lock);		
		break;
	case 2:
		mutex_lock(&ps_data->io_lock);
		stk3a5x_enable_ges(ps_data, 1, 2);
		mutex_unlock(&ps_data->io_lock);		
		break;
	default:
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;	
		break;
	}

    return size;
}

static ssize_t stk_ges_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->ges_enabled);		
}

#endif	/* #ifdef STK_GES */

static ssize_t stk_ps_code_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
    uint32_t reading;
    reading = stk3a5x_get_ps_reading(ps_data);
    return scnprintf(buf, PAGE_SIZE, "%d\n", reading);
}

#ifdef QUALCOMM_PLATFORM
static int stk_ps_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct stk3a5x_data *ps_data = container_of(sensors_cdev,
						struct stk3a5x_data, ps_cdev);
	int err;

	mutex_lock(&ps_data->io_lock);
	err = stk3a5x_enable_ps(ps_data, enabled, 0);
	mutex_unlock(&ps_data->io_lock);

	if (err < 0)
		return err;
	return 0;
}
#endif

static ssize_t stk_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);

	ret = stk3a5x_get_state(ps_data);
	if(ret < 0)
		return ret;	
    ret = (ret & STK_STATE_EN_PS_MASK)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);	
}

static ssize_t stk_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
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
    PRINT_WARN( "%s: Enable PS : %d\n", __func__, en);
    mutex_lock(&ps_data->io_lock);
    stk3a5x_enable_ps(ps_data, en);
    mutex_unlock(&ps_data->io_lock);
    return size;
}

static ssize_t stk_ps_enable_aso_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ret;
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
	
    ret = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_STATE_REG);
    ret = (ret & STK_STATE_EN_ASO_MASK)?1:0;
	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ret);		
}

static ssize_t stk_ps_enable_aso_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
    int32_t ret;
	uint8_t w_state_reg;
	
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else 
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
    PRINT_WARN( "%s: Enable PS ASO : %d\n", __func__, en);
    
    ret = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, STK_STATE_REG);
    if (ret < 0)
    {
        printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
    }		
	w_state_reg = (uint8_t)(ret & (~STK_STATE_EN_ASO_MASK)); 
	if(en)	
		w_state_reg |= STK_STATE_EN_ASO_MASK;	
	
    ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
    if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	
	
	return size;	
}


static ssize_t stk_ps_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
    int32_t word_data;		
	unsigned char value[2];
	int ret;
	
	ret = stk3a5x_i2c_read_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, &value[0]);
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
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
	ret = stk3a5x_i2c_write_data(ps_data->client, STK_DATA1_OFFSET_REG, 2, val);	
	if(ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}
	
	return size;
}


static ssize_t stk_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);
    int32_t dist=1;
#ifndef STK_TUNE1
	int32_t ret;
#endif
	
#ifdef STK_TUNE1
	dist = ps_data->ps_distance_last;
#else	
    ret = stk3a5x_get_flag(ps_data);
	if(ret < 0)
		return ret;
    dist = (ret & STK_FLG_NF_MASK)?1:0;	
#endif	
	stk_ps_report(ps_data, dist);
	PRINT_WARN( "%s: ps input event=%d\n",__func__, dist);	
    return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t stk_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	unsigned long value = 0;
	int ret;
	
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);	
		return ret;	
	}
	stk_ps_report(ps_data, value);
	PRINT_WARN( "%s: ps input event=%d\n",__func__, (int)value);		
    return size;
}


static ssize_t stk_ps_code_thd_l_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_l1_reg, ps_thd_l2_reg;
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
    ps_thd_l1_reg = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL1_PS_REG);
    if(ps_thd_l1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_l1_reg);		
		return -EINVAL;		
	}
    ps_thd_l2_reg = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_THDL2_PS_REG);
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    stk3a5x_set_ps_thd_l(ps_data, value);
    return size;
}

static ssize_t stk_ps_code_thd_h_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_thd_h1_reg, ps_thd_h2_reg;
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
    ps_thd_h1_reg = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH1_PS_REG);
    if(ps_thd_h1_reg < 0)
	{
		printk(KERN_ERR "%s fail, err=0x%x", __func__, ps_thd_h1_reg);		
		return -EINVAL;		
	}
    ps_thd_h2_reg = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,STK_THDH2_PS_REG);
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);		
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	    
	}
    stk3a5x_set_ps_thd_h(ps_data, value);
    return size;
}


static ssize_t stk_all_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int32_t ps_reg[0x23];
	uint8_t cnt;
	int len = 0;
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	
	for(cnt=0;cnt<0x20;cnt++)
	{
		ps_reg[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			PRINT_WARN( "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
			len += scnprintf(buf+len, PAGE_SIZE-len, "[%2X]%2X,", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);	
	
	cnt++;
	ps_reg[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	PRINT_WARN( "reg[0x%x]=0x%2X\n", STK_RSRVD_REG, ps_reg[cnt]);

	cnt++;
	ps_reg[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, 0xE0);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	PRINT_WARN( "reg[0xE0]=0x%2X\n", ps_reg[cnt]);	
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	for(cnt=0;cnt<25;cnt++)
	{
		ps_reg[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, (cnt));
		if(ps_reg[cnt] < 0)
		{
			printk(KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
			return -EINVAL;
		}
		else
		{
			PRINT_WARN( "reg[0x%2X]=0x%2X\n", cnt, ps_reg[cnt]);
		}
	}
	ps_reg[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, STK_PDT_ID_REG);
	if(ps_reg[cnt] < 0)
	{
		printk( KERN_ERR "%s fail, ret=%d", __func__, ps_reg[cnt]);	
		return -EINVAL;
	}
	printk( KERN_INFO "reg[0x%x]=0x%2X\n", STK_PDT_ID_REG, ps_reg[cnt]);	
	cnt++;
	ps_reg[cnt] = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, STK_RSRVD_REG);
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&ps_data->recv_reg));     		
}


static ssize_t stk_recv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long value = 0;
	int ret;
	int32_t recv_data;	
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	
	if((ret = kstrtoul(buf, 16, &value)) < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n", __func__, ret);
		return ret;	
	}
	recv_data = stk3a5x_i2c_smbus_read_byte_data(ps_data->client,value);
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	
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
	PRINT_WARN( "%s: write reg 0x%x=0x%x\n", __func__, addr, cmd);		

	ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, (unsigned char)addr, (unsigned char)cmd);
	if (0 != ret)
	{	
		printk(KERN_ERR "%s: stk3a5x_i2c_smbus_write_byte_data fail\n", __func__);
		return ret;
	}
	
	return size;
}

#ifdef STK_TUNE0

static ssize_t stk_ps_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	int32_t word_data;	
	unsigned char value[2];
	int ret;
	
	ret = stk3a5x_i2c_read_data(ps_data->client, 0x20, 2, &value[0]);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}
	word_data = (value[0]<<8) | value[1];	
	
	ret = stk3a5x_i2c_read_data(ps_data->client, 0x22, 2, &value[0]);
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_max_min_diff);     		
}

static ssize_t stk_ps_ltnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_lt_n_ct);     		
}

static ssize_t stk_ps_htnct_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
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
	struct stk3a5x_data *ps_data =  dev_get_drvdata(dev);	
	return scnprintf(buf, PAGE_SIZE, "%d\n", ps_data->stk_ht_n_ct);     		
}
#endif	/* #ifdef STK_TUNE0 */
 
static struct device_attribute als_enable_attribute = __ATTR(enable,0664,stk_als_enable_show,stk_als_enable_store);
static struct device_attribute als_lux_attribute = __ATTR(lux,0664,stk_als_lux_show,stk_als_lux_store);
static struct device_attribute als_code_attribute = __ATTR(code, 0444, stk_als_code_show, NULL);
static struct device_attribute als_transmittance_attribute = __ATTR(transmittance,0664,stk_als_transmittance_show,stk_als_transmittance_store);
static struct device_attribute als_ir_code_attribute = __ATTR(ircode,0444,stk_als_ir_code_show,NULL);
#ifdef STK_ALS_FIR
static struct device_attribute als_firlen_attribute = __ATTR(firlen,0664,stk_als_firlen_show,stk_als_firlen_store);
#endif

static struct attribute *stk_als_attrs [] =
{
	&als_enable_attribute.attr,
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
	&als_ir_code_attribute.attr,
#ifdef STK_ALS_FIR
	&als_firlen_attribute.attr,
#endif	
    NULL
};

static struct attribute_group stk_als_attribute_group = {
#ifndef QUALCOMM_PLATFORM
	.name = "driver",
#endif	
	.attrs = stk_als_attrs,
};
 
#ifdef STK_GES		
static struct device_attribute ges_enable_attribute = __ATTR(enable,0664,stk_ges_enable_show,stk_ges_enable_store);
static struct device_attribute ges_code_attribute = __ATTR(code, 0664, stk_ges_code_show, stk_ges_code_store);
static struct device_attribute ges_poll_attribute = __ATTR(poll, 0664, stk_ges_poll_show, stk_ges_poll_store);
static struct device_attribute ges_recv_attribute = __ATTR(recv,0664,stk_recv_show,stk_recv_store);
static struct device_attribute ges_send_attribute = __ATTR(send,0664,stk_send_show, stk_send_store);

static struct attribute *stk_ges_attrs [] =
{
    &ges_enable_attribute.attr,	
    &ges_code_attribute.attr,
    &ges_poll_attribute.attr,
	&ges_recv_attribute.attr,
	&ges_send_attribute.attr,
	&als_enable_attribute.attr,
    &als_lux_attribute.attr,
    &als_code_attribute.attr,
    &als_transmittance_attribute.attr,
	&als_ir_code_attribute.attr,
#ifdef STK_ALS_FIR
	&als_firlen_attribute.attr,
#endif	
    NULL
    NULL
};

static struct attribute_group stk_ges_attribute_group = 
{
	.name = "driver",	
	.attrs = stk_ges_attrs,
};
#endif	/* #ifdef STK_GES */

static struct device_attribute ps_enable_attribute = __ATTR(enable,0664,stk_ps_enable_show,stk_ps_enable_store);
static struct device_attribute ps_enable_aso_attribute = __ATTR(enableaso,0664,stk_ps_enable_aso_show,stk_ps_enable_aso_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0664,stk_ps_distance_show, stk_ps_distance_store);
static struct device_attribute ps_offset_attribute = __ATTR(offset,0664,stk_ps_offset_show, stk_ps_offset_store);
static struct device_attribute ps_code_attribute = __ATTR(code, 0444, stk_ps_code_show, NULL);
static struct device_attribute ps_code_thd_l_attribute = __ATTR(codethdl,0664,stk_ps_code_thd_l_show,stk_ps_code_thd_l_store);
static struct device_attribute ps_code_thd_h_attribute = __ATTR(codethdh,0664,stk_ps_code_thd_h_show,stk_ps_code_thd_h_store);
static struct device_attribute ps_recv_attribute = __ATTR(recv,0664,stk_recv_show,stk_recv_store);
static struct device_attribute ps_send_attribute = __ATTR(send,0664,stk_send_show, stk_send_store);
static struct device_attribute all_reg_attribute = __ATTR(allreg, 0444, stk_all_reg_show, NULL);
static struct device_attribute status_attribute = __ATTR(status, 0444, stk_status_show, NULL);
#ifdef STK_TUNE0
static struct device_attribute ps_cali_attribute = __ATTR(cali,0444,stk_ps_cali_show, NULL);
static struct device_attribute ps_maxdiff_attribute = __ATTR(maxdiff,0664,stk_ps_maxdiff_show, stk_ps_maxdiff_store);
static struct device_attribute ps_ltnct_attribute = __ATTR(ltnct,0664,stk_ps_ltnct_show, stk_ps_ltnct_store);
static struct device_attribute ps_htnct_attribute = __ATTR(htnct,0664,stk_ps_htnct_show, stk_ps_htnct_store);
#endif

static struct attribute *stk_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    &ps_enable_aso_attribute.attr,
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
	
    NULL
};

static struct attribute_group stk_ps_attribute_group = {
#ifndef QUALCOMM_PLATFORM
	.name = "driver",	
#endif	
	.attrs = stk_ps_attrs,
};

static int stk_ps_val(struct stk3a5x_data *ps_data)
{
	int32_t lii = 100;	
	unsigned char value[4];
	int ret, i;
	bool out_of_range = false;
	
	ret = stk3a5x_i2c_read_data(ps_data->client, 0x34, 4, value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
		return ret;
	}

	for (i = 0; i < 4; i++)
    {
        if (value[i] >= lii)
        {
            out_of_range = true;
            break;
        }
    }
	if(out_of_range == true)
	{
		PRINT_WARN( "%s: bgir:%d, %d, %d, %d\n", __func__, value[0], value[1], value[2], value[3]);	
		return 0xFFFF;	
	}
	return 0;
}	

#ifdef STK_TUNE0
static int stk_ps_tune_zero_final(struct stk3a5x_data *ps_data)
{
	int ret;
	
	ps_data->tune_zero_init_proc = false;
	ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, ps_data->int_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}	
	
	ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, 0);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	
	if(ps_data->data_count == -1)
	{
		PRINT_WARN( "%s: exceed limit\n", __func__);
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
	stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
	stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);				
	ps_data->boot_cali = 1;
	PRINT_WARN( "%s: set HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h,  ps_data->ps_thd_l);		
	hrtimer_cancel(&ps_data->ps_tune0_timer);					
	return 0;
}
	
static int32_t stk_tune_zero_get_ps_data(struct stk3a5x_data *ps_data)
{
	uint32_t ps_adc;
	int ret;
	
	ret = stk_ps_val(ps_data);	
	if(ret == 0xFFFF)
	{		
		ps_data->data_count = -1;
		stk_ps_tune_zero_final(ps_data);
		return 0;
	}	
	
	ps_adc = stk3a5x_get_ps_reading(ps_data);
	PRINT_WARN( "%s: ps_adc #%d=%d\n", __func__, ps_data->data_count, ps_adc);
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

#ifndef QUALCOMM_PLATFORM
static int stk_ps_tune_zero_init(struct stk3a5x_data *ps_data)
{
	int32_t ret = 0;
	uint8_t w_state_reg;	

#ifdef CALI_EVERY_TIME
	ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
	ps_data->ps_low_thd_boot = ps_data->ps_thd_l;
	if(ps_data->ps_high_thd_boot <= 0)
	{
		ps_data->ps_high_thd_boot = ps_data->stk_ht_n_ct*3;
		ps_data->ps_low_thd_boot = ps_data->stk_lt_n_ct*3;
	}
#endif
	
	ps_data->psi_set = 0;	
	ps_data->ps_stat_data[0] = 0;
	ps_data->ps_stat_data[2] = 9999;
	ps_data->ps_stat_data[1] = 0;
	ps_data->data_count = 0;
	ps_data->boot_cali = 0;
	ps_data->tune_zero_init_proc = true;		
	
	ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_INT_REG, 0);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}			
	
	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);			
	ret = stk3a5x_i2c_smbus_write_byte_data(ps_data->client, STK_STATE_REG, w_state_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;
	}		
	hrtimer_start(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay, HRTIMER_MODE_REL);		

	return 0;	
}
#endif

static uint32_t stk3a5x_get_ps_thd_h(struct stk3a5x_data *ps_data)
{
	unsigned char value[2];
	int err;
	err = stk3a5x_i2c_read_data(ps_data->client, STK_THDH1_PS_REG, 2, &value[0]);
	if(err < 0) {
		printk(KERN_ERR "fail, ret=%d\n", err);
		return 0;
	}
	return ((value[0]<<8) | value[1]);
}

static uint32_t stk3a5x_get_ps_thd_l(struct stk3a5x_data *ps_data)
{
	unsigned char value[2];
	int err;
	err = stk3a5x_i2c_read_data(ps_data->client, STK_THDL1_PS_REG, 2, &value[0]);
	if(err < 0) {
		printk(KERN_ERR "fail, ret=%d\n", err);
		return 0;
	}
	return ((value[0]<<8) | value[1]);
}

static void stk3a5x_dump_reg(struct stk3a5x_data *ps_data)
{
	int ret;
	unsigned char reg[12]={0x00, 0x01, 0x02, 0x03, 0x04, 
                               0x05, 0xA0, 0xA1, 0xDB, 0xF4, 
                               0x3E, 0x3F};
	uint8_t cnt;
	for(cnt = 0; cnt < 12; cnt++){
		ret = stk3a5x_i2c_smbus_read_byte_data(ps_data->client, reg[cnt]);
		if(ret < 0){
			printk(KERN_ERR "stk3a5x_dump_reg fail, ret=%d\n", ret);
		}

		printk("stk3a5x_dump_reg reg[0x%2X]=0x%2X\n", reg[cnt], ret);
	}
}


static int stk_ps_tune_zero_func_fae(struct stk3a5x_data *ps_data)
{
	int32_t word_data;
	int ret, diff;
	unsigned char value[2];
	int32_t flag;
	//printk(KERN_EMERG "%s\n", __func__);			
#ifdef CTTRACKING
	uint16_t ct_value = 0;
#endif
	uint32_t high_thd, low_thd;
#ifdef CALI_PS_EVERY_TIME
	if(!(ps_data->ps_enabled))
#else
	if(ps_data->psi_set || !(ps_data->ps_enabled))
#endif
	{
		return 0;
	}	

	ps_data->data_count++;
	if(ps_data->data_count < 3) {
		printk( "stk count=%d\n", ps_data->data_count);
		return 0;
	}
	//printk(KERN_EMERG "%s data_count:%d\n", __func__,ps_data->data_count);
	if(ps_data->data_count > 10000) {
		ps_data->data_count = 10;
	} else if((ps_data->data_count%10)==3) {
		flag = stk3a5x_get_flag(ps_data);
		if(flag < 0) {
			return flag;
		}

		ret = stk3a5x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
		if(ret < 0) {
			printk("fail, ret=0x%x", ret);
			return ret;
		}
		word_data = (value[0]<<8) | value[1];

		printk( "stk cur ps=%d, flag=0x%x, dis=%d\n", word_data, flag, ps_data->ps_distance_last);
		printk( "stk psi_set=%d, psi=%d, smudge=%d\n", ps_data->psi_set, ps_data->psi, ps_data->ps_thd_update);
	} else if((ps_data->data_count%10)==2) {
		high_thd = stk3a5x_get_ps_thd_h(ps_data);
		if (high_thd < 0) {
			printk("fail, error: %d\n", high_thd);
			return high_thd;
		}

		low_thd = stk3a5x_get_ps_thd_l(ps_data);
		if (low_thd < 0) {
			printk("fail, error: %d\n", low_thd);
			return low_thd;
		}
		printk( "stk cur ht=%d, lt=%d\n", high_thd, low_thd);
	} else if((ps_data->data_count%50)==8) {
		stk3a5x_dump_reg(ps_data);
	}
	

#ifdef CTTRACKING
	if ((ps_data->psi_set != 0)) {
		if (ps_data->ps_distance_last == 1) {
			ret = stk_ps_val(ps_data);

			if (ret == 0) {
				ret = stk3a5x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
				if(ret < 0) {
					printk("fail, ret=0x%x", ret);
					return ret;
				}
				word_data = (value[0]<<8) | value[1];

				if (word_data > 0) {
					ct_value = ps_data->ps_thd_h - ps_data->stk_ht_n_ct;
					flag = stk3a5x_get_flag(ps_data);
					if ((word_data < ct_value) && ((ct_value - word_data) > 10)
					    && ((flag & STK_FLG_PSINT_MASK) == 0)
					    && (ps_data->ps_distance_last == 1)
					    && (ps_data->ps_thd_update == false)) {
						ps_data->tracking_count++;
						printk( "stk CTTRACKING pre ps=%d, count=%d\n", word_data, ps_data->tracking_count);
						if(ps_data->tracking_count > 3) {
							ps_data->psi = word_data;
							ps_data->ps_thd_h = ps_data->psi + ps_data->stk_ht_n_ct;
							ps_data->ps_thd_l = ps_data->psi + ps_data->stk_lt_n_ct;
							stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
							stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
							printk( "stk CTTRACKING set HT=%d, LT=%d, psi=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l, ps_data->psi);
							ps_data->tracking_count = 0;
							if ((ps_data->ps_thd_h + STK_H_LT - STK_HT_N_CT) < ps_data->ps_high_thd_boot) {
								ps_data->ps_high_thd_boot =
								    word_data + STK_H_HT;
								ps_data->ps_low_thd_boot =
								    word_data + STK_H_LT;
								ps_data->boot_cali = 1;
								printk( "stk update boot2 HT=%d, LT=%d\n",
								                ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
							}
						}
					} else {
						ps_data->tracking_count = 0;
					}
				}
			}
		} else if(ps_data->ps_thd_update == false) {
			ret = stk_ps_val(ps_data);
			if (ret == 0) {
				ret = stk3a5x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
				if(ret < 0) {
					printk("fail, ret=0x%x", ret);
					return ret;
				}
				word_data = (value[0]<<8) | value[1];

				if((ps_data->data_count%15)==9) {
					printk( "stk judge ps=%d\n", word_data);
				}

				if(word_data > (ps_data->psi + STK_H_PS)) {
					ps_data->ps_thd_h = ps_data->psi + STK_H_HT;
					ps_data->ps_thd_l = ps_data->psi + STK_H_LT;
					stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
					stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
					ps_data->ps_thd_update = true;
					printk( "stk smudge set HT=%d, LT=%d, psi=%d, ps=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l, ps_data->psi, word_data);
				}
			}
		}

		return 0;
	}
#endif

	ret = stk3a5x_get_flag(ps_data);
	if(ret < 0)
		return ret;
	if(!(ret&STK_FLG_PSDR_MASK))
	{
		//PRINT_WARN( "%s: ps data is not ready yet\n", __func__);
		return 0;
	}
	
	ret = stk_ps_val(ps_data);	
	if(ret == 0)
	{				
		ret = stk3a5x_i2c_read_data(ps_data->client, 0x11, 2, &value[0]);
		if(ret < 0)
		{
			printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret);
			return ret;
		}
		word_data = (value[0]<<8) | value[1];						
		printk( "stk %s: word_data=%d\n", __func__, word_data);
		
		if(word_data == 0)
		{
			//printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		
		if(word_data > ps_data->psa)
		{
			ps_data->psa = word_data;
			printk( "stk %s: update psa: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);
		}
		if(word_data < ps_data->psi)
		{
			ps_data->psi = word_data;	
			printk( "stk %s: update psi: psa=%d,psi=%d\n", __func__, ps_data->psa, ps_data->psi);	
		}	
	}	
	diff = ps_data->psa - ps_data->psi;
	if(diff > ps_data->stk_max_min_diff)
	{
		ps_data->psi_set = ps_data->psi;
#ifndef STK_TUNE1		
	#ifdef CALI_PS_EVERY_TIME
		if(((ps_data->psi + ps_data->stk_ht_n_ct) > (ps_data->ps_thd_h + 500)) && (ps_data->ps_thd_h != 0))
		{
		//	ps_data->ps_thd_h = ps_data->ps_thd_h;
		//	ps_data->ps_thd_l = ps_data->ps_thd_l;
			printk( "stk %s: no update thd, HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);
		}
		else
		{
			ps_data->ps_thd_h = ps_data->psi + ps_data->stk_ht_n_ct;
			ps_data->ps_thd_l = ps_data->psi + ps_data->stk_lt_n_ct;
			printk( "stk %s: update thd, HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);			
		}
	#else
		ps_data->ps_thd_h = ps_data->psi + ps_data->stk_ht_n_ct;
		ps_data->ps_thd_l = ps_data->psi + ps_data->stk_lt_n_ct;
		printk( "stk %s: update thd, HT=%d, LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);			
	#endif			
		
/*
#ifdef CALI_PS_EVERY_TIME
		if(ps_data->ps_thd_h > (ps_data->ps_high_thd_boot))
		{
			ps_data->ps_high_thd_boot = ps_data->ps_thd_h;
			ps_data->ps_low_thd_boot = ps_data->ps_thd_l;
			PRINT_WARN( "%s: update boot HT=%d, LT=%d\n", __func__, ps_data->ps_high_thd_boot, ps_data->ps_low_thd_boot);
		}
#endif		*/
		stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
		stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
			
		printk("%s: FAE tune0 psa-psi(%d) > STK_DIFF found\n", __func__, diff);
#endif		
#ifndef CTTRACKING
		hrtimer_cancel(&ps_data->ps_tune0_timer);		
#endif

	}
	
	return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3a5x_data *ps_data = container_of(work, struct stk3a5x_data, stk_ps_tune0_work);		
	printk(KERN_EMERG "%s :%d\n", __func__,ps_data->tune_zero_init_proc);
	if(ps_data->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(ps_data);
	else
		stk_ps_tune_zero_func_fae(ps_data);
	return;
}	

static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3a5x_data *ps_data = container_of(timer, struct stk3a5x_data, ps_tune0_timer);
	printk(KERN_EMERG "%s\n", __func__);
	queue_work(ps_data->stk_ps_tune0_wq, &ps_data->stk_ps_tune0_work);	
	hrtimer_forward_now(&ps_data->ps_tune0_timer, ps_data->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
#endif

#ifdef STK_TUNE1
struct get_ct_struct
{
	int ps_val[STK_PS_VAL_BUF];
	unsigned int ps_val_buffer_cnt;
	int psi;
	int psa;
};

int stk_tune1_check_stable(int val[], int val_len, int criteria, int *imax, int *imin)
{
	int aa;
	int max, min;
	
	max = INT_MIN;
	min = INT_MAX;
	
	for(aa=0;aa<val_len;aa++)
	{
		if(val[aa] > max)                                                                        
			max = val[aa];
		if(val[aa] < min)
			min = val[aa];
	}
	
	if(imax != NULL)
		*imax = max;
	if(imin != NULL)
		*imin = min;
	
	if((max - min) < criteria)
		return 1;
	return 0;
}

static struct get_ct_struct ps_get_ct;

static void stk_tune1_reset_para(void)
{
	int cnt;
	
	ps_get_ct.psi = 0xFFFF;
	ps_get_ct.psa = 0;	
	ps_get_ct.ps_val_buffer_cnt = 0;
	
	for(cnt=0;cnt<STK_PS_VAL_BUF;cnt++)
		ps_get_ct.ps_val[cnt] = -100;
	return;
}

static int stk_tune1_get_ct(uint32_t ps)
{
	int cnt;
	int ps_max, ps_min;
	
	ps_get_ct.ps_val[ps_get_ct.ps_val_buffer_cnt] = ps;
	ps_get_ct.ps_val_buffer_cnt++;
	if(ps_get_ct.ps_val_buffer_cnt >= STK_PS_VAL_BUF)
	{
		ps_get_ct.ps_val_buffer_cnt = 0;
	}
	
	if(ps < ps_get_ct.psi)
		ps_get_ct.psi = ps;
	if(ps > ps_get_ct.psa)
		ps_get_ct.psa = ps;
			
	// if(ps_get_ct.psa - ps_get_ct.psi > STK_CT_UPDATE_DIFF)
	// {
		if(stk_tune1_check_stable(ps_get_ct.ps_val, STK_PS_VAL_BUF, PS_NOISE, &ps_max, &ps_min) == 1)
		{
			stk_tune1_reset_para();
			return ps_min;
		}
	// }
	return -1;
}

static int stk_tune1_ct_det(struct stk3a5x_data *ps_data, uint32_t ps)
{
	int ct = 0;
	int err;
	
	err = stk_ps_val(ps_data);
	if(err == 0xFFFF)
	{
		if(ps_data->invalid_ps_cnt++ > STK_TUNE1_INVALID_PS_LIMIT)
		{
			printk("%s: invalid_ps_cnt > limit\n", __func__);
			return 3;
		}
		return 0;
	}
	
	ct = stk_tune1_get_ct(ps);
	if(ct >= 0)
	{
		if(ct < STK_PS_CT_LIMIT)
		{
			ps_data->ps_thd_h = ct + ps_data->stk_ht_n_ct;
			ps_data->ps_thd_l = ct + ps_data->stk_lt_n_ct;
			stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
#ifdef STK_POLL_PS			
			stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
#else
			stk3a5x_set_ps_thd_l(ps_data, 0);
#endif	
			// update_thd += STK_THD_F_PS0_CT_OK;
			printk("%s: HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);			
			return 1;
		}
		else
		{
			// update_thd = STK_THD_F_PS0_CT_NG;
			printk("%s: ct>STK_PS_CT_LIMIT,keep HT/LT\n", __func__);			
			return 2;
		}
	}
	return 0;
}

static int stk_tune1_fin_det(struct stk3a5x_data *ps_data, uint32_t ps, int32_t nf_flag)
{
	static uint8_t ps_far_re_cali = 0;
	
	if(nf_flag == 0)
	{
		if(ps >= STK_FIN_THD)
		{
			ps_far_re_cali = 1;
			ps_data->ps_thd_h = STK_PS_HIGHER_THDH;
			ps_data->ps_thd_l = STK_PS_HIGHER_THDL;
#ifdef STK_POLL_PS
			stk3a5x_set_ps_thd_h(ps_data, STK_PS_HIGHER_THDH);
			stk3a5x_set_ps_thd_l(ps_data, STK_PS_HIGHER_THDL);		
#endif
			printk("%s: fixed HT=%d,LT=%d\n", __func__, STK_PS_HIGHER_THDH, STK_PS_HIGHER_THDL);
			return 1;
		}
	}
	else
	{
		if(ps_far_re_cali)
		{
			ps_far_re_cali = 0;
			if(ps < STK_FIN_THD)
			{
				ps_data->ps_thd_h = ps + ps_data->stk_ht_n_ct * 2;
				ps_data->ps_thd_l = ps + ps_data->stk_lt_n_ct * 2;
#ifdef STK_POLL_PS					
				stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
				stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);		
#endif		
				printk("%s: HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);			
			}
			else
			{
				printk("%s: keep HT=%d,LT=%d\n", __func__, ps_data->ps_thd_h, ps_data->ps_thd_l);			
			}
			return 0x10;
		}
	}	
	return 0;
}
#endif

#ifdef STK_POLL_ALS
static enum hrtimer_restart stk_als_timer_func(struct hrtimer *timer)
{
	struct stk3a5x_data *ps_data = container_of(timer, struct stk3a5x_data, als_timer);
	queue_work(ps_data->stk_als_wq, &ps_data->stk_als_work);	
	hrtimer_forward_now(&ps_data->als_timer, ps_data->als_poll_delay);
	return HRTIMER_RESTART;	
}

static void stk_als_poll_work_func(struct work_struct *work)
{
	struct stk3a5x_data *ps_data = container_of(work, struct stk3a5x_data, stk_als_work);	
	int32_t reading = 0, reading_lux, flag_reg;
#ifdef STK_IRS	
	int ret;
#endif
#ifdef STK_GES
	#ifdef QUALCOMM_PLATFORM		
	ktime_t	timestamp = ktime_get_boottime();
	#endif	
	if(ps_data->ges_enabled)
	{
		input_report_abs(ps_data->ps_input_dev, ABS_MISC, ps_data->als_lux_last);
	#ifdef QUALCOMM_PLATFORM
		input_event(ps_data->ps_input_dev, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
		input_event(ps_data->ps_input_dev, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
	#endif
		input_sync(ps_data->ps_input_dev);
	#ifdef STK_DEBUG_PRINTF				
		PRINT_WARN( "%s: ges_enabled=1, als input event %d lux\n",__func__, ps_data->als_lux_last);		
	#endif
	}
	return;
#endif
		
	flag_reg = stk3a5x_get_flag(ps_data);
	if(flag_reg < 0)
		return;		
	if(!(flag_reg&STK_FLG_ALSDR_MASK))
	{
		//PRINT_WARN( "%s: als is not ready\n", __func__);
		return;
	}

#ifdef STK_IRS
	ret = stk_als_ir_skip_als(ps_data);
	if(ret == 1)
		return;
#endif	
	
	reading = stk3a5x_get_als_reading(ps_data);
	if(reading < 0)
		return;
	//printk("%s: als_data=%d\n", __func__, reading);
#ifdef STK_IRS
	stk_als_ir_get_corr(ps_data, reading);
	reading = reading * ps_data->als_correct_factor / 1000;
#endif

	reading_lux =  stk_alscode2lux(ps_data, reading);
	printk("%s: als_data=%d\n", __func__, reading_lux);

#if 0 //qzhu modify for report anytime
	if(abs(ps_data->als_lux_last - reading_lux) >= STK_ALS_CHANGE_THD)
#endif
	{
		stk_als_report(ps_data, reading_lux);
	}

#ifdef STK_IRS		
	stk_als_ir_run(ps_data);
#endif
}
#endif /* #ifdef STK_POLL_ALS */


#ifdef STK_POLL_PS	
static enum hrtimer_restart stk_ps_timer_func(struct hrtimer *timer)
{
	struct stk3a5x_data *ps_data = container_of(timer, struct stk3a5x_data, ps_timer);
	queue_work(ps_data->stk_ps_wq, &ps_data->stk_ps_work);
	hrtimer_forward_now(&ps_data->ps_timer, ps_data->ps_poll_delay);
	return HRTIMER_RESTART;		
}

static void stk_ps_poll_work_func(struct work_struct *work)
{
	struct stk3a5x_data *ps_data = container_of(work, struct stk3a5x_data, stk_ps_work);	
	uint32_t reading;
	int32_t near_far_state;
    uint8_t org_flag_reg;	
	
	if(ps_data->ps_enabled)
	{
#ifdef STK_TUNE0
		// if(!(ps_data->psi_set))
			// return;	
#endif
		org_flag_reg = stk3a5x_get_flag(ps_data);
		if(org_flag_reg < 0)
			return;

		if(!(org_flag_reg&STK_FLG_PSDR_MASK))
		{
			//PRINT_WARN( "%s: ps is not ready\n", __func__);
			return;
		}
		
		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;	
		reading = stk3a5x_get_ps_reading(ps_data);
		ps_data->ps_code_last = reading;
		printk( "%s ps=%d\n",__func__, reading);
		if(ps_data->ps_distance_last != near_far_state)
		{
			stk_ps_report(ps_data, near_far_state);
			printk( "%s ps input event=%d, ps=%d\n",__func__, near_far_state, reading);
		}
	}
}
#endif

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))

#ifdef STK_TUNE1

static int stk_tune1_ps_int_handle(struct stk3a5x_data *ps_data, uint32_t ps_reading)
{
	int ret;
	int cnt;
	int32_t nf_state;
	uint32_t als_reading = 0;
	
	nf_state = (ps_reading >= ps_data->ps_thd_h) ? 0 : (ps_reading<=ps_data->ps_thd_l ? 1 : ps_data->ps_distance_last);
	printk("%s:handle_state=%d,ps=%d,als=%d,nf_flag=%d,ps_distance_last=%d\n", 
		__func__, ps_data->stk_int_handle_state, ps_reading, als_reading, 
		nf_state, ps_data->ps_distance_last);
		
	switch(ps_data->stk_int_handle_state)
	{
	case STK_PSINT_NORM:
		stk_ps_report(ps_data, nf_state);
		PRINT_WARN( "%s: ps input event=%d, ps code=%d\n",__func__, nf_state, ps_reading);
		if(nf_state)
		{
			stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
			stk3a5x_set_ps_thd_l(ps_data, 0);
			ps_data->stk_int_handle_state = STK_PSINT_NORM;
			PRINT_WARN( "%s: HT=%d,LT=0\n", __func__, ps_data->ps_thd_h);
		}
		else
		{
			stk3a5x_set_ps_thd_h(ps_data, STK_FIN_THD);
			stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
			ps_data->stk_int_handle_state = STK_PSINT_FIN_DET;
			PRINT_WARN( "%s: HT=%d,LT=%d\n", __func__, STK_FIN_THD, ps_data->ps_thd_l);				
		}
		break;	
	case STK_PSINT_FIN_DET:
		if(nf_state)
		{
			stk_ps_report(ps_data, nf_state);
			PRINT_WARN( "%s: ps input event=%d, ps=%d\n",__func__, nf_state, ps_reading);			
			stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
			stk3a5x_set_ps_thd_l(ps_data, 0);
			ps_data->stk_int_handle_state = STK_PSINT_NORM;							
			PRINT_WARN( "%s: HT=%d,LT=0\n", __func__, ps_data->ps_thd_h);
		}
		break;
	case STK_PSINT_CT_DET:
		ret = stk_tune1_ct_det(ps_data, ps_reading);
		if(ret > 0)
		{
			stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
			stk3a5x_set_ps_thd_l(ps_data, 0);
			ps_data->stk_int_handle_state = STK_PSINT_NORM;							
			PRINT_WARN( "%s: HT=%d,LT=0\n", __func__, ps_data->ps_thd_h);
		}
		
		if(!nf_state)
		{
			stk_ps_report(ps_data, nf_state);
			PRINT_WARN( "%s: ps input event=%d, ps=%d\n",__func__, nf_state, ps_reading);				
			stk3a5x_set_ps_thd_h(ps_data, STK_FIN_THD);
			stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
			ps_data->stk_int_handle_state = STK_PSINT_FIN_DET;
			PRINT_WARN( "%s: HT=%d,LT=%d\n", __func__, STK_FIN_THD, ps_data->ps_thd_l);				
		}		
		break;			
	default:
		printk(KERN_ERR "%s: invalid stk_int_handle_state %d\n", __func__, ps_data->stk_int_handle_state);
	}
	
	ret = stk_tune1_fin_det(ps_data, ps_reading, nf_state);
	if(ret == 1)
	{
		if(ps_data->stk_int_handle_state == STK_PSINT_FIN_DET)
		{
			ps_data->stk_int_handle_state = STK_PSINT_NORM;
			stk3a5x_set_ps_thd_h(ps_data, 0xffff);
			stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
			PRINT_WARN( "%s: HT=0xffff,LT=%d\n", __func__, ps_data->ps_thd_l);
		}
	}
	else if(ret == 0x10 && nf_state == 1)
	{
		stk3a5x_set_ps_thd_h(ps_data, 0);
		stk3a5x_set_ps_thd_l(ps_data, 0xffff);
		ps_data->stk_int_handle_state = STK_PSINT_CT_DET;
		ps_data->invalid_ps_cnt = 0;		
		PRINT_WARN( "%s: HT=0,LT=0xffff\n", __func__);

		stk_tune1_reset_para();
		PRINT_WARN( "%s: reset ps_get_ct\n", __func__);		
	}
	return 0;
}

#else

#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))
static void stk_ps_int_handle_int_mode_2_3(struct stk3a5x_data *ps_data)
{
	uint32_t reading;
	int32_t near_far_state;
	
#if (STK_INT_PS_MODE	== 0x03)
	near_far_state = gpio_get_value(ps_data->int_pin);
#elif	(STK_INT_PS_MODE == 0x02)
	near_far_state = !(gpio_get_value(ps_data->int_pin));
#endif		
	reading = stk3a5x_get_ps_reading(ps_data);
	stk_ps_report(ps_data, near_far_state);
	PRINT_WARN( "%s: ps input event=%d, ps code=%d\n",__func__, near_far_state, reading);	
}
#endif

static void stk_ps_int_handle(struct stk3a5x_data *ps_data, uint32_t ps_reading, int32_t nf_state)
{
	stk_ps_report(ps_data, nf_state);
	printk(KERN_EMERG "%s: ps input event=%d, ps code=%d\n",__func__, nf_state, ps_reading);	
}

#endif	// #ifdef STK_TUNE1

static int stk_als_int_handle(struct stk3a5x_data *ps_data, uint32_t als_reading)
{
	int32_t als_comperator;	
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD	
	uint32_t nLuxIndex;	
#endif
	int lux;
	
#ifndef CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD
	nLuxIndex = stk_get_lux_interval_index(als_reading);
	stk3a5x_set_als_thd_h(ps_data, code_threshold_table[nLuxIndex]);
	stk3a5x_set_als_thd_l(ps_data, code_threshold_table[nLuxIndex-1]);
#else
	stk_als_set_new_thd(ps_data, als_reading);
#endif //CONFIG_STK_PS_ALS_USE_CHANGE_THRESHOLD

	if(ps_data->ir_code)
	{
		if(als_reading < STK_IRC_MAX_ALS_CODE && als_reading > STK_IRC_MIN_ALS_CODE && 
		ps_data->ir_code > STK_IRC_MIN_IR_CODE)
		{
			als_comperator = als_reading * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(ps_data->ir_code > als_comperator)
				ps_data->als_correct_factor = STK_IRC_ALS_CORREC;
			else
				ps_data->als_correct_factor = 1000;
		}
		// PRINT_WARN( "%s: als=%d, ir=%d, als_correct_factor=%d", __func__, als_reading, ps_data->ir_code, ps_data->als_correct_factor);
		ps_data->ir_code = 0;
	}

		als_reading = als_reading * ps_data->als_correct_factor / 1000;
		ps_data->als_code_last = als_reading;

		lux = stk_alscode2lux(ps_data, als_reading);
		stk_als_report(ps_data, lux);		
	return 0;
}

static void stk_work_func(struct work_struct *work)
{
    int32_t ret;	
#ifndef STK_TUNE1	
	int32_t near_far_state;	
#endif	
#if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02))
    uint8_t disable_flag = 0;
    int32_t org_flag_reg;
#endif	/* #if ((STK_INT_PS_MODE != 0x03) && (STK_INT_PS_MODE != 0x02)) */
	struct stk3a5x_data *ps_data = container_of(work, struct stk3a5x_data, stk_work);	
	uint32_t reading;	
	
#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))
	stk_ps_int_handle_int_mode_2_3(ps_data);
#else
	/* mode 0x01 or 0x04 */	
	org_flag_reg = stk3a5x_get_flag(ps_data);
	if(org_flag_reg < 0)
		goto err_i2c_rw;	
#ifdef STK_DEBUG_PRINTF	
	//PRINT_WARN( "stk3a5x %s: flag=0x%x\n", __func__, org_flag_reg);
#endif	
    if(org_flag_reg & STK_FLG_ALSINT_MASK)
    {
		disable_flag |= STK_FLG_ALSINT_MASK;
		reading = stk3a5x_get_als_reading(ps_data);
		if(reading < 0)
			goto err_i2c_rw;
		ret = stk_als_int_handle(ps_data, reading);
		if(ret < 0)
			goto err_i2c_rw;
    }
    if (org_flag_reg & STK_FLG_PSINT_MASK)
    {
		disable_flag |= STK_FLG_PSINT_MASK;
		reading = stk3a5x_get_ps_reading(ps_data);		
		if(reading < 0)
			goto err_i2c_rw;
		ps_data->ps_code_last = reading;

#ifdef STK_TUNE1
		ret = stk_tune1_ps_int_handle(ps_data, reading);
		if(ret < 0)
			goto err_i2c_rw;		
#else
		near_far_state = (org_flag_reg & STK_FLG_NF_MASK)?1:0;	
#ifdef CTTRACKING
			if (near_far_state == 0) {
				if(reading > (ps_data->psi + STK_H_PS)) {
					ps_data->ps_thd_h = ps_data->psi + STK_H_HT;
					ps_data->ps_thd_l = ps_data->psi + STK_H_LT;
					stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
					stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
					ps_data->ps_thd_update = true;
					printk( "stk  near update HT=%d, LT=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l);
				}
			} else {
				printk( "stk  far ps_update=%d\n", ps_data->ps_thd_update);
				if (ps_data->ps_thd_update) {
					ret = stk_ps_val(ps_data);
					if (ret == 0) {
						if((reading + ps_data->stk_ht_n_ct) < ps_data->ps_thd_h) {
							ps_data->ps_thd_h = reading + ps_data->stk_ht_n_ct;
							ps_data->ps_thd_l = reading + ps_data->stk_lt_n_ct;
							stk3a5x_set_ps_thd_h(ps_data, ps_data->ps_thd_h);
							stk3a5x_set_ps_thd_l(ps_data, ps_data->ps_thd_l);
							printk( "stk  far update HT=%d, LT=%d, ps=%d\n", ps_data->ps_thd_h, ps_data->ps_thd_l, reading);
						}
					}
					ps_data->ps_thd_update = false;
				}
			}
#endif
		stk_ps_int_handle(ps_data, reading, near_far_state);
#endif
	}

	if(disable_flag)	
	{
		ret = stk3a5x_set_flag(ps_data, org_flag_reg, disable_flag);		
		if(ret < 0)
			goto err_i2c_rw;
	}

#endif
	usleep_range(1000, 2000);
    enable_irq(ps_data->irq);
	return;

err_i2c_rw:
	msleep(30);
	enable_irq(ps_data->irq);
	return;	
}

static irqreturn_t stk_oss_irq_handler(int irq, void *data)
{
	struct stk3a5x_data *pData = data;
	//printk(KERN_EMERG "%s \n",__func__);
	//add arvin_liao at 20190417
	__pm_wakeup_event(stk_psensor_timeout_wakelock, msecs_to_jiffies(STK_WAKELOCK_TIMEOUT_INT_MS));
	disable_irq_nosync(irq);
	queue_work(pData->stk_wq,&pData->stk_work);
	return IRQ_HANDLED;
}
#endif	/*	#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))	*/

#ifdef STK_POLL_ALS
static void stk3a5x_als_set_poll_delay(struct stk3a5x_data *ps_data)
{
	uint8_t als_it = ps_data->alsctrl_reg & 0x0F;
		
	if(als_it == 0x8)
	{
		ps_data->als_poll_delay = ns_to_ktime(60 * NSEC_PER_MSEC);	
	}
	else if(als_it == 0x9)
	{
		ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	}
	else if(als_it == 0xA)
	{
		ps_data->als_poll_delay = ns_to_ktime(220 * NSEC_PER_MSEC);		
	}
	else if(als_it == 0xB)
	{
		ps_data->als_poll_delay = ns_to_ktime(440 * NSEC_PER_MSEC);		
	}
	else if(als_it == 0xC)
	{
		ps_data->als_poll_delay = ns_to_ktime(880 * NSEC_PER_MSEC);		
	}
	else
	{
		ps_data->als_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);		
		PRINT_WARN( "%s: unknown ALS_IT=%d, set als_poll_delay=110ms\n", __func__, als_it);
	}	
}
#endif

static int32_t stk3a5x_init_all_setting(struct i2c_client *client, struct stk3a5x_platform_data *plat_data)
{
	int32_t ret;
	struct stk3a5x_data *ps_data = i2c_get_clientdata(client);		
	
	ret = stk3a5x_software_reset(ps_data); 
	if(ret < 0)
		return ret;
	
	ret = stk3a5x_check_pid(ps_data);
	if(ret < 0)
		return ret;
	stk3a5x_proc_plat_data(ps_data, plat_data);
	ret = stk3a5x_init_all_reg(ps_data);
	if(ret < 0)
		return ret;
#ifdef STK_POLL_ALS
	stk3a5x_als_set_poll_delay(ps_data);
#endif	
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
	#ifdef QUALCOMM_PLATFORM		
	ps_data->tune_zero_init_proc = false;	
	ps_data->psi_set = 0;		
	#else
	stk_ps_tune_zero_init(ps_data);
	#endif	
#endif	
#ifdef STK_ALS_FIR
	memset(&ps_data->fir, 0x00, sizeof(ps_data->fir));  
	atomic_set(&ps_data->firlength, STK_FIR_LEN);	
#endif
	atomic_set(&ps_data->recv_reg, 0);  
#ifdef STK_GES	
	ps_data->re_enable_ges = 0;	
	atomic_set(&ps_data->gesture2, 0);  
	//memset(stk_ges_op, 0, sizeof(stk_ges_op));
#endif	
#ifdef STK_IRS
	ps_data->als_data_index = 0;
#endif	
	ps_data->ps_distance_last = -1;
	ps_data->als_code_last = 100;
#ifdef STK_TUNE1	
	ps_data->stk_int_handle_state = STK_PSINT_NORM;
#endif
	PRINT_WARN("stk3a5x_init_all_setting\n");
    return 0;
}

#if (!defined(STK_POLL_PS) || !defined(STK_POLL_ALS))
static int stk3a5x_setup_irq(struct i2c_client *client)
{		
	int irq, err = -EIO;
	struct stk3a5x_data *ps_data = i2c_get_clientdata(client);
	
	err = gpio_request(ps_data->int_pin,"stk-int");        
	if(err < 0)
	{
		printk(KERN_EMERG "%s: gpio_request, err=%d", __func__, err);
		return err;
	}
	// gpio_tlmm_config(GPIO_CFG(ps_data->int_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), GPIO_CFG_ENABLE);	
	
	err = gpio_direction_input(ps_data->int_pin);
	if(err < 0)
	{
		printk(KERN_EMERG "%s: gpio_direction_input, err=%d", __func__, err);
		return err;
	}		

	irq = gpio_to_irq(ps_data->int_pin);

#ifdef STK_DEBUG_PRINTF	
	PRINT_WARN( "%s: int pin #=%d, irq=%d\n",__func__, ps_data->int_pin, irq);	
#endif	
	if (irq <= 0)
	{
		printk(KERN_EMERG "irq number is not specified, irq # = %d, int pin=%d\n",irq, ps_data->int_pin);
		return irq;
	}
	ps_data->irq = irq;	

#if ((STK_INT_PS_MODE == 0x03) || (STK_INT_PS_MODE	== 0x02))	
	//err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, DEVICE_NAME, ps_data);
	err = request_irq(irq, stk_oss_irq_handler, IRQ_TYPE_LEVEL_LOW, DEVICE_NAME, ps_data);
#else	
	//err = request_any_context_irq(irq, stk_oss_irq_handler, IRQF_TRIGGER_FALLING, DEVICE_NAME, ps_data);
	err = request_irq(irq, stk_oss_irq_handler, IRQ_TYPE_LEVEL_LOW, DEVICE_NAME, ps_data);
#endif	
	if (err < 0) 
	{
		printk(KERN_EMERG "%s: request_any_context_irq(%d) failed for (%d)\n", __func__, irq, err);		
		goto err_request_any_context_irq;
	}
	enable_irq_wake(irq);
	//disable_irq(irq);

#if 0
	err = gpio_request(ps_data->vcc_pin,"stk-vcc");
	if(err < 0)
	{
		printk(KERN_EMERG "%s: vcc_gpio_request, err=%d", __func__, err);
		return err;
	}
	err = gpio_direction_output(ps_data->vcc_pin, 1);
	if(err < 0)
	{
		printk(KERN_EMERG "%s: vcc_gpio_direction_input, err=%d", __func__, err);
		return err;
	}	

	err = gpio_request(ps_data->en_pin,"stk-en");
	if(err < 0)
	{
		printk(KERN_EMERG "%s: en_gpio_request, err=%d", __func__, err);
		return err;
	}
	err = gpio_direction_output(ps_data->en_pin, 1);
	if(err < 0)
	{
		printk(KERN_EMERG "%s: en_gpio_direction_input, err=%d", __func__, err);
		return err;
	}
#endif
	
	return 0;
err_request_any_context_irq:	
	
	gpio_free(ps_data->int_pin);		

	return err;
}
#endif

#if 0
static int stk3a5x_suspend(struct device *dev)
{
	struct stk3a5x_data *ps_data = dev_get_drvdata(dev);
#if (defined(STK_CHK_REG) || !defined(STK_POLL_PS))
	int err;
#endif
	
#ifndef STK_POLL_PS	
    struct i2c_client *client = to_i2c_client(dev);	
#endif

	PRINT_WARN( "%s\n", __func__);	
#ifndef SPREADTRUM_PLATFORM	
	mutex_lock(&ps_data->io_lock);  	
#endif	
#ifdef STK_CHK_REG
	err = stk3a5x_validate_n_handle(ps_data->client);
	if(err < 0)	
	{
		printk(KERN_ERR "stk3a5x_validate_n_handle fail: %d\n", err); 
	}
	else if (err == 0xFF)
	{
		if(ps_data->ps_enabled)
			stk3a5x_enable_ps(ps_data, 1, 0);
	}
#endif /* #ifdef STK_CHK_REG */
#ifdef STK_GES	
	if(ps_data->ges_enabled == 1)
	{
		ps_data->re_enable_ges = ps_data->ges_enabled;		
		stk3a5x_enable_ges(ps_data, 0, 1);		
	}
	else if(ps_data->ges_enabled == 2)
	{
		ps_data->re_enable_ges = ps_data->ges_enabled;	
		stk3a5x_enable_ges(ps_data, 0, 2);
	}
#endif	
#ifndef SPREADTRUM_PLATFORM	
	if(ps_data->als_enabled)
	{
		PRINT_WARN( "%s: Enable ALS : 0\n", __func__);
		stk3a5x_enable_als(ps_data, 0);		
		ps_data->re_enable_als = true;
	}
#endif	
	if(ps_data->ps_enabled)
	{
#ifdef STK_POLL_PS		
		wake_lock(ps_data->ps_nosuspend_wl);
#else		
		if(device_may_wakeup(&client->dev))
		{
			err = enable_irq_wake(ps_data->irq);	
			if (err)
				printk(KERN_WARNING "%s: set_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);				
		}
		else
		{
			printk(KERN_ERR "%s: not support wakeup source\n", __func__);
		}
#endif
	}
#ifndef SPREADTRUM_PLATFORM		
	mutex_unlock(&ps_data->io_lock);		
#endif	
	return 0;	
}

static int stk3a5x_resume(struct device *dev)
{
	struct stk3a5x_data *ps_data = dev_get_drvdata(dev);	
#if (defined(STK_CHK_REG) || !defined(STK_POLL_PS))	
	int err;
#endif
#ifndef STK_POLL_PS	    
    struct i2c_client *client = to_i2c_client(dev);	
#endif
	
	PRINT_WARN( "%s\n", __func__);	
#ifndef SPREADTRUM_PLATFORM		
	mutex_lock(&ps_data->io_lock); 		
#endif	
#ifdef STK_CHK_REG
	err = stk3a5x_validate_n_handle(ps_data->client);
	if(err < 0)	
	{
		printk(KERN_ERR "stk3a5x_validate_n_handle fail: %d\n", err); 
	}
	else if (err == 0xFF)
	{
		if(ps_data->ps_enabled)
			stk3a5x_enable_ps(ps_data, 1, 0);
	}	
#endif /* #ifdef STK_CHK_REG */
#ifdef STK_GES	
	if(ps_data->re_enable_ges == 1)
	{
		stk3a5x_enable_ges(ps_data, 1, 1);
		ps_data->re_enable_ges = 0;		
	}
	else if(ps_data->re_enable_ges == 2)
	{
		stk3a5x_enable_ges(ps_data, 1, 2);		
		ps_data->re_enable_ges = 0;				
	}
#endif
#ifndef SPREADTRUM_PLATFORM		
	if(ps_data->re_enable_als)
	{
		PRINT_WARN( "%s: Enable ALS : 1\n", __func__);		
		stk3a5x_enable_als(ps_data, 1);		
		ps_data->re_enable_als = false;		
	}
#endif

	if(ps_data->ps_enabled)
	{
#ifdef STK_POLL_PS		
		wake_unlock(ps_data->ps_nosuspend_wl);		
#else		
		if(device_may_wakeup(&client->dev))
		{	
			err = disable_irq_wake(ps_data->irq);	
			if (err)		
				printk(KERN_WARNING "%s: disable_irq_wake(%d) failed, err=(%d)\n", __func__, ps_data->irq, err);		
		}		
#endif	
	}
#ifndef SPREADTRUM_PLATFORM			
	mutex_unlock(&ps_data->io_lock);
#endif	
	return 0;	
}
#endif

static int stk3a5x_open(struct inode *inode, struct file *file)
{
    //PRINT_WARN( "stk3a5x_open\n");
    return 0;
}

static int stk3a5x_release(struct inode *inode, struct file *file)
{
    //PRINT_WARN( "stk3a5x_release\n");
    return 0;
}

static long stk3a5x_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int flag;//, err;
    //char strbuf[256];
    struct stk3a5x_data *ps_data =  i2c_get_clientdata(this_client);

    printk(KERN_EMERG "stk3a5x_ioctl 1919 cmd = %d, %d\n", _IOC_NR(cmd), cmd);
    switch (cmd) {
    case LTR_IOCTL_SET_PFLAG: //enable/disable ps
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
                return -EFAULT;

            printk(KERN_EMERG "LTR_IOCTL_SET_PFLAG = %d\n", flag);

            if (1 == flag) {
                //msleep(20);
                if (stk3a5x_enable_ps(ps_data, 1)){
					printk(KERN_EMERG "stk3a5x_enable_ps 1 error\n");
                	//return -EIO;	//if here return -5 ,then hal layer will can't get the data . anking.he
				}
            }
            else if (0 == flag) {
                if (stk3a5x_enable_ps(ps_data, 0)){
					printk(KERN_EMERG "stk3a5x_enable_ps 0 error\n");
                	//return -EIO;
				}
            }
            else {
                return -EINVAL;
            }
        }
    	break;
    case LTR_IOCTL_SET_LFLAG:// enable/disable als
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
                return -EFAULT;
            printk(KERN_EMERG "LTR_IOCTL_SET_LFLAG = %d\n", flag);

            if (1 == flag) {
                if (stk3a5x_enable_als(ps_data, 1))
                    return -EIO;
            }
            else if (0 == flag) {
                if (stk3a5x_enable_als(ps_data, 0))
                    return -EIO;
            }
            else {
                return -EINVAL;
            }
        }
    	break;
    default:
    	PRINT_ERR( "unknown command: 0x%08X  (%d)\n", cmd, cmd);
    	break;
    }
    return 0;
}

static struct file_operations stk3a5x_fops = {
    .owner = THIS_MODULE,
    .open = stk3a5x_open,
    .release = stk3a5x_release,
    .unlocked_ioctl = stk3a5x_ioctl,
};

static struct miscdevice stk3a5x_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &stk3a5x_fops,
};


//#ifdef CONFIG_HAS_EARLYSUSPEND
#if 0
static void stk3a5x_early_suspend(struct early_suspend *h)
{
	
}

static void stk3a5x_late_resume(struct early_suspend *h)
{

}
#endif	//#ifdef CONFIG_HAS_EARLYSUSPEND


#ifdef STK_QUALCOMM_POWER_CTRL
static int stk3a5x_power_ctl(struct stk3a5x_data *data, bool on)
{
	int ret = 0;

	if (!on && data->power_enabled) {
		ret = regulator_disable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_disable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed ret=%d\n", ret);
			ret = regulator_enable(data->vdd);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator vdd enable failed ret=%d\n",
					ret);
			}			
			return ret;
		}
		data->power_enabled = on;
		PRINT_WARN( "%s: disable stk3a5x power", __func__);
		dev_dbg(&data->client->dev, "stk3a5x_power_ctl on=%d\n",
				on);
	} else if (on && !data->power_enabled) {
		ret = regulator_enable(data->vdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed ret=%d\n", ret);
			return ret;
		}

		ret = regulator_enable(data->vio);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed ret=%d\n", ret);
			regulator_disable(data->vdd);
			return ret;
		}
		data->power_enabled = on;
		PRINT_WARN( "%s: enable stk3a5x power", __func__);
		dev_dbg(&data->client->dev, "stk3a5x_power_ctl on=%d\n",
				on);
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return ret;
}

static int stk3a5x_power_init(struct stk3a5x_data *data, bool on)
{
	int ret;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd,
					0, STK3A5X_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio,
					0, STK3A5X_VIO_MAX_UV);

		regulator_put(data->vio);
	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			ret = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd ret=%d\n", ret);
			return ret;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			ret = regulator_set_voltage(data->vdd,
					STK3A5X_VDD_MIN_UV,
					STK3A5X_VDD_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd ret=%d\n",
					ret);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			ret = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio ret=%d\n", ret);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			ret = regulator_set_voltage(data->vio,
					STK3A5X_VIO_MIN_UV,
					STK3A5X_VIO_MAX_UV);
			if (ret) {
				dev_err(&data->client->dev,
				"Regulator set failed vio ret=%d\n", ret);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, STK3A5X_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return ret;
}

static int stk3a5x_device_ctl(struct stk3a5x_data *ps_data, bool enable)
{
	int ret;
	struct device *dev = &ps_data->client->dev;

	if (enable && !ps_data->power_enabled) {
		ret = stk3a5x_power_ctl(ps_data, true);
		if (ret) {
			dev_err(dev, "Failed to enable device power\n");
			goto err_exit;
		}
		ret = stk3a5x_init_all_setting(ps_data->client, ps_data->pdata);
		if (ret < 0) {
			stk3a5x_power_ctl(ps_data, false);
			dev_err(dev, "Failed to re-init device setting\n");
			goto err_exit;
		}
	} else if (!enable && ps_data->power_enabled) {
	#ifdef STK_GES		
		if (!ps_data->als_enabled && !ps_data->ps_enabled && !ps_data->ges_enabled) {
	#else
		if (!ps_data->als_enabled && !ps_data->ps_enabled) {
	#endif
			ret = stk3a5x_power_ctl(ps_data, false);
			if (ret) {
				dev_err(dev, "Failed to disable device power\n");
				goto err_exit;
			}
		} else {
			dev_dbg(dev, "device control: als_enabled=%d, ps_enabled=%d\n",
				ps_data->als_enabled, ps_data->ps_enabled);
		}
	} else {
		dev_dbg(dev, "device control: enable=%d, power_enabled=%d\n",
			enable, ps_data->power_enabled);
	}
	return 0;

err_exit:
	return ret;
}
#endif	/* #ifdef STK_QUALCOMM_POWER_CTRL */

#ifdef CONFIG_OF
static int stk3a5x_parse_dt(struct device *dev,
			struct stk3a5x_platform_data *pdata)
{	

	int rc;
	struct device_node *np = dev->of_node;
	 	
	//rc = of_get_named_gpio_flags(np, "stk-int",	0, &pdata->int_flags); // the function of_get_named_gpio_flags hava error
	//printk(KERN_EMERG "stk3a5x_parse_dt,int_flag:%d\n",pdata->int_flags);
	#if defined(CONFIG_BOARD_B65)||defined(CONFIG_BOARD_B71)
	rc = of_get_named_gpio(np, "irq_gpio_1", 0); 
	#elif defined(CONFIG_BOARD_C68)||defined(CONFIG_BOARD_C93)
	rc = of_get_named_gpio(np, "irq_gpio_2", 0);
	#else
	rc = of_get_named_gpio(np, "irq_gpio_3", 0);
	#endif
	printk(KERN_EMERG "stk3a5x_parse_dt,rc:%d\n",rc);
	if (rc < 0) {
		printk(KERN_EMERG "Unable to read irq-gpio\n");
		return pdata->int_pin;
	}
	else{
		pdata->int_pin = rc;
	}

#if 0
	rc = of_get_named_gpio(np, "stk-vcc", 0);
	if (rc < 0) {
		printk(KERN_EMERG "Unable to read vcc-gpio\n");
		return pdata->vcc_pin;
	}
	else{
		pdata->vcc_pin = rc;
	}

	rc = of_get_named_gpio(np, "stk-en", 0);
	if (rc < 0) {
		printk(KERN_EMERG "Unable to read en-gpio\n");
		return pdata->en_pin;
	}
	else{
		pdata->en_pin = rc;
	}
#endif
#if 0
	rc = of_property_read_u32(np, "stk,transmittance", &temp_val);
	if (!rc)
		pdata->transmittance = temp_val;
	else {
		dev_err(dev, "Unable to read transmittance\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,state-reg", &temp_val);
	if (!rc)
		pdata->state_reg = temp_val;
	else {
		dev_err(dev, "Unable to read state-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,psctrl-reg", &temp_val);
	if (!rc)
		pdata->psctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read psctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,alsctrl-reg", &temp_val);
	if (!rc)
		pdata->alsctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read alsctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ledctrl-reg", &temp_val);
	if (!rc)
		pdata->ledctrl_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read ledctrl-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,wait-reg", &temp_val);
	if (!rc)
		pdata->wait_reg = (u8)temp_val;
	else {
		dev_err(dev, "Unable to read wait-reg\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdh", &temp_val);
	if (!rc)
		pdata->ps_thd_h = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdh\n");
		return rc;
	}

	rc = of_property_read_u32(np, "stk,ps-thdl", &temp_val);
	if (!rc)
		pdata->ps_thd_l = (u16)temp_val;
	else {
		dev_err(dev, "Unable to read ps-thdl\n");
		return rc;
	}

	pdata->use_fir = of_property_read_bool(np, "stk,use-fir");
#endif
	return 0;
}
#else
static int stk3a5x_parse_dt(struct device *dev,
			struct stk3a5x_platform_data *pdata)
{
	return -ENODEV;
}
#endif /* !CONFIG_OF */

struct stk3a5x_data *m_ps_data;
static int stk3a5x_pm_suspend(struct device *dev)
{	 
#if defined(CONFIG_BOARD_TH98)
	stk3a5x_enable_ps(m_ps_data,0);
#endif	 
	return 0;
}


static int stk3a5x_pm_resume(struct device *dev)
{ 
	#if defined(CONFIG_BOARD_TH98)
	stk3a5x_enable_ps(m_ps_data,1);
	#endif 
	stk3a5x_reinit_all_reg(m_ps_data);
	return 0;
}
#if defined(CONFIG_ADF_SPRD)
static struct notifier_block adf_event_block;

static int ft_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{
	struct adf_notifier_event *event = data;
	int adf_event_data;
	struct device *pdev;

	if (action != ADF_EVENT_BLANK)
		return NOTIFY_DONE;

	adf_event_data = *(int *)event->data;

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		stk3a5x_pm_resume(pdev);
		break;
	case DRM_MODE_DPMS_OFF:
		stk3a5x_pm_suspend(pdev);
		break;
	default:
		/* TS_WARN("receive adf event with error data, adf_event_data=%d",adf_event_data); */
		break;
	}

	return NOTIFY_OK;
}
#endif

static int stk3a5x_set_wq(struct stk3a5x_data *ps_data)
{
#ifdef STK_POLL_ALS	
	ps_data->stk_als_wq = create_singlethread_workqueue("stk_als_wq");
	INIT_WORK(&ps_data->stk_als_work, stk_als_poll_work_func);
	hrtimer_init(&ps_data->als_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->als_poll_delay = ns_to_ktime(250 * NSEC_PER_MSEC);		
	ps_data->als_timer.function = stk_als_timer_func;
#endif	

#ifdef STK_POLL_PS	
	ps_data->stk_ps_wq = create_singlethread_workqueue("stk_ps_wq");
	INIT_WORK(&ps_data->stk_ps_work, stk_ps_poll_work_func);
	hrtimer_init(&ps_data->ps_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_poll_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	ps_data->ps_timer.function = stk_ps_timer_func;
#endif	

#ifdef STK_TUNE0
	ps_data->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&ps_data->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&ps_data->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_data->ps_tune0_delay = ns_to_ktime(110 * NSEC_PER_MSEC);
	ps_data->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif

#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	ps_data->stk_wq = create_singlethread_workqueue("stk_wq");
	INIT_WORK(&ps_data->stk_work, stk_work_func);
#endif
	return 0;
}
#if !defined(CONFIG_INPUT_AP3426_I2C)
struct input_dev *m_all_input_dev;
#endif

static int stk3a5x_set_input_devices(struct stk3a5x_data *ps_data)
{
	int err;
	
	ps_data->als_input_dev = input_allocate_device();
	if (ps_data->als_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate als device\n", __func__);
		err = -ENOMEM;
		return err;
	}
	#if !defined(CONFIG_INPUT_AP3426_I2C)
	m_all_input_dev = ps_data->ps_input_dev;
	#endif
	ps_data->als_input_dev->name = ALS_NAME;
	set_bit(EV_ABS, ps_data->als_input_dev->evbit);
	input_set_abs_params(ps_data->als_input_dev, ABS_MISC, 0, stk_alscode2lux(ps_data, (1<<16)-1), 0, 0);
	input_set_abs_params(ps_data->als_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	err = input_register_device(ps_data->als_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register als input device\n", __func__);		
		goto err_als_input_register;
	}
	
	ps_data->ps_input_dev = input_allocate_device();
	if (ps_data->ps_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);		
		err = -ENOMEM;
		goto err_ps_input_allocate;
	}
	input_set_drvdata(ps_data->als_input_dev, ps_data);
	err = sysfs_create_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);
	if (err < 0) 
	{
		printk(KERN_ERR "%s:could not create sysfs group for als\n", __func__);
		goto err_als_create_group;
	}
	 
	ps_data->ps_input_dev->name = PS_NAME;
	set_bit(EV_ABS, ps_data->ps_input_dev->evbit);	
	input_set_abs_params(ps_data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	err = input_register_device(ps_data->ps_input_dev);	
	if (err<0)
	{
		printk(KERN_EMERG "%s: can not register ps input device\n", __func__);	
		goto err_ps_input_register;
	}
	
	
	err = sysfs_create_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);
	if (err < 0) 
	{
		printk(KERN_EMERG "%s:could not create sysfs group for ps\n", __func__);
		goto err_ps_create_group;
	}
	
	input_set_drvdata(ps_data->ps_input_dev, ps_data);		
	
#ifdef STK_GES
	ps_data->ges_input_dev = input_allocate_device();
	if (ps_data->ges_input_dev == NULL)
	{
		printk(KERN_EMERG "%s: could not allocate ges device\n", __func__);		
		err = -ENOMEM;
		goto err_ges_input_allocate;
	}
	ps_data->ges_input_dev->name = "stk_ges";
	ps_data->ges_input_dev->evbit[0] = BIT_MASK(EV_KEY);
	set_bit(KEY_PAGEUP, ps_data->ges_input_dev->keybit);
	set_bit(KEY_PAGEDOWN, ps_data->ges_input_dev->keybit);
	set_bit(KEY_VOLUMEUP, ps_data->ges_input_dev->keybit);
	set_bit(KEY_VOLUMEDOWN, ps_data->ges_input_dev->keybit);
	/*
	set_bit(KEY_LEFT, ps_data->ges_input_dev->keybit);
	set_bit(KEY_RIGHT, ps_data->ges_input_dev->keybit);
	set_bit(KEY_UP, ps_data->ges_input_dev->keybit);
	set_bit(KEY_DOWN, ps_data->ges_input_dev->keybit);
	*/	
	err = input_register_device(ps_data->ges_input_dev);	
	if (err<0)
	{
		printk(KERN_EMERG "%s: can not register ges input device\n", __func__);	
		goto err_ges_input_register;
	}
	
	err = sysfs_create_group(&ps_data->ges_input_dev->dev.kobj, &stk_ges_attribute_group);
	if (err < 0) 
	{
		printk(KERN_EMERG "%s:could not create sysfs group for ges\n", __func__);
		goto err_ges_sysfs_create;
	}
	input_set_drvdata(ps_data->ges_input_dev, ps_data);	
#endif	
	printk(KERN_EMERG "%s end\n", __func__);	
	return 0;
	
#ifdef STK_GES
err_ges_sysfs_create:
	input_unregister_device(ps_data->ges_input_dev);	
err_ges_input_register:
	// input_free_device(ps_data->ges_input_dev);
err_ges_input_allocate:
#endif
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);	
err_ps_create_group:
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);	
err_als_create_group:
	input_unregister_device(ps_data->ps_input_dev);
	input_unregister_device(ps_data->als_input_dev);
	return err;
err_ps_input_register:	
	input_free_device(ps_data->ps_input_dev);
	input_unregister_device(ps_data->als_input_dev);
	return err;
err_ps_input_allocate:
	input_unregister_device(ps_data->als_input_dev);	
	return err;
err_als_input_register:
	input_free_device(ps_data->als_input_dev);
	return err;
}

static int stk3a5x_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
    int err = -ENODEV;
    struct stk3a5x_data *ps_data;
	struct stk3a5x_platform_data *plat_data;
    printk(KERN_EMERG "%s:121 driver version = %s\n", __func__, DRIVER_VERSION);
	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_EMERG "%s: No Support for I2C_FUNC_I2C\n", __func__);
        return -ENODEV;
    }

	ps_data = kzalloc(sizeof(struct stk3a5x_data),GFP_KERNEL);
	if(!ps_data)
	{
		printk(KERN_EMERG "%s: failed to allocate stk3a5x_data\n", __func__);
		return -ENOMEM;
	}
	ps_data->client = client;
	i2c_set_clientdata(client,ps_data);
	this_client = client;
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

	if (client->dev.of_node) {
		PRINT_WARN( "%s: probe with device tree\n", __func__);
		plat_data = devm_kzalloc(&client->dev,
			sizeof(struct stk3a5x_platform_data), GFP_KERNEL);
		if (!plat_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = stk3a5x_parse_dt(&client->dev, plat_data);
		if (err)
		{
			dev_err(&client->dev,
				"%s: stk3a5x_parse_dt ret=%d\n", __func__, err);
			return err;
		}
		PRINT_WARN( "%s: stk3a5x_parse_dt init\n", __func__);
        plat_data->state_reg = stk3a5x_pfdata.state_reg;
        plat_data->psctrl_reg = stk3a5x_pfdata.psctrl_reg;
        plat_data->alsctrl_reg = stk3a5x_pfdata.alsctrl_reg;
        plat_data->ledctrl_reg = stk3a5x_pfdata.ledctrl_reg;
        plat_data->wait_reg = stk3a5x_pfdata.wait_reg;
        plat_data->ps_thd_h = stk3a5x_pfdata.ps_thd_h;
        plat_data->ps_thd_l = stk3a5x_pfdata.ps_thd_l;
        //plat_data->int_pin = stk3a5x_pfdata.int_pin;
        plat_data->transmittance = stk3a5x_pfdata.transmittance;
	plat_data->vcc_pin = stk3a5x_pfdata.vcc_pin;
	plat_data->en_pin = stk3a5x_pfdata.en_pin;	
		
	} else {
		PRINT_WARN( "%s: probe with platform data\n", __func__);	
#ifdef SPREADTRUM_PLATFORM		
		plat_data = &stk3a5x_pfdata;	
#else
		plat_data = client->dev.platform_data;	
#endif		
	}
	if (!plat_data) {
		dev_err(&client->dev,
			"%s: no stk3a5x platform data!\n", __func__);
		goto err_als_input_allocate;
	}
	ps_data->als_transmittance = plat_data->transmittance;
	ps_data->int_pin = plat_data->int_pin;
	ps_data->vcc_pin = plat_data->vcc_pin;
	ps_data->en_pin = plat_data->en_pin;
	ps_data->pdata = plat_data;

	if (ps_data->als_transmittance == 0) {
		dev_err(&client->dev,
			"%s: Please set als_transmittance\n", __func__);
		goto err_als_input_allocate;
	}	
	
	stk3a5x_set_wq(ps_data);
#ifdef QUALCOMM_PLATFORM
	ps_data->ps_thd_h = 0;
	ps_data->ps_thd_l = 0;	
#endif

#ifdef STK_TUNE0
	ps_data->stk_max_min_diff = STK_MAX_MIN_DIFF;
	ps_data->stk_lt_n_ct = STK_LT_N_CT;
	ps_data->stk_ht_n_ct = STK_HT_N_CT;
#endif

#ifdef STK_QUALCOMM_POWER_CTRL	
		
	err = stk3a5x_power_init(ps_data, true);
	if (err)
		goto err_power_on;

	err = stk3a5x_power_ctl(ps_data, true);
	if (err)
		goto err_power_on;
	msleep(3);
	err = stk3a5x_check_pid(ps_data);
	if(err < 0)
		goto err_init_all_setting;
		
	ps_data->als_enabled = false;
	ps_data->ps_enabled = false;
#else
	err = stk3a5x_init_all_setting(client, plat_data);
	if(err < 0)
		goto err_init_all_setting;	
#endif	
	
	err = stk3a5x_set_input_devices(ps_data);
	if(err < 0)
		goto err_setup_input_device;	
	//wakeup_source_init(stk_psensor_timeout_wakelock, STK_WAKELOCK_INT_NAME);//add arvin_liao at 20190417	
	stk_psensor_timeout_wakelock  = wakeup_source_register(NULL, STK_WAKELOCK_INT_NAME);
	if (!stk_psensor_timeout_wakelock) {
			return -ENOMEM;
	}	
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	err = stk3a5x_setup_irq(client);
	if(err < 0)
		goto err_stk3a5x_setup_irq;
#endif		

	device_init_wakeup(&client->dev, true);

    err = misc_register(&stk3a5x_device);
    if (err) {
        printk(KERN_EMERG "misc_register failed!\n");
    }

	
//#ifdef CONFIG_HAS_EARLYSUSPEND
#if 0
	ps_data->stk_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ps_data->stk_early_suspend.suspend = stk3a5x_early_suspend;
	ps_data->stk_early_suspend.resume = stk3a5x_late_resume;
	register_early_suspend(&ps_data->stk_early_suspend);
#endif

#ifdef QUALCOMM_PLATFORM
	/* make sure everything is ok before registering the class device */
	ps_data->als_cdev = sensors_light_cdev;
#ifdef STK_POLL_ALS
	sensors_light_cdev.min_delay = ktime_to_us(ps_data->als_poll_delay);
#else
	sensors_light_cdev.min_delay = 0;
#endif	
	ps_data->als_cdev.sensors_enable = stk_als_enable_set;
	err = sensors_classdev_register(&ps_data->als_input_dev->dev, &ps_data->als_cdev);
	if (err)
		goto err_stk3a5x_setup_irq;

	ps_data->ps_cdev = sensors_proximity_cdev;
	ps_data->ps_cdev.sensors_enable = stk_ps_enable_set;
	err = sensors_classdev_register(&ps_data->ps_input_dev->dev, &ps_data->ps_cdev);
	if (err)
		goto err_class_sysfs;
#endif

#ifdef STK_QUALCOMM_POWER_CTRL	
	/* enable device power only when it is enabled */
	err = stk3a5x_power_ctl(ps_data, false);
	if (err)
		goto err_power_ctl;
#endif		
#if defined(CONFIG_BOARD_TH98)
	stk3a5x_enable_ps(ps_data,1);
#endif
	m_ps_data = ps_data;
#ifdef CONFIG_PM_SLEEP
	#if defined(CONFIG_ADF_SPRD)
	adf_event_block.notifier_call = ft_adf_event_handler;
	adf_register_client(&adf_event_block);
	#endif
#endif
	printk(KERN_EMERG  "%s: probe successfully", __func__);
	return 0;

	//device_init_wakeup(&client->dev, false);
#ifdef STK_QUALCOMM_POWER_CTRL	
err_power_ctl:
#endif
#ifdef QUALCOMM_PLATFORM
	sensors_classdev_unregister(&ps_data->ps_cdev);
err_class_sysfs:
	sensors_classdev_unregister(&ps_data->als_cdev);
#endif
err_stk3a5x_setup_irq:
	//wakeup_source_trash(&stk_psensor_timeout_wakelock);//add arvin_liao at 20190417
    if (stk_psensor_timeout_wakelock) {
			wakeup_source_unregister(stk_psensor_timeout_wakelock);
			stk_psensor_timeout_wakelock = NULL;
	}	
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	free_irq(ps_data->irq, ps_data);		
	gpio_free(ps_data->int_pin);		
#endif
#ifdef STK_GES
	sysfs_remove_group(&ps_data->ges_input_dev->dev.kobj, &stk_ges_attribute_group);	
	input_unregister_device(ps_data->ges_input_dev);	
#endif
	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);	
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);	
	input_unregister_device(ps_data->ps_input_dev);		
	input_unregister_device(ps_data->als_input_dev);
err_setup_input_device:
err_init_all_setting:
#ifdef STK_QUALCOMM_POWER_CTRL	
	stk3a5x_power_ctl(ps_data, false);
err_power_on:	
	stk3a5x_power_init(ps_data, false);
#endif	
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


static int stk3a5x_remove(struct i2c_client *client)
{
	struct stk3a5x_data *ps_data = i2c_get_clientdata(client);
	//wakeup_source_trash(&stk_psensor_timeout_wakelock);//add arvin_liao at 20190417
    if (stk_psensor_timeout_wakelock) {
			wakeup_source_unregister(stk_psensor_timeout_wakelock);
			stk_psensor_timeout_wakelock = NULL;
	}	
	device_init_wakeup(&client->dev, false);
#ifdef STK_QUALCOMM_POWER_CTRL	
	stk3a5x_power_ctl(ps_data, false);
#endif	
#ifdef QUALCOMM_PLATFORM
	sensors_classdev_unregister(&ps_data->ps_cdev);
	sensors_classdev_unregister(&ps_data->als_cdev);
#endif	
#ifdef STK_QUALCOMM_POWER_CTRL		
	stk3a5x_power_init(ps_data, false);
#endif	
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	free_irq(ps_data->irq, ps_data);	
	gpio_free(ps_data->int_pin);		
#endif	/* #if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS)) */	


#ifdef STK_GES	
	sysfs_remove_group(&ps_data->ges_input_dev->dev.kobj, &stk_ges_attribute_group);	
	input_unregister_device(ps_data->ges_input_dev);
#endif

	sysfs_remove_group(&ps_data->ps_input_dev->dev.kobj, &stk_ps_attribute_group);	
	sysfs_remove_group(&ps_data->als_input_dev->dev.kobj, &stk_als_attribute_group);	
	input_unregister_device(ps_data->ps_input_dev);		
	input_unregister_device(ps_data->als_input_dev);	

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
#if (!defined(STK_POLL_ALS) || !defined(STK_POLL_PS))
	destroy_workqueue(ps_data->stk_wq);	
#endif	
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

static const struct dev_pm_ops stk3a5x_pm_ops = {
	SET_RUNTIME_PM_OPS(stk3a5x_pm_suspend, stk3a5x_pm_resume, NULL)
};
static const struct i2c_device_id stk_ps_id[] =
{
    { "DEVICE_NAME", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, stk_ps_id);

static struct of_device_id stk_match_table[] = {
	{ .compatible = "stk,stk3a5x", },
	{ },
};

static struct i2c_driver stk_ps_driver =
{
    .driver = {
        .name = DEVICE_NAME,
		.owner = THIS_MODULE,	
#ifdef CONFIG_OF		
		.of_match_table = stk_match_table,		
#endif		
		.pm = &stk3a5x_pm_ops,		
    },
    .probe = stk3a5x_probe,
    .remove = stk3a5x_remove,
    .id_table = stk_ps_id,
};


static int __init stk3a5x_init(void)
{
	int ret;
    ret = i2c_add_driver(&stk_ps_driver);
    if (ret)
	{
		i2c_del_driver(&stk_ps_driver);
        return ret;
	}
    return 0;
}

static void __exit stk3a5x_exit(void)
{
    i2c_del_driver(&stk_ps_driver);	
}

module_init(stk3a5x_init);
module_exit(stk3a5x_exit);
MODULE_AUTHOR("Lex Hsieh <lex_hsieh@sensortek.com.tw>");
MODULE_DESCRIPTION("Sensortek stk3a5x Proximity Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

