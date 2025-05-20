/*
 * drivers/input/touchscreen/ft5306_ts.c
 *
 * FocalTech ft5306 TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */

/* if vdd is not supply,please using this code
	reg_vdd = regulator_get(&client->dev, "vdd28");
	regulator_set_voltage(reg_vdd, 2800000, 2800000);
	regulator_enable(reg_vdd);
*/

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include "ft5306_ts.h"
#include "focaltech.h"
#include "focaltech_ex_fun.h"
#include "focaltech_ctl.h"

#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
#include "revo_sensorhub.h"
#endif

#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
static const struct do_sensorhub* sensorhub_prox_do = NULL ;
#endif

#define APK_DEBUG

#ifndef  SPRD_AUTO_UPGRADE
#if defined(CONFIG_PROJ_C55_CHUANQI_8085AR)
	#define SPRD_AUTO_UPGRADE
#elif defined(PRJ_FEATURE_H_BOARD_TOUCHSCREEN_FW_UPGRADE_FT5306)
	#define SPRD_AUTO_UPGRADE
#endif
#endif /* SPRD_AUTO_UPGRADE */
    
#define SYSFS_DEBUG
#define FTS_CTL_IIC

#if defined(PRJ_FEATURE_H_BOARD_HAVE_PLS_TP)||defined(SENSORHUB_PROXIMITY_TOUCHSCREEN)
#define CONFIG_TOUCHSCREEN_WITH_PROXIMITY
#endif

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
static DECLARE_WAIT_QUEUE_HEAD(open_wq);
static atomic_t  prox_contrl_state;
struct task_struct *proximity_thread;

#endif
struct Upgrade_Info fts_updateinfo[] = {
	{0x64,"FT6x36U",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x1c, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
	{0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 20, 2000},
};

struct Upgrade_Info fts_updateinfo_curr;

#if defined(PRJ_FEATURE_H_BOARD_GESTURE_ENABLE)&&!defined(FT5306_DEL_GESTURE)
	#define FTS_GESTRUE
#endif

#if defined(FTS_GESTRUE) || defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
static struct wakeup_source *wakeup_wakelock;
#endif

#ifdef FTS_GESTRUE

#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_ARROW_LEFT    0x51  //左箭号(0xe8?)
#define GESTURE_ARROW_RIGHT   0x52  //右箭号
#define GESTURE_ARROW_UP      0x53  //上箭号
#define GESTURE_ARROW_DOWN    0x54  //下箭号(字母V)
#define GESTURE_Z             0x41
#define GESTURE_S							0x46
#define GESTURE_U		    			0x47


#define FTS_GESTRUE_POINTS	255

#include "ft_gesture_lib.h"
short pointnum = 0;
unsigned long time_stamp = 0;
bool is_one_finger = true;
int data_xx[150] = {0};
int data_yy[150] = {0};
static int *datax = NULL;
static int *datay = NULL;
#endif

#if defined(TOUCHSCREEN_TLSC_ESD_HELPER_ENABLE)
#define TLSC_ESD_HELPER_EN  // esd helper, close:undef
#endif


extern int ft5306_do_tlsc6x_mmbin_newif(void);
extern unsigned int ft5306_do_get_tlsc6x_keepbin(void);
#ifdef TLSC_ESD_HELPER_EN
int g_is_telink_comp = 0;
extern int ft5306_do_tlsc6x_tp_dect(struct i2c_client * client); 
#endif

#define CONFIG_FT5306_MULTITOUCH 1
//#define    CONFIG_FT5306_UPDATE_FW_WITH_I_FILE   0
static int debug_level=0;

#define TS_DBG(format, ...)	\
	if(debug_level == 1)	\
		printk(KERN_INFO "FT5306 " format "\n", ## __VA_ARGS__)

struct sprd_i2c_setup_data {
	unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
	unsigned short i2c_address;
	int irq;
	char type[I2C_NAME_SIZE];
};

static struct i2c_client *this_client;
/* Attribute */
static int in_suspend = 0; //0: sleep out; 1: sleep in

static spinlock_t irq_lock;
static s32 irq_is_disable;
#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
static int s_nearfar = 1;
#endif

static int sprd_3rdparty_gpio_tp_rst = 0;
static int sprd_3rdparty_gpio_tp_irq = 0;

static unsigned char ft5306_read_fw_ver(void);
static int ft5306_ts_pm_suspend(struct device *dev);
static int ft5306_ts_pm_resume(struct device *dev);
#if defined(CONFIG_FT5306_UPDATE_FW_WITH_I_FILE)
static int fts_ctpm_fw_upgrade_with_i_file(void);
#endif

struct ts_event {
	u16	x1;
	u16	y1;
	u16	id1;
	u16	x2;
	u16	y2;
	u16	id2;
	u16	x3;
	u16	y3;
	u16	id3;
	u16	x4;
	u16	y4;
	u16	id4;
	u16	x5;
	u16	y5;
	u16	id5;
	u16	pressure;
    u8  touch_point;
};

struct ft5306_ts_data {
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	struct ts_event	event;
	struct work_struct	pen_event_work;
	struct workqueue_struct	*ts_workqueue;
	struct work_struct	 resume_work;
	struct workqueue_struct	*ts_resume_workqueue;
	u8 chip_id;
};



static int ft5306_i2c_rxdata(char *rxdata, int length)
{
	int ret;
	struct i2c_msg msgs[2];
	
	msgs[0].addr	= this_client->addr;
	msgs[0].flags	= 0;
	msgs[0].len	= 1;
	msgs[0].buf	= rxdata;

	
	msgs[1].addr	= this_client->addr;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].len	= length;
	msgs[1].buf	= rxdata;
	
	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int ft5306_i2c_txdata(char *txdata, int length)
{
	int ret;
	struct i2c_msg msg[1];
	
	msg[0].addr	= this_client->addr;
	msg[0].flags	= 0;
	msg[0].len	= length;
	msg[0].buf	= txdata;

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5306_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5306

***********************************************************************************************/
static int ft5306_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5306_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}


/***********************************************************************************************
Name	:	ft5306_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5306

***********************************************************************************************/
static int ft5306_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	struct i2c_msg msgs[2];
	u8 buf[2] = {0};

	buf[0] = addr;

	msgs[0].addr	= this_client->addr;
	msgs[0].flags	= 0;
	msgs[0].len	= 1;
	msgs[0].buf	= buf;

	
	msgs[1].addr	= this_client->addr;
	msgs[1].flags	= I2C_M_RD;
	msgs[1].len	= 1;
	msgs[1].buf	= buf;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
}

static void ft5306_ts_reset(void)
{
    if(ft5306_do_get_tlsc6x_keepbin()){
        return;
    }

    gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
    msleep(1);
    gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 0);
    msleep(10);
    gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
    msleep(50);

#if defined(CONFIG_PROJ_B64_ZHUOXT)
    FTP_GPIO_OUTPUT(sprd_3rdparty_gpio_tp_irq, 0);
    msleep(10);
    gpio_direction_input(sprd_3rdparty_gpio_tp_irq);
    msleep(10);
#endif
}

static void tlsc6x_ts_reset(void)
{
    gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 0);
    msleep(10);
    gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
    msleep(30);

#if defined(CONFIG_PROJ_B64_ZHUOXT)
    FTP_GPIO_OUTPUT(sprd_3rdparty_gpio_tp_irq, 0);
    msleep(10);
    gpio_direction_input(sprd_3rdparty_gpio_tp_irq);
    msleep(10);
#endif
}
void tp_irq_disable(void)
{
	unsigned long irqflags;
	spin_lock_irqsave(&irq_lock, irqflags);
	if (!irq_is_disable)
	{
		irq_is_disable = 1;
		disable_irq_nosync(this_client->irq);
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}

void tp_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&irq_lock, irqflags);
	if (irq_is_disable)
	{
		enable_irq(this_client->irq);
		irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&irq_lock, irqflags);
}


#ifdef FTS_GESTRUE
static int tpd_gesture_handle(void)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 4] = { 0 };
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;
	int keycode = 0;
	int gesture_id = 0;
	unsigned char ruby;
	struct ft5306_ts_data *data = i2c_get_clientdata(this_client);

	ret = fts_i2c_Read(data->client, buf, 1, buf, 8);

	if (ret < 0)
	{
		msleep(200);
		ret = fts_i2c_Read(data->client, buf, 1, buf, 8);

		if (ret < 0)
		{
				printk("qzhu fts_i2c_Read fail\n");
				return ret;
		}
	}

	//printk("qzhu fts_i2c_Read success\n");

	if (0x24 == buf[0])
	{
		printk("qzhu  double click    gesture_id = 0x24\n");
		gesture_id = 0x24;
		#ifdef CONFIT_GESTURE_ONLY_DOUBLECLICK
			input_report_key(data->input_dev,KEY_POWER,1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev,KEY_POWER,0);
			input_sync(data->input_dev);
		#else
			input_report_key(data->input_dev,KEY_UP,1);
			input_sync(data->input_dev);
			input_report_key(data->input_dev,KEY_UP,0);
			input_sync(data->input_dev);
		#endif
		//2015-10-28 KN
		ft5306_ts_reset();

		msleep(350);
		return 0x24;
	}
	else
	{
		//qzhu add for ft6336gu,modify gesture read way
		if (data->chip_id == 0x36 || data->chip_id == 0x64 || data->chip_id == 0x54)
		{
			gesture_id = buf[0];
		}
		else
		{
			pointnum = (short)(buf[1]) & 0xff;
			buf[0] = 0xd3;
			ret = fts_i2c_Read(data->client, buf, 1, buf,pointnum*4+8);

			gesture_id = fetch_object_sample(buf,pointnum);
		}
	}

	switch(gesture_id)
	{
		case GESTURE_LEFT:
		keycode =KEY_F12;
		printk("FTS_gesture FTS  KEY_LEFT\n");
		break;
		case GESTURE_RIGHT:
		keycode = KEY_F11;
		printk("FTS_gesture  KEY_RIGHT\n");
		break;
		case GESTURE_UP:
		keycode = KEY_F10;
		printk("FTS_gesture KEY_UP\n");
		break;
		case GESTURE_DOWN:
		keycode = KEY_F9;
		printk("FTS_gesture KEY_DOWN\n");
		break;
		case GESTURE_O:
		keycode = KEY_O;
		printk("FTS_gesture KEY_O\n");
		break;
		case GESTURE_W:
		keycode = KEY_W;
		printk("FTS_gesture KEY_W\n");
		break;
		case GESTURE_M:
		keycode = KEY_M;
		printk("FTS_gesture KEY_M\n");
		break;
		case GESTURE_E:
		keycode = KEY_E;
		printk("FTS_gesture KEY_E\n");
		break;
		case GESTURE_C:
		keycode = KEY_C;
		printk("FTS_gesture KEY_C\n");
		break;
		case GESTURE_ARROW_LEFT:
		printk("FTS_gesture ARROW_LEFT\n");
		keycode = KEY_KPLEFTPAREN;
		break;
		case GESTURE_ARROW_RIGHT:
		printk("FTS_gesture ARROW_RIGHT\n");
		keycode = KEY_KPRIGHTPAREN;
		break;
		case GESTURE_ARROW_UP:
		printk("FTS_gesture ARROW_UP\n");
		keycode = KEY_SCROLLUP;
		break;
		case GESTURE_ARROW_DOWN:
		printk("FTS_gesture ARROW_DOWN\n");
		keycode = KEY_SCROLLDOWN;
		break;

		case GESTURE_S:
		printk("FTS_gesture S\n");
		keycode = KEY_S;
		break;

		case GESTURE_U:
		printk("FTS_gesture U\n");
		keycode = KEY_U;
		break;

		case GESTURE_Z:
		printk("FTS_gesture Z\n");
		keycode = KEY_Z;
		break;

		case 0x24:

		#ifdef CONFIT_GESTURE_ONLY_DOUBLECLICK
		keycode = KEY_POWER;
		#else
		keycode = KEY_UP;
		#endif
		printk("FTS_gesture double click\n");
		break;
		case KEY_POWER:
		keycode = KEY_POWER;
		printk("FTS_gesture KEY_POWER\n");
		break;
		default:
		keycode = 0;
		printk("FTS_gesture default\n");
		break;
	}

	if(keycode > 0){
		input_report_key(data->input_dev, keycode, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, keycode, 0);
		input_sync(data->input_dev);
		msleep(250);
	}
	else
	{
	    unsigned char uc_reg_value;
	    ft5306_read_reg(0xd0, &uc_reg_value);

	    if(uc_reg_value != 0x1)
	    {
	      printk("qzhu ft5206 not in gesture mode\n");
	      ft5306_write_reg(0xd0, 0x1);
	    }

	    msleep(200);
	}

	return keycode;
}
#endif


#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
/*******************************enable property start************************************/
#ifndef SENSORHUB_PROXIMITY_TOUCHSCREEN
static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&prox_contrl_state));
}

static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);

	if(on_off==0)
	{
		printk("proximity is disable\n");
		ft5306_write_reg(0xb0,0x00);
		atomic_set(&prox_contrl_state,0);
	}
	else
	{
		int ret=0;
		printk("proximity is enable \n");
		ret=ft5306_write_reg(0xb0,0x01);

		if(in_suspend)
		{
			printk("%s: working in suspend mode, shoud resume first!!!\n", __func__);
			ft5306_ts_reset();
			ret=ft5306_write_reg(0xb0,0x01);
			tp_irq_enable();
			in_suspend = 0;
		}

		if(ret < 0)
			printk("qzhu ft5306_write_reg fail\n");

		atomic_set(&prox_contrl_state,1);
		wake_up_interruptible(&open_wq);
	}

  return size;
}

static DEVICE_ATTR(enable, 0644,proximity_enable_show, proximity_enable_store);
#endif

static ssize_t proximity_nearfar_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", s_nearfar?"far":"near");
}



static DEVICE_ATTR(nearfar, 0644,proximity_nearfar_show, NULL);

static struct attribute *proximity_attributes[] = {
#ifndef SENSORHUB_PROXIMITY_TOUCHSCREEN
	&dev_attr_enable.attr,
#endif
	&dev_attr_nearfar.attr,
	NULL
};

static const struct attribute_group proximity_attr_group = {
	.attrs = proximity_attributes,
};
/*******************************enable property end************************************/

/*******************************report proximity data start*******************************/
static int ft5306_report_proximity_data(void)
{
	u8 buf[32] = {0};
	int ret = -1;
	u8 sensor_data_status = 0;

	// printk("qzhu ft5306_report_proximity_data \n");
	ret = ft5306_i2c_rxdata(buf, 31);

	if (ret < 0)
	{
		printk("qzhu %s failed: %d\n", __func__, ret);
		return ret;
	}

	mdelay(50);

	sensor_data_status= buf[1]&0xf0;

	ret = ft5306_i2c_rxdata(buf, 31);

	if (ret < 0)
	{
		printk("qzhu %s failed: %d\n", __func__, ret);
		return ret;
	}

	// printk("qzhu sensor_data_status is 0x%x\n",sensor_data_status);
	if(sensor_data_status == (buf[1]&0xf0))
	{
		struct ft5306_ts_data *data = i2c_get_clientdata(this_client);
    	#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
        if(sensorhub_prox_do && sensorhub_prox_do->prox_reprot_func){
            switch(sensor_data_status){
                case 0xc0 :
				    printk("proximity is near\n");
				    s_nearfar = 0;
                    sensorhub_prox_do->prox_reprot_func(data->input_dev,SENSORHUB_FACE_STATUS_NEAR);
                    break;
                case 0xe0 :
				    printk("proximity is far\n");
				    s_nearfar = 1;
                    sensorhub_prox_do->prox_reprot_func(data->input_dev,SENSORHUB_FACE_STATUS_FAR);
                    break;
                default:
				    ft5306_write_reg(0xb0,0x01);    // working int wrong mode?
                    break ;
            }
        }
        #else /* End of SENSORHUB_PROXIMITY_TOUCHSCREEN */
		//struct ft5306_ts_data *data = i2c_get_clientdata(this_client);
		switch (sensor_data_status) {
			case 0xe0: //far
				printk("proximity is far\n");
				s_nearfar = 1;
				input_report_abs(data->input_dev, ABS_DISTANCE, 1);
				input_sync(data->input_dev);
				break;

			case 0xc0: //near
				printk("proximity is near\n");
				s_nearfar = 0;
				input_report_abs(data->input_dev, ABS_DISTANCE, 0);
				input_sync(data->input_dev);
				break;
			default:
				ft5306_write_reg(0xb0,0x01);    // working int wrong mode?
				break;
		}
		#endif
	}
	return 0;
}
static int proximity_work(void *data)
{
	while(!kthread_should_stop())
	{
		wait_event_interruptible(open_wq, (atomic_read(&prox_contrl_state)==1));

		if(atomic_read(&prox_contrl_state)==1)
		{
			ft5306_report_proximity_data();
		}
	}

	return 0;
}
/*******************************report proximity data end*******************************/
#endif


static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
#if defined(CONFIG_B55_SPREADTRUM)
	return sprintf(buf,
   		 __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT) ":60:1320:75:200"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":360:1320:75:200"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) 			":600:1320:75:200"
	 "\n");
#else
	return sprintf(buf,
   		 __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT)   		":40:1500:75:200"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":80:1500:75:200"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) 			":120:1500:75:200"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) 		":160:1500:75:200"
	 "\n");
#endif
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = FT5306_VIRTUAL_KEY,
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

#define VIRTUAL_BOARD_PROPERTIES_SUPPORT
#if defined(TOUCHPANEL_NAME_SHOW)
#define VIRTUAL_TP_TYPE             TOUCHPANEL_NAME_SHOW
#else
#define VIRTUAL_TP_TYPE             "FT5X06"
#endif
#define VIRTUAL_TP_NAME           "tp_name"

unsigned char cmd_write(unsigned char btcmd,unsigned char btPara1,unsigned char btPara2,unsigned char btPara3,unsigned char num);
unsigned char byte_read(unsigned char* pbt_buf, unsigned char bt_len);
static ssize_t tp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
        unsigned char reg_val[4] = {0} ,i;
        static unsigned char fw_ver = FT5306_REG_FIRMID;
        static char * tp_name = NULL ;
        struct ft5306_ts_data  *ft5306_ts = i2c_get_clientdata(this_client);
        
        if( tp_name == NULL ){
            for( i = 0 ; i < sizeof(fts_updateinfo)/sizeof(fts_updateinfo[0]); i++){
                if( fts_updateinfo[i].CHIP_ID == ft5306_ts->chip_id ) {
                    tp_name = fts_updateinfo[i].FTS_NAME ;
                    break ;
                }
            }
            if( tp_name == NULL )
                tp_name = VIRTUAL_TP_TYPE ;
        }
        
        if( fw_ver == FT5306_REG_FIRMID){
            ft5306_read_reg(FT5306_REG_FIRMID, &fw_ver);
            if(0xfe == fw_ver) {
                cmd_write(0x72,0x65,0x56,0x2a,4); //0x2a566572
                byte_read(reg_val,4);
                fw_ver = reg_val[3]>>2;
            }
        }
        
        return sprintf(buf, "TP_NAME: %s(%x)\n"    "FW_VER=0x%x\n" , tp_name ,ft5306_ts->chip_id , fw_ver);
}

static struct kobj_attribute virtual_tp_name_attr = {
    .attr = {
        .name = VIRTUAL_TP_NAME,
        .mode = S_IRUGO|S_IWUSR,
    },
    .show = &tp_name_show,
};

#ifdef FTS_GESTRUE

#if defined(PRJ_FEATURE_H_REVO_B_SMART_WAKE_OPEN_BY_DEFAULT)
static int wake_gesture = 1;
#else
static int wake_gesture = 0;
#endif
#define VIRTUAL_WAKE_GESTURE 		"wake_gesture"
static ssize_t tp_wake_gesture_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", wake_gesture);
}

static ssize_t tp_wake_gesture_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t size)
{
	unsigned int on_off = simple_strtoul(buf, NULL, 10);
	wake_gesture = on_off > 0 ? 1 : 0;
	return size;
}

static struct kobj_attribute virtual_wake_gesture_attr = {
	.attr = {
		.name = VIRTUAL_WAKE_GESTURE,
		.mode = 0664,
	},
	.show = &tp_wake_gesture_show,
	.store = &tp_wake_gesture_store,
};
#endif

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
#ifdef VIRTUAL_BOARD_PROPERTIES_SUPPORT
    &virtual_tp_name_attr.attr,
#endif
#ifdef FTS_GESTRUE
	&virtual_wake_gesture_attr.attr,
#endif
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void ft5306_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;

    printk("%s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

/***********************************************************************************************
Name	:	 ft5306_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/

static unsigned char ft5306_read_fw_ver(void)
{
	unsigned char ver;
	unsigned char reg_val[4] = {0};
	ft5306_read_reg(FT5306_REG_FIRMID, &ver);
	if(0xfe == ver) {
		cmd_write(0x72,0x65,0x56,0x2a,4); //0x2a566572
		byte_read(reg_val,4);
		ver = reg_val[3]>>2;
	}
	return(ver);
}


#define CONFIG_SUPPORT_FTS_CTP_UPG


#ifdef CONFIG_SUPPORT_FTS_CTP_UPG

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70

/*
[function]:
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;

    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]:
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;
    btPara2[in]    :parameter 2;
    btPara3[in]    :parameter 3;
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]:
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]:
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]:
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        128


E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5306_write_reg(0xfc,0xaa);
    msleep(50);
     /*write 0x55 to register 0xfc*/
    ft5306_write_reg(0xfc,0x55);
    printk("[TSP] Step 1: Reset CTPM test, bin-length=%d\n",dw_lenth);

    msleep(100);

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5306_i2c_txdata(auc_i2c_write_buf, 2);
        msleep(5);
    }while(i_ret <= 0 && i < 5 );
    /*********Step 3:check READ-ID***********************/
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        printk("%s: ERR_READID, ID1 = 0x%x,ID2 = 0x%x\n", __func__,reg_val[0],reg_val[1]);
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);

    msleep(1500);
    printk("[TSP] Step 4: erase.\n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade.\n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        msleep(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        msleep(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        msleep(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        printk("%s: ERR_ECC\n", __func__);
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    return ERR_OK;
}

#if defined(CONFIG_FT5306_UPDATE_FW_WITH_I_FILE)
static int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    ft5306_write_reg(0, 0x40);
    msleep(100);   //make sure already enter factory mode
    ft5306_write_reg(2, 0x4);  //write command to start calibration
    msleep(300);
    for(i=0;i<100;i++)
    {
        ft5306_read_reg(0,&uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        msleep(200);
        printk("[FTS] waiting calibration %d\n",i);
    }
    printk("[FTS] calibration OK.\n");

    msleep(300);
    ft5306_write_reg(0, 0x40);  //goto factory mode
    msleep(100);   //make sure already enter factory mode
    ft5306_write_reg(2, 0x5);  //store CLB result
    msleep(300);
    ft5306_write_reg(0, 0x0); //return to normal mode
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
	printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
       //error handling ...
       //TBD
   }
   else
   {
       printk("[FTS] upgrade successfully.\n");
       fts_ctpm_auto_clb();  //start auto CLB
   }

   return i_ret;
}
#endif

#endif

static void ft5306_ts_release(void)
{
	struct ft5306_ts_data *data = i2c_get_clientdata(this_client);
#ifdef CONFIG_FT5306_MULTITOUCH
	//input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#else
	input_report_abs(data->input_dev, ABS_PRESSURE, 0);
	input_report_key(data->input_dev, BTN_TOUCH, 0);
#endif
	input_sync(data->input_dev);
	TS_DBG("%s",__func__);
}


static int ft5306_read_data(void)
{
	struct ft5306_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
//	u8 buf[14] = {0};
	u8 buf[32] = {0};
	int ret = -1;

#ifdef CONFIG_FT5306_MULTITOUCH
//	ret = ft5306_i2c_rxdata(buf, 13);
	ret = ft5306_i2c_rxdata(buf, 31);
#else
    ret = ft5306_i2c_rxdata(buf, 7);
#endif
    if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
//	event->touch_point = buf[2] & 0x03;// 0000 0011
	event->touch_point = buf[2] & 0x07;// 000 0111

    if (event->touch_point == 0) {
        ft5306_ts_release();
        return 1;
    }

#ifdef CONFIG_FT5306_MULTITOUCH
#if 1
    switch (event->touch_point) {
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
			event->id5 = (s16)buf[29]>>4;
			TS_DBG("===x5 = %d,y5 = %d ====",event->x5,event->y5);
		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			event->id4 = (s16)buf[23]>>4;
			TS_DBG("===x4 = %d,y4 = %d ====",event->x4,event->y4);
		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			event->id3 = (s16)buf[17]>>4;
			TS_DBG("===x3 = %d,y3 = %d ====",event->x3,event->y3);
		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
			event->id2 = (s16)buf[11]>>4;
			TS_DBG("===x2 = %d,y2 = %d ====",event->x2,event->y2);
		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			event->id1 = (s16)buf[5]>>4;
			TS_DBG("===x1 = %d,y1 = %d ====",event->x1,event->y1);
            break;
		default:
		    return -1;
	}
#else
    switch (event->touch_point) {
		case 5:
			event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
			event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
			TS_DBG("===x5 = %d,y5 = %d ====",event->x5,event->y5);
		#if defined(CONFIG_MACH_SP6825GA) || defined(CONFIG_MACH_SP6825GB) || defined(CONFIG_MACH_SP8825GB) || defined(CONFIG_MACH_SP8825GA)
			event->x5 = event->x5*8/9;
			event->y5 = event->y5*854/960;
		#else   //for CONFIG_MACH_SP8825EB QHD
			event->x5 = event->x5*8/9;
			event->y5 = event->y5*854/1020;
		#endif
			TS_DBG("===x5 = %d,y5 = %d ====",event->x5,event->y5);
		case 4:
			event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
			event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
			TS_DBG("===x4 = %d,y4 = %d ====",event->x4,event->y4);
		#if defined(CONFIG_MACH_SP6825GA) || defined(CONFIG_MACH_SP6825GB) || defined(CONFIG_MACH_SP8825GB) || defined(CONFIG_MACH_SP8825GA)
			event->x4 = event->x4*8/9;
			event->y4 = event->y4*854/960;
		#else   //for CONFIG_MACH_SP8825EB QHD
			event->x4 = event->x4*8/9;
			event->y4 = event->y4*854/1020;
		#endif
			TS_DBG("===x4 = %d,y4 = %d ====",event->x4,event->y4);
		case 3:
			event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
			event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
			TS_DBG("===x3 = %d,y3 = %d ====",event->x3,event->y3);
		#if defined(CONFIG_MACH_SP6825GA) || defined(CONFIG_MACH_SP6825GB) || defined(CONFIG_MACH_SP8825GB) || defined(CONFIG_MACH_SP8825GA)
			event->x3 = event->x3*8/9;
			event->y3 = event->y3*854/960;
		#else   //for CONFIG_MACH_SP8825EB QHD
			event->x3 = event->x3*8/9;
			event->y3 = event->y3*854/1020;
		#endif
			TS_DBG("===x3 = %d,y3 = %d ====",event->x3,event->y3);
		case 2:
			event->x2 = (s16)(buf[9] & 0x0F)<<8 | (s16)buf[10];
			event->y2 = (s16)(buf[11] & 0x0F)<<8 | (s16)buf[12];
			TS_DBG("===x2 = %d,y2 = %d ====",event->x2,event->y2);
		#if defined(CONFIG_MACH_SP6825GA) || defined(CONFIG_MACH_SP6825GB) || defined(CONFIG_MACH_SP8825GB) || defined(CONFIG_MACH_SP8825GA)
			event->x2 = event->x2*8/9;
			event->y2 = event->y2*854/960;
		#else   //for CONFIG_MACH_SP8825EB QHD
			event->x2 = event->x2*8/9;
			event->y2 = event->y2*854/1020;
		#endif
			TS_DBG("===x2 = %d,y2 = %d ====",event->x2,event->y2);
		case 1:
			event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
			event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
			TS_DBG("===x1 = %d,y1 = %d ====",event->x1,event->y1);
		#if defined(CONFIG_MACH_SP6825GA) || defined(CONFIG_MACH_SP6825GB) || defined(CONFIG_MACH_SP8825GB) || defined(CONFIG_MACH_SP8825GA)
			event->x1 = event->x1*8/9;
			event->y1 = event->y1*854/960;
		#else   //for CONFIG_MACH_SP8825EB QHD
			event->x1 = event->x1*8/9;
			event->y1 = event->y1*854/1020;
		#endif
			TS_DBG("===x1 = %d,y1 = %d ====",event->x1,event->y1);
            break;
		default:
		    return -1;
	}
#endif
#else
    if (event->touch_point == 1) {
        event->x1 = (s16)(buf[3] & 0x0F)<<8 | (s16)buf[4];
        event->y1 = (s16)(buf[5] & 0x0F)<<8 | (s16)buf[6];
    }
#endif
    event->pressure = 200;

	return 0;
}

static void ft5306_report_value(void)
{
	struct ft5306_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;

//		printk("==ft5306_report_value =\n");
	if(event->touch_point)
		input_report_key(data->input_dev, BTN_TOUCH, 1);
#ifdef CONFIG_FT5306_MULTITOUCH
	switch(event->touch_point) {
		case 5:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->id5);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x5);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y5);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);

		case 4:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->id4);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x4);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y4);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);

		case 3:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->id3);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x3);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y3);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);

		case 2:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->id2);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);

		case 1:
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->id1);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(data->input_dev);

		default:
//			printk("==touch_point default =\n");
			break;
	}
#else	/* CONFIG_FT5306_MULTITOUCH*/
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	}
	input_report_key(data->input_dev, BTN_TOUCH, 1);
#endif	/* CONFIG_FT5306_MULTITOUCH*/
	input_sync(data->input_dev);
}	/*end ft5306_report_value*/

static void ft5306_ts_pen_irq_work(struct work_struct *work)
{

	int ret = -1;

#ifdef FTS_GESTRUE
	if(in_suspend == 1)
	{
		#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	  	if(atomic_read(&prox_contrl_state) == 0)
		#endif
	  	{
  			if (wake_gesture == 1) {
  				  ret = tpd_gesture_handle();
  	  			if(ret == 0x24){
  	  				tp_irq_enable();
  	  				return;
  	  			}
  			}
	  	}
	}
#endif
	ret = ft5306_read_data();
	if (ret == 0) {
		ft5306_report_value();
	}

	tp_irq_enable();
}

static irqreturn_t ft5306_ts_interrupt(int irq, void *dev_id)
{

	struct ft5306_ts_data *ft5306_ts = (struct ft5306_ts_data *)dev_id;

	tp_irq_disable();
	#if defined(FTS_GESTRUE) || defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(in_suspend == 1)
	{
		printk("ft5306_ts_interrupt wakeup_wakelock\n");
		__pm_wakeup_event(wakeup_wakelock, msecs_to_jiffies(5000));
	}
	#endif
	if (!work_pending(&ft5306_ts->pen_event_work)) {
		queue_work(ft5306_ts->ts_workqueue, &ft5306_ts->pen_event_work);
	}
	return IRQ_HANDLED;
}


#ifdef TLSC_ESD_HELPER_EN 
static int tpd_esd_flag = 0;
static struct hrtimer tpd_esd_kthread_timer;
static DECLARE_WAIT_QUEUE_HEAD(tpd_esd_waiter);
static int esd_checker_handler(void *unused)
{
    u8 test_val;
    int ret = -1;
    ktime_t ktime;

    do{
        wait_event_interruptible(tpd_esd_waiter, tpd_esd_flag != 0);
        tpd_esd_flag = 0;

        ktime = ktime_set(4, 0);
        hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);

        //mutex_lock(&i2c_access);
        ret = ft5306_read_reg(0x0, &test_val);
        //mutex_unlock(&i2c_access);
        if(ret < 0){    //maybe confused by some noise,so retry is make sense.
            msleep(30);
            //mutex_lock(&i2c_access);
            ret = ft5306_read_reg(0x0, &test_val);
            //mutex_unlock(&i2c_access);
            if(ret < 0){
                tlsc6x_ts_reset();
                if(ft5306_do_get_tlsc6x_keepbin()){
                    if(1 != ft5306_do_tlsc6x_mmbin_newif()){
                        tlsc6x_ts_reset();
                    }
                }
            }
        }
    }while(!kthread_should_stop());

    return 0;
}
	
enum hrtimer_restart tpd_esd_kthread_hrtimer_func(struct hrtimer *timer)
{
    tpd_esd_flag = 1;
    wake_up_interruptible(&tpd_esd_waiter);

    return HRTIMER_NORESTART;
}
#endif  // END_OF_DEFINE TLSC_ESD_HELPER_EN

static void ft5306_ts_pm_resume_process(void)
{
    char vccmd[4];
    unsigned short vcdata[3];
    printk("==%s==\n", __FUNCTION__);
    if(ft5306_do_get_tlsc6x_keepbin()){ 
		vccmd[0] = 0x2a;
		vccmd[0] = 0x56;
		vccmd[0] = 0x65;
		vccmd[0] = 0x72;
        fts_i2c_Read(this_client, vccmd, 1, (char *)vcdata, 6);
        if(vcdata[2] != 0x5572){
            if(1 != ft5306_do_tlsc6x_mmbin_newif()){
                tlsc6x_ts_reset();
            }
        }
    }
	ft5306_ts_reset();
	ft5306_write_reg(FT5306_REG_PERIODACTIVE, 7);//about 70HZ

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(atomic_read(&prox_contrl_state) ==1){
		ft5306_write_reg(0xb0,0x01);
	}
#endif

#ifdef FTS_GESTRUE
	if (wake_gesture == 0) {
	  tp_irq_enable();
	}
#else
	tp_irq_enable();
#endif
#ifdef TLSC_ESD_HELPER_EN
    hrtimer_start(&tpd_esd_kthread_timer,  ktime_set(5, 0), HRTIMER_MODE_REL);
#endif // END_OF TLSC_ESD_HELPER_EN
}

static void ft5306_ts_pm_resume_work(struct work_struct *work)
{
	printk("==%s==\n", __FUNCTION__);
	ft5306_ts_pm_resume_process();
}

static int ft5306_ts_pm_suspend(struct device *dev)
{
#if defined(FTS_GESTRUE)
	struct ft5306_ts_data  *ft5306_ts = (struct ft5306_ts_data *)i2c_get_clientdata(this_client);
#endif

  printk("==%s==\n", __FUNCTION__);

#ifdef TLSC_ESD_HELPER_EN
    hrtimer_cancel(&tpd_esd_kthread_timer);
#endif //  END_OF TLSC_ESD_HELPER_EN

	#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	printk("qzhu suspend, prox_contrl_state=%d\n", atomic_read(&prox_contrl_state));
	if(atomic_read(&prox_contrl_state) == 0)
	{
#if defined(FTS_GESTRUE)
		int count = 0,ret = 0;
		if (wake_gesture == 1) {
			enable_irq_wake(this_client->irq);

retry:
			ret = ft5306_write_reg(0xd0, 0x1);
			if(ret < 0 && count++ < 10)
			{
				msleep(30);
				goto retry;
			}

			irq_set_irq_type(this_client->irq,IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT);
		}
		else
		{
  		tp_irq_disable();
  		ft5306_write_reg(FT5306_REG_PMODE, PMODE_HIBERNATE);
		}
#else
		tp_irq_disable();
		ft5306_write_reg(FT5306_REG_PMODE, PMODE_HIBERNATE);
#endif
	} else {
		printk("ft5306_ts_pm_suspend else, prox_contrl_state=%d\n", atomic_read(&prox_contrl_state));
	}

	#else
#if defined(FTS_GESTRUE)
  int count = 0,ret = 0;
	if (wake_gesture == 1) {
		enable_irq_wake(this_client->irq);

retry:
		ret = ft5306_write_reg(0xd0, 0x01);

		if(ret < 0 && count++ < 10)
		{
		  printk("qzhu ret is %d\n",ret);
			msleep(30);
			goto retry;
		}

    if (ft5306_ts->chip_id == 0x54||ft5306_ts->chip_id == 0x64) {
      ft5306_write_reg(0xd1, 0xff);
      ft5306_write_reg(0xd2, 0xff);
      ft5306_write_reg(0xd5, 0xff);
      ft5306_write_reg(0xd6, 0xff);
      ft5306_write_reg(0xd7, 0xff);
      ft5306_write_reg(0xd8, 0xff);
    }

		irq_set_irq_type(this_client->irq,IRQF_TRIGGER_LOW | IRQF_NO_SUSPEND | IRQF_ONESHOT);
	}
	else
	{
	  tp_irq_disable();
		ft5306_write_reg(FT5306_REG_PMODE, PMODE_HIBERNATE);
	}
#else
		tp_irq_disable();
		ft5306_write_reg(FT5306_REG_PMODE, PMODE_HIBERNATE);
#endif
	#endif
	in_suspend = 1;
	return 0;
}

static int ft5306_ts_pm_resume(struct device *dev)
{
#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	printk("qzhu ft5306_ts_pm_resume, prox_contrl_state=%d\n", atomic_read(&prox_contrl_state));
	if(atomic_read(&prox_contrl_state) == 0)
	{
#ifdef FTS_GESTRUE
		if (wake_gesture == 1) {
			irq_set_irq_type(this_client->irq,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
		}
#endif
		ft5306_ts_pm_resume_process();
	} else {
		#ifdef FTS_GESTRUE
		if (wake_gesture == 1) {
			ft5306_ts_reset();
		}
		#endif
		ft5306_write_reg(0xb0,0x01);
		tp_irq_enable();
		printk("ft5306_ts_pm_resume else, prox_contrl_state=%d\n", atomic_read(&prox_contrl_state));
	}
#else
#ifdef FTS_GESTRUE
	if (wake_gesture == 1) {
		irq_set_irq_type(this_client->irq,IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT);
	}
#endif
	ft5306_ts_pm_resume_process();
#endif

	in_suspend = 0;
	return 0;
}

static int ft5306_ts_check(void)
{
	unsigned char uc_reg_value;

	uc_reg_value = ft5306_read_fw_ver();
	if(uc_reg_value == FT5306_REG_FIRMID)
	{
		printk("ft5306 tpd_i2c_probe failed!!\n");
		msleep(300);

		uc_reg_value = ft5306_read_fw_ver();
		if(uc_reg_value == FT5306_REG_FIRMID)
		{
			printk("ft5306 tpd_i2c_probe retry failed!!\n");
			return -1;
		}
	}

	return 0;
}

static int ft5306_request_gpio(void)
{	
	int ret = 0;

	ret = gpio_request(sprd_3rdparty_gpio_tp_rst, "ts_rst");
	if (ret < 0) {
		printk("Failed to request reset gpio:%d, err:%d", sprd_3rdparty_gpio_tp_rst, ret);
		return -1;
	} 
	
	ret = gpio_request(sprd_3rdparty_gpio_tp_irq, "ts_irq");
	if (ret < 0) {
		printk("Failed to request irq gpio:%d, err:%d", sprd_3rdparty_gpio_tp_irq, ret);
		gpio_free(sprd_3rdparty_gpio_tp_rst);
		return -1;
	}
	
  gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
	gpio_direction_input(sprd_3rdparty_gpio_tp_irq);
	this_client->irq=gpio_to_irq(sprd_3rdparty_gpio_tp_irq);
	msleep(10);
	return ret;
}

//static struct kobject *tp_ctrl_kobj = NULL;
static ssize_t ts_suspend_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
  unsigned int input;

  if (kstrtouint(buf, 10, &input))
      return -EINVAL;

  if (input == 1)
      ft5306_ts_pm_suspend(NULL);
  else if (input == 0)
      ft5306_ts_pm_resume(NULL);
  else
      return -EINVAL;

  return count;
}
static DEVICE_ATTR(ts_suspend, 0664, NULL, ts_suspend_store);

static struct attribute *tp_sysfs_attrs[] = {
  &dev_attr_ts_suspend.attr,
  NULL,
};

static struct attribute_group tp_attr_group = {
  .attrs = tp_sysfs_attrs,
};

static int tp_sysfs_init(struct i2c_client *client)
{
#if 0 //old linux 
  tp_ctrl_kobj = kobject_create_and_add("touchscreen", NULL);
  if (!tp_ctrl_kobj){
      dev_err(&client->dev,"Create tp_sysfs_init failed!\n");
      return -ENOMEM;
  }
  return sysfs_create_group(tp_ctrl_kobj, &tp_attr_group);
#else
   int err = 0;
	/* create sysfs debug files	*/
	err = sysfs_create_group(&client->dev.kobj, &tp_attr_group);
	if (err < 0) {
		dev_err(&client->dev, "Fail to create debug files!");
		return -ENOMEM;
	}
	/* convenient access to sysfs node */
	err = sysfs_create_link(NULL, &client->dev.kobj, "touchscreen");
	if (err < 0) {
		dev_err(&client->dev, "Failed to create link!");
		return -ENOMEM;
	}
  return err;
#endif
}

/* **********************************************************

        sensorhub setting

************************************************************ */
#if defined(SENSORHUB_PROXIMITY_TOUCHSCREEN)
static  int sensorhub_prox_ctrl(int enable){
	if(enable==0)
	{
		printk("proximity is disable\n");
		ft5306_write_reg(0xb0,0x00);
		atomic_set(&prox_contrl_state,0);
	}
	else
	{
		int ret=0;
		printk("proximity is enable \n");
		ret=ft5306_write_reg(0xb0,0x01);

		if(in_suspend)
		{
			printk("%s: working in suspend mode, shoud resume first!!!\n", __func__);
			ft5306_ts_reset();
			ret=ft5306_write_reg(0xb0,0x01);
			tp_irq_enable();
			in_suspend = 0;
		}

		if(ret < 0)
			printk("qzhu ft5306_write_reg fail\n");

		atomic_set(&prox_contrl_state,1);
		wake_up_interruptible(&open_wq);
	}
    return 0 ;
}

static int get_prox_enble_stat(void){
    return atomic_read(&prox_contrl_state);
}
static int get_prox_active_stat(void){
    return s_nearfar ? 0:1;
}

static struct sensorhub_setting  sensorhub_info = {
    .sensorhub_prox_ctrl = sensorhub_prox_ctrl ,
    .get_prox_active_stat = get_prox_active_stat ,
    .get_prox_enble_stat = get_prox_enble_stat ,
    .use_wakelock = false ,
    .input = NULL ,
    .ic_name = "ft5306"
};

//static struct do_sensorhub* sensorhub_prox_do = NULL ;

#endif

static int ft5306_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5306_ts_data *ft5306_ts;
	struct input_dev *input_dev;
	int err = 0;
	
#if defined(CONFIG_FT5306_UPDATE_FW_WITH_I_FILE)	
	unsigned char uc_reg_value;
#endif

	u8 chip_id,i;

  if (client->dev.of_node) {
  	struct device_node *np = client->dev.of_node;
  	sprd_3rdparty_gpio_tp_rst = of_get_named_gpio(np, "reset-gpio", 0);

  	if (sprd_3rdparty_gpio_tp_rst < 0) {
  		printk("fail to get reset_gpio_number\n");
  		return -1;
  	}

  	sprd_3rdparty_gpio_tp_irq = of_get_named_gpio(np, "irq-gpio", 0); 
  	if (sprd_3rdparty_gpio_tp_irq < 0) {
  		printk("fail to get irq_gpio_number\n");
  		return -1;
  	}
  }
	
	printk(KERN_INFO "%s: probe\n",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5306_ts = kzalloc(sizeof(*ft5306_ts), GFP_KERNEL);
	if (!ft5306_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	ft5306_ts->client = client;

	err=ft5306_request_gpio();
	if (err < 0) {
		printk("ft5306 request gpio failed");
		goto exit_request_gpio_failed;
	}
	
	msleep(100);
	tlsc6x_ts_reset();

	revo_get_tp_size(&SCREEN_MAX_X, &SCREEN_MAX_Y);
	printk("%s SCREEN_MAX_X[%d],SCREEN_MAX_Y[%d]\n",__func__,SCREEN_MAX_X,SCREEN_MAX_Y);

#ifdef TLSC_ESD_HELPER_EN 	
	g_is_telink_comp = ft5306_do_tlsc6x_tp_dect(client);
	if(g_is_telink_comp){
		ft5306_ts_reset();
	}
#endif  // TLSC_ESD_HELPER_EN	

	if(ft5306_ts_check() < 0)
	{
		printk("ts %s: device check failed\n", __FUNCTION__);
		err=-1;
		goto exit_device_check_failed;
	}
	i2c_set_clientdata(client, ft5306_ts);

#ifdef FTS_GESTRUE
	init_para(SCREEN_MAX_X,SCREEN_MAX_Y,100,0,0);
#endif

#if defined(CONFIG_FT5306_UPDATE_FW_WITH_I_FILE)
	ft5306_read_reg(FT5306_REG_CIPHER, &uc_reg_value);
	if(uc_reg_value != 0x55)
	{
		if(uc_reg_value == 0xa3) {
			msleep(100);
			fts_ctpm_fw_upgrade_with_i_file();
		}
		else {
			printk("chip id error %x\n",uc_reg_value);
			err = -ENODEV;
			goto exit_alloc_data_failed;
		}
	}
#endif

	spin_lock_init(&irq_lock);
	
	ft5306_write_reg(FT5306_REG_PERIODACTIVE, 7);//about 70HZ

	INIT_WORK(&ft5306_ts->pen_event_work, ft5306_ts_pen_irq_work);

	ft5306_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ft5306_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	INIT_WORK(&ft5306_ts->resume_work, ft5306_ts_pm_resume_work);
	ft5306_ts->ts_resume_workqueue = create_singlethread_workqueue("ft5306_ts_pm_resume_work");
	if (!ft5306_ts->ts_resume_workqueue) {
		err = -ESRCH;
		goto create_singlethread_workqueue_resume;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5306_ts_virtual_keys_init();

	ft5306_ts->input_dev = input_dev;

#ifdef CONFIG_FT5306_MULTITOUCH
#ifdef FTS_GESTRUE
  __set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_APPSELECT,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_LEFT,  input_dev->keybit);
	__set_bit(KEY_RIGHT,  input_dev->keybit);
	__set_bit(KEY_UP,  input_dev->keybit);
	__set_bit(KEY_DOWN,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(EV_KEY, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_U);
	input_set_capability(input_dev, EV_KEY, KEY_O);
  input_set_capability(input_dev, EV_KEY, KEY_F8);
  input_set_capability(input_dev, EV_KEY, KEY_F9);
  input_set_capability(input_dev, EV_KEY, KEY_F10);
  input_set_capability(input_dev, EV_KEY, KEY_F11);
  input_set_capability(input_dev, EV_KEY, KEY_F12);
	input_set_capability(input_dev, EV_KEY, KEY_W);
	input_set_capability(input_dev, EV_KEY, KEY_M);
	input_set_capability(input_dev, EV_KEY, KEY_E);
	input_set_capability(input_dev, EV_KEY, KEY_C);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_APPSELECT);
	input_set_capability(input_dev, EV_KEY, KEY_BACK);
	input_set_capability(input_dev, EV_KEY, KEY_KPLEFTPAREN);
	input_set_capability(input_dev, EV_KEY, KEY_KPRIGHTPAREN);
	input_set_capability(input_dev, EV_KEY, KEY_SCROLLUP);
	input_set_capability(input_dev, EV_KEY, KEY_SCROLLDOWN);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_Z);
	input_set_capability(input_dev, EV_KEY, KEY_U);

#else
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_APPSELECT,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
#endif

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#else
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_PRESSURE, input_dev->absbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(KEY_APPSELECT,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAX, 0 , 0);
#endif

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

  	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_dev->name		= INPUT_DEV_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5306_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	
	err = request_threaded_irq(this_client->irq, NULL, ft5306_ts_interrupt, IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND | IRQF_ONESHOT, client->name, ft5306_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5306_probe: request irq failed %d\n",err);
		goto exit_irq_request_failed;
	}

	tp_irq_disable();

#if defined(CONFIG_FT5306_UPDATE_FW_WITH_I_FILE)
	//get some register information
	uc_reg_value = ft5306_read_fw_ver();
	printk("[FST] Firmware version = 0x%x\n", uc_reg_value);
	printk("[FST] New Firmware version = 0x%x\n", CTPM_FW[sizeof(CTPM_FW)-2]);

	if(uc_reg_value != CTPM_FW[sizeof(CTPM_FW)-2])
	{
		fts_ctpm_fw_upgrade_with_i_file();
	}
#endif

  i2c_smbus_read_i2c_block_data(client,FT_REG_CHIP_ID,1,&chip_id);
  printk("%s chip_id = %x\n", __func__, chip_id);
  ft5306_ts->chip_id = chip_id & 0xff;

  for(i=0;i<10;i++)
  {
    if(ft5306_ts->chip_id == 0)
    {
      msleep(200);  
      i2c_smbus_read_i2c_block_data(client,FT_REG_CHIP_ID,1,&chip_id);
      ft5306_ts->chip_id = chip_id & 0xff;
      printk("final chip_id = %x\n", ft5306_ts->chip_id);
    }
    else
      break;
  }
	
	
  for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
  {
      if(chip_id==fts_updateinfo[i].CHIP_ID)
      {
          memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
          break;
      }
  }
  if(i>=sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
  {
    memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
  }


#ifdef FTS_CTL_IIC
if (ft_rw_iic_drv_init(client) < 0)
{
	dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",	__func__);
}
#endif

#ifdef SPRD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(client);
#endif

#ifdef APK_DEBUG
	ft5x0x_create_apk_debug_channel(client);
#endif
#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	proximity_thread = kzalloc(sizeof(struct task_struct), GFP_KERNEL);
	if (!proximity_thread)
		return -ENOMEM;

    init_waitqueue_head(&open_wq);

	proximity_thread = kthread_create(proximity_work,NULL,"proximity_kthread");
	if(IS_ERR(proximity_thread)){
		err = PTR_ERR(proximity_thread);
		goto exit_input_register_device_failed;
	}

	err = sysfs_create_group(&ft5306_ts->input_dev->dev.kobj, &proximity_attr_group);
	if (err) {
		dev_err(&client->dev, "create device file failed!\n");
		err = -EINVAL;
		goto exit_input_register_device_failed;
	}

	atomic_set(&prox_contrl_state,0);

	wake_up_process(proximity_thread);
#endif

#if defined(FTS_GESTRUE) || defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	wakeup_wakelock = wakeup_source_create("wakeup_wakelock");
	wakeup_source_add(wakeup_wakelock);
#endif

#ifdef TLSC_ESD_HELPER_EN
	{    // esd issue: i2c monitor thread
		ktime_t ktime = ktime_set(30, 0);
		if(g_is_telink_comp){
			hrtimer_init(&tpd_esd_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			tpd_esd_kthread_timer.function = tpd_esd_kthread_hrtimer_func;
			hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);	
			kthread_run(esd_checker_handler, 0, "tlsc6x_esd_helper");
		}
	}
#endif // TLSC_ESD_HELPER_EN

  tp_sysfs_init(this_client);
	tp_irq_enable();
#if defined(SENSORHUB_PROXIMITY_TOUCHSCREEN)
    sensorhub_prox_do = tp_hub_sensorhub_do_funcList(TYPE_SENSORHUB_SETTING, &sensorhub_info);
    if(sensorhub_prox_do&& sensorhub_prox_do->sensorhub_init)
        sensorhub_prox_do->sensorhub_init();
#endif

	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(this_client->irq, ft5306_ts);
exit_irq_request_failed:
	cancel_work_sync(&ft5306_ts->resume_work);
	destroy_workqueue(ft5306_ts->ts_resume_workqueue);
create_singlethread_workqueue_resume:
	cancel_work_sync(&ft5306_ts->pen_event_work);
	destroy_workqueue(ft5306_ts->ts_workqueue);
exit_create_singlethread:
	i2c_set_clientdata(client, NULL);
	kfree(ft5306_ts);
exit_device_check_failed:
	kfree(ft5306_ts);
	gpio_free(sprd_3rdparty_gpio_tp_rst);
	gpio_free(sprd_3rdparty_gpio_tp_irq);
exit_request_gpio_failed:
exit_alloc_data_failed:
exit_check_functionality_failed:  
	return err;
}

static int ft5306_ts_remove(struct i2c_client *client)
{

	struct ft5306_ts_data *ft5306_ts = i2c_get_clientdata(client);

	printk("==ft5306_ts_remove=\n");
	free_irq(this_client->irq, ft5306_ts);
	input_unregister_device(ft5306_ts->input_dev);
	kfree(ft5306_ts);
	cancel_work_sync(&ft5306_ts->resume_work);
	destroy_workqueue(ft5306_ts->ts_resume_workqueue);
	cancel_work_sync(&ft5306_ts->pen_event_work);
	destroy_workqueue(ft5306_ts->ts_workqueue);
	i2c_set_clientdata(client, NULL);

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
	if(proximity_thread){
		kthread_stop(proximity_thread);
		proximity_thread=NULL;
	}
#endif

	return 0;
}

static const struct of_device_id ft5306_match_table[] = {
	{.compatible = "focaltech,ft5306",},
	{ },
};

static struct i2c_driver ft5306_ts_driver = {
	.probe		= ft5306_ts_probe,
	.remove		= ft5306_ts_remove,
	.driver	= {
		.name	= FT5306_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ft5306_match_table,
	},
};

static int __init ft5306_ts_init(void)
{
	printk("%s\n", __func__);
	return i2c_add_driver(&ft5306_ts_driver);
}

static void __exit ft5306_ts_exit(void)
{
	printk("%s\n", __func__);
	i2c_del_driver(&ft5306_ts_driver);
}

module_init(ft5306_ts_init);
module_exit(ft5306_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5306 TouchScreen driver");
MODULE_LICENSE("GPL");
