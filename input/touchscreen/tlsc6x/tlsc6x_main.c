/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * * VERSION		DATE			AUTHOR		Note
 *
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
/* #include <soc/sprd/regulator.h> */
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>

#include <linux/proc_fs.h>
#include <linux/file.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <asm/io.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include <linux/string.h>
#include <linux/module.h>
#include "tlsc6x_main.h"
#include "prj/prj_config.h"
#include "tp_common.h"

#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
#include "revo_sensorhub.h"
#endif

#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	0
#define	TS_MAX_FINGER		2

#define MAX_CHIP_ID   (20)
#define TS_NAME		"tlsc6x_ts"

unsigned char tlsc6x_chip_name[MAX_CHIP_ID][20] = {"null", "tlsc6206a", "0x6306", "tlsc6206", "tlsc6324",
"tlsc6332","tlsc6440","tlsc6448","tlsc6432","tlsc6424",
"chsc6306BF","chsc6417","chsc6413","chsc6540"};

int g_is_telink_comp = 0;
static int in_suspend = 0; //0: sleep out; 1: sleep in /*@zkd add*/

struct tlsc6x_data *g_tp_drvdata = NULL;
static struct i2c_client *this_client;
static struct wakeup_source *tlsc6x_wakelock;

DEFINE_MUTEX(i2c_rw_access);

static int tlsc6x_adf_suspend(void);
static int tlsc6x_adf_resume(void);


#ifdef TLSC_ESD_HELPER_EN
static int tpd_esd_flag = 0;
static struct hrtimer tpd_esd_kthread_timer;
static DECLARE_WAIT_QUEUE_HEAD(tpd_esd_waiter);
#endif

#ifdef TLSC_TPD_PROXIMITY
unsigned char tpd_prox_old_state = 0;
static int tpd_prox_active = 0;
static int tpd_prox_suspend = 0;
static int sensorhub_prox_ctrl(int enable);
static int s_nearfar = 1;
#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
static const struct do_sensorhub* sensorhub_prox_do = NULL ;
static struct wakeup_source *proximity_wake_lock;
#endif

#endif


#ifdef TOUCH_VIRTUAL_KEYS

extern unsigned int g_mccode;
static ssize_t tp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	u8 reg[2];
	u8 readBuf[4];
	char *ptr = buf;
	u8 vender_id;
	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	reg[0] = 0x80;
	reg[1] = 0x04;
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 2);
	ptr += sprintf(ptr,"TP_NAME: tlsc6x \n boot_ver[0x%04X],",(readBuf[0]+(readBuf[1]<<8)));

	if (g_mccode == 0) {
		reg[0] = 0xD6;
		reg[1] = 0xE0;
	} else {
		reg[0] = 0x9E;
		reg[1] = 0x00;
	}
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 4);
	ptr += sprintf(ptr,"Cfg_ver[%d]. ", readBuf[3]>>2);

	vender_id = readBuf[1]>>1;
	ptr += sprintf(ptr,"vender[%02d-", vender_id);

	switch (vender_id) {
	case 1:
		ptr += sprintf(ptr,"xufang");
		break;
	case 2:
		ptr += sprintf(ptr,"xuri");
		break;
	case 3:
		ptr += sprintf(ptr,"yuye");
		break;
	case 4:
		ptr += sprintf(ptr,"tianyi");
		break;
	case 5:
		ptr += sprintf(ptr,"minglang");
		break;
	case 6:
		ptr += sprintf(ptr,"duoxinda");
		break;
	case 7:
		ptr += sprintf(ptr,"zhenhua");
		break;
	case 8:
		ptr += sprintf(ptr,"jitegao");
		break;
	case 9:
		ptr += sprintf(ptr,"guangjishengtai");
		break;
	case 10:
		ptr += sprintf(ptr,"shengguang");
		break;
	case 11:
		ptr += sprintf(ptr,"xintiantong");
		break;
	case 12:
		ptr += sprintf(ptr,"xinyou");
		break;
	case 13:
		ptr += sprintf(ptr,"yanqi");
		break;
	case 14:
		ptr += sprintf(ptr,"zhongcheng");
		break;
	case 15:
		ptr += sprintf(ptr,"xinmaoxin");
		break;
	case 16:
		ptr += sprintf(ptr,"zhenzhixin");
		break;
	case 17:
		ptr += sprintf(ptr,"helitai");
		break;
	case 18:
		ptr += sprintf(ptr,"huaxin");
		break;
	case 19:
		ptr += sprintf(ptr,"lihaojie");
		break;
	case 20:
		ptr += sprintf(ptr,"jiandong");
		break;
	case 21:
		ptr += sprintf(ptr,"xinpengda");
		break;
	case 22:
		ptr += sprintf(ptr,"jiake");
		break;
	case 23:
		ptr += sprintf(ptr,"yijian");
		break;
	case 24:
		ptr += sprintf(ptr,"yixing");
		break;
	case 25:
		ptr += sprintf(ptr,"zhongguangdian");
		break;
	case 26:
		ptr += sprintf(ptr,"hongzhan");
		break;
	case 27:
		ptr += sprintf(ptr,"huaxingda");
		break;
	case 28:
		ptr += sprintf(ptr,"dongjianhuanyu");
		break;
	case 29:
		ptr += sprintf(ptr,"dawosi");
		break;
	case 30:
		ptr += sprintf(ptr,"dacheng");
		break;
	case 31:
		ptr += sprintf(ptr,"mingwangda");
		break;
	case 32:
		ptr += sprintf(ptr,"huangze");
		break;
	case 33:
		ptr += sprintf(ptr,"jinxinxiang");
		break;
	case 34:
		ptr += sprintf(ptr,"gaoge");
		break;
	case 35:
		ptr += sprintf(ptr,"zhihui");
		break;
	case 36:
		ptr += sprintf(ptr,"miaochu");
		break;
	case 37:
		ptr += sprintf(ptr,"qicai");
		break;
	case 38:
		ptr += sprintf(ptr,"zhenghai");
		break;
	case 39:
		ptr += sprintf(ptr,"hongfazhan");
		break;
	case 40:
		ptr += sprintf(ptr,"lianchuang");
		break;
	case 41:
		ptr += sprintf(ptr,"saihua");
		break;
	case 42:
		ptr += sprintf(ptr,"keleli");
		break;

	case 43:
		ptr += sprintf(ptr,"weiyi");
		break;
	case 44:
		ptr += sprintf(ptr,"futuo");
		break;
	default:
		ptr += sprintf(ptr,"unknown");
		break;
	}
	ptr += sprintf(ptr,"].\n");

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);

	return (ptr-buf);

}

static struct kobj_attribute virtual_tp_name_attr = {
	.attr = {
		.name = "tp_name",
		.mode = 0444,
	},
	.show = &tp_name_show,
};

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
#define KEY_POS1  KEY_BACK
#define KEY_POS2  KEY_HOMEPAGE
#define KEY_POS3  KEY_APPSELECT

#ifdef TOUCHSCREEN_TLSC6X_VIRTUAL_KEY_POS1_POS3_SWAP
	#undef KEY_POS1
	#undef KEY_POS3
	#define KEY_POS1  KEY_APPSELECT
	#define KEY_POS3  KEY_BACK
#endif

	return sprintf(buf,
	__stringify(EV_KEY)     ":" __stringify(KEY_POS1)           ":40:1500:75:200"
	":" __stringify(EV_KEY) ":" __stringify(KEY_POS2)       ":80:1500:75:200"
	":" __stringify(EV_KEY) ":" __stringify(KEY_POS3)      ":120:1500:75:200"
	"\n");
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		 //.name =  "virtualkeys.tlsc6x_touch",
		 .name =   "virtualkeys.currency_tp",
		 .mode = 0444,
		 },
	.show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	&virtual_tp_name_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static void tlsc6x_virtual_keys_init(void)
{
	int ret = 0;
	struct kobject *properties_kobj;

	TLSC_FUNC_ENTER();

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj) {
		ret = sysfs_create_group(properties_kobj, &properties_attr_group);
	}
	if (!properties_kobj || ret) {
		tlsc_err("failed to create board_properties\n");
	}
}

#endif

/*@zkd add*/
static spinlock_t irq_lock;
static s32 irq_is_disable;
static void tp_irq_disable(void)
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

static void tp_irq_enable(void)
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
/*@zkd add*/

/*
    iic access interface
*/
int tlsc6x_i2c_read_sub(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	if (client == NULL) {
		tlsc_err("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (readlen > 0) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
				 },
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				tlsc_err("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
				tlsc_err("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
				       writelen);
			}else {
				ret = i2c_transfer(client->adapter, &msgs[1], 1);
				if (ret < 0) {
					tlsc_err("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
					tlsc_err("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
					       writelen);
				}
			}
		} else {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				tlsc_err("[IIC]: i2c_transfer(read) error, ret=%d, rlen=%d, wlen=%d!!", ret, readlen,
				       writelen);
			}
		}
	}

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	/* lock in this function so we can do direct mode iic transfer in debug fun */
	mutex_lock(&i2c_rw_access);
	ret = tlsc6x_i2c_read_sub(client, writebuf, writelen, readbuf, readlen);

	mutex_unlock(&i2c_rw_access);

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_write_sub(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	if (client == NULL) {
		tlsc_err("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0) {
			tlsc_err("[IIC]: i2c_transfer(write) error, ret=%d!!\n", ret);
		}
	}

	return ret;

}

/* fail : <0 */
int tlsc6x_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	ret = tlsc6x_i2c_write_sub(client, writebuf, writelen);
	mutex_unlock(&i2c_rw_access);

	return ret;

}

/* fail : <0 */
int tlsc6x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = { 0 };

	buf[0] = regaddr;
	buf[1] = regvalue;

	return tlsc6x_i2c_write(client, buf, sizeof(buf));
}

/* fail : <0 */
int tlsc6x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return tlsc6x_i2c_read(client, &regaddr, 1, regvalue, 1);
}


static void tlsc6x_clear_report_data(struct tlsc6x_data *drvdata)
{
	int i;

	for (i = 0; i < TS_MAX_FINGER; i++) {
#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(drvdata->input_dev, i);
		input_mt_report_slot_state(drvdata->input_dev, MT_TOOL_FINGER, false);
#endif
	}

	input_report_key(drvdata->input_dev, BTN_TOUCH, 0);
#if !MULTI_PROTOCOL_TYPE_B
	input_mt_sync(drvdata->input_dev);
#endif
	input_sync(drvdata->input_dev);
}

#ifdef TLSC_APK_DEBUG
static unsigned char send_data_flag = 0;
static unsigned char get_data_flag = 0;
static int send_count = 0;
static unsigned char get_data_buf[10] = {0};
static char buf_in[1026] = {0};
static char buf_out[1026] = {0};


void get_data_start(unsigned char *local_buf)
{
    u8 writebuf[4];
    
    g_tp_drvdata->esdHelperFreeze = 1;
    
    tlsc6x_set_dd_mode();
    {
        writebuf[0] = 0x9f; 
        writebuf[1] = 0x22; 
        writebuf[2] = local_buf[2]; 
        writebuf[3] = local_buf[3]; 
        tlsc6x_i2c_write(this_client, writebuf, 4);

        writebuf[0] = 0x9f; 
        writebuf[1] = 0x20; 
        writebuf[2] = 61; 
        tlsc6x_i2c_write(this_client, writebuf, 3);

        writebuf[0] = 0x9f; 
        writebuf[1] = 0x24; 
        writebuf[2] = 0x01; 
        tlsc6x_i2c_write(this_client, writebuf, 3);
    }
}

void get_data_stop(void)
{
    u8 writebuf[4];
    
    writebuf[0] = 0x9f; 
    writebuf[1] = 0x22; 
    writebuf[2] = 0xff; 
    writebuf[3] = 0xff; 
    tlsc6x_i2c_write(this_client, writebuf, 4);		
    msleep(20);
    g_tp_drvdata->esdHelperFreeze = 0;
    send_data_flag = 0;
    tlsc6x_set_nor_mode();
}

int tssc_get_debug_info(struct i2c_client *i2c_client, char *p_data)
{
	char writebuf[10] = {0};
	short size = 61;
	static unsigned int cnt;
	unsigned char loop, k;
	unsigned char cmd[2];
	unsigned short check, rel_size;
	char buft[128];
	unsigned short *p16_buf = (unsigned short *)buft;
	cmd[0] = 1;

	loop = 0;
	rel_size = size * 2;
	
	while (loop++ < 2) {
		writebuf[0] = 0x9f; 
		writebuf[1] = 0x26; 
		tlsc6x_i2c_read(i2c_client, writebuf,  2, buft, rel_size + 2);  //124 2->checksum
		for (k = 0, check = 0; k < size; k++) {
			check += p16_buf[k];
		}
		if (check == p16_buf[size]) {
			p16_buf[size] = 0x5555;
			break;
		} else {
			p16_buf[size] = 0xaaaa;
		}
	}
	buft[124] = (cnt) & 0xff;
	buft[125] = (cnt >> 8) & 0xff;
	buft[126] = (cnt >> 16) & 0xff;
	buft[127] = (cnt >> 24) & 0xff;
	cnt++;
        
        memcpy(&buf_in[send_count * 128], buft, (sizeof(char) * 128));
        if(send_count++ >= 7){
            memcpy(buf_out, buf_in, (sizeof(char) * 1024));
            memset(buf_in, 0xff,(sizeof(char) * 1024));
            send_count = 0;
            get_data_flag = 1;
        }

    {
        static unsigned char msk_o;
        unsigned char msk = 0;
        unsigned short *p_point = &p16_buf[61 - 5];
        unsigned char tcnt = buft[61*2 - 2] & 0xf;
        unsigned short x0 = p_point[0];
        unsigned char id0 = (x0 & 0x8000) >> 15;
        unsigned short y0;
        unsigned char id1;
        unsigned short x1;
        unsigned short y1;
        unsigned char mch;
        unsigned char act;

	x0 = x0 & 0x3fff;
	y0 = p_point[1];
	if(x0>0 && y0>0) {
		msk = 1 << id0;
	}
	x1 = p_point[2];
	id1 = (x1 & 0x8000) >> 15;
	x1 = x1 & 0x3fff;
	y1 = p_point[3];
	if(x1>0 && y1>0) {
		msk |= 1 << id1;
	}
	mch = msk ^ msk_o;
	if ((3 == mch) && (1 == tcnt)) {
		tcnt = 0;
		msk = 0;
		mch = msk_o;
		x0 = x1 = 0;
	}
	msk_o = msk;
	memset(p_data, 0xff, 18);
	
	p_data[0] = 0;
	p_data[1] = 0;

    #ifdef TLSC_TPD_PROXIMITY
        if(tpd_prox_active) {
            if(p_point[0]&0x4000) {
                p_data[1] = 0xC0;
            }
            else {
                p_data[1] = 0xE0;
            }
        }
    #endif

	p_data[2] = tcnt;
	act = 0;
	if (x0 > 0 && y0 > 0) {
		act = (0 == (mch & (0x01 << id0))) ? 0x80 : 0;
	} else {
		id0 = !id1;
		act = 0x40;
	}
	p_data[3] = (act | (x0 >> 8));
	p_data[4] = (x0 & 0xff);
	p_data[5] = (id0 << 4) | (y0 >> 8);
	p_data[6] = (y0 & 0xff);
	p_data[7] = 0x0d;
	p_data[8] = 0x10;
	
	if (x1 > 0 && y1 > 0) {
		act = (0 == (mch & (0x01 << id1))) ? 0x80 : 0;
	} else {
		id1 = !id0;
		act = 0x40;
	}
	p_data[9] = (act | (x1 >> 8));
	p_data[10] = (x1 & 0xff);
	p_data[11] = (id1 << 4) | (y1 >> 8);
	p_data[12] = (y1 & 0xff);
	p_data[13] = 0x0d;
	p_data[14] = 0x10;
    }
	return 0;
}
#endif

static int tlsc6x_update_data(void)
{
	struct tlsc6x_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[20] = { 0 };
	int ret = -1;
	int i;/*@zkd add*/
	u16 x, y;
	u8 ft_pressure, ft_size;


  #ifdef TLSC_APK_DEBUG
        if(send_data_flag) {
	    tssc_get_debug_info(this_client,buf);
        }
        else {
            ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 18);
            if (ret < 0) {
            	tlsc_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
            	return ret;
            }
        }
    #else
	ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 18);
	if (ret < 0) {
		tlsc_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
    #endif
    
	

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

#ifdef TLSC_TPD_PROXIMITY
	#if  0
	printk("%s proximity tpd_prox_active=%d touch_point=%d !!\n",  __func__, tpd_prox_active, event->touch_point);
	printk("%s proximity buf[1]=0x%x tpd_prox_old_state=0x%x!!\n", __func__, buf[1], tpd_prox_old_state);
	printk("%s proximity  if buf[1] is {0xc0,0xe0}  then proximity dis is {0,1} \n", __func__);
	#endif

	#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
	if(sensorhub_prox_do && sensorhub_prox_do->prox_reprot_func){
		if (tpd_prox_active && (event->touch_point == 0)) {
			if (((buf[1] == 0xc0) || (buf[1] == 0xe0)) && (tpd_prox_old_state != buf[1])) {
				if (0xC0 == buf[1]){
					printk ("tlsc6x proximity near\n");
					s_nearfar = 0;
					sensorhub_prox_do->prox_reprot_func(g_tp_drvdata->ps_input_dev, SENSORHUB_FACE_STATUS_NEAR);
				}else if (0xE0 == buf[1]){
					printk ("tlsc6x proximity far-away\n");
					s_nearfar = 1;
					sensorhub_prox_do->prox_reprot_func(g_tp_drvdata->ps_input_dev, SENSORHUB_FACE_STATUS_FAR);
				}
			}
			tpd_prox_old_state = buf[1];
		}
	}
	#else //  end of SENSORHUB_PROXIMITY_TOUCHSCREEN
	if (tpd_prox_active && (event->touch_point == 0)) {
		if ((buf[1] == 0xc0) || (buf[1] == 0xe0)) {
			// printk("%s proximity dis is __ %d __ \n", __func__, (buf[1] == 0xc0) ? 0 : 1);
			s_nearfar = (buf[1] == 0xc0) ? 0 : 1;
			input_report_abs(g_tp_drvdata->ps_input_dev, ABS_DISTANCE, s_nearfar);
			input_mt_sync(g_tp_drvdata->ps_input_dev);
			input_sync(g_tp_drvdata->ps_input_dev);
		}
		tpd_prox_old_state = buf[1];
	}
	#endif
#endif

	for (i = 0; i < TS_MAX_FINGER; i++) {
		if ((buf[6 * i + 3] & 0xc0) == 0xc0) {
			continue;
		}
		x = (s16) (buf[6 * i + 3] & 0x0F) << 8 | (s16) buf[6 * i + 4];
		y = (s16) (buf[6 * i + 5] & 0x0F) << 8 | (s16) buf[6 * i + 6];
		ft_pressure = buf[6 * i + 7];
		if (ft_pressure > 127) {
			ft_pressure = 127;
		}
		ft_size = (buf[6 * i + 8] >> 4) & 0x0F;
		if ((buf[6 * i + 3] & 0x40) == 0x0) {
#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6 * i + 5] >> 4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
#else
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, buf[6 * i + 5] >> 4);
#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, 15);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, ft_size);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
#endif
		} else {
#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6 * i + 5] >> 4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#endif
		}
	}
	if (event->touch_point == 0) {
		tlsc6x_clear_report_data(data);
	}
	input_sync(data->input_dev);

	return 0;

}

/*@zkd add*/
static void tlsc6x_irq_work(struct work_struct *work)
{
  tlsc6x_update_data();
  tp_irq_enable();
}
/*@zkd add*/

static irqreturn_t touch_event_thread_handler(int irq, void *devid)
{   /*@zkd add*/
    tp_irq_disable();
#if defined(TLSC_TPD_PROXIMITY) || defined(TLSC_GESTRUE)
    if(in_suspend == 1){
        __pm_wakeup_event(tlsc6x_wakelock, msecs_to_jiffies(5000));
    }
#endif

    if (!work_pending(&g_tp_drvdata->report_event_work)) {
        queue_work(g_tp_drvdata->report_workqueue, &g_tp_drvdata->report_event_work);
    }
    /*@zkd add*/
	//tlsc6x_update_data();

	return IRQ_HANDLED;
}

void tlsc6x_tpd_reset_force(void)
{
	struct tlsc6x_platform_data *pdata = g_tp_drvdata->platform_data;

	TLSC_FUNC_ENTER();
	gpio_direction_output(pdata->reset_gpio_number, 1);
	usleep_range(10000, 11000);
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(20);
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(30);
}

static void tlsc6x_tpd_reset(void)
{
	TLSC_FUNC_ENTER();
	if (g_tp_drvdata->needKeepRamCode) {
		return;
	}

	tlsc6x_tpd_reset_force();
}

static unsigned char real_suspend_flag = 0;

static int tlsc6x_do_suspend(void)
{
	int ret = -1;

	in_suspend = 1;  /*@zkd add*/

	TLSC_FUNC_ENTER();

#ifdef TLSC_ESD_HELPER_EN
	hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TLSC_TPD_PROXIMITY
	if (tpd_prox_active) {
		real_suspend_flag = 0;
		return 0;
	}
#endif
	tp_irq_disable();  /*@zkd add*/
	//disable_irq_nosync(this_client->irq);
	ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
	if (ret < 0) {
		tlsc_err("tlsc6x error::setup suspend fail!\n");
	}
	real_suspend_flag = 1;
	tlsc6x_clear_report_data(g_tp_drvdata);
	return 0;
}

static int tlsc6x_do_resume(void)
{
	TLSC_FUNC_ENTER();
	in_suspend = 0;  /*@zkd add*/

#ifdef TLSC_TPD_PROXIMITY
	if (tpd_prox_active && (real_suspend_flag == 0)) {
		return 0;
	}
#endif

	queue_work(g_tp_drvdata->tp_resume_workqueue, &g_tp_drvdata->resume_work);
	return 0;
}


static int tlsc6x_adf_suspend(void)
{
	TLSC_FUNC_ENTER();

	return tlsc6x_do_suspend();
}

static int tlsc6x_adf_resume(void)
{
	TLSC_FUNC_ENTER();

	return tlsc6x_do_resume();
}

int glb_tlsc6x_adf_suspend(void) { return tlsc6x_adf_suspend();}
int glb_tlsc6x_adf_resume(void)  { return tlsc6x_adf_resume();}

static void tlsc6x_resume_work(struct work_struct *work)
{
	TLSC_FUNC_ENTER();

	tlsc6x_tpd_reset();

	if (g_tp_drvdata->needKeepRamCode) {	/* need wakeup cmd in this mode */
		tlsc6x_write_reg(this_client, 0xa5, 0x00);
	}

#ifdef TLSC_TPD_PROXIMITY
	if(tpd_prox_suspend == 1) {
		tlsc6x_write_reg(this_client, 0xb0, 0x01);
		tpd_prox_suspend = 0;
	}
#endif

	tlsc6x_clear_report_data(g_tp_drvdata);

	tp_irq_enable();  /*@zkd add*/

	#ifdef TLSC_ESD_HELPER_EN
	hrtimer_start(&tpd_esd_kthread_timer, ktime_set(3, 0), HRTIMER_MODE_REL);
	#endif

	real_suspend_flag = 0;
}



static int tlsc6x_hw_init(struct tlsc6x_data *drvdata)
{
	struct tlsc6x_platform_data *pdata = drvdata->platform_data;

	TLSC_FUNC_ENTER();

	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);

	tlsc6x_tpd_reset();
	return 0;
}



static struct tlsc6x_platform_data *tlsc6x_parse_dt(struct device *dev)
{
	int ret;
	struct tlsc6x_platform_data *pdata;
	struct device_node *np = dev->of_node;

	TLSC_FUNC_ENTER();
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct tlsc6x_platform_data");
		return NULL;
	}

	// pdata->x_res_max = SCREEN_MAX_X;
	// pdata->y_res_max = SCREEN_MAX_Y;
	revo_get_tp_size(&pdata->x_res_max, &pdata->y_res_max);
	printk("%s x_res_max[%d],y_res_max[%d]\n",__func__,pdata->x_res_max,pdata->y_res_max);

	pdata->reset_gpio_number = of_get_named_gpio(np, "reset-gpio", 0);
	printk("qzhu reset gpio is %d\n",pdata->reset_gpio_number);
	if (pdata->reset_gpio_number < 0) {
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}

	pdata->irq_gpio_number = of_get_named_gpio(np, "irq-gpio", 0);
  printk("qzhu irq gpio is %d\n",pdata->irq_gpio_number);

	if (pdata->irq_gpio_number < 0) {
		dev_err(dev, "fail to get irq_gpio_number\n");
		goto fail;
	}

  ret = gpio_request(pdata->irq_gpio_number, "tlsc6x_irq");
  if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", pdata->irq_gpio_number, ret);
		return NULL;
	} 
	
  ret = gpio_request(pdata->reset_gpio_number, "tlsc_reset");
  if (ret < 0) {
		printk("Failed to request GPIO:%d, ERRNO:%d", pdata->reset_gpio_number, ret);
		gpio_free(pdata->irq_gpio_number);
		return NULL;
	}
	
  gpio_direction_output(pdata->reset_gpio_number, 1);
  gpio_direction_input(pdata->irq_gpio_number);

	return pdata;

fail:
	kfree(pdata);
	return NULL;
}

#if 0
int tlsc6x_fif_write(char *fname, u8 *pdata, u16 len)
{
	int ret = 0;
	loff_t pos = 0;
	static struct file *pfile = NULL;

	pfile = filp_open(fname, O_TRUNC | O_CREAT | O_RDWR, 0644);
	if (IS_ERR(pfile)) {
		ret = -EFAULT;
		tlsc_err("tlsc6x tlsc6x_fif_write:open error!\n");
	} else {
		tlsc_info("tlsc6x tlsc6x_fif_write:start write!\n");
		ret = (int)kernel_write(pfile, pdata, (size_t)len, &pos);
		vfs_fsync(pfile, 0);
		filp_close(pfile, NULL);
	}
	return ret;
}
#endif

#if defined(TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
extern int tlsx6x_update_running_cfg(u16 *ptcfg);
extern int tlsx6x_update_burn_cfg(u16 *ptcfg);
extern int tlsc6x_load_ext_binlib(u8 *pcode, u16 len);
extern int tlsc6x_update_f_combboot(u8 *pdata, u16 len);
int auto_upd_busy = 0;
/* 0:success */
/* 1: no file OR open fail */
/* 2: wrong file size OR read error */
/* -1:op-fial */
int tlsc6x_proc_cfg_update(u8 *dir, int behave)
{
	int ret = 1;
	u8 *pbt_buf = NULL;
	u32 fileSize;
	static struct file *file = NULL;

	TLSC_FUNC_ENTER();
	tlsc_info("tlsc6x proc-file:%s\n", dir);

	file = filp_open(dir, O_RDONLY, 0);
	if (IS_ERR(file)) {
		tlsc_err("tlsc6x proc-file:open error!\n");
	} else {
		ret = 2;
		fileSize = file->f_op->llseek(file, 0, SEEK_END);
		tlsc_info("tlsc6x proc-file, size:%d\n", fileSize);
		pbt_buf = kmalloc(fileSize, GFP_KERNEL);

		file->f_op->llseek(file, 0, SEEK_SET);
		if (fileSize == kernel_read(file, pbt_buf, fileSize, &file->f_pos)) {
			tlsc_info("tlsc6x proc-file, read ok1!\n");
			ret = 3;
		}

		if (ret == 3) {
			auto_upd_busy = 1;
			tp_irq_disable();  /*@zkd add*/
			msleep(1000);
			__pm_wakeup_event(tlsc6x_wakelock, msecs_to_jiffies(2000));
			if (behave == 0) {
				if (fileSize == 204) {
					ret = tlsx6x_update_running_cfg((u16 *) pbt_buf);
				} else if (fileSize > 0x400) {
					tlsc6x_load_ext_binlib((u8 *) pbt_buf, (u16) fileSize);
				}
			} else if (behave == 1) {
				if (fileSize == 204) {
					ret = tlsx6x_update_burn_cfg((u16 *) pbt_buf);
				} else if (fileSize > 0x400) {
					ret = tlsc6x_update_f_combboot((u8 *) pbt_buf, (u16) fileSize);
				}
				tlsc6x_tpd_reset();
			}
			tp_irq_enable();  /*@zkd add*/
			auto_upd_busy = 0;
		}

		filp_close(file, NULL);

		kfree(pbt_buf);
	}

	return ret;
}

#endif

#ifdef TLSC_APK_DEBUG
unsigned char proc_out_len;
unsigned char proc_out_buf[256];

unsigned char debug_type;
unsigned char iic_reg[2];
unsigned char sync_flag_addr[3];
unsigned char sync_buf_addr[2];
unsigned char reg_len;

static struct proc_dir_entry *tlsc6x_proc_entry = NULL;

static int debug_read(char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	ret = tlsc6x_i2c_read_sub(this_client, writebuf, writelen, readbuf, readlen);

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret > 0) {
		ret = readlen;
	}
	return ret;
}

static int debug_write(char *writebuf, int writelen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	ret = tlsc6x_i2c_write_sub(this_client, writebuf, writelen);

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret > 0) {
		ret = writelen;
	}
	return ret;
}

static int debug_read_sync(char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	int retryTime;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();
	sync_flag_addr[2] = 1;
	ret = tlsc6x_i2c_write_sub(this_client, sync_flag_addr, 3);

	retryTime = 100;
	do {
		ret = tlsc6x_i2c_read_sub(this_client, sync_flag_addr, 2, &sync_flag_addr[2], 1);
		if (ret < 0) {
			mutex_unlock(&i2c_rw_access);
			return ret;
		}
		retryTime--;
	} while (retryTime>0&&sync_flag_addr[2] == 1);
	if(retryTime==0&&sync_flag_addr[2] == 1) {
		mutex_unlock(&i2c_rw_access);
		return -EFAULT;
	}
	if (ret >= 0) {
		/* read data */
		ret = tlsc6x_i2c_read_sub(this_client, sync_buf_addr, 2, readbuf, readlen);
	}

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret > 0) {
		ret = readlen;
	}
	return ret;
}

static int tlsc6x_rawdata_test_3535allch(u8 * buf,int len)
{
	int ret;
	int retryTime;
	u8 writebuf[4];
	buf[len] = '\0';
	ret=0;
	tp_irq_disable();  /*@zkd add*/
	g_tp_drvdata->esdHelperFreeze=1;
	tlsc6x_tpd_reset();
	if (tlsc6x_proc_cfg_update(&buf[2], 0)) {
		ret = -EIO;
	}
	msleep(30);

	mutex_lock(&i2c_rw_access);
	//write addr
	writebuf[0]= 0x9F;
	writebuf[1]= 0x20;
	writebuf[2]= 48;
	writebuf[3]= 0xFF;
	ret = tlsc6x_i2c_write_sub(this_client, writebuf, 4);
	writebuf[0]= 0x9F;
	writebuf[1]= 0x24;
	writebuf[2]= 1;

	ret = tlsc6x_i2c_write_sub(this_client, writebuf, 3);
	retryTime = 10;
	do {
		ret = tlsc6x_i2c_read_sub(this_client, writebuf, 2, &writebuf[2], 1);
		if (ret < 0) {
			break;
		}
		retryTime--;
	} while (retryTime>0&&writebuf[2] == 1);

	if (ret>=0) {
		writebuf[0]= 0x9F;
		writebuf[1]= 0x26;
		ret = tlsc6x_i2c_read_sub(this_client, writebuf, 2, proc_out_buf, 96);
		if (ret>=0){
			proc_out_len=96;
		}
	}

	mutex_unlock(&i2c_rw_access);

	tlsc6x_tpd_reset();



	g_tp_drvdata->esdHelperFreeze=0;
	tp_irq_enable();  /*@zkd add*/

	return ret;
}

static ssize_t tlsc6x_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int ret;
	int buflen = len;
	unsigned char *local_buf;
	if (buflen > 4100) {
		return -EFAULT;
	}
	local_buf = kmalloc(buflen+1, GFP_KERNEL);
	if(local_buf == NULL) {
		tlsc_err("%s,Can not malloc the buf!\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(local_buf, buff, buflen)) {
		tlsc_err("%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	ret = 0;
	debug_type = local_buf[0];
	/* format:cmd+para+data0+data1+data2... */
	switch (local_buf[0]) {
	case 0:		/* cfg version */
		proc_out_len = 4;
		proc_out_buf[0] = g_tlsc6x_cfg_ver;
		proc_out_buf[1] = g_tlsc6x_cfg_ver >> 8;
		proc_out_buf[2] = g_tlsc6x_cfg_ver >> 16;
		proc_out_buf[3] = g_tlsc6x_cfg_ver >> 24;
		break;
	case 1:
		local_buf[buflen] = '\0';
		if (tlsc6x_proc_cfg_update(&local_buf[2], 0)<0) {
			len = -EIO;
		}
		break;
	case 2:
		local_buf[buflen] = '\0';
		if (tlsc6x_proc_cfg_update(&local_buf[2], 1)<0) {
			len = -EIO;
		}
		break;
	case 3:
		ret = debug_write(&local_buf[1], len - 1);
		break;
	case 4:		/* read */
		reg_len = local_buf[1];
		iic_reg[0] = local_buf[2];
		iic_reg[1] = local_buf[3];
		break;
	case 5:		/* read with sync */
		ret = debug_write(&local_buf[1], 4);	/* write size */
		if (ret >= 0) {
			ret = debug_write(&local_buf[5], 4);	/* write addr */
		}
		sync_flag_addr[0] = local_buf[9];
		sync_flag_addr[1] = local_buf[10];
		sync_buf_addr[0] = local_buf[11];
		sync_buf_addr[1] = local_buf[12];
		break;
	case 8: // Force reset ic
		tlsc6x_tpd_reset_force();
		break;
	case 9: // Force reset ic
		ret=tlsc6x_rawdata_test_3535allch(local_buf,buflen);
		break;
	case 14:	/* e, esd control */
		g_tp_drvdata->esdHelperFreeze = (int)local_buf[1];
		break;
    case 15:	
             memset(get_data_buf, 0x00, (sizeof(char) * 8));
             memcpy(get_data_buf, local_buf, (sizeof(char) * 8));
             get_data_start(get_data_buf);

             send_data_flag = 1;
             send_count = 0;
             get_data_flag = 0;
        
		break;
    case 16:
        get_data_stop();
    
		break;
		
	default:
		break;
	}
	if (ret < 0) {
		len = ret;
	}
	kfree(local_buf);
	return len;
}

static ssize_t tlsc6x_proc_read(struct file *filp, char __user *page, size_t len, loff_t *pos)
{
	int ret = 0;
	if (*pos!=0) {
		return 0;
	}
	switch (debug_type) {
	case 0:		/* version information */
		proc_out_len = 4;
		proc_out_buf[0] = g_tlsc6x_cfg_ver;
		proc_out_buf[1] = g_tlsc6x_cfg_ver >> 8;
		proc_out_buf[2] = g_tlsc6x_cfg_ver >> 16;
		proc_out_buf[3] = g_tlsc6x_cfg_ver >> 24;
		if (copy_to_user(page, proc_out_buf, proc_out_len)) {
			ret = -EFAULT;
		} else {
			ret = proc_out_len;
		}
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		len = debug_read(iic_reg, reg_len, proc_out_buf, len);
		if (len > 0) {
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = len;
			}
		} else {
			ret = len;
		}
		break;
	case 5:
		len = debug_read_sync(iic_reg, reg_len, proc_out_buf, len);
		if (len > 0) {
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = len;
			}
		} else {
			ret = len;
		}
		break;
	case 9:
		if (proc_out_buf>0){
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = proc_out_len;
			}
		}
		break;
    case 15:
		if (1 == get_data_flag){  
			get_data_flag = 0;  
			if (copy_to_user(page, buf_out, len)) {
				ret = -EFAULT;   
			} else { 
				ret = len;   
			} 
		}
		else{
			ret = -EFAULT;  
		}  

		break;
	default:
		break;
	}

	if(ret>0) {
		*pos +=ret;
	}

	return ret;
}

static struct file_operations tlsc6x_proc_ops = {
	.owner = THIS_MODULE,
	.read = tlsc6x_proc_read,
	.write = tlsc6x_proc_write,
};

void tlsc6x_release_apk_debug_channel(void)
{
	if (tlsc6x_proc_entry) {
		remove_proc_entry("tlsc6x-debug", NULL);
	}
}

int tlsc6x_create_apk_debug_channel(struct i2c_client *client)
{
	tlsc6x_proc_entry = proc_create("tlsc6x-debug", 0666, NULL, &tlsc6x_proc_ops);

	if (tlsc6x_proc_entry == NULL) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	}
	dev_info(&client->dev, "Create proc entry success!\n");

	return 0;
}
#endif

extern unsigned int g_mccode;
static ssize_t show_tlsc_version(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	u8 reg[2];
	u8 readBuf[4];
	char *ptr = buf;
	u8 vender_id;
	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	reg[0] = 0x80;
	reg[1] = 0x04;
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 2);
	ptr += sprintf(ptr,"The boot version is %04X.\n",(readBuf[0]+(readBuf[1]<<8)));

	if (g_mccode == 0) {
		reg[0] = 0xD6;
		reg[1] = 0xE0;
	} else {
		reg[0] = 0x9E;
		reg[1] = 0x00;
	}
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 4);
	ptr += sprintf(ptr,"The config version is %d.\n", readBuf[3]>>2);

	vender_id = readBuf[1]>>1;
	ptr += sprintf(ptr,"The vender id is %d, the vender name is ", vender_id);

	switch (vender_id) {
	case 1:
		ptr += sprintf(ptr,"xufang");
		break;
	case 2:
		ptr += sprintf(ptr,"xuri");
		break;
	case 3:
		ptr += sprintf(ptr,"yuye");
		break;
	case 4:
		ptr += sprintf(ptr,"tianyi");
		break;
	case 5:
		ptr += sprintf(ptr,"minglang");
		break;
	case 6:
		ptr += sprintf(ptr,"duoxinda");
		break;
	case 7:
		ptr += sprintf(ptr,"zhenhua");
		break;
	case 8:
		ptr += sprintf(ptr,"jitegao");
		break;
	case 9:
		ptr += sprintf(ptr,"guangjishengtai");
		break;
	case 10:
		ptr += sprintf(ptr,"shengguang");
		break;
	case 11:
		ptr += sprintf(ptr,"xintiantong");
		break;
	case 12:
		ptr += sprintf(ptr,"xinyou");
		break;
	case 13:
		ptr += sprintf(ptr,"yanqi");
		break;
	case 14:
		ptr += sprintf(ptr,"zhongcheng");
		break;
	case 15:
		ptr += sprintf(ptr,"xinmaoxin");
		break;
	case 16:
		ptr += sprintf(ptr,"zhenzhixin");
		break;
	case 17:
		ptr += sprintf(ptr,"helitai");
		break;
	case 18:
		ptr += sprintf(ptr,"huaxin");
		break;
	case 19:
		ptr += sprintf(ptr,"lihaojie");
		break;
	case 20:
		ptr += sprintf(ptr,"jiandong");
		break;
	case 21:
		ptr += sprintf(ptr,"xinpengda");
		break;
	case 22:
		ptr += sprintf(ptr,"jiake");
		break;
	case 23:
		ptr += sprintf(ptr,"yijian");
		break;
	case 24:
		ptr += sprintf(ptr,"yixing");
		break;
	case 25:
		ptr += sprintf(ptr,"zhongguangdian");
		break;
	case 26:
		ptr += sprintf(ptr,"hongzhan");
		break;
	case 27:
		ptr += sprintf(ptr,"huaxingda");
		break;
	case 28:
		ptr += sprintf(ptr,"dongjianhuanyu");
		break;
	case 29:
		ptr += sprintf(ptr,"dawosi");
		break;
	case 30:
		ptr += sprintf(ptr,"dacheng");
		break;
	case 31:
		ptr += sprintf(ptr,"mingwangda");
		break;
	case 32:
		ptr += sprintf(ptr,"huangze");
		break;
	case 33:
		ptr += sprintf(ptr,"jinxinxiang");
		break;
	case 34:
		ptr += sprintf(ptr,"gaoge");
		break;
	case 35:
		ptr += sprintf(ptr,"zhihui");
		break;
	case 36:
		ptr += sprintf(ptr,"miaochu");
		break;
	case 37:
		ptr += sprintf(ptr,"qicai");
		break;
	case 38:
		ptr += sprintf(ptr,"zhenghai");
		break;
	case 39:
		ptr += sprintf(ptr,"hongfazhan");
		break;
	case 40:
		ptr += sprintf(ptr,"lianchuang");
		break;
	case 41:
		ptr += sprintf(ptr,"saihua");
		break;
	case 42:
		ptr += sprintf(ptr,"keleli");
		break;

	case 43:
		ptr += sprintf(ptr,"weiyi");
		break;
	case 44:
		ptr += sprintf(ptr,"futuo");
		break;
	default:
		ptr += sprintf(ptr,"unknown");
		break;
	}
	ptr += sprintf(ptr,".\n");

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);


	return (ptr-buf);
}

static ssize_t store_tlsc_version(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	return -EPERM;
}

static ssize_t show_tlsc_info(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
	ptr += sprintf(ptr,"Max finger number is %0d.\n",TS_MAX_FINGER);
	ptr += sprintf(ptr,"Int irq is %d.\n",this_client->irq);
	ptr += sprintf(ptr,"I2c address is 0x%02X(0x%02X).\n",this_client->addr,(this_client->addr)<<1);

	return (ptr-buf);
}

static ssize_t store_tlsc_info(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	return -EPERM;
}

static ssize_t show_tlsc_ps(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
#ifdef TLSC_TPD_PROXIMITY

	ptr += sprintf(ptr,"%d\n",tpd_prox_active);
#else
	ptr += sprintf(ptr,"No proximity function.\n");
#endif

	return (ptr-buf);
}

static ssize_t store_tlsc_ps(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TLSC_TPD_PROXIMITY
	if (buf[0] == '0') {
		sensorhub_prox_ctrl(0);
		//ps off

	} else if (buf[0] == '1') {
		//ps on
		sensorhub_prox_ctrl(1);
	}
#endif
	return count;
}

static ssize_t show_tlsc_gesture(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
#ifdef TLSC_GESTRUE

	ptr += sprintf(ptr,"%d\n",gesture_enable);
#else
	ptr += sprintf(ptr,"No gesture function.\n");
#endif

	return (ptr-buf);
}

static ssize_t store_tlsc_gesture(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TLSC_GESTRUE
	if (buf[0] == '0') {
		//gesture off
		gesture_enable = 0;

	} else if (buf[0] == '1') {
		//gesture on
		gesture_enable = 1;
	}
#endif
	return count;
}

//static int debugResult=0;
u8 readFlashbuf[204];


static ssize_t show_tlsc_debug_flash(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
	int i;

	for(i=0;i<204;i++) {
		ptr += sprintf(ptr,"%d,",readFlashbuf[i]);
	}

	ptr += sprintf(ptr,"\n");

	return (ptr-buf);
}

extern int tlsc6x_download_ramcode(u8 *pcode, u16 len);
extern int tlsc6x_write_burn_space(u8 *psrc, u16 adr, u16 len);
extern int tlsc6x_read_burn_space(u8 *pdes, u16 adr, u16 len);
extern int tlsc6x_set_nor_mode(void);
extern unsigned char fw_fcode_burn[2024];
int writeFlash(u8* buf ,u16 addr,int len)
{
	//int ret=0;
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)	
	auto_upd_busy = 1;
#endif

	tp_irq_disable();  /*@zkd add*/
	if (tlsc6x_download_ramcode(fw_fcode_burn, sizeof(fw_fcode_burn))) {
		tlsc_err("Tlsc6x:write flash error:ram-code error!\n");
		return -EPERM;
	}

	if (tlsc6x_write_burn_space((unsigned char *)buf, addr, len)) {
		tlsc_err("Tlsc6x:write flash  fail!\n");
		return -EPERM;
	}
	tlsc6x_set_nor_mode();

	tlsc6x_tpd_reset();

#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
	auto_upd_busy=0;
#endif
	
	tp_irq_enable(); /*@zkd add*/
	return 0;
}

int readFlash(u16 addr,int len)
{
	//int ret=0;
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)	
	auto_upd_busy = 1;
#endif
	
	tp_irq_enable();  /*@zkd add*/
	if (tlsc6x_download_ramcode(fw_fcode_burn, sizeof(fw_fcode_burn))) {
		tlsc_err("Tlsc6x:write flash error:ram-code error!\n");
		return -EPERM;
	}

	if (tlsc6x_read_burn_space((unsigned char *)readFlashbuf, addr, len)) {
		tlsc_err("Tlsc6x:write flash  fail!\n");
		return -EPERM;
	}
	tlsc6x_set_nor_mode();

	tlsc6x_tpd_reset();

#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
	auto_upd_busy=0;
#endif
	
	tp_irq_enable();  /*@zkd add*/
	return 0;
}

static ssize_t store_tlsc_debug_flash(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u8 wBuf[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	if (buf[0] == '0') {
		//gesture off


	} else if (buf[0] == '1') {
		//gesture on
		tlsc_info("not support!\n");
	} else if (buf[0] == '2') {
		//gesture on
		writeFlash(wBuf ,0,16);
	} else if (buf[0] == '3') {
		//gesture on
		writeFlash(wBuf ,0x8000,16);
	} else if (buf[0] == '4') {
		//gesture on
		readFlash(0xF000,204);
	} else if (buf[0] == '5') {
		//gesture on
		readFlash(0,204);
	} else if (buf[0] == '6') {
		//gesture on
		readFlash(0x8000,204);
	}

	return count;
}

static DEVICE_ATTR(tlsc_version, 0664, show_tlsc_version, store_tlsc_version);
static DEVICE_ATTR(tlsc_tp_info, 0664, show_tlsc_info, store_tlsc_info);
static DEVICE_ATTR(tlsc_ps_ctl, 0664, show_tlsc_ps, store_tlsc_ps);
static DEVICE_ATTR(tlsc_gs_ctl, 0664, show_tlsc_gesture, store_tlsc_gesture);
static DEVICE_ATTR(tlsc_flash_ctl, 0664, show_tlsc_debug_flash, store_tlsc_debug_flash);
static struct attribute *tlsc_attrs[] = {
	&dev_attr_tlsc_version.attr,
	&dev_attr_tlsc_tp_info.attr,
	&dev_attr_tlsc_ps_ctl.attr,
	&dev_attr_tlsc_gs_ctl.attr,
	&dev_attr_tlsc_flash_ctl.attr,
	NULL, // Can not delete this line!!! The phone will reset.
};

static struct attribute_group tlsc_attr_group = {
	.attrs = tlsc_attrs,
};

#ifdef TLSC_ESD_HELPER_EN
static int esd_check_work(void)
{
	int ret = -1;
	u8 test_val = 0;

	TLSC_FUNC_ENTER();
	ret = tlsc6x_read_reg(this_client, 0x0, &test_val);

	if (ret < 0) {		/* maybe confused by some noise,so retry is make sense. */
		msleep(60);
		tlsc6x_read_reg(this_client, 0x0, &test_val);
		ret = tlsc6x_read_reg(this_client, 0x0, &test_val);
		if (ret < 0) {
			tlsc6x_tpd_reset();
		}
	}

	return ret;
}

static int esd_checker_handler(void *unused)
{
	ktime_t ktime;

	do {
		wait_event_interruptible(tpd_esd_waiter, tpd_esd_flag != 0);
		tpd_esd_flag = 0;

		ktime = ktime_set(4, 0);
		hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);

		if (g_tp_drvdata->esdHelperFreeze) {
			continue;
		}
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
		if (auto_upd_busy) {
			continue;
		}
#endif
		esd_check_work();

	} while (!kthread_should_stop());

	return 0;
}

enum hrtimer_restart tpd_esd_kthread_hrtimer_func(struct hrtimer *timer)
{
	tpd_esd_flag = 1;
	wake_up_interruptible(&tpd_esd_waiter);

	return HRTIMER_NORESTART;
}
#endif


#ifdef TLSC_TPD_PROXIMITY
static int sensorhub_prox_ctrl(int enable)
{
	TLSC_FUNC_ENTER();

	if (enable == 1) {
		tpd_prox_active = 1;
		if(real_suspend_flag == 1) {
			tpd_prox_suspend = 1;
			return 1;
		} else {
			tlsc6x_write_reg(this_client, 0xb0, 0x01);
		}
#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
		__pm_stay_awake(proximity_wake_lock);
#endif // end of SENSORHUB_PROXIMITY_TOUCHSCREEN
		printk("tlsc6x TP_face_mode_switch on\n");

	} else if (enable == 0) {
#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
		__pm_relax(proximity_wake_lock);
#endif // end of SENSORHUB_PROXIMITY_TOUCHSCREEN
		tpd_prox_active = 0;
		tlsc6x_write_reg(this_client, 0xb0, 0x00);
		printk("tlsc6x TP_face_mode_switch off\n");
	}
	tpd_prox_old_state = 0xe0;	/* default is far away*/
	return 1;
}

static ssize_t tlsc6x_prox_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ps enable %d\n", tpd_prox_active);
}

static ssize_t tlsc6x_prox_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
					size_t count)
{
	unsigned int enable;
	int ret = 0;

	TLSC_FUNC_ENTER();
	ret = kstrtouint(buf, 0, &enable);
	if (ret)
		return -EINVAL;
	enable = (enable > 0) ? 1 : 0;

	sensorhub_prox_ctrl(enable);

	return count;

}

static DEVICE_ATTR(enable/*proximity*/, 0664, tlsc6x_prox_enable_show, tlsc6x_prox_enable_store);

static struct attribute *proximity_attributes[] = {
	&dev_attr_enable.attr,
	//&dev_attr_proximity.attr,
	NULL
};

static const struct attribute_group proximity_attr_group = {
	.attrs = proximity_attributes,
};


#endif
static int tlsc6x_request_irq_work(void)
{
	int ret = 0;

	this_client->irq = gpio_to_irq(g_tp_drvdata->platform_data->irq_gpio_number);
	tlsc_info("The irq node num is %d", this_client->irq);

	ret = request_threaded_irq(this_client->irq,
				   NULL, touch_event_thread_handler,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "tlsc6x_tpd_irq", NULL);
	if (ret < 0) {
		tlsc_err("Request irq thread error!");
		return  ret;
	}

	return ret;
}

static ssize_t ts_suspend_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
  unsigned int input;

  if (kstrtouint(buf, 10, &input))
      return -EINVAL;

  if (input == 1)
      glb_tlsc6x_adf_suspend();
  else if (input == 0)
      glb_tlsc6x_adf_resume();
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
}

static int tlsc6x_tp_sysfs_init(struct i2c_client *client){
	return tp_sysfs_init(client);
}


#if defined(SENSORHUB_PROXIMITY_TOUCHSCREEN)
static int get_prox_enble_stat(void){
    return tpd_prox_active;
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
    .ic_name = "tlsc6x"
};
#endif

static int tlsc6x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef  TLSC_TPD_PROXIMITY	
	int ret = -1;
#endif
	
	int err = 0;
	int reset_count;
	struct input_dev *input_dev;
	struct tlsc6x_platform_data *pdata = NULL;
	TLSC_FUNC_ENTER();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_alloc_platform_data_failed;
	}

	if (client->dev.of_node) {
		pdata = tlsc6x_parse_dt(&client->dev);
		if (pdata) {
			client->dev.platform_data = pdata;
		}
		else
		{
      err = -ENOMEM;
  		tlsc_err("%s: no platform data!!!\n", __func__);
  		goto exit_alloc_platform_data_failed;
		}
	}

	g_tp_drvdata = kzalloc(sizeof(*g_tp_drvdata), GFP_KERNEL);	/* auto clear */
	if (!g_tp_drvdata) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	g_tp_drvdata->client = client;
	g_tp_drvdata->platform_data = pdata;

	err = tlsc6x_hw_init(g_tp_drvdata);
	if (err < 0) {
		goto exit_gpio_request_failed;
	}

	i2c_set_clientdata(client, g_tp_drvdata);

	/* #ifdef CONFIG_I2C_SPRD */
	/* sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000); */
	/* #endif */
	reset_count = 0;
	g_is_telink_comp = 0;
	while (++reset_count <= 3) {
		tlsc6x_tpd_reset();
		g_is_telink_comp = tlsc6x_tp_dect(client);
		if (g_is_telink_comp) {
			break;
		}
	}

	g_tp_drvdata->needKeepRamCode = g_needKeepRamCode;

	if (g_is_telink_comp) {
		tlsc6x_tpd_reset();
	} else {
		tlsc_err("tlsc6x:%s, no tlsc6x!\n", __func__);
		err = -ENODEV;
		goto exit_chip_check_failed;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "tlsc6x error::failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	g_tp_drvdata->input_dev = input_dev;

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_HOMEPAGE, input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->x_res_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->y_res_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 127, 0, 0);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);	/* give this capability aways */
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name =   "currency_tp" ; // "tlsc6x_touch";
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "tlsc6x error::failed to register input device: %s\n", dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
#ifdef  TLSC_TPD_PROXIMITY

#if 1 // revo     ps_input_dev ==  input_dev
    printk("__%s__ proximity_attr_group start \n",__func__);

    g_tp_drvdata->ps_input_dev = input_dev ;
	ret = sysfs_create_group(&g_tp_drvdata->ps_input_dev->dev.kobj, &proximity_attr_group);
	if (0 != ret) {
		dev_err(&client->dev,"%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&g_tp_drvdata->ps_input_dev->dev.kobj, &proximity_attr_group);
	}

	input_set_capability(g_tp_drvdata->ps_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(g_tp_drvdata->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    printk("__%s__ proximity_attr_group end \n",__func__);
#else
	tlsc6x_prox_cmd_path_init();
	g_tp_drvdata->ps_input_dev = input_allocate_device();
	if (!g_tp_drvdata->ps_input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "tlsc6x error::failed to allocate ps-input device\n");
		goto exit_input_register_device_failed;
	}
	g_tp_drvdata->ps_input_dev->name = "currency_tp" ; // "proximity_tp";
	set_bit(EV_ABS, g_tp_drvdata->ps_input_dev->evbit);
	input_set_capability(g_tp_drvdata->ps_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(g_tp_drvdata->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	err = input_register_device(g_tp_drvdata->ps_input_dev);
	if (err) {
		dev_err(&client->dev, "tlsc6x error::failed to register ps-input device: %s\n", dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
#endif
#endif

#ifdef TOUCH_VIRTUAL_KEYS
	tlsc6x_virtual_keys_init();
#endif

	INIT_WORK(&g_tp_drvdata->resume_work, tlsc6x_resume_work);
	
	g_tp_drvdata->tp_resume_workqueue = create_singlethread_workqueue("tlsc6x_resume_work");
	if (!g_tp_drvdata->tp_resume_workqueue) {
		err = -ESRCH;
		goto exit_input_register_device_failed;
	}
/*@zkd add*/
	INIT_WORK(&g_tp_drvdata->report_event_work, tlsc6x_irq_work);

	g_tp_drvdata->report_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
	if (!g_tp_drvdata->report_workqueue) {
		err = -ESRCH;
		goto exit_input_register_device_failed;
	}
	
	tlsc6x_wakelock = wakeup_source_create("tlsc6x_wakelock");
	wakeup_source_add(tlsc6x_wakelock);
	
	spin_lock_init(&irq_lock);
/*@zkd add*/
	err = tlsc6x_request_irq_work();
	if (err < 0) {
		dev_err(&client->dev, "tlsc6x error::request irq failed %d\n", err);
		goto exit_irq_request_failed;
	}

	tp_irq_disable();  /*@zkd add*/

	tlsc6x_tp_sysfs_init(client);  /*@zkd change current*/

#ifdef TLSC_APK_DEBUG
	tlsc6x_create_apk_debug_channel(client);
#endif

	err=sysfs_create_group(&client->dev.kobj, &tlsc_attr_group);
	if (err < 0) {
		tlsc_err("Can not create sysfs group!");
	}

#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
  sensorhub_info.input = g_tp_drvdata->ps_input_dev;
  sensorhub_prox_do = tp_hub_sensorhub_do_funcList(TYPE_SENSORHUB_SETTING, &sensorhub_info);
  if(sensorhub_prox_do&& sensorhub_prox_do->sensorhub_init)
      sensorhub_prox_do->sensorhub_init();

	proximity_wake_lock = wakeup_source_create("proximity_wake_lock");
	wakeup_source_add(proximity_wake_lock);		

#endif // end of SENSOR_HUB_PROXIMITY

#ifdef TLSC_ESD_HELPER_EN
	{			/* esd issue: i2c monitor thread */
		ktime_t ktime = ktime_set(30, 0);

		hrtimer_init(&tpd_esd_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		tpd_esd_kthread_timer.function = tpd_esd_kthread_hrtimer_func;
		hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);
		kthread_run(esd_checker_handler, 0, "tlsc6x_esd_helper");
	}
#endif

	tp_irq_enable();  /*@zkd add*/

	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_chip_check_failed:
exit_gpio_request_failed:
	kfree(g_tp_drvdata);
exit_alloc_data_failed:
  if (pdata != NULL) {
    gpio_free(pdata->irq_gpio_number);
    gpio_free(pdata->reset_gpio_number);
    kfree(pdata);
  }
	g_tp_drvdata = NULL;
	i2c_set_clientdata(client, g_tp_drvdata);
exit_alloc_platform_data_failed:
	return err;
}

static int tlsc6x_remove(struct i2c_client *client)
{
	struct tlsc6x_data *drvdata = i2c_get_clientdata(client);

	TLSC_FUNC_ENTER();

#ifdef TLSC_APK_DEBUG
	tlsc6x_release_apk_debug_channel();
#endif

#ifdef TLSC_ESD_HELPER_EN
	hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

	if (drvdata == NULL) {
		return 0;
	}

	free_irq(client->irq, drvdata);
	input_unregister_device(drvdata->input_dev);
	input_free_device(drvdata->input_dev);
/*@zkd add*/
#ifdef CONFIG_HAS_EARLYSUSPEND
	cancel_work_sync(&drvdata->report_event_work);
	destroy_workqueue(drvdata->report_workqueue);
	
	cancel_work_sync(&drvdata->resume_work);
	destroy_workqueue(drvdata->tp_resume_workqueue);
#endif
/*@zkd add*/
	kfree(drvdata);
	drvdata = NULL;
	i2c_set_clientdata(client, drvdata);

	return 0;
}

static const struct i2c_device_id tlsc6x_id[] = {
	{TS_NAME, 0}, {}
};

MODULE_DEVICE_TABLE(i2c, tlsc6x_id);

static const struct of_device_id tlsc6x_of_match[] = {
	{.compatible = "tlsc6x,tlsc6x_ts",},
	{}
};

MODULE_DEVICE_TABLE(of, tlsc6x_of_match);
static struct i2c_driver tlsc6x_driver = {
	.probe = tlsc6x_probe,
	.remove = tlsc6x_remove,
	.id_table = tlsc6x_id,
	.driver = {
		   .name = TS_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = tlsc6x_of_match,
	},
};

static int __init tlsc6x_init(void)
{
  return i2c_add_driver(&tlsc6x_driver);
}

static void __exit tlsc6x_exit(void)
{
	i2c_del_driver(&tlsc6x_driver);
}

late_initcall(tlsc6x_init);
module_exit(tlsc6x_exit);

MODULE_DESCRIPTION("Chipsemi touchscreen driver");
MODULE_LICENSE("GPL");
