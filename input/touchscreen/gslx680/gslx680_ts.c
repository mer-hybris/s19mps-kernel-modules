/*
 * drivers/input/touchscreen/gslx680.c
 *
 * Sileadinc gslx680 TouchScreen driver.
 *
 * Copyright (c) 2012  Sileadinc
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
 * VERSION          DATE            AUTHOR
 *   1.0         2012-04-18        zxw
 *
 * note: only support mulititouch   Wenfs 2010-10-01
 */

/*sensorhub prox */
#include <linux/miscdevice.h>//luyuan
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/byteorder/generic.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include <linux/proc_fs.h>
//#include <linux/regulator/consumer.h>
//#include <mach/regulator.h>
#include <linux/input-event-codes.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/pm_wakeup.h>
#include <linux/uaccess.h>
//add start by xuan_h
#include <linux/notifier.h>
#include "prj/prj_config.h"
#include "tp_common.h"
#include "cust_firmware.h"

#define GSLX680_CONFIG_OF
#define REVO_PM_TP_SYSFS

#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
#include "revo_sensorhub.h"
#endif

#ifdef REVO_PM_VIDEO_ADF_NOTIFIER
#include <video/adf_notifier.h>
#endif

#ifdef SCREEN_PHY_WIDTH
static unsigned short SCREEN_MAX_X = (unsigned short)SCREEN_PHY_WIDTH ;
#else
static unsigned short SCREEN_MAX_X = 800 ;
#endif
#ifdef SCREEN_PHY_HIGHT
static unsigned short SCREEN_MAX_Y = (unsigned short)SCREEN_PHY_HIGHT ;
#else
static unsigned short SCREEN_MAX_Y = 1280 ;
#endif

#define  dac_index(num)       (num)
#define  DAC_INDEX_MAX_SIZE    4
#define  DAC_INDEX_ALL         DAC_INDEX_MAX_SIZE


#ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
static const struct do_sensorhub* sensorhub_prox_do = NULL ;
#endif

enum { TYPE_0 = 0 , TYPE_1 , TYPE_2 , TYPE_3 , TYPE_4 ,
    TYPE_DAC01 ,TYPE_DAC02 , TYPE_DAC03, TYPE_DAC04,
    TYPE_ID_MAX } ;
static const int fw_typeId[TYPE_ID_MAX] = {
    0 , 1 , 2, 3, 4,
    0xDAC01, 0xDAC02 ,0xDAC03, 0xDAC04 } ;
#define  FW_LCD_SIZE_CFG(cfg_num,cond_x,cond_y,cfg_data_id_name,fw_name)  \
    [cfg_num] = {\
        .condition = { \
            .x_resolution = cond_x, \
            .y_resolution = cond_y, \
            .type_id = cfg_num \
        } ,\
        .fw_info = { \
            .gsl_config_data_id_cfg = cfg_data_id_name, \
            .gsl_fw_data = fw_name, \
            .gsl_fw_data_size = sizeof(fw_name)/sizeof(fw_name[0])} \
    }

/*******************************************************************
    GSL_IDENTY_TP :
                    default  setting
********************************************************************/

#ifndef  DAC_IDENTIFY_PAGE_ADDRESS
/* This is GSL_DAC_IDENTY_TP  setting  :def  close  */
#define  DAC_IDENTIFY_PAGE_ADDRESS    {0}
#define  DAC_IDENTIFY_OFFSET           0
#define  DAC_IDENTIFY_ID2_RULES       {{dac_index(0xff),"no",0xff}}
#endif

#ifndef GSL_CONFIG_DATA_ID_CFG1
    #define GSL_CONFIG_DATA_ID_CFG1    gsl_config_data_id
    #define GSLX680_FW_CFG1            GSLX680_FW
#endif /*End of : GSL_CONFIG_DATA_ID_CFG1*/

#ifndef GSL_CONFIG_DATA_ID_CFG2
    #define GSL_CONFIG_DATA_ID_CFG2    gsl_config_data_id
    #define GSLX680_FW_CFG2            GSLX680_FW
#endif /* End of :GSL_CONFIG_DATA_ID_CFG2  */

#ifndef GSL_CONFIG_DATA_ID_CFG3
    #define GSL_CONFIG_DATA_ID_CFG3    gsl_config_data_id
    #define GSLX680_FW_CFG3            GSLX680_FW
#endif /* End of :GSL_CONFIG_DATA_ID_CFG3  */

#ifndef GSL_CONFIG_DATA_ID_CFG4
    #define GSL_CONFIG_DATA_ID_CFG4    gsl_config_data_id
    #define GSLX680_FW_CFG4            GSLX680_FW
#endif /* End of :GSL_CONFIG_DATA_ID_CFG4  */


#ifndef GSL_CHIP_1
    #define GSL_C       100
    #define GSL_CHIP_1  0xffffffff
    #define GSL_CHIP_2  0xffffffff
#endif

#ifndef GSL_CHIP_3
    #define GSL_CHIP_3  0xffffffff
#endif

#ifndef GSL_CHIP_4
    #define GSL_CHIP_4  0xffffffff
#endif

#include  "gsl_tp_check_fw_common.h" //  control "define GSL_TP_CHECK_FW_DATA_H"  ex: "firmware/e101_tengzhi/gsl_tp_check_fw.h"

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen);
#ifdef REVO_PM_TP_SYSFS
extern int glb_gslx680_tp_sysfs_init(struct i2c_client *client);
extern int glb_gslx680_tp_sysfs_remove(struct i2c_client *client);
#endif

#define GSLX680_TS_NAME     "gslX680_ts"                  //"gslX680_ts"
#define INPUT_DEV_NAME      "currency_tp"                  //"gslX680_ts"
#define GSLX680_TS_DEVICE   "gslX680"      //"gslX680"
#define GSLX680_TS_ADDR     0x40

//add end by xuan_h
#define TS_DEBUG_MSG 0
#if TS_DEBUG_MSG
#define GSL168X_DBG(format, ...)    printk(KERN_INFO "GSL168X " format "\n", ## __VA_ARGS__)
#else
#define GSL168X_DBG(format, ...)
#endif

#define GSL168X_INFO(format, ...)    printk(KERN_INFO "GSL168X " format "\n", ## __VA_ARGS__)
#define GSL168X_ERR(format, ...)    printk(KERN_ERR "GSL168X " format "\n", ## __VA_ARGS__)

//#define GSL9XX_IO_CTR

#define MAX_FINGERS     10
#define MAX_CONTACTS    10
#define DMA_TRANS_LEN   0x20
#define GSL_PAGE_REG    0xf0

#if   defined(PRJ_FEATURE_H_BOARD_GESTURE_ENABLE)
#define GSL_GESTURE             //luyuan
#endif

#define GSL_TIMER

#ifdef GSL_GESTURE
static unsigned int gsl_model_extern[]={
    0x10,0x56,
    0x08000f00,0x0f0c2f1f,0x12125040,0x13127060,0x16149181,0x1a18b1a1,0x1c1bd2c2,0x201df2e2,
    0x3324f7fe,0x4f41e7ef,0x6d5ed8df,0x8a7bc8d0,0xa698b7c0,0xc3b4a6af,0xe0d2959d,0xffee848d,
    0x10,0x57,
    0x00062610,0x03015c41,0x06049277,0x0f09c8ad,0x2918f7e0,0x5142e4fb,0x685eb2cb,0x77707c97,
    0x857d9177,0x978dc5ab,0xb4a1f3de,0xdbcbd5ec,0xebe4a2bd,0xf4f06c87,0xfaf73651,0xfffd001b,
    0x10,0x49,
    0x0e00f4ff,0x2f1ee4ec,0x4f3ed4dc,0x6f5fc4cc,0x8f7fb3bc,0xae9ea3ab,0xcebe949b,0xf0df858d,
    0xf0ff707a,0xcfdf6268,0xadbe525a,0x8d9e434a,0x6d7d353c,0x4c5c262e,0x2c3c151e,0x0c3c000b,
    0x10,0x49,
    0x775df6f9,0xab91e6ef,0xdac3cedb,0xf9eda2b9,0xfdff6d88,0xf1f93a53,0xcce21424,0x9ab50209,
    0x65800101,0x354d1409,0x0f1f3a25,0x01056e53,0x0100a288,0x1407d3bc,0x4128f1e4,0x765bfffb,

    0x10,0x1041,
    0xfdff859f,0xe0f2566c,0xbdcf2d41,0x90a90f1d,0x5a750106,0x253f0400,0x020d2b11,0x10015c46,
    0x3823806e,0x664e9f91,0x9a80b2ab,0xd0b5bab8,0xaac5bbbc,0x7590bebb,0x445bd3c4,0x244fffe5,
    0x10,0x1042,
    0xe5ff795e,0xb0cb4f54,0x7c96444a,0x4e63293a,0x223a0917,0x39271601,0x5c4a402c,0x7a6c6e55,
    0x837fa388,0x8084d7bd,0x5871fdef,0x293febfd,0x1019bcd5,0x040987a1,0x0101526d,0x11271e38,
    0x10,0x1044,
    0x86867995,0x8386415d,0x687a1026,0x324e0003,0x03151d07,0x04005539,0x240e836f,0x553ca293,
    0x8c70b5ac,0xc4a8bebb,0xfce0bfbf,0xcae6bfbf,0x91adbfbf,0x5975bdbe,0x253dccbf,0x0534ffe4,
    0x10,0x1041,
    0x0007775b,0x1004ae93,0x3520dac6,0x6b50f4e7,0xa487fdfe,0xdec1f3f9,0xfdf8c7e3,0xd7ee9dae,
    0xa4be818f,0x6f8a6672,0x3552595d,0x33185556,0x6d505555,0xa78a5053,0xddc43a48,0xdd05001d,
    0x10,0x1042,
    0x1a00d1d1,0x4f34d6d3,0x8369dfda,0xb89eebe4,0xecd2fef4,0xd5edecf9,0xb0bec6de,0x99a597af,
    0x8a8f637d,0x87882e49,0xa28c0214,0xd4bc0c01,0xece53b22,0xfcf56f55,0xfffea489,0xf91ed9be,
    0x10,0x1044,
    0x93958166,0x9a94b79c,0xb5a5e7d1,0xe6cbfff8,0xfefbd2ed,0xeef79eb8,0xcce07386,0x9fb75562,
    0x6b854049,0x35503539,0x02193331,0x381d3535,0x6f543736,0xa58a3938,0xdbc13239,0xf409001b,
    0x10,0x1045,
    0x6d6d1e00,0x6d6d5b3d,0x6e6d987a,0x7e73d4b7,0xab8ffef0,0xe4cae9fa,0xfcf3afcd,0xfdff7290,
    0xe2f03a55,0xb5ce0f21,0x79970104,0x3d5a0900,0x1324361c,0x02077153,0x0200ae90,0x1c2be9cc,
    0x10,0x1045,
    0x9999d9f8,0x97989ab9,0x90965b7a,0x7f8a1c3b,0x48680006,0x0d291704,0x00045535,0x07019375,
    0x2112ceb2,0x5638f5e6,0x9575fffc,0xd1b4f0fc,0xf5e7bbd9,0xfffc7c9b,0xf3fc3e5c,0xd407021f,

    0x10,0x47,
    0xc9e00105,0x9ab11008,0x6f832c1c,0x525d533c,0x765f6c68,0xa68e5b64,0xd2bc4552,0xf8e62435,
    0xf2fa462e,0xdfea745d,0xc4d29e89,0xa4b5c5b3,0x8093e9d9,0x526bfef7,0x2139f2fc,0x002fcae0,
    0x10,0x36,
    0x18009fa5,0x4930969a,0x79619193,0xaa929190,0xdbc39893,0xfdf0baa3,0xeafae5d1,0xbfd6fbf4,
    0x8ea6ffff,0x6477eaf9,0x5356bcd5,0x5f588ea5,0x756a6176,0x94843a4c,0xb9a61829,0xe5ed000a,


    0x0a,0x36,
    0xebff0002,0xc1d60100,0x98ac0c06,0x75862116,0x53633b2e,0x35435949,0x1f297c6a,0x0c15a38f,
    0x0004ccb7,0x0e03f2e0,0x3621fffd,0x5b4aecf9,0x776acbdc,0x797ba1b6,0x566b8a90,0x2c61928b,
    0x0a,0x36,
    0x8f901700,0x8186452e,0x737b725c,0x536a9287,0x273da59c,0x0414c5b3,0x1905e9db,0x452efbf4,
    0x735cfffd,0xa28afaff,0xceb8ecf4,0xf7e4d5e2,0xf9ffa7be,0xcce3979b,0x9eb59495,0x6fa79593,
    0x0a,0x36,
    0xebff0900,0xc1d61e14,0x99ad372b,0x73875044,0x4c5f695c,0x2c398977,0x121eaf9b,0x0005d9c3,
    0x1d08f8ed,0x4b34fffd,0x7762f5fb,0xa38ee7ee,0xceb9d5de,0xdfe2b6ca,0xb3caa6aa,0x85bca5a5,

    0x10,0x4f,
    0x5d76fefd,0x2d45e5f4,0x0b1abdd4,0x01058aa3,0x02005670,0x1b0b293d,0x4a320f19,0x7d630005,
    0xae960f05,0xd7c5311e,0xf4e85e46,0xfefc9278,0xf6fec4ac,0xcee4e8d7,0x9cb6fbf6,0x6882fefe,
    0x10,0x4f,
    0x795ffffc,0xaf94fdff,0xddc7dff0,0xf8eeafc9,0xfffd7a94,0xf8fe455f,0xd3e81c2f,0xa1bb0610,
    0x6b860600,0x3b521e0f,0x16264731,0x020a7a60,0x0000b095,0x1706e0cb,0x492efaf0,0x7f84fefe,

    0x10,0x00,
    0xf4ff0d00,0xe6ee2f1f,0xd8df5240,0xc8d17262,0xbac29483,0xacb3b6a4,0x9ea4d8c7,0x8f99f9ea,
    0x7883e7f6,0x666fc7d6,0x595fa4b5,0x49518495,0x383f6474,0x27304254,0x161e2132,0x012d0411,

};
#endif

#ifdef GSL_TIMER
static struct delayed_work gsl_monitor_work;
static struct workqueue_struct *gsl_monitor_workqueue = NULL;
static char int_1st[4] = {0};
static char int_2nd[4] = {0};
//static char dac_counter = 0;
static char b0_counter = 0;
static char bc_counter = 0;
static char i2c_lock_flag = 0;
#endif

static int in_suspend = 0;

#if defined(PRJ_FEATURE_H_BOARD_HAVE_PLS_TP)||defined(SENSORHUB_PROXIMITY_TOUCHSCREEN)
#define  CONFIG_TOUCHSCREEN_WITH_PROXIMITY
#endif

#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
static atomic_t  prox_contrl_state;
static int PROXIMITY_SWITCH =0;

static struct wakeup_source  *wakeup_wakelock;

static int s_nearfar = 1;
static int PROXIMITY_STATE = 0;
static int gsl_halt_flag = 0;
#define GSLX680_PS_INPUT_DEV            "currency_tp"
#endif

char sprd_tp_name[256];
EXPORT_SYMBOL(sprd_tp_name);
static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen);

#ifdef GSL_GESTURE
#define REPORT_KEY_VALUE KEY_POWER//KEY_F1      //report key set

static void gsl_irq_mode_change(struct i2c_client *client,u32 flag);
static void gsl_quit_doze(struct i2c_client *client);

typedef enum{
    GE_DISABLE = 0,
    GE_ENABLE = 1,
    GE_WAKEUP = 2,
    GE_NOWORK =3,
}GE_T;
extern void gsl_FunIICRead(unsigned int (*fun) (unsigned int *,unsigned int,unsigned int));
static struct  wakeup_source *gsl_wake_lock;
static GE_T gsl_gesture_status = GE_DISABLE;
#if defined(PRJ_FEATURE_H_REVO_B_SMART_WAKE_OPEN_BY_DEFAULT)
static unsigned int gsl_gesture_flag = 1;
#else
static unsigned int gsl_gesture_flag = 0;
#endif
static char gsl_gesture_c =0;
static int power_key_status = 0;
extern void gsl_GestureExternInt(unsigned int *model,int len);
#endif
//static volatile int gsl_halt_flag = 0;
#define PRESS_MAX           255
#ifdef FILTER_POINT
#define FILTER_MAX  3
#endif

#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

/* *******************************************

    This is  GSL_IDENTY_TP   :
        struct gsl_identy_tp_infogsl_identy_info

************************************************** */

/* revo@start { add  GSL_GSL_DAC_IDENTY */
typedef struct dac_identify_rules {
    u8   index;
    char operation[3] ;
    u32   reference_value  ;
} dac_identify_rules_t ;

static u8 s_dac_identify_page_address[4] ;
static u8 s_dac_identify_offset ;
static struct dac_id_condition {
    int  rules_size ;
    dac_identify_rules_t rules[DAC_INDEX_MAX_SIZE];
} s_dac_id2_condition ;
/*  add  GSL_GSL_DAC_IDENTY   revo@End  */

static struct gsl_identy_tp_info {
    int gsl_tp_type ;
    unsigned int gsl_chip_base ;
    unsigned int *gsl_config_data_id_cfg ;
    struct fw_data *gsl_fw_data ;
    int gsl_fw_data_size ;

} gsl_identy_info = {
    .gsl_tp_type = 0 ,
    .gsl_chip_base = 0 ,
    .gsl_config_data_id_cfg = gsl_config_data_id ,
    .gsl_fw_data = GSLX680_FW ,
    .gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW)
};

static void init_chip(struct i2c_client *client);

#ifdef GSL_REPORT_POINT_SLOT
    #include <linux/input/mt.h>
#endif


struct sprd_i2c_setup_data {
    unsigned i2c_bus;  //the same number as i2c->adap.nr in adapter probe function
    unsigned short i2c_address;
    int irq;
    char type[I2C_NAME_SIZE];
};

static int sprd_3rdparty_gpio_tp_rst = -1 ;
static int sprd_3rdparty_gpio_tp_irq = -1 ;


//add start by xuan_h
struct notifier_block adf_event_block;
static int gslx680_ts_resume(struct device *pDevice);
static int gslx680_ts_suspend(struct device *pDevice);
//add end by xuan_h


//static int gslx680_irq;
//spinlock_t resume_lock; // add 20141030
//static int *flag;

static u32 id_sign[MAX_CONTACTS+1] = {0};
static u8 id_state_flag[MAX_CONTACTS+1] = {0};
static u8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static u16 x_old[MAX_CONTACTS+1] = {0};
static u16 y_old[MAX_CONTACTS+1] = {0};
static u16 x_new = 0;
static u16 y_new = 0;

static struct i2c_client *this_client = NULL;


struct gslx680_ts_data {
    struct input_dev    *input_dev;
    struct i2c_client   *client;
    u8 touch_data[44];
    struct work_struct  pen_event_work;
    struct workqueue_struct *ts_workqueue;
    //struct early_suspend  early_suspend;
    struct gslx680_ts_platform_data *platform_data;
	int int_pin;
	int reset_pin;
};

static void gslx680_hw_reset(struct gslx680_ts_data *gslx680_ts);

#ifdef GSL_GESTURE
static struct class *Gesture_class;
static struct device *Gesture_cmd_dev;

#define VIRTUAL_WAKE_GESTURE        "wake_gesture"
static ssize_t tp_wake_gesture_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", gsl_gesture_flag);
}

static ssize_t tp_wake_gesture_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t size)
{
    unsigned int on_off = simple_strtoul(buf, NULL, 10);
    gsl_gesture_flag = on_off > 0 ? 1 : 0;
    return size;
}

static struct kobj_attribute virtual_wake_gesture_attr = {
    .attr = {
        .name = VIRTUAL_WAKE_GESTURE,
        .mode = 0666,
    },
    .show = &tp_wake_gesture_show,
    .store = &tp_wake_gesture_store,
};
#endif

static struct gslx680_ts_data *g_gslx680_ts;

#ifdef HAVE_TOUCH_KEY
static u16 key = 0;
static int key_state_flag = 0;
struct key_data {
    u16 key;
    u16 x_min;
    u16 x_max;
    u16 y_min;
    u16 y_max;
};

#define MAX_KEY_NUM     (sizeof(key_array)/sizeof(key_array[0]))
const u16 key_array[]={
KEY_ARRAY
};

struct key_data gsl_key_data[MAX_KEY_NUM] = {
  GSL_KEY_DATA
};
#endif

#ifdef TOUCH_VIRTUAL_KEYS

#define CTP_BUTTON_KEY_Y  1500
#define MENU_CTP_BUTTON_X  40
#define HOME_CTP_BUTTON_X   80
#define BACK_CTP_BUTTON_X  120

#define BUTTON_WIDTH 40
#define BUTTON_HEIGHT 40
#define VIRTUAL_VEBRATOR_FUC    1


static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

    return sprintf(buf,
        "0x%02x:%d:%d:%d:%d:%d:0x%02x:%d:%d:%d:%d:%d:0x%02x:%d:%d:%d:%d:%d\n",
        EV_KEY, KEY_APPSELECT, MENU_CTP_BUTTON_X, CTP_BUTTON_KEY_Y, BUTTON_WIDTH, BUTTON_HEIGHT,
        EV_KEY, KEY_HOMEPAGE, HOME_CTP_BUTTON_X, CTP_BUTTON_KEY_Y, BUTTON_WIDTH, BUTTON_HEIGHT,
        EV_KEY, KEY_BACK, BACK_CTP_BUTTON_X, CTP_BUTTON_KEY_Y, BUTTON_WIDTH, BUTTON_HEIGHT);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.currency_tp",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};
#endif


#define VIRTUAL_TP_TYPE         "TP_NAME: GSLX680"
#define VIRTUAL_TP_NAME       "tp_name"

typedef struct revo_TpId { u8 index ; u8 id ; } REVO_TP_ID_T;

/* *****************************************************************
    chip  0xFC  val   fc.[3-0]
1680E / 1688E / 2681B / 2682B                  0x{A0,82,XX,XX}
1680F / 1688F / 2681C / 2682C                  0x{BX,82,XX,XX}
3673 / 3675 / 3676 / 3679 / 3680B / 3692       0x{80,36,XX,XX}
915 / 960 / 968 / 3670                         0x{50,91,XX,XX}
2236 / 2133 / 2138 / 2336                      0x{AX,83,XX,XX}
****************************************************************** */
/* GSL1691  (c64 huawf)*/
#define TP_ID_GSL1691     {{2,0x82},{3,0xb4}}
#define TP_NAME_GSL1691   "TP_NAME: GSL1691"

#define TP_ID_GSL915     {{2,0x91},{3,0x50}}
#define TP_NAME_GSL915   "TP_NAME: GSL915"

/* GSL2681 (c801)*/
#define TP_ID_GSL2681     {{2,0x82},{3,0xb4}}
#define TP_NAME_GSL2681   "TP_NAME: GSL2681"

/* GSL3676 (e101) */
#define TP_ID_GSL3676     {{2,0x36},{3,0x80}}
#define TP_NAME_GSL3676   "TP_NAME: GSL3676"

#define REVO_GET_GSLXXX_TP_NAME(_str)\
    do{ \
        REVO_TP_ID_T temp_id[] = TP_ID_##_str ; \
        int count = sizeof(temp_id)/sizeof(temp_id[0]);\
        while(count--) \
            if( temp_id[count].id != reg_FC[ temp_id[count].index ] ) \
                break ;\
        if( count == -1 )\
            return sprintf(buf, "%s \nTYPE(%x,%08x) VER:%s \nFW Ver:0x%x\n",\
                    TP_NAME_##_str , gsl_identy_info.gsl_tp_type , gsl_identy_info.gsl_chip_base ,version_type ,fw_version);\
    }while(0)

int gsl_version_type(char *buf,int size)//
{
    u32 count=0;
    u32 tmp;
    tmp = gsl_version_id();
    count = scnprintf(buf,PAGE_SIZE,"%08x.",tmp);
    tmp = gsl_identy_info.gsl_config_data_id_cfg[0];
    count += scnprintf(buf+count,PAGE_SIZE-count,"%08x",tmp);
//  tmp = 0x3;
//  gsl_write_interface(this_client,0xf0,(char *)&tmp,4);
//  gsl_read_interface(this_client,0,(char *)&tmp,4);
//  count += scnprintf(buf+count,PAGE_SIZE-count,"%08x",tmp);
    return count;

}

static int fw_version = 0;
static void gsl_read_version(struct i2c_client *client)
{
	u8 buf[4] = {0};
	int err= 0;
	buf[0] = 0x03;
	err = gsl_ts_write(client, 0xf0, buf, 4);
	if(err < 0) {
		return;
	}
	gsl_ts_read(client, 0x04, buf, 4);
	gsl_ts_read(client, 0x04, buf, 4);
	if(err < 0) {
		printk("read version error ,ret=%d\n",err);
		return;
	}
	fw_version = (buf[3]<<24) |(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
}

static ssize_t tp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    char version_type[PAGE_SIZE/2]={0};
    u8 reg_FC[4] ={};
    gsl_ts_read(this_client, 0xfc, reg_FC, 4);
    gsl_version_type( version_type, ARRAY_SIZE(version_type) );

#ifdef CONFIG_PROJ_C64_HUAWF
    REVO_GET_GSLXXX_TP_NAME(GSL1691);
#else
    REVO_GET_GSLXXX_TP_NAME(GSL915);
    REVO_GET_GSLXXX_TP_NAME(GSL2681);
    REVO_GET_GSLXXX_TP_NAME(GSL3676);
#endif

    return sprintf(buf, "%s \nTYPE(%x,%08x) VER:%s \nFW Ver:0x%x\n",
                    VIRTUAL_TP_TYPE, gsl_identy_info.gsl_tp_type , gsl_identy_info.gsl_chip_base ,version_type ,fw_version);
}

static struct kobj_attribute virtual_tp_name_attr = {
    .attr = {
        .name = VIRTUAL_TP_NAME,
        .mode = S_IRUGO|S_IWUSR,
    },
    .show = &tp_name_show,
};

static struct attribute *properties_attrs[] = {
#if defined(TOUCH_VIRTUAL_KEYS)	
    &virtual_keys_attr.attr,
#endif
    &virtual_tp_name_attr.attr,
#ifdef GSL_GESTURE
    &virtual_wake_gesture_attr.attr,
#endif
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void gslx680_ts_virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}


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

static inline u16 join_bytes(u8 a, u8 b)
{
    u16 ab = 0;
    ab = ab | a;
    ab = ab << 8 | b;
    return ab;
}



/*
 * touchscreen's suspend and resume state should rely on screen state,
 * as fb_notifier and early_suspend are all disabled on our platform,
 * we can only use adf_event now
 */
#if defined(REVO_PM_VIDEO_ADF_NOTIFIER)
static int ts_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{

    struct adf_notifier_event *event = data;
    int adf_event_data;
    struct device *pDevice;

    if (action != ADF_EVENT_BLANK)
        return NOTIFY_DONE;

    adf_event_data = *(int *)event->data;
    GSL168X_DBG("receive adf event with adf_event_data=%d", adf_event_data);

    switch (adf_event_data) {
    case DRM_MODE_DPMS_ON:
        gslx680_ts_resume(pDevice);
        break;
    case DRM_MODE_DPMS_OFF:
        gslx680_ts_suspend(pDevice);
        break;
    default:
        GSL168X_DBG("receive adf event with error data, adf_event_data=%d",
            adf_event_data);
        break;
    }

    return NOTIFY_OK;
}
#endif

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
    struct i2c_msg xfer_msg[1];
    buf[0] = reg;

    xfer_msg[0].addr = client->addr;
    xfer_msg[0].len = num + 1;
    xfer_msg[0].flags = client->flags & I2C_M_TEN;
    xfer_msg[0].buf = buf;
    return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata, int datalen)
{
    int ret = 0;
    u8 tmp_buf[128];
    unsigned int bytelen = 0;
    if (datalen > 125)
    {
        GSL168X_INFO("%s too big datalen = %d!\n", __func__, datalen);
        return -1;
    }

    tmp_buf[0] = addr;
    bytelen++;

    if (datalen != 0 && pdata != NULL)
    {
        memcpy(&tmp_buf[bytelen], pdata, datalen);
        bytelen += datalen;
    }
    ret = i2c_master_send(client, tmp_buf, bytelen);
    return ret;
}

static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata, unsigned int datalen)
{
    int ret = 0;

    if (datalen > 126)
    {
        GSL168X_INFO("%s too big datalen = %d!\n", __func__, datalen);
        return -1;
    }

    ret = gsl_ts_write(client, addr, NULL, 0);
    if (ret < 0)
    {
        GSL168X_INFO("%s set data address fail!\n", __func__);
        return ret;
    }

    return i2c_master_recv(client, pdata, datalen);
}
#ifdef GSL_TIMER
static void gsl_monitor_worker(struct work_struct *work)
{

    char read_buf[4]  = {0};
    char init_chip_flag = 0;

    GSL168X_DBG("----------------gsl_monitor_worker-----------------\n");

    #ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
        return;
     #endif

    if(i2c_lock_flag != 0)
            goto queue_monitor_work;
        else
            i2c_lock_flag = 1;

    gsl_ts_read(this_client, 0xb0, read_buf, 4);
    if(read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
        b0_counter ++;
    else
        b0_counter = 0;

    if(b0_counter > 0)
    {
        GSL168X_DBG("======read 0xb0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
        init_chip_flag = 1;
        b0_counter = 0;
        goto queue_monitor_init_chip;
    }

    gsl_ts_read(this_client, 0xb4, read_buf, 4);
    int_2nd[3] = int_1st[3];
    int_2nd[2] = int_1st[2];
    int_2nd[1] = int_1st[1];
    int_2nd[0] = int_1st[0];
    int_1st[3] = read_buf[3];
    int_1st[2] = read_buf[2];
    int_1st[1] = read_buf[1];
    int_1st[0] = read_buf[0];

    if(int_1st[3] == int_2nd[3] && int_1st[2] == int_2nd[2] &&int_1st[1] == int_2nd[1] && int_1st[0] == int_2nd[0])
    {
        GSL168X_DBG("======int_1st: %x %x %x %x , int_2nd: %x %x %x %x ======\n",int_1st[3], int_1st[2], int_1st[1], int_1st[0], int_2nd[3], int_2nd[2],int_2nd[1],int_2nd[0]);
        init_chip_flag = 1;
        goto queue_monitor_init_chip;
    }
#if 1 //version 1.4.0 or later than 1.4.0 read 0xbc for esd checking
        gsl_ts_read(this_client, 0xbc, read_buf, 4);
        if(read_buf[3] != 0 || read_buf[2] != 0 || read_buf[1] != 0 || read_buf[0] != 0)
            bc_counter++;
        else
            bc_counter = 0;
        if(bc_counter > 0)
        {
            GSL168X_DBG("<<<< wanghe ======read 0xbc: %x %x %x %x======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
            init_chip_flag = 1;    // wanghe 2014-03-18 entry here to reset.. mtk_gsl1688_huling2_12_.h 0x74 & 0x7c  !!!!
            bc_counter = 0;
        }
#else

    write_buf[3] = 0x01;
    write_buf[2] = 0xfe;
    write_buf[1] = 0x10;
    write_buf[0] = 0x00;
    gsl_ts_write(this_client, 0xf0, write_buf, 4);
    gsl_ts_read(this_client, 0x10, read_buf, 4);
    gsl_ts_read(this_client, 0x10, read_buf, 4);
    if(read_buf[3] < 10 && read_buf[2] < 10 && read_buf[1] < 10 && read_buf[0] < 10)
        dac_counter ++;
    else
        dac_counter = 0;

    if(dac_counter > 0)
    {
        GSL168X_DBG("======read DAC1_0: %x %x %x %x ======\n",read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
        init_chip_flag = 1;
        dac_counter = 0;
    }
#endif
queue_monitor_init_chip:



    if(init_chip_flag)
        init_chip(this_client);

    i2c_lock_flag = 0;

queue_monitor_work:
    queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 200);//modify by 50

}
#endif

#ifdef GSL_GESTURE
static unsigned int gsl_read_oneframe_data(unsigned int *data,unsigned int addr,unsigned int len)
{
	u8 buf[4];
	int i;

   /*
   	i=0;
    buf[0] = ((addr+i*8)/0x80)&0xff;
    buf[1] = (((addr+i*8)/0x80)>>8)&0xff;
    buf[2] = (((addr+i*8)/0x80)>>16)&0xff;
    buf[3] = (((addr+i*8)/0x80)>>24)&0xff;
    i2c_smbus_write_i2c_block_data(this_client,0xf0,4,buf);
    i2c_smbus_read_i2c_block_data(this_client,((addr+i*8)%0x80+8)&0x5c,8,(char *)&data[i*2]);
    i2c_smbus_read_i2c_block_data(this_client,(addr+i*8)%0x80,8,(char *)&data[i*2]);


    for(i=0;i<len/2;i++){
        i2c_smbus_read_i2c_block_data(this_client,(addr+i*8)%0x80,8,(char *)&data[i*2]);
    }
    if(len%2){
        i2c_smbus_read_i2c_block_data(this_client,(addr+len*4 - 4)%0x80,4,(char *)&data[len-1]);
    }
    */

   printk("======gsl_read_oneframe_data enter======\n");
   buf[0] = ((addr+i*4)/0x80)&0xff;
   buf[1] = (((addr+i*4)/0x80)>>8)&0xff;
   buf[2] = (((addr+i*4)/0x80)>>16)&0xff;
   buf[3] = (((addr+i*4)/0x80)>>24)&0xff;
   //i2c_smbus_write_i2c_block_data(this_client, 0xf0, 4, buf);
   //i2c_smbus_read_i2c_block_data(this_client, (((addr+i*4)%0x80+8)&0x5f), 4, (char *)&data[i]);
   //i2c_smbus_read_i2c_block_data(this_client, (addr+i*4)%0x80, 4, (char *)&data[i]);
   gsl_ts_write(this_client, 0xf0, buf, 4);
	 gsl_ts_read(this_client, (((addr+i*4)%0x80+8)&0x5f), (char *)&data[i], 4);
	 gsl_ts_read(this_client, (addr+i*4)%0x80, (char *)&data[i], 4);

   for(i=0;i<len;i++)
   {
	   //i2c_smbus_read_i2c_block_data(this_client, (addr+i*4)%0x80, 4, (char *)&data[i]);
	   gsl_ts_read(this_client, (addr+i*4)%0x80, (char *)&data[i], 4);
	   printk("gsl_read_oneframe_data[%d] = 0x%08x\n", i, data[i]);
   	}
    return len;
}
#endif

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
    u32 *u32_buf = (int *)buf;
    *u32_buf = *fw;
}


static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)

{
    u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
    u8 send_flag = 1;
    u8 *cur = buf + 1;
    u32 source_line = 0;
    u32 source_len;
    struct fw_data *ptr_fw;

    GSL168X_DBG("=============gsl_load_fw start==============\n");


    ptr_fw = (struct fw_data *)GSL_DOWNLOAD_DATA;
    source_len = data_len;

    for (source_line = 0; source_line < source_len; source_line++)
    {
        /* init page trans, set the page val */
        if (GSL_PAGE_REG == ptr_fw[source_line].offset)
        {
            fw2buf(cur, &ptr_fw[source_line].val);
            gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
            send_flag = 1;
        }
        else
        {
            if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
                    buf[0] = (u8)ptr_fw[source_line].offset;

            fw2buf(cur, &ptr_fw[source_line].val);
            cur += 4;

            if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
            {
                    gsl_write_interface(client, buf[0], buf, cur - buf - 1);
                    cur = buf + 1;
            }

            send_flag++;
        }
    }

    GSL168X_DBG("=============gsl_load_fw end==============\n");

}

static int test_i2c(struct i2c_client *client)
{
/*  u8 read_buf = 0;
    u8 write_buf = 0x12;
    int ret, rc = 1;

    ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
    if  (ret  < 0)
            rc --;
    else
        GSL168X_DBG("I read reg 0xf0 is %x\n", read_buf);

    msleep(2);
    ret = gsl_ts_write(client, 0xf0, &write_buf, sizeof(write_buf));
    if(ret  >=  0 )
        GSL168X_DBG("I write reg 0xf0 0x12\n");

    msleep(2);
    ret = gsl_ts_read( client, 0xf0, &read_buf, sizeof(read_buf) );
    if(ret <  0 )
        rc --;
    else
        GSL168X_DBG("I read reg 0xf0 is 0x%x\n", read_buf);

    return rc;
    */
    int i =0;
    int err =0;
    u8 buf[4]={0};
    for(i=0;i<5;i++)
    {
        err=gsl_ts_read(client,0xfc,buf,4);
        if(((buf[3]&0xf0)==0xb0)|| ((buf[3]&0xf0)==0x80) || ((buf[3]&0xf0)==0x50) || ((buf[3]&0xf0)==0xa0))
            err = 1;
        else
            err = -1;
        if(err>0)
        {
            printk("[tp-gsl] i2c read 0xfc = 0x%02x%02x%02x%02x\n",
                buf[3],buf[2],buf[1],buf[0]);
            break;
        }
    }


    return (err<0?-1:0);
}

static void startup_chip(struct i2c_client *client)
{
    u8 tmp = 0x00;

#ifdef GSL_NOID_VERSION
    gsl_DataInit(gsl_identy_info.gsl_config_data_id_cfg);
#endif
    gsl_ts_write(client, 0xe0, &tmp, 1);
    msleep(10);
}

#ifdef GSL9XX_IO_CTR
static void gsl_io_control(struct i2c_client *client)
{
    u8 buf[4] = {0};
    int i;
    for(i=0;i<5;i++){
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0xfe;
        buf[3] = 0x1;
        i2c_smbus_write_i2c_block_data(client, 0xf0, 4, buf);
        buf[0] = 0x5;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0x80;
        i2c_smbus_write_i2c_block_data(client, 0x78, 4, buf);
        msleep(5);
    }
    msleep(50);
}
#endif

static void reset_chip(struct i2c_client *client)
{
    u8 tmp = 0x88;
    u8 buf[4] = {0x00};

    gsl_ts_write(client, 0xe0, &tmp, sizeof(tmp));
    msleep(20);
    tmp = 0x04;
    gsl_ts_write(client, 0xe4, &tmp, sizeof(tmp));
    msleep(10);
    gsl_ts_write(client, 0xbc, buf, sizeof(buf));
    msleep(10);
    #ifdef GSL9XX_IO_CTR
    gsl_io_control(client);
    #endif
}

static void clr_reg(struct i2c_client *client)
{
    u8 write_buf[4] = {0};

    write_buf[0] = 0x88;
    gsl_ts_write(client, 0xe0, &write_buf[0], 1);
    msleep(20);
    write_buf[0] = 0x03;
    gsl_ts_write(client, 0x80, &write_buf[0], 1);
    msleep(5);
    write_buf[0] = 0x04;
    gsl_ts_write(client, 0xe4, &write_buf[0], 1);
    msleep(5);
    write_buf[0] = 0x00;
    gsl_ts_write(client, 0xe0, &write_buf[0], 1);
    msleep(20);
}


int gsl_identify_tp(struct i2c_client *client);
static int use_dac_identify(void);
static void gsl_identify_tp_by_dac(struct i2c_client *client);

static void gsl_fw_cfg_from_lcd_size(void)
{
#if defined(CONFIG_PROJ_T30_HUAYI)
  bool b_use_second_fw = false;
  if(SCREEN_MAX_X == 800 && SCREEN_MAX_Y == 1280)
    b_use_second_fw = true;

  if(b_use_second_fw)
  {
    GSL168X_DBG("gsl_fw_cfg_from_lcd_size use second gsl fw by lcd size\n");
    gsl_identy_info.gsl_config_data_id_cfg = gsl_config_data_id_second;
    gsl_identy_info.gsl_fw_data = gslx680_fw_second;
    gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(gslx680_fw_second);
  }
#endif
}

static void init_chip(struct i2c_client *client)
{
    int rc;

    rc = test_i2c(client);
    if(rc < 0)
    {
        GSL168X_DBG("------gslx680 test_i2c error------\n");
        return;
    }
    clr_reg(client);
    reset_chip(client);

    rc = use_dac_identify() ;
    GSL168X_INFO("------use_dac_identify return %d ------\n" , rc );
    if( rc == 0 )
        gsl_identify_tp_by_dac(client);
    else if(0 == gsl_identy_info.gsl_tp_type )
        gsl_identify_tp(client);

    gsl_fw_cfg_from_lcd_size();

    gsl_load_fw( client, gsl_identy_info.gsl_fw_data , gsl_identy_info.gsl_fw_data_size);

    startup_chip(client);
    reset_chip(client);
    startup_chip(client);
}


static void check_mem_data(struct i2c_client *client)
{
    u8 read_buf[4]  = {0};
    u8 buf[4]  = {0};
    msleep(30);

    gsl_ts_read(client,0xfc, read_buf, sizeof(read_buf));
    GSL168X_DBG("#########check mem read 0xfc = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
    gsl_ts_read(client,0xbc, read_buf, sizeof(read_buf));
    GSL168X_DBG("#########check mem read 0xbc = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
    gsl_ts_read(client,0xb4, read_buf, sizeof(read_buf));
    GSL168X_DBG("#########check mem read 0x1b4 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
    gsl_ts_read(client,0xb4, read_buf, sizeof(read_buf));
    GSL168X_DBG("#########check mem read 0x2b4 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
    buf[0] =0x3;
    gsl_ts_write(client,0xf0, buf, 4);
    gsl_ts_read(client,0x0, read_buf, sizeof(read_buf));
    GSL168X_DBG("#########check mem read 0x30 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);  gsl_ts_read(client,0xb4, read_buf, sizeof(read_buf));

    gsl_ts_read(client,0xb0, read_buf, sizeof(read_buf));
    GSL168X_DBG("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
    if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
    {
        GSL168X_DBG("#########check mem read 0xb0 = %x %x %x %x #########\n", read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
        init_chip(client);
    }
}

#ifdef TPD_PROC_DEBUG

static int char_to_int(char ch)
{
    if(ch>='0' && ch<='9')
        return (ch-'0');
    else
        return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
    char temp_data[5] = {0};
    unsigned int tmp=0;

    if('v'==gsl_read[0]&&'s'==gsl_read[1])
    {
#ifdef GSL_NOID_VERSION
        tmp=gsl_version_id();
#else
        tmp=0x20121215;
#endif
        seq_printf(m,"version:%x\n",tmp);
    }
    else if('r'==gsl_read[0]&&'e'==gsl_read[1])
    {
        if('i'==gsl_read[3])
        {
#ifdef GSL_NOID_VERSION
            tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
            seq_printf(m,"gsl_config_data_id_cfg[%d] = ",tmp);
            if(tmp>=0&&tmp<512) {
                seq_printf(m,"%d\n",gsl_identy_info.gsl_config_data_id_cfg[tmp]);
            }
#endif
        }
        else
        {
            gsl_ts_write(this_client,0Xf0,&gsl_data_proc[4],4);
            if(gsl_data_proc[0] < 0x80)
                gsl_ts_read(this_client,gsl_data_proc[0],temp_data,4);
            gsl_ts_read(this_client,gsl_data_proc[0],temp_data,4);

            seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
            seq_printf(m,"%02x",temp_data[3]);
            seq_printf(m,"%02x",temp_data[2]);
            seq_printf(m,"%02x",temp_data[1]);
            seq_printf(m,"%02x};\n",temp_data[0]);
        }
    }
//  *eof = 1;
    return (0);
}
#if 1
static ssize_t gsl_config_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *pPos)
{
    u8 buf[8] = {0};
    char temp_buf[CONFIG_LEN];
    char *path_buf;
#ifdef GSL_NOID_VERSION
    int tmp = 0;
    int tmp1 = 0;
#endif
    GSL168X_DBG("[tp-gsl][%s] \n",__func__);
    if(count > 512)
    {
        return -EFAULT;
    }
    path_buf=kzalloc(count,GFP_KERNEL);
    if(!path_buf)
    {
        GSL168X_DBG("alloc path_buf memory error \n");
        return -1;
    }
    if(copy_from_user(path_buf, buffer, count))
    {
        GSL168X_DBG("copy from user fail\n");
        goto exit_write_proc_out;
    }
    memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
    GSL168X_DBG("[tp-gsl][%s][%s]\n",__func__,temp_buf);

    buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);
    buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
    buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
    buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);

    buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
    buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
    buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
    buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
    if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
    {
        memcpy(gsl_read,temp_buf,4);
        GSL168X_DBG("gsl version\n");
    }
    else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
    {
#if 0//
    cancel_delayed_work_sync(&gsl_timer_check_work);
#endif

        gsl_proc_flag = 1;
        reset_chip(this_client);
    }
    else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
    {
        msleep(20);
        reset_chip(this_client);
        startup_chip(this_client);

        gsl_proc_flag = 0;
    }
    else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
    {
        memcpy(gsl_read,temp_buf,4);
        memcpy(gsl_data_proc,buf,8);
    }
    else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
    {
        gsl_ts_write(this_client,buf[4],buf,4);
    }
#ifdef GSL_NOID_VERSION
    else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
    {
        tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
        tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
        if(tmp1>=0 && tmp1<512)
        {
            gsl_identy_info.gsl_config_data_id_cfg[tmp1] = tmp;
        }
    }
#endif
exit_write_proc_out:
    kfree(path_buf);
    return count;
}
#endif
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
    return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
    .open = gsl_server_list_open,
    .read = seq_read,
    .release = single_release,
    .write = gsl_config_write_proc,
    .owner = THIS_MODULE,
};
#endif


#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y , u8 id)
{
    u16 x_err =0;
    u16 y_err =0;
    u16 filter_step_x = 0, filter_step_y = 0;

    id_sign[id] = id_sign[id] + 1;
    if(id_sign[id] == 1)
    {
        x_old[id] = x;
        y_old[id] = y;
    }

    x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
    y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);

    if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
    {
        filter_step_x = x_err;
        filter_step_y = y_err;
    }
    else
    {
        if(x_err > FILTER_MAX)
            filter_step_x = x_err;
        if(y_err> FILTER_MAX)
            filter_step_y = y_err;
    }

    if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
    {
        filter_step_x >>= 2;
    SCREEN_MAX_Y,   filter_step_y >>= 2;
    }
    else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
    {
        filter_step_x >>= 1;
        filter_step_y >>= 1;
    }
    else if(x_err <= 4*FILTER_MAX && y_err <= 4*FILTER_MAX)
    {
        filter_step_x = filter_step_x*3/4;
        filter_step_y = filter_step_y*3/4;
    }

    x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
    y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);

    x_old[id] = x_new;
    y_old[id] = y_new;
}
#else

static void record_point(u16 x, u16 y , u8 id)
{
    u16 x_err =0;
    u16 y_err =0;

    id_sign[id]=id_sign[id]+1;

    if(id_sign[id]==1){
        x_old[id]=x;
        y_old[id]=y;
    }

    x = (x_old[id] + x)/2;
    y = (y_old[id] + y)/2;

    if(x>x_old[id]){
        x_err=x -x_old[id];
    }
    else{
        x_err=x_old[id]-x;
    }

    if(y>y_old[id]){
        y_err=y -y_old[id];
    }
    else{
        y_err=y_old[id]-y;
    }

    if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
        x_new = x;     x_old[id] = x;
        y_new = y;     y_old[id] = y;
    }
    else{
        if(x_err > 3){
            x_new = x;     x_old[id] = x;
        }
        else
            x_new = x_old[id];
        if(y_err> 3){
            y_new = y;     y_old[id] = y;
        }
        else
            y_new = y_old[id];
    }

    if(id_sign[id]==1){
        x_new= x_old[id];
        y_new= y_old[id];
    }

}
#endif

#ifdef HAVE_TOUCH_KEY

static void report_key(struct gslx680_ts_data *ts, u16 x, u16 y,u8 pressure,u8 id)
{
    u16 i = 0;

    for(i = 0; i < MAX_KEY_NUM; i++)
    {
        if((gsl_key_data[i].x_min < x) && (x < gsl_key_data[i].x_max)&&(gsl_key_data[i].y_min < y) && (y < gsl_key_data[i].y_max))
        {
            key = gsl_key_data[i].key;
            input_report_key(ts->input_dev, key, 1);
            input_sync(ts->input_dev);
            key_state_flag = 1;
            break;
        }
    }
}
#endif

static unsigned int gsl_count_one(unsigned int flag)
{
    unsigned int tmp=0;
    int i =0;

    for (i=0 ; i<32 ; i++) {
        if (flag & (0x1 << i)) {
            tmp++;
        }
    }
    return tmp;
}

static  int use_dac_identify(void)
{
    u8 dac_page_addr_tmp[] = DAC_IDENTIFY_PAGE_ADDRESS ;
    dac_identify_rules_t dac_id2_tmp[] = DAC_IDENTIFY_ID2_RULES ;
    const int sz = sizeof(dac_page_addr_tmp)/sizeof(dac_page_addr_tmp[0]) ;
    int i ;

    if( sz == 1 && dac_page_addr_tmp[0] == 0 )
        return -1 ;

    if( sizeof(dac_id2_tmp)/sizeof(dac_id2_tmp[0]) == 1 )
    {
        if( (dac_id2_tmp[0].operation[0] == 'n' &&  dac_id2_tmp[0].operation[1] == 'o')
            || dac_id2_tmp[0].index == 0xff )
        return -2 ;
    }

    if( sz != 4 ){
        GSL168X_ERR("Error dac_identify_page_address size [%d]  != you setting size [%d] \n",
            sizeof(s_dac_identify_page_address)/sizeof(s_dac_identify_page_address[0]),
            sz);
        return -3 ;
    }

    if( sizeof(dac_id2_tmp)/sizeof(dac_id2_tmp[0]) > 4 ){
        GSL168X_ERR("Error s_dac_id2_condition.rules_size must be <= 4 , you setting size [%d] \n",
                sizeof(dac_id2_tmp)/sizeof(dac_id2_tmp[0]));
        return -4 ;
    }

    for( i=0; i<sz ; i++ )
       s_dac_identify_page_address[i] = dac_page_addr_tmp[(sz-1)-i];

    s_dac_identify_offset = DAC_IDENTIFY_OFFSET ;

    s_dac_id2_condition.rules_size = sizeof(dac_id2_tmp)/sizeof(dac_id2_tmp[0]);
    for( i=0 ; i<s_dac_id2_condition.rules_size ; i++ )
        s_dac_id2_condition.rules[i] = dac_id2_tmp[i];

    return 0 ;
}


static bool dac_is_id_xxxx(const struct dac_id_condition *const dac_id_info, u8 cur_data[])
{
    int  i  ;
#define DO_DAC_COMPARE_STEP(opt)  \
    if((cur_data[dac_id_info->rules[i].index]) opt (dac_id_info->rules[i].reference_value)){\
        GSL168X_DBG(" dac_is_id_xxxx ===== (%d/%d) rd[%d] = %x  opt[%s]  reference_value = %x ==>> Do Next ===\n",\
            (i+1), dac_id_info->rules_size ,\
            (dac_id_info->rules[i].index) , (cur_data[dac_id_info->rules[i].index]), \
            #opt,\
            (dac_id_info->rules[i].reference_value) );\
        continue ; \
    }

    if( dac_id_info->rules_size == 1 && dac_id_info->rules[0].index == DAC_INDEX_ALL ){
        char opt[2] = { dac_id_info->rules[0].operation[0] , dac_id_info->rules[0].operation[1] };
        const u32  all_data = (cur_data[3]<<24)|(cur_data[2]<<16)|(cur_data[1]<<8)|cur_data[0] ;
        bool ret = false  ;
        if( opt[0] == '<' && opt[1] == '=' ){
            ret = all_data <= (dac_id_info->rules[0].reference_value);
        }else if( opt[0] == '>' && opt[1] == '=' ){
            ret = all_data >= (dac_id_info->rules[0].reference_value);
        }else if( opt[0] == '=' && opt[1] == '=' ){
            ret = all_data == (dac_id_info->rules[0].reference_value);
        } else if( opt[0] == '<'){
            ret = all_data < (dac_id_info->rules[0].reference_value);
        } else if( opt[0] == '>'){
            ret = all_data > (dac_id_info->rules[0].reference_value);
        }
        GSL168X_INFO("Do DAC_INDEX_ALL compare !!!!");
        return ret ;
    }


    for( i=0 ; i< dac_id_info->rules_size ; i++ ){
        char opt[2] = {0};
        opt[0] = dac_id_info->rules[i].operation[0];
        opt[1] = dac_id_info->rules[i].operation[1] ;
        if( opt[0] == '<' && opt[1] == '=' ){
            DO_DAC_COMPARE_STEP(<=)
        }else if( opt[0] == '>' && opt[1] == '=' ){
            DO_DAC_COMPARE_STEP(>=)
        }else if( opt[0] == '=' && opt[1] == '=' ){
            DO_DAC_COMPARE_STEP(==)
        }else if( opt[0] == '<'){
            DO_DAC_COMPARE_STEP(<)
        }else if( opt[0] == '>'){
            DO_DAC_COMPARE_STEP(>)
        }
        return false ;
    }
    return true ;
}



static void gsl_identify_tp_by_dac(struct i2c_client *client)
{
    u32 tmp = 0 ;
    u8  readbuf[4] = {0} ;
    int i;

    clr_reg(client);
    reset_chip(client);
    tmp = ARRAY_SIZE(GSLX680_FW_CFG1);
    gsl_load_fw(client,GSLX680_FW_CFG1,tmp);
    startup_chip(client);
    msleep(200);

    for( i = 0; i < 30; i++ ) {
        i2c_smbus_read_i2c_block_data(client, 0xb4, 4, readbuf);
        tmp /*count*/ = ((readbuf[3]<<24)|(readbuf[2]<<16)|(readbuf[1]<<8)|(readbuf[0]));
        if( tmp >= 41 )
            break;
        msleep(10);
    }

    // GSL168X_DBG("======gsl_identify_tp_by_dac the , test 0xb4(0x%x) ======\n",tmp);
    i = sizeof(s_dac_identify_page_address)/sizeof(s_dac_identify_page_address[0]);
    gsl_ts_write(client, 0xf0, s_dac_identify_page_address,i);
    msleep(20);

    i = 2 ;
    while(i--){
        if ( gsl_ts_read(client, s_dac_identify_offset , readbuf, 4 ) >= 0 )
            break ;
    }
    if( i == -1 ){
        tmp = (s_dac_identify_page_address[3] << 24)|(s_dac_identify_page_address[2]<<16)|
              (s_dac_identify_page_address[1] << 8) |(s_dac_identify_page_address[0]) ;
        GSL168X_ERR("DAC_GET page_addr[%x],offset[%x] Error",tmp , s_dac_identify_offset ) ;
        return ;
    }

    GSL168X_DBG("======DAC read[%d] (%s)0x%02x%02x%02x%02x,(%s)0x%02x : (%s)[3-0] = %02x,%02x,%02x,%02x======\n",
        i,
        "page_address" ,s_dac_identify_page_address[3],s_dac_identify_page_address[2],
        s_dac_identify_page_address[1],s_dac_identify_page_address[0],
        "offset" , s_dac_identify_offset ,
        "return_val" , readbuf[3], readbuf[2], readbuf[1], readbuf[0]);


    if( dac_is_id_xxxx(&s_dac_id2_condition, readbuf) == true )
        gsl_identy_info.gsl_tp_type = 0xDAC02 ; /* dac id 2 */
    else
        gsl_identy_info.gsl_tp_type = 0xDAC01 ; /* dac id 1 :cfg1 [def] gsl_config_data_id , GSLX680_FW */

    GSL168X_DBG("======dac_is_id_xxxx(&s_dac_id2_condition, readbuf) [%d] gsl_tp_type(%d)======\n",
        dac_is_id_xxxx(&s_dac_id2_condition, readbuf),
        gsl_identy_info.gsl_tp_type );

    gsl_identy_info.gsl_chip_base = (readbuf[3]<<24)|(readbuf[2]<<16)|(readbuf[1]<<8)|readbuf[0];

    switch(gsl_identy_info.gsl_tp_type ){
        case 0xDAC01 :
            gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG1 ;
            gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG1 ;
            gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG1) ;
            break ;
        case 0xDAC02 :
            gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG2 ;
            gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG2 ;
            gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG2) ;
            break ;
        case 0xDAC03 :
            gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG3 ;
            gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG3 ;
            gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG3) ;
            break ;
        case 0xDAC04 :
            gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG4 ;
            gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG4 ;
            gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG4) ;
            break ;
    }
}

 int gsl_identify_tp(struct i2c_client *client)
{
    u8 buf[4];
    int flag=0 , err=1;
    unsigned int tmp,tmp0;
    unsigned int tmp1,tmp2,tmp3,tmp4;
    u32 num;

    if( GSL_CHIP_1 == GSL_CHIP_2 )
    {
        printk("GSL168X <%s> line:%d  GSL_CHIP_1 == GSL_CHIP_2 , do nothing!!! \n",__func__,__LINE__);
        return 0 ;
    }

identify_tp_repeat:
    clr_reg(client);
    reset_chip(client);
    num = ARRAY_SIZE(GSL_TP_CHECK_FW);
    gsl_load_fw(client,GSL_TP_CHECK_FW,num);
    startup_chip(client);
    msleep(200);
    i2c_smbus_read_i2c_block_data(client,0xb4,4,buf);
    GSL168X_DBG("the test 0xb4 = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],buf[1],buf[0]);

    if (((buf[3] << 8) | buf[2]) > 1) {
        GSL168X_DBG("[TP-GSL][%s] is start ok\n",__func__);
        msleep(20);
        i2c_smbus_read_i2c_block_data(client,0xb8,4,buf);
        tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
        GSL168X_DBG("the test 0xb8 = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],buf[1],buf[0]);
        gsl_identy_info.gsl_chip_base = tmp ;

        tmp1 = gsl_count_one(GSL_CHIP_1^tmp);
        tmp0 = gsl_count_one((tmp&GSL_CHIP_1)^GSL_CHIP_1);
        tmp1 += tmp0*GSL_C;
        GSL168X_DBG("[TP-GSL] tmp1 = %d\n",tmp1);

        tmp2 = gsl_count_one(GSL_CHIP_2^tmp);
        tmp0 = gsl_count_one((tmp&GSL_CHIP_2)^GSL_CHIP_2);
        tmp2 += tmp0*GSL_C;
        GSL168X_DBG("[TP-GSL] tmp2 = %d\n",tmp2);

        tmp3 = gsl_count_one(GSL_CHIP_3^tmp);
        tmp0 = gsl_count_one((tmp&GSL_CHIP_3)^GSL_CHIP_3);
        tmp3 += tmp0*GSL_C;
        GSL168X_DBG("[TP-GSL] tmp3 = %d\n",tmp3);

        tmp4 = gsl_count_one(GSL_CHIP_4^tmp);
        tmp0 = gsl_count_one((tmp&GSL_CHIP_4)^GSL_CHIP_4);
        tmp4 += tmp0*GSL_C;
        GSL168X_DBG("[TP-GSL] tmp4 = %d\n",tmp4);

        if (0xffffffff == GSL_CHIP_1) {
            tmp1=0xffff;
        }
        if (0xffffffff == GSL_CHIP_2) {
            tmp2=0xffff;
        }
        if (0xffffffff == GSL_CHIP_3) {
            tmp3=0xffff;
        }
        if (0xffffffff == GSL_CHIP_4) {
            tmp4=0xffff;
        }

        GSL168X_DBG("[TP-GSL] tmp = %d\n",tmp);
        GSL168X_DBG("[TP-GSL] tmp1 = %d\n",tmp1);
        GSL168X_DBG("[TP-GSL] tmp2 = %d\n",tmp2);
        GSL168X_DBG("[TP-GSL] tmp3 = %d\n",tmp3);
        GSL168X_DBG("[TP-GSL] tmp4 = %d\n",tmp4);
        tmp = tmp1;
        if (tmp1 > tmp2) {
            tmp = tmp2;
        }

        if(tmp > tmp3){
            tmp = tmp3;
        }
        if(tmp>tmp4){
            tmp = tmp4;
        }

        gsl_identy_info.gsl_tp_type =
            tmp == tmp1 ? 1 :
            tmp == tmp2 ? 2 :
            tmp == tmp3 ? 3 :
            tmp == tmp4 ? 4 :
            0 ;

        switch(gsl_identy_info.gsl_tp_type ){
            case 1 :
                gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG1 ;
                gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG1 ;
                gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG1) ;
                break ;
            case 2 :
                gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG2 ;
                gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG2 ;
                gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG2) ;
                break ;
            case 3 :
                gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG3 ;
                gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG3 ;
                gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG3) ;
                break ;
            case 4 :
                gsl_identy_info.gsl_config_data_id_cfg = GSL_CONFIG_DATA_ID_CFG4 ;
                gsl_identy_info.gsl_fw_data = GSLX680_FW_CFG4 ;
                gsl_identy_info.gsl_fw_data_size = ARRAY_SIZE(GSLX680_FW_CFG4) ;
                break ;
        }

        err = 1;
    } else {
        flag++;
        if(flag < 3) {
            goto identify_tp_repeat;
        }
        err = 0;
    }
    return err;
}


static void report_data(struct gslx680_ts_data *ts, u16 x, u16 y, u8 pressure, u8 id)
{
    GSL168X_DBG("report_data: id %d, x %d, y %d \n",id, x, y);
#if defined(PRJ_FEATURE_H_BOARD_TP_AXIS_MIRROR)
    if(x <= SCREEN_MAX_X && y <= SCREEN_MAX_Y)
    {
    	y = SCREEN_MAX_Y - y;
    	x = SCREEN_MAX_X - x;
    }
#endif

#ifdef VIRTUAL_VEBRATOR_FUC
        // input_mt_slot(ts->input_dev, id);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, pressure);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
        //input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 1);
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
        input_report_key(ts->input_dev, BTN_TOUCH, 1);
        input_mt_sync(ts->input_dev);
#else
#ifdef HAVE_TOUCH_KEY
        if(x > SCREEN_MAX_X || y > SCREEN_MAX_Y)
        {				
                report_key(ts,x,y,pressure,id);
                return;
        }
#endif

        input_mt_slot(ts->input_dev, id);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, pressure);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 1);
        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
#endif
}

#if defined(CONFIG_TOUCHSCREEN_WITH_PROXIMITY)
/*******************************enable property start************************************/
void TP_proximity_mode_switch(int on)///enable  ctp sensor
{
    //int proximity_switch;
    u8 data[4] = {0};
    if (1 == on)
    {
        PROXIMITY_SWITCH = 1;
        data[3]=0;
        data[2]=0;
        data[1]=0;
        data[0]=0x4;
        gsl_ts_write(this_client,0xf0,data,4);
        gsl_ts_write(this_client,0xf0,data,4);
        data[3]=0;
        data[2]=0;
        data[1]=0;
        data[0]=0x2;
        gsl_ts_write(this_client,0x0,data,4);
        gsl_ts_write(this_client,0x0,data,4);
        GSL168X_INFO("tpd-ps function is on\n");
    }
    else
    {
        PROXIMITY_SWITCH = 0;
        data[3]=0;
        data[2]=0;
        data[1]=0;
        data[0]=0x4;
        gsl_ts_write(this_client,0xf0,data,4);
        gsl_ts_write(this_client,0xf0,data,4);
        data[3]=0;
        data[2]=0;
        data[1]=0;
        data[0]=0x0;
        gsl_ts_write(this_client,0x0,data,4);
        gsl_ts_write(this_client,0x0,data,4);
        GSL168X_INFO("tpd-ps function is off\n");
    }
}

static ssize_t do_proximity_enable_store(unsigned long on_off ,size_t size)
{
    if(on_off==0)
    {
        printk("proximity is disable\n");
        TP_proximity_mode_switch(0);
        atomic_set(&prox_contrl_state,0);
    }
    else
    {
        printk("proximity is enable \n");

        if(in_suspend)
        {
            printk("%s: working in suspend mode, shoud resume first!!!\n", __func__);
            #ifdef GSL_GESTURE
            printk("===lxl-gslx680_ts_resume gsl_gesture_flag = %d\n",gsl_gesture_flag);

            if(gsl_gesture_flag == 1) {
                irq_set_irq_type(this_client->irq,IRQF_TRIGGER_RISING);
                gsl_quit_doze(this_client);
                gsl_irq_mode_change(this_client,0);
            }
            #endif
            gpio_set_value(sprd_3rdparty_gpio_tp_rst, 1);

            msleep(20);
            reset_chip(this_client);
            startup_chip(this_client);
            tp_irq_enable();
            in_suspend = 0;
        }

        TP_proximity_mode_switch(1);
        atomic_set(&prox_contrl_state,1);
    }
    return size ;
}


static ssize_t proximity_nearfar_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s\n", s_nearfar?"far":"near");
}
static DEVICE_ATTR(nearfar, 0644,proximity_nearfar_show, NULL);

#ifndef SENSORHUB_PROXIMITY_TOUCHSCREEN
static ssize_t proximity_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", atomic_read(&prox_contrl_state));
}

static ssize_t proximity_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned long on_off = simple_strtoul(buf, NULL, 10);
    return  do_proximity_enable_store(on_off,size);
}
static DEVICE_ATTR(enable, 0644,proximity_enable_show, proximity_enable_store);
#endif

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
#endif


static void gslx680_ts_worker(struct work_struct *work)
{
    int rc;
    u8 id, touches;
    u16 x, y;
    int i = 0;
#ifdef GSL_NOID_VERSION
    struct gsl_touch_info cinfo;
    u32 tmp1;
    u8 buf[4];
#endif
    struct gslx680_ts_data *ts = i2c_get_clientdata(this_client);
#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
    u8 reg_val[4] = {0};
#endif
    GSL168X_DBG("gslx680_ts_pen_irq_work \n");

#ifdef GSL_TIMER
    if(i2c_lock_flag != 0)
        goto i2c_lock_schedule;
    else
        i2c_lock_flag = 1;
#endif


#ifdef TPD_PROC_DEBUG
    if(gsl_proc_flag == 1)
        goto schedule;
#endif

#ifdef GSL_GESTURE
    if(gsl_gesture_flag==1 && GE_NOWORK == gsl_gesture_status){
        goto schedule;
    }
#endif
#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
if(PROXIMITY_SWITCH == 1)
{
    gsl_ts_read(this_client, 0xfc, reg_val, 4);
    gsl_ts_read(this_client, 0xac, reg_val, 4);
    if (reg_val[0] == 1)//near
    {
        if(PROXIMITY_STATE==1)
        {
            GSL168X_DBG("%s() Line:%d Tp already to near\n", __FUNCTION__, __LINE__);
        }
        else
        {
            PROXIMITY_STATE = 1;
        }
        s_nearfar = 1 ;
    }
    else if(reg_val[0] == 0)//far
    {
        if(PROXIMITY_STATE==0)
        {
            GSL168X_DBG("%s() Line:%d Tp already to far\n", __FUNCTION__, __LINE__);
        }
        else
        {
            PROXIMITY_STATE = 0;
        }
        s_nearfar = 0 ;
    }
}
    #ifdef SENSORHUB_PROXIMITY_TOUCHSCREEN
    if(sensorhub_prox_do && sensorhub_prox_do->prox_reprot_func){
        switch(PROXIMITY_STATE){
            case 0 :
                sensorhub_prox_do->prox_reprot_func(ts->input_dev, SENSORHUB_FACE_STATUS_FAR);
                break;
            case 1 :
                sensorhub_prox_do->prox_reprot_func(ts->input_dev, SENSORHUB_FACE_STATUS_NEAR);
                break;
            default:
                break ;
        }
    }
    #else /* End of SENSORHUB_PROXIMITY_TOUCHSCREEN */
    if(PROXIMITY_SWITCH == 1)
        input_report_abs(ts->input_dev, ABS_DISTANCE, !PROXIMITY_STATE);
    input_report_key(ts->input_dev, BTN_TOUCH, 0);
    input_mt_sync(ts->input_dev);
    //input_sync(ts->input_dev);
    #endif
#endif

    rc = gsl_ts_read(this_client, 0x80, ts->touch_data, sizeof(ts->touch_data));
    GSL168X_DBG("--zxw--0x80--ts->touch_data[3]=%x [2]=%x [1]=%x [0]=%x--\n",ts->touch_data[3],ts->touch_data[2],ts->touch_data[1],ts->touch_data[0]);
    if (rc < 0)
    {
        dev_err(&this_client->dev, "read failed\n");
        goto schedule;
    }

    touches = ts->touch_data[0];
#ifdef GSL_NOID_VERSION
    cinfo.finger_num = touches;
    GSL168X_DBG("tp-gsl  finger_num = %d\n",cinfo.finger_num);
    for(i = 0; i < (touches < MAX_CONTACTS ? touches : MAX_CONTACTS); i ++)
    {
        cinfo.x[i] = join_bytes( ( ts->touch_data[4 * i + 7] & 0xf),ts->touch_data[4 * i + 6]);
        cinfo.y[i] = join_bytes(ts->touch_data[4 * i + 5],ts->touch_data[4 * i +4]);
        cinfo.id[i] = (ts->touch_data[4*i + 7]&0xf0)>>4;
        GSL168X_DBG("tp-gsl  x = %d y = %d \n",cinfo.x[i],cinfo.y[i]);
    }
    cinfo.finger_num=(ts->touch_data[3]<<24)|(ts->touch_data[2]<<16)
        |(ts->touch_data[1]<<8)|(ts->touch_data[0]);
    gsl_alg_id_main(&cinfo);
    tmp1=gsl_mask_tiaoping();
    GSL168X_DBG("[tp-gsl] tmp1=%x\n",tmp1);
    if(tmp1>0&&tmp1<0xffffffff)
    {
        buf[0]=0xa;buf[1]=0;buf[2]=0;buf[3]=0;
        gsl_ts_write(this_client,0xf0,buf,4);
        buf[0]=(u8)(tmp1 & 0xff);
        buf[1]=(u8)((tmp1>>8) & 0xff);
        buf[2]=(u8)((tmp1>>16) & 0xff);
        buf[3]=(u8)((tmp1>>24) & 0xff);
        GSL168X_DBG("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",tmp1,buf[0],buf[1],buf[2],buf[3]);
        gsl_ts_write(this_client,0x8,buf,4);
    }
    touches = cinfo.finger_num;
#endif

#ifdef GSL_GESTURE
    GSL168X_DBG("lxl-gsl_gesture_status1111111=%d  gsl_gesture_flag=%d--\n",gsl_gesture_status,gsl_gesture_flag);
    if(GE_ENABLE == gsl_gesture_status && gsl_gesture_flag == 1){
        unsigned int key_data=0;
        int tmp_c = 0;
        tmp_c = gsl_obtain_gesture();

        GSL168X_DBG("lxl-gsl_obtain_gesture():tmp_c=%d\n",tmp_c);
        switch(tmp_c){
        case (int)'C':
            key_data = KEY_C;
            tmp_c = 'c';
            break;
         case (int)'E':
            key_data = KEY_E;
            tmp_c = 'e';
            break;
         case (int)'M':
            key_data = KEY_M;
            tmp_c = 'm';
            break;
        case (int )'O':
            key_data = KEY_O;
            tmp_c = 'o';
            break;
        case (int)'V':
            key_data = KEY_SCROLLDOWN;
            tmp_c = 'v';
            break;
        case (int)'S':
            key_data = KEY_S;
            tmp_c = 's';
            break;
        case (int)'W':
            key_data = KEY_W;
            tmp_c = 'w';
            break;
        case (int)'Z':
            key_data = KEY_Z;
            tmp_c = 'z';
            break;
        case 0xa1fa:
            key_data = KEY_F11;
            tmp_c = 'R';
            break;
        case 0xa1fb:
            key_data = KEY_F12;
            tmp_c = 'L';
            break;
        case 0xa1fd:
            key_data = KEY_F9;
            tmp_c = 'D';
            break;
        case 0xa1fc:
            key_data = KEY_F10;
            tmp_c = 'U';
            break;
        case '*':
            key_data = KEY_POWER;
            tmp_c = 'd';  //double clik
            break;
        default:
            GSL168X_DBG("zxw---can't reconition gsl_gesture_[key_data] = %d, [Char] = %c\n",key_data,key_data);
            break;
        }

        if(key_data != 0){

            msleep(10);
            gsl_gesture_c = (char)(tmp_c & 0xff);
            GSL168X_DBG("tp-gsl gesture eeeeeeeeeeeeeee %c\n",gsl_gesture_c);

            input_report_key(ts->input_dev,key_data,1); //KEY_POWER
            input_sync(ts->input_dev);
            input_report_key(ts->input_dev,key_data,0);  //KEY_POWER
            input_sync(ts->input_dev);

            msleep(400);
            power_key_status = 1;
        }
        goto schedule;
    }
#endif




    for(i = 1; i <= MAX_CONTACTS; i ++)
    {
        if(touches == 0)
            id_sign[i] = 0;
        id_state_flag[i] = 0;
    }
    for(i = 0; i < (touches > MAX_FINGERS ? MAX_FINGERS : touches); i ++)
    {
    #ifdef GSL_NOID_VERSION
        id = cinfo.id[i];
        x =  cinfo.x[i];
        y =  cinfo.y[i];
    #else
        id = ts->touch_data[4 * i + 7] >> 4;
        x = join_bytes( ( ts->touch_data[4 * i + 7] & 0xf),ts->touch_data[4 * i + 6]);
        y = join_bytes(ts->touch_data[4 * i + 5],ts->touch_data[4 * i +4]);

    #endif

        #if 0
        //huangxl 2014.03.07
        GSL168X_DBG("=====get x=%d 1st====\r\n");
        x = SCREEN_MAX_X>=x?SCREEN_MAX_X-x:0;
        GSL168X_DBG("=====get x=%d 2nd====\r\n");
        #endif

        if(1 <= id && id <= MAX_CONTACTS)
        {
        #ifdef FILTER_POINT
            filter_point(x, y ,id);
        #else
            record_point(x, y , id);
        #endif
            report_data(ts, x, y, 10, id);
            id_state_flag[id] = 1;
        }

    }
    for(i = 1; i <= MAX_CONTACTS; i ++)
    {
        if( (0 == touches) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
        {
            //if(key_state_flag==0){
#ifndef VIRTUAL_VEBRATOR_FUC
                input_mt_slot(ts->input_dev, i);
                            input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
                            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
#endif          //}
            id_sign[i]=0;
        }
        id_state_old_flag[i] = id_state_flag[i];
    }
#ifdef VIRTUAL_VEBRATOR_FUC
    if(0 == touches)
    {
        input_report_key(ts->input_dev, BTN_TOUCH, 0);
        input_mt_sync(ts->input_dev);
    }
#else
    if(0 == touches)
    {

    #ifdef HAVE_TOUCH_KEY
        if(key_state_flag)
        {
            input_report_key(ts->input_dev, key, 0);
            input_sync(ts->input_dev);
            key_state_flag = 0;
        }
        else
    #endif
        {
            input_mt_slot(ts->input_dev, i);
                        input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
                        input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
        }
    }
#endif
    input_sync(ts->input_dev);

schedule:
#ifdef GSL_TIMER
    i2c_lock_flag = 0;
i2c_lock_schedule:
#endif
    tp_irq_enable();
}


static irqreturn_t gslx680_ts_interrupt(int irq, void *dev_id)
{

    struct gslx680_ts_data *gslx680_ts = (struct gslx680_ts_data *)dev_id;

#ifdef GSL_GESTURE
    __pm_wakeup_event(gsl_wake_lock, msecs_to_jiffies(2000));
#endif

    tp_irq_disable();
    if (!work_pending(&gslx680_ts->pen_event_work)) {
        queue_work(gslx680_ts->ts_workqueue, &gslx680_ts->pen_event_work);
    }

    return IRQ_HANDLED;
}

#ifdef GSL_GESTURE
static void gsl_enter_doze(struct i2c_client *client)
{
    u8 buf[4] = {0};
    int i = 0;

    GSL168X_DBG("===lxl-gsl_enter_doze====\n");

    for(i=0;i<3;i++)
        {
            buf[0] = 0xa;
            buf[1] = 0;
            buf[2] = 0;
            buf[3] = 0;
            gsl_ts_write(client,0xf0,buf,4);
            buf[0] = 0;
            buf[1] = 0;
            buf[2] = 0x1;
            buf[3] = 0x5a;
            gsl_ts_write(client,0x8,buf,4);

        }
    gsl_ts_read(this_client, 0x8, buf, 4);
    //printk("lxl 1enter_doze_data =%x %x %x %x\n",buf[3],buf[2],buf[1],buf[0]);
    gsl_ts_read(this_client, 0x8, buf, 4);
    msleep(5);
    gsl_gesture_status = GE_ENABLE;
    printk("%d: lxl gsl_gesture_status : %d\n", __LINE__, gsl_gesture_status);
}

static void gsl_quit_doze(struct i2c_client *client)
{
    u8 buf[4] = {0};

    gsl_gesture_status = GE_DISABLE;
    printk("%d: lxl gsl_gesture_status : %d\n", __LINE__, gsl_gesture_status);
    gslx680_hw_reset(gslx680_ts);
    GSL168X_DBG("===lxl-gsl_quit_doze====\n");
    buf[0] = 0xa;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    gsl_ts_write(this_client,0xf0,buf,4);
    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0x5a;
    gsl_ts_write(this_client,0x8,buf,4);
    msleep(10);
}


static void gsl_irq_mode_change(struct i2c_client *client,u32 flag)
{
    u8 buf[4]={0};
    buf[0] = 0x6;
    buf[1] = 0;
    buf[2] = 0;
    buf[3] = 0;
    gsl_ts_write(client,0xf0,buf,4);
    if(flag == 1){
        buf[0] = 0;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
    }else if(flag == 0){
        buf[0] = 1;
        buf[1] = 0;
        buf[2] = 0;
        buf[3] = 0;
    }else{
        return;
    }
    gsl_ts_write(client,0x1c,buf,4);
}

static ssize_t show_gesture(struct device *dev, struct device_attribute *attr, char *buf)
{
    int len = -1;
    if ( buf != NULL ){
        len = snprintf( buf, PAGE_SIZE, "%c", gsl_gesture_c );
        printk("xxxxx mycat gesture: buf=%d\n\n", *buf);
        return len;
    }
    return len;
}
static ssize_t store_gesture(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    return 1;
}
static DEVICE_ATTR(gesture, S_IRUGO | S_IWUSR, show_gesture, store_gesture);

static unsigned int gsl_gesture_init(void)
{
#ifdef GSL_GESTURE
    Gesture_class = class_create(THIS_MODULE, "sprd-tpd");
    if(IS_ERR(Gesture_class))
    {
        GSL168X_DBG("Failed to create class(firmware)!\n");
    }

    Gesture_cmd_dev = device_create(Gesture_class, NULL, 0, NULL, "device");
    if(IS_ERR(Gesture_cmd_dev))
    {
        GSL168X_DBG("Failed to create device(firmware_cmd_dev)!\n");
    }

    device_create_file(Gesture_cmd_dev, &dev_attr_gesture);
#endif

    return 1;
}

#endif

static int gslx680_ts_suspend(struct device *pDevice)
{
#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
    printk("qzhu suspend1, prox_contrl_state1=%d\n", atomic_read(&prox_contrl_state));
        if ((atomic_read(&prox_contrl_state) == 1)||(PROXIMITY_SWITCH==1))
        {
            printk("tpdsuspend1\n");
            in_suspend =1;
            return 0;
        }
        gsl_halt_flag = 1;
#endif
    GSL168X_DBG("===lxl-gslx680_ts_suspends====\n");

#ifdef GSL_TIMER
    cancel_delayed_work_sync(&gsl_monitor_work);
    i2c_lock_flag = 0;
#endif

#ifdef GSL_GESTURE
  GSL168X_DBG("lxl-gslx680_ts_suspend gsl_gesture_flag = %d\n",gsl_gesture_flag);
    power_key_status = 0;
    if(gsl_gesture_flag == 1){
        gsl_irq_mode_change(this_client,1);
        irq_set_irq_type(this_client->irq,IRQF_TRIGGER_HIGH | IRQF_NO_SUSPEND| IRQF_ONESHOT);
        enable_irq_wake(this_client->irq);
        gsl_enter_doze(this_client);
        in_suspend =1;
        return 0;
    }
#endif
    tp_irq_disable();
    gpio_direction_output(sprd_3rdparty_gpio_tp_rst,0);
    GSL168X_DBG("==gslx680_ts_suspend----end=\n");
    in_suspend =1;
    return 0;
}

static int gslx680_ts_resume(struct device *pDevice)
{
  GSL168X_DBG("===lxl-gslx680_ts_resume====\n");
#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
    printk("qzhu resume1, prox_contrl_state1=%d\n", atomic_read(&prox_contrl_state));
    if ((atomic_read(&prox_contrl_state) == 1)&&(gsl_halt_flag == 0))
    {
        printk("tpdresume1\n");
        TP_proximity_mode_switch(1);
        in_suspend =0;
        return 0;
    }
    gsl_halt_flag = 0;
#endif

#ifdef GSL_GESTURE
    GSL168X_DBG("===lxl-gslx680_ts_resume gsl_gesture_flag = %d\n",gsl_gesture_flag);

    if(gsl_gesture_flag == 1) {

        irq_set_irq_type(this_client->irq,IRQF_TRIGGER_RISING);
        gsl_quit_doze(this_client);
        gsl_irq_mode_change(this_client,0);
    }
    power_key_status = 0;
#endif
    gpio_set_value(sprd_3rdparty_gpio_tp_rst, 1);

    msleep(20);
    reset_chip(this_client);
    startup_chip(this_client);

#ifdef GSL_TIMER
    queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 300);
    i2c_lock_flag = 0;
#endif

    tp_irq_enable();
    GSL168X_DBG("--zxw--==gslx680_ts_resume end=\n");
    in_suspend =0;
    return 0;
}

int glb_gslx680_ts_suspend(void) { return gslx680_ts_suspend(NULL);}
int glb_gslx680_ts_resume(void) {return gslx680_ts_resume(NULL);}

static void gslx680_hw_reset(struct gslx680_ts_data *gslx680_ts)
{
    // gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
    // gpio_set_value(sprd_3rdparty_gpio_tp_rst, 0);
    // msleep(10);
    // gpio_set_value(sprd_3rdparty_gpio_tp_rst, 1);
    // msleep(200);
    
    gpio_direction_output(gslx680_ts->reset_pin, 1);
    gpio_set_value(gslx680_ts->reset_pin, 0);
    msleep(10);
    gpio_set_value(gslx680_ts->reset_pin, 1);
    msleep(200);
}

static int gslx680_ts_hw_init(struct gslx680_ts_data *gslx680_ts)
{

    // gpio_request(sprd_3rdparty_gpio_tp_irq, "ts_irq");
    // gpio_request(sprd_3rdparty_gpio_tp_rst, "ts_rst");
    // gpio_direction_output(sprd_3rdparty_gpio_tp_rst, 1);
    // gpio_direction_input(sprd_3rdparty_gpio_tp_irq);

    // msleep(100);
    // //reset
    // gslx680_hw_reset();

	int ret = 0;
    GSL168X_DBG(KERN_INFO "-------------gslx680_ts_hw_init start!!!!\n");

	if(gpio_is_valid(gslx680_ts->int_pin)){
		ret = gpio_request(gslx680_ts->int_pin, "int_pin");
		if (ret) {
			pr_err("Unable to request int_pin [%d]\n", gslx680_ts->int_pin);
			ret = -EINVAL;
			goto err_pwr_off;
		}
		ret = gpio_direction_input(gslx680_ts->int_pin);
		if (ret) {
			pr_err("Unable to set direction for int_pin [%d]\n", gslx680_ts->int_pin);
			goto err_free_int_pin;
		}
	} else {
		pr_err("Invalid int_pin [%d]!\n", gslx680_ts->int_pin);
		ret = -EINVAL;
		goto err_pwr_off;
	}

	if (gpio_is_valid(gslx680_ts->reset_pin)) {
		ret = gpio_request(gslx680_ts->reset_pin, "reset_pin");
		if (ret) {
			pr_err("Unable to request reset_pin [%d]\n", gslx680_ts->reset_pin);
			goto err_free_int_pin;
		}

		ret = gpio_direction_output(gslx680_ts->reset_pin, 1);
		if (ret) {
			pr_err("Unable to set direction for reset gpio [%d]\n", gslx680_ts->reset_pin);
			goto err_free_reset_pin;
		}
	} else {
		pr_err("Invalid reset_pin [%d]!\n", gslx680_ts->reset_pin);
		ret = -EINVAL;
		goto err_free_int_pin;
	}

    gslx680_hw_reset(gslx680_ts);

	return ret;

err_free_reset_pin:
	if (gpio_is_valid(gslx680_ts->reset_pin))
		gpio_free(gslx680_ts->reset_pin);

err_free_int_pin:
	if (gpio_is_valid(gslx680_ts->int_pin))
		gpio_free(gslx680_ts->int_pin);
err_pwr_off:
	return ret;
}

/* **********************************************************

        sensorhub setting

************************************************************ */
#if defined(SENSORHUB_PROXIMITY_TOUCHSCREEN)
static  int sensorhub_prox_ctrl(int enable){
    do_proximity_enable_store(enable,0);
    return 0 ;
}

static int get_prox_enble_stat(void){
    return PROXIMITY_SWITCH ;
}
static int get_prox_active_stat(void){
    return PROXIMITY_STATE ;
}

static struct sensorhub_setting  sensorhub_info = {
    .sensorhub_prox_ctrl = sensorhub_prox_ctrl ,
    .get_prox_active_stat = get_prox_active_stat ,
    .get_prox_enble_stat = get_prox_enble_stat ,
    .use_wakelock = false ,
    .input = NULL ,
    .ic_name = "gslx680"
};
#endif

static int gslx680_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct gslx680_ts_data *gslx680_ts;
    struct input_dev *input_dev;
    struct device *dev= &client->dev;
	struct device_node *np = dev->of_node;
    int err = 0;
    u8 read_buf[4]  = {0};

    printk(" %s Enter \n",__func__);

    revo_get_tp_size(&SCREEN_MAX_X, &SCREEN_MAX_Y);
    printk("%s SCREEN_MAX_X[%d],SCREEN_MAX_Y[%d]\n",__func__,SCREEN_MAX_X,SCREEN_MAX_Y);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("---zdy-- i2c check failed !!!! \n");
        err = -ENODEV;
        goto exit_alloc_platform_data_failed;
    }

    gslx680_ts = kzalloc(sizeof(*gslx680_ts), GFP_KERNEL);
    if (!gslx680_ts)    {
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }
    
    gslx680_ts->reset_pin = of_get_named_gpio(np, "reset-gpio", 0);
    gslx680_ts->int_pin = of_get_named_gpio(np, "irq-gpio", 0);
    sprd_3rdparty_gpio_tp_rst = gslx680_ts->reset_pin;
    sprd_3rdparty_gpio_tp_irq = gslx680_ts->int_pin;
    client->irq = gpio_to_irq(gslx680_ts->int_pin);

    g_gslx680_ts = gslx680_ts;
    gslx680_ts_hw_init(gslx680_ts);
    err = test_i2c(client);
    if(err < 0)
    {
        printk("------gslx680 test_i2c error !!!------\n");
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }
    i2c_set_clientdata(client, gslx680_ts);
    this_client = client;

    printk("I2C addr=%x", client->addr);

    spin_lock_init(&irq_lock);

    INIT_WORK(&gslx680_ts->pen_event_work, gslx680_ts_worker);
    gslx680_ts->ts_workqueue = create_singlethread_workqueue(dev_name(&client->dev));
    if (!gslx680_ts->ts_workqueue) {
        err = -ESRCH;
        goto exit_create_singlethread;
    }

    err = request_threaded_irq(this_client->irq, NULL, gslx680_ts_interrupt, IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND|IRQF_ONESHOT, client->name, gslx680_ts);
    if (err < 0) {
        dev_err(&client->dev, "gslx680_probe: request irq failed\n");
        goto exit_irq_request_failed;
    }

    tp_irq_disable();
    input_dev = input_allocate_device();
    if (!input_dev) {
        err = -ENOMEM;
        dev_err(&client->dev, "failed to allocate input device\n");
        goto exit_input_dev_alloc_failed;
    }

    gslx680_ts_virtual_keys_init();

    gslx680_ts->input_dev = input_dev;
    set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
    set_bit(ABS_MT_POSITION_X, input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
#ifdef VIRTUAL_VEBRATOR_FUC
    set_bit(BTN_TOUCH, input_dev->keybit);
    __set_bit(KEY_MENU,  input_dev->keybit);
    __set_bit(KEY_BACK,  input_dev->keybit);
    __set_bit(KEY_HOMEPAGE,  input_dev->keybit);
#endif
    input_set_abs_params(input_dev,
                ABS_MT_TRACKING_ID, 0, 255, 0, 0);
    input_set_abs_params(input_dev,
                 ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
    input_set_abs_params(input_dev,
                 ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
    input_set_abs_params(input_dev,
                 ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
    input_set_abs_params(input_dev,
                 ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

    set_bit(EV_ABS, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#ifndef VIRTUAL_VEBRATOR_FUC
    input_mt_init_slots(input_dev, 11,0);

#ifdef HAVE_TOUCH_KEY
  {
    int i;
    for(i = 0; i < MAX_KEY_NUM; i++)
    {
        input_set_capability(input_dev, EV_KEY, key_array[i]);
    }
  }
#endif
#endif
#ifdef GSL_GESTURE
    __set_bit(REPORT_KEY_VALUE, input_dev->keybit);
    __set_bit(KEY_F12,  input_dev->keybit);
    __set_bit(KEY_F11,  input_dev->keybit);
    __set_bit(KEY_F10,  input_dev->keybit);
    __set_bit(KEY_F9,  input_dev->keybit);
    __set_bit(KEY_U,  input_dev->keybit);
    __set_bit(KEY_O,  input_dev->keybit);
    __set_bit(KEY_W,  input_dev->keybit);
    __set_bit(KEY_M,  input_dev->keybit);
    __set_bit(KEY_E,  input_dev->keybit);
    __set_bit(KEY_C,  input_dev->keybit);
    __set_bit(KEY_S,  input_dev->keybit);
    __set_bit(KEY_V,  input_dev->keybit);
    __set_bit(KEY_Z,  input_dev->keybit);
    input_set_capability(input_dev, EV_KEY, KEY_SCROLLDOWN);
#endif
    input_dev->name = INPUT_DEV_NAME;       //dev_name(&client->dev)
    strcpy(sprd_tp_name,GSLX680_TS_NAME); //zxw add
    err = input_register_device(input_dev);
    if (err) {
        dev_err(&client->dev,
        "gslx680_ts_probe: failed to register input device: %s\n",
        dev_name(&client->dev));
        goto exit_input_register_device_failed;
    }

#ifdef GSL_GESTURE
    gsl_FunIICRead(gsl_read_oneframe_data);
    //gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern)/sizeof(unsigned int)/18);
	gsl_wake_lock = wakeup_source_create("gsl_wake_lock");
	wakeup_source_add(gsl_wake_lock);
#endif

#ifdef CONFIG_TOUCHSCREEN_WITH_PROXIMITY
    set_bit(EV_ABS, input_dev->evbit);
    input_set_capability(input_dev, EV_ABS, ABS_DISTANCE);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);
    err = sysfs_create_group(&gslx680_ts->input_dev->dev.kobj, &proximity_attr_group);
    if (err) {
        dev_err(&client->dev, "create device file failed!\n");
          printk("create device file failed!\n");
        err = -EINVAL;
        goto exit_input_register_device_failed;
    }
      printk("create device file  sucess!\n");
    atomic_set(&prox_contrl_state,0);
    wakeup_wakelock = wakeup_source_create("poll-wake-lock");
    wakeup_source_add(wakeup_wakelock);
#endif
    init_chip(this_client);
    check_mem_data(this_client);

    GSL168X_DBG("==register_early_suspend =");
#ifdef REVO_PM_TP_SYSFS
    err = glb_gslx680_tp_sysfs_init(client);
    if (err) {
        GSL168X_ERR( "could not create sysfs device attributes , < ts_suspend >");
    }
#elif defined(REVO_PM_VIDEO_ADF_NOTIFIER)
    adf_event_block.notifier_call = ts_adf_event_handler;
    /* make sure we're the first to suspend/resume */
    adf_event_block.priority = 1000;
    err = adf_register_client(&adf_event_block);
    if (err < 0)
        GSL168X_DBG("register adf notifier fail, cannot sleep when screen off");
    else
        GSL168X_DBG("register adf notifier succeed");
#endif

    tp_irq_enable();

#ifdef TPD_PROC_DEBUG
    proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
    gsl_proc_flag = 0;
#endif

#ifdef GSL_TIMER
    INIT_DELAYED_WORK(&gsl_monitor_work, gsl_monitor_worker);
    gsl_monitor_workqueue = create_singlethread_workqueue("gsl_monitor_workqueue");
    queue_delayed_work(gsl_monitor_workqueue, &gsl_monitor_work, 1500);
#endif

#ifdef GSL_GESTURE
    #if 1
    gsl_gesture_init();
    #else
    ret= sysfs_create_group(&client->dev.kobj, &gslx680_gesture_group);
    if (ret < 0) {
        printk("--zxw--sysfs_create_group fail--\n");
        return -ENOMEM;
    }
    #endif
#endif

    msleep(30);
    if(gsl_ts_read(client, 0xbc, read_buf, sizeof(read_buf)) <0 ){
        printk("%s,zxw i2c  err ::%d ----------------------------->\r\n",__func__,err);
    }
    printk("---zxw--->%s: ==probe over =\n",__func__);


#if defined(SENSORHUB_PROXIMITY_TOUCHSCREEN)
    sensorhub_prox_do = tp_hub_sensorhub_do_funcList(TYPE_SENSORHUB_SETTING, &sensorhub_info);
    if(sensorhub_prox_do&& sensorhub_prox_do->sensorhub_init)
        sensorhub_prox_do->sensorhub_init();
#endif

    gsl_read_version(this_client);
    
    return 0;

exit_input_register_device_failed:
    input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_irq_request_failed:
    cancel_work_sync(&gslx680_ts->pen_event_work);
    destroy_workqueue(gslx680_ts->ts_workqueue);
    free_irq(client->irq, gslx680_ts);
exit_create_singlethread:
    GSL168X_DBG("==singlethread error =\n");
    i2c_set_clientdata(client, NULL);
exit_check_functionality_failed:
    gpio_free(gslx680_ts->reset_pin);
    gpio_free(gslx680_ts->int_pin);
    kfree(gslx680_ts);
exit_alloc_data_failed:
exit_alloc_platform_data_failed:
    return err;
}

static int gslx680_ts_remove(struct i2c_client *client)
{

    struct gslx680_ts_data *gslx680_ts = i2c_get_clientdata(client);

    GSL168X_DBG("==gslx680_ts_remove=\n");
    glb_gslx680_tp_sysfs_remove(client);

#ifdef GSL_GESTURE
    device_remove_file(Gesture_cmd_dev, &dev_attr_gesture);
#endif
#ifdef GSL_TIMER
    cancel_delayed_work_sync(&gsl_monitor_work);
    destroy_workqueue(gsl_monitor_workqueue);
#endif

#if defined(REVO_PM_VIDEO_ADF_NOTIFIER)
    adf_unregister_client(&adf_event_block);
#endif
    free_irq(client->irq, gslx680_ts);
    input_unregister_device(gslx680_ts->input_dev);

    cancel_work_sync(&gslx680_ts->pen_event_work);
    destroy_workqueue(gslx680_ts->ts_workqueue);
    i2c_set_clientdata(client, NULL);
    if (gpio_is_valid(gslx680_ts->reset_pin))
        gpio_free(gslx680_ts->reset_pin);
    if (gpio_is_valid(gslx680_ts->int_pin))
        gpio_free(gslx680_ts->int_pin);

    kfree(gslx680_ts);
    return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops gslx680_ts_dev_pmops = {
    .suspend = gslx680_ts_suspend,
    .resume = gslx680_ts_resume,
};
#endif

static const struct i2c_device_id gslx680_ts_id[] = {
    { GSLX680_TS_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, gslx680_ts_id);


static const struct of_device_id gslx680_of_match[] = {
    { .compatible = "gslx680,gslx680_ts", },
    { }
};

MODULE_DEVICE_TABLE(of, gslx680_of_match);

static struct i2c_driver gslx680_ts_driver = {
    .probe      = gslx680_ts_probe,
    .remove     = gslx680_ts_remove,
    .id_table   = gslx680_ts_id,
    .driver = {
        .name   = GSLX680_TS_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = gslx680_of_match,
    },
};


static void sprd_del_i2c_device(struct i2c_client *client, struct i2c_driver *driver)
{
    printk("%s : slave_address=0x%x; i2c_name=%s\n",__func__, client->addr, client->name);
    i2c_unregister_device(client);
    i2c_del_driver(driver);
}


static int __init gslx680_ts_init(void)
{

    int32_t ret = 0;

    #ifdef GSLX680_CONFIG_OF
    //---add i2c driver---
    ret = i2c_add_driver(&gslx680_ts_driver);
    if (ret) {
        printk("failed to add i2c driver",__func__);
    }
    return ret;
    #else
    GSL168X_ERR(" NOT_DTS Enter....");
    if( revo_dts_gpio_is_ready() != 0)
    {
        GSL168X_ERR("revo_dts_gpio_is_ready fail");
        return -ENODEV;
    }
    s_set_sprd_3rdparty_gpios();
    ret = revo_ts_init_gpio("gslx680");
    if ( ret != 0 ){
            GSL168X_ERR(" rst_pin and irq_ping Used by other touchScreen, please confirm the actual chip , [ Compatibility problems ]");
            return -ENODEV;
    }
    return revo_ts_add_i2c_device_and_driver(GSLX680_TS_ADDR,&gslx680_ts_driver);
    #endif
}

static void __exit gslx680_ts_exit(void)
{
    printk("%s\n", __func__);
    sprd_del_i2c_device(this_client, &gslx680_ts_driver);
}

module_init(gslx680_ts_init);
module_exit(gslx680_ts_exit);

MODULE_AUTHOR("zxw");
MODULE_DESCRIPTION("GSLX680 TouchScreen Driver");
MODULE_LICENSE("GPL");



