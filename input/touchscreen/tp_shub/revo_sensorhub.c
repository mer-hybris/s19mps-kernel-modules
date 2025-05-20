#define  SENSORHUB_C_FILE
#include "revo_sensorhub.h"

static struct wakeup_source *proximity_wake_lock;
static struct spinlock s_spinlock ;

// static int shub_report_proximity_flush_event(u32 value);
// static int shub_report_proximity_event(u32 value);
static void  sensorhub_proximity_wake_lock(int tp_face_switch_on);

static const struct sensorhub_setting * ts_sensorhub_setting  = NULL ;

#define  tp_sensorhub_info(fmt,arg...)   \
            printk("<Revo_ts>  <<%s>> sensorhub-info: " fmt  "\n" , \
            ts_sensorhub_setting->ic_name , ##arg);


static ssize_t show_proximity_sensor( struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    int nearfar = -1 , enable = -1 ;
    if(ts_sensorhub_setting->get_prox_active_stat)
        nearfar = ts_sensorhub_setting->get_prox_active_stat();
    if(ts_sensorhub_setting->get_prox_enble_stat)
        enable = ts_sensorhub_setting->get_prox_enble_stat();
    return sprintf(buf, "tp proximity enable = %d nearfar = %d \n", enable , nearfar );
}


static ssize_t store_proximity_sensor( struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int ps_enable = 0;
    sscanf(buf, "%d\n",&ps_enable);
    printk("%s sys control tp face mode! ps_enable = %d\n", __func__, ps_enable);//debug
    if(ts_sensorhub_setting->use_wakelock == true)
        sensorhub_proximity_wake_lock(ps_enable);
    if(ts_sensorhub_setting->sensorhub_prox_ctrl)
        ts_sensorhub_setting->sensorhub_prox_ctrl(ps_enable);
    return count;
}

static DEVICE_ATTR(psensor_enable, 0644, show_proximity_sensor, store_proximity_sensor);


static ssize_t proximity_sensor_flush_show( struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    printk("buf==%d\n", (int)*buf);
    return sprintf(buf, "tp psensor show\n");
}

static ssize_t proximity_sensor_fulsh_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t count)
{
    //int ps_enable = 0;
    //sscanf(buf, "%d\n",&ps_enable);
    // shub_report_proximity_flush_event(0);
    static int flag = 1;

    flag = !flag;
    if(ts_sensorhub_setting->input) {
        if(flag) {
            input_report_abs(ts_sensorhub_setting->input, ABS_DISTANCE, 3);
        } else {
            input_report_abs(ts_sensorhub_setting->input, ABS_DISTANCE, 4);
        }
        input_sync(ts_sensorhub_setting->input);
    }
    return count;
}

static DEVICE_ATTR(psensor_flush, 0644, proximity_sensor_flush_show, proximity_sensor_fulsh_store);

static ssize_t proximity_sensor_lable_show( struct device *dev,
                    struct device_attribute *attr,
                    char *buf)
{
    return sprintf(buf, "tp name show :%s\n" , 
        ts_sensorhub_setting ? ts_sensorhub_setting->ic_name : "unknown" );
}


static DEVICE_ATTR(psensor_lable, 0444, proximity_sensor_lable_show, NULL);

// static int shub_report_proximity_event(u32 value)
// {
//     s64 k_timestamp;
//     struct shub_event_params event;

//     k_timestamp = ktime_to_us(ktime_get_boottime());

//     event.Cmd = 129;
//     event.HandleID = 58;
//     event.fdata[0] = value;
//     event.fdata[1] = 0;
//     event.fdata[2] = 0;
//     event.Length = sizeof(struct shub_event_params);
//     event.timestamp = k_timestamp;


//     // tp_sensorhub_info("report distance = 0x%x\n",value);

//     peri_send_sensor_event_to_iio((u8 *)&event, sizeof(struct shub_event_params));

//     return 0;
// }

// static int shub_report_proximity_flush_event(u32 value)
// {
//     s64 k_timestamp;
//     struct shub_event_params event;

//     k_timestamp = ktime_to_us(ktime_get_boottime());

//     event.Cmd = 130;//flush
//     event.HandleID = 58;
//     event.fdata[0] = value;
//     event.fdata[1] = 0;
//     event.fdata[2] = 0;
//     event.Length = sizeof(struct shub_event_params);
//     event.timestamp = k_timestamp;

//     tp_sensorhub_info("report flush = %d\n",value);

//     peri_send_sensor_event_to_iio((u8 *)&event,sizeof(struct shub_event_params));
//     return 0;
// }


static void sensorhub_tp_init(void)
{
    struct class  *psensor_class;
    struct device *psensor_dev;

    psensor_class = class_create(THIS_MODULE,"sprd_sensorhub_tp");

    if(IS_ERR(psensor_class))
        printk("Failed to create class!\n");

    psensor_dev = device_create(psensor_class, NULL, 0, NULL, "device");

    if(IS_ERR(psensor_dev))
        printk("Failed to create device!\n");

    if(device_create_file(psensor_dev, &dev_attr_psensor_enable) < 0) {
        /* path: sys/class/sprd_sensorhub_tp/device/psensor_enable */
        printk("Failed to create device file(%s)!\n", dev_attr_psensor_enable.attr.name);
    }

    if(device_create_file(psensor_dev, &dev_attr_psensor_flush) < 0) {
        /* path : /sys/class/sprd_sensorhub_tp/device/psensor_flush */
        printk("Failed to create device file(%s)!\n", dev_attr_psensor_flush.attr.name);
    }

    if(device_create_file(psensor_dev, &dev_attr_psensor_lable) < 0) {
        /* path : /sys/class/sprd_sensorhub_tp/device/lable */
        printk("Failed to create device file(%s)!\n", dev_attr_psensor_lable.attr.name);
    }

    if(ts_sensorhub_setting->input) {
        __set_bit(INPUT_PROP_DIRECT, ts_sensorhub_setting->input->propbit);
    }

	proximity_wake_lock = wakeup_source_create("prox_delayed_work");
	wakeup_source_add(proximity_wake_lock);

    spin_lock_init(&s_spinlock);
    return ;
}


static void  sensorhub_proximity_wake_lock(int tp_face_switch_on)
{
    static bool   prox_is_lock  = false ;
    if(tp_face_switch_on){
        /* TP_face_mode_switch on */
        if( prox_is_lock == false ){
            __pm_stay_awake(proximity_wake_lock);
            spin_lock(&s_spinlock);
            prox_is_lock = true ;
            spin_unlock(&s_spinlock);
        }
    }else{
        /* TP_face_mode_switch off */
        if( prox_is_lock == true ){
            __pm_relax(proximity_wake_lock);
            spin_lock(&s_spinlock);
            prox_is_lock = false ;
            spin_unlock(&s_spinlock);
        }

    }
}


static void sensorhub_prox_reprot(struct input_dev *dev, enum SENSORHUB_FACE_STATUS status )
{
     switch(status)
     {
         case SENSORHUB_FACE_STATUS_NEAR :
			input_report_abs(dev, ABS_DISTANCE, 0);
             break ;
         case SENSORHUB_FACE_STATUS_FAR:
			input_report_abs(dev, ABS_DISTANCE, 5);
             break ;
         default :
             break;
     }

    input_sync(dev);
}

static struct do_sensorhub * sensorhub_do_funcList(enum SENHUB_PROX_DATA_T data_type , void *data )
{
    static /*const*/ struct do_sensorhub do_fun = {
        .prox_reprot_func = sensorhub_prox_reprot ,
        .sensorhub_init = sensorhub_tp_init
    }   ;

    if( data_type == TYPE_SENSORHUB_SETTING && data)
        ts_sensorhub_setting = (struct sensorhub_setting * ) data ;

    if( ts_sensorhub_setting == NULL )
    {
        return NULL ;
    }
    return (struct do_sensorhub *) &do_fun ;
}


#define  GLOBAL_SENSORHUB_DO_FUNC_LIST(glb_name,use_func) \
struct do_sensorhub *glb_name##_##use_func(enum SENHUB_PROX_DATA_T data_type ,void* data)\
{\
    return use_func(data_type,data); \
}

GLOBAL_SENSORHUB_DO_FUNC_LIST(tp_hub, sensorhub_do_funcList)




