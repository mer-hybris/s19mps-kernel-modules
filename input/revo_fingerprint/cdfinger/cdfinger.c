#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#if  1// revo
#include <linux/wait.h>
#include <linux/freezer.h>
#else
//#include <linux/wakelock.h>
#endif
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/semaphore.h>
#include <linux/poll.h>
#include <linux/fcntl.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/signal.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
typedef struct key_report {
    int key;
    int value;
}key_report_t;


#define DEVICE_NAME "fpsdev0"
#define CDFINGER_IOCTL_MAGIC_NO          0xFB
#define CDFINGER_INIT                    _IOW(CDFINGER_IOCTL_MAGIC_NO, 0, uint8_t)
#define CDFINGER_GETIMAGE                _IOW(CDFINGER_IOCTL_MAGIC_NO, 1, uint8_t)
#define CDFINGER_INITERRUPT_MODE         _IOW(CDFINGER_IOCTL_MAGIC_NO, 2, uint8_t)
#define CDFINGER_INITERRUPT_KEYMODE      _IOW(CDFINGER_IOCTL_MAGIC_NO, 3, uint8_t)
#define CDFINGER_INITERRUPT_FINGERUPMODE _IOW(CDFINGER_IOCTL_MAGIC_NO, 4, uint8_t)
#define CDFINGER_RELEASE_WAKELOCK        _IO(CDFINGER_IOCTL_MAGIC_NO, 5)
#define CDFINGER_CHECK_INTERRUPT         _IO(CDFINGER_IOCTL_MAGIC_NO, 6)
#define CDFINGER_SET_SPI_SPEED           _IOW(CDFINGER_IOCTL_MAGIC_NO, 7, uint32_t)
#define CDFINGER_POWERDOWN               _IO(CDFINGER_IOCTL_MAGIC_NO, 11)
#define CDFINGER_GETID                   _IO(CDFINGER_IOCTL_MAGIC_NO,12)

#define CDFINGER_HW_RESET               _IOW(CDFINGER_IOCTL_MAGIC_NO, 14, uint8_t)
#define CDFINGER_GET_STATUS               _IO(CDFINGER_IOCTL_MAGIC_NO, 15)
#define CDFINGER_NEW_KEYMODE        _IOW(CDFINGER_IOCTL_MAGIC_NO, 37, uint8_t)

#define CDFINGER_REPORT_KEY       _IOW(CDFINGER_IOCTL_MAGIC_NO,19,key_report_t)
#define CDFINGER_INIT_GPIO       _IO(CDFINGER_IOCTL_MAGIC_NO,20)
#define CDFINGER_INIT_IRQ        _IO(CDFINGER_IOCTL_MAGIC_NO,21)
#define CDFINGER_POWER_ON        _IO(CDFINGER_IOCTL_MAGIC_NO,22)
#define CDFINGER_RESET           _IO(CDFINGER_IOCTL_MAGIC_NO,23)
#define CDFINGER_RELEASE_DEVICE  _IO(CDFINGER_IOCTL_MAGIC_NO,25)
#define CDFINGER_WAKE_LOCK   _IOW(CDFINGER_IOCTL_MAGIC_NO,26,uint8_t)
#define CDFINGER_POLL_TRIGGER            _IO(CDFINGER_IOCTL_MAGIC_NO,31)
#define CDFINGER_CONTROL_IRQ _IOW(CDFINGER_IOCTL_MAGIC_NO, 38, uint8_t)

static int isInKeyMode = 0; // key mode
static int screen_status = 1; // screen on
static int sign_sync = 0; // for poll
static DECLARE_WAIT_QUEUE_HEAD(cdfinger_waitqueue);

static u8 cdfinger_debug = 0x01;
#define CDFINGER_DBG(fmt, args...) \
    do{ \
        if(cdfinger_debug & 0x01) \
        printk( "[DBG][cdfinger]:%5d: <%s>" fmt, __LINE__,__func__,##args ); \
    }while(0)
#define CDFINGER_ERR(fmt, args...) \
    do{ \
        printk( "[DBG][cdfinger]:%5d: <%s>" fmt, __LINE__,__func__,##args ); \
    }while(0)
#define CDFINGER_PROBE(fmt, args...) \
    do{ \
        printk( "[PROBE][cdfinger]:%5d: <%s>" fmt, __LINE__,__func__,##args ); \
    }while(0)


/*
extern void wakeup_source_prepare(struct wakeup_source *ws, const char *name);
extern struct wakeup_source *wakeup_source_create(const char *name);
extern void wakeup_source_drop(struct wakeup_source *ws);
extern void wakeup_source_destroy(struct wakeup_source *ws);
extern void wakeup_source_add(struct wakeup_source *ws);
extern void wakeup_source_remove(struct wakeup_source *ws);
extern struct wakeup_source *wakeup_source_register(const char *name);
extern void wakeup_source_unregister(struct wakeup_source *ws);
extern int device_wakeup_enable(struct device *dev);
extern int device_wakeup_disable(struct device *dev);
extern void device_set_wakeup_capable(struct device *dev, bool capable);
extern int device_init_wakeup(struct device *dev, bool val);
extern int device_set_wakeup_enable(struct device *dev, bool enable);
extern void __pm_stay_awake(struct wakeup_source *ws);
extern void pm_stay_awake(struct device *dev);
extern void __pm_relax(struct wakeup_source *ws);
extern void pm_relax(struct device *dev);
extern void pm_wakeup_ws_event(struct wakeup_source *ws, unsigned int msec, bool hard);
extern void pm_wakeup_dev_event(struct device *dev, unsigned int msec, bool hard);
*/

struct cdfingerfp_data {
    struct platform_device *cdfinger_dev;
    struct miscdevice *miscdev;
    u32 irq_num;
    u32 reset_num;
    int irq_enabled;
    struct fasync_struct *async_queue;
    struct wakeup_source *cdfinger_lock;
    struct wakeup_source *cdfinger_lock_permanent;
    struct input_dev* cdfinger_input;
    struct notifier_block notifier;
    struct regulator *vdd;
    struct mutex buf_lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
}*g_cdfingerfp_data;

static void cdfinger_disable_irq(struct cdfingerfp_data *cdfinger)
{
    if (cdfinger->irq_enabled == 1)
    {
        disable_irq_nosync(gpio_to_irq(cdfinger->irq_num));
        cdfinger->irq_enabled = 0;
        CDFINGER_DBG("irq disable\n");
    }
}

static void cdfinger_enable_irq(struct cdfingerfp_data *cdfinger)
{
    if (cdfinger->irq_enabled == 0)
    {
        enable_irq(gpio_to_irq(cdfinger->irq_num));
        cdfinger->irq_enabled = 1;
        CDFINGER_DBG("irq enable\n");
    }
}

static int cdfinger_init_gpio(struct cdfingerfp_data *cdfinger)
{
    int err = 0;
    if (gpio_is_valid(cdfinger->reset_num)) {
        err = gpio_request(cdfinger->reset_num, "cdfinger-reset");
        if (err) {
            CDFINGER_DBG("Could not request reset gpio.\n");
            return err;
        }
    }
    else {
        CDFINGER_DBG("not valid reset gpio\n");
        return -EIO;
    }

    if (gpio_is_valid(cdfinger->irq_num)) {
        err = gpio_request(cdfinger->irq_num,"cdfinger-irq gpio");
        if (err) {
            CDFINGER_DBG("Could not request irq gpio.\n");
            gpio_free(cdfinger->reset_num);
            return err;
        }
    }
    else {
        CDFINGER_DBG(KERN_ERR "not valid irq gpio\n");
        gpio_free(cdfinger->reset_num);
        return -EIO;
    }
    gpio_direction_input(cdfinger->irq_num);
    CDFINGER_DBG("cdfinger_init_gpio reset_num =%d irq_num=%d \n ",cdfinger->reset_num,cdfinger->irq_num);
    CDFINGER_DBG("%s(..) ok! exit.\n", __FUNCTION__);
    return err;
}

static int cdfinger_free_gpio(struct cdfingerfp_data *cdfinger)
{
    int err = 0;
    CDFINGER_DBG("%s(..) enter.\n", __FUNCTION__);

    if (gpio_is_valid(cdfinger->irq_num)) {
        gpio_free(cdfinger->irq_num);
        free_irq(gpio_to_irq(cdfinger->irq_num), (void*)cdfinger);
    }

    if (gpio_is_valid(cdfinger->reset_num)) {
        gpio_free(cdfinger->reset_num);
    }


    CDFINGER_DBG("%s(..) ok! exit.\n", __FUNCTION__);

    return err;
}

static int cdfinger_parse_dts(struct device *dev,struct cdfingerfp_data *cdfinger)
{
    int err = 0;
    cdfinger->reset_num = of_get_named_gpio(dev->of_node,"fpreset-gpios",0);
    if(cdfinger->reset_num < 0)
    {
        CDFINGER_ERR("fail to get reset_num %d\n",cdfinger->reset_num);
        return cdfinger->reset_num;
    }
    cdfinger->irq_num = of_get_named_gpio(dev->of_node,"fpint-gpios",0);
    if(cdfinger->irq_num < 0)
    {
        CDFINGER_ERR("fail to get irq_num %d\n",cdfinger->irq_num);
        return cdfinger->irq_num;
    }
    CDFINGER_PROBE("cdfinger_parse_dts reset_num=%d  irq_num =%d ",cdfinger->reset_num,cdfinger->irq_num);
    //cdfinger->reset_num = 131;
    //cdfinger->irq_num = 130;
    return err;
}

static int cdfinger_power_on(struct cdfingerfp_data *pdata)
{

   /* pdata->vdd = regulator_get(NULL,"vddsim2");
    regulator_set_voltage(pdata->vdd, 3300000, 3300000);
    regulator_enable(pdata->vdd);
    */
    return 0;

}

static int cdfinger_open(struct inode *inode,struct file *file)
{
    file->private_data = g_cdfingerfp_data;
    return 0;
}

static int cdfinger_async_fasync(int fd,struct file *file,int mode)
{
    struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;
    return fasync_helper(fd,file,mode,&cdfingerfp->async_queue);
}

static int cdfinger_release(struct inode *inode,struct file *file)
{
    struct cdfingerfp_data *cdfingerfp = file->private_data;
    if(NULL == cdfingerfp)
    {
        return -EIO;
    }
    file->private_data = NULL;
    return 0;
}

static void cdfinger_async_report(void)
{
    struct cdfingerfp_data *cdfingerfp = g_cdfingerfp_data;

    sign_sync = 1;
    wake_up_interruptible(&cdfinger_waitqueue);
    __pm_wakeup_event(cdfingerfp->cdfinger_lock, msecs_to_jiffies(1000));
    //kill_fasync(&cdfingerfp->async_queue,SIGIO,POLL_IN);
}

static int cdfinger_get_gpio_value(unsigned gpio)
{
    int val = -1;

    if(gpio_is_valid(gpio))
    {
        val = gpio_get_value(gpio);
        CDFINGER_DBG("gpio %d, value %d\n", gpio, val);
    }
    return val;
}

static irqreturn_t cdfinger_eint_handler(int irq, void *dev_id)
{
    struct cdfingerfp_data *cdfinger = g_cdfingerfp_data;

    if (1 == cdfinger->irq_enabled)
    {
        if (cdfinger_get_gpio_value(cdfinger->irq_num) == 1)
            cdfinger_async_report();
    }
    return IRQ_HANDLED;
}

static void cdfinger_reset_gpio_init(struct cdfingerfp_data *pdata, int ms)
{
    gpio_direction_output(pdata->reset_num, 1);
    mdelay(ms);
    gpio_set_value(pdata->reset_num, 0);
    mdelay(ms);
    gpio_set_value(pdata->reset_num, 1);
    mdelay(ms);
}

static int cdfinger_eint_gpio_init(struct cdfingerfp_data *pdata)
{
    int error = 0;
    int ret;
    ret = gpio_to_irq(pdata->irq_num);
    CDFINGER_DBG("gpio_to_irq =%d \n",ret);
    error =request_irq(gpio_to_irq(pdata->irq_num),cdfinger_eint_handler,IRQF_TRIGGER_RISING | IRQF_ONESHOT,"cdfinger_eint", NULL);
    if (error < 0)
    {
        CDFINGER_ERR("request irq err %d", error);
        return error;
    }
    enable_irq_wake(gpio_to_irq(pdata->irq_num));
    pdata->irq_enabled = 1;
    return error;
}

int cdfinger_report_key(struct cdfingerfp_data *cdfinger, unsigned long arg)
{
    key_report_t report;
    if (copy_from_user(&report, (key_report_t *)arg, sizeof(key_report_t)))
    {
        CDFINGER_ERR("%s err\n", __func__);
        return -1;
    }
    CDFINGER_DBG("cdfinger key=%d\r\n,value = %d\r\n",report.key,report.value);
    input_report_key(cdfinger->cdfinger_input, report.key, !!report.value);
    input_sync(cdfinger->cdfinger_input);

    return 0;
}

static void cdfinger_wake_lock(struct cdfingerfp_data *pdata,uint8_t arg)
{

    CDFINGER_DBG("cdfinger_wake_lock enter [%d]----+++\n", arg);
    if(arg)
    {
        __pm_stay_awake(pdata->cdfinger_lock);

    }
    else
    {
        __pm_relax(pdata->cdfinger_lock);
    }

}

static unsigned int cdfinger_poll(struct file *filp, struct poll_table_struct *wait)
{
    int mask = 0;
    poll_wait(filp, &cdfinger_waitqueue, wait);
    if (sign_sync == 1)
    {
        mask |= POLLIN|POLLPRI;
    } else if (sign_sync == 2)
    {
        mask |= POLLOUT;
    }
    sign_sync = 0;
    CDFINGER_DBG("mask %u\n",mask);
    return mask;
}

static long cdfinger_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    int err = 0;
    struct cdfingerfp_data *cdfinger = filp->private_data;
    mutex_lock(&cdfinger->buf_lock);
    switch (cmd) {
        case CDFINGER_INIT_GPIO:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_INIT_GPIO start\n");
            err = cdfinger_init_gpio(cdfinger);
            break;
        case CDFINGER_INIT_IRQ:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_INIT_IRQ start\n");
            err = cdfinger_eint_gpio_init(cdfinger);
            break;
        case CDFINGER_RELEASE_DEVICE:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_RELEASE_DEVICE start\n");
            cdfinger_free_gpio(cdfinger);
            if (cdfinger->cdfinger_input != NULL) {
            }
            misc_deregister(cdfinger->miscdev);
            break;
        case CDFINGER_POWER_ON:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_POWER_ON start\n");
            cdfinger_power_on(cdfinger);
            break;
        case CDFINGER_RESET:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_RESET start\n");
            cdfinger_reset_gpio_init(cdfinger,10);
            break;
        case CDFINGER_REPORT_KEY:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_REPORT_KEY start\n");
            err = cdfinger_report_key(cdfinger,arg);
            break;
        case CDFINGER_NEW_KEYMODE:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_NEW_KEYMODE start\n");
            isInKeyMode = 0;
            break;
        case CDFINGER_INITERRUPT_MODE:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_INITERRUPT_MODE start\n");
            isInKeyMode = 1;  // not key mode
            break;
        case CDFINGER_HW_RESET:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_HW_RESET start\n");
            cdfinger_reset_gpio_init(cdfinger,arg);
            break;
        case CDFINGER_GET_STATUS:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_GET_STATUS start\n");
            err = screen_status;
            break;
        case CDFINGER_WAKE_LOCK:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_WAKE_LOCK start\n");
            cdfinger_wake_lock(cdfinger,arg);
            break;
        case CDFINGER_POLL_TRIGGER:
            CDFINGER_DBG("cdfinger_ioctl  CDFINGER_POLL_TRIGGER start\n");
            sign_sync = 2;
            wake_up_interruptible(&cdfinger_waitqueue);
            err = 0;
            break;
        case CDFINGER_CONTROL_IRQ:
            if (1 == arg)
                cdfinger_enable_irq(cdfinger);
            else
                cdfinger_disable_irq(cdfinger);
            break;
        default:
            break;
    }
    mutex_unlock(&cdfinger->buf_lock);
    return err;
}

static const struct file_operations cdfinger_fops = {
    .owner   = THIS_MODULE,
    .open    = cdfinger_open,
    .unlocked_ioctl = cdfinger_ioctl,
    .release = cdfinger_release,
    .poll = cdfinger_poll,
    .fasync  = cdfinger_async_fasync,
};

static struct miscdevice st_cdfinger_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DEVICE_NAME,
    .fops = &cdfinger_fops,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cdfinger_early_suspend(struct early_suspend *h)
{
    CDFINGER_DBG("early suspend %d\n", isInKeyMode);
    mutex_lock(&g_cdfingerfp_data->buf_lock);
    screen_status = 0;
    if (isInKeyMode == 0)
        cdfinger_async_report();
    mutex_unlock(&g_cdfingerfp_data->buf_lock);
}

static void cdfinger_late_resume(struct early_suspend *h)
{
    int ret = 0;
    CDFINGER_ERR(" late resume %d\n", isInKeyMode);
    mutex_lock(&g_cdfingerfp_data->buf_lock);
    screen_status = 1;
    if (isInKeyMode == 0)
        cdfinger_async_report();
    mutex_unlock(&g_cdfingerfp_data->buf_lock);
}
#endif

static int cdfinger_fb_notifier_callback(struct notifier_block* self,
                                        unsigned long event, void* data)
{
    struct fb_event* evdata = data;
    unsigned int blank;
    int retval = 0;

    if (event != FB_EVENT_BLANK /* FB_EARLY_EVENT_BLANK */) {
        return 0;
    }
    blank = *(int*)evdata->data;
    switch (blank) {
        case FB_BLANK_UNBLANK:
        mutex_lock(&g_cdfingerfp_data->buf_lock);
        screen_status = 1;
        if (isInKeyMode == 0)
            cdfinger_async_report();
        mutex_unlock(&g_cdfingerfp_data->buf_lock);
        CDFINGER_DBG("sunlin==FB_BLANK_UNBLANK==\n");
            break;
        case FB_BLANK_POWERDOWN:
        mutex_lock(&g_cdfingerfp_data->buf_lock);
        screen_status = 0;
        if (isInKeyMode == 0)
            cdfinger_async_report();
        mutex_unlock(&g_cdfingerfp_data->buf_lock);
        CDFINGER_DBG("sunlin==FB_BLANK_POWERDOWN==\n");
            break;
        default:
            break;
    }

    return retval;
}

static int cdfinger_probe(struct platform_device *pdev)
{
    struct cdfingerfp_data *cdfingerdev= NULL;
    int status = -ENODEV;
    status = misc_register(&st_cdfinger_dev);

    printk("revo_fingprint  enter %s \n",__func__);

    CDFINGER_PROBE("enter.......\n");

    if (status) {
        CDFINGER_DBG("cdfinger misc register err%d\n",status);
        return -1;
    }

    cdfingerdev = kzalloc(sizeof(struct cdfingerfp_data),GFP_KERNEL);
    cdfingerdev->miscdev = &st_cdfinger_dev;
    cdfingerdev->cdfinger_dev = pdev;
    mutex_init(&cdfingerdev->buf_lock);

	cdfingerdev->cdfinger_lock = wakeup_source_register(NULL, "cdfinger wakelock");
	cdfingerdev->cdfinger_lock_permanent = wakeup_source_register(NULL, "cdfinger wakelock permanent");
	if (!cdfingerdev->cdfinger_lock || !cdfingerdev->cdfinger_lock_permanent) {
		goto unregister_lock;
	}

    status=cdfinger_parse_dts(&cdfingerdev->cdfinger_dev->dev, cdfingerdev);
    if (status != 0) {
        CDFINGER_DBG("cdfinger parse err %d\n",status);
        goto unregister_dev;
    }

    cdfingerdev->cdfinger_input = input_allocate_device();
    if(!cdfingerdev->cdfinger_input){
        CDFINGER_ERR("crate cdfinger_input faile!\n");
        goto unregister_dev;
    }
    __set_bit(EV_KEY,cdfingerdev->cdfinger_input->evbit);
    __set_bit(KEY_F11,cdfingerdev->cdfinger_input->keybit);
    cdfingerdev->cdfinger_input->id.bustype = BUS_HOST;
    cdfingerdev->cdfinger_input->name="cdfinger_input";
    if(input_register_device(cdfingerdev->cdfinger_input))
    {
      input_free_device(cdfingerdev->cdfinger_input);
        goto unregister_dev;
    }

    cdfingerdev->notifier.notifier_call = cdfinger_fb_notifier_callback;
    fb_register_client(&cdfingerdev->notifier);
#ifdef CONFIG_HAS_EARLYSUSPEND
    cdfingerdev->early_suspend.suspend = cdfinger_early_suspend;
    cdfingerdev->early_suspend.resume = cdfinger_late_resume;
    register_early_suspend(&cdfingerdev->early_suspend);
#endif
    g_cdfingerfp_data = cdfingerdev;
    CDFINGER_PROBE("exit\n");
    return 0;
unregister_dev:
	wakeup_source_unregister(cdfingerdev->cdfinger_lock);
	wakeup_source_unregister(cdfingerdev->cdfinger_lock_permanent);
unregister_lock:
    misc_deregister(&st_cdfinger_dev);
    return  status;
}


static const struct of_device_id cdfinger_of_match[] = {
    { .compatible = "cdfinger,fingerprint", },
    {},
};

static const struct platform_device_id cdfinger_id[] = {
    {"cdfinger_fp", 0},
    {}
};

static struct platform_driver cdfinger_driver = {
    .driver = {
        .name = "cdfinger_fp",
        .owner = THIS_MODULE,
        .of_match_table = cdfinger_of_match,
    },
    .id_table = cdfinger_id,
    .probe = cdfinger_probe,
};

static int __init cdfinger_fp_init(void)
{
    printk("revo_fingprint  enter %s \n",__func__);
    return platform_driver_register(&cdfinger_driver);
}

static void __exit cdfinger_fp_exit(void)
{
    platform_driver_unregister(&cdfinger_driver);
}

module_init(cdfinger_fp_init);
module_exit(cdfinger_fp_exit);

MODULE_DESCRIPTION("cdfinger spi Driver");
MODULE_AUTHOR("cdfinger@cdfinger.com");
MODULE_LICENSE("GPL");
MODULE_ALIAS("cdfinger");
