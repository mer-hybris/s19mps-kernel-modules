#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include  "revo_android.h"


extern int glb_focaltech_ft8756_resume(void) ;
extern int glb_focaltech_ft8756_suspend(void) ;


#define  REVO_TS_SUSPEND_FUNC            glb_focaltech_ft8756_resume()
#define  REVO_TS_RESUME_FUNC             glb_focaltech_ft8756_suspend()

//static struct kobject *tp_ctrl_kobj = NULL;

static ssize_t ts_suspend_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (kstrtouint(buf, 10, &input))
        return -EINVAL;

    if (input == 1)
        REVO_TS_SUSPEND_FUNC;
    else if (input == 0)
        REVO_TS_RESUME_FUNC;
    else
        return -EINVAL;

  return count;
}
static DEVICE_ATTR(ts_suspend, 0664, NULL, ts_suspend_store);


static ssize_t ts_name_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", TS_IC_LABEL);
}

static DEVICE_ATTR(ts_name, 0444, ts_name_show, NULL);

static struct attribute *tp_sysfs_attrs[] = {
    &dev_attr_ts_suspend.attr,
    &dev_attr_ts_name.attr,
    NULL,
};

static struct attribute_group tp_attr_group = {
    .attrs = tp_sysfs_attrs,
};

static int tp_sysfs_init(struct i2c_client *client)
{
//    tp_ctrl_kobj = kobject_create_and_add("touchscreen", NULL);
//    if (!tp_ctrl_kobj){
//        dev_err(&client->dev,"Create tp_sysfs_init failed!\n");
//        return -ENOMEM;
//    }
//    return sysfs_create_group(tp_ctrl_kobj, &tp_attr_group);
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



/*
	Global
*/

int glb_focaltech_ft8756_tp_sysfs_init(struct i2c_client *client){
    return tp_sysfs_init(client);
}


