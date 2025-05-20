#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>

extern int glb_chsc5x_ts_suspend(void);
extern int glb_chsc5x_ts_resume(void);


#define  REVO_TS_SUSPEND_FUNC           \
	do { \
		glb_chsc5x_ts_suspend();\
	}while(0)

#define  REVO_TS_RESUME_FUNC             \
	do { \
		glb_chsc5x_ts_resume() ;\
	}while(0)


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

static ssize_t input_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
  return sprintf(buf, "%s\n", "currency_tp");
}

static DEVICE_ATTR(ts_suspend, 0664, NULL, ts_suspend_store);
static DEVICE_ATTR(input_name, 0664, input_name_show, NULL);

static struct attribute *tp_sysfs_attrs[] = {
  &dev_attr_ts_suspend.attr,
  &dev_attr_input_name.attr,
  NULL,
};

static struct attribute_group tp_attr_group = {
  .attrs = tp_sysfs_attrs,
};

static int tp_sysfs_init(struct i2c_client *client)
{  
  int err = 0;
	err = sysfs_create_group(&client->dev.kobj, &tp_attr_group);
	if (err < 0) {
		dev_err(&client->dev, "Fail to create debug files!");
		return -ENOMEM;
	}

	err = sysfs_create_link(NULL, &client->dev.kobj, "touchscreen");
	if (err < 0) {
		dev_err(&client->dev, "Failed to create link!");
		return -ENOMEM;
	}
  return err;
}

int glb_chsc5x_tp_sysfs_init(struct i2c_client *client){
	return tp_sysfs_init(client);
}






