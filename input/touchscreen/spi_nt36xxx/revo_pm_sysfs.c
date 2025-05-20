#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

/*
	static
*/

extern int glb_novatek_revo_nvt_ts_suspend(void) ;
extern int glb_novatek_revo_nvt_ts_resume(void) ;


#define  REVO_TS_SUSPEND_FUNC           \
	do { \
		glb_novatek_revo_nvt_ts_suspend();\
	}while(0)

#define  REVO_TS_RESUME_FUNC             \
	do { \
		glb_novatek_revo_nvt_ts_resume() ;\
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
static DEVICE_ATTR(ts_suspend, 0664, NULL, ts_suspend_store);

static struct attribute *tp_sysfs_attrs[] = {
  &dev_attr_ts_suspend.attr,
  NULL,
};

static struct attribute_group tp_attr_group = {
  .attrs = tp_sysfs_attrs,
};

int glb_novatek_tp_sysfs_init(struct spi_device *client)
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
  return 0;
}
