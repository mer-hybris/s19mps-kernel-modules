#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include "prj/prj_config.h"

struct prox_pwr_data {
  int ldo_en_gpio;
};

static struct prox_pwr_data *ppd = NULL;

static int ldo_pwron(struct prox_pwr_data *ppd)
{
  gpio_direction_output(ppd->ldo_en_gpio, 1);
  return 0;
}

static int prox_ldo_probe(struct platform_device *pdev)
{
  int ret = 0;

  printk(KERN_INFO "%s\n", __FUNCTION__);
  /*kmalloc for struct prox_pwr_data*/
  ppd = kmalloc(sizeof(struct prox_pwr_data), GFP_KERNEL);
  if (!ppd) {
    printk(KERN_WARNING "prox_ldo: request memory failed\n");
    ret = -ENOMEM;
    goto err1;
  }
  pdev->dev.platform_data = (void *)ppd;

  ppd->ldo_en_gpio = of_get_named_gpio(pdev->dev.of_node, "proxldo-gpio", 0);
  if (!gpio_is_valid(ppd->ldo_en_gpio)) {
    pr_err("ldo_en_gpio is invalid\n");
    goto err2;
  }

  ret = devm_gpio_request(&pdev->dev, ppd->ldo_en_gpio, "proxldo-gpio");
  if (ret) {
    pr_err("ldo_en_gpio request fail\n");
    goto err2;
  }

  ldo_pwron(ppd);

err2:
  kfree(ppd);
err1:
  return ret;
}

static int prox_ldo_remove(struct platform_device *pdev)
{
  printk(KERN_INFO "%s\n", __FUNCTION__);
  /*free gpio*/
  if (ppd) {
    if (gpio_is_valid(ppd->ldo_en_gpio)) {
      gpio_direction_output(ppd->ldo_en_gpio, 0);
      gpio_free(ppd->ldo_en_gpio);
    }
    kfree(ppd);
  }

  return 0;
}

static const struct of_device_id hall_of_match[] = { 
    { .compatible = "revo,prox_ldo" }, 
};

static struct platform_driver hall_drvier = {
  .probe = prox_ldo_probe,
  .remove = prox_ldo_remove,
  .driver = { 
        .name = "prox_ldo",
        .of_match_table = of_match_ptr(hall_of_match),
  },
};

module_platform_driver(hall_drvier);

MODULE_DESCRIPTION("Prox ldo Driver");
MODULE_LICENSE("GPL");
