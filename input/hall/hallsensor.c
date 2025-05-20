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

struct hall_data {
	struct input_dev *hs_dev;
	int irq_gpio;
	int hall_flag;
	int hall_irq;
	struct work_struct work; /*work queue*/
	spinlock_t irq_lock;
};
static struct hall_data *phd = NULL;

void hall_irq_disable(void)
{
	unsigned long irqflags;

	spin_lock_irqsave(&phd->irq_lock, irqflags);
	disable_irq_nosync(phd->hall_irq);
	spin_unlock_irqrestore(&phd->irq_lock, irqflags);
}
void hall_irq_enable(void)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&phd->irq_lock, irqflags);
	enable_irq(phd->hall_irq);
	spin_unlock_irqrestore(&phd->irq_lock, irqflags);
}

static void hall_sensor_work(struct work_struct *work)
{
	/*report data*/
	int hall_state;
	mdelay(10);
	hall_state = gpio_get_value(phd->irq_gpio); /*get hall_sensor status*/
	if(phd->hall_flag) {	//IRQF_TRIGGER_RISING
		if(hall_state == phd->hall_flag) {
			input_report_key(phd->hs_dev, BTN_THUMBL, 1);
            input_report_key(phd->hs_dev, BTN_THUMBL, 0);
			input_sync(phd->hs_dev);
		}
	} else {	//IRQF_TRIGGER_FALLING
		if(hall_state == phd->hall_flag) {
			input_report_key(phd->hs_dev, BTN_THUMBR, 1);
            input_report_key(phd->hs_dev, BTN_THUMBR, 0);
			input_sync(phd->hs_dev);
		}
	}
	hall_irq_enable();
}
static irqreturn_t hall_irq_handler(int irq, void *dev_id)
{ 
	phd->hall_flag = gpio_get_value(phd->irq_gpio);	/*record hall_sensor status */
	hall_irq_disable();
	schedule_work(&phd->work);
	return IRQ_HANDLED;
}

static int hall_probe(struct platform_device *pdev)
{
	int error = 0;
	int irq = 0;
	int ret = 0;
	struct input_dev *input_dev = NULL;
	printk(KERN_INFO "%s\n", __FUNCTION__);

	if (!pdev->dev.of_node) {
		pr_err("no device node %s", __func__);
		return -ENODEV;
	}
	pr_info("tjd hall_probe\n");

	/*kmalloc for struct hall_data*/
	phd = kmalloc(sizeof(struct hall_data), GFP_KERNEL);
	if(!phd) {
		printk(KERN_WARNING "hall: request memory failed\n");
		error = -ENOMEM;
		goto err1;
	}
	pdev->dev.platform_data = (void *)phd;

	/*request GPIO and IRQ*/
	phd->irq_gpio = of_get_named_gpio(pdev->dev.of_node,"hall-irq-gpio", 0);
	if (gpio_is_valid(phd->irq_gpio)) {
		ret = devm_gpio_request(&pdev->dev,	phd->irq_gpio, "hall-irq-gpio");
		if (ret) {
			pr_err("hall gpio err\n");
			goto err2;
		}

		ret = gpio_direction_input(phd->irq_gpio);
		if (ret) {
			pr_err("flash gpio output err\n");
			goto err2;
		}
		irq = gpio_to_irq(phd->irq_gpio);
		phd->hall_irq = irq;
		printk(KERN_INFO "hall: request_irq gpio's irq is %d\n", irq);
	}

	/*input subsystem*/
	//Application input_dev
	input_dev = input_allocate_device();
	if(!(input_dev)) {
		printk(KERN_WARNING "hall: can't get input_dev!\n");
		error = -ENOMEM;
		goto err3;
	}

	input_dev->name = "hall_sensor";
	input_dev->phys = "input/hs";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.product = 0x0001;
	input_dev->id.vendor = 0x0001;
	input_dev->id.version = 0x0100;
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_THUMBL, input_dev->keybit);
	__set_bit(BTN_THUMBR, input_dev->keybit);
	//Register input_dev
	input_register_device(input_dev);
	phd->hs_dev = input_dev;
	
	spin_lock_init(&phd->irq_lock);
	INIT_WORK(&phd->work, hall_sensor_work);
	if(irq > 0) {	//IRQ
		error = request_irq(irq, hall_irq_handler, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND|IRQF_ONESHOT, "hall_irq", phd);
		if (error) {
			printk(KERN_WARNING "hall: request_irq gpio %d's irq failed!\n", irq);
			goto err4;
		}
	} else {
		printk(KERN_WARNING "hall: gpio_to_irq %d's irq failed!\n", irq);
		goto err4;
	}
	hall_irq_disable();
	phd->hall_flag = gpio_get_value(phd->irq_gpio);
	printk(KERN_INFO "%s: hall_flag = %d\n", __FUNCTION__, phd->hall_flag);

	//dev_set_drvdata(&pdev->dev, irq);
	hall_irq_enable();
	return 0;

err4:
	input_unregister_device(input_dev);
err3:
	gpio_free(phd->irq_gpio);
err2:
	kfree(phd);
err1:
	return error;
}
static int hall_remove(struct platform_device *pdev)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	/*free gpio*/
	gpio_direction_output(phd->irq_gpio, 0);
	gpio_free(phd->irq_gpio);
	/*unregister input_dev*/
	input_unregister_device(phd->hs_dev);
	/*free irq*/
	free_irq(phd->hall_irq, NULL);
	kfree(phd);
	return 0;
}

static const struct of_device_id hall_of_match[] = {
	{.compatible = "revo,hall"},
};

static struct platform_driver hall_drvier = {
	.probe = hall_probe,
	.remove = hall_remove,
	.driver = {
		.name = "hallsensor",
		.of_match_table = of_match_ptr(hall_of_match),
	},
};

module_platform_driver(hall_drvier);
MODULE_DESCRIPTION("Revo Hall sensor Driver");
MODULE_LICENSE("GPL");
