/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include "prj/prj_config.h"


#define PRINT_INFO(x...)  pr_info("[PROX_KPLED_INFO]" x)

#define KPLED_V_SHIFT           12
#define KPLED_V_MSK             (0x0F << KPLED_V_SHIFT)
#define KPLED_PD                (1 << 11)
#define KPLED_PULLDOWN_EN       (1 << 10)

#define LDO_V_SHIFT           7
#define LDO_V_MSK             (0xFF << LDO_V_SHIFT)
#define LDO_PD_MSK            (1 << 15)
#define LDO_PD_VALUE_ON       (~LDO_PD_MSK)
#define LDO_PD_VALUE_OFF      LDO_PD_MSK


enum sprd_pmic_kpled_switch {
	KPLED_SWITCH_OFF,
	KPLED_SWITCH_ON,
};

#if defined(CONFIG_REGULATOR_SC2730_MODULE)||defined(CONFIG_REGULATOR_SC2730)
#define SC27XX_ANA_REGS_GLB_BASE   (0x1800)
#define KPLED_CTRL_REG0            (SC27XX_ANA_REGS_GLB_BASE + 0x388)
#define KPLED_CTRL_REG1            (SC27XX_ANA_REGS_GLB_BASE + 0x38c)
#elif defined(CONFIG_REGULATOR_SC2720_MODULE)||defined(CONFIG_REGULATOR_SC2720)
#define SC27XX_ANA_REGS_GLB_BASE   (0xC00)
#define KPLED_CTRL_REG0            (SC27XX_ANA_REGS_GLB_BASE + 0x1f8)
#define KPLED_CTRL_REG1            (SC27XX_ANA_REGS_GLB_BASE + 0x1fc)
#else
#define SC27XX_ANA_REGS_GLB_BASE   (0xC00)
#define KPLED_CTRL_REG0            (SC27XX_ANA_REGS_GLB_BASE + 0x2ac)
#define KPLED_CTRL_REG1            (SC27XX_ANA_REGS_GLB_BASE + 0x2b0)
#endif

static struct regmap *flash_kpled_regmap;

static void sprd_kpled_switch(int power)
{	
	if (power == KPLED_SWITCH_ON)
	{
		regmap_update_bits(flash_kpled_regmap,KPLED_CTRL_REG1,LDO_PD_MSK, LDO_PD_VALUE_ON);
	}
	else
	{
		regmap_update_bits(flash_kpled_regmap,KPLED_CTRL_REG0,KPLED_PULLDOWN_EN, KPLED_PULLDOWN_EN);
		regmap_update_bits(flash_kpled_regmap,KPLED_CTRL_REG1,LDO_PD_MSK, LDO_PD_VALUE_OFF);
	}
}

static void prox_kpled_enable(void)
{
	PRINT_INFO("prox_kpled_enable\n");  
	sprd_kpled_switch(KPLED_SWITCH_ON);
}

static void prox_kpled_disable(void)
{
	PRINT_INFO("prox_kpled_disable\n");
	sprd_kpled_switch(KPLED_SWITCH_OFF);
}

static int sprd_prox_kpled_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk("sprd_prox_kpled_probe\n");
	if (IS_ERR(pdev))
	{
	  printk("pdev fail \n");
		return -EINVAL;
  }
	
	flash_kpled_regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!flash_kpled_regmap) {
		PRINT_INFO("%s :NULL spi parent property for kpled!", __func__);
		return -ENOMEM;
	}
	prox_kpled_enable();
	printk("sprd_prox_kpled_probe =%d\n",ret);
	
	return ret;
}

static int sprd_prox_kpled_remove(struct platform_device *pdev)
{
	prox_kpled_disable();
	return 0;
}

static const struct of_device_id prox_kpled_of_match[] = {
	{ .compatible = "sprd,prox-kpled", },
	{},
};

static struct platform_driver sprd_prox_kpled_driver = {
	.probe = sprd_prox_kpled_probe,
	.remove = sprd_prox_kpled_remove,
	.driver = {
		.name = "prox-kpled",
		.of_match_table = of_match_ptr(prox_kpled_of_match),
	},
};

module_platform_driver(sprd_prox_kpled_driver);

MODULE_DESCRIPTION("Prox kpled Driver");
MODULE_LICENSE("GPL");
