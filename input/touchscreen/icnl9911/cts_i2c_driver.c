#define LOG_TAG         "I2CDrv"

#include <linux/i2c.h>
#include <linux/of.h>
#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_driver.h"
#include "cts_strerror.h"

extern glb_chipone_icnt9911_tp_sysfs_init(struct i2c_client *client);
extern glb_chipone_icnt9911_tp_sysfs_remove(struct i2c_client *client);

#ifdef CONFIG_CTS_I2C_HOST
static int cts_i2c_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    if (client == NULL) {
        cts_err("I2C driver probe with client = NULL");
        return -EINVAL;
    }
    cts_info("I2C driver probe client: "
             "name='%s', addr=0x%02X, flags=0x%02X irq=%d",
             client->name, client->addr, client->flags, client->irq);

#if !defined(CONFIG_MTK_PLATFORM)
    if (client->addr != CTS_DEV_NORMAL_MODE_I2CADDR) {
        cts_err("Probe i2c addr 0x%02x != driver config addr 0x%02x",
            client->addr, CTS_DEV_NORMAL_MODE_I2CADDR);
        return -ENODEV;
    };
#endif

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        cts_err("Check functionality failed");
        return -ENODEV;
    }
    glb_chipone_icnt9911_tp_sysfs_init(client);

    return cts_driver_probe(&client->dev, CTS_I2C_BUS);
}

static int cts_i2c_driver_remove(struct i2c_client *client)
{
    if (client == NULL) {
        cts_err("I2C driver remove with client = NULL");
        return -EINVAL;
    }
    glb_chipone_icnt9911_tp_sysfs_remove(client);

    cts_info("I2C driver remove client: "
             "name='%s', addr=0x%02X, flags=0x%02X irq=%d",
        client->name, client->addr, client->flags, client->irq);

    return cts_driver_remove(&client->dev);
}

#ifdef CONFIG_CTS_PM_LEGACY
static int cts_i2c_driver_suspend(struct device *dev, pm_message_t state)
{
    cts_info("Suspend by legacy power management");
    return cts_driver_suspend(dev_get_drvdata(dev));
}

static int cts_i2c_driver_resume(struct device *dev)
{
    cts_info("Resume by legacy power management");
    return cts_driver_resume(dev_get_drvdata(dev));
}
#endif /* CONFIG_CTS_PM_LEGACY */

#ifdef CONFIG_CTS_PM_GENERIC
static int cts_i2c_driver_pm_suspend(struct device *dev)
{
    cts_info("Suspend by bus power management");
    return cts_driver_suspend(dev_get_drvdata(dev));
}

static int cts_i2c_driver_pm_resume(struct device *dev)
{
    cts_info("Resume by bus power management");
    return cts_driver_resume(dev_get_drvdata(dev));
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops cts_i2c_driver_pm_ops = {
    .suspend = cts_i2c_driver_pm_suspend,
    .resume = cts_i2c_driver_pm_resume,
};
#endif /* CONFIG_CTS_PM_GENERIC */


static const struct i2c_device_id cts_i2c_device_id_table[] = {
    {CFG_CTS_DEVICE_NAME, 0},
    {}
};

struct i2c_driver cts_i2c_driver = {
    .probe = cts_i2c_driver_probe,
    .remove = cts_i2c_driver_remove,
    .driver = {
        .name = CFG_CTS_DRIVER_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_CTS_OF
        .of_match_table = of_match_ptr(cts_driver_of_match_table),
#endif /* CONFIG_CTS_OF */
#ifdef CONFIG_CTS_SYSFS
        .groups = cts_driver_config_groups,
#endif /* CONFIG_CTS_SYSFS */
#ifdef CONFIG_CTS_PM_LEGACY
        .suspend = cts_i2c_driver_suspend,
        .resume  = cts_i2c_driver_resume,
#endif /* CONFIG_CTS_PM_LEGACY */
#ifdef CONFIG_CTS_PM_GENERIC
        .pm = &cts_i2c_driver_pm_ops,
#endif /* CONFIG_CTS_PM_GENERIC */

    },
    .id_table = cts_i2c_device_id_table,
};
#endif /* CONFIG_CTS_I2C_HOST */

