#define LOG_TAG         "Sysfs"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_test.h"
#include "cts_sfctrl.h"
#include "cts_spi_flash.h"
#include "cts_firmware.h"
#include "cts_strerror.h"

#ifdef CONFIG_CTS_SYSFS

extern int kstrtobool(const char *s, bool *res);

#define SPLIT_LINE_STR \
    "-----------------------------------------------------------------------------------------------\n"
#define ROW_NUM_FORMAT_STR  "%2d | "
#define COL_NUM_FORMAT_STR  "%4u "
#define DATA_FORMAT_STR     "%5d"


#define DIFFDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

#define MAX_ARG_NUM                 (100)
#define MAX_ARG_LENGTH              (1024)

static char cmdline_param[MAX_ARG_LENGTH + 1];
int  argc;
char *argv[MAX_ARG_NUM];

static s16 *manualdiff_base = NULL;
u16 cts_spi_speed = 1000;

int parse_arg(const char *buf, size_t count)
{
    char *p;

    if (count > MAX_ARG_LENGTH) {
        return -EINVAL;
    }

    memcpy(cmdline_param, buf, min((size_t)MAX_ARG_LENGTH, count));
    cmdline_param[count] = '\0';

    argc = 0;
    p = strim(cmdline_param);
    if (p == NULL || p[0] == '\0') {
        return 0;
    }

    while (p && p[0] != '\0' && argc < MAX_ARG_NUM) {
        argv[argc++] = strsep(&p, " ,");
    }

    return argc;
}

/* echo addr value1 value2 value3 ... valueN > write_reg */
static ssize_t write_firmware_register_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 addr;
    int i, ret;
    u8 *data = NULL;

    parse_arg(buf, count);

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    if (argc < 2) {
        cts_err("Too few args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou16(argv[0], 0, &addr);
    if (ret) {
        cts_err("Invalid address: '%s'", argv[0]);
        return -EINVAL;
    }

    data = (u8 *)kmalloc(argc - 1, GFP_KERNEL);
    if (data == NULL) {
        cts_err("Alloc mem for write data failed");
        return -ENOMEM;
    }

    for (i = 1; i < argc; i++) {
        ret = kstrtou8(argv[i], 0, data + i - 1);
        if (ret) {
            cts_err("Invalid value: '%s'", argv[i]);
            goto free_data;
        }
    }

    cts_err("Write fw reg addr: 0x%04x size: %d", addr, argc - 1);

    ret = cts_fw_reg_writesb(cts_dev, addr, data, argc - 1);
    if (ret) {
        cts_err("Write fw reg addr: 0x%04x size: %d failed %d(%s)",
            addr, argc - 1, ret, cts_strerror(ret));
        goto free_data;
    }

free_data:
    kfree(data);

    return (ret < 0 ? ret : count);
}
static DEVICE_ATTR(write_reg, S_IWUSR, NULL, write_firmware_register_store);

static ssize_t read_firmware_register_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 addr, size;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    if (argc < 2 || argc > 4) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args %d\n"
            "  1. echo addr size [filepath]> read_reg\n"
            "  2. cat read_reg\n", argc);
    }

    ret = kstrtou16(argv[0], 0, &addr);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid address: '%s'\n", argv[0]);
    }
    ret = kstrtou16(argv[1], 0, &size);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid size: '%s'\n", argv[1]);
    }

    data = (u8 *)kmalloc(size, GFP_KERNEL);
    if (data == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for read data failed\n");
    }

    cts_info("Read fw reg from addr: 0x%04x size: %u%s%s",
        addr, size, argc == 3 ? " to file: " : "",
        argc == 3 ? argv[2] : "");

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readsb(cts_dev, addr, data, (size_t)size);
    cts_unlock_device(cts_dev);
    if (ret) {
        count = scnprintf(buf, PAGE_SIZE,
            "Read fw reg addr: 0x%04x size: %u failed %d(%s)\n",
            addr, size, ret, cts_strerror(ret));
        goto err_free_data;
    }

    if (argc > 2) {
        struct file *file;
        loff_t pos = 0;

        cts_info("Write fw reg data to file '%s'", argv[2]);

        file = filp_open_block(argv[2], O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (IS_ERR(file)) {
            ret = (int)PTR_ERR(file);
            count += scnprintf(buf, PAGE_SIZE,
                "Open file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));
            goto err_free_data;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
        ret = revo_kernel_write(file, data, size, &pos);
#else
        ret = revo_kernel_write(file, data, size, pos);
#endif
        if (ret != size) {
            count += scnprintf(buf, PAGE_SIZE,
                "Write fw reg data to file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));
        }

        ret = filp_close(file, NULL);
        if (ret) {
            count += scnprintf(buf, PAGE_SIZE,
                "Close file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));

        }
    }  else {
#define PRINT_ROW_SIZE          (16)
        u32 i, remaining;

        remaining = size;
        for (i = 0; i < size && count < PAGE_SIZE; i += PRINT_ROW_SIZE) {
            size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);
            remaining -= PRINT_ROW_SIZE;

            count += scnprintf(buf + count, PAGE_SIZE - count,
                "%04x: ", addr);

            /* Lower version kernel return void */
            hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
                buf + count, PAGE_SIZE - count, true);
            count += strlen(buf + count);

            if (count < PAGE_SIZE) {
                buf[count++] = '\n';
                addr += PRINT_ROW_SIZE;
            } else {
                break;
            }
        }
#undef PRINT_ROW_SIZE
    }


err_free_data:
    kfree(data);

    return count;
}

/* echo addr size > read_reg */
static ssize_t read_firmware_register_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return (argc == 0 ? 0 : count);
}
static DEVICE_ATTR(read_reg, S_IWUSR | S_IRUSR,
    read_firmware_register_show, read_firmware_register_store);

static ssize_t read_hw_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 addr, size;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    if (argc < 2 || argc > 4) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args %d\n"
            "  1. echo addr size [filepath]> read_hw_reg\n"
            "  2. cat read_hw_reg\n", argc);
    }

    ret = kstrtou32(argv[0], 0, &addr);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid address: '%s'\n", argv[0]);
    }
    ret = kstrtou32(argv[1], 0, &size);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid size: '%s'\n", argv[1]);
    }

    data = (u8 *)kmalloc(size, GFP_KERNEL);
    if (data == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for read data failed\n");
    }

    cts_info("Read hw reg from addr: 0x%08x size: %u%s%s",
        addr, size, argc == 3 ? " to file: " : "",
        argc == 3 ? argv[2] : "");

    cts_lock_device(cts_dev);
    ret = cts_hw_reg_readsb(cts_dev, addr, data, size);
    cts_unlock_device(cts_dev);

    if (ret) {
        count = scnprintf(buf, PAGE_SIZE,
            "Read hw reg addr: 0x%08x size: %u failed %d(%s)",
            addr, size, ret, cts_strerror(ret));
        goto err_free_data;
    }

    if (argc > 2) {
        struct file *file;
        loff_t pos = 0;

        cts_info("Write hw reg data to file '%s'", argv[2]);

        file = filp_open_block(argv[2], O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (IS_ERR(file)) {
            ret = (int)PTR_ERR(file);
            count += scnprintf(buf, PAGE_SIZE,
                "Open file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));
            goto err_free_data;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
        ret = revo_kernel_write(file, data, size, &pos);
#else
        ret = revo_kernel_write(file, data, size, pos);
#endif
        if (ret != size) {
            count += scnprintf(buf, PAGE_SIZE,
                "Write hw reg data to file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));
        }

        ret = filp_close(file, NULL);
        if (ret) {
            count += scnprintf(buf, PAGE_SIZE,
                "Close file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));

        }
    }  else {
#define PRINT_ROW_SIZE          (16)
        u32 i, remaining;

        remaining = size;
        for (i = 0; i < size && count < PAGE_SIZE; i += PRINT_ROW_SIZE) {
            size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);
            remaining -= PRINT_ROW_SIZE;

            count += scnprintf(buf + count, PAGE_SIZE - count,
                "%04x-%04x: ", (u16)(addr >> 16), (u16)addr);

            /* Lower version kernel return void */
            hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
                buf + count, PAGE_SIZE - count, true);
            count += strlen(buf + count);

            if (count < PAGE_SIZE) {
                buf[count++] = '\n';
                addr += PRINT_ROW_SIZE;
            } else {
                break;
            }
        }
#undef PRINT_ROW_SIZE
    }

err_free_data:
    kfree(data);
    return count;
}

/* echo addr size [filepath]> read_reg */
static ssize_t read_hw_reg_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return (argc == 0 ? 0 : count);
}

static DEVICE_ATTR(read_hw_reg, S_IRUSR | S_IWUSR,
    read_hw_reg_show, read_hw_reg_store);

static ssize_t write_hw_reg_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 addr;
    int i, ret;
    u8 *data = NULL;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc < 2) {
        cts_err("Too few args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou32(argv[0], 0, &addr);
    if (ret) {
        cts_err("Invalid address: '%s'", argv[0]);
        return -EINVAL;
    }

    data = (u8 *)kmalloc(argc - 1, GFP_KERNEL);
    if (data == NULL) {
        cts_err("Alloc mem for write data failed\n");
        return -ENOMEM;
    }

    for (i = 1; i < argc; i++) {
        ret = kstrtou8(argv[i], 0, data + i - 1);
        if (ret) {
            cts_err("Invalid value: '%s'", argv[i]);
            goto free_data;
        }
    }

    cts_info("Write hw reg addr: 0x%08x size: %u", addr, argc - 1);

    cts_lock_device(cts_dev);
    ret = cts_hw_reg_writesb(cts_dev, addr, data, argc - 1);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Write hw reg addr: 0x%08x size: %u failed %d(%s)",
            addr, argc - 1, ret, cts_strerror(ret));
    }

free_data:
    kfree(data);

    return (ret < 0 ? ret : count);
}

static DEVICE_ATTR(write_hw_reg, S_IWUSR, NULL, write_hw_reg_store);


static ssize_t curr_firmware_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u16 version;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_get_firmware_version(&cts_data->cts_dev, &version);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Get firmware version failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE, "Firmware version: %04x\n", version);
}
static DEVICE_ATTR(curr_version, S_IRUGO, curr_firmware_version_show, NULL);

static ssize_t curr_ddi_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u8 version;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_get_ddi_version(&cts_data->cts_dev, &version);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Get initcode version failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE, "Initcode version: %02x\n", version);
}
static DEVICE_ATTR(curr_ddi_version, S_IRUGO, curr_ddi_version_show, NULL);

static ssize_t rows_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u8  rows;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_get_num_rows(&cts_data->cts_dev, &rows);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Get num rows failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE, "Num rows: %u\n", rows);
}
static DEVICE_ATTR(rows, S_IRUGO, rows_show, NULL);

static ssize_t cols_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u8  cols;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_get_num_cols(&cts_data->cts_dev, &cols);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Get num cols failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE, "Num cols: %u\n", cols);
}
static DEVICE_ATTR(cols, S_IRUGO, cols_show, NULL);

static ssize_t res_x_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u16 res_x;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_get_x_resolution(&cts_data->cts_dev, &res_x);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Get X resolution failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE, "X Resolution: %u\n", res_x);
}
static DEVICE_ATTR(res_x, S_IRUGO, res_x_show, NULL);

static ssize_t res_y_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    u16 res_y;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_get_y_resolution(&cts_data->cts_dev, &res_y);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Get Y resolution failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE, "Y Resolution: %u\n", res_y);
}
static DEVICE_ATTR(res_y, S_IRUGO, res_y_show, NULL);

static ssize_t esd_protection_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8 esd_protection;

    cts_info("Read '%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb(&cts_data->cts_dev,
        CTS_DEVICE_FW_REG_ESD_PROTECTION, &esd_protection);
    cts_unlock_device(cts_dev);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Read firmware ESD protection register failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE,
        "ESD protection: %u\n", esd_protection);
}
static DEVICE_ATTR(esd_protection, S_IRUGO, esd_protection_show, NULL);

static ssize_t monitor_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8  value;

    cts_info("Read '%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb(&cts_data->cts_dev,
        CTS_DEVICE_FW_REG_FLAG_BITS, &value);
    cts_unlock_device(cts_dev);

    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Read firmware monitor enable register failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    return scnprintf(buf, PAGE_SIZE, "Monitor mode: %s\n",
        value & BIT(0) ? "Enable" : "Disable");
}

static ssize_t monitor_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8  value;
    bool enable;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num of args");
        return -EINVAL;
    }

    ret = kstrtobool(argv[0], &enable);
    if (ret) {
        cts_err("Invalid param of enable");
        return ret;
    }

    cts_info("%s firmware monitor mode", enable ? "Enable" : "Disable");

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb(&cts_data->cts_dev,
        CTS_DEVICE_FW_REG_FLAG_BITS, &value);
    cts_unlock_device(cts_dev);
    if (ret) {
        cts_err("Read firmware monitor enable register failed %d(%s)",
            ret, cts_strerror(ret));
        return -EIO;
    }

    if ((value & BIT(0)) && enable) {
        cts_info("Monitor mode already enabled");
    } else if ((value & BIT(0)) == 0 && enable == 0) {
        cts_info("Monitor mode already disabled");
    } else {
        if (enable) {
            value |= BIT(0);
        } else {
            value &= ~BIT(0);
        }

        cts_lock_device(cts_dev);
        ret = cts_fw_reg_writeb(&cts_data->cts_dev,
            CTS_DEVICE_FW_REG_FLAG_BITS, value);
        cts_unlock_device(cts_dev);
        if (ret) {
            cts_err("Write firmware monitor enable register failed %d(%s)",
                ret, cts_strerror(ret));
            return -EIO;
        }
    }

    return count;
}
static DEVICE_ATTR(monitor_mode, S_IWUSR | S_IRUGO,
    monitor_mode_show, monitor_mode_store);

static ssize_t auto_compensate_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;
    u8  value;

    cts_info("Read '%s'", attr->attr.name);

    cts_lock_device(cts_dev);
    ret = cts_fw_reg_readb(&cts_data->cts_dev,
        CTS_DEVICE_FW_REG_AUTO_CALIB_COMP_CAP_ENABLE, &value);
    cts_unlock_device(cts_dev);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Read auto compensate enable register failed %d\n", ret);
    }

    return scnprintf(buf, PAGE_SIZE,
        "Auto compensate: %s\n", value ? "Enable" : "Disable");
}
static DEVICE_ATTR(auto_compensate, S_IRUGO, auto_compensate_show, NULL);

#ifdef CFG_CTS_DRIVER_BUILTIN_FIRMWARE
static ssize_t driver_builtin_firmware_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int i, count = 0;

    cts_info("Read '%s'", attr->attr.name);

    count += scnprintf(buf + count, PAGE_SIZE - count,
        "Total %d builtin firmware:\n",
        cts_get_num_driver_builtin_firmware());

    for (i = 0; i < cts_get_num_driver_builtin_firmware(); i++) {
        const struct cts_firmware *firmware =
            cts_request_driver_builtin_firmware_by_index(i);
        if (firmware) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                "%-2d: hwid: %04x fwid: %04x ver: %04x size: %6zu desc: %s\n",
                i, firmware->hwid, firmware->fwid, FIRMWARE_VERSION(firmware),
                firmware->size, firmware->name);
         } else {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                "%-2d: INVALID\n", i);
         }
    }

    return count;
}

/* echo index/name [flash/sram] > driver_builtin_firmware */
static ssize_t driver_builtin_firmware_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_firmware *firmware;
    bool to_flash = true;
    int ret, index = -1;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc != 1 && argc != 2) {
        cts_err("Invalid num args %d\n"
            "  echo index/name [flash/sram] > driver_builtin_firmware\n",
            argc);
        return -EFAULT;
    }

    if (isdigit(*argv[0])) {
        index = simple_strtoul(argv[0], NULL, 0);
    }

    if (argc > 1) {
        if (strncasecmp(argv[1], "flash", 5) == 0) {
            to_flash = true;
        } else if (strncasecmp(argv[1], "sram", 4) == 0) {
            to_flash = false;
        } else {
            cts_err("Invalid location '%s', must be 'flash' or 'sram'",
                argv[1]);
            return -EINVAL;
        }
    }

    cts_info("Update driver builtin firmware '%s' to %s",
        argv[1], to_flash ? "flash" : "sram");

    if (index >= 0 && index < cts_get_num_driver_builtin_firmware()) {
        firmware = cts_request_driver_builtin_firmware_by_index(index);
    } else {
        firmware = cts_request_driver_builtin_firmware_by_name(argv[0]);
    }

    if (firmware) {
        ret = cts_stop_device(cts_dev);
        if (ret) {
            cts_err("Stop device failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }

        cts_lock_device(cts_dev);
        ret = cts_update_firmware(cts_dev, firmware, to_flash);
        cts_unlock_device(cts_dev);

        if (ret) {
            cts_err("Update firmware failed %d(%s)",
                ret, cts_strerror(ret));
            goto err_start_device;
        }

        ret = cts_start_device(cts_dev);
        if (ret) {
            cts_err("Start device failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }
    } else {
        cts_err("Firmware '%s' NOT found", argv[0]);
        return -ENOENT;
    }

    return count;

err_start_device:
    cts_start_device(cts_dev);

    return ret;
}
static DEVICE_ATTR(driver_builtin_firmware, S_IWUSR | S_IRUGO,
    driver_builtin_firmware_show, driver_builtin_firmware_store);
#endif /* CFG_CTS_DRIVER_BUILTIN_FIRMWARE */

#ifdef CFG_CTS_FIRMWARE_IN_FS
/* echo filepath [flash/sram] > update_firmware_from_file */
static ssize_t update_firmware_from_file_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_firmware *firmware;
    bool to_flash = true;
    int ret;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc > 2) {
        cts_err("Invalid num args %d\n"
                "  echo filepath [flash/sram] > update_from_file\n", argc);
        return -EFAULT;
    }

    if (argc > 1) {
        if (strncasecmp(argv[1], "flash", 5) == 0) {
            to_flash = true;
        } else if (strncasecmp(argv[1], "sram", 4) == 0) {
            to_flash = false;
        } else {
            cts_err("Invalid location '%s', must be 'flash' or 'sram'",
                argv[1]);
            return -EINVAL;
        }
    }

    cts_info("Update firmware from file '%s'", argv[0]);

    firmware = cts_request_firmware_from_fs(argv[0]);
    if (firmware == NULL) {
        cts_err("Request firmware from file '%s' failed", argv[0]);
        return -ENOENT;
    }

    ret = cts_stop_device(cts_dev);
    if (ret) {
        cts_err("Stop device failed %d(%s)",
            ret, cts_strerror(ret));
        goto err_release_firmware;
    }

    cts_lock_device(cts_dev);
    ret = cts_update_firmware(cts_dev, firmware, to_flash);
    cts_unlock_device(cts_dev);

    if (ret) {
        cts_err("Update firmware failed %d(%s)",
            ret, cts_strerror(ret));
        goto err_release_firmware;
    }

    ret = cts_start_device(cts_dev);
    if (ret) {
        cts_err("Start device failed %d(%s)",
            ret, cts_strerror(ret));
        goto err_release_firmware;
    }

    return count;

err_release_firmware:
    cts_release_firmware(firmware);

    return ret;
}
static DEVICE_ATTR(update_from_file, S_IWUSR,
    NULL, update_firmware_from_file_store);
#endif /* CFG_CTS_FIRMWARE_IN_FS */

static ssize_t updating_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "Updating: %s\n",
        cts_data->cts_dev.rtdata.updating ? "Y" : "N");
}
static DEVICE_ATTR(updating, S_IRUGO, updating_show, NULL);

static struct attribute *cts_dev_firmware_atts[] = {
    &dev_attr_curr_version.attr,
    &dev_attr_curr_ddi_version.attr,
    &dev_attr_rows.attr,
    &dev_attr_cols.attr,
    &dev_attr_res_x.attr,
    &dev_attr_res_y.attr,
    &dev_attr_esd_protection.attr,
    &dev_attr_monitor_mode.attr,
    &dev_attr_auto_compensate.attr,
#ifdef CFG_CTS_DRIVER_BUILTIN_FIRMWARE
    &dev_attr_driver_builtin_firmware.attr,
#endif /* CFG_CTS_DRIVER_BUILTIN_FIRMWARE */
#ifdef CFG_CTS_FIRMWARE_IN_FS
    &dev_attr_update_from_file.attr,
#endif /* CFG_CTS_FIRMWARE_IN_FS */
    &dev_attr_updating.attr,
    NULL
};

static const struct attribute_group cts_dev_firmware_attr_group = {
    .name  = "cts_firmware",
    .attrs = cts_dev_firmware_atts,
};

static ssize_t flash_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    const struct cts_flash *flash;

    cts_info("Read '%s'", attr->attr.name);

    if (cts_dev->flash == NULL) {
        bool program_mode;
        bool enabled;
        int  ret;

        program_mode = cts_is_device_program_mode(cts_dev);
        enabled = cts_is_device_enabled(cts_dev);

        ret = cts_prepare_flash_operation(cts_dev);
        if (ret) {
            return scnprintf(buf, PAGE_SIZE,
                "Prepare flash operation failed %d(%s)\n",
                ret, cts_strerror(ret));
        }

        cts_post_flash_operation(cts_dev);

        if (!program_mode) {
            ret = cts_enter_normal_mode(cts_dev);
            if (ret) {
                return scnprintf(buf, PAGE_SIZE,
                    "Enter normal mode failed %d(%s)\n",
                    ret, cts_strerror(ret));
            }
        }

        if (enabled) {
            ret = cts_start_device(cts_dev);
            if (ret) {
                return scnprintf(buf, PAGE_SIZE,
                    "Start device failed %d(%s)\n",
                    ret, cts_strerror(ret));
            }
        }

        if (cts_dev->flash == NULL) {
            return scnprintf(buf, PAGE_SIZE, "Flash not found\n");
        }
    }

    flash = cts_dev->flash;
    return scnprintf(buf, PAGE_SIZE,
        "%s:\n"
        "  JEDEC ID   : %06X\n"
        "  Page size  : 0x%zx\n"
        "  Sector size: 0x%zx\n"
        "  Block size : 0x%zx\n"
        "  Total size : 0x%zx\n",
        flash->name, flash->jedec_id, flash->page_size,
        flash->sector_size, flash->block_size, flash->total_size);
}
static DEVICE_ATTR(info, S_IRUGO, flash_info_show, NULL);

static ssize_t read_flash_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 flash_addr, size, i, remaining;
    u8 *data = NULL;
    ssize_t count = 0;
    int ret;
    bool program_mode;
    bool enabled;
    loff_t pos = 0;

    cts_info("Read '%s'", attr->attr.name);

    if (argc != 2 && argc != 3) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args %d\n", argc);
    }

    ret = kstrtou32(argv[0], 0, &flash_addr);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid flash addr: '%s'\n", argv[0]);
    }
    ret = kstrtou32(argv[1], 0, &size);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid size: '%s'\n", argv[1]);
    }

    data = (u8 *)kmalloc(size, GFP_KERNEL);
    if (data == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for read data failed\n");
    }

    cts_info("Read flash from addr: 0x%06x size: %u%s%s",
        flash_addr, size, argc == 3 ? " to file " : "",
        argc == 3 ? argv[2] : "");

    cts_lock_device(cts_dev);
    program_mode = cts_is_device_program_mode(cts_dev);
    enabled = cts_is_device_enabled(cts_dev);

    ret = cts_prepare_flash_operation(cts_dev);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Prepare flash operation failed %d(%s)\n",
            ret, cts_strerror(ret));
        goto err_free_data;
    }

    ret = cts_read_flash(cts_dev, flash_addr, data, size);
    if (ret) {
        count = scnprintf(buf, PAGE_SIZE,
            "Read flash from addr: 0x%06x size: %u failed %d(%s)\n",
            flash_addr, size, ret, cts_strerror(ret));
        goto err_post_flash_operation;
    }

    if (argc == 3) {
        struct file *file;

        cts_info("Write flash data to file '%s'", argv[2]);

        file = filp_open_block(argv[2], O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (IS_ERR(file)) {
            ret = (int)PTR_ERR(file);
            count += scnprintf(buf, PAGE_SIZE,
                "Open file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));
            goto err_post_flash_operation;
        }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
        ret = revo_kernel_write(file, data, size, &pos);
#else
        ret = revo_kernel_write(file, data, size, pos);
#endif
        if (ret != size) {
            count += scnprintf(buf, PAGE_SIZE,
                "Write flash data to file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));
        }

        ret = filp_close(file, NULL);
        if (ret) {
            count += scnprintf(buf, PAGE_SIZE,
                "Close file '%s' failed %d(%s)\n",
                argv[2], ret, cts_strerror(ret));

        }
    } else {
#define PRINT_ROW_SIZE          (16)
        remaining = size;
        for (i = 0; i < size && count <= PAGE_SIZE; i += PRINT_ROW_SIZE) {
            size_t linelen = min((size_t)remaining, (size_t)PRINT_ROW_SIZE);
            remaining -= PRINT_ROW_SIZE;

            count += scnprintf(buf + count, PAGE_SIZE - count - 1,
                "%04x-%04x: ", flash_addr >> 16, flash_addr & 0xFFFF);
            /* Lower version kernel return void */
            hex_dump_to_buffer(data + i, linelen, PRINT_ROW_SIZE, 1,
                buf + count, PAGE_SIZE - count - 1, true);
            count += strlen(buf + count);
            buf[count++] = '\n';
            flash_addr += linelen;
#undef PRINT_ROW_SIZE
        }
    }

err_post_flash_operation:
    cts_post_flash_operation(cts_dev);

    if (!program_mode) {
        int r = cts_enter_normal_mode(cts_dev);
        if (r) {
            count += scnprintf(buf, PAGE_SIZE,
                "Enter normal mode failed %d(%s)\n",
                r, cts_strerror(r));
        }
    }

    if (enabled) {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_unlock_device(cts_dev);
            return scnprintf(buf, PAGE_SIZE,
                "Start device failed %d(%s)\n",
                r, cts_strerror(r));
        }
    }
err_free_data:
    cts_unlock_device(cts_dev);
    kfree(data);

    return (ret < 0 ? ret : count);
}

/* echo addr size [filepath] > read */
static ssize_t read_flash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(read, S_IWUSR | S_IRUGO,
    read_flash_show, read_flash_store);

/* echo addr size > erase */
static ssize_t erase_flash_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u32 flash_addr, size;
    int ret;
    bool program_mode;
    bool enabled;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc != 2) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou32(argv[0], 0, &flash_addr);
    if (ret) {
        cts_err("Invalid flash addr: '%s'", argv[0]);
        return -EINVAL;
    }
    ret = kstrtou32(argv[1], 0, &size);
    if (ret) {
        cts_err("Invalid size: '%s'", argv[1]);
        return -EINVAL;
    }

    cts_info("Erase flash from 0x%06x size %u", flash_addr, size);

    cts_lock_device(cts_dev);
    program_mode = cts_is_device_program_mode(cts_dev);
    enabled = cts_is_device_enabled(cts_dev);

    ret = cts_prepare_flash_operation(cts_dev);
    if (ret) {
        cts_err("Prepare flash operation failed %d(%s)",
            ret, cts_strerror(ret));
        cts_unlock_device(cts_dev);
        return ret;
    }

    ret = cts_erase_flash(cts_dev, flash_addr, size);
    if (ret) {
        cts_err("Erase flash from 0x%06x size %u failed %d(%s)",
            flash_addr, size, ret, cts_strerror(ret));
        goto err_post_flash_operation;
    }

err_post_flash_operation:
    cts_post_flash_operation(cts_dev);

    if (!program_mode) {
        int r = cts_enter_normal_mode(cts_dev);
        if (r) {
            cts_err("Enter normal mode failed %d(%s)",
                r, cts_strerror(r));
        }
    }

    if (enabled) {
        int r = cts_start_device(cts_dev);
        if (r) {
            cts_err("Start device failed %d(%s)",
                r, cts_strerror(r));
        }
    }
    cts_unlock_device(cts_dev);

    return (ret < 0 ? ret : count);
}
static DEVICE_ATTR(erase, S_IWUSR, NULL, erase_flash_store);

static struct attribute *cts_dev_flash_attrs[] = {
    &dev_attr_info.attr,
    &dev_attr_read.attr,
    &dev_attr_erase.attr,
    NULL
};

static const struct attribute_group cts_dev_flash_attr_group = {
    .name  = "flash",
    .attrs = cts_dev_flash_attrs,
};

static ssize_t open_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 elapsed_time_ms = 0;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_OPEN,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath =
            "/sdcard/chipone-tddi/test/open-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &elapsed_time_ms,
    };
    int min = 0;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    if (argc != 1) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args %d\n", argc);
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid min thres: '%s'\n", argv[0]);
    }

    cts_info("Open test, threshold = %u", min);

    test_param.min = &min;

    ret = cts_test_open(cts_dev, &test_param);

    return cts_print_test_result_to_buffer(buf, PAGE_SIZE,
        "Open", ret, elapsed_time_ms);
}

/* echo threshold > open_test */
static ssize_t open_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(open_test, S_IWUSR | S_IRUGO,
    open_test_show, open_test_store);

static ssize_t short_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 elapsed_time_ms = 0;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_SHORT,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath =
            "/sdcard/chipone-tddi/test/short-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &elapsed_time_ms,
    };
    int min = 0;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    if (argc != 1) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args %d\n", argc);
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid min thres: '%s'\n", argv[0]);
    }

    cts_info("Short test, threshold = %u", min);

    test_param.min = &min;

    ret = cts_test_short(cts_dev, &test_param);

    return cts_print_test_result_to_buffer(buf, PAGE_SIZE,
        "Short", ret, elapsed_time_ms);
}

/* echo threshod > short_test */
static ssize_t short_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(short_test, S_IWUSR | S_IRUGO,
        short_test_show, short_test_store);

static ssize_t testing_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "Testting: %s\n",
        cts_data->cts_dev.rtdata.testing ? "Y" : "N");
}
static DEVICE_ATTR(testing, S_IRUGO, testing_show, NULL);

static ssize_t reset_pin_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#ifdef CFG_CTS_HAS_RESET_PIN
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 elapsed_time_ms = 0;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_RESET_PIN,
        .flags = 0,
        .elapsed_time_ms = &elapsed_time_ms,
    };
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_test_reset_pin(cts_dev, &test_param);

    return cts_print_test_result_to_buffer(buf, PAGE_SIZE,
        "Reset-Pin", ret, elapsed_time_ms);
#else /* CFG_CTS_HAS_RESET_PIN */
    return scnprintf(buf, PAGE_SIZE,
        "Reset-Pin test NOT supported(CFG_CTS_HAS_RESET_PIN not defined)\n");
#endif
}
static DEVICE_ATTR(reset_pin_test, S_IRUGO, reset_pin_test_show, NULL);

static ssize_t int_pin_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 elapsed_time_ms = 0;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_INT_PIN,
        .flags = 0,
        .elapsed_time_ms = &elapsed_time_ms,
    };
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    ret = cts_test_int_pin(cts_dev, &test_param);

    return cts_print_test_result_to_buffer(buf, PAGE_SIZE,
        "Int-Pin", ret, elapsed_time_ms);
}
static DEVICE_ATTR(int_pin_test, S_IRUGO, int_pin_test_show, NULL);

static ssize_t compensate_cap_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 elapsed_time_ms = 0;
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_COMPENSATE_CAP,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath =
            "/sdcard/chipone-tddi/test/comp-cap-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &elapsed_time_ms,
    };
    int min = 0, max = 0;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    if (argc != 2) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args\n"
            "USAGE:\n"
            "  1. echo min max > compensate_cap_test\n"
            "  2. cat compensate_cap_test\n");
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid min thres: '%s'\n", argv[0]);
    }

    ret = kstrtoint(argv[1], 0, &max);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid max thres: '%s'\n", argv[1]);
    }

    cts_info("Compensate cap test, min: %u, max: %u",
         min, max);

    test_param.min = &min;
    test_param.max = &max;

    ret = cts_test_compensate_cap(cts_dev, &test_param);

    return cts_print_test_result_to_buffer(buf, PAGE_SIZE,
        "Comp-CAP", ret, elapsed_time_ms);
}

static ssize_t compensate_cap_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(compensate_cap_test, S_IWUSR | S_IRUGO,
        compensate_cap_test_show, compensate_cap_test_store);

static ssize_t rawdata_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 elapsed_time_ms = 0;
    struct cts_rawdata_test_priv_param priv_param = {
        .frames = 16,
        //.work_mode = 0,
    };
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_RAWDATA,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath =
            "/sdcard/chipone-tddi/test/rawdata-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &elapsed_time_ms,
        .priv_param = &priv_param,
        .priv_param_size = sizeof(priv_param),
    };

    int min, max;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    if (argc < 2 || argc > 3) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args\n"
            "USAGE:\n"
            "  1. echo min max [frames] > rawdata_test\n"
            "  2. cat rawdata_test\n");
    }

    ret = kstrtoint(argv[0], 0, &min);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid min thres: '%s'\n", argv[0]);
    }

    ret = kstrtoint(argv[1], 0, &max);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid max thres: '%s'\n", argv[1]);
    }

    if (argc > 2) {
        ret = kstrtou32(argv[2], 0, &priv_param.frames);
        if (ret) {
            return scnprintf(buf, PAGE_SIZE,
                "Invalid frames: '%s'\n", argv[2]);
        }
    }
    cts_info("Rawdata test, frames: %u, min: %d, max: %d",
        priv_param.frames, min, max);

    test_param.min = &min;
    test_param.max = &max;

    ret = cts_test_rawdata(cts_dev, &test_param);

    return cts_print_test_result_to_buffer(buf, PAGE_SIZE,
        "Rawdata", ret, elapsed_time_ms);
}

static ssize_t rawdata_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(rawdata_test, S_IWUSR | S_IRUGO,
        rawdata_test_show, rawdata_test_store);

static ssize_t noise_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 elapsed_time_ms = 0;
    struct cts_noise_test_priv_param priv_param = {
        .frames = 50,
        //.work_mode = 0,
    };
    struct cts_test_param test_param = {
        .test_item = CTS_TEST_NOISE,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath =
            "/sdcard/chipone-tddi/test/noise-test-data.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &elapsed_time_ms,
        .priv_param = &priv_param,
        .priv_param_size = sizeof(priv_param),
    };

    int max;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    if (argc < 1 || argc > 2) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args\n"
            "USAGE:\n"
            "  1. echo threshold [frames] > noise_test\n"
            "  2. cat noise_test\n");
    }

    ret = kstrtoint(argv[0], 0, &max);
    if (ret) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid max thres: '%s'\n", argv[0]);
    }

    if (argc > 1) {
        ret = kstrtou32(argv[1], 0, &priv_param.frames);
        if (ret) {
            return scnprintf(buf, PAGE_SIZE,
                "Invalid frames: '%s'\n", argv[1]);
        }
    }
    cts_info("Noise test, frames: %u threshold: %d",
        priv_param.frames, max);

    test_param.max = &max;

    ret = cts_test_noise(cts_dev, &test_param);

    return cts_print_test_result_to_buffer(buf, PAGE_SIZE,
        "Noise", ret, elapsed_time_ms);
}

static ssize_t noise_test_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    return count;
}
static DEVICE_ATTR(noise_test, S_IWUSR | S_IRUGO,
        noise_test_show, noise_test_store);

static ssize_t factory_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define TOUCH_DATA_DIRECTORY_PREFIX           "/sdcard/chipone-tddi/test/touch_data_"
#define RAWDATA_TEST_DATA_FILENAME            "rawdata-test-data.txt"
#define NOISE_TEST_DATA_FILENAME              "noise-test-data.txt"
#define OPEN_TEST_DATA_FILENAME               "open-test-data.txt"
#define SHORT_TEST_DATA_FILENAME              "short-test-data.txt"
#define COMP_CAP_TEST_DATA_FILENAME           "comp-cap-test-data.txt"

    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s64 reset_pin_test_elapsed_time = 0;
    struct cts_test_param reset_pin_test_param = {
        .test_item = CTS_TEST_RESET_PIN,
        .flags = 0,
        .elapsed_time_ms = &reset_pin_test_elapsed_time,
    };
    s64 int_pin_test_elapsed_time = 0;
    struct cts_test_param int_pin_test_param = {
        .test_item = CTS_TEST_INT_PIN,
        .flags = 0,
        .elapsed_time_ms = &int_pin_test_elapsed_time,
    };
    struct cts_rawdata_test_priv_param rawdata_test_priv_param = {
        .frames = 16,
        //.work_mode = 0,
    };
    s64 rawdata_test_elapsed_time = 0;
    struct cts_test_param rawdata_test_param = {
        .test_item = CTS_TEST_RAWDATA,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &rawdata_test_elapsed_time,
        .priv_param = &rawdata_test_priv_param,
        .priv_param_size = sizeof(rawdata_test_priv_param),
    };
    struct cts_noise_test_priv_param noise_test_priv_param = {
        .frames = 50,
        //.work_mode = 0,
    };
    s64 noise_test_elapsed_time = 0;
    struct cts_test_param noise_test_param = {
        .test_item = CTS_TEST_NOISE,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &noise_test_elapsed_time,
        .priv_param = &noise_test_priv_param,
        .priv_param_size = sizeof(noise_test_priv_param),
    };
    s64 open_test_elapsed_time = 0;
    struct cts_test_param open_test_param = {
        .test_item = CTS_TEST_OPEN,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &open_test_elapsed_time,
    };
    s64 short_test_elapsed_time = 0;
    struct cts_test_param short_test_param = {
        .test_item = CTS_TEST_SHORT,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &short_test_elapsed_time,
    };
    s64 comp_cap_test_elapsed_time = 0;
    struct cts_test_param comp_cap_test_param = {
        .test_item = CTS_TEST_COMPENSATE_CAP,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &comp_cap_test_elapsed_time,
    };
    int rawdata_min = 0;
    int rawdata_max = 0;
    int noise_max = 50;
    int open_min = 200;
    int short_min = 200;
    int comp_cap_min = 20;
    int comp_cap_max = 100;
    int rawdata_test_result = 0;
    int reset_pin_test_result = 0;
    int int_pin_test_result = 0;
    int noise_test_result = 0;
    int open_test_result = 0;
    int short_test_result = 0;
    int comp_cap_test_result = 0;
    int count = 0;
    struct timespec64 ts;
    struct rtc_time rtc_tm;
    char touch_data_filepath[256];
    ktime_t start_time;
    u16 fw_version;
    int ret;

    cts_info("Read '%s'", attr->attr.name);

    rawdata_min = cts_dev->fwdata.rawdata_target * 3 / 4;
    rawdata_max = cts_dev->fwdata.rawdata_target * 5 / 4;
    rawdata_test_param.min = &rawdata_min;
    rawdata_test_param.max = &rawdata_max;
    noise_test_param.max = &noise_max;
    open_test_param.min = &open_min;
    short_test_param.min = &short_min;
    comp_cap_test_param.min = &comp_cap_min;
    comp_cap_test_param.max = &comp_cap_max;

    ktime_get_real_ts64(&ts);
    ts.tv_sec -= sys_tz.tz_minuteswest * 60;
    rtc_time_to_tm(ts.tv_sec, &rtc_tm);

    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        RAWDATA_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    rawdata_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);
    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        NOISE_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    noise_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);
    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        OPEN_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    open_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);
    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        SHORT_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    short_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);
    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        COMP_CAP_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    comp_cap_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);

    cts_info("Factory test: "
             "rawdata: [%d, %d], "
             "noise: %d, "
             "open: %d, "
             "short: %d, "
             "comp_cap: [%d, %d], "
             "touch data dir: "
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/",
        rawdata_min, rawdata_max, noise_max, open_min, short_min,
        comp_cap_min, comp_cap_max,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);

    start_time = ktime_get();

    ret = cts_get_firmware_version(cts_dev, &fw_version);
    if (ret) {
        count += scnprintf(buf + count, PAGE_SIZE,
            "Factory test get firmware version failed %d(%s)\n",
            ret, cts_strerror(ret));
        fw_version = 0;
    }

    reset_pin_test_result =
        cts_test_reset_pin(cts_dev, &reset_pin_test_param);
    int_pin_test_result =
        cts_test_int_pin(cts_dev, &int_pin_test_param);
    rawdata_test_result =
        cts_test_rawdata(cts_dev, &rawdata_test_param);
    noise_test_result =
        cts_test_noise(cts_dev, &noise_test_param);
    open_test_result =
        cts_test_open(cts_dev, &open_test_param);
    short_test_result =
        cts_test_short(cts_dev, &short_test_param);
    comp_cap_test_result =
        cts_test_compensate_cap(cts_dev, &comp_cap_test_param);

    if (rawdata_test_param.test_data_filepath) {
        kfree(rawdata_test_param.test_data_filepath);
    }
    if (noise_test_param.test_data_filepath) {
        kfree(noise_test_param.test_data_filepath);
    }
    if (open_test_param.test_data_filepath) {
        kfree(open_test_param.test_data_filepath);
    }
    if (short_test_param.test_data_filepath) {
        kfree(short_test_param.test_data_filepath);
    }
    if (comp_cap_test_param.test_data_filepath) {
        kfree(comp_cap_test_param.test_data_filepath);
    }

    count += scnprintf(buf + count, PAGE_SIZE,
        "Factory test, total ELAPSED TIME: %lldms\n"
        "Touch Data Dir : "
            TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d\n",
        ktime_ms_delta(ktime_get(), start_time),
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);

    count += scnprintf(buf + count, PAGE_SIZE,
        "FirmwareVersion: 0x%04X\n", fw_version);
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "Reset-Pin", reset_pin_test_result,
        *(reset_pin_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "Int-Pin", int_pin_test_result,
        *(int_pin_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "Rawdata", rawdata_test_result,
        *(rawdata_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "Noise", noise_test_result,
        *(noise_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "Open", open_test_result,
        *(open_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "Short", short_test_result,
        *(short_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "Comp-Cap", comp_cap_test_result,
        *(comp_cap_test_param.elapsed_time_ms));

    return count;

#undef RAWDATA_TEST_DATA_FILENAME
#undef NOISE_TEST_DATA_FILENAME
#undef OPEN_TEST_DATA_FILENAME
#undef SHORT_TEST_DATA_FILENAME
#undef COMP_CAP_TEST_DATA_FILENAME
}
static DEVICE_ATTR(factory_test,S_IRUGO,
        factory_test_show, NULL);

static ssize_t gesture_test_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define GESTURE_RAWDATA_TEST_DATA_FILENAME    "gesture-rawdata-test-data.txt"
#define GESTURE_LP_RAWDATA_TEST_DATA_FILENAME "gesture-lp-rawdata-test-data.txt"
#define GESTURE_NOISE_TEST_DATA_FILENAME      "gesture-noise-test-data.txt"
#define GESTURE_LP_NOISE_TEST_DATA_FILENAME   "gesture-lp-noise-test-data.txt"

    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    struct cts_rawdata_test_priv_param gesture_rawdata_test_priv_param = {
        .frames = 3,
        .work_mode = 1,
    };
    s64 gesture_rawdata_test_elapsed_time = 0;
    struct cts_test_param gesture_rawdata_test_param = {
        .test_item = CTS_TEST_GESTURE_RAWDATA,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &gesture_rawdata_test_elapsed_time,
        .priv_param = &gesture_rawdata_test_priv_param,
        .priv_param_size = sizeof(gesture_rawdata_test_priv_param),
    };
    struct cts_rawdata_test_priv_param gesture_lp_rawdata_test_priv_param = {
        .frames = 3,
        .work_mode = 0,
    };
    s64 gesture_lp_rawdata_test_elapsed_time = 0;
    struct cts_test_param gesture_lp_rawdata_test_param = {
        .test_item = CTS_TEST_GESTURE_LP_RAWDATA,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &gesture_lp_rawdata_test_elapsed_time,
        .priv_param = &gesture_lp_rawdata_test_priv_param,
        .priv_param_size = sizeof(gesture_lp_rawdata_test_priv_param),
    };
    struct cts_noise_test_priv_param gesture_noise_test_priv_param = {
        .frames = 3,
        .work_mode = 1,
    };
    s64 gesture_noise_test_elapsed_time = 0;
    struct cts_test_param gesture_noise_test_param = {
        .test_item = CTS_TEST_GESTURE_NOISE,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &gesture_noise_test_elapsed_time,
        .priv_param = &gesture_noise_test_priv_param,
        .priv_param_size = sizeof(gesture_noise_test_priv_param),
    };
    struct cts_noise_test_priv_param gesture_lp_noise_test_priv_param = {
        .frames = 3,
        .work_mode = 0,
    };
    s64 gesture_lp_noise_test_elapsed_time = 0;
    struct cts_test_param gesture_lp_noise_test_param = {
        .test_item = CTS_TEST_GESTURE_LP_NOISE,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_CONSOLE |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_FILE,
        .test_data_filepath = NULL,
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .elapsed_time_ms = &gesture_lp_noise_test_elapsed_time,
        .priv_param = &gesture_lp_noise_test_priv_param,
        .priv_param_size = sizeof(gesture_lp_noise_test_priv_param),
    };

    int gesture_rawdata_min = 0;
    int gesture_rawdata_max = 0;
    int gesture_lp_rawdata_min = 0;
    int noise_max = 50;
    int gesture_rawdata_test_result = 0;
    int gesture_lp_rawdata_test_result = 0;
    int gesture_noise_test_result = 0;
    int gesture_lp_noise_test_result = 0;
    int count = 0;
    struct timespec64 ts;
    struct rtc_time rtc_tm;
    char touch_data_filepath[256];
    ktime_t start_time;

    cts_info("Read '%s'", attr->attr.name);

    gesture_rawdata_min = cts_dev->fwdata.gstr_rawdata_target * 7 / 10;
    gesture_rawdata_max = cts_dev->fwdata.gstr_rawdata_target * 13 / 10;
    gesture_rawdata_test_param.min = &gesture_rawdata_min;
    gesture_rawdata_test_param.max = &gesture_rawdata_max;
    gesture_lp_rawdata_test_param.min = &gesture_lp_rawdata_min;
    gesture_lp_rawdata_test_param.max = &gesture_rawdata_max;
    gesture_noise_test_param.max = &noise_max;
    gesture_lp_noise_test_param.max = &noise_max;

    ktime_get_real_ts64(&ts);
    ts.tv_sec -= sys_tz.tz_minuteswest * 60;
    rtc_time_to_tm(ts.tv_sec, &rtc_tm);

    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        GESTURE_RAWDATA_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    gesture_rawdata_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);
    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        GESTURE_LP_RAWDATA_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    gesture_lp_rawdata_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);
    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        GESTURE_NOISE_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    gesture_noise_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);
    snprintf(touch_data_filepath, sizeof(touch_data_filepath),
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/"
        GESTURE_LP_NOISE_TEST_DATA_FILENAME,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);
    gesture_lp_noise_test_param.test_data_filepath =
        kstrdup(touch_data_filepath, GFP_KERNEL);

    cts_info("Gesture test: "
             "Gesture raw: [%d, %d], "
             "Gesture noise: %d, "
             "touch data dir: "
        TOUCH_DATA_DIRECTORY_PREFIX"%04d%02d%02d_%02d%02d%02d/",
        gesture_rawdata_min, gesture_rawdata_max, noise_max,
        rtc_tm.tm_year + 1900, rtc_tm.tm_mon + 1, rtc_tm.tm_mday,
        rtc_tm.tm_hour, rtc_tm.tm_min,rtc_tm.tm_sec);

    start_time = ktime_get();

    cts_display_on(cts_dev, false);

    gesture_rawdata_test_result =
        cts_test_gesture_rawdata(cts_dev, &gesture_rawdata_test_param);
    gesture_lp_rawdata_test_result =
        cts_test_gesture_rawdata(cts_dev, &gesture_lp_rawdata_test_param);
    gesture_noise_test_result =
        cts_test_gesture_noise(cts_dev, &gesture_noise_test_param);
    gesture_lp_noise_test_result =
        cts_test_gesture_noise(cts_dev, &gesture_lp_noise_test_param);

    cts_display_on(cts_dev, true);

    cts_plat_reset_device(cts_dev->pdata);

    if (gesture_rawdata_test_param.test_data_filepath) {
        kfree(gesture_rawdata_test_param.test_data_filepath);
    }
    if (gesture_lp_rawdata_test_param.test_data_filepath) {
        kfree(gesture_lp_rawdata_test_param.test_data_filepath);
    }
    if (gesture_noise_test_param.test_data_filepath) {
        kfree(gesture_noise_test_param.test_data_filepath);
    }
    if (gesture_lp_noise_test_param.test_data_filepath) {
        kfree(gesture_lp_noise_test_param.test_data_filepath);
    }

    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "G-Raw", gesture_rawdata_test_result,
        *(gesture_rawdata_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "G-LP-Raw", gesture_lp_rawdata_test_result,
        *(gesture_lp_rawdata_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "G-Noise", gesture_noise_test_result,
        *(gesture_noise_test_param.elapsed_time_ms));
    count += cts_print_test_result_to_buffer(buf + count, PAGE_SIZE,
        "G-LP-Noise", gesture_lp_noise_test_result,
        *(gesture_lp_noise_test_param.elapsed_time_ms));
    return count;

#undef GESTURE_RAWDATA_TEST_DATA_FILENAME
#undef GESTURE_LP_RAWDATA_TEST_DATA_FILENAME
#undef GESTURE_NOISE_TEST_DATA_FILENAME
#undef GESTURE_LP_NOISE_TEST_DATA_FILENAME
}
#undef TOUCH_DATA_DIRECTORY_PREFIX
static DEVICE_ATTR(gesture_test,S_IRUGO,
        gesture_test_show, NULL);

static struct attribute *cts_dev_test_atts[] = {
    &dev_attr_testing.attr,
    &dev_attr_reset_pin_test.attr,
    &dev_attr_int_pin_test.attr,
    &dev_attr_rawdata_test.attr,
    &dev_attr_noise_test.attr,
    &dev_attr_open_test.attr,
    &dev_attr_short_test.attr,
    &dev_attr_compensate_cap_test.attr,
    &dev_attr_factory_test.attr,
    &dev_attr_gesture_test.attr,
    NULL
};

static const struct attribute_group cts_dev_test_attr_group = {
    .name  = "test",
    .attrs = cts_dev_test_atts,
};

static ssize_t ic_type_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "IC Type : %s\n",
        cts_data->cts_dev.hwdata->name);
}
static DEVICE_ATTR(ic_type, S_IRUGO, ic_type_show, NULL);

static ssize_t program_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "Program mode: %s\n",
        cts_data->cts_dev.rtdata.program_mode ? "Y" : "N");
}
static ssize_t program_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    bool program_mode;
    int ret;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EINVAL;
    }

    ret = kstrtobool(argv[0], &program_mode);
    if (ret) {
        cts_err("Invalid param of program_mode");
        return ret;
    }

    cts_lock_device(&cts_data->cts_dev);
    if (program_mode) {
        ret = cts_enter_program_mode(&cts_data->cts_dev);
    } else {
        ret = cts_enter_normal_mode(&cts_data->cts_dev);
    }
    cts_unlock_device(&cts_data->cts_dev);

    if (ret) {
        cts_err("%s program mode failed %d(%s)",
            program_mode ? "Enter" : "Exit",
            ret, cts_strerror(ret));
        return ret;
   }

    return count;
}
static DEVICE_ATTR(program_mode, S_IWUSR | S_IRUGO,
        program_mode_show, program_mode_store);

static ssize_t rawdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define RAWDATA_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 *rawdata = NULL;
    int ret, r, c, count = 0;
    u32 max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;

    cts_info("Read '%s'", attr->attr.name);

    rawdata = (u16 *)kmalloc(RAWDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for rawdata failed\n");
    }

    cts_lock_device(cts_dev);
    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Enable read touch data failed %d(%s)\n",
            ret, cts_strerror(ret));
        goto err_free_rawdata;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Send cmd QUIT_GESTURE_MONITOR failed %d(%s)\n",
            ret, cts_strerror(ret));
        goto err_free_rawdata;
    }
    msleep(50);

    ret = cts_get_rawdata(cts_dev, rawdata);
    if(ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Get raw data failed %d(%s)\n",
            ret, cts_strerror(ret));
        data_valid = false;
        /* Fall through to disable get touch data */
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Disable read touch data failed %d(%s)\n",
            ret, cts_strerror(ret));
        /* Fall through to show rawdata */
    }

    if (data_valid) {
        max = min = rawdata[0];
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                u16 val = rawdata[r * cts_dev->fwdata.cols + c];
                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += scnprintf(buf + count, PAGE_SIZE - count,
            SPLIT_LINE_STR
            "Raw data MIN: [%d][%d]=%u, MAX: [%d][%d]=%u, AVG=%u\n"
            SPLIT_LINE_STR
            "   |  ", min_r, min_c, min, max_r, max_c, max, average);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                COL_NUM_FORMAT_STR, c);
        }
        count += scnprintf(buf + count, PAGE_SIZE - count,
            "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows && count < PAGE_SIZE; r++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols && count < PAGE_SIZE; c++) {
                count += scnprintf(buf + count, PAGE_SIZE - count,
                    DATA_FORMAT_STR, rawdata[r * cts_dev->fwdata.cols + c]);
            }
            buf[count++] = '\n';
        }
    }

err_free_rawdata:
    cts_unlock_device(cts_dev);
    kfree(rawdata);

    return (data_valid ? count : ret);

#undef RAWDATA_BUFFER_SIZE
}
static DEVICE_ATTR(rawdata, S_IRUGO, rawdata_show, NULL);

static ssize_t diffdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s16 *diffdata = NULL;
    int ret, r, c, count = 0;
    int max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;

    cts_info("Read '%s'", attr->attr.name);

    diffdata = (s16 *)kmalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (diffdata == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for rawdata failed\n");
    }

    cts_lock_device(cts_dev);
    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Enable read touch data failed %d(%s)\n",
            ret, cts_strerror(ret));
        goto err_free_diffdata;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Send cmd QUIT_GESTURE_MONITOR failed %d(%s)\n",
            ret, cts_strerror(ret));
        goto err_free_diffdata;
    }
    msleep(50);

    ret = cts_get_diffdata(cts_dev, diffdata);
    if(ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Get diff data failed %d(%s)\n",
            ret, cts_strerror(ret));
        data_valid = false;
        /* Fall through to disable get touch data */
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        count += scnprintf(buf, PAGE_SIZE,
            "Disable read touch data failed %d(%s)\n",
            ret, cts_strerror(ret));
        /* Fall through to show diffdata */
    }

    if (data_valid) {
        max = min = diffdata[0];
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                s16 val = diffdata[r * cts_dev->fwdata.cols + c];

                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += scnprintf(buf + count, PAGE_SIZE - count,
            SPLIT_LINE_STR
            "Diff data MIN: [%d][%d]=%d, MAX: [%d][%d]=%d, AVG=%d\n"
            SPLIT_LINE_STR
            "   |  ", min_r, min_c, min, max_r, max_c, max, average);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                COL_NUM_FORMAT_STR, c);
        }
        count += scnprintf(buf + count, PAGE_SIZE - count,
            "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                count += scnprintf(buf + count, PAGE_SIZE - count,
                    DATA_FORMAT_STR, diffdata[r * cts_dev->fwdata.cols + c]);
           }
           buf[count++] = '\n';
        }
    }

err_free_diffdata:
    cts_unlock_device(cts_dev);
    kfree(diffdata);

    return (data_valid ? count : ret);
}
static DEVICE_ATTR(diffdata, S_IRUGO, diffdata_show, NULL);

static ssize_t baseline_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
#define BASELINE_BUFFER_SIZE(cts_dev) \
    (cts_dev->fwdata.rows * cts_dev->fwdata.cols * 2)

    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u16 *baseline = NULL;
    int ret, r, c, count = 0;
    u32 max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    u16 addr = 0;
    u8  data_width = 0;

    cts_info("Read '%s'", attr->attr.name);

    baseline = (u16 *)kmalloc(BASELINE_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (baseline == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for baseline failed\n");
    }

    cts_lock_device(cts_dev);
    ret = cts_get_baseline(cts_dev, baseline, CTS_WORK_MODE_NORMAL_ACTIVE,
        CTS_GET_TOUCH_DATA_FLAG_ENABLE_GET_TOUCH_DATA_BEFORE |
        CTS_GET_TOUCH_DATA_FLAG_CLEAR_DATA_READY |
        CTS_GET_TOUCH_DATA_FLAG_REMOVE_TOUCH_DATA_BORDER |
        CTS_GET_TOUCH_DATA_FLAG_FLIP_TOUCH_DATA |
        CTS_GET_TOUCH_DATA_FLAG_DISABLE_GET_TOUCH_DATA_AFTER,
        addr, data_width);
    cts_unlock_device(cts_dev);

    if (ret) {
        kfree(baseline);
        return scnprintf(buf, PAGE_SIZE,
            "Get baseline failed %d(%s)\n", ret, cts_strerror(ret));
    }

    max = min = baseline[0];
    sum = 0;
    max_r = max_c = min_r = min_c = 0;
    for (r = 0; r < cts_dev->fwdata.rows; r++) {
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            u16 val = baseline[r * cts_dev->fwdata.cols + c];
            sum += val;
            if (val > max) {
                max = val;
                max_r = r;
                max_c = c;
            } else if (val < min) {
                min = val;
                min_r = r;
                min_c = c;
            }
        }
    }
    average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

    count += scnprintf(buf + count, PAGE_SIZE - count,
        SPLIT_LINE_STR
        "Baseline MIN: [%d][%d]=%u, MAX: [%d][%d]=%u, AVG=%u\n"
        SPLIT_LINE_STR
        "   |  ", min_r, min_c, min, max_r, max_c, max, average);
    for (c = 0; c < cts_dev->fwdata.cols; c++) {
        count += scnprintf(buf + count, PAGE_SIZE - count,
            COL_NUM_FORMAT_STR, c);
    }
    count += scnprintf(buf + count, PAGE_SIZE - count,
        "\n" SPLIT_LINE_STR);

    for (r = 0; r < cts_dev->fwdata.rows && count < PAGE_SIZE; r++) {
        count += scnprintf(buf + count, PAGE_SIZE - count,
            ROW_NUM_FORMAT_STR, r);
        for (c = 0; c < cts_dev->fwdata.cols && count < PAGE_SIZE; c++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                DATA_FORMAT_STR, baseline[r * cts_dev->fwdata.cols + c]);
        }
        buf[count++] = '\n';
    }

    kfree(baseline);

    return count;

#undef BASELINE_BUFFER_SIZE
}
static DEVICE_ATTR(baseline, S_IRUGO, baseline_show, NULL);

static ssize_t manualdiffdata_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s16 *rawdata = NULL;
    int ret, r, c, count = 0;
    int max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = true;
    int frame = 1;
    struct file *file = NULL;
    int i;
    loff_t pos = 0;

    cts_info("Read '%s'", attr->attr.name);

    if (argc != 1 && argc != 2) {
        return scnprintf(buf, PAGE_SIZE,
            "Invalid num args\n"
            "USAGE:\n"
            "  1. echo updatebase > manualdiff\n"
            "  2. cat manualdiff\n"
            "  or\n"
            "  1. echo updatebase > manualdiff\n"
            "  2. echo frame file > manualdiff\n"
            "  3. cat manualdiff\n");
    }

    if (argc == 2) {
        ret = kstrtou32(argv[0], 0, &frame);
        if (ret) {
            return scnprintf(buf, PAGE_SIZE,
                "Invalid frame num: '%s'\n", argv[0]);
        }
        file = filp_open_block(argv[1], O_RDWR | O_CREAT | O_TRUNC, 0666);
        if (IS_ERR(file)) {
            ret = PTR_ERR(file);
            return scnprintf(buf, PAGE_SIZE,
                "Open file '%s' failed %d(%s)",
                argv[1], ret, cts_strerror(ret));
        }
    }

    rawdata = (s16 *)kmalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata == NULL) {
        cts_err("Alloc mem for rawdata failed");
        filp_close(file, NULL);
        return -ENOMEM;
    }

    cts_lock_device(cts_dev);
    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Enable read raw data failed %d(%s)",
            ret, cts_strerror(ret));
        goto err_free_diffdata;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
            ret, cts_strerror(ret));
        goto err_free_diffdata;
    }
    msleep(50);

    cts_info("frame %d, file:%s", frame, argv[1]);
    for (i = 0; i < frame; i++) {
        ret = cts_get_rawdata(cts_dev, rawdata);
        if(ret) {
            cts_err("Get raw data failed %d(%s)",
            ret, cts_strerror(ret));
            data_valid = false;
        }
        else {
            data_valid = true;
        }
        msleep(50);

        if (data_valid) {
            max = -32768;
            min = 32767;
            sum = 0;
            max_r = max_c = min_r = min_c = 0;
            for (r = 0; r < cts_dev->fwdata.rows; r++) {
                for (c = 0; c < cts_dev->fwdata.cols; c++) {
                    s16 val;

                    rawdata[r * cts_dev->fwdata.cols + c] -=
                        manualdiff_base[r * cts_dev->fwdata.cols + c];
                    val = rawdata[r * cts_dev->fwdata.cols + c];
                    sum += val;
                    if (val > max) {
                        max = val;
                        max_r = r;
                        max_c = c;
                    } else if (val < min) {
                        min = val;
                        min_r = r;
                        min_c = c;
                    }
                }
            }
            average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);
            count += scnprintf(buf + count, PAGE_SIZE - count,
                SPLIT_LINE_STR
                "Manualdiff data MIN: [%d][%d]=%d, MAX: [%d][%d]=%d, AVG=%d\n"
                SPLIT_LINE_STR
                "   |  ", min_r, min_c, min, max_r, max_c, max, average);
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                count += scnprintf(buf + count,PAGE_SIZE - count,
                    COL_NUM_FORMAT_STR, c);
            }
            count += scnprintf(buf + count, PAGE_SIZE - count,
                "\n" SPLIT_LINE_STR);

            for (r = 0; r < cts_dev->fwdata.rows; r++) {
                count += scnprintf(buf + count, PAGE_SIZE - count,
                    ROW_NUM_FORMAT_STR, r);
                for (c = 0; c < cts_dev->fwdata.cols; c++) {
                    count += scnprintf(buf + count, PAGE_SIZE - count,
                        DATA_FORMAT_STR, rawdata[r * cts_dev->fwdata.cols + c]);
               }
               buf[count++] = '\n';
            }

            if (argc == 2) {
                pos = file->f_pos;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
                ret = revo_kernel_write(file, buf, count, &pos);
#else
                ret = revo_kernel_write(file, buf, count, pos);
#endif
                if (ret != count) {
                    cts_err("Write data to file '%s' failed %d(%s)",
                        argv[1], ret, cts_strerror(ret));
                }
                file->f_pos += count;
                count = 0;
            }
        }
    }
    if (argc == 2) {
        filp_close(file, NULL);
    }

    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Disable read raw data failed %d(%s)",
            ret, cts_strerror(ret));
        /* Fall through to show diffdata */
    }

err_free_diffdata:
    cts_unlock_device(cts_dev);
    kfree(rawdata);

    return (data_valid ? count : ret);
}

static ssize_t manualdiffdata_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    cts_lock_device(cts_dev);
    if (strncasecmp("updatebase", argv[0], 10) == 0) {
        ret = cts_enable_get_rawdata(cts_dev);
        if (ret) {
            cts_err("Enable read raw data failed %d(%s)",
                ret, cts_strerror(ret));
            goto err_manual_diff_store;
        }

        ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
        if (ret) {
            cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
                ret, cts_strerror(ret));
            goto err_manual_diff_store;
        }
        msleep(50);

        if (manualdiff_base != NULL) {
            ret = cts_get_rawdata(cts_dev, manualdiff_base);
            if(ret) {
                cts_err("Get raw data failed %d(%s)",
                    ret, cts_strerror(ret));
            }
        }

        ret = cts_disable_get_rawdata(cts_dev);
        if (ret) {
            cts_err("Disable read raw data failed %d(%s)",
                ret, cts_strerror(ret));
        }
    }
err_manual_diff_store:
    cts_unlock_device(cts_dev);
    return count;
}

static DEVICE_ATTR(manualdiff, S_IRUSR | S_IWUSR,
    manualdiffdata_show, manualdiffdata_store);

static ssize_t jitter_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    cts_info("Write '%s' size %zu", attr->attr.name, count);
    return count;
}

static ssize_t jitter_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    s16 *rawdata = NULL;
    s16 *rawdata_min = NULL;
    s16 *rawdata_max = NULL;
    int ret, r, c, count = 0;
    int max, min, sum, average;
    int max_r, max_c, min_r, min_c;
    bool data_valid = false;
    u32 i, frames;

    cts_info("Read '%s'", attr->attr.name);

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou32(argv[0], 0, &frames);
    if (ret == 0) {
        cts_err("Invalid frames: '%s'", argv[0]);
        return -EFAULT;
    }

    cts_info("Jitter test frame: %d", frames);

    cts_lock_device(cts_dev);
    rawdata = (s16 *)kmalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata == NULL) {
        cts_err("Alloc mem for curr rawdata failed");
        ret = -ENOMEM;
        goto err_jitter_show_exit;
    }
    rawdata_min = (s16 *)kmalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata_min == NULL) {
        cts_err("Alloc mem for rawdata min failed");
        ret = -ENOMEM;
        goto err_free_rawdata;
    }
    rawdata_max = (s16 *)kmalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (rawdata_max == NULL) {
        cts_err("Alloc mem for rawdata failed");
        ret = -ENOMEM;
        goto err_free_rawdata_min;
    }

    for (i = 0; i < DIFFDATA_BUFFER_SIZE(cts_dev)/2; i++) {
        rawdata_min[i] = 32767;
        rawdata_max[i] = -32768;
    }

    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Enable read raw data failed %d(%s)",
            ret, cts_strerror(ret));
        goto err_free_rawdata_max;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
            ret, cts_strerror(ret));
        goto err_free_rawdata_max;
    }
    msleep(50);
    data_valid = true;
    for (i = 0; i < frames; i++) {
        ret = cts_get_rawdata(cts_dev, rawdata);
        if(ret) {
            cts_err("Get raw data failed %d(%s)",
                ret, cts_strerror(ret));
            continue;
        }
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                int index;
                index = r * cts_dev->fwdata.cols + c;
                if (rawdata_min[index] > rawdata[index])
                    rawdata_min[index] = rawdata[index];
                else if (rawdata_max[index] < rawdata[index])
                    rawdata_max[index] = rawdata[index];
            }
        }
        msleep(1);
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Disable read raw data failed %d(%s)",
            ret, cts_strerror(ret));
    }

    if (data_valid) {
        max = -32768;
        min = 32767;
        sum = 0;
        max_r = max_c = min_r = min_c = 0;
        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                s16 val;
                int index = r * cts_dev->fwdata.cols + c;

                val = rawdata_max[index] - rawdata_min[index];
                rawdata[index] = val;
                sum += val;
                if (val > max) {
                    max = val;
                    max_r = r;
                    max_c = c;
                } else if (val < min) {
                    min = val;
                    min_r = r;
                    min_c = c;
                }
            }
        }
        average = sum / (cts_dev->fwdata.rows * cts_dev->fwdata.cols);

        count += scnprintf(buf + count, PAGE_SIZE - count,
            SPLIT_LINE_STR
            "Jitter data MIN: [%d][%d]=%d, MAX: [%d][%d]=%d, AVG=%d, TOTAL FRAME=%u\n"
            SPLIT_LINE_STR
            "   |  ",
            min_r, min_c, min, max_r, max_c, max, average, frames);
        for (c = 0; c < cts_dev->fwdata.cols; c++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                COL_NUM_FORMAT_STR, c);
        }
        count += scnprintf(buf + count, PAGE_SIZE - count,
            "\n" SPLIT_LINE_STR);

        for (r = 0; r < cts_dev->fwdata.rows; r++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                ROW_NUM_FORMAT_STR, r);
            for (c = 0; c < cts_dev->fwdata.cols; c++) {
                count += scnprintf(buf + count, PAGE_SIZE - count,
                    DATA_FORMAT_STR, rawdata[r * cts_dev->fwdata.cols + c]);
           }
           buf[count++] = '\n';
        }
    }

err_free_rawdata_max:
    kfree(rawdata_max);
err_free_rawdata_min:
    kfree(rawdata_min);
err_free_rawdata:
    kfree(rawdata);
err_jitter_show_exit:
    cts_unlock_device(cts_dev);
    return (data_valid ? count : ret);
}

static DEVICE_ATTR(jitter, S_IRUSR|S_IWUSR, jitter_show, jitter_store);

static ssize_t compensate_cap_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    u8 *cap = NULL;
    int ret;
    ssize_t count = 0;
    int r, c, min, max, max_r, max_c, min_r, min_c, sum, average;

    cts_info("Read '%s'", attr->attr.name);

    cap = kzalloc(cts_dev->hwdata->num_row * cts_dev->hwdata->num_col,
        GFP_KERNEL);
    if (cap == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "Alloc mem for compensate cap failed\n");
    }

    cts_lock_device(cts_dev);
    ret = cts_get_compensate_cap(cts_dev, cap);
    cts_unlock_device(cts_dev);
    if (ret) {
        kfree(cap);
        return scnprintf(buf, PAGE_SIZE,
            "Get compensate cap failed %d(%s)\n",
            ret, cts_strerror(ret));
    }

    max = min = cap[0];
    sum = 0;
    max_r = max_c = min_r = min_c = 0;
    for (r = 0; r < cts_dev->hwdata->num_row; r++) {
        for (c = 0; c < cts_dev->hwdata->num_col; c++) {
            u16 val = cap[r * cts_dev->hwdata->num_col + c];
            sum += val;
            if (val > max) {
                max = val;
                max_r = r;
                max_c = c;
            } else if (val < min) {
                min = val;
                min_r = r;
                min_c = c;
            }
        }
    }
    average = sum / (cts_dev->hwdata->num_row * cts_dev->hwdata->num_col);

    count += scnprintf(buf + count, PAGE_SIZE - count,
        "----------------------------------------------------------------------------\n"
        " Compensatete Cap MIN: [%d][%d]=%u, MAX: [%d][%d]=%u, AVG=%u\n"
        "---+------------------------------------------------------------------------\n"
        "   |", min_r, min_c, min, max_r, max_c, max, average);
    for (c = 0; c < cts_dev->hwdata->num_col; c++) {
        count += scnprintf(buf + count, PAGE_SIZE - count, " %3u", c);
    }
    count += scnprintf(buf + count, PAGE_SIZE - count,
        "\n"
        "---+------------------------------------------------------------------------\n");

    for (r = 0; r < cts_dev->hwdata->num_row; r++) {
        count += scnprintf(buf + count, PAGE_SIZE - count, "%2u |", r);
        for (c = 0; c < cts_dev->hwdata->num_col; c++) {
            count += scnprintf(buf + count, PAGE_SIZE - count,
                " %3u", cap[r * cts_dev->hwdata->num_col + c]);
       }
       buf[count++] = '\n';
    }
    count += scnprintf(buf + count, PAGE_SIZE - count,
        "---+------------------------------------------------------------------------\n");

    kfree(cap);

    return count;
}
static DEVICE_ATTR(compensate_cap, S_IRUGO, compensate_cap_show, NULL);

#ifdef CFG_CTS_HAS_RESET_PIN
static ssize_t reset_pin_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE,
        "Reset pin: %d, status: %d\n",
        cts_data->pdata->rst_gpio,
        gpio_get_value(cts_data->pdata->rst_gpio));
}

static ssize_t reset_pin_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    cts_info("Write '%s' size %zu", attr->attr.name, count);
    cts_info("Chip status maybe changed");

    cts_plat_set_reset(cts_dev->pdata, (buf[0] == '1') ? 1 : 0);
    return count;
}
static DEVICE_ATTR(reset_pin, S_IRUSR | S_IWUSR, reset_pin_show, reset_pin_store);
#endif

static ssize_t irq_pin_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE,
        "IRQ pin: %d, status: %d\n",
        cts_data->pdata->int_gpio,
        gpio_get_value(cts_data->pdata->int_gpio));
}
static DEVICE_ATTR(irq_pin, S_IRUGO, irq_pin_show, NULL);

static ssize_t irq_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct irq_desc *desc;

    cts_info("Read '%s'", attr->attr.name);

    desc = irq_to_desc(cts_data->pdata->irq);
    if (desc == NULL) {
        return scnprintf(buf, PAGE_SIZE,
            "IRQ: %d descriptor not found\n",
            cts_data->pdata->irq);
    }

    return scnprintf(buf, PAGE_SIZE,
        "IRQ num: %d, depth: %u, "
        "count: %u, unhandled: %u, last unhandled eslape: %lu\n",
        cts_data->pdata->irq, desc->depth,
        desc->irq_count, desc->irqs_unhandled,
        desc->last_unhandled);
}
static DEVICE_ATTR(irq_info, S_IRUGO, irq_info_show, NULL);

#ifdef CFG_CTS_FW_LOG_REDIRECT
static ssize_t fw_log_redirect_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "Fw log redirect: %s\n",
        cts_is_fw_log_redirect(cts_dev)? "ENABLED" : "DISABLED");
}

static ssize_t fw_log_redirect_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    bool enable;
    int ret;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EINVAL;
    }

    ret = kstrtobool(argv[0], &enable);
    if (ret) {
        cts_err("Invalid param of enable");
        return ret;
    }

    cts_lock_device(&cts_data->cts_dev)
    if (enable) {
        ret = cts_enable_fw_log_redirect(&cts_data->cts_dev);
    } else {
        ret = cts_disable_fw_log_redirect(&cts_data->cts_dev);
    }
    cts_unlock_device(&cts_data->cts_dev);

    if (ret) {
        cts_err("%s fw log redirect failed %d(%s)",
            enable ? "Enable" : "Disable",
            ret, cts_strerror(ret));
        return ret;
    }

    return count;
}

static DEVICE_ATTR(fw_log_redirect, S_IRUSR | S_IWUSR,
    fw_log_redirect_show, fw_log_redirect_store);
#endif

static ssize_t debug_spi_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "spi_speed=%d\n", cts_spi_speed);
}

static ssize_t debug_spi_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    u16 s = 0;
    int ret = 0;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EFAULT;
    }

    ret = kstrtou16(argv[0], 0, &s);
    if (ret) {
        cts_err("Invalid spi speed: %s", argv[0]);
        return -EINVAL;
    }

    cts_spi_speed = s;

    return count;
}

static DEVICE_ATTR(debug_spi, S_IRUSR | S_IWUSR,
    debug_spi_show, debug_spi_store);

#ifdef CFG_CTS_GESTURE
static ssize_t gesture_en_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;

    cts_info("Read '%s'", attr->attr.name);

    return scnprintf(buf, PAGE_SIZE, "Gesture wakup is %s\n",
        cts_is_gesture_wakeup_enabled(cts_dev)? "ENABLED" : "DISABLED");
}

static ssize_t gesture_en_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    bool enable;
    int ret;

    cts_info("Write '%s' size %zu", attr->attr.name, count);

    parse_arg(buf, count);

    if (argc != 1) {
        cts_err("Invalid num args %d", argc);
        return -EINVAL;
    }

    ret = kstrtobool(argv[0], &enable);
    if (ret) {
        cts_err("Invalid param of enable");
        return ret;
    }

    if (enable) {
        cts_enable_gesture_wakeup(cts_dev);
    } else {
        cts_disable_gesture_wakeup(cts_dev);
    }

    return count;
}
static DEVICE_ATTR(gesture_en, S_IRUSR | S_IWUSR,
    gesture_en_show, gesture_en_store);
#endif

static struct attribute *cts_dev_misc_atts[] = {
    &dev_attr_ic_type.attr,
    &dev_attr_program_mode.attr,
    &dev_attr_rawdata.attr,
    &dev_attr_diffdata.attr,
    &dev_attr_baseline.attr,
    &dev_attr_manualdiff.attr,
    &dev_attr_jitter.attr,
#ifdef CFG_CTS_HAS_RESET_PIN
    &dev_attr_reset_pin.attr,
#endif
    &dev_attr_irq_pin.attr,
    &dev_attr_irq_info.attr,
#ifdef CFG_CTS_FW_LOG_REDIRECT
    &dev_attr_fw_log_redirect.attr,
#endif
    &dev_attr_compensate_cap.attr,
    &dev_attr_read_reg.attr,
    &dev_attr_write_reg.attr,
    &dev_attr_read_hw_reg.attr,
    &dev_attr_write_hw_reg.attr,
    &dev_attr_debug_spi.attr,
#ifdef CFG_CTS_GESTURE
    &dev_attr_gesture_en.attr,
#endif /* CFG_CTS_GESTURE */
    NULL
};

static const struct attribute_group cts_dev_misc_attr_group = {
    .name  = "misc",
    .attrs = cts_dev_misc_atts,
};

static const struct attribute_group *cts_dev_attr_groups[] = {
    &cts_dev_firmware_attr_group,
    &cts_dev_flash_attr_group,
    &cts_dev_test_attr_group,
    &cts_dev_misc_attr_group,
    NULL
};

int cts_sysfs_add_device(struct device *dev)
{
    struct chipone_ts_data *cts_data = dev_get_drvdata(dev);
    struct cts_device *cts_dev = &cts_data->cts_dev;
    int ret = 0, i;

    cts_info("Add device attr groups");

    /* Low version kernel NOT support sysfs_create_groups() */
    for (i = 0; cts_dev_attr_groups[i]; i++) {
        ret = sysfs_create_group(&dev->kobj, cts_dev_attr_groups[i]);
        if (ret) {
            cts_err("Create sysfs group '%s' failed %d(%s)",
                cts_dev_attr_groups[i]->name,
                ret, cts_strerror(ret));
            while (--i >= 0) {
                sysfs_remove_group(&dev->kobj, cts_dev_attr_groups[i]);
            }
            break;
        }
    }

    if (ret) {
        cts_err("Add device attr failed %d", ret);
        return ret;
    }

    manualdiff_base = (s16 *)kzalloc(DIFFDATA_BUFFER_SIZE(cts_dev), GFP_KERNEL);
    if (manualdiff_base == NULL) {
        cts_err("Malloc manualdiff_base failed");
        return -ENOMEM;
    }

    ret = sysfs_create_link(NULL, &dev->kobj, "chipone-tddi");
    if (ret) {
        cts_err("Create sysfs link error:%d", ret);
    }
    return 0;
}

void cts_sysfs_remove_device(struct device *dev)
{
    int i;

    cts_info("Remove device attr groups");

    if (manualdiff_base != NULL) {
        kfree(manualdiff_base);
        manualdiff_base = NULL;
    }

    sysfs_remove_link(NULL, "chipone-tddi");
    // Low version kernel NOT support sysfs_remove_groups()
    for (i = 0; cts_dev_attr_groups[i]; i++) {
        sysfs_remove_group(&dev->kobj, cts_dev_attr_groups[i]);
    }
}

#undef SPLIT_LINE_STR
#undef ROW_NUM_FORMAT_STR
#undef COL_NUM_FORMAT_STR
#undef DATA_FORMAT_STR

#endif /* CONFIG_CTS_SYSFS */
