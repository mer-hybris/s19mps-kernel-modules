#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include  <linux/sysfs.h>

//static struct chipone_ts_data *cts_data = NULL ;
// cts_data->cts_dev.hwdata->hwid
// cts_data->cts_dev.hwdata->fwid
// ret = cts_fw_reg_readw_retry(cts_dev, CTS_DEVICE_FW_REG_VERSION, &device_fw_ver, 5, 0);

static struct show_name {
    char *drv_name ;
    char *ic_name ;
    int  fwid ;
    int  hwid ;
    u16 version;
} s_tpName_info ;


static ssize_t tp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   return  sprintf(buf, "%s:%s %s:%x %s:%x\n" "%s:%s %s:%04x\n", 
        "TP_NAME", s_tpName_info.ic_name ,
        "FW" , s_tpName_info.fwid ,
        "HW" , s_tpName_info.hwid ,
        "DRV", s_tpName_info.drv_name,
        "VER", s_tpName_info.version);
}

static struct kobj_attribute virtual_tp_name_attr = {
    .attr = {
        .name = "tp_name",
        .mode = S_IRUGO|S_IWUSR,
    },
    .show = &tp_name_show,
};


static struct attribute *properties_attrs[] = {
    &virtual_tp_name_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};


static void revo_tp_name_init(void *ts_data)
{
    int ret = 0;
	
    struct kobject *properties_kobj;

    pr_info(" create node about TP_NAME ");

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_info("failed to create board_properties\n");
}

void chipone_icnt9911_add_tp_name(void *ts_data)
{
    u16 firm_ver = 0;
    struct chipone_ts_data *cts_data = ts_data ;
    s_tpName_info.fwid = cts_data->cts_dev.hwdata->fwid ;
    s_tpName_info.hwid = cts_data->cts_dev.hwdata->hwid ;
    s_tpName_info.ic_name = (char *)cts_data->cts_dev.hwdata->name ;
    s_tpName_info.drv_name = "chipone,incl9911" ;
    
    cts_get_firmware_version(&cts_data->cts_dev, &firm_ver);
    s_tpName_info.version = firm_ver;

    revo_tp_name_init(cts_data);
}

