#include "revo_android.h"
#include "../focaltech_core.h"
#include  <linux/sysfs.h>

struct fts_ts_data*  revo_ts_data (struct fts_ts_data * set_data )
{
    static struct fts_ts_data*  get_data = NULL ;
    
    if( get_data ) /* is  set */
        return get_data ;
 
    if(set_data){  /* input data valid */
        get_data = set_data ;
    }

    return get_data ;  // return    valid  or  null 
}

static struct show_name {
    char *drv_name ;
    char *ic_name ;
    int  fwid ;
    int  hwid ;
} s_tpName_info ;


static ssize_t tp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
   
   return  sprintf(buf, "%s:%s \n" "%s:%s\n", 
        "TP_NAME", s_tpName_info.ic_name ,
//        "FW" , s_tpName_info.fwid ,
//        "HW" , s_tpName_info.hwid ,
        "DRV", s_tpName_info.drv_name);
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

    REVO_D_LOG(" create node about TP_NAME ");

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        REVO_E_LOG("failed to create board_properties\n");
}

void focaltech_ft8756_add_tp_name(void *ts_data)
{

    struct fts_ts_data *cts_data = revo_ts_data(ts_data);  
    
    s_tpName_info.fwid = 0;//cts_data->cts_dev.hwdata->fwid ;
    s_tpName_info.hwid = 0;//cts_data->cts_dev.hwdata->hwid ;
    s_tpName_info.ic_name = "ft8756";//cts_data->cts_dev.hwdata->name ;
    s_tpName_info.drv_name = "focaltech,ft8756" ;
    
    revo_tp_name_init(cts_data);
}

