#include  <linux/input.h>
#include  <linux/sysfs.h>
#include  "head_def.h"

#ifndef  REVO_E_LOG
#define  REVO_TS_E_LOG_ON         1
#define  REVO_TS_D_LOG_ON         1

#define  TS_IC_LABEL             "chipsemi.chsc5x"

#define REVO_E_LOG(fmt,arg...)         \
    do{\
        if(REVO_TS_E_LOG_ON)\
            printk("<Revo_ts> <<-%s-Revo_E->> <func:%s> [line%d] "fmt"\n",TS_IC_LABEL,__func__,__LINE__, ##arg);\
    }while(0)
        
#define REVO_D_LOG(fmt,arg...)   \
    do{\
        if(REVO_TS_D_LOG_ON)\
            printk("<Revo_ts> <<-%s-Revo_D->> <func:%s> [line%d] "fmt"\n",TS_IC_LABEL,__func__,__LINE__, ##arg);\
    }while(0)
#endif




static ssize_t tp_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "TP_NAME: %s \n", TS_IC_LABEL );
}

static struct kobj_attribute virtual_tp_name_attr = {
    .attr = {
        .name = "tp_name",
        .mode = 0444,
    },
    .show = &tp_name_show,
};

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY)     ":" __stringify(KEY_BACK)           ":40:1500:75:200"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)       ":80:1500:75:200"
	":" __stringify(EV_KEY) ":" __stringify(KEY_APPSELECT)      ":120:1500:75:200"
	"\n");
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		 .name = "virtualkeys.currency_tp",
		 .mode = 0444,
		 },
	.show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	//&virtual_keys_attr.attr,
	&virtual_tp_name_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};


void glb_chipsemi_chsc5x_revo_tp_name_init(void *ts_data)
{
    int ret = 0;
	struct sm_touch_dev *tp_data ;
    void *nop ;
    struct kobject *properties_kobj;

    nop = &virtual_keys_attr ;
	tp_data = ts_data ;

    REVO_D_LOG(" create node about TP_NAME ");

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        REVO_E_LOG("failed to create board_properties\n");
}
