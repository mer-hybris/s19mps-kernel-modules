#ifndef REVO_ANDORID10_H
#define REVO_ANDORID10_H

#include <linux/i2c.h>
#include <linux/version.h>
#include "prj/prj_config.h"

#define  REVO_ADD_TP_NAME

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,133)
#define  REVO_ANDORID10_FOCALTECH_FT8006S
#define  REVO_ANDORID10
#define  REVO_PM_SYSFS
#else
#define  REVO_ANDORID9_FOCALTECH_FT8006S
#define  REVO_ANDORID9
/*sprd Android9.0 compile: adf_notifier */
#define  REVO_PM_FB_NOTIFIER
#include <linux/notifier.h>
#include <video/adf_notifier.h>
#endif



#define  REVO_TS_E_LOG_ON         1
#define  REVO_TS_D_LOG_ON         1

#define  TS_IC_LABEL             "focaltech,ft8006s"

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


extern int  glb_focaltech_ft8006s_tp_sysfs_init(struct i2c_client *client) ;
extern void focaltech_ft8006s_add_tp_name(void *ts_data);


#endif /* REVO_ANDORID10_H */