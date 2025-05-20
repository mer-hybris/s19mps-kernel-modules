#ifndef REVO_FT8756
#define REVO_FT8756

/* drivers/input/touchscreen/revo_ts */
#include "revo_android.h"
#include "tp_common.h"

#if defined(REVO_PM_SYSFS)||defined(REVO_PM_FB_NOTIFIER)
    #define REVO_SUSPEND_INTERFACES
#else
    #ifdef  CONFIG_FB
    #define CONFIG_FOCALTECH_FB
    #endif

    #ifdef CONFIG_DRM
    #define CONFIG_FOCALTECH_DRM
    #endif

    #ifdef CONFIG_HAS_EARLYSUSPEND
    #define CONFIG_FOCALTECH_HAS_EARLYSUSPEND
    #endif
#endif 


#define REVO_INPUT_DEV_NAME    "currency_tp"




#endif