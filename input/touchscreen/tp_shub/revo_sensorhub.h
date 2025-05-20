#ifndef  REVO_SENSORHUB_H
#define  REVO_SENSORHUB_H
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/input.h>
#include "shub_common.h"

#ifdef   SENSORHUB_C_FILE
#define  FUNC_EXT
#else
#define  FUNC_EXT   extern
#endif

struct shub_event_params {
	/*
	 * eHalSenData : 129
	 * eHalFlush : 130
	 */
	u8 Cmd;
	/*
	 * sizeof(struct shub_event_params)
	 */
	u8 Length;
	/*
	 * SENSOR_HANDLE_PROXIMITY : 8
	 * SENSOR_HANDLE_WAKE_UP_PROXIMITY : 58 (SensorHub use this)
	 */
	u16 HandleID;
	union {
		u8 udata[4];
		struct {
			s8 status;
			s8 type;
		};
	};
	union {
		/*
		 * For Proximity:
		 *   approaching : fdata[0] = 0x40a00000
		 *   away : fdata[0] = 0x0
		 */
		u32 fdata[3];
		struct {
			u32 x;
			u32 y;
			u32 z;
		};
		struct {
			u64 pedometer;
		};
		struct {
			u32 heart_rate;
		};
	};
	s64 timestamp;
};

enum SENSORHUB_FACE_STATUS {
    SENSORHUB_FACE_STATUS_NEAR = 0,
    SENSORHUB_FACE_STATUS_FAR,
    SENSORHUB_FACE_STATUS_MAX
};


struct sensorhub_setting {
    /* show ==  far or near */
    int(*get_prox_active_stat)(void);
    /* DEVICE_ATTR(enable) show ==  switch */
    int(*get_prox_enble_stat)(void);
    /*  DEVICE_ATTR(enable) store == PLS attr enable */
    int(*sensorhub_prox_ctrl)(int enable);
    const bool use_wakelock ;
    struct input_dev *input;
    char * const ic_name ;
};



struct do_sensorhub
{
    /* === input_report_abs(input_dev, ABS_DISTANCE, !PROXIMITY_STATE); ==== */
    void (*prox_reprot_func)(struct input_dev *dev, enum SENSORHUB_FACE_STATUS status);
    /* ==== probe success do init ====*/
    void (*sensorhub_init)(void);
};

enum SENHUB_PROX_DATA_T {
    TYPE_SENSORHUB_SETTING = 0
};

FUNC_EXT struct do_sensorhub* tp_hub_sensorhub_do_funcList(enum SENHUB_PROX_DATA_T data_type , void *data );

#endif  /* REVO_SENSORHUB_H */
