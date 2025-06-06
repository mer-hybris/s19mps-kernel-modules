/*android / kernel / msm / android - msm - bullhead - 3.10 - marshmallow - dr / . / include / linux / sensors.h*/

/*blob: 56a0d7314245bc10ef2526542ea84cd81d876191 [file] [log] [blame] */

/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 * * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 * * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#ifndef __LINUX_SENSORS_H_INCLUDED
#define __LINUX_SENSORS_H_INCLUDED
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>

#define SENSORS_ACCELERATION_HANDLE  0
#define SENSORS_MAGNETIC_FIELD_HANDLE  1
#define SENSORS_ORIENTATION_HANDLE  2
#define SENSORS_LIGHT_HANDLE   3
#define SENSORS_PROXIMITY_HANDLE  4
#define SENSORS_GYROSCOPE_HANDLE  5
#define SENSORS_PRESSURE_HANDLE   6
#define SENSOR_TYPE_ACCELEROMETER  1
#define SENSOR_TYPE_GEOMAGNETIC_FIELD  2
#define SENSOR_TYPE_MAGNETIC_FIELD SENSOR_TYPE_GEOMAGNETIC_FIELD
#define SENSOR_TYPE_ORIENTATION   3
#define SENSOR_TYPE_GYROSCOPE   4
#define SENSOR_TYPE_LIGHT   5
#define SENSOR_TYPE_PRESSURE   6
#define SENSOR_TYPE_TEMPERATURE   7
#define SENSOR_TYPE_PROXIMITY   8
#define SENSOR_TYPE_GRAVITY   9
#define SENSOR_TYPE_LINEAR_ACCELERATION  10
#define SENSOR_TYPE_ROTATION_VECTOR  11
#define SENSOR_TYPE_RELATIVE_HUMIDITY  12
#define SENSOR_TYPE_AMBIENT_TEMPERATURE  13
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED 14
#define SENSOR_TYPE_GAME_ROTATION_VECTOR 15
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED 16
#define SENSOR_TYPE_SIGNIFICANT_MOTION  17
#define SENSOR_TYPE_STEP_DETECTOR  18
#define SENSOR_TYPE_STEP_COUNTER  19
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR 20
#define SENSOR_TYPE_CAPSENSE 21 //yasin: added

enum LIS3DH_AXIS {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_XYZ,
};

enum LIS3DH_THRES {
    AXIS_THRESHOLD_H = 0,
    AXIS_THRESHOLD_L,
    AXIS_BIAS,
};

#define AXIS_FACTOR  0
#define AXIS_OFFSET  1

struct cal_result_t {
    union {
        struct {
            int offset_x; /*axis offset of x axis*/
            int offset_y; /*axis offset of x axis*/
            int offset_z; /*axis offset of x axis*/
        };
        struct {
            int threshold_h; /*proximity threshold_h*/
            int threshold_l; /*proximity threshold_l*/
            int bias; /*proximity measure data noise*/
        };
        int offset[3];
    };
    int factor; /*light sensor factor for real ligt strength*/
    int range;
    struct cal_result_t *node;
};

/**
 * struct sensors_classdev - hold the sensor general parameters and APIs
 * @dev:  The device to register.
 * @node:  The list for the all the sensor drivers.
 * @name:  Name of this sensor.
 * @vendor:  The vendor of the hardware part.
 * @handle:  The handle that identifies this sensors.
 * @type:  The sensor type.
 * @max_range:  The maximum range of this sensor's value in SI units.
 * @resolution:  The smallest difference between two values reported by
 *   this sensor.
 * @sensor_power: The rough estimate of this sensor's power consumption
 *   in mA.
 * @min_delay:  This value depends on the trigger mode:
 *   continuous: minimum period allowed in microseconds
 *   on-change : 0
 *   one-shot :-1
 *   special : 0, unless otherwise noted
 * @fifo_reserved_event_count: The number of events reserved for this sensor
 *    in the batch mode FIFO.
 * @fifo_max_event_count: The maximum number of events of this sensor
 *    that could be batched.
 * @max_delay:  The slowest rate the sensor supports in millisecond.
 * @flags:  Should be '1' if the sensor is a wake up sensor.
 *   set it to '0' otherwise.
 * @enabled:  Store the sensor driver enable status.
 * @delay_msec:  Store the sensor driver delay value. The data unit is
 *   millisecond.
 * @wakeup:  Indicate if the wake up interrupt has been enabled.
 * @max_latency: Max report latency in millisecond
 * @sensors_enable: The handle for enable and disable sensor.
 * @sensors_poll_delay: The handle for set the sensor polling delay time.
 * @sensors_set_latency:Set the max report latency of the sensor.
 * @sensors_flush: Flush sensor events in FIFO and report it to user space.
 * @params  The sensor calibrate string format params up to userspace.
 * @cal_result  The sensor calibrate parameters, cal_result is a struct for sensor.
 */

struct sensors_classdev {
    struct device  *dev;
    struct list_head node;
    const char  *name;
    const char  *vendor;
    int   version;
    int   handle;
    int   type;
    const char  *max_range;
    const char  *resolution;
    const char  *sensor_power;
    int   min_delay;
    int   fifo_reserved_event_count;
    int   fifo_max_event_count;
    int32_t   max_delay;
    uint32_t  flags;
    unsigned int  enabled;
    unsigned int  delay_msec;
    unsigned int  wakeup;
    unsigned int  max_latency;
    char   *params;
    struct cal_result_t cal_result; /* enable and disable the sensor handle*/
    int (*sensors_enable)(struct sensors_classdev *sensors_cdev, unsigned int enabled);
    int (*sensors_poll_delay)(struct sensors_classdev *sensors_cdev, unsigned int delay_msec);
    int (*sensors_self_test)(struct sensors_classdev *sensors_cdev);
    int (*sensors_set_latency)(struct sensors_classdev *sensor_cdev, unsigned int max_latency);
    int (*sensors_enable_wakeup)(struct sensors_classdev *sensor_cdev, unsigned int enable);
    int (*sensors_flush)(struct sensors_classdev *sensors_cdev);
    int (*sensors_calibrate)(struct sensors_classdev *sensor_cdev, int axis, int apply_now);
    int (*sensors_write_cal_params)(struct sensors_classdev *sensor_cdev, struct cal_result_t *cal_result);
};
extern int sensors_classdev_register(struct device *parent, struct sensors_classdev *sensors_cdev);
extern void sensors_classdev_unregister(struct sensors_classdev *sensors_cdev);
#endif  /* __LINUX_SENSORS_H_INCLUDED */
