#ifndef __LINUX_FT5306_TS_H__
#define __LINUX_FT5306_TS_H__
#include "prj/prj_config.h"
#include "tp_common.h"

#ifdef SCREEN_PHY_WIDTH
static unsigned short SCREEN_MAX_X = (unsigned short)SCREEN_PHY_WIDTH ;
#else
static unsigned short SCREEN_MAX_X = 800 ;
#endif
#ifdef SCREEN_PHY_HIGHT
static unsigned short SCREEN_MAX_Y = (unsigned short)SCREEN_PHY_HIGHT ;
#else
static unsigned short SCREEN_MAX_Y = 1280 ;
#endif


#define PRESS_MAX       255

#define INPUT_DEV_NAME			"currency_tp"
#define FT5306_TS_NAME			 "ft5306_ts"
#define FT5306_VIRTUAL_KEY	 "virtualkeys.currency_tp"
#define FT5306_TS_ADDR			0x38

enum ft5306_ts_regs {
	FT5306_REG_THGROUP					= 0x80,
	FT5306_REG_THPEAK						= 0x81,
	FT5306_REG_THCAL						= 0x82,
	FT5306_REG_THWATER					= 0x83,
	FT5306_REG_THTEMP					= 0x84,
	FT5306_REG_THDIFF						= 0x85,
	FT5306_REG_CTRL						= 0x86,
	FT5306_REG_TIMEENTERMONITOR			= 0x87,
	FT5306_REG_PERIODACTIVE				= 0x88,
	FT5306_REG_PERIODMONITOR			= 0x89,
	FT5306_REG_HEIGHT_B					= 0x8a,
	FT5306_REG_MAX_FRAME					= 0x8b,
	FT5306_REG_DIST_MOVE					= 0x8c,
	FT5306_REG_DIST_POINT				= 0x8d,
	FT5306_REG_FEG_FRAME					= 0x8e,
	FT5306_REG_SINGLE_CLICK_OFFSET		= 0x8f,
	FT5306_REG_DOUBLE_CLICK_TIME_MIN	= 0x90,
	FT5306_REG_SINGLE_CLICK_TIME			= 0x91,
	FT5306_REG_LEFT_RIGHT_OFFSET		= 0x92,
	FT5306_REG_UP_DOWN_OFFSET			= 0x93,
	FT5306_REG_DISTANCE_LEFT_RIGHT		= 0x94,
	FT5306_REG_DISTANCE_UP_DOWN		= 0x95,
	FT5306_REG_ZOOM_DIS_SQR				= 0x96,
	FT5306_REG_RADIAN_VALUE				=0x97,
	FT5306_REG_MAX_X_HIGH                       	= 0x98,
	FT5306_REG_MAX_X_LOW             			= 0x99,
	FT5306_REG_MAX_Y_HIGH            			= 0x9a,
	FT5306_REG_MAX_Y_LOW             			= 0x9b,
	FT5306_REG_K_X_HIGH            			= 0x9c,
	FT5306_REG_K_X_LOW             			= 0x9d,
	FT5306_REG_K_Y_HIGH            			= 0x9e,
	FT5306_REG_K_Y_LOW             			= 0x9f,
	FT5306_REG_AUTO_CLB_MODE			= 0xa0,
	FT5306_REG_LIB_VERSION_H 				= 0xa1,
	FT5306_REG_LIB_VERSION_L 				= 0xa2,
	FT5306_REG_CIPHER						= 0xa3,
	FT5306_REG_MODE						= 0xa4,
	FT5306_REG_PMODE						= 0xa5,	/* Power Consume Mode */
	FT5306_REG_FIRMID						= 0xa6,
	FT5306_REG_STATE						= 0xa7,
	FT5306_REG_FT5201ID					= 0xa8,
	FT5306_REG_ERR						= 0xa9,
	FT5306_REG_CLB						= 0xaa,
};

//FT5306_REG_PMODE
#define PMODE_ACTIVE        0x00
#define PMODE_MONITOR       0x01
#define PMODE_STANDBY       0x02
#define PMODE_HIBERNATE     0x03

#define FTP_GPIO_OUTPUT(pin,level)      {clear_bit(FLAG_USED_AS_IRQ, &gpio_to_desc(pin)->flags);gpio_direction_output(pin,level);}

/*register address*/
#define FT_REG_CHIP_ID				0xA3    //chip ID
#define FT_REG_FW_VER				0xA6   //FW  version
#define FT_REG_VENDOR_ID			0xA8   // TP vendor ID
#define TPD_MAX_POINTS_2                        2
#define TPD_MAX_POINTS_5                        5
#define TPD_MAXPOINTS_10                        10
#define AUTO_CLB_NEED                              1
#define AUTO_CLB_NONEED                          0

#ifndef ABS_MT_TOUCH_MAJOR
#define ABS_MT_TOUCH_MAJOR	0x30	/* touching ellipse */
#define ABS_MT_TOUCH_MINOR	0x31	/* (omit if circular) */
#define ABS_MT_WIDTH_MAJOR	0x32	/* approaching ellipse */
#define ABS_MT_WIDTH_MINOR	0x33	/* (omit if circular) */
#define ABS_MT_ORIENTATION	0x34	/* Ellipse orientation */
#define ABS_MT_POSITION_X	0x35	/* Center X ellipse position */
#define ABS_MT_POSITION_Y	0x36	/* Center Y ellipse position */
#define ABS_MT_TOOL_TYPE	0x37	/* Type of touching device */
#define ABS_MT_BLOB_ID		0x38	/* Group set of pkts as blob */
#define ABS_MT_PRESSURE		0x3a	/* Pressure on contact area */
#define ABS_MT_TRACKING_ID      0x39	/* Unique ID of initiated contact */
#endif /* ABS_MT_TOUCH_MAJOR */

struct Upgrade_Info {
    u8 CHIP_ID;
    u8 FTS_NAME[20];
    u8 TPD_MAX_POINTS;
    u8 AUTO_CLB;
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};
#endif
