#ifndef __FOCALTECH_EX_FUN_H__
#define __FOCALTECH_EX_FUN_H__

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>

#define FT_UPGRADE_AA					0xAA
#define FT_UPGRADE_55 					0x55
#define FTS_UPGRADE_AA                             0xAA
#define FTS_UPGRADE_55 				0x55
#define FTS_REG_ECC		                            0xCC
#define FTS_RST_CMD_REG2		              0xBC
#define FTS_READ_ID_REG		              0x90
#define FTS_ERASE_APP_REG	                     0x61
#define FTS_ERASE_PARAMS_CMD	              0x63
#define FTS_FW_WRITE_CMD		              0xBF



/*****************************************************************************/
#define FTS_PAGE_SIZE                                        128
#define FTS_PACKET_LENGTH        		       128
#define FTS_SETTING_BUF_LEN      		128
#define FTS_DMA_BUF_SIZE 				1024

#define FTS_UPGRADE_LOOP				30

#define FTS_FACTORYMODE_VALUE			0x40
#define FTS_WORKMODE_VALUE			0x00

/*create sysfs for debug*/
int fts_create_sysfs(struct i2c_client * client);

void fts_release_sysfs(struct i2c_client * client);
int ft5x0x_create_apk_debug_channel(struct i2c_client *client);
void ft5x0x_release_apk_debug_channel(void);
int fts_ctpm_auto_upgrade(struct i2c_client *client);

/*
*fts_write_reg- write register
*@client: handle of i2c
*@regaddr: register address
*@regvalue: register value
*
*/

int fts_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);

int fts_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

int fts_write_reg(struct i2c_client * client,u8 regaddr, u8 regvalue);

int fts_read_reg(struct i2c_client * client,u8 regaddr, u8 *regvalue);

#endif
