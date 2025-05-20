/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *
 */
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include "tlsc6x_mmbin.h"

static unsigned int g_tlsc6x_cfg_ver = 0;
static unsigned int g_tlsc6x_keepbin = 0;

static struct i2c_client *g_tlsc6x_client;

// Telink CTP
typedef struct __test_cmd_wr{
    //offset 0;
    unsigned char id; //cmd_id;
    unsigned char idv; //inverse of cmd_id
    unsigned short d0; //data 0
    unsigned short d1; //data 1
    unsigned short d2; //data 2
    //offset 8;
    unsigned char resv;  //offset 8
    unsigned char tag;   //offset 9
    unsigned short chk;  // 16 bit checksum
    unsigned short s2Pad0;  //
    unsigned short s2Pad1;  //
}ctp_test_wr_t;

typedef struct __test_cmd_rd{
    //offset 0;
    unsigned char id; //cmd_id;
    unsigned char cc; //complete code
    unsigned short d0; //data 0
    unsigned short sn; //session number
    unsigned short chk;  // 16 bit checksum
}ctp_test_rd_t;
#define DIRECTLY_MODE   (0x0)
#define DEDICATE_MODE   (0x1)
#define MAX_TRX_LEN (64)   // max IIC data length
#define CMD_ADDR    (0xb400)
#define RSP_ADDR    (0xb440)
#define MTK_TXRX_BUF    (0xcc00)  // 1k, buffer used for memory r &w

#define LEN_CMD_CHK_TX  (10)
#define LEN_CMD_PKG_TX  (16)

#define LEN_RSP_CHK_RX  (8)
#define MAX_BULK_SIZE    (1024)

static unsigned char cmd_2dma_42bd[6]={/*0x42, 0xbd, */0x28, 0x35, 0xc1, 0x00, 0x35, 0xae};  // to direct memory access mode


// in directly memory access mode
// RETURN:0->pass else->fail
static int tlsc6x_read_bytes_u16addr(struct i2c_client *client, u16 addr, u8 *rxbuf, u16 len)
{
    int err = 0;
    int retry = 0;
    u16 offset = 0;
    u8 buffer[2];

    struct i2c_msg msgs[2] ={
        {
            .addr = client->addr,
            .flags = 0,
            .len = 2,   // 16bit memory address
            .buf = buffer,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
        },
    };

    if(NULL == rxbuf){
        return -1;
    }

    //mutex_lock(&g_mutex_i2c_access);

    while(len > 0){
        buffer[0] = (u8)((addr+offset)>>8);
        buffer[1] = (u8)(addr+offset);

        msgs[1].buf = &rxbuf[offset];
        if(len > MAX_TRX_LEN){
            len -= MAX_TRX_LEN;
            msgs[1].len = MAX_TRX_LEN;
        }else{
            msgs[1].len = len;
            len = 0;
        }

        retry = 0;
        while(i2c_transfer(client->adapter, &msgs[0], 2) != 2){
            if(retry++ == 3){
                err = -1;
                break;
            }
        }
        offset += MAX_TRX_LEN;
        if(err < 0){
            break;
        }
    }

    //mutex_unlock(&g_mutex_i2c_access);

    return err;
}

// in directly memory access mode
// RETURN:0->pass else->fail
static int tlsc6x_write_bytes_u16addr(struct i2c_client *client, u16 addr, u8 *txbuf, u16 len)
{
    u8 buffer[MAX_TRX_LEN];
    u16 offset = 0;
    u8 retry = 0;
    int err = 0;

    struct i2c_msg msg ={
        .addr = client->addr,
        .flags = 0,
        .buf = buffer,
    };

    if(NULL == txbuf){
        return -1;
    }

    //mutex_lock(&g_mutex_i2c_access);

    while(len){
        buffer[0] = (u8)((addr+offset)>>8);
        buffer[1] = (u8)(addr+offset);

        if(len > (MAX_TRX_LEN-2)){ // (sizeof(addr)+payload) <= MAX_TRX_LEN
            memcpy(&buffer[2], &txbuf[offset], (MAX_TRX_LEN-2));
            len -= (MAX_TRX_LEN-2);
            offset += (MAX_TRX_LEN-2);
            msg.len = MAX_TRX_LEN;
        }else{
            memcpy(&buffer[2], &txbuf[offset], len);
            msg.len = len + 2;
            len = 0;
        }

        retry = 0;
        while(i2c_transfer(client->adapter, &msg, 1) != 1){
            if(retry++ == 3){
                err = -1;
                break;
            }
        }
        if(err < 0){
            break;
        }
    }

    //mutex_unlock(&g_mutex_i2c_access);

    return err;
}


// <0 : i2c error
// 0: direct address mode
//// 1: protect mode
 static int tlsc6x_get_i2cmode(void)
{
    u8 regData[4];

    if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0x01, regData, 3)){
        return -1;
    }
    if(((regData[0]>>1) == (u8)(g_tlsc6x_client->addr)) && (0x01 == regData[2])){
        tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0x28, regData, 2);
        if((0x10==regData[0]) && (0xdf==regData[1])){
            return DIRECTLY_MODE;
        }
    }

    return DEDICATE_MODE;
}

// 0:successful
static int tlsc6x_set_dd_mode(void)
{    
    int mod = -1;
    int retry = 0;

    if(DIRECTLY_MODE == tlsc6x_get_i2cmode()){
        return 0;
    }
    
    while(retry++ < 5){
        tlsc6x_write_bytes_u16addr(g_tlsc6x_client, 0x42bd, cmd_2dma_42bd, 6);
        msleep(30);
        mod = tlsc6x_get_i2cmode();
        if(DIRECTLY_MODE == mod){
            break;
        }
    }

    if(DIRECTLY_MODE == mod){
        return 0;
    }else{
        return -1;
    }
}

// 0:successful
static int tlsc6x_set_nor_mode(void)
{    
    int mod = -1;
    int retry = 0;
    u8 reg = 0x05;
    
    while(retry++ < 5){
        tlsc6x_write_bytes_u16addr(g_tlsc6x_client, 0x03, &reg, 1);
        msleep(5);
        mod = tlsc6x_get_i2cmode();
        if(DEDICATE_MODE == mod){
            break;
        }
        msleep(50);
    }
    if(DEDICATE_MODE == mod){
        return 0;
    }else{
        return -1;
    }
}
// ret=0 : successful
// write with read-back check, in dd mode
static int tlsc6x_bulk_down_check(u8* pbuf, u16 addr, u16 len)
{
    unsigned int j, k, retry;
    u8 rback[128];

    while(len){
        k = (len<128)?len:128;
        retry = 0;
        do{
            rback[k-1] = pbuf[k-1]+1;
            tlsc6x_write_bytes_u16addr(g_tlsc6x_client, addr, pbuf, k);
            tlsc6x_read_bytes_u16addr(g_tlsc6x_client, addr, rback, k);
            for(j=0; j<k; j++){
                if(pbuf[j] != rback[j]){
                    break;
                }
            }
            if(j >= k){
                break;  // match
            }
        } while(++retry < 3);
        
        if(j < k){
            break;
        }
        
        addr += k;
        pbuf += k;
        len -= k;
    }
    
    return (int)len;
}

// 0:successful
static int tlsc6x_load_rambin(u8* pcode, u16 len)
{
    u8 dwr, retry;
    int ret = -2;

    if(tlsc6x_set_dd_mode()){
        return -1;
    }

    dwr = 0x05;
    if(0 == tlsc6x_bulk_down_check(&dwr, 0x0602, 1)){   // stop mcu
        dwr = 0x00;
        tlsc6x_bulk_down_check(&dwr, 0x0643, 1);            //  disable irq
    }else{
        return -1;
    }
    if(0 == tlsc6x_bulk_down_check(pcode, 0x8000, len)){
        dwr = 0x88;
        retry = 0; 
        do{
            ret = tlsc6x_write_bytes_u16addr(g_tlsc6x_client, 0x0602, &dwr, 1);
        }while((++retry < 3) && (0 != ret));
    }

    //msleep(50);   // let caller decide the delay time
    
    return ret;
}

// NOT a public function, only one caller!!!
static void tlsc6x_tp_cfg_version(void)
{
    unsigned int tmp;

    if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0xd6e0, (u8*)&tmp, 4)){
        return;
    }

    g_tlsc6x_cfg_ver = tmp;
}

static int tlsc6x_mmbin_newif(void)
{
    int ret = 0;
    unsigned int chkdata;

    if(tlsc6x_set_dd_mode()){
        goto exit;
    }

    chkdata = 0;
    if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0x8004, (u8*)&chkdata, 4)){
        goto exit;
    }
    if(0x3d2c020c != chkdata){
        goto exit;
    }

    chkdata = 0;
    if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0xbd1c, (u8*)&chkdata, 4)){
        goto exit;
    }
    if(0x22212024 != chkdata){
        goto exit;
    }

    if(tlsc6x_load_rambin(ft5036_do_tlsc6x_mmbin, sizeof(ft5036_do_tlsc6x_mmbin))){
        ret = -1;
    }else{
        ret = 1;
    }
exit:
    tlsc6x_set_nor_mode();

    return ret;
}
// 1:telink module
static int tlsc6x_tp_dect(struct i2c_client * client)
{
    g_tlsc6x_client = client;
	
    if(tlsc6x_set_dd_mode()){
        return 0;
    }

    tlsc6x_tp_cfg_version();    // MUST: call this function there!!!

    if(1 == tlsc6x_mmbin_newif()){
        g_tlsc6x_keepbin = 1;
    }

    tlsc6x_set_nor_mode();

    return 1;
}



/*  *********************************************
         revo  ft5306 add tlsc6x 
********************************************** */
int ft5306_do_tlsc6x_tp_dect(struct i2c_client * client){
	return tlsc6x_tp_dect(client);
}

int ft5306_do_tlsc6x_mmbin_newif(void){
	return tlsc6x_mmbin_newif();
}

unsigned int ft5306_do_get_tlsc6x_keepbin(void){
	return g_tlsc6x_keepbin ;
}