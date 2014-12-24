/*
 * cy8ctst242.h -- cy8ctst242 touch driver core interface
 *
 * Copyright (C) 2012 Liu Yang
 * All rights reserved.
 *
 */

#ifndef __CY8CTST242__
#define __CY8CTST242__

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>
//#include <linux/debug_control.h>
#include <linux/dma-mapping.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/leds-mt65xx.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
#include "tpd.h"
#include <cust_eint.h>
#include "cust_gpio_usage.h"

/*#if defined(MT6577)
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#include <mach/mt6577_pm_ldo.h>
#elif defined(MT6575)
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#include <mach/mt6575_pm_ldo.h>
#else
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_boot.h>
#include <mach/mt6573_pll.h>
#endif
*/
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#define TPD_OK	0
#define TPD_DISTANCE_LIMIT	100
#define TPD_REG_BASE	0x00
#define TPD_SOFT_RESET_MODE	0x01
#define TPD_OP_MODE 0x00
#define TPD_LOW_PWR_MODE 0x04
#define TPD_SYSINFO_MODE 0x10
#define TPD_TYPE_CAPACITIVE
#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON
#define TPD_YMAX			540
#define TPD_BUTTON_HEIGHT	20
#define TPD_Y_OFFSET		2
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_BUTTON_SIZE_HEIGHT  (TPD_YMAX - TPD_BUTTON_HEIGHT - TPD_Y_OFFSET)+10
#define TPD_BUTTON_Y_CENTER   	(TPD_BUTTON_HEIGHT + (TPD_YMAX - TPD_BUTTON_HEIGHT)/2 + TPD_Y_OFFSET)


#define TPD_B1_FP	13		//Button 1 pad space
#define TPD_B1_W	93		//Button 1 Width
#define TPD_B2_FP	13		//Button 2 pad space
#define TPD_B2_W	93		//Button 2 Width
#define TPD_B3_FP	13		//Button 3 pad space
#define TPD_B3_W	93		//Button 3 Width

#define TPD_BUTTON1_X_CENTER	TPD_B1_FP + TPD_B1_W/2
#define TPD_BUTTON2_X_CENTER	TPD_B1_FP + TPD_B1_W + TPD_B2_FP + TPD_B2_W/2 + 1
#define TPD_BUTTON3_X_CENTER	TPD_B1_FP + TPD_B1_W + TPD_B2_FP + TPD_B2_W + TPD_B3_FP + TPD_B3_W/2 + 1
/*#define TPD_KEYS_DIM            {{TPD_BUTTON1_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B1_W, TPD_BUTTON_SIZE_HEIGHT},	\
				 			{TPD_BUTTON2_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B2_W, TPD_BUTTON_SIZE_HEIGHT},	\
							{TPD_BUTTON3_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B3_W, TPD_BUTTON_SIZE_HEIGHT}}
*/
#define TPD_KEYS_DIM            {{TPD_BUTTON1_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B1_W+TPD_B1_FP, TPD_BUTTON_SIZE_HEIGHT},	\
				 			{TPD_BUTTON2_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B2_W+TPD_B1_FP, TPD_BUTTON_SIZE_HEIGHT},	\
							{TPD_BUTTON3_X_CENTER, TPD_BUTTON_Y_CENTER, TPD_B3_W+TPD_B1_FP, TPD_BUTTON_SIZE_HEIGHT}}
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
extern void tpd_button(unsigned int x, unsigned int y, unsigned int down) ;
#endif
//#define mt6575_touch_info printk
//#define mt6575_touch_debug printk
#define mt6575_touch_info(format, args...)  do {} while (0)
#define mt6575_touch_debug(format, args...)  do {} while (0)
void cy8ctst_power(int on);
void cy8ctst_reset(void);
int cy8ctst_read_reg(uint8_t reg, uint8_t *value);
int cy8ctst_read_reg_ext(u8 addr, unsigned char *pdata);
int cy8ctst_write_reg(uint8_t reg, uint8_t *value);
int cy8ctst_write_reg_ext(u8 addr, u8 para);
int cy8ctst_read_block(uint8_t reg, uint8_t *rxbuf, int length);
int cy8ctst_read_block_ext(uint8_t addr, uint8_t *rxbuf, int len);
int cy8ctst_write_block_simple(uint8_t reg, uint8_t *txbuf, int length);
#if defined(MT6575) || defined(MT6577)
int cy8ctst_write_block(uint8_t *txbuf, int length);
#elif defined(MT6573)
ssize_t cy8ctst_dma_read_block(uint8_t reg, uint8_t *rxbuf, int length);
int cy8ctst_dma_write_block(u32 pa, int length);
#endif
int cy8ctst_data_toggle(void);
int cy8ctst_to_zero(void);
int cy8ctst_bootloader_info (int show);
void cy8ctst_exit_bootloader(void);
int cy8ctst_swtich_mode(uint8_t mode);
int cy8ctst_get_vendor_fw(void);
int cy8ctst_get_fw(void);
#ifdef CY8CTST242_USE_ALLOC_MEM
int cy8ctst_mem_alloc(void);
int cy8ctst_mem_free(void);
#endif
int cy8ctst_dump_cmd_list(int num);
int cy8ctst_read_i2c_bin(const char *filename);
int cy8ctst_update(void);
int cy8ctst_get_blk_data(void);
int cy8ctst_dump_blk_info(void);
int cy8ctst_block_update(void);
int cy8ctst_open_sersor(int on);
int cy8ctst_get_sensor_state(void);
int cy8ctst_irq_enable(int on);
extern kal_bool upmu_is_chr_det(void);
#endif
