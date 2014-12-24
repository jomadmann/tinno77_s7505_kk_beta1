/*
 * cy8ctst242.c -- cy8ctst242 touch driver
 *
 * Copyright (C) 2012 Li Yang
 * All rights reserved.
 *
 */
#include "cy8ctst242.h"

extern struct tpd_device *tpd;
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct early_suspend early_suspend;
#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
        kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
        kal_bool auto_umask);
#endif
static void tpd_eint_interrupt_handler(void);

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
int g_bcy8stmstar = true;
//#define CY8CTST_BOOTLOADER
#define CY8CTST_SENSOR
//#define CY8CTST_DEBUG
#define GET_HSTMODE(reg)  ((reg & 0x70) >> 4)  // in op mode or not 
#define GET_BOOTLOADERMODE(reg) ((reg & 0x10) >> 4)  // in bl mode 
#define CY8CTST_VENDOR_REG 		0x10
#define CY8CTST_FW_REG 			0x11
#define CY8CTST_SENSOR_EN_REG 	0x12
#define CY8CTST_SENSOR_STATE_REG 	0x13
static int tpd_flag = 0;
static int boot_mode;
static int binit = 0;
extern int updating;

struct i2c_client_ctxt{
	 struct i2c_client *client;
	 struct mutex lock;
	 struct mutex power_lock;
};

static struct i2c_client_ctxt *_cy8ctst_client;

static uint8_t bootloader_exit_cmd[] = {
	0x00, 0x00, 0xFF, 0xA5, 
	0x00, 0x01, 0x02, 0x03, 
	0x04, 0x05, 0x06, 0x07
};

struct tpd_operation_data_t{
	uint8_t hst_mode; //0x00
	uint8_t tt_mode;
	uint8_t tt_stat;
	uint8_t x1_H;
	uint8_t x1_L;
	uint8_t y1_H;
	uint8_t y1_L;
	uint8_t z1;
	uint8_t touch12_id; //0x08
	uint8_t x2_H;
	uint8_t x2_L;
	uint8_t y2_H;
	uint8_t y2_L;
	uint8_t z2;
	uint8_t gest_cnt;
	uint8_t gest_id;
	uint8_t gest_set; //0x10
	uint8_t res1;
	uint8_t sensor_en; //0x12
	uint8_t sensor_state; //0x13
#ifdef CY8CTST_DEBUG
	uint8_t diff_L; //0x14
	uint8_t diff_H;
	uint8_t rawdata_L; //0x16
	uint8_t rawdata_H;
	uint8_t baseline_L; //0x18
	uint8_t baseline_H;
#endif
};

struct tpd_bootloader_data_t{
    uint8_t bl_file;
    uint8_t bl_status;
    uint8_t bl_error;
    uint8_t blver_hi,blver_lo;
    uint8_t bld_blver_hi,bld_blver_lo;

    uint8_t ttspver_hi,ttspver_lo;
    uint8_t appid_hi,appid_lo;
    uint8_t appver_hi,appver_lo;

    uint8_t cid_0;
    uint8_t cid_1;
    uint8_t cid_2;

};

struct tpd_sysinfo_data_t{
    uint8_t   hst_mode;
    uint8_t  mfg_cmd;
    uint8_t  mfg_stat;
    uint8_t cid[3];
    uint8_t tt_undef1;

    uint8_t uid[8];
    uint8_t  bl_verh;
    uint8_t  bl_verl;

    uint8_t tts_verh;
    uint8_t tts_verl;

    uint8_t app_idh;
    uint8_t app_idl;
    uint8_t app_verh;
    uint8_t app_verl;

    uint8_t tt_undef2[6];
    uint8_t  act_intrvl;
    uint8_t  tch_tmout;
    uint8_t  lp_intrvl;	 

};

struct touch_info {
    int x1, y1;
    int x2, y2;
    int p1, p2;
    int id1,id2;
    int count;
};

struct id_info{
    int pid1;
    int pid2;
    int reportid1;
    int reportid2;
    int id1;
    int id2;

};

struct cy8ctst_i2c_cmd 
{
	uint8_t opration;
	uint8_t addr;
	uint8_t reg;
	uint8_t cmd_buf[20];
	int cmd_length;
	int delay_time;
};

struct i2c_blk_info {
	uint8_t blk_buf[80];
	uint8_t blk_length;
};

#define MAX_READ_BYTE		8
#define I2C_UNKNOW_CMD		0x0f
#define I2C_READ_CMD		0
#define I2C_WRITE_CMD		1
#define I2C_DELAY_CMD		2
#define I2C_BIN_SIZE			200 * 1024
#ifdef CY8CTST242_USE_ALLOC_MEM
static uint8_t *i2c_bin_buf;
static struct cy8ctst_i2c_cmd *i2c_cmd_list;
#else
static uint8_t i2c_bin_buf[I2C_BIN_SIZE];
static struct cy8ctst_i2c_cmd i2c_cmd_list[50 * 1024];
#endif
static int total_cmd_num = 0;
static int process_cmd = 0;
static int sensor_open = 0;
static struct i2c_blk_info blk_list[800];
static int total_blk_num;

#if defined(MT6573)
static uint8_t *cy8ctst_dma_va = NULL;
static u32 cy8ctst_dma_pa = 0;
#endif

static struct tpd_operation_data_t g_operation_data;
static struct tpd_bootloader_data_t g_bootloader_data;
//static struct tpd_sysinfo_data_t g_sysinfo_data;

/*static const struct i2c_device_id tpd_id[] = {{TPD_DEVICE,0},{}};
static unsigned short force[] = {0,0x48,I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
static struct i2c_client_address_data addr_data = { .forces = forces, };
*/
#define DRIVER_NAME "cy8ctst242"
#define TPD_I2C_SLAVE_ADDR1 (0x48 >> 1)
#define TPD_I2C_GROUP_ID 0
static const struct i2c_device_id tpd_id[] = {{DRIVER_NAME,0},{}};
static struct i2c_board_info __initdata cy8ctst242_i2c_tpd[]={ {I2C_BOARD_INFO(DRIVER_NAME, TPD_I2C_SLAVE_ADDR1)}};
static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .name = TPD_DEVICE,
        .owner = THIS_MODULE,
    },
    .probe = tpd_probe,
    .remove = __devexit_p(tpd_remove),
    .id_table = tpd_id,
    .detect = tpd_detect,
 //   .address_data = &addr_data,
};

void cy8ctst_power(int on)
{
	mt6575_touch_info("+%s: on=%d--Li\n", __func__, on);
	if(on) {
	
	#if defined(MT6575) || defined(MT6577)
		hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_1800, "touch"); 
	#else
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	#endif 
		msleep(10);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT); 
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		udelay(300);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
		msleep(30);
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	}
	else {
	#if defined(MT6575) || defined(MT6577)
		hwPowerDown(MT65XX_POWER_LDO_VGP, "touch"); 
	#else
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_IN);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	#endif
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_IN);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EINT_PIN, GPIO_OUT_ZERO);
	}
	mt6575_touch_info("-%s: on=%d--Li\n", __func__, on);
}

static void cy8ctst_i2c_gpio(void) 
{
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_mode(GPIO_I2C0_SCA_PIN, GPIO_I2C0_SCA_PIN_M_GPIO);
    mt_set_gpio_mode(GPIO_I2C0_SDA_PIN, GPIO_I2C0_SDA_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_dir(GPIO_I2C0_SCA_PIN, GPIO_DIR_OUT);
    mt_set_gpio_dir(GPIO_I2C0_SDA_PIN, GPIO_DIR_OUT);

    mt_set_gpio_pull_enable(GPIO_CTP_RST_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_RST_PIN, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO_I2C0_SCA_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_I2C0_SCA_PIN, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(GPIO_I2C0_SDA_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_I2C0_SDA_PIN, GPIO_PULL_UP);

    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    mt_set_gpio_out(GPIO_I2C0_SCA_PIN, GPIO_OUT_ZERO);
    mt_set_gpio_out(GPIO_I2C0_SDA_PIN, GPIO_OUT_ZERO); 
}

static void cy8ctst_gpio_i2c(void) 
{
    mt_set_gpio_mode(GPIO_I2C0_SCA_PIN, GPIO_I2C0_SCA_PIN_M_SCL);
    mt_set_gpio_mode(GPIO_I2C0_SDA_PIN, GPIO_I2C0_SDA_PIN_M_SDA);
}

void cy8ctst_reset(void)
{
	mt6575_touch_info("%s:--Li\n", __func__);
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	mutex_lock(&cy8ctst_client->power_lock);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	cy8ctst_power(0);
	cy8ctst_i2c_gpio();
	msleep(10);  
	cy8ctst_power(1);
	cy8ctst_gpio_i2c();
	msleep(150);  
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mutex_unlock(&cy8ctst_client->power_lock);
}

int cy8ctst_read_reg(uint8_t reg, uint8_t *value)
{
	int ret = 0, iRetry = 5;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	if(cy8ctst_client->client) {
		mutex_lock(&cy8ctst_client->lock);
		while (iRetry) {
			ret = i2c_smbus_read_i2c_block_data(cy8ctst_client->client, reg, 1, value);
			if(ret < 0) {
				--iRetry;
				msleep(200);
				continue;
			}
			else {
				mt6575_touch_debug("%s: success, reg=0x%02x, value=0x%02x, iRetry=%d--Li\n", 
					__func__, reg, *value, iRetry);
				break;
			}	
		}
		if(!iRetry && (ret < 0)) {
			mt6575_touch_info("%s: error, ret=%d--Li\n", 
					__func__, ret);
			mutex_unlock(&cy8ctst_client->lock);
			return -1;
		}
		mutex_unlock(&cy8ctst_client->lock);
	}
	else {
		mt6575_touch_info("%s: cy8ctst_client->client is NULL--Li\n", __func__);
		return -1;
	}
	return ret;
	
}

int cy8ctst_read_reg_ext(u8 addr, unsigned char *pdata)
{
	int rc;
	unsigned char buf[2];
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	buf[0] = addr;     
	mutex_lock(&cy8ctst_client->lock);
	i2c_master_send(cy8ctst_client->client, &buf[0], 1);
	rc = i2c_master_recv(cy8ctst_client->client, &buf[0], 1);
	mutex_unlock(&cy8ctst_client->lock);
	if (rc < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, rc);
	*pdata = buf[0];
	
	return rc;
}

int cy8ctst_write_reg(uint8_t reg, uint8_t *value)
{
	int ret = 0, iRetry = 5;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	if(cy8ctst_client->client) {
		mutex_lock(&cy8ctst_client->lock);
		while (iRetry) {
			ret = i2c_smbus_write_i2c_block_data(cy8ctst_client->client, reg, 1, value);
			if(ret < 0) {
				--iRetry;
				msleep(200);
				continue;
			}
			else {
				mt6575_touch_debug("%s: success, reg=0x%02x, value=0x%02x, iRetry=%d--Li\n", 
					__func__, reg, *value, iRetry);
				break;
			}
		}
		if(!iRetry && (ret < 0)) {
			mt6575_touch_info("%s: error, ret=%d--Li\n", 
					__func__, ret);
			mutex_unlock(&cy8ctst_client->lock);
			return -1;
		}
		mutex_unlock(&cy8ctst_client->lock);
	}
	else {
		mt6575_touch_info("%s: cy8ctst_client->client is NULL--Li\n", __func__);
		return -1;
	}
	return ret;
	
}

int cy8ctst_write_reg_ext(u8 addr, u8 para)
{
	int rc;
	char buf[3];
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	buf[0] = addr;
	buf[1] = para;
	mutex_lock(&cy8ctst_client->lock);
	rc = i2c_master_send(cy8ctst_client->client, buf, 2);
	mutex_unlock(&cy8ctst_client->lock);
	return rc;
}

int cy8ctst_read_block(uint8_t reg, uint8_t *rxbuf, int length)
{
	int i, ret = 0;
	int multiple = length / MAX_READ_BYTE;
	int remainder = length % MAX_READ_BYTE;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	if(length <= 0) {
		mt6575_touch_info("%s: nothing to read--Li\n", __func__);
		return ret;
	}
	mutex_lock(&cy8ctst_client->lock);
	if(multiple > 0) {
		for(i = 0; i < multiple; i ++) {
			ret = i2c_smbus_read_i2c_block_data(cy8ctst_client->client, (reg + i * MAX_READ_BYTE), 
				MAX_READ_BYTE, & (rxbuf[MAX_READ_BYTE * i]));
			if(ret < 0) 
				goto I2C_ERROR;
		}
		if(remainder > 0) {
			ret = i2c_smbus_read_i2c_block_data(cy8ctst_client->client, (reg + i * MAX_READ_BYTE), 
				remainder, & (rxbuf[MAX_READ_BYTE * i]));
			if(ret < 0)
				goto I2C_ERROR;
		}
	}
	else {
		ret = i2c_smbus_read_i2c_block_data(cy8ctst_client->client, reg, 
			remainder, & (rxbuf[0]));
		if(ret < 0) 
			goto I2C_ERROR;
	}
	mutex_unlock(&cy8ctst_client->lock);
	return ret;
	
I2C_ERROR:
	mt6575_touch_info("%s: i2c error, ret=%d--Li\n", __func__, ret);
	mutex_unlock(&cy8ctst_client->lock);
	cy8ctst_reset();
	return ret;
}

#define MAX_I2C_TRANSFER_SIZE 6
int cy8ctst_read_block_ext(uint8_t addr, uint8_t *rxbuf, int len)
{
	u8 retry;
	u16 left = len;
	u16 offset = 0;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	if ( rxbuf == NULL )
		return -1;

	mt6575_touch_info("device 0x%02X address %04X len %d\n", cy8ctst_client->client->addr, addr, len );

	while ( left > 0 ){
		if ( left > MAX_I2C_TRANSFER_SIZE ){
			rxbuf[offset] = ( addr+offset ) & 0xFF;
			cy8ctst_client->client->addr = cy8ctst_client->client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;
			retry = 0;
			while ( i2c_master_send(cy8ctst_client->client, &rxbuf[offset], (MAX_I2C_TRANSFER_SIZE << 8 | 1)) < 0 ){
				retry++;

				if ( retry == 5 ){
					cy8ctst_client->client->addr = cy8ctst_client->client->addr & I2C_MASK_FLAG;
					mt6575_touch_info("I2C read 0x%X length=%d failed\n", addr + offset, MAX_I2C_TRANSFER_SIZE);
					return -1;
				}
			}
			left -= MAX_I2C_TRANSFER_SIZE;
			offset += MAX_I2C_TRANSFER_SIZE;
		}
		else{
			rxbuf[offset] = ( addr+offset ) & 0xFF;
			cy8ctst_client->client->addr = cy8ctst_client->client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;

			retry = 0;
			while ( i2c_master_send(cy8ctst_client->client, &rxbuf[offset], (left << 8 | 1)) < 0 ){
				retry++;

				if ( retry == 5 ){
					cy8ctst_client->client->addr = cy8ctst_client->client->addr & I2C_MASK_FLAG;
					mt6575_touch_info("I2C read 0x%X length=%d failed\n", addr + offset, left);
					return -1;
				}
			}
			offset += left;
			left = 0;
		}
	}
	cy8ctst_client->client->addr = cy8ctst_client->client->addr & I2C_MASK_FLAG;
	return 0;
}

int cy8ctst_write_block_simple(uint8_t reg, uint8_t *txbuf, int length)
{
	int i, ret = 0;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	for(i = 0; i < length; i ++) {
		ret = i2c_smbus_write_i2c_block_data(cy8ctst_client->client, reg + i, 1, & (txbuf[i]));
		if (ret)
			goto I2C_ERROR;
	}	
	return ret;
	
I2C_ERROR:
	mt6575_touch_info("%s: i2c error, ret=%d--Li\n", __func__, ret);
	return ret;
}

#if defined(MT6575) || defined(MT6577)
int cy8ctst_write_block(uint8_t *txbuf, int length) 
{
	 /*
	  * Li.Yang
	  * Important here!
	  * the first Byte of txbuf must be the start reg
	  */
	int i, rc = 0;
	 struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	if (length > 0) {
		rc = i2c_master_send(cy8ctst_client->client, (char *)txbuf, length);
		if (rc < 0) {
			mt6575_touch_info("%s:i2c error--Li\n", __func__);
			return -1;
		}
		mt6575_touch_info("%s:success, write_buf=", __func__);
		for(i = 0; i < length; i++) {
			mt6575_touch_info("%02x ", txbuf[i]);
		}
		mt6575_touch_info("--Li\n");
	}
	return rc; 
}
#endif

#if defined(MT6573)
ssize_t cy8ctst_dma_read_block(uint8_t reg, uint8_t *rxbuf, int length)
{
	int rc = 0;
	uint8_t start_addr = reg;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	memset((void *)rxbuf, 0, length);
	mutex_lock(&cy8ctst_client->lock);
	rc = i2c_master_send(cy8ctst_client->client, &start_addr, 1);
	if (rc < 0) {
		mt6575_touch_info("%s:send start addr error--Li\n", __func__);
		mutex_unlock(&cy8ctst_client->lock);
		return rc;
	}

	if (length > 0){
		cy8ctst_client->client->addr = (cy8ctst_client->client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		rc = i2c_master_recv(cy8ctst_client->client, (char *)cy8ctst_dma_pa, length);
		cy8ctst_client->client->addr = cy8ctst_client->client->addr & I2C_MASK_FLAG;
		if (rc < 0) {
			mt6575_touch_info("%s:i2c error--Li\n", __func__);
			mutex_unlock(&cy8ctst_client->lock);
			return rc;
		}
		memcpy((void *)rxbuf, (void *)cy8ctst_dma_va, length);
	}
	mutex_unlock(&cy8ctst_client->lock);
	//mt6575_touch_info("%s:success--Li\n", __func__);
	return 1;
}

int cy8ctst_dma_write_block(u32 pa, int length)
{
	int  i, rc = 0;
	 /*
	  * Li.Yang
	  * Important here!
	  * the first Byte of dma_va must be the start reg
	  */
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	mutex_lock(&cy8ctst_client->lock);
	if (length > 0) {
		cy8ctst_client->client->addr = (cy8ctst_client->client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG;
		rc = i2c_master_send(cy8ctst_client->client, (char *)pa, length);
		cy8ctst_client->client->addr = cy8ctst_client->client->addr & I2C_MASK_FLAG;
		if (rc < 0) {
			mt6575_touch_info("%s:i2c error--Li\n", __func__);
			mutex_unlock(&cy8ctst_client->lock);
			return -1;
		}
		mt6575_touch_info("%s:success, write_buf=", __func__);
		for(i = 0; i < length; i++) {
			mt6575_touch_info("%02x ", cy8ctst_dma_va[i]);
		}
		mt6575_touch_info("--Li\n");
	}
	mutex_unlock(&cy8ctst_client->lock);
	return 0;
}
#endif

int cy8ctst_data_toggle(void)
{
	int ret = 0;
	mt6575_touch_info("+%s: --Li\n", __func__);
	ret = cy8ctst_read_reg(TPD_REG_BASE, (uint8_t*)&g_operation_data);
	if((g_operation_data.hst_mode & 0x80)==0)
		g_operation_data.hst_mode = g_operation_data.hst_mode|0x80;
	else
		g_operation_data.hst_mode = g_operation_data.hst_mode & (~0x80);
	ret = cy8ctst_write_reg(TPD_REG_BASE, &(g_operation_data.hst_mode));
	mt6575_touch_info("-%s: --Li\n", __func__);
	return ret;
	
}

int cy8ctst_to_zero(void)
{
	int rc = 0;
	uint8_t buf[2];
	buf[0] = 0;
	buf[1] = 0;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	mutex_lock(&cy8ctst_client->lock);
	rc = i2c_master_send(cy8ctst_client->client, buf, 2);
	if (rc < 0) {
		mt6575_touch_info("%s:i2c error--Li\n", __func__);
		mutex_unlock(&cy8ctst_client->lock);
		return -1;
	}
	mt6575_touch_info("%s:success--Li\n", __func__);
	mutex_unlock(&cy8ctst_client->lock);
	return rc; 
	
}

int cy8ctst_bootloader_info (int show)
{
	int ret = TPD_OK;
	ret = cy8ctst_read_block(TPD_REG_BASE, (uint8_t *)&g_bootloader_data, sizeof(g_bootloader_data));
	if(show)
	mt6575_touch_info("BootLoader: bl_file=%02X, bl_status=%02X, bl_error=%02X\n", 
		g_bootloader_data.bl_file, g_bootloader_data.bl_status, g_bootloader_data.bl_error);

	mt6575_touch_info("BootLoader: blver=%02X%02X, bld_blver=%02X%02X\n", 
		g_bootloader_data.blver_hi, g_bootloader_data.blver_lo, 
		g_bootloader_data.bld_blver_hi, g_bootloader_data.bld_blver_lo);

	mt6575_touch_info("BootLoader: ttspver=0x%02X%02X, appid=0x%02X%02X, appver=0x%02X%02X\n", 
		g_bootloader_data.ttspver_hi, g_bootloader_data.ttspver_lo, 
		g_bootloader_data.appid_hi, g_bootloader_data.appid_lo, 
		g_bootloader_data.appver_hi, g_bootloader_data.appver_lo);
	 
	mt6575_touch_info("BootLoader: cid=0x%02X%02X%02X\n", 
		g_bootloader_data.cid_0, g_bootloader_data.cid_1, 
		g_bootloader_data.cid_2);
	return ret;
 }

void cy8ctst_exit_bootloader(void)
{
	int cmd_length = sizeof(bootloader_exit_cmd);
	mt6575_touch_info("%s--Li\n", __func__);
#if defined(MT6575) || defined(MT6577)
	cy8ctst_write_block(bootloader_exit_cmd, cmd_length);
#else
	memcpy((void *)cy8ctst_dma_va, (void *)bootloader_exit_cmd, cmd_length);
	cy8ctst_dma_write_block(cy8ctst_dma_pa, cmd_length);
#endif
	mdelay(10);
}

int cy8ctst_swtich_mode(uint8_t mode)
{
	int ret;
	ret = cy8ctst_write_reg(TPD_REG_BASE, &mode);
	return ret;
}

/*int e()
{
	uint8_t vendor, fw;
	int ret = 0;
	ret = cy8ctst_read_reg(CY8CTST_VENDOR_REG, &vendor);
	if(ret < 0) {
		mt6575_touch_info("%s: get vendor error--Li\n", __func__);
		return -1;
	}
	ret = cy8ctst_read_reg(CY8CTST_FW_REG, &fw);
	if(ret < 0) {
		mt6575_touch_info("%s: get fw version error--Li\n", __func__);
		return -1;
	}
	mt6575_touch_info("%s: OP mode, vendor=0x%02x, fw=0x%02x--Li\n", 
		__func__, vendor, fw);
	return ret;
}*/
int cy8ctst_get_vendor_fw()
{
	uint8_t vendor, fw;
	int ret = 0;
	ret = cy8ctst_read_reg(CY8CTST_VENDOR_REG, &vendor);
	if(ret < 0) {
		mt6575_touch_info("%s: get vendor error--Liu\n", __func__);
		return -1;
	}
	ret = cy8ctst_read_reg(CY8CTST_FW_REG, &fw);
	if(ret < 0) {
		mt6575_touch_info("%s: get fw version error--Liu\n", __func__);
		return -1;
	}
	mt6575_touch_info("%s: OP mode, vendor=0x%02x, fw=0x%02x--Liu\n", 
		__func__, vendor, fw);
	return ret;
}
int cy8ctst_get_fw()
{
	uint8_t fw =-1;
	int ret = 0;
	ret = cy8ctst_read_reg(CY8CTST_FW_REG, &fw);
	if(ret < 0) {
		mt6575_touch_info("%s: get fw version error--Li\n", __func__);
		return -1;
	}
	return (int)fw;
}
EXPORT_SYMBOL(cy8ctst_get_fw);
#ifdef CY8CTST242_USE_ALLOC_MEM
int cy8ctst_mem_alloc(void)
{
	i2c_bin_buf = (uint8_t *)kzalloc(I2C_BIN_SIZE, GFP_KERNEL);
	if(!i2c_bin_buf) {
		mt6575_touch_info("%s: NO MEM for i2c_bin_buf--Li\n", __func__);
		return -ENOMEM;
	}
	i2c_cmd_list = (struct cy8ctst_i2c_cmd *)kzalloc(sizeof(struct cy8ctst_i2c_cmd) * 50 * 1024, GFP_KERNEL);
	if(!i2c_cmd_list) {
		mt6575_touch_info("%s: NO MEM for i2c_cmd_list--Li\n", __func__);
		return -ENOMEM;
	}
	return 0;
}


int cy8ctst_mem_free(void)
{
	kfree(i2c_bin_buf);
	kfree(i2c_cmd_list);
	i2c_bin_buf = NULL;
	i2c_cmd_list = NULL;
	return 0;
}
EXPORT_SYMBOL(cy8ctst_mem_free);
#endif
bool cy8ctst_msg2133(void)
{
	return g_bcy8stmstar ;
}
EXPORT_SYMBOL(cy8ctst_msg2133);
int cy8ctst_dump_cmd_list(int num)
{
	int i, j;
	mt6575_touch_info("+%s:--Li\n", __func__);
	for(i = 0; i < num; i ++) {
		mt6575_touch_info("%s: opration=%d, addr=0x%02x, cmd_length=%d, delay_time=%d, cmd_buf=", 
			__func__, i2c_cmd_list[i].opration, i2c_cmd_list[i].addr, 
			i2c_cmd_list[i].cmd_length, i2c_cmd_list[i].delay_time);
		for(j = 0; j < 20; j++) {
			mt6575_touch_info("%02x ", i2c_cmd_list[i].cmd_buf[j]);
		}
		mt6575_touch_info("\n");
	}
	mt6575_touch_info("-%s:--Li\n", __func__);
	return 0;
}
EXPORT_SYMBOL(cy8ctst_dump_cmd_list);

int cy8ctst_read_i2c_bin(const char *filename)
{
	int length, ret, num = 0;
	struct file	*filp = NULL;
	char *start;
	uint8_t *tmp;
	uint32_t pos = 0;
	filp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(filp) ||  !filp) {
		mt6575_touch_info("%s: unable to open i2c bin--Li\n", __func__);
		return PTR_ERR(filp);
	}
#ifdef CY8CTST242_USE_ALLOC_MEM
	ret = cy8ctst_mem_alloc();
	if(ret) {
		return -1;
	}
#endif
	length = kernel_read(filp, filp->f_pos, i2c_bin_buf, I2C_BIN_SIZE);
	mt6575_touch_info("%s: read i2c bin file success, length=%d--Li\n", __func__, length);
	if(length >= I2C_BIN_SIZE) {
		mt6575_touch_info("%s:i2c bin to large, length=%d--Li\n", __func__, length);
		//return -1;
	}
	tmp = i2c_bin_buf;
	process_cmd = 1;
	while(tmp) {
		tmp = strstr(&i2c_bin_buf[pos], "\r\n");
		if(!tmp) {
			mt6575_touch_info("%s: reach the end of  i2c bin file--Li\n", __func__);
			break;
		}
		else {
			if((*(tmp-1) != 'p' ) && (*(tmp-1) =!']')) {
				mt6575_touch_info("%s: unknow command--Li\n", __func__);
				continue;
			}
		}
		i2c_cmd_list[num].cmd_length = 0;
		start = &i2c_bin_buf[pos];
		//mt6575_touch_info("start=0x%02x, 0x%02x, 0x%02x, 0x%02x, ...\n", 
			//*start, *(start + 1), *(start + 2), *(start + 3));
		switch(*start) {
			case 'w':
				i2c_cmd_list[num].opration = I2C_WRITE_CMD;
				memset(i2c_cmd_list[num].cmd_buf, -1, sizeof(i2c_cmd_list[num].cmd_buf));
				++start;
				while(*start == ' ')
					++start;
				i2c_cmd_list[num].addr = (uint8_t)simple_strtoul(start, &start, 16);
				while(*start == ' ')
					++start;
				i2c_cmd_list[num].reg= (uint8_t)simple_strtoul(start, &start, 16);
				while(*(start + 1) != 'p') {
					++start;
					if(*start == ' ') 
						continue;
					i2c_cmd_list[num].cmd_buf[i2c_cmd_list[num].cmd_length++] = 
						(uint8_t)simple_strtoul(start, &start, 16);
				}
				//mt6575_touch_info("opration=%d, addr=0x%02x, reg=0x%02x, cmd_length=%d\n", 
					//i2c_cmd_list[num].opration, i2c_cmd_list[num].addr, 
					//i2c_cmd_list[num].reg, i2c_cmd_list[num].cmd_length);
				break;
			case 'r':
				i2c_cmd_list[num].opration = I2C_READ_CMD;
				memset(i2c_cmd_list[num].cmd_buf, -1, sizeof(i2c_cmd_list[num].cmd_buf));
				++start;
				while(*start == ' ')
					++start;
				i2c_cmd_list[num].addr =  (uint8_t)simple_strtoul(start, &start, 16);
				break;
			case '[':
				i2c_cmd_list[num].opration = I2C_DELAY_CMD;
				memset(i2c_cmd_list[num].cmd_buf, -1, sizeof(i2c_cmd_list[num].cmd_buf));
				while(*start != '=')
					++start;
				++start;
				i2c_cmd_list[num].delay_time = (int)simple_strtoul(start, &start, 10);
				break;
			default:
				i2c_cmd_list[num].opration = I2C_UNKNOW_CMD;
				break;
		}
		
		pos = tmp -i2c_bin_buf + 2;
		num ++;	
	}
	total_cmd_num = num;
	process_cmd = 0;
	cy8ctst_dump_cmd_list(total_cmd_num);
	return 0;
}
EXPORT_SYMBOL(cy8ctst_read_i2c_bin);

int cy8ctst_update(void)
{
	int i, iLoop, iCheck = 0;
	int blk_num = 0, ret = 0;
	uint8_t buf[2];
	if(!i2c_cmd_list || !total_cmd_num) {
		mt6575_touch_info("%s: i2c_cmd_list=%p, total_cmd_num=%d--Li\n", 
			__func__, i2c_cmd_list, total_cmd_num);
		return -1;
	}
	mt6575_touch_info("%s: start--Li\n", __func__);
	cy8ctst_bootloader_info(1);
	process_cmd = 1;
	for (i = 0; i < total_cmd_num; i++) {
		switch(i2c_cmd_list[i].opration) {
			case I2C_WRITE_CMD:
				if( i2c_cmd_list[i].cmd_length > 1) {
					iLoop = 0;
					while(iLoop < 10) {
					#if defined(MT6575) || defined(MT6577)
						ret = cy8ctst_write_block(i2c_cmd_list[i].cmd_buf, i2c_cmd_list[i].cmd_length);
					#else
						memcpy((void *)cy8ctst_dma_va, (void*)i2c_cmd_list[i].cmd_buf, 
							i2c_cmd_list[i].cmd_length);
						ret = cy8ctst_dma_write_block(cy8ctst_dma_pa, i2c_cmd_list[i].cmd_length);
					#endif
						if(ret >= 0) {
							break;
						}
						else {
							mt6575_touch_info("%s: i2c error, i=%d, iLoop=%d--Li\n", 
								__func__, i, iLoop);
							++iLoop;
							msleep(50);
							continue;
						}
					}
					if(iLoop == 10) {
						mt6575_touch_info("%s: error, i=%d--Li\n", __func__, i);
						return -1;
					}
				}
				break;
			case I2C_READ_CMD:
				if(iCheck > 10) {
					mt6575_touch_info("%s: too many times checking, i=%d--Li\n", 
							__func__, i);
					return -1;
				}
				cy8ctst_read_block(TPD_REG_BASE, buf, 3);
				if(buf[2] == 0x20) {
					//write a block OK
					++blk_num;
					mt6575_touch_info("%s: write %d blocks, i=%d--Li\n", 
							__func__, blk_num, i);
					iCheck = 0;
				}
				else {
					++iCheck;
					mt6575_touch_info("%s: write a block error, i=%d, iCheck=%d, buf[2]=0x%02x--Li\n", 
							__func__, i, iCheck, buf[2]);
					if(i > 8)
						i -= 8;
				}
				break;	
			case I2C_DELAY_CMD:
				msleep(i2c_cmd_list[i].delay_time);
				#if 0
				switch(i2c_cmd_list[i].delay_time) 
				{
					case 100:
						msleep(i2c_cmd_list[i].delay_time/2);
						break;
					case 12000:
						msleep(8000);
						break;
					default:
						msleep(i2c_cmd_list[i].delay_time);
						break;
				}
				#endif
					
				break;
			case I2C_UNKNOW_CMD:
				mt6575_touch_info("%s: unknow cmd, i=%d--Li\n", 
					__func__, i);
				break;
			default:
				break;
		}
	}
	process_cmd = 0;
	cy8ctst_exit_bootloader();
	cy8ctst_get_vendor_fw();
	return 0;
}
EXPORT_SYMBOL(cy8ctst_update);

int cy8ctst_get_blk_data(void)
{
	int i = 0, j = 0, l = 0, k = -1;
	for(i = 0; i < total_cmd_num; i++) {
		if((i2c_cmd_list[i].opration == I2C_WRITE_CMD) && (i2c_cmd_list[i].cmd_length > 1)) {
			if((i2c_cmd_list[i].cmd_buf[0] == 0x00) && 
				(i2c_cmd_list[i].cmd_buf[1] == 0xFF)) {
				++k; j = 0;
				blk_list[k].blk_length= 0;	
			}
			if(j < 5) {
				mt6575_touch_info("%s: k=%d, j=%d", __func__, k, j);
				for(l = 0; l < i2c_cmd_list[i].cmd_length; l++) {
					mt6575_touch_info(" %02x", i2c_cmd_list[i].cmd_buf[l]);
				}
				mt6575_touch_info("--Li\n");
				memcpy((void *)&(blk_list[k].blk_buf[16 * j]), (void *)&(i2c_cmd_list[i].cmd_buf[1]), 
					(i2c_cmd_list[i].cmd_length-1));
				blk_list[k].blk_length +=  (i2c_cmd_list[i].cmd_length -1);
				++j;
			}
		}
	}
	total_blk_num = k + 1;
	return 0;
}
EXPORT_SYMBOL(cy8ctst_get_blk_data);

int cy8ctst_dump_blk_info(void)
{
	int i, j;
	mt6575_touch_info("+%s: total_blk_num=%d--Li\n", __func__, total_blk_num);
	for(i = 0; i < total_blk_num; i++) {
		mt6575_touch_info("%s: %d block, blk_length=%d, blk_buf=", 
			__func__, i, blk_list[i].blk_length);
		for(j = 0; j < blk_list[i].blk_length; j++) {
			mt6575_touch_info("%02x ", blk_list[i].blk_buf[j]);
		}
		mt6575_touch_info("\n");
	}
	mt6575_touch_info("-%s:--Li\n", __func__);
}
EXPORT_SYMBOL(cy8ctst_dump_blk_info);

int cy8ctst_block_update(void)
{
	int i, ret = 0;
	uint8_t buf[100], check_buf[3];
	uint8_t reg = TPD_REG_BASE;
	buf[0] = 0x00;
	struct i2c_client_ctxt *cy8ctst_client = _cy8ctst_client;
	if(!i2c_cmd_list || !blk_list) {
		mt6575_touch_info("%s: i2c_cmd_list or blk_list is NULL--Li\n", __func__);
	}
	cy8ctst_reset();
	memcpy((void *)&buf[1], i2c_cmd_list[0].cmd_buf, i2c_cmd_list[0].cmd_length);
#if defined(MT6575) || defined(MT6577)
	//Set command to set Touch in download mode
	ret = cy8ctst_write_block(buf, (i2c_cmd_list[0].cmd_length + 1));
	msleep(8000);
	//Set the reg to base
	i2c_master_send(cy8ctst_client->client, (char *)&reg, 1);
	cy8ctst_read_block(TPD_REG_BASE, check_buf, 3);
	if(check_buf[2] != 0x20) {
		mt6575_touch_info("%s: check_buf[2]=0x%02x, check error for download cmd--Li\n", 
			__func__, check_buf[2]);
	}
	for(i = 0; i < total_blk_num; i++) {
		//Block size should be 78 Byte
		if(blk_list[i].blk_length > 16) {
			memcpy((void *)&buf[1], blk_list[i].blk_buf, blk_list[i].blk_length);
			ret = cy8ctst_write_block(buf, (blk_list[i].blk_length + 1));
		}
		msleep(50);
		//Set the reg to base
		i2c_master_send(cy8ctst_client->client, (char *)&reg, 1);
		//Check whether the write cmd is OK
		cy8ctst_read_block(TPD_REG_BASE, check_buf, 3);
		if(check_buf[2] != 0x20) {
			
			mt6575_touch_info("%s: check error for %d blocks write--Li\n", __func__, i);
		}
		else {
			mt6575_touch_info("%s: wrtie %d blocks(total %d)--Li\n", 
				__func__, i, total_blk_num);
		}
	}	
#else
	memcpy((void *)cy8ctst_dma_va, (void *)buf, i2c_cmd_list[0].cmd_length + 1);
	//Set command to set Touch in download mode
	ret = cy8ctst_dma_write_block(cy8ctst_dma_pa, i2c_cmd_list[0].cmd_length + 1);
	msleep(8000);
	//Set the reg to base
	i2c_master_send(cy8ctst_client->client, (char *)&reg, 1);
	cy8ctst_read_block(TPD_REG_BASE, check_buf, 3);
	if(check_buf[2] != 0x20) {
		mt6575_touch_info("%s: check error for download cmd--Li\n", __func__);
	}
	for(i = 0; i < total_blk_num; i++) {
		//Block size should be 78 Byte
		if(blk_list[i].blk_length > 16) {
			memcpy((void *)&buf[1], blk_list[i].blk_buf, blk_list[i].blk_length);
			memcpy((void *)cy8ctst_dma_va, (void *)buf, blk_list[i].blk_length + 1);
			ret = cy8ctst_dma_write_block(cy8ctst_dma_pa, blk_list[i].blk_length + 1);
		}
		msleep(50);
		//Set the reg to base
		i2c_master_send(cy8ctst_client->client, (char *)&reg, 1);
		//Check whether the write cmd is OK
		cy8ctst_read_block(TPD_REG_BASE, buf, 3);
		if(buf[2] != 0x20) {
			mt6575_touch_info("%s: check error for %d blocks write--Li\n", __func__, i);
		}
		else {
			mt6575_touch_info("%s: wrtie %d blocks(total %d)--Li\n", 
				__func__, i, total_blk_num);
		}
	}
#endif
}
EXPORT_SYMBOL(cy8ctst_block_update);

int cy8ctst_open_sersor(int on)
{
	int ret = 0, value = 0;
	mt6575_touch_info("%s:on=%d--Li\n", __func__, on);
	value = !!on;
	sensor_open = value;
	ret = cy8ctst_write_reg(CY8CTST_SENSOR_EN_REG, &value);
	return ret;
}
EXPORT_SYMBOL(cy8ctst_open_sersor);

int cy8ctst_get_sensor_state(void)
{
	int ret = 0, value = 0;
	ret = cy8ctst_read_reg(CY8CTST_SENSOR_STATE_REG, &value);
	mt6575_touch_info("%s:ret=%d, value=%d--Li\n", 
		__func__, ret, value);
	if(ret >= 0) 
		return value;
	else
		return ret;
	
}
EXPORT_SYMBOL(cy8ctst_get_sensor_state);

int cy8ctst_irq_enable(int on)
{
	if(on) {
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	}
	else {
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	}
	return 0;
}

static void tpd_down(int x, int y, int p) {
#ifdef TPD_HAVE_BUTTON
    if ((boot_mode != NORMAL_BOOT) && (boot_mode != ALARM_BOOT))  {
	    if(y > 320) {
		    tpd_button(x, y, 1);
	    }
    }
#endif
    input_report_abs(tpd->dev, ABS_PRESSURE,p);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    //input_report_abs(tpd->dev,ABS_MT_TRACKING_ID,i);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    mt6575_touch_debug("%s: x=%d, y=%d, p=%d--Li\n", __func__, x, y, p);
    input_mt_sync(tpd->dev);
    TPD_DOWN_DEBUG_TRACK(x,y);
}

static void tpd_up(int x, int y,int p) {

    input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
   // input_report_abs(tpd->dev,ABS_MT_TRACKING_ID,i);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    mt6575_touch_debug("%s: x=%d, y=%d, p=%d--Li\n", 
		__func__, x, y, 0);
    input_mt_sync(tpd->dev);
    TPD_UP_DEBUG_TRACK(x,y);
}

#ifdef CY8CTST_SENSOR
extern void cy8ctst_alsps_do_work(void);
#endif
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	u32 retval;
	static uint8_t tt_mode;
	uint8_t data0, data1;
	memcpy(pinfo, cinfo, sizeof(struct touch_info));
	memset(cinfo, 0, sizeof(struct touch_info));
#if defined(MT6575) || defined(MT6577)
	retval = cy8ctst_read_block(TPD_REG_BASE, (uint8_t *)&g_operation_data, 
		sizeof(g_operation_data));
#else
	retval = cy8ctst_dma_read_block(TPD_REG_BASE, (uint8_t *)&g_operation_data, 
		sizeof(g_operation_data));
#endif
	mt6575_touch_debug("hst_mode=%02X, tt_mode=%02X, tt_stat=%02X, sensor_en=%02x, sensor_state=%02x\n", 
		g_operation_data.hst_mode, g_operation_data.tt_mode, g_operation_data.tt_stat, 
		g_operation_data.sensor_en, g_operation_data.sensor_state);
#ifdef CY8CTST_DEBUG
	mt6575_touch_debug("diff_L=%02X, diff_H=%02X, rawdata_L=%02X, rawdata_H=%02x, baseline_L=%02x, baseline_H=%02x,\n", 
		g_operation_data.diff_L, g_operation_data.diff_H, g_operation_data.rawdata_L, 
		g_operation_data.rawdata_H, g_operation_data.baseline_L, g_operation_data.baseline_H);
#endif
#ifdef CY8CTST_SENSOR
	if(g_operation_data.sensor_en) {
		cy8ctst_alsps_do_work();
	}
#endif
	cinfo->count = (g_operation_data.tt_stat & 0x0f) ; //point count
	mt6575_touch_debug("cinfo->count =%d\n",cinfo->count);
	mt6575_touch_debug("Procss raw data...\n");

	cinfo->x1 = (( g_operation_data.x1_H << 8) | ( g_operation_data.x1_L)); //point 1		
	cinfo->y1  = (( g_operation_data.y1_H << 8) | ( g_operation_data.y1_L));
	cinfo->p1 = g_operation_data.z1;
	cinfo->id1 = ((g_operation_data.touch12_id & 0xf0) >>4) -1;
	mt6575_touch_debug("cinfo->x1=%3d, cinfo->y1=%3d, cinfo->p1=%3d\n", 
		cinfo->x1 ,cinfo->y1 ,cinfo->p1);

	if(cinfo->count >1) {
		cinfo->x2 = (( g_operation_data.x2_H << 8) | ( g_operation_data.x2_L)); //point 2
		cinfo->y2 = (( g_operation_data.y2_H << 8) | ( g_operation_data.y2_L));
		cinfo->p2 = g_operation_data.z2;
		cinfo->id2 = (g_operation_data.touch12_id & 0x0f)-1;
		mt6575_touch_debug("cinfo->x2=%3d, cinfo->y2=%3d, cinfo->p2=%3d\n", 
			cinfo->x2, cinfo->y2, cinfo->p2);	
	}

	if (!cinfo->count) 
		return true; // this is a touch-up event

	if (g_operation_data.tt_mode & 0x20) {
		// buffer is not ready for use
		mt6575_touch_debug("uffer is not ready for use!\n");
		memcpy(cinfo, pinfo, sizeof(struct touch_info));
		return false;
	}

	// data toggle 
	data0 = cy8ctst_read_reg(TPD_REG_BASE, (uint8_t*)&g_operation_data);
	if((g_operation_data.hst_mode & 0x80)==0)
		g_operation_data.hst_mode = g_operation_data.hst_mode|0x80;
	else
		g_operation_data.hst_mode = g_operation_data.hst_mode & (~0x80);
	data1 = cy8ctst_write_reg(TPD_REG_BASE, &(g_operation_data.hst_mode));

	if (tt_mode == g_operation_data.tt_mode) {
 		// sampling not completed
            mt6575_touch_debug("sampling not completed!\n");
            memcpy(cinfo, pinfo, sizeof(struct touch_info));
            return false; 
	}
	else 
		tt_mode = g_operation_data.tt_mode;

	return true;

};

static int touch_event_handler(void *unused)
{
	struct touch_info cinfo, pinfo;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
	do {
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE); 
		wait_event_interruptible(waiter,tpd_flag!=0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		if (tpd_touchinfo(&cinfo, &pinfo)) {
			if(cinfo.count > 0) {
				tpd_down(cinfo.x1, cinfo.y1, cinfo.p1);
			if(cinfo.count == 1 && pinfo.count == 2) {
				if (cinfo.id1 == pinfo.id1 && cinfo.id1 != pinfo.id2)
					tpd_up(pinfo.x2, pinfo.y2, pinfo.p2);
				else
					tpd_up(pinfo.x1, pinfo.y1, pinfo.p1);
			}
			if(cinfo.count>1)
				tpd_down(cinfo.x2, cinfo.y2, cinfo.p2);
			}
			else {
			#ifdef TPD_HAVE_BUTTON
				if ((boot_mode != NORMAL_BOOT) && (boot_mode != ALARM_BOOT)) {
					if(cinfo.y1  > 320) 
						tpd_button(cinfo.x1,  cinfo.y1, 0);
				}
			#endif
				if(pinfo.count > 0) {
					tpd_up(pinfo.x1, pinfo.y1, pinfo.p1);
				if(pinfo.count > 1)
					tpd_up(pinfo.x2, pinfo.y2, pinfo.p2);
				}
			}
			input_sync(tpd->dev);
		}
	}while(!kthread_should_stop());
	return 0;
}

static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, "mtk-tpd");
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
    mt6575_touch_debug("TPD interrupt has been triggered\n");
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}

extern int cy8ctst_issp_init(void);
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
	mt6575_touch_info("======================%s: -Li==========\n", __func__);
	int i, ret = TPD_OK;
	int retry = 5;
	char buffer[2], rxbuf[11];
	struct i2c_client_ctxt *cy8ctst_client;
	cy8ctst_client = kzalloc(sizeof(*cy8ctst_client), GFP_KERNEL);
	if (!cy8ctst_client) {
		mt6575_touch_info("%s: no memery!--Li\n", __func__);
		return -ENOMEM;
	}
	cy8ctst_client->client = client;
	mutex_init(&cy8ctst_client->lock);
	mutex_init(&cy8ctst_client->power_lock);
	_cy8ctst_client = cy8ctst_client;
	if(!binit) {
		binit = 1;
		cy8ctst_issp_init();
	}
	cy8ctst_power(1);
	msleep(100);
	while (retry) {
		ret = cy8ctst_read_reg(TPD_REG_BASE, &(buffer[0]));
		if(ret < 0) {
			mt6575_touch_info("%s: ret=%d, retry=%d--Li\n", __func__, ret, retry);
			msleep(100);
			--retry;
		}
		else {
			mt6575_touch_info("%s: success retry=%d--Li\n", __func__, retry);
			break;
		}

	}
	if(!retry) {
		g_bcy8stmstar = false;
		mt6575_touch_info("%s: i2c error, stop probe here!--Li\n", __func__);
		cy8ctst_client->client= NULL;
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		cy8ctst_power(0);
		return ret;
	}
#if defined(MT6573)
	cy8ctst_dma_va = (uint8_t *)dma_alloc_coherent(NULL, 4096, &cy8ctst_dma_pa, GFP_KERNEL);
	if(!cy8ctst_dma_va) {
		mt6575_touch_info("%s: Allocate DMA I2C Buffer failed!--Li\n", __func__);
		return -1;
	}
#endif
#ifdef CY8CTST_BOOTLOADER
	cy8ctst_swtich_mode(TPD_SOFT_RESET_MODE);
	retry = 10;
	while(retry) {
		mdelay(100);
		ret = cy8ctst_bootloader_info(1);
		if(ret < 0) {
			mt6575_touch_info("%s: get bootloader_info i2c error!--Li\n", __func__);
			break;
		}
		else {
			if(GET_BOOTLOADERMODE(g_bootloader_data.bl_status))  {
				mt6575_touch_info("%s: break, get bootloader mode, bl_status=0x%x, bl_file=0x%x, retry=%d--Li\n", 
					__func__, g_bootloader_data.bl_status, g_bootloader_data.bl_file, retry);
				break;
			}
			else {
				if(g_bootloader_data.bl_file == TPD_LOW_PWR_MODE) {
					mt6575_touch_info("%s: break, get LOW_PWR_MODE mode, bl_status=0x%x, bl_file=0x%x, retry=%d--Li\n", 
						__func__, g_bootloader_data.bl_status, g_bootloader_data.bl_file, retry);
					break;
				}
				else {
					--retry;
				}
					
			}
		}
	}
	
	cy8ctst_read_block(TPD_REG_BASE, rxbuf, 16);
	mt6575_touch_info("rxbuf before exit_bootloader");

	for(i = 0; i < 12; i++) {
		mt6575_touch_info(" %02x", rxbuf[i]);		
	}
	mt6575_touch_info("--Li\n");
	cy8ctst_exit_bootloader();
	cy8ctst_read_block(TPD_REG_BASE, rxbuf, 16);
	mt6575_touch_info("rxbuf after exit_bootloader");		
	for(i = 0; i < 12; i++) {
		mt6575_touch_info(" %02x", rxbuf[i]);		
	}
	mt6575_touch_info("--Li\n");
#endif
	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) { 
		ret = PTR_ERR(thread);
		return ret;
		mt6575_touch_info(TPD_DEVICE " failed to create kernel thread: %d\n", ret);
	}
	mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM,CUST_EINT_EDGE_SENSITIVE);//CUST_EINT_LEVEL_SENSITIVE
	mt_eint_set_polarity(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_POLARITY);
	mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	msleep(100);
	mt6575_touch_info("CY8CTST242 Touch Panel Device Probe %s--Li\n", 
		(ret < TPD_OK) ? "FAIL" : "PASS");
	return ret;
}

static int __devexit tpd_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */
	mt6575_touch_info("TPD removed\n");
	return 0;
}

static int tpd_local_init(void)
{
	mt6575_touch_info("CY8CTST242 I2C Touchscreen Driver (Built %s @ %s)--Li\n", 
		__DATE__, __TIME__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}

	if(tpd_load_status == 0){
		mt6575_touch_info("add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}

#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, 
		tpd_keys_local, tpd_keys_dim_local);
	boot_mode = get_boot_mode();
#endif
	return 0;
}

static void tpd_resume(struct early_suspend *h)
{
	mt6575_touch_info("+%s:--Li\n", __func__);
	if(process_cmd || updating || sensor_open) {
		mt6575_touch_info("%s: process_cmd=%d, updating=%d, sensor_open=%d--Li\n", 
			__func__, process_cmd, updating, sensor_open);
		return;
	}
	cy8ctst_power(1);
#ifdef CY8CTST_BOOTLOADER
	cy8ctst_exit_bootloader();
#endif
	//Avoid useless interrupt after touch power on
	msleep(150);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	mt6575_touch_info("-%s:--Li\n", __func__);
    mt6575_touch_info("=======cy8ctst_get_fw()=====%d",cy8ctst_get_fw());
}

static void tpd_suspend(struct early_suspend *h)
{
	mt6575_touch_info("+%s:--Li\n", __func__);
	if(process_cmd || updating || sensor_open) {
		mt6575_touch_info("%s: process_cmd=%d, updating=%d, sensor_open=%d--Li\n", 
			__func__, process_cmd, updating, sensor_open);
		return;
	}
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	cy8ctst_power(0);
	mt6575_touch_info("-%s:--Li\n", __func__);
}

static struct tpd_driver_t tpd_device_driver = {
    .tpd_device_name = "cy8ctst242",
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif		
};

static int __init tpd_driver_init(void) {
	mt6575_touch_info("Tinno cy8ctst242 touch panel driver init\n");
        i2c_register_board_info(TPD_I2C_GROUP_ID, &cy8ctst242_i2c_tpd, sizeof(cy8ctst242_i2c_tpd)/sizeof(cy8ctst242_i2c_tpd[0]));
	if(tpd_driver_add(&tpd_device_driver) < 0)
		mt6575_touch_info("Add cy8ctst242 driver failed\n");
	return 0;
}

static void __exit tpd_driver_exit(void) {
	mt6575_touch_info("Tinno cy8ctst242 touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

