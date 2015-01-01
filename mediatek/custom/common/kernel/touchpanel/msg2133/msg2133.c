/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be strictly prohibited.
*
* MediaTek Inc. (C) 2010. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* The following software/firmware and/or related documentation ("MediaTek Software")
* have been modified by MediaTek Inc. All revisions are subject to any receiver's
* applicable license agreements with MediaTek Inc.
*/


#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/dma-mapping.h>
#include <linux/slab.h>

//add by liyaohua start
#include <linux/wakelock.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <asm/unistd.h>
#include <linux/device.h>
//add by liyaohua end


#if defined(MT6575)|| defined(MT6577)
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#include "cust_gpio_usage.h"

#include "msg2133.h"

#define TP_PROXIMITY_SENSOR
#ifdef TP_PROXIMITY_SENSOR
//#define TPD_PROXIMITY_DEBUG
#define TPD_PROXIMITY_DEVICE            "mtk-tpd-msg2133"
#define TPD_PROXIMITY_DMESG(a,arg...) printk(TPD_PROXIMITY_DEVICE ": " a,##arg)
#if defined(TPD_PROXIMITY_DEBUG)
#undef TPD_PROXIMITY_DEBUG
#define TPD_PROXIMITY_DEBUG(a,arg...) printk(TPD_PROXIMITY_DEVICE ": " a,##arg)
#else
#define TPD_PROXIMITY_DEBUG(arg...)
#endif
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>

static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;//0-->close ; 1--> far away
#endif
#define TOUCH_ADDR_MSG21XX   (0x4C>>1)
#define FW_ADDR_MSG21XX   (0xC4>>1)
// debug macros
//#define TP_DEBUG_MSG
#ifdef TP_DEBUG_MSG
/*#define pr_tp(format, args...) printk("<MSG>" format, ##args)
#define pr_ch(format, args...)                      \
	printk("<MSG>" "%s <%d>,%s(),cheehwa_print:\n\t"  \
	format,__FILE__,__LINE__,__func__, ##arg)*/
	
//#define pr_tp printk
//#define pr_ch printk

#else
#define pr_tp(format, args...)  do {} while (0)
#define pr_ch(format, args...)  do {} while (0)
#define pr_k(format, args...)  do {} while (0)
#endif

//#define EN_PIN_1_8_V

//add by liyaohua 
#define MEM_MALLOC_SIZE 	4096	
#define MEM_MAJOR      999
#define MEM_MINOR    	0 
struct wake_lock  Tp_wake_lock;
bool  tfopensenor = false;
static int boot_mode;
static int g_update;
char *mem_spvm; 	 
struct class *mem_class;
static struct mutex tp_mutex;
static struct mutex readtp_mutex;
static struct mutex rwtp_mutex;
static int sensor_open = 0;
static int updating;
static int iIsMsg2133  = 1;
static int icy =0;
static int ig_sensor = 0;
static int ig_batstate = 0;
static int point_num = 0;
int msg2133chipid =-1;
static int update_switch = 0;
int tempx = 0;
int tempy = 0;
#define ABS(X)                  ((X > 0) ? (X) : (-X))
//add by liyaohua end
static unsigned char touch_up_num = 0;
#define GPIO_CTP_MSG2133_EN_PIN           GPIO_CTP_RST_PIN
#define GPIO_CTP_MSG2133_EN_PIN_M_GPIO    GPIO_MODE_00

#define GPIO_CTP_MSG2133_EINT_PIN           GPIO_CTP_EINT_PIN
#define GPIO_CTP_MSG2133_EINT_PIN_M_GPIO   GPIO_CTP_EINT_PIN_M_EINT

#define TPD_POINT_INFO_LEN      8
#define TPD_MAX_POINTS          2

struct touch_info {
	unsigned int mask;
	int x[TPD_MAX_POINTS];
	int y[TPD_MAX_POINTS];
};
typedef struct
{
	unsigned short pos_x;
	unsigned short pos_y;
	unsigned short pos_x2;
	unsigned short pos_y2;
	unsigned short temp2;
	unsigned short temp;
	short dst_x;
	short dst_y;
	unsigned char checksum;
} SHORT_TOUCH_STATE;

extern struct tpd_device *tpd;

static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static void tpd_eint_interrupt_handler(void);
#if 0
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									 kal_bool auto_umask);
#endif
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag = 0;
#ifdef TP_PROXIMITY_SENSOR
char ps_data_state[1] = {0};
enum
{
	DISABLE_CTP_PS,
	ENABLE_CTP_PS,
	RESET_CTP_PS
};
// Jake.L, DATE20130527, NOTE, DATE20130527-01 START
static int tp_err = 0;
static int is_tp_sleeping = 0;
// Jake.L, DATE20130527-01 END
#endif
#if 0//adair:根据平台不同选择不同位的i2c地址
#define FW_ADDR_MSG21XX   (0xC4)
#define FW_ADDR_MSG21XX_TP   (0x4C)
#define FW_UPDATE_ADDR_MSG21XX   (0x92)
#else
#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
#endif

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
static int mem_open(struct inode *ind, struct file *filp); 
static int mem_release(struct inode *ind, struct file *filp);
static ssize_t mem_read(struct file *filp, char __user *buf, size_t size, loff_t *fpos);
static ssize_t mem_write(struct file *filp, const char __user *buf, size_t size, loff_t *fpos);
void Get_Chip_Version(void);

static const struct i2c_device_id msg2133_tpd_id[] = {{"msg2133",0},{}};
static struct i2c_board_info __initdata msg2133_i2c_tpd={ I2C_BOARD_INFO("msg2133", TOUCH_ADDR_MSG21XX)};
static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = "msg2133",
	},
	.probe = tpd_probe,
	.remove = __devexit_p(tpd_remove),
	.id_table = msg2133_tpd_id,
	.detect = tpd_detect,
};
struct file_operations mem_fops = 
{
	//.owner=THIS_MODULE,  
	.open = mem_open,
	.release = mem_release,	
	.read = mem_read,	  
	.write = mem_write,  
};

#define MSG2133_INVALID_VER 0xffff
static  unsigned short msg2133_fm_major=MSG2133_INVALID_VER, msg2133_fm_minor=MSG2133_INVALID_VER;
static  unsigned short msg2133_bin_major=-1, msg2133_bin_minor=-1;
static  unsigned short msg2133a_fm_major=MSG2133_INVALID_VER, msg2133a_fm_minor=MSG2133_INVALID_VER;
static  unsigned short msg2133a_bin_major=-1, msg2133a_bin_minor=-1;
int msg2133_read_fw_ver_custom(void);
#ifdef TP_PROXIMITY_SENSOR
extern void cy8ctst_alsps_do_work(void);
extern int  cy8ctst_alsps_get_openstate(void);
#endif
extern int cy8ctst_get_fw();
extern int cy8ctst_FTM_Get_Fw();


//add by liyaohua
static void i2c_write_msg2133(u8 *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->timing = 40;
	i2c_client->addr = FW_ADDR_MSG21XX;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
	i2c_client->addr = TOUCH_ADDR_MSG21XX;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	i2c_client->timing = 240;

	// ret = i2c_smbus_write_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);
	if(ret <= 0)
	{
		pr_tp("i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
	}
}
static void i2c_read_msg2133(u8 *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->timing = 40;
	i2c_client->addr = FW_ADDR_MSG21XX;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
	i2c_client->addr = TOUCH_ADDR_MSG21XX;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	i2c_client->timing = 240;
	//  ret=i2c_smbus_read_i2c_block_data(i2c_client, *pbt_buf, dw_lenth-1, pbt_buf+1);

	if(ret <= 0)
	{
		pr_tp("i2c_read_interface error line = %d, ret = %d\n", __LINE__, ret);
	}
}
static bool msg2133_i2c_read(char *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->timing = 100;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
	if(ret <= 0)
	{
		pr_tp("msg_i2c_read_interface error\n");
		return false;
	}

	return true;
}
int __init register_unregister_chrdev_init(void)
{
	int res;
	pr_tp("<0>into register_unregister_chrdev_init\n");
	mem_spvm = (char *)vmalloc(MEM_MALLOC_SIZE);  
	res=register_chrdev(MEM_MAJOR,"my_char_dev",&mem_fops);
	if(res)    //×￠2áê§°ü
	{
		unregister_chrdev(MEM_MAJOR,"my_char_dev"); 
		pr_tp("<0>register char dev failed\n");
		return -1;
	}
	pr_tp("<0>register char dev success\n");
 	mem_class = class_create(THIS_MODULE, "my_char_dev"); 
	mutex_init(&tp_mutex);	
	mutex_init(&readtp_mutex);	
	mutex_init(&rwtp_mutex);	
	

	
  	if(IS_ERR(mem_class)) 
       {
  	      pr_tp("<0>failed in creating class.\n");
	      class_destroy(mem_class);
 	      return -1;
 	}
	pr_tp("<0>class create success\n");
    	device_create(mem_class, NULL, MKDEV(MEM_MAJOR,MEM_MINOR), NULL, "my_char_dev");  //?—????????¨?????t????-3?????????????????o???
	pr_tp("<0>device create success\n");
	pr_tp("<0>out register_unregister_chrdev_init\n");
	return 0;
}

void __exit register_unregister_chrdev_exit (void)
{
	pr_tp("<0>into register_unregister_chrdev_exit\n");	
	unregister_chrdev(MEM_MAJOR,"my_char_dev"); 
	pr_tp("<0>unregister char dev success\n");
       device_destroy(mem_class, MKDEV(MEM_MAJOR,MEM_MINOR));
	pr_tp("<0>device destroy success\n");
       class_destroy(mem_class);	    
	pr_tp("<0>class destroy success\n");
	if (mem_spvm != NULL)
		vfree(mem_spvm);     
	pr_tp("<0>vfree ok!\n");
	pr_tp("<0>out register_unregister_chrdev_exit\n");
}
int mem_open(struct inode *ind, struct file *filp)
{	
	pr_tp("<0>open vmalloc space\n");
	try_module_get(THIS_MODULE); 
	pr_tp("<0>open vmalloc space success\n");
	return 0;
}

ssize_t mem_read(struct file *filp, char *buf, size_t size, loff_t *lofp)
{
	int res = -1;
	char *tmp =NULL;
	pr_tp("<0>copy data to the user space\n");
	tmp = mem_spvm;
	
	if (size > MEM_MALLOC_SIZE)  
		size = MEM_MALLOC_SIZE;
	if (tmp != NULL)
		res = copy_to_user(buf, tmp, size); 
	if (res == 0)
	{
		pr_tp("<0>copy data success and the data is:%s\n",tmp);
		return size;
	}
	else
	{
		pr_tp("<0>copy data fail to the user space\n");
		return 0;
	}
}


int mem_release(struct inode *ind, struct file *filp)
{
	pr_tp("<0>close vmalloc space\n");
	module_put(THIS_MODULE);
	pr_tp("<0>close vmalloc space success\n");
	return 0;
}

static int i2c_write(u8 addr, u8 *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->addr = addr;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
	i2c_client->addr = FW_ADDR_MSG21XX_TP;
	i2c_client->addr|=I2C_ENEXT_FLAG;

	if(ret <= 0)
	{
		pr_tp("i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
	}
	return ret;
}
#ifdef TP_PROXIMITY_SENSOR
int cy8ctst_sersor_snedC0(void)
{
	U8 ps_store_data[4];

	int ret = 0;
	ps_store_data[0] = 0x52;
	ps_store_data[1] = 0x00;
	ps_store_data[2] = 0x62;
	ps_store_data[3] = 0xa0;
	
    ret =	i2c_write(TOUCH_ADDR_MSG21XX, &ps_store_data[0], 4);
	pr_tp("tpd_touchinfo cy8ctst_sersor_snedC0_ ret= %d\n", ret);
	
}


int msg2133_open_sersor(int on)
{

	
	U8 ps_store_data[4];
	int ret = 0, value = 0;
	pr_tp("tpd_touchinfo---Enable msg2133_open_sersor on:%d \n",on);
	if(on)
	{
			//add by liyaohua start
			  wake_lock(&Tp_wake_lock);
			//add by liyaohua end
			
					
			tfopensenor = true;
			ps_store_data[0] = 0x52;
			pr_tp("tpd_touchinfo---0x52\n");
			ps_store_data[1] = 0x00;
			pr_tp("tpd_touchinfo---0x00\n");
			ps_store_data[2] = 0x62;
			ps_store_data[3] = 0xa0;			
			ret =	i2c_write(TOUCH_ADDR_MSG21XX, &ps_store_data[0], 4);
			pr_tp("==@@@@@@@@tpd_touchinfo---Enable msg2133_open_sersor%d\n",ret);	
			if(ret < 0)
			{
			sensor_open = 0;
			ig_sensor = 0;
			return ret;
			}
			sensor_open = 1;
			ig_sensor = 1;
			
	}
	else
	{
	    // Jake.L, DATE20130527, NOTE, DATE20130527-01 START
	    if (is_tp_sleeping)
        {
            tp_err = 1;
            pr_tp("Err: Disable alps while sleeping.\n");
        }
	    // Jake.L, DATE20130527-01 END
        
	        //add by liyaohua start
			wake_unlock(&Tp_wake_lock);
			//add by liyaohua end
			pr_tp("@@@@@@@tpd_touchinfo---DISABLE msg2133_open_sersor\n");
			tfopensenor =false;
			ps_store_data[0] = 0x52;
			pr_tp("tpd_touchinfo---0x52\n");
			ps_store_data[1] = 0x00;
			pr_tp("tpd_touchinfo---0x00\n");
			ps_store_data[2] = 0x62;
			ps_store_data[3] = 0xa1;		
			ret =	i2c_write(TOUCH_ADDR_MSG21XX, &ps_store_data[0], 4);	
			pr_tp("==@@@@@@@tpd_touchinfo---DISABLE msg2133_open_sersor%d\n",ret);	
			if(ret < 0)
			{
			sensor_open = 1;
			ig_sensor = 0;
			 return ret;
			}	
			sensor_open = 0;	
			ig_sensor = 1;
					
	}	
	return ret;
}
EXPORT_SYMBOL(msg2133_open_sersor);
int msg2133_get_sensor_state(void)
{
	pr_tp("@@@@@tpd_touchinfo---msg2133_get_sensor_stateps_data_state[0] %u ",
ps_data_state[0]);
	return ps_data_state[0];
//	return 0;
}
EXPORT_SYMBOL( msg2133_get_sensor_state);



#endif

#define __FIRMWARE_UPDATE__
#ifdef __FIRMWARE_UPDATE__

/*adair:0777为打开apk升级功能，0664为关闭apk升级功能，无需将宏__FIRMWARE_UPDATE__关闭*/
#define CTP_AUTHORITY 0777//0664

#define ENABLE_AUTO_UPDATA
static unsigned char MstartCTP_FWData[]=
{
//#include "MSG2133_S7500_20120701.i"
//#include "msg2133a-0516-3234.i"
//#include "msg2133a-0528-3236.i"
#include "msg2133a-0529-3238.i"
};
static unsigned char CTPM_FW[]=
{
#include "msg2133-0516-0114.i"
//#include "S7505-0130504.i"
};

void msg2133a_ctpm_get_upg_ver_custom(void)
{
	//msg2133_bin_major=(CTPM_FW[0x3076]<<8)|CTPM_FW[0x3077];
	//msg2133_bin_minor=(CTPM_FW[0x3074]<<8)|CTPM_FW[0x3075];
	msg2133a_bin_major=MstartCTP_FWData[0x7f4f]<<8|MstartCTP_FWData[0x7f4e];
	msg2133a_bin_minor=MstartCTP_FWData[0x7f51]<<8|MstartCTP_FWData[0x7f50];
	

	pr_tp("***msg2133a_bin_majormajor = %d ***\n", msg2133a_bin_major);
	pr_tp("***msg2133a_bin_majorminor = %d ***\n", msg2133a_bin_minor);
}
void msg2133_ctpm_get_upg_ver_custom(void)
{
	msg2133_bin_major=(CTPM_FW[0x3076]<<8)|CTPM_FW[0x3077];
	msg2133_bin_minor=(CTPM_FW[0x3074]<<8)|CTPM_FW[0x3075];
	

	pr_tp("***msg2133_bin_majormajor = %d ***\n", msg2133_bin_major);
	pr_tp("***msg2133_bin_majorminor = %d ***\n", msg2133_bin_minor);
}

#if 0
#define TP_DEBUG(format, ...)	printk(KERN_INFO "MSG2133_MSG21XXA_update_INFO***" format "\n", ## __VA_ARGS__)
#else
#define TP_DEBUG(format, ...)
#endif
#if 0//adair:正式版本关闭
#define TP_DEBUG_ERR(format, ...)	printk(KERN_ERR "MSG2133_MSG21XXA_update_ERR***" format "\n", ## __VA_ARGS__)
#else
#define TP_DEBUG_ERR(format, ...)
#endif
static  char *fw_version;
static u8 temp[94][1024];
//u8  Fmr_Loader[1024];
u32 crc_tab[256];
static u8 g_dwiic_info_data[1024];   // Buffer for info data

static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

#define N_BYTE_PER_TIME (8)//adair:1024的约数,根据平台修改
#define UPDATE_TIMES (1024/N_BYTE_PER_TIME)


/*adair:以下5个以Hal开头的函数需要根据平台修改*/
/*disable irq*/
static void HalDisableIrq(void)
{
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, NULL, 1);
}
/*enable irq*/
static void HalEnableIrq(void)
{
    mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}
/*reset the chip*/
static void _HalTscrHWReset(void)
{
    pr_tp("_HalTscrHWReset\n");
    
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(200);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
	msleep(500);
}
static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size)
{
    int ret;
    i2c_client->addr = addr;
    ret = i2c_master_recv(i2c_client, read_data, size);
    i2c_client->addr = FW_ADDR_MSG21XX_TP;

    if(ret <= 0)
    {
		TP_DEBUG_ERR("HalTscrCReadI2CSeq error %d,addr = %d\n", ret,addr);
	}
}

void msta_power(int on)
{
	//pr_tp("====================================================msta_power %d===========================\n",on);
	if(on) {
	#if defined(MT6575) || defined(MT6577)
		hwPowerOn(MT65XX_POWER_LDO_VGP, VOL_2800, "touch"); 
	#else
		mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	#endif 
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT); 
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
		msleep(200);
		mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
		mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
		ig_batstate = 1;
		pr_tp("====msta_power 1============================\n");
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
		ig_batstate = 0;
		pr_tp("====msta_power 0============================\n");
	}
}


static int i2c_read(u8 addr, u8 *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->addr = addr;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	ret = i2c_master_recv(i2c_client, pbt_buf, dw_lenth);
	i2c_client->addr = TOUCH_ADDR_MSG21XX;
	i2c_client->addr|=I2C_ENEXT_FLAG;

	if(ret <= 0)
	{
		pr_tp("i2c_read_interface error line = %d, ret = %d\n", __LINE__, ret);
	}
	return ret;
}


static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    int ret;
    i2c_client->addr = addr;
    ret = i2c_master_send(i2c_client, data, size);
    i2c_client->addr = FW_ADDR_MSG21XX_TP;

    if(ret <= 0)
    {
		TP_DEBUG_ERR("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", ret,addr);
	}
}

/*
static void Get_Chip_Version(void)
{
    pr_tp("[%s]: Enter!\n", __func__);
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[2];

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCE;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    if (dbbus_rx_data[1] == 0)
    {
        // it is Catch2
        TP_DEBUG(pr_tp("*** Catch2 ***\n");)
        //FwVersion  = 2;// 2 means Catch2
    }
    else
    {
        // it is catch1
        TP_DEBUG(pr_tp("*** Catch1 ***\n");)
        //FwVersion  = 1;// 1 means Catch1
    }

}
*/

static void dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

    // Delay some interval to guard the next transaction
    udelay ( 150);//200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
	TP_DEBUG("\n******%s come in*******\n",__FUNCTION__);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);
    udelay ( 150 );//200 );        // delay about 0.1ms
}

static u8 drvISP_Read(u8 n, u8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0};
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay( 800 );//200);
    if (n == 1)
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
        *pDataToRead = dbbus_rx_data[0];
        TP_DEBUG("dbbus=%d,%d===drvISP_Read=====\n",dbbus_rx_data[0],dbbus_rx_data[1]);
  	}
    else
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, pDataToRead, n);
    }

    return 0;
}

static void drvISP_WriteEnable(void)
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    udelay(150);//1.16
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1);
    udelay( 150 );//200);
}

static u8 drvISP_ReadStatus(void)
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(150);//200);
    drvISP_Read(1, &bReadData);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    return bReadData;
}


static void drvISP_BlockErase(u32 addr)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
	TP_DEBUG("\n******%s come in*******\n",__FUNCTION__);
	u32 timeOutCount=0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 3);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
	//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
    timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	}
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;//0xD8;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    //bWriteData[4] = (addr & 0xFF) ;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 5);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
		//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
	timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
	}
}

static void drvISP_Program(u16 k, u8* pDataToWrite)
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
		u32 timeOutCount=0;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);
        for (i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        //msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(150);//200);

        timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
			timeOutCount++;
			if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
		}

        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, TX_data, 133);   //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    }
}

static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    return sprintf ( buf, "%s\n", fw_version );
}


static void drvISP_Verify ( u16 k, u8* pDataToVerify )
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] ={ 0x10, 0x03, 0, 0, 0 };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index = 0;
    u32 timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( u8 ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( u8 ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( u8 ) ( addr + j * 128 );
        udelay ( 100 );        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 ); //write read flash addr
        udelay ( 100 );        // delay about 100us*****
        drvISP_Read ( 128, RX_data );
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                TP_DEBUG ( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static void drvISP_ChipErase()
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
    u32 timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 3 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}

/* update the firmware part, used by apk*/
/*show the fw version*/

static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG_ERR ( "update_C2 OK\n" );
    drvISP_ExitIspMode();
    _HalTscrHWReset();
    FwDataCnt = 0;
    HalEnableIrq();
    return size;
}

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
	ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}
static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////

    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size,  EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
      // Buffer for slave's firmware

    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();
    _HalTscrHWReset();
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x1C70 );


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );

        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x9432 );

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        TP_DEBUG_ERR ( "update_C32 FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	HalEnableIrq();
        return ( 0 );
    }

    TP_DEBUG_ERR ( "update_C32 OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
	HalEnableIrq();

    return size;
#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}


static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
    u8  dwiic_rx_data[4];
    u16 reg_data=0;
    mdelay ( 300 );

    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );
    mdelay ( 100 );
    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );
    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
    mdelay ( 50 );

    // recive info data
    //HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 1024 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 8 );
    return ( 1 );
}

static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;

    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;
    drvTP_read_info_dwiic_c33();
	pr_tp("===firmware_update_c33==");

    if ( 0/*g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' */)
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

		g_dwiic_info_data[8]=temp[32][8];
		g_dwiic_info_data[9]=temp[32][9];
		g_dwiic_info_data[10]=temp[32][10];
		g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
		g_dwiic_info_data[12]=life_counter[0];
		g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
		drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 50 );
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

        }
        while ( reg_data != 0x2F43 );
        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[0], 1024 );
        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
    }

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }
    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );
    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO )
			i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }
        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        #if 1
        {
            u32 n = 0;
            for(n=0;n<UPDATE_TIMES;n++)
            {
            	pr_tp("==== C 33 UP ===========");
                HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i]+n*N_BYTE_PER_TIME, N_BYTE_PER_TIME );
            }
        }
        #else
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );
        #endif
        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        if ( crc_info_tp != crc_info )
            update_pass = 0;
    }
    if ( !update_pass )
    {
        TP_DEBUG_ERR ( "update_C33 FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	HalEnableIrq();
        return size;
    }
    TP_DEBUG_ERR ( "update_C33 OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
    HalEnableIrq();
    return size;
}

#define _FW_UPDATE_C3_
#ifdef _FW_UPDATE_C3_
static ssize_t firmwarea_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
	pr_tp("============firmware_update_store ==========\n");
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	HalDisableIrq();
	pr_tp("============HalDisableIrq ==========\n");

    _HalTscrHWReset();
	pr_tp("============_HalTscrHWReset ==========\n");



    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
		pr_tp("============dbbusDWIICEnterSerialDebugMode ==========\n");
    dbbusDWIICStopMCU();
	pr_tp("============dbbusDWIICStopMCU( ==========\n");

    dbbusDWIICIICUseBus();
		pr_tp("============  dbbusDWIICIICUseBus(); ==========\n");
    dbbusDWIICIICReshape();
	pr_tp("============  dbbusDWIICIICReshape();==========\n");
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
	pr_tp("============   HalTscrCDevWriteI2CSeq ;==========\n");
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
	pr_tp("============   HalTscrCDevWriteI2CSeq =========\n");

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
		pr_tp("============   HalTscrCDevWriteI2CSeq =========\n");
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133      c33:2133a(2138a)
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
		pr_tp("============ HalTscrCDevWriteI2CSeq (=========\n");
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
		pr_tp("============HalTscrCReadI2CSeq ( =========\n");
    pr_tp( "111dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        pr_tp( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
			pr_tp("\n========33====\n");
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
		}
        else{
			pr_tp("\n========32====\n");

            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
         pr_tp("\n========c2====\n");
        return firmware_update_c2 ( dev, attr, buf, size );
    }
}
//#else
//delete

/*static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    _HalTscrHWReset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
        pr_tp("============update %d===",i);
    }
    TP_DEBUG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;

    return size;
}*/
 static ssize_t firmware_update_store(struct device *dev,
struct device_attribute *attr, const char *buf, size_t size)
{
	

	U8 i;
	U8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	update_switch = 1;

	
	pr_ch("\n");
	//drvISP_EntryIspMode();
	//drvISP_BlockErase(0x00000);
	//M by cheehwa _HalTscrHWReset();

	pr_tp("firmware_update_store start\n");
	 
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);		
	msta_power(1);
	msleep(200);

	dbbusDWIICEnterSerialDebugMode();
	pr_ch("dbbusDWIICEnterSerialDebugMode\n");
	dbbusDWIICStopMCU();
	pr_ch("dbbusDWIICStopMCU\n");
	dbbusDWIICIICUseBus();
	pr_ch("dbbusDWIICIICUseBus\n");
	dbbusDWIICIICReshape();
	pr_ch("dbbusDWIICIICReshape\n");
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;
	pr_tp(" Enable slave's ISP ECO mode \n");
	// Disable the Watchdog
	pr_ch("Disable the Watchdog\n");
	i2c_write_msg2133(dbbus_tx_data, 4);
	//Get_Chip_Version();
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	dbbus_tx_data[3] = 0x00;
	pr_ch("Get_Chip_Version\n");
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	i2c_write_msg2133(dbbus_tx_data, 4);
	pr_ch("update\n");
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	i2c_write_msg2133(dbbus_tx_data, 4);
	pr_ch("Stop MCU\n");
	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	i2c_write_msg2133(dbbus_tx_data, 4);
	pr_ch("Enable SPI Pad\n");
	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	i2c_write_msg2133(dbbus_tx_data, 3);
	i2c_read_msg2133(&dbbus_rx_data[0], 2);
	pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	i2c_write_msg2133(dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	i2c_read_msg2133(&dbbus_rx_data[0], 2);
	pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	i2c_write_msg2133(dbbus_tx_data, 4);
	/*
	//------------
	// ISP Speed Change to 400K
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	i2c_write_msg2133( dbbus_tx_data, 3);
	i2c_read_msg2133( &dbbus_rx_data[3], 1);
	//pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);
	dbbus_tx_data[3] = dbbus_tx_data[3]&0xf7;  //Clear Bit3
	i2c_write_msg2133( dbbus_tx_data, 4);
	*/
	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	i2c_write_msg2133(dbbus_tx_data, 4);
	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	i2c_write_msg2133(dbbus_tx_data, 4);
	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();
	///////////////////////////////////////
	// Start to load firmware
	///////////////////////////////////////
	pr_tp("====================================================  Start to load firmware ===============================================\n");
	drvISP_EntryIspMode();
    pr_tp("======================================== entryisp======================================== \n");
	drvISP_BlockErase(0x00000);
	pr_tp("======================================== drvISP_BlockErase======================================== \n");
	msleep(500);
	pr_tp("FwVersion=2");
	updating = 1;
	for(i = 0; i < 59; i++)    // total  59 KB : 1 byte per R/W
	{

		pr_tp(" drvISP_Program%d\n" ,i+1);
		if (temp == NULL)
		{
		
			pr_tp("download_firmware_buf NULL==== \n");
			return 0;
		}
			
		drvISP_Program(i,&temp[i*1024]);
		msleep(10);
		//pr_tp("drvISP_Program%d %s\n",i,&download_firmware_buf[i*1024]);
		//pr_ch("drvISP_Verify\n");
		//drvISP_Verify ( i, download_firmware_buf[i*1024] ); //verify data
	}

	pr_tp(" update OK \n");
	drvISP_ExitIspMode();
	pr_tp("update OK2 \n");

	updating = 0;
	update_switch = 0;
		
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
	msleep(100);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT); 
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);  
	msleep(500);
	
	msg2133_read_fw_ver_custom();
	
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return size;
}
#endif
static DEVICE_ATTR(update, CTP_AUTHORITY, firmware_update_show, firmware_update_store);
#if 0
/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	pr_tp(" +++++++ [%s] Enter!++++++\n", __func__);
	u16 k=0,i = 0, j = 0;
	u8 bWriteData[5] =
	{
        0x10, 0x03, 0, 0, 0
	};
	u8 RX_data[256];
	u8 bWriteData1 = 0x12;
	u32 addr = 0;
	u32 timeOutCount=0;
	for (k = 0; k < 94; i++)   // total  94 KB : 1 byte per R/W
	{
		addr = k * 1024;
		for (j = 0; j < 8; j++)   //128*8 cycle
		{
			bWriteData[2] = (u8)((addr + j * 128) >> 16);
			bWriteData[3] = (u8)((addr + j * 128) >> 8);
			bWriteData[4] = (u8)(addr + j * 128);
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);

			timeOutCount=0;
			while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
			{
				timeOutCount++;
				if ( timeOutCount >= 100000 )
					break; /* around 1 sec timeout */
	  		}

			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);
			drvISP_Read(128, RX_data);
			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
			for (i = 0; i < 128; i++)   //log out if verify error
			{
				if (RX_data[i] != 0xFF)
				{
					//TP_DEBUG(pr_tp("k=%d,j=%d,i=%d===============erase not clean================",k,j,i);)
					pr_tp("k=%d,j=%d,i=%d  erase not clean !!",k,j,i);
				}
			}
		}
	}
	TP_DEBUG("read finish\n");
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{

	u8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	pr_tp(" +++++++ [%s] Enter!++++++\n", __func__);
	//msctpc_LoopDelay ( 100 ); 	   // delay about 100ms*****

	// Enable slave's ISP ECO mode

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;

	// Disable the Watchdog
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	dbbus_tx_data[3] = 0x00;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);

	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	//set FRO to 50M
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(pr_tp("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
	TP_DEBUG(pr_tp("chip erase+\n");)
    drvISP_BlockErase(0x00000);
	TP_DEBUG(pr_tp("chip erase-\n");)
    drvISP_ExitIspMode();
    return size;
}
static DEVICE_ATTR(clear, CTP_AUTHORITY, firmware_clear_show, firmware_clear_store);
#endif //0
/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/


static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    int i;
	TP_DEBUG_ERR("***FwDataCnt = %d ***\n", FwDataCnt);
   // for (i = 0; i < 1024; i++)
    {
        memcpy(temp[FwDataCnt], buf, 1024);
    }
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, CTP_AUTHORITY, firmware_data_show, firmware_data_store);
#ifdef ENABLE_AUTO_UPDATA
#define	CTP_ID_MSG21XX		0
#define	CTP_ID_MSG21XXA		1

unsigned char getchipType(void)
{
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    pr_tp("========getchipType()\n");

	_HalTscrHWReset();
    mdelay ( 100 );

	dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );

    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
	_HalTscrHWReset();
    if ( dbbus_rx_data[0] == 2 )
    {
    	return CTP_ID_MSG21XXA;
    }
    else
    {
    	return CTP_ID_MSG21XX;
    }

}

unsigned int msg2133a_read_fw_ver_custom(void)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;
    
    pr_tp("msg2133a_read_fw_ver_custom() major=%d, minor=%d\n", msg2133a_fm_major, msg2133a_fm_minor);

    // Jake.L, DATE20130528, fw read one at phone startup, DATE20130528-01 START
    if ((MSG2133_INVALID_VER != msg2133a_fm_major)
        && (MSG2133_INVALID_VER != msg2133a_fm_minor))
    {
        pr_tp("read before\n");
        
        return ((msg2133a_fm_major<<16)|(msg2133a_fm_minor));
    }
    // Jake.L, DATE20130528-01 END
    
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    // Jake.L, DATE20130527, wait for TP ready, DATE20130527-01 LINE
    mdelay(10);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

 	//add by liyaohua
 	msg2133a_fm_major = major;
	msg2133a_fm_minor = minor;
 	//add by liyaohua end
    TP_DEBUG_ERR("***FW Version major = %d ***\n", msg2133a_fm_major);
    TP_DEBUG_ERR("***FW Version minor = %d ***\n", msg2133a_fm_minor);

    return ((major<<16)|(minor));
}
static int fwAutoUpdate(void *unused)
{
    firmwarea_update_store(NULL, NULL, NULL, 0);

}
#endif

/* Add for device info. */
// tpd_info
static ssize_t firmware_tpdinfo_show(struct device *dev,
    struct device_attribute *attr, char *buf)
{
	pr_tp("*** firmware_version_show fw_version \n");
    if (msg2133chipid == CTP_ID_MSG21XXA)
    	return  sprintf(buf, "%s\n", "Mstar2133a");
    else
    	return  sprintf(buf, "%s\n", "Mstar2133");
}

static ssize_t firmware_tpdinfo_store(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t size)
{
    return 0;
}

static DEVICE_ATTR(tpd_info, CTP_AUTHORITY, firmware_tpdinfo_show, firmware_tpdinfo_store);

//version
static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    pr_tp("====================firmware_version_show fw_version ==================================***\n");

    if (msg2133chipid == CTP_ID_MSG21XXA)
        return sprintf(buf, "%d%d\n",msg2133a_fm_major,msg2133a_fm_minor);
    else
        return sprintf(buf, "%d%d\n",msg2133_fm_major,msg2133_fm_minor);
}


static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;
/*
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

*/
    fw_version = kzalloc(sizeof(char), GFP_KERNEL);

    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    TP_DEBUG_ERR("***major = %d ***\n", major);
    TP_DEBUG_ERR("***minor = %d ***\n", minor);
    sprintf(fw_version,"%03d%03d", major, minor);
    //TP_DEBUG(printk("***fw_version = %s ***\n", fw_version);)

    return size;
}
static DEVICE_ATTR(version, CTP_AUTHORITY, firmware_version_show, firmware_version_store);


#endif



#ifdef TP_PROXIMITY_SENSOR
/*static void i2c_write(u8 addr, u8 *pbt_buf, int dw_lenth)
{
	int ret;
	i2c_client->addr = addr;
	i2c_client->addr|=I2C_ENEXT_FLAG;
	ret = i2c_master_send(i2c_client, pbt_buf, dw_lenth);
	i2c_client->addr = TOUCH_ADDR_MSG21XX;
	i2c_client->addr|=I2C_ENEXT_FLAG;

	if(ret <= 0)
	{
		TPD_PROXIMITY_DMESG("i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
	}
}*/
static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	U8 ps_store_data[4];
	if (enable)
	{
		ps_store_data[0] = 0x52;
		ps_store_data[1] = 0x00;
		ps_store_data[2] = 0x4a;
		ps_store_data[3] = 0xa0;//0xa2//0xa4
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DMESG("*********open CTP TP_PROXIMITY_SENSOR 0xa0 \n");
	}
	else
	{
		ps_store_data[0] = 0x52;
		ps_store_data[1] = 0x00;
		ps_store_data[2] = 0x4a;
		ps_store_data[3] = 0xa1;
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DMESG("*********close CTP TP_PROXIMITY_SENSOR 0xa1 \n");
	}
	i2c_write(TOUCH_ADDR_MSG21XX, &ps_store_data[0], 4);
	return 0;
}

static int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
						  void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;

	switch (command)
	{
	case SENSOR_DELAY:
		if((buff_in == NULL) || (size_in < sizeof(int)))
		{
			err = -EINVAL;
		}
		// Do nothing
		break;

	case SENSOR_ENABLE:
		if((buff_in == NULL) || (size_in < sizeof(int)))
		{
			err = -EINVAL;
		}
		else
		{
			value = *(int *)buff_in;
			if(value)
			{
				if((tpd_enable_ps(1) != 0))
				{
					TPD_PROXIMITY_DMESG("enable ps fail: %d\n", err);
					return -1;
				}
			}
			else
			{
				if((tpd_enable_ps(0) != 0))
				{
					TPD_PROXIMITY_DMESG("disable ps fail: %d\n", err);
					return -1;
				}
			}
		}
		break;

	case SENSOR_GET_DATA:
		if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
		{
			TPD_PROXIMITY_DMESG("get sensor data parameter error!\n");
			err = -EINVAL;
		}
		else
		{
			sensor_data = (hwm_sensor_data *)buff_out;

			sensor_data->values[0] = tpd_get_ps_value();
			sensor_data->value_divide = 1;
			sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		}
		break;

	default:
		TPD_PROXIMITY_DMESG("proxmy sensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;

}
#endif
static  void tpd_down(int x, int y)
{

#ifdef TPD_HAVE_BUTTON
	  if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())  
		{
				tpd_button(x, y, 1);
				pr_tp("===================tpd_down: x=%d, y=%d\n\n", x, y);
			
		}
#endif

	pr_tp("===================tpd_down: x=%d, y=%d\n\n", x, y);

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_mt_sync(tpd->dev);
	TPD_DOWN_DEBUG_TRACK(x, y);
}
static  int tpd_up(int x, int y)
{
     	

	  if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
     	pr_tp("===================tpd_up button: x=%d, y=%d\n\n", x, y);
        tpd_button(x, y, 0); 
     }   		 
//add by liyaohua 
	 input_report_abs(tpd->dev, ABS_PRESSURE, 0);
	 input_report_key(tpd->dev, BTN_TOUCH, 0);
	// input_report_abs(tpd->dev,ABS_MT_TRACKING_ID,i);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	// mt6575_touch_debug("%s: x=%d, y=%d, p=%d--Liu\n", 	 __func__, x, y, 0);

	input_mt_sync(tpd->dev);
	//input_sync(tpd->dev);
	TPD_UP_DEBUG_TRACK(x, y);
	pr_tp("===================tpd_up x=%d, y=%d\n\n", x, y);
    
	return 1;
}
unsigned char tpd_check_sum(unsigned char *pval)
{
	int i, sum = 0;

	for(i = 0; i < 7; i++)
	{
		sum += pval[i];
	}

	return (unsigned char)((-sum) & 0xFF);
}
//static int tpd_touchinfo(struct touch_info *cinfo)
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)

{
	pr_tp("tpd_touchinfo---start!\n");

	SHORT_TOUCH_STATE ShortTouchState;
	BYTE reg_val[8] = {0};
	unsigned int  temp = 0;

	if(update_switch)
	{
		return false;
	}
	
	
//	memset(cinfo, 0, sizeof(struct touch_info));
	
	pr_tp("tpd_touchinfo---msg2133_i2c_readstart!\n");
	msg2133_i2c_read(reg_val, 8);
	pr_tp("tpd_touchinfo---msg2133_i2c_readend!\n");

	ShortTouchState.pos_x = ((reg_val[1] & 0xF0) << 4) | reg_val[2];
	ShortTouchState.pos_y = ((reg_val[1] & 0x0F) << 8) | reg_val[3];
	ShortTouchState.dst_x = ((reg_val[4] & 0xF0) << 4) | reg_val[5];
	ShortTouchState.dst_y = ((reg_val[4] & 0x0F) << 8) | reg_val[6];

	if((ShortTouchState.dst_x) & 0x0800)
	{
		ShortTouchState.dst_x |= 0xF000;
	}

	if((ShortTouchState.dst_y) & 0x0800)
	{
		ShortTouchState.dst_y |= 0xF000;
	}

	ShortTouchState.pos_x2 = ShortTouchState.pos_x + ShortTouchState.dst_x;
	ShortTouchState.pos_y2 = ShortTouchState.pos_y + ShortTouchState.dst_y;
	temp = tpd_check_sum(reg_val);

	pr_tp("tpd_touchinfo---msg2133_i2c_readendtemp %u  reg_val[7] %u !\n",temp,reg_val[7]);
	if(temp == reg_val[7])
	{
	  pr_tp("tpd_touchinfo---msg2133_i2c_readend reg_val[0] %u !\n",reg_val[0]);
		if(reg_val[0] == 0x52) //CTP  ID
		{
		 pr_tp("tpd_touchinfo---msg2133_i2c_readend reg_val[1] %u reg_val[4] %u !\n",reg_val[1],reg_val[4] );
			if(reg_val[1] == 0xFF&& reg_val[4] == 0xFF)
				
			{
			pr_tp("tpd_touchinfo---msg2133_i2c_readend reg_val[5] %u !\n",reg_val[5] );
		#ifdef TP_PROXIMITY_SENSOR
				if(reg_val[5] == 0x80) // close to
				{
					 pr_tp("tpd_touchinfo---msg2133_get_sensor_state read ps_data_state[0] 1!\n");
					ps_data_state[0] = 1;
					if(tfopensenor)
					{
					      pr_tp("@@@@@@@@@@@@@@@@tpd_touchinfo-- cy8ctst_alsps_do_work  0x80 start!\n");
						  cy8ctst_alsps_do_work();
						  pr_tp("@@@@@@@@@@@@@@@@tpd_touchinfo-- cy8ctst_alsps_do_work 0x80 end !\n");
					}

		        }
				else  if(reg_val[5] == 0x40) // leave
				{
					 pr_tp("tpd_touchinfo---msg2133_get_sensor_state read ps_data_state[0] 0!\n");
					ps_data_state[0] = 0;
					if(tfopensenor)
					{
					      pr_tp("@@@@@@@@@@@@@@@tpd_touchinfo-- cy8ctst_alsps_do_work start 0x40!\n");
						  cy8ctst_alsps_do_work();
						  pr_tp("@@@@@@@@@@@@@@@tpd_touchinfo-- cy8ctst_alsps_do_work 0x40 end !\n");
					}
				}
				else if(reg_val[5] == 0xC0) 
				{
					 //pr_tp("tpd_touchinfo---msg2133_get_sensor_state read ps_data_state[0] 0!\n");
					 	if( sensor_open || updating )
						{
							cy8ctst_sersor_snedC0();
						}
					
					 
				}
				else
		#endif
#ifdef TPD_HAVE_BUTTON
				if(reg_val[5] == 0x01)
				{
				     cinfo->x[0] = tpd_keys_dim_local[0][0];
				     cinfo->y[0] = tpd_keys_dim_local[0][1];
				     point_num = 1;
				}
				#if (TPD_KEY_COUNT>=2)
				else if(reg_val[5] == 0x02)
				{
				     cinfo->x[0] = tpd_keys_dim_local[1][0];
				     cinfo->y[0] = tpd_keys_dim_local[1][1];
				     point_num = 1;
				}
				#endif
				#if (TPD_KEY_COUNT>=3)
				else if(reg_val[5] == 0x04)
				{
				     cinfo->x[0] = tpd_keys_dim_local[2][0];
				     cinfo->y[0] = tpd_keys_dim_local[2][1];
				     point_num = 1;
				}
				#endif
				#if (TPD_KEY_COUNT>=4)
				else if(reg_val[5] == 0x08)
				{
				     cinfo->x[0] = tpd_keys_dim_local[3][0];
				     cinfo->y[0] = tpd_keys_dim_local[3][1];
				     point_num = 1;
				}
				#endif
				else
#endif
				{
					if(point_num ==2)
					{
						touch_up_num =2;
					}
					point_num = 0;
				}
			}
			else if(ShortTouchState.pos_x > 2047 || ShortTouchState.pos_y > 2047)
			{
				return  false;
			}
			else if((ShortTouchState.dst_x == 0) && (ShortTouchState.dst_y == 0))
			{
				//cinfo->x[0] = (ShortTouchState.pos_x * TPD_RES_X) / 2048;
				//cinfo->y[0] = (ShortTouchState.pos_y *TPD_RES_Y) / 2048;

				tempx= (ShortTouchState.pos_x * TPD_RES_X) / 2048;
				tempy = (ShortTouchState.pos_y *TPD_RES_Y) / 2048;

				
				if(point_num == 2)
				{
					if((ABS(cinfo->x[0]-tempx)<5)&&(ABS(cinfo->y[0]-tempy)<5))
					{
						 tpd_up(cinfo->x[1], cinfo->y[1]);
						
					}else
					{
						 tpd_up(cinfo->x[0], cinfo->y[0]);
					}
				}

				cinfo->x[0]= tempx;
				cinfo->y[0]= tempy;
				point_num = 1;
			}
			else
			{
				if(ShortTouchState.pos_x2 > 2047 || ShortTouchState.pos_y2 > 2047)
					return false;
				cinfo->x[0] = (ShortTouchState.pos_x * TPD_RES_X) / 2048;
				cinfo->y[0] = (ShortTouchState.pos_y *  TPD_RES_Y) / 2048;
				cinfo->x[1] = (ShortTouchState.pos_x2 * TPD_RES_X) / 2048;
				cinfo->y[1] = (ShortTouchState.pos_y2 * TPD_RES_Y) / 2048;
				point_num = 2;
			}
		}
		pr_tp("tpd_touchinfo---end true!\n");

		return true;
	}
	else
	{
	pr_tp("tpd_touchinfo---end false!\n");
		return  false;
	}

}

static int touch_event_handler(void *unused)
{
	struct touch_info cinfo,pinfo;;
   
    static unsigned short Pos_x = 0,Pos_y = 0;
	unsigned long time_eclapse;
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);
	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);

        pr_tp("========touch_event_handler()\n");
		if(tpd_touchinfo(&cinfo, &pinfo))
		{
			if(point_num == 1)
			{
				
				tpd_down(cinfo.x[0], cinfo.y[0]);	
				
				input_sync(tpd->dev);
			}
			else if(point_num == 2)
			{
	            tpd_down(cinfo.x[0], cinfo.y[0]);
	            tpd_down(cinfo.x[1], cinfo.y[1]);
				input_sync(tpd->dev);
			}
			else if(point_num == 0)
			{

			    tpd_up(cinfo.x[0], cinfo.y[0]);
				if(touch_up_num ==2)
				{
					tpd_up(cinfo.x[1], cinfo.y[1]);
					touch_up_num =0;
				}
		
				input_sync(tpd->dev);
			}
		}
		
	}
	while(!kthread_should_stop());

	return 0;
}

static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

static void tpd_eint_interrupt_handler(void)
{
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_tp("========cy8ctst_get_fw()=%d",cy8ctst_get_fw());
	if(-1!=cy8ctst_get_fw()) return -1;
#ifdef TP_PROXIMITY_SENSOR
	int err;
	struct hwmsen_object obj_ps;
#endif
	char buf[2];
	int reset_count = 0;

	i2c_client = client;
    i2c_client->addr |= I2C_ENEXT_FLAG; //I2C_HS_FLAG;
	i2c_client->timing = 100;

	msta_power(1);
	msg2133chipid =getchipType();
    
    if (CTP_ID_MSG21XXA == msg2133chipid)
    {
        msg2133a_read_fw_ver_custom();
    }
    else
    {
        msg2133_read_fw_ver_custom();
    }

	//pr_tp("========probe==msta_power1==");
	//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//pr_tp("========probe==msta_power2==");
	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//pr_tp("========probe==msta_power3==");
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE, tpd_eint_interrupt_handler, 1);
	//pr_tp("========probe==msta_power4==");
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	//pr_tp("========probe==msta_power5==");
	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if(IS_ERR(thread))
	{
//		retval = PTR_ERR(thread);
//		pr_tp(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
	}
   #ifdef __FIRMWARE_UPDATE__
	//firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
	firmware_class = class_create(THIS_MODULE, "mtk-tpd");
    if (IS_ERR(firmware_class))
        pr_err("Failed to create class(firmware)!\n");
    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
    if (IS_ERR(firmware_cmd_dev))
        pr_err("Failed to create device(firmware_cmd_dev)!\n");

    // tpd-info
    if (device_create_file(firmware_cmd_dev, &dev_attr_tpd_info) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_tpd_info.attr.name);
    // version
    if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	// clear
 //   if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
 //       pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);

	dev_set_drvdata(firmware_cmd_dev, NULL);
   /* #ifdef	ENABLE_AUTO_UPDATA
	TP_DEBUG_ERR("[TP] check auto updata\n");
	pr_tp("me======================================================\n");
	if(getchipType() == CTP_ID_MSG21XXA)
	{
	 pr_tp("====================CTP_ID_MSG21XXA==================================\n");
		//msg2133A
		pr_tp("[TP] TP IC is msg21xxA Version = %d \n", (getFWPrivateVersion()&0xff));

			if((getFWPrivateVersion()&0xff) < 4)
			{
			    int i = 0, j=0;
				TP_DEBUG_ERR("[TP] TP FW version is less than 4\n");

				for (i = 0; i < 33; i++)
    			{
    				for (j = 0; j < 1024; j++)
    				{
        				temp[i][j] = MstartCTP_FWData[i*1024+j];
    				}
    			}
				//firmware_update_store(NULL, NULL, NULL, 0);
				
				 pr_tp("====================CTP_ID_MSG21XXA==================================\n");
	kthread_run(fwAutoUpdate, 0, client->name);
			}
	}
#endif*/

#endif
//msg2133_read_fw_ver_custom();

#ifdef TP_PROXIMITY_SENSOR
	obj_ps.polling = 0;//interrupt mode
	obj_ps.sensor_operate = tpd_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		TPD_PROXIMITY_DMESG("zhuoshineng gt868 proximity attach fail = %d\n", err);
		//		goto exit_create_attr_failed;
	}
#endif
	//msg2133chipid =1;//getchipType();
	/*msg2133_ctpm_get_upg_ver_custom();
	msg2133a_ctpm_get_upg_ver_custom();
	getFWPrivateVersion();
	msg2133_read_fw_ver_custom();*/
	return 0;
}

static int __devexit tpd_remove(struct i2c_client *client)
{
	//   TPD_DEBUG("TPD removed\n");
	return 0;
}

static int tpd_local_init(void)
{
	TPD_DMESG("Mstar msg2133 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("msg2133 unable to add i2c driver.\n");
		return -1;
	}
	if(tpd_load_status == 0)
	{
		TPD_DMESG("msg2133 add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}

#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
#endif
	//add by liyaohua
	 wake_lock_init(&Tp_wake_lock,WAKE_LOCK_SUSPEND,"TP_WakeLock");
	//add by liyaohua

	//input_mt_init_slots(tpd->dev, TPD_MAX_POINTS);
	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);
	tpd_type_cap = 1;
	return 0;
}
//extern int cy8ctst_get_vendor_fw (void);
int msg2133_read_fw_ver_custom(void)

{
  int rc = 0;
  int icyfw=0;
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ;
	unsigned short major = 0, minor = 0;
	pr_tp("==================================================== msg2133_read_fw_ver_custom customupdate start ==========================\n");
    
    pr_tp("msg2133_read_fw_ver_custom() major=%d, minor=%d\n", msg2133_fm_major, msg2133_fm_minor);

    // Jake.L, DATE20130528, fw read one at phone startup, DATE20130528-01 START
    if ((MSG2133_INVALID_VER != msg2133_fm_major)
        && (MSG2133_INVALID_VER != msg2133_fm_minor))
    {
        pr_tp("read before\n");
        return msg2133_fm_major;        
    }
    // Jake.L, DATE20130528-01 END

	
	pr_ch("\n");
	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x74;
	rc = i2c_write(TOUCH_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
	if(rc<0)
		return -1;
	rc = i2c_read(TOUCH_ADDR_MSG21XX, &dbbus_rx_data[0], 4);
	msg2133_fm_major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
	msg2133_fm_minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
	pr_tp("***msg2133_fm_majormajor = %d ***\n", msg2133_fm_major);
	pr_tp("***msg2133_fm_majorminor = %d ***\n", msg2133_fm_minor);
	return msg2133_fm_major;
}
int msg2133_IIC(void)
{
	unsigned char dbbus_tx_data[3];
	dbbus_tx_data[0] = 0x00;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x00;
	return i2c_write(TOUCH_ADDR_MSG21XX, &dbbus_tx_data[0], 1);
}
/*int msg2133a_read_fw_ver_custom(void)

{
  int rc = 0;
  int icyfw=0;
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4] ;
	unsigned short major = 0, minor = 0;
	msg2133_fm_major = 100;
	pr_tp("==================================================== msg2133_read_fw_ver_custom customupdate start ==========================\n");

	
	pr_ch("\n");
	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = 0x00;
	dbbus_tx_data[2] = 0x74;
	rc = i2c_write(TOUCH_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
	if(rc<0)
		return -1;
	rc = i2c_read(TOUCH_ADDR_MSG21XX, &dbbus_rx_data[0], 4);
	msg2133a_fm_major = (dbbus_rx_data[1] << 8) + dbbus_rx_data[0];
	msg2133a_fm_minor = (dbbus_rx_data[3] << 8) + dbbus_rx_data[2];
	pr_tp("***msg2133_fm_majormajor = %d ***\n", msg2133_fm_major);
	pr_tp("***msg2133_fm_majorminor = %d ***\n", msg2133_fm_minor);
	return msg2133_fm_major;
}
*/
static int tpd_resume(struct i2c_client *client)
{
	pr_tp("==============TPD enter sleep tpd_resume  start===============\n");

	int retval = TPD_OK;
	 
	pr_tp("TPD wake up==%d==updating=%d.ig_b%d\n",sensor_open, updating, ig_batstate);

	if (updating) {
		pr_tp("============resume==sensor_open:%d || updating:%d===============\n",sensor_open , updating);
		return 1;
	}
    
	#if defined (TP_PROXIMITY_SENSOR)
	if ((ig_batstate) && (ps_data_state[0] == 1))//if(sensor_open)
	{
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        //Add by liyaohua 20121102
        pr_tp("TPD restart.\n");

        msta_power(0);
        {
            ps_data_state[0] = 0;
            pr_tp("@@@@@@@@@@@@@@@tpd_touchinfo-- cy8ctst_alsps_do_work start 0x40!\n");
            cy8ctst_alsps_do_work();
            pr_tp("@@@@@@@@@@@@@@@tpd_touchinfo-- cy8ctst_alsps_do_work 0x40 end !\n");
        }			
        msleep(10);
        msta_power(1);

        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
        // Jake.L, DATE20130528, NOTE, DATE20130528-01 LINE
        if (sensor_open)
            msg2133_open_sersor(1);		
    }     
	#endif 
        
	if(!ig_batstate)
                {           
                    msta_power(1);          
                    //mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);         
        
    	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	}
#ifdef TP_PROXIMITY_SENSOR
	if((!ig_sensor)&&(!sensor_open))
	{
		pr_tp("==============msg2133_open_sersor======1\n");
	  	msg2133_open_sersor(1);	
	}
#endif


	return retval;
}
static int tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	
	int retval = TPD_OK;
	static char data = 0x3;
	pr_tp("TPD enter sleep\n");

    
	if( sensor_open || updating ) {
		//mt6575_touch_info("%s: process_cmd=%d, updating=%d, sensor_open=%d--Liu\n", __func__,  sensor_open);
		pr_tp("==============sensor_open:%d || updating:%d===============\n",sensor_open , updating);
		return 1;
	}

#ifdef TP_FIRMWARE_UPDATE
	if(update_switch==0)
#endif
	{
		if(ig_batstate)
		{
			#ifdef TP_PROXIMITY_SENSOR
				if(cy8ctst_alsps_get_openstate())
				{
					pr_tp("==============msg2133_open_sersor======0\n");
					msg2133_open_sersor(0);
				}
			#endif
			mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);	
			msta_power(0);
		}		
	}
	return retval;
}

static struct tpd_driver_t tpd_device_driver =
	{
	.tpd_device_name = "msg2133",
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
	//  pr_tp("MediaTek msg2133 touch panel driver init\n");
	i2c_register_board_info(0, &msg2133_i2c_tpd, 1);
	if(tpd_driver_add(&tpd_device_driver) < 0) {
		TPD_DMESG("add msg2133 driver failed\n");
	}
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek msg2133 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}
ssize_t mem_write(struct file *filp, const char *buf, size_t size, loff_t *lofp)
{
    int res = -1;
    char *tmp;
    int imemcy =0 ;
    
    pr_tp("<0>read data from the user space\n");
    tmp = mem_spvm;
    if (size > MEM_MALLOC_SIZE)    
        size = MEM_MALLOC_SIZE;

    if (tmp != NULL)
        res = copy_from_user(tmp, buf, size);
    
    if (res == 0)
    {
        pr_tp("=======<0>read data success and the data is22222222222:%s=================\n",buf);
        pr_tp("====cy8ctst_get_fw()==%d===\n",cy8ctst_get_fw());

        //add by liyaohua 
        if (0 == strcmp(buf,"I"))
        {      
            pr_tp("====cy8ctst_get_fw()==%d===\n",cy8ctst_get_fw());
            if (-1 != cy8ctst_get_fw())
            {
                pr_tp("==========cy8ctst_get_fw()=============\n");
                strcpy(mem_spvm,"-1");
                return 0;
            }		
            //pr_tp("====cy8ctst_get_fw()==%d===\n",cy8ctst_get_fw());
            pr_tp("================================IMsg2133Version start ====================================\n");
            pr_tp("mem_write() msg2133chipid=%d\n", msg2133chipid);
            if (-1 == msg2133chipid)
                msg2133chipid = getchipType();
            if (msg2133chipid == CTP_ID_MSG21XXA)
            {
                msg2133a_read_fw_ver_custom();
                msg2133a_ctpm_get_upg_ver_custom();
                pr_tp("====msg2133a_fm_major===%d===msg2133a_fm_minor===%d==\n",msg2133a_fm_major,msg2133a_fm_minor);
                pr_tp("====msg2133a_bin_major===%d=====msg2133a_bin_minor==%d=\n",msg2133a_bin_major,msg2133a_bin_minor);
                
                //if ((msg2133a_fm_major != msg2133a_bin_major)
                //    || (msg2133a_fm_minor != msg2133a_bin_minor))
                if ((msg2133a_fm_major < msg2133a_bin_major)
                    || ((msg2133a_fm_major == msg2133a_bin_major)&&(msg2133a_fm_minor < msg2133a_bin_minor)))
                {
                    pr_tp("==============IMsg2133aVersion 0 \n");
					//strcpy(mem_spvm,"0");
					strcpy(mem_spvm,"1");
                }
                else
                {
                    pr_tp("==============IMsg2133aVersion 1\n");
					//strcpy(mem_spvm,"1");
					strcpy(mem_spvm,"0");
                }
            }
            else
            {
                msg2133_read_fw_ver_custom();
                msg2133_ctpm_get_upg_ver_custom();
                pr_tp("====msg2133_fm_major===%d====msg2133_fm_minor==%d\n",msg2133_fm_major,msg2133_fm_minor);
                pr_tp("====msg2133_bin_major===%d====msg2133_bin_minor==%d==\n",msg2133_bin_major,msg2133_bin_minor);
                
                //if ((msg2133_fm_major != msg2133_bin_major)
                //    || (msg2133_fm_minor != msg2133_bin_minor))
                if ((msg2133_fm_major < msg2133_bin_major)
                    || ((msg2133_fm_major == msg2133_bin_major)&&(msg2133_fm_minor < msg2133_bin_minor)))
                {
                    pr_tp("===============IMsg2133Version 0 \n");
					//strcpy(mem_spvm,"0");
					strcpy(mem_spvm,"1");
                }
                else
                {
                    pr_tp("================IMsg2133Version 1\n");
					//strcpy(mem_spvm,"1");
					strcpy(mem_spvm,"0");
                }
            }
        }

        if (0 == strcmp(buf,"V"))
        {
            char str_version[16] = {0};
            if (-1 != cy8ctst_get_fw())
            {
                pr_tp("==========cy8ctst_get_fw()=============\n");
                strcpy(mem_spvm,"-1");
                return 0;
            }
            
            if (-1 == msg2133chipid )
                msg2133chipid = getchipType();
            if (msg2133chipid == CTP_ID_MSG21XXA)
            {
                msg2133a_ctpm_get_upg_ver_custom();
                msg2133a_read_fw_ver_custom();
                pr_tp("====msg2133a_fm_major===%d===\n",msg2133a_fm_major);
                pr_tp("====msg2133a_fm_minor===%d===\n",msg2133a_fm_minor);
                pr_tp("====msg2133_fm_major===%d===\n",msg2133a_bin_major);
                pr_tp("====msg2133_bin_minor==%d===\n",msg2133a_bin_minor);
                
                pr_tp("IMsg2133Version 0 \n");
                snprintf(str_version,16,"msg2133a%d,%d",msg2133a_fm_major,msg2133a_fm_minor);
                strcpy(mem_spvm,str_version);
                pr_tp("================================version%s====================================\n",str_version);	
            }
            else
            {
                msg2133_ctpm_get_upg_ver_custom();
                msg2133_read_fw_ver_custom();
                pr_tp("====msg2133_fm_major===%d===\n",msg2133_fm_major);
                pr_tp("====msg2133_fm_minor===%d===\n",msg2133_fm_minor);
                pr_tp("====msg2133_fm_major===%d===\n",msg2133_bin_major);
                pr_tp("====msg2133_bin_minor==%d===\n",msg2133_bin_minor);
                
                pr_tp("IMsg2133Versiona  \n");
                snprintf(str_version,16,"msg2133%d,%d",msg2133_fm_major,msg2133_fm_minor);
                strcpy(mem_spvm,str_version);
                pr_tp("================================version%s====================================\n",str_version);	
            }	
            return 0;
        }
        
        if (0 == strcmp(buf,"C"))
        {
            char str_version[10]={0};
            if (-1 != cy8ctst_get_fw())
            {
                pr_tp("==========cy8ctst_get_fw()=============\n");
                snprintf(str_version,10,"%d",cy8ctst_FTM_Get_Fw());
                strcpy(mem_spvm,str_version);
                return 0;
            }	
            return 0;	
        }

        if (0 == strcmp(buf,"O"))
        {
            int itpup =0;
            mutex_lock(&readtp_mutex);
            itpup =  g_update;
            mutex_unlock(&readtp_mutex);
            pr_tp("================================OK====================================\n");
            return sprintf(mem_spvm,"%d",itpup);
        }

        if (0 == strcmp(buf,"X"))
        { //imemcy =0;

            mutex_lock(&tp_mutex);
            pr_tp("=XXXXXXXXXXXXXXX %s==\n",buf);

            //#ifdef TP_FIRMWARE_UPDATE

            //#ifdef POWERUP_AUTO_CHECK			

            pr_tp("=tpd_resume=msg2133_read_fw_ver_custom%d\n",msg2133_fm_major);			
            updating =1;		 
            pr_tp("====================================================ifwupdate===============================================\n");
            /*msg2133chipid =getchipType();
                    CTP_ID_MSG21XXA
                    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
                    msta_power(0);
                    msleep(300);

                    msta_power(1);*/
            mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);	
            if (-1 == msg2133chipid )
                pr_tp("================================msg2133chipid %d====================================\n",getchipType());
            else
                pr_tp("================================msg2133chipid %d====================================\n",msg2133chipid);
            
            if(msg2133chipid == CTP_ID_MSG21XXA)
            //download_firmware_buf=CTPM_FW;
            {
                int i,j;
                for (i = 0; i < 33; i++)
                {
                    for (j = 0; j < 1024; j++)
                    {
                        temp[i][j] = MstartCTP_FWData[i*1024+j];
                    }
                }
                //firmware_update_store(NULL, NULL, NULL, 0);

				 pr_tp("====================CTP_ID_MSG21XXA==================================\n");
                // kthread_run(fwAutoUpdate, 0, "adsf");
                firmwarea_update_store(NULL, NULL, NULL, 0);
            }
            else
            {
                int i,j;
                for (i = 0; i < 94; i++)
                {
                    for (j = 0; j < 1024; j++)
                    {
                        temp[i][j] = CTPM_FW[i*1024+j];
                    }
                }
                //firmware_update_store(NULL, NULL, NULL, 0);
                firmware_update_c2(NULL, NULL, NULL, 0);

				 pr_tp("====================CTP_ID_MSG21XX==================================\n");	
            }

            //firmware_update_store(NULL,NULL,NULL,0);

            pr_tp("tpd_resume2\n");

            //mutex_unlock(&tp_mutex);
            updating =0;
            mutex_lock(&readtp_mutex);
            g_update = 1;
            mutex_unlock(&readtp_mutex);
            //#endif
            //#endif
            //add by liyaohua end 
            strcpy(mem_spvm,"EEEEEEEEEEEEE");
            mutex_unlock(&tp_mutex);
        }

        return size;
    }
    else
    {
        pr_tp("<0>read data from user space fail\n");
        return 0;
    }
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
module_init(register_unregister_chrdev_init);   
module_exit(register_unregister_chrdev_exit);   


