#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/miscdevice.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/time.h>

#include "cy8ctst242.h"
#include "cy8ctst242_cypress.h"

#define CY8CTST_ISSP_MAGIC 			'C'
#define CY8CTST_ISSP_CHECK_FW 		_IOWR(CY8CTST_ISSP_MAGIC, 0, long long)
#define CY8CTST_ISSP_START 				_IOWR(CY8CTST_ISSP_MAGIC, 1, long long)
#define CY8CTST_ISSP_VERIFY_ID 			_IOWR(CY8CTST_ISSP_MAGIC, 2, long long)
#define CY8CTST_ISSP_ERASE_FLASH		_IOWR(CY8CTST_ISSP_MAGIC, 3, long long)
#define CY8CTST_ISSP_WRITE_FLASH		_IOWR(CY8CTST_ISSP_MAGIC, 4, long long)
#define CY8CTST_ISSP_VERIFY_WRITE		_IOWR(CY8CTST_ISSP_MAGIC, 5, long long)
#define CY8CTST_ISSP_SECURE_FLASH		_IOWR(CY8CTST_ISSP_MAGIC, 6, long long)
#define CY8CTST_ISSP_VERIFY_SECURITY	_IOWR(CY8CTST_ISSP_MAGIC, 7, long long)
#define CY8CTST_ISSP_CHECKSUM			_IOWR(CY8CTST_ISSP_MAGIC, 8, long long)
#define CY8CTST_ISSP_LAST				_IOWR(CY8CTST_ISSP_MAGIC, 9, long long)
#define CY8CTST_ISSP_LED_SHOW			_IOWR(CY8CTST_ISSP_MAGIC, 10, long long)

struct cy8ctst_issp_ctxt
{
	int opened;
	struct mutex lock;
	uint32_t checksum_data;
	uint32_t checksum_target;
	mm_segment_t old_fs; 
	int led_show;
};

static struct cy8ctst_issp_ctxt *_cy8ctst_issp;
unsigned char target_data_in;
unsigned char target_data_out[TARGET_DATABUFF_LEN];

unsigned char target_address;
unsigned char target_data_ptr = 0;
unsigned char target_id[10];
unsigned char target_status[10];
int debug = 1;
int check = 0;
int updating = 0;
static short param = 1;
mm_segment_t old_fs;   
#define HEX_FILE_PATH "/system/etc/cy8ctst.hex"
#define LASTEST_VERSION 2

int cy8ctst_load_program_data(unsigned char bank_num, unsigned char block_num, struct file *filp)
{
    unsigned char buffer[TARGET_DATABUFF_LEN];
    unsigned char temp[2];
    int retval, i = 0, index = 0;

    retval = filp->f_op->read(filp, buffer, 9, &filp->f_pos);

    for (i = 0; i < 64; i++, index++)
    {
        retval = filp->f_op->read(filp, temp, 2, &filp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;

        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);    
    }
    msleep(1);

    retval = filp->f_op->read(filp, buffer, 4, &filp->f_pos);

    retval = filp->f_op->read(filp, buffer, 9, &filp->f_pos);
    
    for (i = 0; i < 64; i++, index++)
    {
        retval = filp->f_op->read(filp, temp, 2, &filp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;

        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);    
    }
    msleep(1);

    retval = filp->f_op->read(filp, buffer, 4, &filp->f_pos);

    if (debug)
    {
        mt6575_touch_info("read data from file:\n");
        for (i = 0; i < TARGET_DATABUFF_LEN; i++)
        {
            mt6575_touch_info("%02X ", target_data_out[i]);
        }
        mt6575_touch_info("\n");
    }
    return PASS;
}

int cy8ctst_load_security_data(unsigned char bank_num, struct file *filp)
{ 
    unsigned char buffer[TARGET_DATABUFF_LEN];
    unsigned char temp[2];
    int retval, i = 0, index = 0;

    for (i = 0; i < BLOCKS_PER_BANK; i++)
    {
        retval = filp->f_op->read(filp, buffer, 71, &filp->f_pos);
        retval = filp->f_op->read(filp, buffer, 70, &filp->f_pos);
        retval = filp->f_op->read(filp, buffer, 71, &filp->f_pos);
        retval = filp->f_op->read(filp, buffer, 70, &filp->f_pos);
        msleep(1);
    }

    retval = filp->f_op->read(filp, buffer, 17, &filp->f_pos);

    retval = filp->f_op->read(filp, buffer, 9, &filp->f_pos);

    for (i = 0; i < 64; i++, index++)
    {
        retval = filp->f_op->read(filp, temp, 2, &filp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;
      
        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);   
    }
    msleep(1);

    retval = filp->f_op->read(filp, buffer, 4, &filp->f_pos);

    retval = filp->f_op->read(filp, buffer, 9, &filp->f_pos);
    
    for (i = 0; i < 64; i++, index++)
    {
        retval = filp->f_op->read(filp, temp, 2, &filp->f_pos);

        if (((temp[0] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = (temp[0] & 0x0F) << 4;
        
        if (((temp[0] & 0xF0) >> 4) == 0x06 || ((temp[0] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = ((temp[0] & 0x0F) + 9) << 4;

        if (((temp[1] & 0xF0) >> 4) == 0x03)
            target_data_out[index] = target_data_out[index] | (temp[1] & 0x0F);
        
        if (((temp[1] & 0xF0) >> 4) == 0x06 || ((temp[1] & 0xF0) >> 4) == 0x04)
            target_data_out[index] = target_data_out[index] | ((temp[1] & 0x0F) + 9);   
    }
    msleep(1);

    retval = filp->f_op->read(filp, buffer, 4, &filp->f_pos);

    if (debug)
    {
        mt6575_touch_info("read data from file:\n");
        for (i = 0; i < TARGET_DATABUFF_LEN; i++)
        {
            mt6575_touch_info("%02X ", target_data_out[i]);
        }
        mt6575_touch_info("\n");
    }

    mt6575_touch_info("#");

    return PASS;
}

// ============================================================================
// Description: 
// Run Clock without sending/receiving bits. Use this when transitioning from 
// write to read and read to write "num_cycles" is number of SCLK cycles, not
// number of counter cycles.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
//
// ============================================================================
void cy8ctst_run_clock(unsigned int num_cycles)
{
    int i;

    for (i = 0; i < num_cycles; i++) 
    {
        mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
        mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ONE);
    }
}

// ============================================================================
// Clocks the SCLK pin (high-low-high) and reads the status of the SDATA pin
// after the rising edge.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if SDATA was low
//     1 if SDATA was high
// ============================================================================
unsigned char cy8ctst_receive_bit(void)
{
    mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
    mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ONE);
    return mt_get_gpio_in(CYPRESS_I2C_SDA);
}          

// ============================================================================
// Calls ReceiveBit 8 times to receive one byte.
// Returns:
//     The 8-bit values recieved.
// ============================================================================
unsigned char cy8ctst_receive_byte(void)
{
    unsigned char b;
    unsigned char curr_byte = 0x00;

    for (b = 0; b < 8; b++) 
    {
        curr_byte = (curr_byte << 1) + cy8ctst_receive_bit();
    }
    return curr_byte;
}          

// ============================================================================
// This routine sends up to one byte of a vector, one bit at a time.
//    bCurrByte   the byte that contains the bits to be sent.
//    bSize       the number of bits to be sent. Valid values are 1 to 8.
//l
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
// ============================================================================
void cy8ctst_send_byte(unsigned char curr_byte, unsigned char size)
{
    unsigned char b = 0;

    for (b = 0; b < size; b++) 
    {
        if (curr_byte & 0x80) 
        {
            mt_set_gpio_out(CYPRESS_I2C_SDA, GPIO_OUT_ONE); 
            mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ONE);
            mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
        }
        else 
        {
            mt_set_gpio_out(CYPRESS_I2C_SDA, GPIO_OUT_ZERO); 
            mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ONE);
            mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
        }
        curr_byte = curr_byte << 1;
    }
}

// ============================================================================
// SendVector()
// This routine sends the vector specifed. All vectors constant strings found
// in ISSP_Vectors.h.  The data line is returned to HiZ after the vector is
// sent.
//    bVect      a pointer to the vector to be sent.
//    nNumBits   the number of bits to be sent.
//    bCurrByte  scratch var to keep the byte to be sent.
//
// There is no returned value.
// ============================================================================
void cy8ctst_send_vector(const unsigned char* vect, unsigned int num_bits)
{
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);
    while (num_bits > 0)
    {
        if (num_bits >= 8) 
        {
            cy8ctst_send_byte(*(vect), 8);
            num_bits -= 8;
            vect++;
        }
        else 
        {
            cy8ctst_send_byte(*(vect), num_bits);
            num_bits = 0;
        }
    }
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_IN);
}


// ============================================================================
// Waits for transition from SDATA = 1 to SDATA = 0.  Has a 100 msec timeout.
// TRANSITION_TIMEOUT is a loop counter for a 100msec timeout when waiting for
// a high-to-low transition. This is used in the polling loop of 
// fDetectHiLoTransition(). The timing of the while(1) loops can be calculated
// and the number of loops is counted, using iTimer, to determine when 100 
// msec has passed.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some 
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency 
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if successful
//    -1 if timed out.
// ============================================================================
signed char cy8ctst_detect_hi_lo_transition(void)
{
    unsigned int n;
    static unsigned int iTimer;
    
    mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
    
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);
    mt_set_gpio_out(CYPRESS_I2C_SDA, GPIO_OUT_ZERO);
    
    mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ONE);
    mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
    
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_IN);

    msleep(50);
    
    return PASS;
}

signed char cy8ctst_detect_hi_lo_transition_ext(void)
{
    unsigned int n;
    static unsigned int iTimer;
    unsigned char HiLo_result=0;
    unsigned TRANSITION_TIMEOUT1= 0x100000  ; 	//6740
	iTimer = TRANSITION_TIMEOUT1;
	
	while (1) {
		//SCLKLow();
		 mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
		 mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_IN);
		 if (mt_get_gpio_in(CYPRESS_I2C_SDA))
			 break; //HiLo_result = 1;
		else
			 HiLo_result = 0;
		mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ONE);  //SCLKHigh();
				
		// If the wait is too long then timeout
		if (iTimer-- == 0) {
			return FAIL;
		}
	}
	// Generate Clocks and wait for Target to pull SDATA Low again
	iTimer = TRANSITION_TIMEOUT1;	// reset the timeout counter

	while (1) {
		//SCLKLow();
		 mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
		 mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_IN);
		 if (mt_get_gpio_in(CYPRESS_I2C_SDA))
			 break; //HiLo_result = 1;
		else
			 HiLo_result = 0;
		mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ONE);  //SCLKHigh();
				
		// If the wait is too long then timeout
		if (iTimer-- == 0) {
			return FAIL;
		}
	}
	
	mt6575_touch_info(KERN_ERR"fDetectHiLoTransition OUT!!!!\n");
	return PASS;
    
    
}

int cy8ctst_switch_to_issp(void)
{
	mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_IN);
	mt_set_gpio_dir(CYPRESS_I2C_SCL, GPIO_DIR_OUT);
	mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);

	mt_set_gpio_dir(CYPRESS_I2C_RST, GPIO_DIR_OUT);
	mt_set_gpio_out(CYPRESS_I2C_RST, GPIO_OUT_ONE);
	mdelay(15);
	mt_set_gpio_out(CYPRESS_I2C_RST, GPIO_OUT_ZERO); 
	udelay(1);//udelay(10);

	//  The timing spec that requires that the first Init-Vector happen within
	//  1 msec after the reset/power up. For this reason, it is not advisable
	//  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the 
	//  Init-Vector instructions below. Doing so could introduce excess delay
	//  and cause the target device to exit ISSP Mode.

	//cy8ctst_send_vector(id_setup_1, num_bits_id_setup_1); 
	cy8ctst_send_vector(setup1_v_kryp, num_bits_setup1_kryp); 
	mdelay(15);
	if (!cy8ctst_detect_hi_lo_transition()) 
	{
	    mt6575_touch_info("%s: fail--Liu\n", __func__);
	    //return FAIL;
	}
	cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
	mdelay(15);
	return PASS;
}

int cy8ctst_verify_id(void)
{
    cy8ctst_send_vector(id_setup_2, num_bits_id_setup_2);
    if (!cy8ctst_detect_hi_lo_transition()) 
    {
        mt6575_touch_info("send id_setup_2 fail\n");
        //return FAIL;
    }
    cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end); 
    
    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);	

    //Send Read ID vector and get Target ID
    cy8ctst_send_vector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    cy8ctst_run_clock(2);                    // Two SCLK cycles between write & read
    target_id[0] = cy8ctst_receive_byte();
    cy8ctst_run_clock(1);
    cy8ctst_send_vector(read_id_v + 2, 12);    // 1+11 bits starting from the 3rd byte

    cy8ctst_run_clock(2);                    // Read-LSB Command
    target_id[1] = cy8ctst_receive_byte();

    cy8ctst_run_clock(1);
    cy8ctst_send_vector(read_id_v + 4, 1);     // 1 bit starting from the 5th byte
    
    //read Revision ID from Accumulator A and Accumulator X
    cy8ctst_send_vector(read_id_v + 5, 11);	//11 bits starting from the 6th byte
    cy8ctst_run_clock(2);
    target_id[2] = cy8ctst_receive_byte();	//Read from Acc.X
    cy8ctst_run_clock(1);
    cy8ctst_send_vector(read_id_v + 7, 12);    //1+11 bits starting from the 8th byte
    
    cy8ctst_run_clock(2);
    target_id[3] = cy8ctst_receive_byte();	//Read from Acc.A
    
    cy8ctst_run_clock(1);
    cy8ctst_send_vector(read_id_v + 4, 1);     //1 bit starting from the 5th byte,
    
    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);

    mt6575_touch_info("target_id   = %02X %02X\n", target_id[0], target_id[1]);
    mt6575_touch_info("target_id_v = %02X %02X\n", target_id_v[0], target_id_v[1]);

    //mt6575_touch_info("#");

    if (target_id[0] != target_id_v[0] || target_id[1] != target_id_v[1])
    {
        return FAIL;
    }
    else 
    {
        return PASS;
    }
}

int cy8ctst_read_status(void)
{
    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);

    //Send Read ID vector and get Target ID
    cy8ctst_send_vector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    cy8ctst_run_clock(2);                    // Two SCLK cycles between write & read
    target_status[0] = cy8ctst_receive_byte();
    cy8ctst_run_clock(1);
    cy8ctst_send_vector(read_id_v+2, 12);    // 12 bits starting from the 3rd character

    cy8ctst_run_clock(2);                    // Read-LSB Command
    target_status[1] = cy8ctst_receive_byte();

    cy8ctst_run_clock(1);
    cy8ctst_send_vector(read_id_v+4, 1);     // 1 bit starting from the 5th character

    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);			
    
    if (target_status[0] == target_status00_v) 
    {
        return PASS;     
    }
    if (target_status[0] == target_status01_v) 
    {
        return FAIL; 
    }
    if (target_status[0] == target_status03_v) 
    {
        return FAIL;
    }
    if (target_status[0] == target_status04_v) 
    {
        return FAIL; 
    }
    if (target_status[0] == target_status06_v) 
    {
        return FAIL;
    }
    else 
    {
        return FAIL;
    }
}

int cy8ctst_erase_target(void)
{
    cy8ctst_send_vector(erase, num_bits_erase);
    //if (!cy8ctst_detect_hi_lo_transition()) 
    if(!cy8ctst_detect_hi_lo_transition_ext())
    {
        mt6575_touch_info("send erase fail\n");
        //return FAIL;
    }
    cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return PASS;
}

// ============================================================================
// Transfers data from array in Host to RAM buffer in the target.
// Returns the checksum of the data.
// ============================================================================
unsigned int cy8ctst_load_target(void)
{
    unsigned char temp;
    unsigned int  checksum_data = 0;

    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);	

    cy8ctst_send_vector(read_write_setup, num_bits_read_write_setup);

    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);

    // Transfer the temporary RAM array into the target.
    // In this section, a 128-Byte array was specified by #define, so the entire
    // 128-Bytes are written in this loop.
    target_address = 0x00;
    target_data_ptr = 0x00;
    
    while (target_data_ptr < TARGET_DATABUFF_LEN) 
    {   
        temp = target_data_out[target_data_ptr]; //PROGRAM_DATA;
        checksum_data += temp;

        cy8ctst_send_byte(write_byte_start,4);    // we need to be able to write 128 bytes from address 0x80 to 0xFF  
        cy8ctst_send_byte(target_address, 7);	 // we need to be able to write 128 bytes from address 0x80 to 0xFF 
        cy8ctst_send_byte(temp, 8);
        cy8ctst_send_byte(write_byte_end, 3);
        
        // SendByte() uses MSbits, so inc by '2' to put the 0..128 address into
        // the seven MSBit locations.
        //
        // This can be confusing, but check the logic:
        //   The address is only 7-Bits long. The SendByte() subroutine will
        // send however-many bits, BUT...always reads them bits from left-to-
        // right. So in order to pass a value of 0..128 as the address using
        // SendByte(), we have to left justify the address by 1-Bit.
        //   This can be done easily by incrementing the address each time by
        // '2' rather than by '1'.

        target_address += 2;			// inc by 2 in order to support a 128 byte address space
        target_data_ptr++; 
    }
    msleep(1);
    
    return checksum_data;
}

// ============================================================================
// Program one block with data that has been loaded into a RAM buffer in the 
// target device.
// ============================================================================
int cy8ctst_program_target_block(unsigned char bBankNumber, unsigned char bBlockNumber)
{
    // TSYNC should still be set when entering this function so this call is not necessary but added for insurance
    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);	

    cy8ctst_send_vector(set_block_num, num_bits_set_block_num);
	
    // Set the drive here because SendByte() does not.
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);
    cy8ctst_send_byte(bBlockNumber,8);
    cy8ctst_send_byte(set_block_num_end, 3);
    
    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);	

    cy8ctst_send_vector(program_and_verify, num_bits_program_and_verify);		

    if (!cy8ctst_detect_hi_lo_transition()) 
    {
        mt6575_touch_info("send program_and_verify fail\n");
        //return FAIL;
    }
    // Send the Wait-For-Poll-End vector
    cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return PASS;
}

// ============================================================================
// Reads and adds the target bank checksum to the referenced accumulator.
// ============================================================================
int cy8ctst_target_bank_checksum(unsigned int* acc)
{
    unsigned char msb = 0;
    unsigned char lsb = 0;
	
    unsigned int n;
	
    cy8ctst_send_vector(checksum_setup, num_bits_checksum_setup);
    if (!cy8ctst_detect_hi_lo_transition()) 
    {					
        mt6575_touch_info("send checksum_setup fail\n");
        //return FAIL;		
    }

	msleep(250);

    cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end); 
   
    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);			

    //Send Read Checksum vector and get Target Checksum
    cy8ctst_send_vector(read_checksum_v, 11);     // first 11-bits is ReadCKSum-MSB		
    cy8ctst_run_clock(2);                         // Two SCLKs between write & read
    msb = cy8ctst_receive_byte();		 
    cy8ctst_run_clock(1);                         // See Fig. 6
    cy8ctst_send_vector(read_checksum_v + 2, 12); // 12 bits starting from 3rd character	
    cy8ctst_run_clock(2);                         // Read-LSB Command
    lsb = cy8ctst_receive_byte();		 

    cy8ctst_run_clock(1);
    cy8ctst_send_vector(read_checksum_v + 3, 1);  // Send the final bit of the command	
    
    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);			
    
    *acc = (msb << 8) | lsb;  // combine the MSB and the LSB
	
    return PASS;    
}    


// ============================================================================
// After programming, the target PSoC must be reset to take it out of 
// programming mode. This routine performs a reset.
// ============================================================================
void cy8ctst_restart_target(void)
{
	mt_set_gpio_out(CYPRESS_I2C_RST, GPIO_OUT_ONE);
	udelay(XRES_CLK_DELAY);
	mt_set_gpio_out(CYPRESS_I2C_RST, GPIO_OUT_ZERO);
}

// ============================================================================
// Verify the block just written to. This can be done byte-by-byte before the
// protection bits are set.
// ============================================================================
int cy8ctst_verify_setup(unsigned char bank_number, unsigned char block_number)
{
    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);	

    cy8ctst_send_vector(read_write_setup, num_bits_read_write_setup);
	
    cy8ctst_send_vector(set_block_num, num_bits_set_block_num);					
	
    //Set the drive here because SendByte() does not
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);
    cy8ctst_send_byte(block_number,8);
    cy8ctst_send_byte(set_block_num_end, 3);					
    
    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);	
    
    cy8ctst_send_vector(verify_setup, num_bits_my_verify_setup);	

	if (!cy8ctst_detect_hi_lo_transition()) 
	{
	    mt6575_touch_info("send verify_setup fail\n");
        //return FAIL;		
    }
    cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);     

    return PASS;
}

// ============================================================================
// Reads the data back from Target SRAM and compares it to expected data in
// Host SRAM
// ============================================================================
int cy8ctst_read_byte_loop(void)
{
    target_address = 0;
    target_data_ptr = 0;
	
    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);	

    cy8ctst_send_vector(read_write_setup, num_bits_read_write_setup);

    mt6575_touch_info("cy8ctst_read_byte_loop:\n");
    while(target_data_ptr < TARGET_DATABUFF_LEN) 
    {
        //Send Read Byte vector and then get a byte from Target
        cy8ctst_send_vector(read_byte_v, 4);					
        
        mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);
        cy8ctst_send_byte(target_address,7);

        cy8ctst_run_clock(2);       // Run two SCLK cycles between writing and reading
        mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_IN);     // Set to HiZ so Target can drive SDATA

        target_data_in = cy8ctst_receive_byte();
        mt6575_touch_info("%02x ", target_data_in);

        cy8ctst_run_clock(1);
        cy8ctst_send_vector(read_byte_v + 1, 1);     // Send the ReadByte Vector End
		
        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
        if (target_data_in != target_data_out[target_data_ptr])
        {
            mt6575_touch_info("target_data_in      = %02x\n", target_data_in);
            mt6575_touch_info("target_data_out[%d] = %02x\n", target_data_ptr, target_data_out[target_data_ptr]);
            return FAIL;
        }
        
        target_data_ptr++;
        // Increment the address by 2 to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        target_address += 2;

        

    }
    msleep(1);
    
    mt6575_touch_info("\n");
    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);	

    return PASS;
}

// ============================================================================
// Before calling, load the array, abTargetDataOUT, with the desired security
// settings using LoadArrayWithSecurityData(StartAddress,Length,SecurityType).
// The can be called multiple times with different SecurityTypes as needed for
// particular Flash Blocks. Or set them all the same using the call below:
// LoadArrayWithSecurityData(0,SECURITY_BYTES_PER_BANK, 0); 
// ============================================================================
int cy8ctst_secure_target_flash(void)
{
    unsigned char temp;

    // Transfer the temporary RAM array into the target
    target_address = 0x00;
    target_data_ptr = 0x00;
	
    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);	

    cy8ctst_send_vector(read_write_setup, num_bits_read_write_setup);
		
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);
    while(target_data_ptr < SECURITY_BYTES_PER_BANK) 
    {     
        temp = target_data_out[target_data_ptr];
        cy8ctst_send_byte(write_byte_start,4);    	
        cy8ctst_send_byte(target_address, 7);
        cy8ctst_send_byte(temp, 8);
        cy8ctst_send_byte(write_byte_end, 3);

        // SendBytes() uses MSBits, so increment the address by '2' to put
        // the 0..n address into the seven MSBit locations
        target_address += 2;				// inc by 2 in order to support a 128 byte address space
        target_data_ptr++; 
    }
	msleep(1);
	
    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);	
 
    cy8ctst_send_vector(secure, num_bits_secure);	
    if (!cy8ctst_detect_hi_lo_transition()) 
    {
        mt6575_touch_info("send secure fail\n");
        //return FAIL;	
    }
    cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return PASS;
}

// ============================================================================
// This step is optional. Verifies that the security bits have been written correctly
// ============================================================================
int cy8ctst_verify_security(void)
{
    target_address = 0x00;

    cy8ctst_send_vector(verify_security, num_bits_verify_security);

    if (!cy8ctst_detect_hi_lo_transition()) 
    {
        mt6575_touch_info("send verify_security fail\n");
        //return FAIL;	
    }

    cy8ctst_send_vector(wait_and_poll_end, num_bits_wait_and_poll_end);
    
    target_address = 0x00;
    target_data_ptr = 0x00;

    cy8ctst_send_vector(tsync_enable, num_bits_tsync_enable);	

    cy8ctst_send_vector(read_write_setup, num_bits_read_write_setup);
	
    //fReadWriteSetup();
    
    while(target_address <(SECURITY_BYTES_PER_BANK * 2)) 
    {			
        // we do SECURITY_BYTES_PER_BANK * 2 because we bTargetAddress += 2
        
        //Send Read Byte vector and then get a byte from Target
        cy8ctst_send_vector(read_byte_v, 4);									
        // Set the drive here because SendByte() does not
        mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);

        cy8ctst_send_byte(target_address,7);

        mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_IN);
        cy8ctst_run_clock(2);       // Run two SCLK cycles between writing and reading
        target_data_in = cy8ctst_receive_byte();

        cy8ctst_run_clock(1);
        cy8ctst_send_vector(read_byte_v + 1, 1);     // Send the ReadByte Vector End
        
        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
		
        if (target_data_in != target_data_out[target_data_ptr])
            return FAIL;
        
        // Increment the address by two to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        
        target_data_ptr++;
        target_address += 2;
    }
    msleep(1);

    cy8ctst_send_vector(tsync_disable, num_bits_tsync_disable);	

    return PASS;
}

static void cy8ctst_gpio_to_i2c(void) 
{
    mt_set_gpio_mode(CYPRESS_I2C_SCL, 0x01);
    mt_set_gpio_mode(CYPRESS_I2C_SDA, 0x01);
}

static void cy8ctst_i2c_to_gpio(void) 
{
    mt_set_gpio_mode(CYPRESS_I2C_RST, RST_GPIO_MODE);
    mt_set_gpio_mode(CYPRESS_I2C_SCL, SCL_GPIO_MODE);
    mt_set_gpio_mode(CYPRESS_I2C_SDA, SDA_GPIO_MODE);
    mt_set_gpio_dir(CYPRESS_I2C_RST, GPIO_DIR_OUT);
    mt_set_gpio_dir(CYPRESS_I2C_SCL, GPIO_DIR_OUT);
    mt_set_gpio_dir(CYPRESS_I2C_SDA, GPIO_DIR_OUT);

    mt_set_gpio_pull_enable(CYPRESS_I2C_RST, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(CYPRESS_I2C_RST, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(CYPRESS_I2C_SCL, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(CYPRESS_I2C_SCL, GPIO_PULL_UP);
    mt_set_gpio_pull_enable(CYPRESS_I2C_SDA, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(CYPRESS_I2C_SDA, GPIO_PULL_UP);

    mt_set_gpio_out(CYPRESS_I2C_RST, GPIO_OUT_ONE);
    mt_set_gpio_out(CYPRESS_I2C_SCL, GPIO_OUT_ZERO);
    mt_set_gpio_out(CYPRESS_I2C_SDA, GPIO_OUT_ZERO); 
}

int cy8ctst_issp_do_all(void)
{
    int i, retval;
    struct file *filp;
    unsigned char buffer[128];
    
    unsigned char bank_counter;
    unsigned int  block_counter;
    unsigned int  checksum_data;
    unsigned int  checksum_target;

    mt6575_touch_info("%s:--Liu\n", __func__);
    /* 
     * debug = 0 means turn off debug mode, debug = 1 means turn on debug mode
     * check = 0 means turn off check mode, check = 1 means turn on check mode
     */
     updating = 1;
    cy8ctst_irq_enable(0);
    switch (param)
    {
        case 0:
            debug = 0;   
            check = 0;
            break;
        case 1:
            debug = 1;
            check = 0;  
            break;
        case 2:
            debug = 0;
            check = 1;  
            break;
        case 3:
            debug = 1;  
            check = 1;
            break;
        default:
            debug = 0;
            check = 0; 
    }    

    //retval = misc_register(&cy8ctst_issp_dev);
    if(unlikely(retval)) 
    {
        mt6575_touch_info(KERN_ERR "failed to register misc device!\n");
        return retval;
    }

    cy8ctst_i2c_to_gpio();

    mt6575_touch_info("start downloading firmware.hex to the target\n\n");

    mt6575_touch_info("stage 1: [");
    if (!cy8ctst_switch_to_issp()) 
    {
        mt6575_touch_info("\n");
        mt6575_touch_info("cy8ctst initializazion failed\n\n");
        return FAIL;
    }
    mt6575_touch_info("]\n");
    mt6575_touch_info("\ncy8ctst initializazion success\n\n");

    mt6575_touch_info("stage 2: [");
    if (!cy8ctst_verify_id()) 
    { 
        mt6575_touch_info("\n");
        mt6575_touch_info("cy8ctst verify id failed\n\n");
        return FAIL;
    }
    mt6575_touch_info("]\n");
    mt6575_touch_info("\ncy8ctst verify id success\n\n");

    mt6575_touch_info("stage 3: [");
    if (!cy8ctst_erase_target()) 
    {
        mt6575_touch_info("\n");
        mt6575_touch_info("cy8ctst erase target failed\n\n");
        return FAIL;
    }
    mt6575_touch_info("]\n");
    mt6575_touch_info("\ncy8ctst erase target success\n\n");

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    filp = filp_open(HEX_FILE_PATH, O_RDONLY, 0);
    if (!filp)
    {
        mt6575_touch_info("\n");
        mt6575_touch_info("can not open file [firmware.hex]\n");
        return FAIL;
    }

    mt6575_touch_info("stage 4: [");
    checksum_data = 0;
    for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)		
    {
        for (block_counter = 0; block_counter < BLOCKS_PER_BANK; block_counter++) 
        {
            cy8ctst_load_program_data(bank_counter, (unsigned char)block_counter, filp);
            checksum_data += cy8ctst_load_target();
            																		
            if (!cy8ctst_program_target_block(bank_counter,(unsigned char)block_counter)) 
            {
                mt6575_touch_info("\n");
                mt6575_touch_info("cy8ctst_program_target_block failed\n\n");
                return FAIL;
            }

            if (!cy8ctst_read_status()) 
            {
                mt6575_touch_info("\n");
                mt6575_touch_info("cy8ctst_read_status failed\n\n");
                return FAIL;
            }
            mt6575_touch_info("#");
        }
    }
    mt6575_touch_info("]\n");
    mt6575_touch_info("\ncy8ctst load and program target success\n\n");

    filp_close(filp, NULL);
    set_fs(old_fs);
    
    if (check){
        old_fs = get_fs();
        set_fs(KERNEL_DS);
    
        filp = filp_open(HEX_FILE_PATH, O_RDONLY, 0);
        if (!filp)
        {
            mt6575_touch_info("\n");
            mt6575_touch_info("can not open file [firmware.hex]\n");
            return FAIL;
        }
        
        mt6575_touch_info("stage 5: [");
        for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)
        {
            for (block_counter = 0; block_counter < BLOCKS_PER_BANK; block_counter++) 
            {
                cy8ctst_load_program_data(bank_counter, (unsigned char)block_counter, filp);
    			
                if (!cy8ctst_verify_setup(bank_counter,(unsigned char)block_counter)) 
                {
                    mt6575_touch_info("\n");
                    mt6575_touch_info("cy8ctst_verify_setup fail\n");
                    return FAIL;
                }
    
                if (!cy8ctst_read_status()) 
                { 
                    mt6575_touch_info("\n");
                    mt6575_touch_info("cy8ctst_read_status fail\n");
                    return FAIL;
                }
    
                if (!cy8ctst_read_byte_loop()) 
                {
                    mt6575_touch_info("\n");
                    mt6575_touch_info("cy8ctst_read_byte_loop fail\n");
                    return FAIL;
                }
                mt6575_touch_info("#");
            }
        }
        mt6575_touch_info("]\n");
        mt6575_touch_info("\ncy8ctst load and verify target success\n\n");
    
        filp_close(filp, NULL);
        set_fs(old_fs);	
    }
    
    old_fs = get_fs();
    set_fs(KERNEL_DS);

    filp = filp_open(HEX_FILE_PATH, O_RDONLY, 0);
    if (!filp)
    {
        mt6575_touch_info("\n");
        mt6575_touch_info("can not open file [firmware.hex]\n");
        return FAIL;
    }

    mt6575_touch_info("stage 6: [");
    for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)
    {
        // Load one bank of security data from hex file into buffer
        if (!cy8ctst_load_security_data(bank_counter, filp)) 
        {
            mt6575_touch_info("\n");
            mt6575_touch_info("cy8ctst_load_security_data fail\n\n");
            return FAIL;
        }

        // Secure one bank of the target flash
        if (!cy8ctst_secure_target_flash()) 
        {
            mt6575_touch_info("\n");
            mt6575_touch_info("cy8ctst_secure_target_flash fail\n\n");
            return FAIL;
        }
        mt6575_touch_info("#");
    }
    mt6575_touch_info("]\n");
    mt6575_touch_info("\ncy8ctst load and secure target success\n\n");

    filp_close(filp, NULL);
    set_fs(old_fs);	
    
    if (check) {
        old_fs = get_fs();
        set_fs(KERNEL_DS);
    
        filp = filp_open(HEX_FILE_PATH, O_RDONLY, 0);
        if (!filp)
        {
            mt6575_touch_info("\n");
            mt6575_touch_info("can not open file [firmware.hex]\n");
            return FAIL;
        }
    
        mt6575_touch_info("stage 7: [");
        if (!cy8ctst_load_security_data(bank_counter, filp)) 
        {
    	    mt6575_touch_info("\n");
    	    mt6575_touch_info("cy8ctst_load_security_data fail\n\n");
            return FAIL;
        }
    
        filp_close(filp, NULL);
        set_fs(old_fs);	
    
        if (!cy8ctst_verify_security()) 
        {
            return FAIL;
        }
        mt6575_touch_info("]\n");
        mt6575_touch_info("\ncy8ctst load and verify security success\n\n");
    }
    
    mt6575_touch_info("stage 8: [");
    checksum_target = 0;
    for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)
    {
        if (!cy8ctst_target_bank_checksum(&checksum_target)) 
        {
            mt6575_touch_info("\n");
            mt6575_touch_info("cy8ctst_target_bank_checksum fail.\n\n");
            return FAIL;
        }
        mt6575_touch_info("#");
    }

    mt6575_touch_info("checksum_target = %02x, checksum_data = %02x\n", checksum_target, (checksum_data & 0xFFFF));
    if (checksum_target != (checksum_data & 0xFFFF))
    {
        mt6575_touch_info("\n");
        mt6575_touch_info("checksum_target != checksum_data\n\n");
        return FAIL;
    }  
    mt6575_touch_info("#");
    mt6575_touch_info("]\n");
    mt6575_touch_info("\ncy8ctst checksum verify success\n\n");

    cy8ctst_restart_target();
    cy8ctst_gpio_to_i2c();

    mt6575_touch_info("\n");
    mt6575_touch_info("download firmware and configuration success.\n\n");
	cy8ctst_irq_enable(1);
	updating = 0;
    return 0;
} 
EXPORT_SYMBOL(cy8ctst_issp_do_all);

static int cy8ctst_issp_open(struct inode *inode, struct file *file) 
{
	int rc = 0;
	struct cy8ctst_issp_ctxt *cy8ctst_issp = _cy8ctst_issp;
	mt6575_touch_info("%s--Liu\n", __func__);
	mutex_lock(&cy8ctst_issp->lock);
	if(cy8ctst_issp->opened == 0) {
		file->private_data = cy8ctst_issp;
		cy8ctst_issp->opened = 1;
	}
	else {
		mt6575_touch_info("cy8ctst_issp already opened\n");
		rc = -EBUSY;
	}
	mutex_unlock(&cy8ctst_issp->lock);
	return rc;
}

static int cy8ctst_issp_release(struct inode *inode, struct file *file)
{
	struct cy8ctst_issp_ctxt *cy8ctst_issp = file->private_data;
	mt6575_touch_info("%s--Liu\n", __func__);
	mutex_lock(&cy8ctst_issp->lock);
	cy8ctst_issp->opened = 0;
	mutex_unlock(&cy8ctst_issp->lock);
	return 0;
}
 int cy8ctst_FTM_Get_Fw(void) 
{
	int fw = 0, iRetry = 3;
	while (iRetry) {
		fw = cy8ctst_get_fw();
		if((fw <= 0) || (fw > 0x10)) {
			mt6575_touch_info("%s: fw=0x%02x, unknown version, iRetry=%d--Liu\n", 
				__func__, fw, iRetry);
		}
		else {
			break;
		}
		--iRetry;
		msleep(100);
	}
	if(!iRetry) {
		mt6575_touch_info("%s: too many retry!--Liu\n", __func__);
		return -1;
	}
	mt6575_touch_info("%s: s0uccess--Liu\n", __func__);
        return fw;
}
EXPORT_SYMBOL(cy8ctst_FTM_Get_Fw);
static int cy8ctst_check_fw(void) 
{
	int fw = 0, iRetry = 3;
	while (iRetry) {
		fw = cy8ctst_get_fw();
		if((fw <= 0) || (fw > 0x10)) {
			mt6575_touch_info("%s: fw=0x%02x, unknown version, iRetry=%d--Liu\n", 
				__func__, fw, iRetry);
		}
		else {
			break;
		}
		--iRetry;
		msleep(100);
	}
	if(!iRetry) {
		mt6575_touch_info("%s: too many retry!--Liu\n", __func__);
		return -1;
	}
	if(fw == LASTEST_VERSION) {
		mt6575_touch_info("%s: fw=0x%02x, LASTEST_VERSION=%d, do not need to update--Liu\n", 
			__func__, fw, LASTEST_VERSION);
		return -1;
	}
	mt6575_touch_info("%s: s0uccess--Liu\n", __func__);
        return 0;
	//return -1;
}

static int cy8ctst_issp_start(void) 
{
	updating = 1;
	cy8ctst_irq_enable(0);
	cy8ctst_i2c_to_gpio();		
	cy8ctst_switch_to_issp();
	mt6575_touch_info("%s: success--Liu\n", __func__);
	return 0;
}

static int cy8ctst_issp_write_flash(struct cy8ctst_issp_ctxt *cy8ctst_issp)
{
	int rc = 0;
	struct file *filp;
	uint8_t bank_counter;
	uint32_t block_counter;
	cy8ctst_issp->old_fs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(HEX_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(filp) || !filp) {
		mt6575_touch_info("%s: unable to open %s--Liu\n", 
			__func__, HEX_FILE_PATH);
		return PTR_ERR(filp);
	}
	cy8ctst_issp->checksum_data = 0;
	for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++) {
		for (block_counter = 0; block_counter < BLOCKS_PER_BANK; block_counter++) {
			cy8ctst_load_program_data(bank_counter, (unsigned char)block_counter, filp);
			cy8ctst_issp->checksum_data += cy8ctst_load_target();																			
			if (!cy8ctst_program_target_block(bank_counter,(unsigned char)block_counter)) {
				mt6575_touch_info("%s: program target block error--Liu\n", __func__);
				return -1;
			}
			msleep(5);
			if (!cy8ctst_read_status()) {
				mt6575_touch_info("%s: read status error--Liu\n", 
					__func__);
				return -1;
			}
			if(cy8ctst_issp->led_show)
				mt65xx_leds_brightness_set(MT65XX_LED_TYPE_RED, (block_counter % 2));
		}
	}
	mt6575_touch_info("%s: success--Liu\n", __func__);
	filp_close(filp, NULL);
	set_fs(cy8ctst_issp->old_fs);
	return rc;
}

static int cy8ctst_issp_secure_flash(struct cy8ctst_issp_ctxt *cy8ctst_issp)
{
	int rc = 0;
	struct file *filp;
	uint8_t bank_counter;
	cy8ctst_issp->old_fs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(HEX_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(filp) || !filp) {
		mt6575_touch_info("%s: unable to open %s--Liu\n", 
			__func__, HEX_FILE_PATH);
		return PTR_ERR(filp);
	}
	for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++)
	{
		// Load one bank of security data from hex file into buffer
		if (!cy8ctst_load_security_data(bank_counter, filp)) {
			mt6575_touch_info("%s: load security data error--Liu\n", 
				__func__);
			return -1;
		}

		// Secure one bank of the target flash
		if (!cy8ctst_secure_target_flash()) {
			mt6575_touch_info("%s: secure target flash error--Liu\n", 
				__func__);
			return -1;
	    }
	}
	mt6575_touch_info("%s: success--Liu\n", __func__);
	filp_close(filp, NULL);
	set_fs(cy8ctst_issp->old_fs);
	return rc;
}

static int cy8ctst_issp_checksum(struct cy8ctst_issp_ctxt *cy8ctst_issp)
{
	int rc = 0;
	uint8_t bank_counter;
	cy8ctst_issp->checksum_target = 0;
	for (bank_counter = 0; bank_counter < NUM_BANKS; bank_counter++) {
		if (!cy8ctst_target_bank_checksum(&cy8ctst_issp->checksum_target)) {
			mt6575_touch_info("%s: bank checksum error--Liu\n", 
				__func__);
			return -1;
		}
	}

	mt6575_touch_info("%s: checksum_target=0x%02x, checksum_data=0x%02x\n", 
		__func__, cy8ctst_issp->checksum_target, (cy8ctst_issp->checksum_data & 0xFFFF));
	if (cy8ctst_issp->checksum_target != (cy8ctst_issp->checksum_data & 0xFFFF)) {
		mt6575_touch_info("\n");
		mt6575_touch_info("checksum_target != checksum_data\n\n");
		return -1;
	}  
	mt6575_touch_info("%s: success--Liu\n", __func__);
	return rc;
}

static int cy8ctst_issp_last(struct cy8ctst_issp_ctxt *cy8ctst_issp)
{
	int rc = 0;
	cy8ctst_issp->checksum_data = 0;
	cy8ctst_issp->checksum_target = 0;
	cy8ctst_restart_target();
	cy8ctst_gpio_to_i2c();
	cy8ctst_irq_enable(1);
	updating = 0;
	return rc;
}

static int cy8ctst_issp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) 
{
	int rc;
	struct cy8ctst_issp_ctxt *cy8ctst_issp = filp->private_data;
	if(!cy8ctst_issp) {
		mt6575_touch_info("%s: cy8ctst_issp is NULL--Liu\n", __func__);
		return -1;
	}
	mt6575_touch_info("%s:333cmd=%x--Liu\n", __func__, cmd);
	mutex_lock(&cy8ctst_issp->lock);
	//cy8ctst_issp_do_all();
	switch(cmd) {
		case CY8CTST_ISSP_CHECK_FW:
			rc = cy8ctst_check_fw();
			break;
		case CY8CTST_ISSP_START:
			mt6575_touch_info("%s: CY8CTST_ISSP_START--Liu\n", 
				__func__);
			rc = cy8ctst_issp_start();
			break;
		case CY8CTST_ISSP_VERIFY_ID:
			mt6575_touch_info("%s: CY8CTST_ISSP_VERIFY_ID--Liu\n", 
				__func__);
			rc = cy8ctst_verify_id() ? 0 : -1;
			break;	
		case CY8CTST_ISSP_ERASE_FLASH:
			mt6575_touch_info("%s: CY8CTST_ISSP_ERASE_FLASH--Liu\n", 
				__func__);
			rc = cy8ctst_erase_target();
			break;
		case CY8CTST_ISSP_WRITE_FLASH:
			mt6575_touch_info("%s: CY8CTST_ISSP_WRITE_FLASH--Liu\n", 
				__func__);
			rc = cy8ctst_issp_write_flash(cy8ctst_issp);
			break;
		case CY8CTST_ISSP_SECURE_FLASH:
			mt6575_touch_info("%s: CY8CTST_ISSP_SECURE_FLASH--Liu\n", 
				__func__);
			rc = cy8ctst_issp_secure_flash(cy8ctst_issp);
			break;
		case CY8CTST_ISSP_CHECKSUM:
			mt6575_touch_info("%s: CY8CTST_ISSP_CHECKSUM--Liu\n", 
				__func__);
			rc = cy8ctst_issp_checksum(cy8ctst_issp);
			break;
		case CY8CTST_ISSP_LAST:
			mt6575_touch_info("%s: CY8CTST_ISSP_LAST--Liu\n", 
				__func__);
			rc = cy8ctst_issp_last(cy8ctst_issp);
			break;
		case CY8CTST_ISSP_LED_SHOW:
			rc = copy_from_user(&(cy8ctst_issp->led_show), 
				(void __user *) arg, sizeof(cy8ctst_issp->led_show));
			if (rc) {
				mt6575_touch_info("get led_show error: invalid pointer\n");
				rc = -EFAULT;
			}
			mt6575_touch_info("%s: CY8CTST_ISSP_LAST, led_show=%d--Liu\n", 
				__func__, cy8ctst_issp->led_show);
			break;
		default:
			rc = -1;
			break;
	}
	mutex_unlock(&cy8ctst_issp->lock);
	return rc;
}

static struct file_operations cy8ctst_issp_fops = {
	.owner   = THIS_MODULE,
	.unlocked_ioctl = cy8ctst_issp_ioctl,
	.open    = cy8ctst_issp_open,
	.release = cy8ctst_issp_release,
};

static struct miscdevice cy8ctst_issp_dev = {
	.minor   = MISC_DYNAMIC_MINOR,
	.name    = "cy8ctst_issp",
	.fops    = &cy8ctst_issp_fops,
};

int cy8ctst_issp_init(void)
{
	int ret = 0;
	struct cy8ctst_issp_ctxt *cy8ctst_issp;
	mt6575_touch_info("%s:--Liu\n", __func__);
	cy8ctst_issp = kzalloc(sizeof(*cy8ctst_issp), GFP_KERNEL);
	if (!cy8ctst_issp) {
		mt6575_touch_info("%s: no memery!--Liu\n", __func__);
		return -ENOMEM;
	}
	mutex_init(&cy8ctst_issp->lock);
	cy8ctst_issp->opened = 0;
	cy8ctst_issp->led_show = 0;
	_cy8ctst_issp = cy8ctst_issp;
	ret = misc_register(&cy8ctst_issp_dev);
	if(ret < 0) {
		mt6575_touch_info("%s: error, ret=%d--Liu\n", __func__, ret);
	}
	return ret;
}
EXPORT_SYMBOL(cy8ctst_issp_init);

void cy8ctst_issp_exit(void)
{
	misc_deregister(&cy8ctst_issp_dev);
}
EXPORT_SYMBOL(cy8ctst_issp_exit);
