/*****************************************************************************************
 *
 * \file iqs263_i2c.h
 *
 * \brief IQS263 driver over i2c interface header file.
 * 
******************************************************************************************/

#ifndef _I2C_IQS263_H_
#define _I2C_IQS263_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IQS263_I2C_DRV
 *
 * \brief IQS263 over i2c driver
 *
 * \{
******************************************************************************************/
 
/* Set Active Channels (Address 0x0D) */
#define ACTIVE_CHS							0x0F //CH0,CH1,CH2,CH3

/* Change the Timing settings (Address 0x0B) */
#define LOW_POWER							0x10 //16*16ms=256ms
#define ATI_TARGET_TOUCH					0x40 //64*8=512 Target
#define ATI_TARGET_PROX						0x80 //128*8=1024 Target

/* Change the Thresholds for each channel (Address 0x0A) */
#define PROX_THRESHOLD						0x04 //Prox Threshold
//#define TOUCH_THRESHOLD_CH1					0x0C //Touch Threshold CH1
//#define TOUCH_THRESHOLD_CH2					0x0C //Touch Threshold CH2
//#define TOUCH_THRESHOLD_CH3					0x0C //Touch Threshold CH3
//Mark 23 Sep - FPC thresholds
#define TOUCH_THRESHOLD_CH1					0x08 //Touch Threshold CH1
#define TOUCH_THRESHOLD_CH2					0x08 //Touch Threshold CH2
#define TOUCH_THRESHOLD_CH3					0x08 //Touch Threshold CH3
#define MOVEMENT_THRESHOLD					0x03 //Movement Threshold (not used)
#define RESEED_BLOCK						0x00 //Halt timeout Reseed Block (not used)
#define HALT_TIME							0xFF //Always halt
#define I2C_TIMEOUT							0x08 //8*1.28ms=10.24ms

/* Change the Prox Settings or setup of the IQS263 (Address 0x09) */
#define PROXSETTINGS0_VAL					0x10 //Redo ATI
#define PROXSETTINGS1_VAL					0x58 //Event Mode, Wheel
#define PROXSETTINGS2_VAL					0x20 //Wake from LP for counts in both direction
#define PROXSETTINGS3_VAL					0x00
#define EVENT_MASK_VAL						0x0F //ATI, Slide, Touch, Prox

/* System Flags (Address 0x01 Byte1) */
#define SYSTEM_FLAGS_VAL					0x00 //Write Show Reset to 

#define IQS263_PROD_NUM (0x3C)

#define DEVICE_INFO             0x00
#define SYS_FLAGS               0x01
#define COORDINATES             0x02
#define TOUCH_BYTES             0x03
#define COUNTS                  0x04
#define LTA                     0x05
#define DELTAS                  0x06
#define MULTIPLIERS             0x07
#define COMPENSATION            0x08
#define PROX_SETTINGS           0x09
#define THRESHOLDS              0x0A
#define TIMINGS_AND_TARGETS     0x0B
#define GESTURE_TIMERS          0x0C
#define ACTIVE_CHANNELS         0x0D


typedef enum i2c_iqs263_operation_result
{
    I2C_IQS263_OPERATION_SUCCESS = 0,
    I2C_IQS263_OPERATION_FAIL = 1,
}
i2c_iqs263_operation_result_t;


/*****************************************************************************************
 * @brief Initialize I2C controller as a master for BMI055 handling.
 *
 * @param[in] dev_address   Slave device address
 * @param[in] speed         Speed
 * @param[in] address_mode  Addressing mode
******************************************************************************************/
void i2c_iqs263_init (uint16_t dev_address, uint8_t speed, uint8_t address_mode);

i2c_iqs263_operation_result_t i2c_iqs263_repeat_read_data(uint32_t address1, uint32_t address2, uint8_t *rd_data_ptr, uint32_t size1, uint32_t size2);

uint32_t i2c_iqs263_repeat_read_tri_data(uint32_t address1, uint32_t address2,uint32_t address3 ,  uint8_t *rd_data_ptr, uint32_t size1, uint32_t size2, uint32_t size3);


/*****************************************************************************************
 * @brief Disable I2C controller and clock
******************************************************************************************/
void i2c_iqs263_release(void);


 /****************************************************************************************
 * @brief Read single byte from I2C BMI055.
 *
 * @param[in] addr  Memory address to read the byte from.
 * @param[in] buffer Buffer
 *
 * @return
******************************************************************************************/
i2c_iqs263_operation_result_t i2c_iqs263_read_byte(uint8_t addr, uint8_t *buffer);

/*****************************************************************************************
 * @brief Reads data from I2C BMI055 to memory position of given pointer.
 *
 * @param[in] rd_data_ptr   Read data pointer.
 * @param[in] address       Starting memory address.
 * @param[in] size          Size of the data to be read.
 *
 * @return Bytes that were actually read (due to memory size limitation).
******************************************************************************************/
uint32_t i2c_iqs263_read_data(uint32_t address, uint8_t *rd_data_ptr, uint32_t size);

/*****************************************************************************************
 * @brief Write single byte to I2C BMI055.
 *
 * @param[in] address   Memory position to write the byte to.
 * @param[in] wr_data   Byte to be written.
******************************************************************************************/
void i2c_iqs263_write_byte(uint32_t address, uint8_t wr_data);

i2c_iqs263_operation_result_t i2c_iqs263_write_bytes(uint32_t address, uint8_t *wr_data, uint8_t size);

i2c_iqs263_operation_result_t i2c_iqs263_read_system_flags_and_events(uint8_t *rd_data_ptr);

void iqs263_sensor_trigger_recovery(void);

/**
 * \}
 * \}
 * \}
 */

#endif // _I2C_IQS263_H_
