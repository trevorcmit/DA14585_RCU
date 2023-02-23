/**
 ****************************************************************************************
 *
 * @file i2c_core.h
 *
 * @brief i2c interface header file.
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */

#ifndef _I2C_CORE_H_
#define _I2C_CORE_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>

/*
 * DEFINES
 ****************************************************************************************
 */

enum I2C_SPEED_MODES{
  I2C_STANDARD = 1,
  I2C_FAST,
};

enum I2C_ADDRESS_MODES{
  I2C_7BIT_ADDR,
  I2C_10BIT_ADDR,
};

enum I2C_ADRESS_BYTES_COUNT{
  I2C_1BYTE_ADDR,
  I2C_2BYTES_ADDR,
  I2C_3BYTES_ADDR,
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize I2C controller as a master for device handling.
 * @param[in] dev_address  Slave device address
 * @param[in] speed        Speed
 * @param[in] address_mode Addressing mode
 * @retun void
 ****************************************************************************************
 */
void i2c_init(uint16_t dev_address, uint8_t speed, uint8_t address_mode);

/**
 ****************************************************************************************
 * @brief Disable I2C controller and clock.
 * @return void
 ****************************************************************************************
 */
void i2c_release(void);

/**
 ****************************************************************************************
 * @brief Reads data from I2C to memory position of given pointer.
 * @param[in] rd_data_ptr     Read data pointer
 * @param[in] address         Starting memory address
 * @param[in] size            Size of the data to be read
 * @return Bytes that were actually read (due to memory size limitation)
 ****************************************************************************************
 */
uint32_t i2c_read_data(uint8_t *rd_data_ptr, uint32_t address, uint32_t size);

/**
 ****************************************************************************************
 * @brief Write single byte to I2C.
 * @param[in] address     Memory position to write the byte to
 * @param[in] wr_data     Byte to be written
 * @return void
 ****************************************************************************************
 */
void i2c_write_byte(uint32_t address, uint8_t wr_data);

/**
 ****************************************************************************************
 * @brief Writes data to I2C.
 * @param[in] wr_data_ptr     Pointer to the first of bytes to be written
 * @param[in] address         Starting address of the write process
 * @param[in] size            Size of the data to be written
 * @return Bytes that were actually written (due to memory size limitation)
 ****************************************************************************************
 */
uint16_t i2c_write_data(uint8_t *wr_data_ptr, uint32_t address, uint16_t size);

#endif // _I2C_CORE_H_
