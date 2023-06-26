/*****************************************************************************************
 *
 * \file i2c_azotouch.h
 *
 * \brief Azoteq I2C communication driver provides simple functions for initializing
 * the MCU's I2C peripheral for communicating with the IQS modules as well as
 * functions used for register/memory reading/writing.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

#ifndef _I2C_AZOTOUCH_H_
#define _I2C_AZOTOUCH_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IQS572_I2C_DRV
 *
 * \brief IQ572 over i2c driver
 *
 * \{
******************************************************************************************/
 
/*
 * INCLUDE FILES
 *
 */

#include <stdint.h>



enum I2C_AZOTOUCH_SPEED_MODES
{
    I2C_AZOTOUCH_STANDARD = 1,
    I2C_AZOTOUCH_FAST,
};

enum I2C_AZOTOUCH_ADDRESS_MODES
{
    I2C_AZOTOUCH_7BIT_ADDR,
    I2C_AZOTOUCH_10BIT_ADDR,
};


/*
 * FUNCTION DECLARATIONS
 *
 */

/*****************************************************************************************
 * \brief Initialize and Enable I2C controller as a master for I2C Touch Controller handling.
 * \param[in] dev_address  Slave device address
 * \param[in] speed        Speed
 * \param[in] address_mode Addressing mode
 * \param[in] address_size The internal address size of the touch controller
******************************************************************************************/
void i2c_azotouch_init(uint16_t dev_address, uint8_t speed, uint8_t address_mode, uint8_t address_size);


/*****************************************************************************************
 * \brief Disable I2C Controller and I2C Peripheral Clock.
 * \return void
******************************************************************************************/
void i2c_azotouch_release(void);


/*****************************************************************************************
 * \brief Read data from the I2C Azoteq Touch Controller
 * \param[in] mmap_reg_addr  Memory Mapped Register address to read the data from
 * \param[out] data          Pointer to where the received data will be placed
 * \param[in] len            The size of the data we want to read
 * \return uint8_t           1-Success, 0-Failure
******************************************************************************************/
uint8_t i2c_azotouch_random_read(uint16_t mmap_reg_addr, uint8_t *data, uint16_t len);


/*****************************************************************************************
 * \brief Writes a Register of the I2C Azoteq Touch Controller
 * \param[in] mmap_reg_addr  Starting address of the registers we want to write to
 * \param[in] data           Pointer to the first of the bytes to be written
 * \param[in] size           Size of the data to be written
 * \return uint8_t           1-Success, 0-Failure
******************************************************************************************/
uint8_t i2c_azotouch_data_write(uint16_t mmap_reg_addr, uint8_t *data, uint16_t size);


/*****************************************************************************************
 * \brief Forces start of I2C communication with the Azoteq Touch Controller (IQS5xx)
 * (used when trackpad is not in active mode)
 * \return bool true: Forcing start of communication was successful, false: Failed
******************************************************************************************/
bool i2c_azotouch_force_communication(void);


/*****************************************************************************************
 * \brief Reads 2 registers sequentially - Used for IQS devices that do not support
 * incremental reading (IQS263)
 * \param[in] reg1       Memory Mapped Register 1 address to read the data from
 * \param[in] reg2       Memory Mapped Register 2 address to read the data from
 * \param[out] data   	 Pointer to where the received data will be placed
 * \param[in] len1     	 The size of the data we want to read from reg1
 * \param[in] len2     	 The size of the data we want to read from reg2
 * \return bool          true if successful
******************************************************************************************/
bool i2c_azotouch_repeat_read_data(uint8_t reg1, uint8_t reg2, uint8_t *data, uint8_t len1, uint8_t len2);

/**
 * \}
 * \}
 * \}
 */

#endif // _I2C_AZOTOUCH_H_
