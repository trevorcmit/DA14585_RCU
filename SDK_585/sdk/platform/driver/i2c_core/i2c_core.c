/*****************************************************************************************
 *
 * @file i2c_core.c
 *
 * @brief device driver over i2c interface.
 *
 * Copyright (C) 2012. Dialog Semiconductor Ltd, unpublished work. This computer 
 * program includes Confidential, Proprietary Information and is a Trade Secret of 
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited 
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
******************************************************************************************/

#include "i2c_core.h"
#include <user_periph_setup.h>

#define I2C_PAGE (32)

// macros
#define RECEIVE_I2C_DATA()      (uint8_t)(GetWord16(I2C_DATA_CMD_REG) & I2C_DAT)
#define SEND_I2C_DATA(X)        SetWord16(I2C_DATA_CMD_REG, (X) & I2C_DAT)
#define SEND_I2C_CMD(X)         SetBits16(I2C_DATA_CMD_REG, I2C_CMD, (X))
#define I2C_FIFO_IS_FULL()      (!(GetWord16(I2C_STATUS_REG) & TFNF))
#define I2C_FIFO_IS_EMPTY()     (GetWord16(I2C_STATUS_REG) & TFE)
#define I2C_MASTER_IS_ACTIVE()  (GetWord16(I2C_STATUS_REG) & MST_ACTIVITY)
#define I2C_DATA_AVAILABLE()    (uint8_t)(GetWord16(I2C_RXFLR_REG) & RXFLR)

enum I2C_COMMAND {
    I2C_WRITE,
    I2C_READ
};

void i2c_init(uint16_t dev_address, enum I2C_SPEED_MODES speed, enum I2C_ADDRESS_MODES address_mode)
{
    SetBits16(CLK_PER_REG, I2C_ENABLE, 1);                                          // enable  clock for I2C 
    SetBits16(I2C_ENABLE_REG, CTRL_ENABLE, 0);                                      // Disable the I2C controller	

    SetWord16(I2C_CON_REG, I2C_MASTER_MODE | I2C_SLAVE_DISABLE | I2C_RESTART_EN);   // Slave is disabled
    SetBits16(I2C_CON_REG, I2C_SPEED, speed);                                       // Set speed
    SetBits16(I2C_CON_REG, I2C_10BITADDR_MASTER, address_mode);                     // Set addressing mode

    SetWord16(I2C_TAR_REG, dev_address & IC_TAR);                                   // Set Slave device address
    SetBits16(I2C_ENABLE_REG, CTRL_ENABLE, 1);                                      // Enable the I2C controller
    while (I2C_MASTER_IS_ACTIVE());                                                 // Wait for I2C master FSM to be IDLE
}

void i2c_release(void)
{	
    SetBits16(I2C_ENABLE_REG, CTRL_ENABLE, 0);  // Disable the I2C controller	
    SetBits16(CLK_PER_REG, I2C_ENABLE, 0);      // Disable clock for I2C
}

/*****************************************************************************************
 * @brief Write a byte to the I2C slave
 *
 * @param[in] data The byte to write
******************************************************************************************/
static inline void send_data(uint8_t data)
{
    while (I2C_FIFO_IS_FULL());
    SEND_I2C_DATA(data);
}

/*****************************************************************************************
 * @brief Request a byte from the I2C slave
******************************************************************************************/
static inline void send_read_cmd(void)
{
    while (I2C_FIFO_IS_FULL());
    SEND_I2C_CMD(I2C_READ);
}

/*****************************************************************************************
 * @brief Read single series of bytes from I2C (for driver's internal use)
 *
 * @param[out] p        Memory address to read the series of bytes to (all in the same page)
 * @param[in] address   Starting I2C register
 * @param[in] size      count of bytes to read (must not cross page)
******************************************************************************************/
static void read_data_single(uint8_t **p, uint32_t address, uint32_t size)
{
    int j;

    SEND_I2C_DATA(address);             // Set address, write access
    
    for (j = 0; j < size; j++)
    {    
        send_read_cmd();
    }

    while (I2C_DATA_AVAILABLE() < size);
    
    // Get the received data
    for (j = 0; j < size; j++)                                         
    {
        **p = RECEIVE_I2C_DATA();       // Get the received byte
        (*p)++;
    }
}

uint32_t i2c_read_data(uint8_t *rd_data_ptr, uint32_t address, uint32_t size)
{
    if (size == 0)
    {
        return 0;
    }
    
    uint32_t    tmp_size = size,
                bytes_read = size;
    
    // Read 32 bytes at a time
    while (tmp_size >= I2C_PAGE)
    {
        read_data_single(&rd_data_ptr, address, I2C_PAGE);
            
        address += I2C_PAGE;  // Update base address for read
        tmp_size -= I2C_PAGE; // Update tmp_size for bytes remaining to be read
    }

    if (tmp_size)
    {
        read_data_single(&rd_data_ptr, address, tmp_size);
    }
    
    return bytes_read;
}

void i2c_write_byte(uint32_t address, uint8_t wr_data)
{
    send_data(address);
    send_data(wr_data);
    
    while (!I2C_FIFO_IS_EMPTY());
    while (I2C_MASTER_IS_ACTIVE());
}

uint16_t i2c_write_data(uint8_t *wr_data_ptr, uint32_t address, uint16_t size)
{
    uint16_t    bytes_written = 0,
                feasible_size = size;   // adjust limit accordingly
    
    // Critical section
    GLOBAL_INT_DISABLE();
    
    send_data(address);
    
    do
    {
        send_data(*wr_data_ptr);
        wr_data_ptr++;
        feasible_size--;
        bytes_written++;
    }
    while (feasible_size);

    // End of critical section
    GLOBAL_INT_RESTORE();
    
    while (!I2C_FIFO_IS_EMPTY());
    while (I2C_MASTER_IS_ACTIVE());
    
    return bytes_written;
}
