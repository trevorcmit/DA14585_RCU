/*****************************************************************************************
 *
 * \file i2c_azotouch.c
 *
 * \brief Azoteq IQS572 Trackpad driver over i2c interface.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup AZOTEQ_I2C_DRV
 *
 * \{
******************************************************************************************/
 

/*
 * INCLUDE FILES
 *
 */

#include "gpio.h"
#include <user_periph_setup.h>
#include "i2c_azotouch.h"
#include "port_platform.h"


/*
 * DEFINES
 *
 */

// I2C Peripheral macros
#define I2C_CLK_ENABLE()                SetBits16(CLK_PER_REG, I2C_ENABLE, 1)   // Enable the I2C peripheral clock
#define I2C_CLK_DISABLE()	      	SetBits16(CLK_PER_REG, I2C_ENABLE, 0)   // Disable the I2C peripheral clock
#define I2C_ENABLE_CTRL()               SetBits16(I2C_ENABLE_REG, CTRL_ENABLE, 1)       // Enable the I2C controller
#define I2C_DISABLE_CTRL()	      	SetBits16(I2C_ENABLE_REG, CTRL_ENABLE, 0)       // Disable the I2C controller and flush the FIFOs

// I2C Command Macros   
#define I2C_WRITE_ACCESS_CMD(x)		SetWord16(I2C_DATA_CMD_REG, (x))        // Issues an I2C Write Access Cmd. x is the byte to be written
#define I2C_READ_ACCESS_CMD()		SetWord16(I2C_DATA_CMD_REG, 0x100)      // Issues an I2C Read Access Cmd
#define I2C_READ_FIFO_BYTE()		GetWord16(I2C_DATA_CMD_REG)     // Get received byte from I2C Rx FIFO (if any)

// I2C Status Macros    
#define I2C_RX_FIFO_IS_EMPTY()		((GetWord16(I2C_STATUS_REG) & RFNE) != 0)
#define I2C_RX_FIFO_IS_FULL()		((GetWord16(I2C_STATUS_REG) & RFF) != 0)
#define I2C_TX_FIFO_IS_FULL()      	((GetWord16(I2C_STATUS_REG) & TFNF) == 0)
#define I2C_TX_FIFO_IS_EMPTY()		((GetWord16(I2C_STATUS_REG) & TFE) != 0)
#define I2C_IS_MASTER_ACTIVE()    	((GetWord16(I2C_STATUS_REG) & MST_ACTIVITY) !=0)
#define I2C_RX_BYTE_AVAILABLE()         (GetWord16(I2C_RXFLR_REG) != 0)


#define I2C_RXTX_FIFO_MAX_SIZE				32
#define I2C_RX_TIMEOUT_RETRIES				(10000)

/*
 * LOCAL VARIABLE DEFINITIONS
 *
 */

static uint8_t _i2c_azotouch_dev_address;        // Device address
static uint8_t _i2c_addr_size	__PORT_RETAINED; // The address size

/*
 * LOCAL FUNCTION DEFINITIONS
 *
 */

static uint8_t i2c_azotouch_read_and_get_bytes(uint8_t ** data, uint16_t size);


/*
 * FUNCTION DEFINITIONS
 *
 */

void i2c_azotouch_init(uint16_t dev_address, uint8_t speed, uint8_t address_mode, uint8_t address_size)
{
        _i2c_azotouch_dev_address = dev_address;
        _i2c_addr_size = address_size;

        I2C_CLK_ENABLE();       // Enable  clock for I2C
        I2C_DISABLE_CTRL();     // Disable the I2C controller

        // Setup the MCU's I2C peripheral
        SetWord16(I2C_CON_REG, I2C_MASTER_MODE | I2C_SLAVE_DISABLE | I2C_RESTART_EN);   // Enable the I2C Master functionality
        SetBits16(I2C_CON_REG, I2C_SPEED, speed);       // Set I2C speed
        SetBits16(I2C_CON_REG, I2C_10BITADDR_MASTER, address_mode);     // Set addressing mode
        SetWord16(I2C_TAR_REG, _i2c_azotouch_dev_address & 0x3FF);      // Set Slave device address, i.e the I2C address provided by the manufacturer
        I2C_ENABLE_CTRL();      // Enable the I2C controller
        while (I2C_IS_MASTER_ACTIVE()); // Wait until the I2C Master is NOT active
}


void i2c_azotouch_release(void)
{
        I2C_DISABLE_CTRL();     // Disable the I2C controller
        I2C_CLK_DISABLE();      // Disable clock for I2C
}


/*****************************************************************************************
 * \brief Send I2C Azoteq Memory Mapped Register Address
 *
 * \param[in] mmap_reg_addr The I2C Azoteq Memory Mapped Register Address
******************************************************************************************/
static void i2c_azoteq_send_mem_reg_addr(uint16_t mmap_reg_addr)
{
        if(_i2c_addr_size == 2) {
        while (I2C_TX_FIFO_IS_FULL());  // Wait if I2C Tx FIFO is full
        I2C_WRITE_ACCESS_CMD(mmap_reg_addr >> 8 & 0xFF);        // Write the High-Order byte of the Memory Map Register Address (ADR-HIGH)
        }
        while (I2C_TX_FIFO_IS_FULL());  // Wait if I2C Tx FIFO is full
        I2C_WRITE_ACCESS_CMD(mmap_reg_addr & 0xFF);     // Write the Low-Order byte of the Memory Map Register Address (ADR-LOW)      
}



/*****************************************************************************************
 * \brief Issue N I2C read accesses(N=number of bytes to be read) and pop the received 
 *        bytes from the Rx FIFO
 *
 * \param[in] data A double pointer to the data buffer that the received data is going to be placed
 * \param[in] size The number of bytes to be read
 * \return uint8_t 1 if read was successful, 0 otherwise
******************************************************************************************/
static uint8_t i2c_azotouch_read_and_get_bytes(uint8_t ** data, uint16_t size)
{
        uint16_t tlen = size;
        uint16_t iterSize;
        uint8_t rxIterations = 1;
        uint16_t retries = I2C_RX_TIMEOUT_RETRIES;

        // Split the main read into smaller reads
        if (size >= (I2C_RXTX_FIFO_MAX_SIZE / 2)) {
                rxIterations =
                        size / (I2C_RXTX_FIFO_MAX_SIZE / 2) + size % (I2C_RXTX_FIFO_MAX_SIZE / 2);
        }

        while (rxIterations) {

                // Calculate the size of each iteration( always <=I2C_RXTX_FIFO_MAX_SIZE )
                if (size > (I2C_RXTX_FIFO_MAX_SIZE / 2)) {
                        iterSize = (I2C_RXTX_FIFO_MAX_SIZE / 2);
                }
                else {
                        iterSize = size;
                }

                tlen = iterSize;

                // Make Read accesses
                while (tlen) {
                        while (I2C_TX_FIFO_IS_FULL());  // Wait until the FIFO is not full so that we can send a Read Command
                        I2C_READ_ACCESS_CMD();  // Issue N I2C read accesses, where N is the number of bytes to be read 
                        --tlen; // -1 bytes remaining
                }

                // Pop the received data out of the I2C Rx FIFO
                while (iterSize) {

                        while (!I2C_RX_BYTE_AVAILABLE()) {
                                if (retries-- == 0)
                                        return 0;
                        }       // Wait until there are bytes available in the RX fifo
                        **data = I2C_READ_FIFO_BYTE();  // Get and store the bytes from the FIFO to our buffer
                        (*data)++;      // Increment the buffer pointer
                        --size;
                        --iterSize;     // -1 bytes remaining
                }

                --rxIterations;
        }
        return 1;
}


uint8_t i2c_azotouch_random_read(uint16_t mmap_reg_addr, uint8_t * data, uint16_t len)
{
        uint8_t stat;

        // NOTE: RDY pin MUST be high in order to READ successfully

        GLOBAL_INT_DISABLE();   // Entering Critical Section

        i2c_azoteq_send_mem_reg_addr(mmap_reg_addr);    // Send the Memory Map Register Address we want to access

        stat = i2c_azotouch_read_and_get_bytes(&data, len);

        GLOBAL_INT_RESTORE();   // End of critical section

        while (!I2C_TX_FIFO_IS_EMPTY());        // Wait until Tx FIFO is empty
        while (I2C_IS_MASTER_ACTIVE()); // Wait until master is NOT active

        return stat;
}


uint8_t i2c_azotouch_data_write(uint16_t mmap_reg_addr, uint8_t * bytes, uint16_t bytenum)
{

        GLOBAL_INT_DISABLE();   // Entering Critical Section

        // NOTE: RDY pin MUST be high in order to WRITE successfully                                                     

        i2c_azoteq_send_mem_reg_addr(mmap_reg_addr);    // Send the Memory Map Register Address to which we want to write to

        while (bytenum) {
                while (I2C_TX_FIFO_IS_FULL());  // Wait if I2C Tx FIFO is full
                I2C_WRITE_ACCESS_CMD(*bytes & 0xFF);    // Send the data we want to write
                ++bytes;        // Increase the data pointer
                --bytenum;      // Decrease the remaining bytes number
        }


        GLOBAL_INT_RESTORE();   // End of critical section

        while (!I2C_TX_FIFO_IS_EMPTY());        // Wait until Tx FIFO is empty
        while (I2C_IS_MASTER_ACTIVE()); // Wait until no master activity (NACK)

        return 1;
}


bool i2c_azotouch_force_communication()
{
        uint16_t i, txstat;
        for (i = 0; i < 30000; i++) {
                I2C_WRITE_ACCESS_CMD(0x00);     // Make a dummy access
                while (!I2C_TX_FIFO_IS_EMPTY());        // Wait until Tx FIFO is empty
                while (I2C_IS_MASTER_ACTIVE()); // Wait until no master activity (NACK)
                txstat = GetWord16(I2C_TX_ABRT_SOURCE_REG);     // Read the I2C_TX_ABRT_SOURCE_REG register
                GetWord16(I2C_CLR_TX_ABRT_REG); // Clear I2C_TX_ABRT_SOURCE register
                if ((txstat & ABRT_7B_ADDR_NOACK) == 0) {
                        return true;
                }
        }

        return false;
}


bool i2c_azotouch_repeat_read_data(uint8_t reg1, uint8_t reg2, uint8_t *data, uint8_t len1, uint8_t len2)
{
    uint8_t j;
    uint16_t retries = I2C_RX_TIMEOUT_RETRIES;
    
    // NOTE: RDY pin MUST be low in order to READ successfully

    GLOBAL_INT_DISABLE();   // Entering Critical Section

    i2c_azoteq_send_mem_reg_addr(reg1);    // Send the Memory Map Register Address we want to access

    for (j = 0; j < len1; j++) {    
        while (I2C_TX_FIFO_IS_FULL());              // Wait if Tx FIFO is full
        I2C_READ_ACCESS_CMD();                   // Set read access for <size> times
    }
    
    i2c_azoteq_send_mem_reg_addr(reg2);    // Send the Memory Map Register Address we want to access
    
    for (j = 0; j < len2; j++) {    
        while (I2C_TX_FIFO_IS_FULL());             // Wait if Tx FIFO is full
        I2C_READ_ACCESS_CMD();                   // Set read access for <size> times
    }
    
    GLOBAL_INT_RESTORE();   // End of critical section
    
    for (j = 0; j < len2 + len1; j++) { 
        while (!I2C_RX_BYTE_AVAILABLE()) {
                if (retries-- == 0) {
                        return false;
                }
        }
        
        *data = I2C_READ_FIFO_BYTE();
        ++data;
    }



    while (!I2C_TX_FIFO_IS_EMPTY()) // Wait until Tx FIFO is empty
    while (I2C_IS_MASTER_ACTIVE()); // Wait until master is NOT active
    return true;   
}

/**
 * \}
 * \}
 * \}
 */
