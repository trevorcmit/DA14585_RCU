/*****************************************************************************************
 *
 * \file iqs263_i2c.c
 *
 * \brief iqs263 I2C low level interface driver.
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
 * \addtogroup IQS263_I2C_DRV
 *
 * \{
******************************************************************************************/
 
#include "iqs263_i2c.h"
#include "arch.h"

// macros
#define SEND_I2C_COMMAND(X)             SetWord16(I2C_DATA_CMD_REG, (X))
#define WAIT_WHILE_I2C_FIFO_IS_FULL()   while(!(GetWord16(I2C_STATUS_REG) & TFNF))
#define WAIT_UNTIL_I2C_FIFO_IS_EMPTY()  while(!(GetWord16(I2C_STATUS_REG) & TFE))
#define WAIT_UNTIL_NO_MASTER_ACTIVITY() while(GetWord16(I2C_STATUS_REG) & MST_ACTIVITY)
#define WAIT_FOR_RECEIVED_BYTE()        while(!GetWord16(I2C_RXFLR_REG))


/*****************************************************************************************
 * @brief Initialize I2C controller as a master for BMI055 handling.
 *
 * @param[in] dev_address   Slave device address
 * @param[in] speed         Speed
 * @param[in] address_mode  Addressing mode
******************************************************************************************/
void i2c_iqs263_init(uint16_t dev_address, uint8_t speed, uint8_t address_mode)
{
    SetBits16(CLK_PER_REG, I2C_ENABLE, 1);                                          // enable  clock for I2C 
    SetWord16(I2C_ENABLE_REG, 0x0);                                                 // Disable the I2C controller	
    SetWord16(I2C_CON_REG, I2C_MASTER_MODE | I2C_SLAVE_DISABLE |I2C_RESTART_EN);    // Slave is disabled
    SetBits16(I2C_CON_REG, I2C_SPEED, speed);                                       // Set speed
    SetBits16(I2C_CON_REG, I2C_10BITADDR_MASTER, address_mode);                     // Set addressing mode
    SetWord16(I2C_TAR_REG, dev_address & 0xFF);                                     // Set Slave device address
    SetWord16(I2C_ENABLE_REG, 0x1);                                                 // Enable the I2C controller
    while(GetWord16(I2C_STATUS_REG) & 0x20);                                        // Wait for I2C master FSM to be IDLE
}

/*****************************************************************************************
 * @brief Disable I2C controller and clock
******************************************************************************************/
void i2c_iqs263_release(void)
{	
    SetWord16(I2C_ENABLE_REG, 0x0);         // Disable the I2C controller	
    SetBits16(CLK_PER_REG, I2C_ENABLE, 0);  // Disable clock for I2C
}

/*****************************************************************************************
 * @brief Read single byte from BMI055.
 *
 * @param[in] addr  Memory address to read the byte from.
 * @param[out] buffer buffer containing data read
 *
 * @return I2C_IQS263_OPERATION_SUCCESS on success, I2C_IQS263_OPERATION_FAIL otherwise
******************************************************************************************/
i2c_iqs263_operation_result_t i2c_iqs263_read_byte(uint8_t addr, uint8_t *buffer)
{
    volatile uint32_t i2c_iqs263_read_byte_retries = 0;
//    while ((GetWord16(I2C_STATUS_REG) & TFE) == 0)
//    {
//        i2c_iqs263_read_byte_retries++;
//        if (i2c_iqs263_read_byte_retries > 1000)
//        {
//            i2c_iqs263_init(0x44, 2, 0);          
//        }
//    }
    
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();
	SEND_I2C_COMMAND(addr);                         // Write the address of the register
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();
    SEND_I2C_COMMAND(0x0100);                       // Write the address of the device
    while(!GetWord16(I2C_RXFLR_REG))                // Wait for received data
    {
        if GetBits16(I2C_RAW_INTR_STAT_REG, TX_ABRT)
        {
            if (GetBits16(I2C_TX_ABRT_SOURCE_REG, ABRT_7B_ADDR_NOACK) == 1)
            {
                return I2C_IQS263_OPERATION_FAIL;
            }
        }
    }
    *buffer = 0xFF & GetWord16(I2C_DATA_CMD_REG);    // Get received byte
    return I2C_IQS263_OPERATION_SUCCESS;

    
    

}


void iqs263_sensor_trigger_recovery(void)
{
    
}


i2c_iqs263_operation_result_t i2c_iqs263_repeat_read_data(uint32_t address1, uint32_t address2, uint8_t *rd_data_ptr, uint32_t size1, uint32_t size2)
{
    int j;
    
    GetWord16(I2C_CLR_TX_ABRT_REG); //clear the TX_ABRT interrupt (bit 6) of the IC_RAW_INTR_STAT register, and the
                                    //I2C_TX_ABRT_SOURCE register
    
	WAIT_UNTIL_I2C_FIFO_IS_EMPTY();
	GLOBAL_INT_DISABLE();
	SEND_I2C_COMMAND(address1);         // Write the address of the register
                
    for (j = 0; j < size1; j++) {    
        WAIT_WHILE_I2C_FIFO_IS_FULL();              // Wait if Tx FIFO is full
        SEND_I2C_COMMAND(0x0100);                   // Set read access for <size> times
    }
		
 	SEND_I2C_COMMAND(address2);         // Write the address of the register
                
    for (j = 0; j < size2; j++) {    
        WAIT_WHILE_I2C_FIFO_IS_FULL();              // Wait if Tx FIFO is full
        SEND_I2C_COMMAND(0x0100);                   // Set read access for <size> times
    } 

    GLOBAL_INT_RESTORE();	
    // Critical section
    //
    
    // Get the received data
    for (j = 0; j < (size1+size2); j++) {
        
        
        //#warning "REVISIT!! Must repeat reading"
        //WAIT_FOR_RECEIVED_BYTE();                   // Wait for received data
        while(!GetWord16(I2C_RXFLR_REG))
        {
            if GetBits16(I2C_RAW_INTR_STAT_REG, TX_ABRT)
            {
                if (GetBits16(I2C_TX_ABRT_SOURCE_REG, ABRT_7B_ADDR_NOACK) == 1)
                {
                    return I2C_IQS263_OPERATION_FAIL;
                }
            }
        }
                
        *rd_data_ptr =(0xFF & GetWord16(I2C_DATA_CMD_REG));  // Get the received byte
        (rd_data_ptr)++;
    }
    // End of critical section
   // GLOBAL_INT_RESTORE();
    return I2C_IQS263_OPERATION_SUCCESS;
}




i2c_iqs263_operation_result_t i2c_iqs263_read_system_flags_and_events(uint8_t *rd_data_ptr)
{
    int j;
    
    GetWord16(I2C_CLR_TX_ABRT_REG); //clear the TX_ABRT interrupt (bit 6) of the IC_RAW_INTR_STAT register, and the
                                    //I2C_TX_ABRT_SOURCE register
 

//    volatile uint32_t i2c_iqs263_read_system_flags_and_events_retries = 0;
//    while ((GetWord16(I2C_STATUS_REG) & TFE) == 0)
//    {
//        i2c_iqs263_read_system_flags_and_events_retries++;
//        if (i2c_iqs263_read_system_flags_and_events_retries > 1000)
//        {
//            i2c_iqs263_init(0x44, 2, 0);     
//        }
//    }
 
	WAIT_UNTIL_I2C_FIFO_IS_EMPTY();
    
	SEND_I2C_COMMAND(SYS_FLAGS);         // Write the address of the register
                
    for (j = 0; j < 2; j++) {    
        WAIT_WHILE_I2C_FIFO_IS_FULL();              // Wait if Tx FIFO is full
        SEND_I2C_COMMAND(0x0100);                   // Set read access for <size> times
    }
    
    // Get the received data
    for (j = 0; j < 2; j++)
    {               
        //#warning "REVISIT!! Must repeat reading"
        //WAIT_FOR_RECEIVED_BYTE();                   // Wait for received data
        while(!GetWord16(I2C_RXFLR_REG))
        {
            if GetBits16(I2C_RAW_INTR_STAT_REG, TX_ABRT)
            {
                if (GetBits16(I2C_TX_ABRT_SOURCE_REG, ABRT_7B_ADDR_NOACK) == 1)
                {
                    return I2C_IQS263_OPERATION_FAIL; 
                }
            }
        }
                
        *rd_data_ptr =(0xFF & GetWord16(I2C_DATA_CMD_REG));  // Get the received byte
        (rd_data_ptr)++;
    }
    return I2C_IQS263_OPERATION_SUCCESS; 
}


i2c_iqs263_operation_result_t i2c_iqs263_write_bytes(uint32_t address, uint8_t *wr_data, uint8_t size)
{
    uint8_t current_size;
    uint8_t *current_wr_data;

    current_size = size;
    current_wr_data = wr_data;
    // Read this register to clear the TX_ABRT interrupt (bit 6) of the
    // IC_RAW_INTR_STAT register, and the
    // I2C_TX_ABRT_SOURCE register. This also releases the TX
    // FIFO from the flushed/reset state, allowing more writes to the
    // TX FIFO. Refer to Bit 9 of the I2C_TX_ABRT_SOURCE register
    // for an exception to clearing IC_TX_ABRT_SOURCE.
    GetWord16(I2C_CLR_TX_ABRT_REG);
    
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();
    GLOBAL_INT_DISABLE();
    
    SEND_I2C_COMMAND(address);          // Write the address of the register
        
    while(current_size--)
    {
        SEND_I2C_COMMAND(*(current_wr_data++) & 0xFF);   // Write the data of the register
    }
    GLOBAL_INT_RESTORE();
    
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();     // Wait if I2C Tx FIFO is full

    WAIT_UNTIL_NO_MASTER_ACTIVITY();    // Wait until no master activity 
    
    if (GetBits16(I2C_RAW_INTR_STAT_REG, TX_ABRT) == 0)
    {
        return I2C_IQS263_OPERATION_SUCCESS;
    }
    return I2C_IQS263_OPERATION_FAIL;
}

/**
 * \}
 * \}
 * \}
 */
