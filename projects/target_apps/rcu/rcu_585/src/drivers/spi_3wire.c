/**
****************************************************************************************
* \file spi_3wire.c
* \brief custom app 3-wire SPI low level driver.
****************************************************************************************
*/

/**
****************************************************************************************
* \addtogroup USER
* \{
* \addtogroup USER_DRIVERS
* \{
* \addtogroup SPI_3WIRE_DRV
* \{
****************************************************************************************
*/
 
#include <stdint.h>
#include "port_platform.h"
#include "gpio.h"
#include "spi_3wire.h"

uint16_t TsradCounter;
SPI_Pin sdio;
SPI_Pin cs;

void deactivate_3wire_spi(void)
{
    GPIO_SetActive(cs.port, cs.pin);                                            // leave CS high        
    GPIO_SetPinFunction( sdio.port, sdio.pin, INPUT_PULLUP, PID_SPI_DI);
	NVIC_DisableIRQ(SPI_IRQn);                                                  // disable SPI interrupt
	SetBits16(SPI_CTRL_REG, SPI_ON, 0);                                         // close SPI block, if opened
	SetBits16(CLK_PER_REG, SPI_ENABLE, 0);                                      // disable clock for SPI
}

uint16_t do_transaction(uint16_t data)
{
    uint8_t data_read;
    
    ASSERT_ERROR(GetBits16(SPI_CTRL_REG, SPI_INT_BIT)==0)   
    ASSERT_ERROR(GetBits16(SPI_CTRL_REG, SPI_BUSY)==0)   
    ASSERT_ERROR(GetBits16(SPI_CTRL_REG, SPI_TXH)==0)      
    port_delay_usec(10);    
    SetWord16(SPI_RX_TX_REG0, data);          // MSB set to HIGH - A6..A0 Address of register to write to

    while (GetBits16(SPI_CTRL_REG, SPI_INT_BIT)==0);
    SetBits16(SPI_CLEAR_INT_REG, SPI_CLEAR_INT, 1);
    data_read = GetWord16(SPI_RX_TX_REG0) & 0xFF;    
    SetWord16(SPI_CLEAR_INT_REG, 0x0001);                               //Clear SPI_CTRL_REG[SPI_INT_BIT] interrupt
    
	ASSERT_ERROR(GetBits16(SPI_CTRL_REG, SPI_INT_BIT)==0)
 
    return data_read;
}

void write_to_3wire_SPI_register(uint8_t registerIndex, uint8_t valueToWrite)
{
    GPIO_SetInactive(cs.port, cs.pin);                              // pull CS low   
    GPIO_SetPinFunction( sdio.port, sdio.pin, OUTPUT, PID_SPI_DO);  // configure SDIO as output       
    do_transaction((uint16_t)(registerIndex | 0x80));// MSB set to HIGH - A6..A0 Address of register to write to    
    do_transaction((uint16_t)valueToWrite); //write data
     
    GPIO_SetActive(cs.port, cs.pin);                                // set CS high
    GPIO_SetPinFunction( sdio.port, sdio.pin, INPUT_PULLUP, PID_SPI_DI);  // configure SDIO as input        
}    

uint8_t read_from_3wire_SPI_register(uint8_t registerIndex, bool is_last_transaction)
{
    static uint8_t i, dataRead;

    GPIO_SetInactive(cs.port, cs.pin);                              // pull CS low        
    GPIO_SetPinFunction( sdio.port, sdio.pin, OUTPUT, PID_SPI_DO);  // configure SDIO as output 
    
    do_transaction((uint16_t)(registerIndex));// MSB set to HIGH - A6..A0 Address of register to write to
    
    GPIO_SetPinFunction( sdio.port, sdio.pin, INPUT_PULLUP, PID_SPI_DI);   // configure SDIO as input  
    
    for (i=0; i<TsradCounter; i++);                                 // {DEV.NOTE#: For Mouse sensor Delay > Tsrad = 4us <-- suitable counter is 6}   
    dataRead = do_transaction(0x0000);                              // read received byte
	
    if(is_last_transaction)
        GPIO_SetActive(cs.port, cs.pin);                              // set CS high 
    
    return dataRead;
}   

void burst_write_to_3wire_SPI_register(uint8_t registerIndex)
{ 
   GPIO_SetPinFunction( sdio.port, sdio.pin, OUTPUT, PID_SPI_DO);   // configure SDIO as output 

   do_transaction((uint16_t)(registerIndex));// MSB set to HIGH - A6..A0 Address of register to write to

   GPIO_SetPinFunction( sdio.port, sdio.pin, INPUT_PULLUP, PID_SPI_DI);    // configure SDIO as input  
}   

uint8_t burst_read_from_3wire_SPI_register(void)
{
    return do_transaction(0x0000);                             // read received byte
}   

void spi3wire_cs_high(void)
{
    GPIO_SetActive(cs.port, cs.pin);                              // set CS high 
}

void spi3wire_cs_low(void)
{
    GPIO_SetInactive(cs.port, cs.pin);                            // set CS low 
}

/**
* \}
* \}
* \}
*/
