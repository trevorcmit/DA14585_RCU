/**
****************************************************************************************
* \file spi_3wire.h
* \brief 3-wire SPI low level driver.
****************************************************************************************
*/

#ifndef _APP_SPI_3WIRE_H_
#define _APP_SPI_3WIRE_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup SPI_3WIRE_DRV
 * \brief SPI 3wire driver
 * \{
******************************************************************************************/
 
#include <stdint.h>
#include "spi.h"
#include "gpio.h"
 
typedef struct
{
    GPIO_PORT port;
    GPIO_PIN pin;
} SPI_Pin;
 
struct SPI_Config
{
    SPI_Word_Mode_t SPI_Word_Mode;
    SPI_Role_t SPI_Role;
    SPI_Polarity_Mode_t SPI_Polarity_Mode;
    SPI_PHA_Mode_t SPI_PHA_Mode;
    SPI_MINT_Mode_t SPI_MINT_Mode;
    SPI_XTAL_Freq_t SPI_XTAL_Freq;
    SPI_Pin cs;
    SPI_Pin sdio;
    SPI_Pin clk;
};
 
/*****************************************************************************************
 * @brief Read from 3-wire SPI
 * @param[in] registerIndex: Target address (A6..A0)
 * @param[in] is_last_transaction: Ttrue if this is the last transaction of the SPI access
 * @return  byte read
******************************************************************************************/
uint8_t read_from_3wire_SPI_register(uint8_t registerIndex, bool is_last_transaction);

extern uint16_t TsradCounter;
extern SPI_Pin sdio;
extern SPI_Pin cs;

/*****************************************************************************************
 * @brief Initialize communication with the 3-wire SPI
 * @param[in] cfg: SDIO pin configuration
 * @param[in] TsradCounterToSet: Counter for the delay between Address and Data phase in reading
******************************************************************************************/
__forceinline void initialize_3wire_spi(struct SPI_Config *cfg,  uint16_t TsradCounterToSet)
{ 
    sdio.port = cfg->sdio.port;                                   // initialize with param data;
    sdio.pin = cfg->sdio.pin;   
    cs.port = cfg->cs.port;
    cs.pin = cfg->cs.pin;
            
    TsradCounter = TsradCounterToSet;                             // SPI read address-data delay (refer to sensor datasheet)
    
	// init SPI
    SetBits16(CLK_PER_REG, SPI_ENABLE, 1);                        // enable  clock for SPI                  
//	SetBits16(SPI_CTRL_REG,SPI_ON,0);                             // close SPI block, if opened              
//	SetBits16(SPI_CTRL_REG,SPI_WORD, cfg->SPI_Word_Mode);         // SPI word mode              
//	SetBits16(SPI_CTRL_REG,SPI_SMN, cfg->SPI_Role);               // SPI master mode              
//	SetBits16(SPI_CTRL_REG,SPI_POL, cfg->SPI_Polarity_Mode);      // SPI mode selection - polarity              
//	SetBits16(SPI_CTRL_REG,SPI_PHA, cfg->SPI_PHA_Mode);           // SPI mode selection - phase              
//	SetBits16(SPI_CTRL_REG,SPI_MINT, cfg->SPI_MINT_Mode);         // disable SPI interrupt to the ICU              
//	SetBits16(SPI_CTRL_REG,SPI_CLK, cfg->SPI_XTAL_Freq);          // SPI block clock divider              
    
    SetWord16(SPI_CTRL_REG,(((uint16_t)cfg->SPI_Word_Mode     ) << SHIF16(SPI_WORD) |  
                            ((uint16_t)cfg->SPI_Role          ) << SHIF16(SPI_SMN ) | 
                            ((uint16_t)cfg->SPI_Polarity_Mode ) << SHIF16(SPI_POL ) | 
                            ((uint16_t)cfg->SPI_PHA_Mode      ) << SHIF16(SPI_PHA ) | 
                            ((uint16_t)cfg->SPI_MINT_Mode     ) << SHIF16(SPI_MINT) | 
                            ((uint16_t)cfg->SPI_XTAL_Freq     ) << SHIF16(SPI_CLK )   ) & (~SPI_ON));
                            
	SetBits16(SPI_CTRL_REG,SPI_ON, 1);                            // enable SPI block              
	SetBits16(SPI_CTRL_REG1, SPI_FIFO_MODE, 3);                    // no FIFOs used
 	SetBits16(SPI_CTRL_REG, SPI_DO, 1);

    //configure pins
    GPIO_ConfigurePin( cfg->cs.port, cfg->cs.pin, OUTPUT, PID_SPI_EN, true);
    GPIO_SetPinFunction( cfg->sdio.port, cfg->sdio.pin, INPUT, PID_SPI_DI);
    GPIO_SetPinFunction( cfg->clk.port, cfg->clk.pin, OUTPUT, PID_SPI_CLK);
    
//    if (enable_interrupt)                                         
//        NVIC_EnableIRQ(SPI_IRQn);                                 // enable SPI interrupt, if MINT is '1' and enable_interrupt is set              
    
}

/*****************************************************************************************
 * @brief Deactivate communication with the 3-wire SPI
******************************************************************************************/
void deactivate_3wire_spi(void);

/*****************************************************************************************
 * @brief Write to 3-wire SPI
 * @param[in] registerIndex: Target address (A6..A0)
 * @param[in] valueToWrite: Value to write
 * 
 * @return  Number of read bytes
******************************************************************************************/
void write_to_3wire_SPI_register(uint8_t registerIndex, uint8_t valueToWrite);

/*****************************************************************************************
 * @brief Burst write to 3-wire SPI
 * @param[in] registerIndex: Target address (A6..A0)
******************************************************************************************/
void burst_write_to_3wire_SPI_register(uint8_t registerIndex);

/*****************************************************************************************
 * @brief Burst read from 3-wire SPI
 * @return  byte read
******************************************************************************************/
uint8_t burst_read_from_3wire_SPI_register(void);

/*****************************************************************************************
 * @brief Deactivate 3-wire SPI CS
******************************************************************************************/
void spi3wire_cs_high(void);

/*****************************************************************************************
 * @brief Activate 3-wire SPI CS
******************************************************************************************/
void spi3wire_cs_low(void);

/**
 * \}
 * \}
 * \}
 */

#endif // _APP_SPI_3WIRE_H_
