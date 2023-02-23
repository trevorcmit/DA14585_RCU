/**
 ****************************************************************************************
 *
 * \file user_periph_setup.h
 *
 * \brief Peripherals setup header file.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/**
 ****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "arch.h"
#include "da1458x_periph_setup.h"
#include "i2c_core.h"

/**
 ****************************************************************************************
 * DEFINES
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * UART2 pin configuration (debug print console)
 ****************************************************************************************
*/
#ifdef CFG_PRINTF_UART2
    #define UART_TX_PORT  GPIO_PORT_0
    #define UART_TX_PIN   GPIO_PIN_4

    #ifndef DISABLE_UART_RX    
        #define UART_RX_PORT  GPIO_PORT_0
        #define UART_RX_PIN   GPIO_PIN_5
    #endif             
#endif

/**
 ***************************************************************************************
 * I2C pin configuration                                                                
 ***************************************************************************************
*/
#define I2C_SDA_PORT          GPIO_PORT_2
#define I2C_SDA_PIN           GPIO_PIN_0

#define I2C_SCL_PORT          GPIO_PORT_2
#define I2C_SCL_PIN           GPIO_PIN_1

/**
 ***************************************************************************************
 * SPI pin configuration                                                                
 ***************************************************************************************
*/
#define SPI_CLK_PORT   GPIO_PORT_0
#define SPI_CLK_PIN    GPIO_PIN_0

#define SPI_DO_PORT    GPIO_PORT_0
#define SPI_DO_PIN     GPIO_PIN_6

#define SPI_DI_PORT    GPIO_PORT_0
#define SPI_DI_PIN     GPIO_PIN_5

/****************************************************************************************/
/* SPI FLASH configuration                                                              */
/****************************************************************************************/

#ifdef HAS_SPI_FLASH_STORAGE
    // Values defined in app_flash_config.h
    #define SPI_FLASH_DEFAULT_SIZE  SPI_FLASH_SIZE          
    #define SPI_FLASH_DEFAULT_PAGE  SPI_FLASH_PAGE_SIZE
    #define SPI_SECTOR_SIZE         SPI_FLASH_SECTOR
#else
    #define SPI_FLASH_DEFAULT_SIZE  131072    
    #define SPI_FLASH_DEFAULT_PAGE  256
    #define SPI_SECTOR_SIZE         4096
#endif

/**
 ***************************************************************************************
 * LED and button configuration                                                         
 ***************************************************************************************
 */
#define USE_BAT_LEVEL_ALERT 0
#define GPIO_BAT_LED_PORT       GPIO_PORT_1
#define GPIO_BAT_LED_PIN        GPIO_PIN_2

#define I2C_AZOTOUCH_SLAVE_ADDRESS 							0x74            // Set slave device address
#define I2C_AZOTOUCH_SPEED_MODE    							I2C_FAST    	// Speed mode: I2C_STANDARD (100 kbits/s), I2C_FAST (400 kbits/s)
#define I2C_AZOTOUCH_ADDRESS_MODE  							I2C_7BIT_ADDR   // Addressing mode: {I2C_7BIT_ADDR, I2C_10BIT_ADDR}



/**
 ****************************************************************************************
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
void periph_init(void);

void GPIO_reservations(void);

#endif // _USER_PERIPH_SETUP_H_
