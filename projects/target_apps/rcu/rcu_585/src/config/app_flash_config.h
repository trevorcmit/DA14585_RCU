/*****************************************************************************************
 *
 * \file app_flash_config.h
 *
 * \brief  SPI Flash configuration file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/ 
 
#ifndef _APP_FLASH_CONFIG_H_
#define _APP_FLASH_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup FLASH_CFG
 *
 * \brief SPI Flash configuration
 * \{
******************************************************************************************/

#ifdef HAS_SPI_FLASH_STORAGE

#include <user_periph_setup.h>
#include "port_platform.h"

/**
 *************************************************************************************
 * Define HAS_FLASH_SPI_POWER_DOWN to use GPIO a pin to control SPI Flash power      
 * FLASH_SPI_POWER_DOWN_PORT and FLASH_SPI_POWER_DOWN_PIN must be defined as well    
 *************************************************************************************
 */
// #define HAS_FLASH_SPI_POWER_DOWN
// #define FLASH_SPI_POWER_DOWN_PORT GPIO_PORT_x
// #define FLASH_SPI_POWER_DOWN_PIN  GPIO_PIN_x

/**
 *************************************************************************************
 * pin configuration                                                                 
 *************************************************************************************
 */
enum flash_pin_ids {
    FLASH_SPI_CS_PIN,
#ifdef HAS_FLASH_SPI_POWER_DOWN
    FLASH_POWER_DOWN_PIN,
#endif    
};

static const pin_type_t app_flash_pins[] = {
    [FLASH_SPI_CS_PIN]      = {.port = GPIO_PORT_0, .pin = GPIO_PIN_3, .high = 1, .mode_function = INPUT_PULLUP | PID_GPIO },
#ifdef HAS_FLASH_SPI_POWER_DOWN
    [FLASH_POWER_DOWN_PIN]  = {.port = GPIO_PORT_XX, .pin = GPIO_PIN_XX, .high = 1, .mode_function = OUTPUT | PID_GPIO },
#endif    
};

enum flash_spi_pin_ids {
    FLASH_SPI_CLK_PIN,
    FLASH_SPI_DO_PIN,
    FLASH_SPI_DI_PIN,
};

static const pin_type_t app_flash_spi_pins[] = {
    [FLASH_SPI_CLK_PIN] = {.port = SPI_CLK_PORT, .pin = SPI_CLK_PIN, .high = 0,  .mode_function = INPUT_PULLUP | PID_GPIO },
    [FLASH_SPI_DO_PIN]  = {.port = SPI_DO_PORT,  .pin = SPI_DO_PIN,  .high = 0,  .mode_function = INPUT_PULLUP | PID_GPIO },
    [FLASH_SPI_DI_PIN]  = {.port = SPI_DI_PORT,  .pin = SPI_DI_PIN,  .high = 0,  .mode_function = INPUT_PULLUP | PID_GPIO },
};

#if defined(HAS_FLASH_SPI_POWER_DOWN) && (!defined(FLASH_SPI_POWER_DOWN_PORT) || !defined(FLASH_SPI_POWER_DOWN_PIN))
#error "FLASH_SPI_POWER_DOWN_PORT and FLASH_SPI_POWER_DOWN_PIN must be defined"
#endif

/*****************************************************************************************
 * SPI FLASH configuration
******************************************************************************************/
// SPI Flash options
// SPI flash must be compatible with Winbond W25X10 or W25X20 parts
// SPI Flash sector size is 4KB, page size is 256 bytes

#define SPI_FLASH_SIZE      0x40000

#define SPI_FLASH_VIRTUAL_PAGE_SIZE 0x100

#define SPI_FLASH_PAGE_SIZE 0x100
#define SPI_FLASH_PAGE      SPI_FLASH_PAGE_SIZE   // SPI Flash memory page size in bytes
#define SPI_FLASH_SECTOR    0x1000
    
void user_flash_erase_callback(void);

static const flash_erase_callback_t flash_erase_callback = user_flash_erase_callback;

#endif  // HAS_SPI_FLASH_STORAGE
    
/**
 * \}
 * \}
 * \}
 */

#endif	// _APP_FLASH_CONFIG_H_
