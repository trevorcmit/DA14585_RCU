/**
 ****************************************************************************************
 *
 * \file app_mouse_config.h
 *
 * \brief  Mouse module configuration header file
 ****************************************************************************************
 */ 
 
#ifndef _APP_MOUSE_CONFIG_H_
#define _APP_MOUSE_CONFIG_H_

/**
 ****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup MOUSE_CFG
 *
 * \brief Mouse module configuration
 * \{
 ****************************************************************************************
 */

#define MOUSE_SPI_NCS_PORT   GPIO_PORT_0
#define MOUSE_SPI_NCS_PIN    GPIO_PIN_3
#define MOUSE_SPI_SCLK_PORT  GPIO_PORT_0
#define MOUSE_SPI_SCLK_PIN   GPIO_PIN_0
#define MOUSE_SPI_SDIO_PORT  GPIO_PORT_0
#define MOUSE_SPI_SDIO_PIN   GPIO_PIN_6

#define MOUSE_MOTION_PORT      GPIO_PORT_1
#define MOUSE_MOTION_PIN       GPIO_PIN_0  
#define MOUSE_MOTION_POLARITY  GPIO_ACTIVE_LOW

#define MOUSE_SEN_NRESET_PORT GPIO_PORT_2
#define MOUSE_SEN_NRESET_PIN  GPIO_PIN_4

#define MOUSE_MOTION_DATA_QUOTA (10)

/**
 * \}
 * \}
 * \}
 */

#endif	// _APP_MOUSE_CONFIG_H_
