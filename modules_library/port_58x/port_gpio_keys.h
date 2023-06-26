/*****************************************************************************************
 *
 * \file port_gpio_keys.h
 *
 * \brief The port_gpio_keys module provides an abstraction layer between the platform and
 * the application and is meant to be used in order to initialize GPIO pins for handling
 * key presses/releases/debouncing.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/
 
/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup GPIO_KEYS
 * \{
 * \addtogroup PORT_GPIO_KEYS
 * \{
 ****************************************************************************************	 
 */
 
#ifndef PORT_GPIO_KEYS_H_
#define PORT_GPIO_KEYS_H_

 
/*
 * INCLUDE FILES
******************************************************************************************/

#include "gpio.h"
#include "port_platform.h"


#define ACTIVE_HIGH 1
#define ACTIVE_LOW  0


/*****************************************************************************************
 * \brief GPIO Keys pin reservation function
******************************************************************************************/
void port_gpio_keys_declare(void);    
    
/*****************************************************************************************
 * \brief GPIO Keys initialization function
******************************************************************************************/    
void port_gpio_keys_init(void);

/*****************************************************************************************
 * \brief  GPIO Keys wakeup interrupt handler
******************************************************************************************/
void port_gpio_keys_wakeup_handler(void);

/*****************************************************************************************
 * \brief  GPIO Keys systick callback handler
******************************************************************************************/
void port_gpio_keys_systick_callback(void);

/*****************************************************************************************
 * \brief   Enables the GPIO_KEYS interrupt
******************************************************************************************/
void port_gpio_keys_enable_irq(void);


#endif // PORT_GPIO_KEYS_H_

/**
 * \}
 * \}
 * \}
 */

