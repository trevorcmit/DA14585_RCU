/*****************************************************************************************
 *
 * \file port_touchpad.h
 *
 * \brief The port_touchpad module provides a hardware abstraction layer for interacting
 * with a touchpad module. The functions provide a simple API for initializing touch module
 * related peripherals as well as setting an application touchpad event handler
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
 * \addtogroup TOUCHPAD
 * \{
 * \addtogroup PORT_TOUCHPAD
 * \{
 ****************************************************************************************	 
 */

#ifndef PORT_TOUCHPAD_H
#define PORT_TOUCHPAD_H

#include <stdint.h>
#include "app_touchpad_defs.h"

#ifndef HAS_I2C
    #define HAS_I2C
#endif

#define AZOTEQ_TOUCHPAD_IQS_5XX                 0
#define AZOTEQ_TOUCHPAD_IQS_263                 1

#define TOUCHPAD_LOCAL_FIFO_MAX_SIZE            8

#define TOUCHPAD_PIN_WAKEUP_PIN_POLARITY		WKUPCT_PIN_POLARITY( TOUCHPAD_INT_PORT, TOUCHPAD_INT_PIN, TOUCHPAD_INT_POLARITY)

/*****************************************************************************************
 * \brief Touchpad Platform de-init function
******************************************************************************************/
void port_touchpad_deinit(void);

/*****************************************************************************************
 * \brief Enables Touchpad interrupts
******************************************************************************************/
void port_touchpad_enable_irq(void);

/*****************************************************************************************
 * \brief Disables Touchpad interrupts
******************************************************************************************/
void port_touchpad_disable_irq(void);

/*****************************************************************************************
 * \brief Touchpad interrupt handling function
******************************************************************************************/
void port_touchpad_handler(void);

/*****************************************************************************************
 * \brief Touchpad gpio reservation function
******************************************************************************************/
void port_touchpad_reserve_gpios(void);
 
/*****************************************************************************************
 * \brief Touchpad gpio initialization function
******************************************************************************************/
void port_touchpad_init_gpios(void);


/*****************************************************************************************
 * \brief Touchpad module initialization function
******************************************************************************************/
void port_touchpad_init(void);


/*****************************************************************************************
 * \brief Touchpad module de-initialization function
******************************************************************************************/
void port_touchpad_deinit(void);



/*****************************************************************************************
 * \brief Touchpad polling function. Polls the touchpad FIFO for any new events
 * \param[out] touchpad_element A pointer to where the touchpad event(if any) will be placed
 * \return bool True if there are any new events, false otherwise
******************************************************************************************/
bool port_touchpad_poll(app_touchpad_evt_t * touchpad_element);
#endif // PORT_TOUCHPAD_H

/**
 * \}
 * \}
 * \}
 */
