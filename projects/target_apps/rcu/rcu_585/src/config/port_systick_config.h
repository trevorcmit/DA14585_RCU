/**
 ****************************************************************************************
 *
 * \file port_systick_config.h
 *
 * \brief  Systick controller configuration file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */ 
 
#ifndef _PORT_SYSTICK_CONFIG_H
#define _PORT_SYSTICK_CONFIG_H

/**
 ****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup SYSTICK_CFG
 *
 * \brief Systick module configuration
 * \{
 ****************************************************************************************
 */

#include "port_gpio_keys.h"
#include "user_rcu.h"
#include "user_rcu_motion.h"

#define SYSTICK_PERIOD_IN_US 500

enum port_systick_channel {
#ifdef HAS_GPIO_KEYS    
    SYSTICK_GPIO_KEYS_CHANNEL,
#endif    
    
#ifdef HAS_MOUSE
    SYSTICK_MOUSE_SENSOR_CHANNEL,
#endif    
#ifdef HAS_ACTION_INACTIVITY_TIMEOUT
    SYSTICK_ACTION_INACTIVITY_CHANNEL,
#endif    
    NUM_OF_SYSTICK_CHANNELS  // Do not remove. Must always be the last entry.
};

static const systick_config_t systick_config[NUM_OF_SYSTICK_CHANNELS] = {
#ifdef HAS_GPIO_KEYS    
    [SYSTICK_GPIO_KEYS_CHANNEL].callback = port_gpio_keys_systick_callback,
#endif    
    
#ifdef HAS_MOUSE
    [SYSTICK_MOUSE_SENSOR_CHANNEL].callback = user_systick_mouse_callback,
#endif    
    
#ifdef HAS_ACTION_INACTIVITY_TIMEOUT
    [SYSTICK_ACTION_INACTIVITY_CHANNEL].callback = user_systick_callback,
#endif   
};

/**
 * \}
 * \}
 * \}
 */

#endif	// _PORT_SYSTICK_CONFIG_H
