/*****************************************************************************************
 *
 * \file user_modules.c
 *
 * \brief
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
 * \addtogroup USER_APP
 * \{
 * \addtogroup USER_MODULES
 *
 * \{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/
#include "user_modules.h"
#include <user_modules_config.h>

#define CALL_CALLBACK(cb) {if (cb != NULL) cb();}

void user_modules_reserve_gpios(void)
{
    for(int i = 0; i < sizeof(user_module_config)/sizeof(module_config_t); i++) {
        pin_type_t *pin = user_module_config[i].pins_config.pins;
        if (pin != NULL){      
            for(int j = 0; j < user_module_config[i].pins_config.num_of_pins; j++) {
                PORT_RESERVE_GPIO(pin[j]);
            }
        }
    }
}

void user_modules_init_gpios(void)
{
    for(int i = 0; i < sizeof(user_module_config)/sizeof(module_config_t); i++) {
        if(user_module_config[i].init_gpios != NULL) {
            user_module_config[i].init_gpios();
        }
        else {
            pin_type_t *pin = user_module_config[i].pins_config.pins;
            if(pin != NULL) {
                for(int j = 0; j < user_module_config[i].pins_config.num_of_pins; j++) {
                    PORT_CONFIGURE_GPIO_DEFAULT(pin[j]);
                }
            }
        }
    }
}

void user_modules_init(void)
{
    for(int i = 0; i < sizeof(user_module_config)/sizeof(module_config_t); i++) {
        CALL_CALLBACK(user_module_config[i].init);
    }
}

void user_modules_on_disconnect(void)
{
    for(int i = 0; i < sizeof(user_module_config)/sizeof(module_config_t); i++) {
        CALL_CALLBACK(user_module_config[i].on_disconnect);
    }
}

bool user_modules_on_ble_powered(void)
{
    bool keep_powered = false;
    for(int i = 0; i < sizeof(user_module_config)/sizeof(module_config_t); i++) {
        if(user_module_config[i].on_ble_powered != NULL) {
            keep_powered |= user_module_config[i].on_ble_powered();
        }
    }
    
    return keep_powered;
}

uint8_t user_modules_on_system_powered(void)
{
    uint8_t power_status = false;
    for(int i = 0; i < sizeof(user_module_config)/sizeof(module_config_t); i++) {
        if(user_module_config[i].on_system_powered != NULL) {
            power_status |= user_module_config[i].on_system_powered();
        }
    }
    
    return power_status;
}

/**
 * \}
 * \}
 * \}
 */
