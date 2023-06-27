/*****************************************************************************************
 *
 * \file app_gpio_keys_config.h
 *
 * \brief GPIO keys module configuration header file.
 * 
******************************************************************************************/

#ifndef _APP_GPIO_KEYS_CONFIG_H_
#define _APP_GPIO_KEYS_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup GPIO_KEYS_CFG
 *
 * \brief GPIO key module configuration
 * \{
******************************************************************************************/

#include "port_gpio_keys.h"
#include "app_gpio_keys_defs.h"
#include "port_platform.h"

/*****************************************************************************************
 * \brief Define the keys to be used. Maximum number of keys is 8
******************************************************************************************/
#define GPIO_NUM_OF_KEYS     3

/*****************************************************************************************
 * pin configuration                                                                 
******************************************************************************************/
#define GPIO_KEY_0_PORT		 GPIO_PORT_2
#define GPIO_KEY_0_PIN		 GPIO_PIN_8
#define GPIO_KEY_0_POLARITY  ACTIVE_LOW
                             
#define GPIO_KEY_1_PORT		 GPIO_PORT_0
#define GPIO_KEY_1_PIN		 GPIO_PIN_1
#define GPIO_KEY_1_POLARITY  ACTIVE_LOW
                             
#define GPIO_KEY_2_PORT		 GPIO_PORT_0
#define GPIO_KEY_2_PIN		 GPIO_PIN_2
#define GPIO_KEY_2_POLARITY  ACTIVE_LOW


/*****************************************************************************************
 * \brief Key debouncing time
******************************************************************************************/
#define GPIO_DEBOUNCE_TIME_IN_MS        12

/*****************************************************************************************
 * \brief GPIO sampling period for debouncing
******************************************************************************************/
#define GPIO_DEBOUNCE_PERIOD_IN_US      500

/*****************************************************************************************
 * \brief Define the keys names
******************************************************************************************/
enum gpio_key {
    MOUSE_BUTTON_LEFT = 0,
    MOUSE_BUTTON_MIDDLE,
    MOUSE_BUTTON_RIGHT,
};


static const app_gpio_keys_funcs_t app_gpio_keys_functions = {
    .app_gpio_keys_init =      port_gpio_keys_init,
    .app_gpio_keys_get_state = port_gpio_get_pin_status
};

void user_gpio_keys_notification_cb(enum gpio_key key, bool pressed);

/*****************************************************************************************
 * \brief This callback will be called to notify the application that a key has been
 *        pressed or released.
******************************************************************************************/
#define APP_GPIO_KEYS_NOTIFICATION_CB user_gpio_keys_notification_cb
                                              
/**
 * \}
 * \}
 * \}
 */

#endif // _APP_GPIO_KEYS_CONFIG_H_
