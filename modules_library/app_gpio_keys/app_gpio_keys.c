/*****************************************************************************************
 *
 * \file app_gpio_keys.c
 *
 * \brief GPIO key module source file
 * 
******************************************************************************************/

 /*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup GPIO_KEYS
 * \{	 
 * \addtogroup APP_GPIO_KEYS
 * \{
 ****************************************************************************************	 	 
 */
 
/*
 * INCLUDE FILES
******************************************************************************************/
 
#ifdef HAS_GPIO_KEYS

#include "port_platform.h"
#include "app_gpio_keys.h"
#include <app_gpio_keys_config.h>
#include "port_gpio_keys.h"

#if (GPIO_NUM_OF_KEYS > 8)
    #error "Number of keys must be <= 8"
#endif

#define BUTTON_SAME_SAMPLES_QUOTA (GPIO_DEBOUNCE_TIME_IN_MS*1000/GPIO_DEBOUNCE_PERIOD_IN_US)
typedef struct
{
    uint16_t same_samples_counter;
    bool previous_sample_state;   
} button_debouncing_status_t;

typedef void (*app_gpio_keys_notification_t)(enum gpio_key key, bool pressed);
static const app_gpio_keys_notification_t app_gpio_keys_notification = APP_GPIO_KEYS_NOTIFICATION_CB;

button_debouncing_status_t  button_debouncing_status_array[GPIO_NUM_OF_KEYS];

bool gpio_key_last_state[GPIO_NUM_OF_KEYS];

void app_gpio_keys_init(void)
{
    for (uint8_t i = 0; i < GPIO_NUM_OF_KEYS; i++) {        
        button_debouncing_status_array[i].same_samples_counter = 0;
        gpio_key_last_state[i] = false;
        button_debouncing_status_array[i].previous_sample_state = false;
    }
    if(app_gpio_keys_functions.app_gpio_keys_init) {
        app_gpio_keys_functions.app_gpio_keys_init();
    }
    else {
        ASSERT_ERROR(0);
    }
}
/*****************************************************************************************
 * \brief Returns the state of the keys
 * \return bool     true: gpio is active, false otherwise
******************************************************************************************/
static bool gpio_keys_get_state(uint8_t key)
{
    bool status;
    
    if(app_gpio_keys_functions.app_gpio_keys_get_state) {
        status = app_gpio_keys_functions.app_gpio_keys_get_state(app_gpio_keys_pins[key].port, app_gpio_keys_pins[key].pin);
    }
    else {
        ASSERT_ERROR(0);
    }
    return (app_gpio_keys_pins[key].high == 0) ? status : !status;
}

bool app_gpio_keys_debounce(void)
{
    bool current_sample_state;
    bool ret = false;
    
    ASSERT_ERROR(app_gpio_keys_notification != NULL);
    
    for (uint8_t i=0; i<GPIO_NUM_OF_KEYS; i++) {        
        current_sample_state = gpio_keys_get_state(i);
        
        if (button_debouncing_status_array[i].previous_sample_state == current_sample_state) {
            if (button_debouncing_status_array[i].same_samples_counter < BUTTON_SAME_SAMPLES_QUOTA) {
                button_debouncing_status_array[i].same_samples_counter++;
            }
        }
        else {
            ret = true; 
            button_debouncing_status_array[i].same_samples_counter = 0;
            button_debouncing_status_array[i].previous_sample_state = current_sample_state;                            
        }

        if (button_debouncing_status_array[i].same_samples_counter == BUTTON_SAME_SAMPLES_QUOTA) {
            if(gpio_key_last_state[i] != current_sample_state) {
                gpio_key_last_state[i] = current_sample_state;
                app_gpio_keys_notification((enum gpio_key)i, current_sample_state);   
            }
        }
    }
    return ret;
}

bool app_gpio_keys_is_pressed(uint8_t key)
{
    ASSERT_ERROR(key < GPIO_NUM_OF_KEYS);
    return gpio_key_last_state[key];
}

#endif // HAS_GPIO_KEYS

/**
 * \}
 * \}
 * \}
 */
