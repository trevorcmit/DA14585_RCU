/*****************************************************************************************
 *
 * \file app_gpio_keys_defs.h
 *
 * \brief GPIO key module definitions
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

#ifndef APP_GPIO_KEYS_DEFS_H_
#define APP_GPIO_KEYS_DEFS_H_
#include <stdint.h>

typedef void (*app_gpios_init_t)(void);
typedef bool (*app_gpios_get_pin_status_t)(uint8_t port, uint8_t pin);


typedef struct {
    app_gpios_init_t app_gpio_keys_init;
    app_gpios_get_pin_status_t app_gpio_keys_get_state;
}app_gpio_keys_funcs_t;

#endif // APP_GPIO_KEYS_DEFS_H_

/**
 * \}
 * \}
 * \}
 */
