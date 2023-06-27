/*****************************************************************************************
 *
 * \file app_gpio_keys.h
 *
 * \brief The gpio_keys module provides an API to initialize, check and process GPIOs
 *        for key presses or releases.
 *
 * Define symbol HAS_GPIO_KEYS to include this module in the application.
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

#ifndef APP_GPIO_KEYS_H_
#define APP_GPIO_KEYS_H_

#ifdef HAS_GPIO_KEYS
#include <app_gpio_keys_config.h>

#define PORT_UNUSED 0xFF
#define PIN_UNUSED  0xFF


#if (!defined(GPIO_KEY_0_PORT) || !defined(GPIO_KEY_0_PIN))
    #define GPIO_KEY_0_PORT (PORT_UNUSED)
    #define GPIO_KEY_0_PIN  (PIN_UNUSED)
#endif
#if (!defined(GPIO_KEY_1_PORT) || !defined(GPIO_KEY_1_PIN))
    #define GPIO_KEY_1_PORT (PORT_UNUSED)
    #define GPIO_KEY_1_PIN  (PIN_UNUSED)
#endif
#if (!defined(GPIO_KEY_2_PORT) || !defined(GPIO_KEY_2_PIN))
    #define GPIO_KEY_2_PORT (PORT_UNUSED)
    #define GPIO_KEY_2_PIN  (PIN_UNUSED)
#endif
#if (!defined(GPIO_KEY_3_PORT) || !defined(GPIO_KEY_3_PIN))
    #define GPIO_KEY_3_PORT (PORT_UNUSED)
    #define GPIO_KEY_3_PIN  (PIN_UNUSED)
#endif
#if (!defined(GPIO_KEY_4_PORT) || !defined(GPIO_KEY_4_PIN))
    #define GPIO_KEY_4_PORT (PORT_UNUSED)
    #define GPIO_KEY_4_PIN  (PIN_UNUSED)
#endif
#if (!defined(GPIO_KEY_5_PORT) || !defined(GPIO_KEY_5_PIN))
    #define GPIO_KEY_5_PORT (PORT_UNUSED)
    #define GPIO_KEY_5_PIN  (PIN_UNUSED)
#endif
#if (!defined(GPIO_KEY_6_PORT) || !defined(GPIO_KEY_6_PIN))
    #define GPIO_KEY_6_PORT (PORT_UNUSED)
    #define GPIO_KEY_6_PIN  (PIN_UNUSED)
#endif
#if (!defined(GPIO_KEY_7_PORT) || !defined(GPIO_KEY_7_PIN))
    #define GPIO_KEY_7_PORT (PORT_UNUSED)
    #define GPIO_KEY_7_PIN  (PIN_UNUSED)
#endif


#define _GPIO_KEYS_GET_PORT(x)      (GPIO_KEY_##x##_PORT)
#define _GPIO_KEYS_GET_PIN(x)       (GPIO_KEY_##x##_PIN)
#define _GPIO_KEYS_GET_POLARITY(x)  (GPIO_KEY_##x##_POLARITY)

#define GPIO_KEYS_GET_PORT(x)      _GPIO_KEYS_GET_PORT(x)
#define GPIO_KEYS_GET_PIN(x)       _GPIO_KEYS_GET_PIN(x)
#define GPIO_KEYS_GET_POLARITY(x)  _GPIO_KEYS_GET_POLARITY(x)

#define SET_WKUP_MASK_FROM_GPIO(m, x) ( (GPIO_KEY_##x##_PORT == m) ? (1 << GPIO_KEY_##x##_PIN) : 0 )

// Masks for the initialization of the WKUP controller
#define GPIO_WKUP_MASK_P0    (  SET_WKUP_MASK_FROM_GPIO(0, 0)  | SET_WKUP_MASK_FROM_GPIO(0, 1)  | SET_WKUP_MASK_FROM_GPIO(0, 2)  | SET_WKUP_MASK_FROM_GPIO(0, 3)    \
                              | SET_WKUP_MASK_FROM_GPIO(0, 4)  | SET_WKUP_MASK_FROM_GPIO(0, 5)  | SET_WKUP_MASK_FROM_GPIO(0, 6)  | SET_WKUP_MASK_FROM_GPIO(0, 7) )    
                                                                                                                                                                            
#define GPIO_WKUP_MASK_P1    (  SET_WKUP_MASK_FROM_GPIO(1, 0)  | SET_WKUP_MASK_FROM_GPIO(1, 1)  | SET_WKUP_MASK_FROM_GPIO(1, 2)  | SET_WKUP_MASK_FROM_GPIO(1, 3)    \
                              | SET_WKUP_MASK_FROM_GPIO(1, 4)  | SET_WKUP_MASK_FROM_GPIO(1, 5)  | SET_WKUP_MASK_FROM_GPIO(1, 6)  | SET_WKUP_MASK_FROM_GPIO(1, 7) )   

#define GPIO_WKUP_MASK_P2    (  SET_WKUP_MASK_FROM_GPIO(2, 0)  | SET_WKUP_MASK_FROM_GPIO(2, 1)  | SET_WKUP_MASK_FROM_GPIO(2, 2)  | SET_WKUP_MASK_FROM_GPIO(2, 3)    \
                              | SET_WKUP_MASK_FROM_GPIO(2, 4)  | SET_WKUP_MASK_FROM_GPIO(2, 5)  | SET_WKUP_MASK_FROM_GPIO(2, 6)  | SET_WKUP_MASK_FROM_GPIO(2, 7) )  

#define GPIO_WKUP_MASK_P3    (  SET_WKUP_MASK_FROM_GPIO(3, 0)  | SET_WKUP_MASK_FROM_GPIO(3, 1)  | SET_WKUP_MASK_FROM_GPIO(3, 2)  | SET_WKUP_MASK_FROM_GPIO(3, 3)    \
                              | SET_WKUP_MASK_FROM_GPIO(3, 4)  | SET_WKUP_MASK_FROM_GPIO(3, 5)  | SET_WKUP_MASK_FROM_GPIO(3, 6)  | SET_WKUP_MASK_FROM_GPIO(3, 7) )

#define GPIO_WKUP_MASK   WKUP_MASK( GPIO_WKUP_MASK_P0, GPIO_WKUP_MASK_P1, GPIO_WKUP_MASK_P2, GPIO_WKUP_MASK_P3)



static const pin_type_t app_gpio_keys_pins[GPIO_NUM_OF_KEYS] = {
#if (GPIO_NUM_OF_KEYS>0)    
    {.port = GPIO_KEY_0_PORT, .pin = GPIO_KEY_0_PIN, .high = (GPIO_KEY_0_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_0_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif                                                                                     
#if (GPIO_NUM_OF_KEYS>1)                                                                   
    {.port = GPIO_KEY_1_PORT, .pin = GPIO_KEY_1_PIN, .high = (GPIO_KEY_1_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_1_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif                                                                                     
#if (GPIO_NUM_OF_KEYS>2)                                                                   
    {.port = GPIO_KEY_2_PORT, .pin = GPIO_KEY_2_PIN, .high = (GPIO_KEY_2_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_2_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif                                                                                     
#if (GPIO_NUM_OF_KEYS>3)                                                                   
    {.port = GPIO_KEY_3_PORT, .pin = GPIO_KEY_3_PIN, .high = (GPIO_KEY_3_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_3_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif                                                                                     
#if (GPIO_NUM_OF_KEYS>4)                                                                   
    {.port = GPIO_KEY_4_PORT, .pin = GPIO_KEY_4_PIN, .high = (GPIO_KEY_4_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_4_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif                                                                                     
#if (GPIO_NUM_OF_KEYS>5)                                                                   
    {.port = GPIO_KEY_5_PORT, .pin = GPIO_KEY_5_PIN, .high = (GPIO_KEY_5_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_5_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif                                                                                     
#if (GPIO_NUM_OF_KEYS>6)                                                                   
    {.port = GPIO_KEY_6_PORT, .pin = GPIO_KEY_6_PIN, .high = (GPIO_KEY_6_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_6_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif                                                                                     
#if (GPIO_NUM_OF_KEYS>7)                                                                   
    {.port = GPIO_KEY_7_PORT, .pin = GPIO_KEY_7_PIN, .high = (GPIO_KEY_7_POLARITY == ACTIVE_LOW), .mode_function = ((GPIO_KEY_7_POLARITY == ACTIVE_LOW) ?  INPUT_PULLUP : INPUT_PULLDOWN) | PID_GPIO },
#endif    
};

 
/*****************************************************************************************
 * \brief Initialize GPIO key handling
******************************************************************************************/ 
void app_gpio_keys_init(void);

/*****************************************************************************************
 * \brief Debounce GPIO key presses and releases
 *
 * \return bool true if debounce is ok, false otherwise
******************************************************************************************/ 
bool app_gpio_keys_debounce(void);

/*****************************************************************************************
 * \brief    Check if a GPIO key is pressed
 *
 * \param[in]   key
 *
 * \return   True if key is pressed
******************************************************************************************/ 
bool app_gpio_keys_is_pressed(uint8_t key);


#endif // HAS_GPIO_KEYS

#endif // APP_GPIO_KEYS_H_

/**
 * \}
 * \}
 * \}
 */
