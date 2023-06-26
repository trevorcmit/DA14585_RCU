/*****************************************************************************************
 *
 * \file app_leds_defs.h
 *
 * \brief LED module definitions
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
 * \addtogroup LEDS
 * \{
 * \addtogroup APP_LEDS
 * \brief Definitions of LED handling functions used by several modules
 * \{
 ****************************************************************************************	 	 
 */  
 
#ifndef APP_LEDS_DEFS_H_
#define APP_LEDS_DEFS_H_


typedef enum {LED_ACTIVE_HIGH = 0,
              LED_ACTIVE_LOW  = 1,
} LED_GPIO_POLARITY;

typedef enum {LED_GPIO    = 0xFF, ///< The LED is connected to a GPIO defined by gpio_pad in led_io_t
              LED_CUSTOM  = 0x00, ///< The LED needs custom handling implemented in the callback defined in callback in led_io_t
} LED_type_t;

typedef enum {
	LED_RESULT_FAIL=0,
    LED_RESULT_OK,
    LED_RESULT_PENDING
} led_result_t;

enum led_command {
    LED_COMMAND_ENABLE_CONTROL = 0,
    LED_COMMAND_DISABLE_CONTROL,
    LED_COMMAND_TURN_ON,
    LED_COMMAND_TURN_OFF
};

typedef void (*leds_custom_type_callback_t)(enum led_command state);

typedef union {
    leds_custom_type_callback_t callback;
	LED_type_t type;
} led_io_t;

/** 
 ****************************************************************************************
 * \brief LED indication modes
 *
 * Define the LED indication pattern
******************************************************************************************/ 
enum led_mode {LED_BLINK,         ///< LED is turned on and then off for a number of times defined in count 
               LED_NO_BLINK,      ///< LED is turned on and then off once 
               LED_TURN_OFF,      ///< Keep LED off 
#ifdef LED_USE_DOUBLE_BLINK_FEATURE    
               LED_DOUBLE_BLINK   ///< LED is turned on and then off twice for a number of times defined in count 
#endif    
};

typedef struct {
    led_id_t id;
    enum led_mode mode;
    uint8_t count;          // number of times to blink
    bool high_priority;
    uint16_t on_time;       // in msec
    uint16_t off_time;      // in msec   
#ifdef LED_USE_RAMP_FEATURE        
    uint16_t ramp_on_time;  // in msec
    uint16_t ramp_off_time; // in msec
#endif    
    uint16_t delay;         // in msec
} led_params_t;

#endif // APP_LEDS_DEFS_H_

/**
 * \}
 * \}
 * \}
 */
