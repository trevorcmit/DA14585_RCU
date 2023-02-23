/**
 ****************************************************************************************
 *
 * \file app_leds_config.h
 *
 * \brief  LED module configuration header file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */ 
 
#ifndef _APP_LEDS_CONFIG_H_
#define _APP_LEDS_CONFIG_H_

/**
 ****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup LEDS_CFG
 *
 * \brief LEDs module configuration
 * \{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Define LED_USE_RAMP_FEATURE to enable LED intensity to ramp-up and down when turned 
 * on and off respectively.
 ****************************************************************************************
 */
#define LED_USE_RAMP_FEATURE

#ifdef LED_USE_RAMP_FEATURE
    /**
     ************************************************************************************
     * The frequency in Hz of the PWM that is used for dimming the LED
     ************************************************************************************
     */
    #define LED_RAMP_PWM_FREQUENCY_IN_HZ  200 

    /**
     ************************************************************************************
     * The minimum duty cycle of the PWM in %
     ************************************************************************************
     */
    #define LED_RAMP_PWM_MIN_DC           1 
    
    /**
     ************************************************************************************
     * The step period in msec used for increasing or decreasing the PWM duty cycle
     ************************************************************************************
     */
    #define LED_RAMP_STEP_PERIOD_IN_MS    10 
#endif

/**
 ****************************************************************************************
 * Define LED_USE_DOUBLE_BLINK_FEATURE to enable LED double-blink function
 ****************************************************************************************
 */
#define LED_USE_DOUBLE_BLINK_FEATURE

#ifdef LED_USE_DOUBLE_BLINK_FEATURE
    #define LED_DOUBLE_BLINK_DELAY   200 // in msec
#endif

/**
 ****************************************************************************************
 * Define the number of LEDs and the corresponding names
 ****************************************************************************************
 */
#undef HAS_LED_CAPS_LOCK
#undef HAS_LED_NUM_LOCK
#undef HAS_LED_SCROLL_LOCK
#undef HAS_LED_FN_LOCK

typedef enum {
    LED_RED=0, 
    LED_GREEN, 
#ifdef HAS_LED_CAPS_LOCK	
	LED_CAPS_LOCK,
#endif	
#ifdef HAS_LED_NUM_LOCK	
	LED_NUM_LOCK,
#endif	
#ifdef HAS_LED_SCROLL_LOCK	
	LED_SCROLL_LOCK,
#endif	
#ifdef HAS_LED_FN_LOCK
	LED_FN_LOCK,
#endif		
	NUM_OF_LEDS
} led_id_t;                
                
#include "app_leds_defs.h"
#include <port_timer_config.h>

/**
 ****************************************************************************************
 * LED timer assignment. Two timers are available. LEDs using the same timer cannot 
 * be used simultaneously
 ****************************************************************************************
 */
static const enum port_timer_ids led_timer[NUM_OF_LEDS] = {[LED_RED]        = APP_LED_1_TIMER,
                                                           [LED_GREEN]      = APP_LED_2_TIMER,
#ifdef HAS_LED_CAPS_LOCK	                          
										                   [LED_CAPS_LOCK]  = APP_LED_1_TIMER,
#endif	                                                  
#ifdef HAS_LED_NUM_LOCK	                                  
										                   [LED_NUM_LOCK]   = APP_LED_1_TIMER,
#endif	                                                  
#ifdef HAS_LED_SCROLL_LOCK	                              
										                   [LED_SCROLL_LOCK]= APP_LED_1_TIMER,
#endif	
};

static const pin_type_t app_led_pins[] = {
    [LED_RED]         = {.port = GPIO_PORT_1,  .pin = GPIO_PIN_2,  .high = LED_ACTIVE_HIGH,  .mode_function = OUTPUT | PID_GPIO },
    [LED_GREEN]       = {.port = GPIO_PORT_1,  .pin = GPIO_PIN_3,  .high = LED_ACTIVE_HIGH,  .mode_function = OUTPUT | PID_GPIO },
#ifdef HAS_LED_CAPS_LOCK	
    [LED_CAPS_LOCK]   = {.port = DEFINE_LED_PORT, .pin = DEFINE_LED_PIN, .high = DEFINE_LED_POLARITY, .mode_function = OUTPUT | PID_GPIO },
#endif	
#ifdef HAS_LED_NUM_LOCK	
    [LED_NUM_LOCK]    = {.port = DEFINE_LED_PORT, .pin = DEFINE_LED_PIN, .high = DEFINE_LED_POLARITY, .mode_function = OUTPUT | PID_GPIO },
#endif	
#ifdef HAS_LED_SCROLL_LOCK	
    [LED_SCROLL_LOCK] = {.port = DEFINE_LED_PORT, .pin = DEFINE_LED_PIN, .high = DEFINE_LED_POLARITY, .mode_function = OUTPUT | PID_GPIO },
#endif	
#ifdef HAS_LED_FN_LOCK	
    [LED_FN_LOCK]     = {.port = DEFINE_LED_PORT, .pin = DEFINE_LED_PIN, .high = DEFINE_LED_POLARITY, .mode_function = OUTPUT | PID_GPIO },
#endif	
};

static const led_io_t led_pads[] = {[LED_RED]         = {.type=LED_GPIO},  
                                    [LED_GREEN]       = {.type=LED_GPIO},
#ifdef HAS_LED_CAPS_LOCK	
                                    [LED_CAPS_LOCK]   = {.type=LED_GPIO},
#endif	
#ifdef HAS_LED_NUM_LOCK	
                                    [LED_NUM_LOCK]    = {.type=LED_GPIO},
#endif	
#ifdef HAS_LED_SCROLL_LOCK	
                                    [LED_SCROLL_LOCK] = {.type=LED_GPIO},
#endif	
#ifdef HAS_LED_FN_LOCK	
                                    [LED_FN_LOCK]     = {.type=LED_GPIO},
#endif	
								   }; 

static const led_params_t led_connected_param[] = {
    {.id             = LED_GREEN,
     .mode           = LED_NO_BLINK,
     .high_priority  = true,
     .on_time        = 100,
     .off_time       = 0,
#ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
#endif																		
     .count          = 1,
     .delay          = 0
    }
};		
static const led_params_t led_disconnected_param[] = {
    {.id             = LED_GREEN,
     .mode           = LED_BLINK,
     .high_priority  = true,
     .on_time        = 50,
     .off_time       = 100,
#ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
#endif																		
     .count          = 3,
     .delay          = 0}
};

static const led_params_t led_advertise_param[] = {
    {.id             = LED_RED,
     .mode           = LED_BLINK,
     .high_priority  = true,
     .on_time        = 500,
     .off_time       = 500,
#ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 200,
     .ramp_off_time  = 200,
#endif																		
     .count          = 0xFF,
     .delay          = 0
    }
};

static const led_params_t led_connection_in_progress_param[] = {
    {.id             = LED_GREEN,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 20,
     .off_time       = 80,
#ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
#endif																		
     .count          = 0xFF,
     .delay          = 0
    }
};

static const led_params_t led_battery_low_param[] = {
    {.id             = LED_RED,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 400,
     .off_time       = 1600,
#ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 200,
     .ramp_off_time  = 200,
#endif
     .count          = 15,
     .delay          = 1000
    },
    {.id             = LED_GREEN,   // Blink RED LED only when green LED is IDLE
     .mode           = LED_TURN_OFF,
     .high_priority  = false,
     .on_time        = 0,
     .off_time       = 0,
#ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
#endif
     .count          = 0,
     .delay          = 0
    }
};

#if (BLE_SUOTA_RECEIVER)
static const led_params_t led_suota_start_param[] = {
    {.id             = LED_RED,
     .mode           = LED_BLINK,
     .high_priority  = true,
     .on_time        = 150,
     .off_time       = 850,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 0xFF,
     .delay          = 0
    }
};
#endif

static const led_params_t led_motion_kbd_page_param[] = {
    {.id             = LED_GREEN,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 50,
     .off_time       = 900,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 0xFF,
     .delay          = 0
    }
};

#ifdef HAS_IR
static const led_params_t led_ir_mode_on_param[] = {
    {.id             = LED_RED,
     .mode           = LED_NO_BLINK,
     .high_priority  = false,
     .on_time        = 1000,
     .off_time       = 1000,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 1,
     .delay          = 0
    }
};

static const led_params_t led_ir_mode_off_param[] = {
    {.id             = LED_RED,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 100,
     .off_time       = 100,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 2,
     .delay          = 0
    }
};
#endif

#ifdef AUDIO_TEST_MODE
static const led_params_t led_audio_test_mode_mic_on_param[] = {
    {.id             = LED_RED,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 100,
     .off_time       = 100,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 0xFF,
     .delay          = 0
    },
    {.id             = LED_GREEN,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 100,
     .off_time       = 100,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 0xFF,
     .delay          = 100
    }
};

static const led_params_t led_audio_test_mode_mic_off_param[] = {
    {.id             = LED_RED,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 100,
     .off_time       = 900,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 0xFF,
     .delay          = 0
    },
    {.id             = LED_GREEN,
     .mode           = LED_BLINK,
     .high_priority  = false,
     .on_time        = 100,
     .off_time       = 900,
    #ifdef LED_USE_RAMP_FEATURE
     .ramp_on_time   = 0,
     .ramp_off_time  = 0,
    #endif																		
     .count          = 0xFF,
     .delay          = 500
    }
};

#endif

/**
 * \}
 * \}
 * \}
 */

#endif	// _APP_LEDS_CONFIG_H_
