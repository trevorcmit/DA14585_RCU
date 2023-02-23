/**
 ****************************************************************************************
 *
 * \file app_leds.h
 *
 * \brief This provides an API for driving LED indicators in a variety of
 * patterns such as turning on/off a LED, blinking a LED, ramping up/down a LED etc.
 *
 * Define symbol HAS_LED_INDICATORS to include this module in the application.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */
 
 /**
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup LEDS
 * \{
 * \addtogroup APP_LEDS
 * \brief Definitions of LED handling functions used by several modules
 * \{
 ****************************************************************************************	 	 
 */
 
#ifndef APP_LEDS_H_
#define APP_LEDS_H_

#ifdef HAS_LED_INDICATORS

#include <app_leds_config.h>
#include "port_platform.h"
#include "gpio.h"


/**
 ****************************************************************************************
 * \brief Initializes the LEDs
 ****************************************************************************************
 */
void app_leds_init(void);


/**
 ****************************************************************************************
 * \brief Turn on LED
 *
 * \param[in]  led_id    The id of the LED    
 * \param[in]  on_time  The time in msec that the LED will remain ON until 
 *                      it is automatically turned off
 *
 * \return    LED_RESULT_OK if LED patterns starts. 
 *            LED_RESULT_FAIL if LED pattern cannot be started
 *            LED_RESULT_PENDING if LED pattern is set to pending
 ****************************************************************************************
 */
led_result_t app_leds_on(led_id_t led_id, uint16_t on_time);


/**
 ****************************************************************************************
 * \brief Turn off LED uncoditionally
 *
 * \param[in] led_id  The id of the LED    
 ****************************************************************************************
 */
void app_leds_off(led_id_t led_id);


/**
 ****************************************************************************************
 * \brief Turn off LED combination.
 *
 * \param[in] param: The definition of the LEDs in the combination
 ****************************************************************************************
 */
#define LEDS_OFF_PARAM(param) app_leds_off_param(param, sizeof(param)/sizeof(led_params_t))    


/**
 ****************************************************************************************
 * \brief Turn off LED combination.
 *
 * \param[in] param: The definition of the LEDs in the combination
 * \param[in] num_of_leds: Number of LEDs in the combination defined in param
 *
 * \remarks   Use LEDS_OFF_PARAM(param) instead.
 ****************************************************************************************
 */
void app_leds_off_param(const led_params_t *param, uint8_t num_of_leds);


/**
 ****************************************************************************************
 * \brief Turn off all LEDs uncoditionally
 ****************************************************************************************
 */
void app_leds_all_off(void);


/**
 ****************************************************************************************
 * \brief Sets the LED to blink
 *
 * \param[in]  led_id   The id of the LED    
 * \param[in]  mode     Blink mode, either single or double blink 
 * \param[in]  on_time  The time in msec that the LED will remain on during blink 
 * \param[in]  off_time The time in msec that the LED will remain off during blink
 * \param[in]  count    Number of times to repeat the blink pattern
 *
 * \return    LED_RESULT_OK if LED patterns starts. 
 *            LED_RESULT_FAIL if LED pattern cannot be started
 *            LED_RESULT_PENDING if LED pattern is set to pending
 ****************************************************************************************
 */
led_result_t app_leds_blink(led_id_t led_id, enum led_mode mode, uint16_t on_time, uint16_t off_time, uint8_t count);


/**
 ****************************************************************************************
 * \brief Start LED indication pattern using an LED combination.
 *
 * \param[in] param: The definition of the patterns of the LEDs in the combination
 ****************************************************************************************
 */
#define LEDS_RAMP(param) app_leds_ramp(param, sizeof(param)/sizeof(led_params_t))    


/**
 ****************************************************************************************
 * \brief
 *
 * \param[in] param
 ****************************************************************************************
 */
#define LEDS_OFF(param) app_leds_off_param(param, sizeof(param)/sizeof(led_params_t))    


/**
 ****************************************************************************************
 * \brief Start LED indication pattern using an LED combination.
 *
 * \param[in] param: The definition of the patterns of the LEDs in the combination
 * \param[in] num_of_leds: Number of LEDs in the combination defined in param
 *
 * \return    LED_RESULT_OK if LED patterns starts. 
 *            LED_RESULT_FAIL if LED pattern cannot be started
 *            LED_RESULT_PENDING if LED pattern is set to pending
 *
 * \remarks   Use LEDS_RAMP(param) instead.
 ****************************************************************************************
 */
led_result_t app_leds_ramp(const led_params_t *param, uint8_t num_of_leds);


/**
 ****************************************************************************************
 * \brief Block the ramp feature. This function is used when timer resources used for 
 *        ramp features are needed by other modules
 *
 * \param[in] block: If true then block the ramp feature
 ****************************************************************************************
 */ 
void app_leds_block_ramp(bool block);


/**
 ****************************************************************************************
 * \brief Initialize LED GPIOs
 ****************************************************************************************
 */ 
void app_leds_init_gpios(void);


/**
 ****************************************************************************************
 * \brief Handler for LED timer 1 expiration event
 ****************************************************************************************
 */ 
void app_leds_timer_1_handler(void);


/**
 ****************************************************************************************
 * \brief Handler for LED timer 2 expiration event
 ****************************************************************************************
 */ 
void app_leds_timer_2_handler(void);

#endif // HAS_LED_INDICATORS


#endif // APP_LEDS_H_


/**
 * \}
 * \}
 * \}
 */
