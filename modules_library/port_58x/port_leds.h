/**
 ****************************************************************************************
 *
 * \file port_leds.c
 *
 * \brief The port_leds module provides a hardware abstraction layer for initializing and
 * handling the MCU's pins for LED driving. Internally the module uses the MCU's timers and PWM 
 * peripherals in order to drive the LEDs in various patterns such as blinking, ramping up/down.
 * The software LED timer1 and LED timer 2 timers are used in order to time the LED blinking/ramping
 * while the hardware timers and PWM peripheral are used in order to dim/ramp the LEDs.
 * 
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
 * \addtogroup PORT_LEDS
 * \{
 ****************************************************************************************	 
 */

#ifndef PORT_LEDS_H_
#define PORT_LEDS_H_

#include <stdint.h>

#define LED_TIMER_CLOCK_DIVIDER      CLK_PER_REG_TMR_DIV_8
#define LED_TIMER_FREQUENCY          2000000
#define LED_RAMP_PWM_COUNTER_MAX     (LED_TIMER_FREQUENCY / LED_RAMP_PWM_FREQUENCY_IN_HZ)
#define LED_RAMP_PWM_COUNTER_MIN     (LED_RAMP_PWM_COUNTER_MAX * LED_RAMP_PWM_MIN_DC / 100)
#define LED_RAMP_STEP_PERIOD_DIVIDER (LED_TIMER_FREQUENCY * LED_RAMP_STEP_PERIOD_IN_MS / 1000)
#define LED_RAMP_STEP_MULTIPLIER     ((LED_RAMP_PWM_COUNTER_MAX-LED_RAMP_PWM_COUNTER_MIN) * LED_RAMP_STEP_PERIOD_IN_MS)

typedef void (*led_ramp_handler_t)(void);

typedef void (*led_timer_handler_t)(void);

/**
 ****************************************************************************************
 * \brief Reserves LED Pins
 ****************************************************************************************
 */
void port_leds_declare_led_gpios(void);

/**
 ****************************************************************************************
 * \brief Initializes a LED Pin with their default configuration
 *
 * \param[in] idx The LED pin ID 
 ****************************************************************************************
 */
void port_leds_init_gpios_default(uint8_t idx);

/**
 ****************************************************************************************
 * \brief Initializes a LED Pin for PWM driving
 *
 * \param[in] idx The LED pin ID 
 ****************************************************************************************
 */
void port_leds_init_gpios_pwm(uint8_t idx);

/**
 ****************************************************************************************
 * \brief Initializes a LED Pin as Output
 *
 * \param[in] idx The LED pin ID
 ****************************************************************************************
 */
void port_leds_init_gpios_output(uint8_t idx);

/**
 ****************************************************************************************
 * \brief Inits Timer0/Timer2 and PWM peripherals, starts the PWM in order to drive the leds
 *
 * \param[in] handler a function to handle pwm events
 * \param[in] pwmTime The desired PWM duty cycle
 ****************************************************************************************
 */
void port_leds_start_pwm(led_ramp_handler_t handler, uint16_t pwmTime);

/**
 ****************************************************************************************
 * \brief Stops the PWM related peripherals
 ****************************************************************************************
 */
void port_leds_stop_pwm(void);

/**
 ****************************************************************************************
 * \brief Sets the desired PWM duty cycle
 *
 * \param[in] rampVal The desired PWM duty cycle
 *
 * \return true if the value has been applied successfully
 ****************************************************************************************
 */
bool port_leds_set_next_ramp_lvl(int16_t rampVal);

/**
 ****************************************************************************************
 * \brief Checks if a LED pin is in conflict with any of the debug pins
 * \param[in] led_id The LED pin's id
 *
 * \return True if there is a conflict. False otherwise
 ****************************************************************************************
 */
bool port_leds_dbg_confict(uint8_t led_id);

/**
 ****************************************************************************************
 * \brief Checks if LED boost is enabled
 *
 * \return True if boost is enabled. False otherwise
 ****************************************************************************************
 */
bool port_leds_boost_enabled(void);

/**
 ****************************************************************************************
 * \brief LED timer 1 handling function
 ****************************************************************************************
 */
void port_leds_timer1_handler(void);

/**
 ****************************************************************************************
 * \brief LED timer 2 handling function
 ****************************************************************************************
 */
void port_leds_timer2_handler(void);

#endif // PORT_LEDS_H_

/**
 * \}
 * \}
 * \}
 */
