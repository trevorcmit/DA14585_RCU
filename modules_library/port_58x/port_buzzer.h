/**
 ****************************************************************************************
 *
 * \file port_buzzer.h
 *
 * \brief The port_buzzer module provides an abstraction layer between the platform and the
 * application for initializing and using a ping for driving a buzzer to play musical notes.
 * For that purpose, the module uses the platform's timers and pwm peripherals. 
 *
 * Copyright (C) 2016. Dialog Semiconductor Ltd, unpublished work. This computer
 * program includes Confidential, Proprietary Information and is a Trade Secret of
 * Dialog Semiconductor Ltd.  All use, disclosure, and/or reproduction is prohibited
 * unless authorized in writing. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */

 
/**
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BUZZER
 * \{
 * \addtogroup PORT_BUZZER
 * \{
 ****************************************************************************************	 
 */ 
 
#ifndef PORT_BUZZER_H
#define PORT_BUZZER_H

#ifdef HAS_SOUND_INDICATION

#include "app_buzzer_defs.h"

/**
 ****************************************************************************************
 * \brief Buzzer Pin Declaration/Reservation function 
 ****************************************************************************************
 */
void port_buzzer_pins_declare(void);


/**
 ****************************************************************************************
 * \brief Buzzer Pin Initialization function 
 ****************************************************************************************
 */
void port_buzzer_pins_init(void);


/**
 ****************************************************************************************
 * \brief Inits Timer0 and PWM peripherals and starts the PWM
 *
 * \param[in] hndl a function to handle buzzer note end events
 ****************************************************************************************
 */
void port_buzzer_start(buzzer_note_handler_t hndl);


/**
 ****************************************************************************************
 * \brief Sets the next note to be played while buzzer is active
 *
 * \param[in] noteValue  the note to be played
 * \param[in] noteRepeat the number of times to repeat the note
 ****************************************************************************************
 */
void port_buzzer_change_note(uint32_t noteValue, uint8_t noteRepeat);


/**
 ****************************************************************************************
 * \brief Deinitializes and stops Buzzer related peripherals
 ****************************************************************************************
 */
void port_buzzer_stop(void);

#endif
#endif // PORT_BUZZER_H

/**
 * \}
 * \}
 * \}
 */
