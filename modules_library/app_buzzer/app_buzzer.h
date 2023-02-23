/**
 ****************************************************************************************
 *
 * \file app_buzzer.h
 *
 * \brief The buzzer application provides a simple API to drive a buzzer for producing
 * sound indications/melodies.
 *
 * Define symbol HAS_SOUND_INDICATION to include this module in the application.
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
 * \addtogroup BUZZER
 * \{
 * \addtogroup APP_BUZZER
 *
 * \brief Buzzer Application Header
 * \{
 ****************************************************************************************	 
 */
 
#ifndef APP_BUZZER_H
#define APP_BUZZER_H

#include <app_buzzer_config.h>

/**
 ****************************************************************************************
 * \brief Initializes the buzzer to start and play a melody
 *
 * \param[in] melody The melody to be played
 ****************************************************************************************
 */
void app_buzzer_play_melody(buzzer_melody_t melody);

#endif // APP_BUZZER_H

/**
 * \}
 * \}
 * \}
 */
