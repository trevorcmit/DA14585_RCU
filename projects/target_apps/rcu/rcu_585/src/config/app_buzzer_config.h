/**
 ****************************************************************************************
 *
 * \file app_buzzer_config.h
 *
 * \brief Buzzer module configuration file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */

#ifndef _APP_BUZZER_CONFIG_H_
#define _APP_BUZZER_CONFIG_H_

/**
 ****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup BUZZER_CFG
 *
 * \brief Buzzer module configuration
 * \{
 ****************************************************************************************
 */

#include "port_platform.h"
#include "port_buzzer.h"
#include "app_buzzer_defs.h"
/**
 ****************************************************************************************
 * \brief Buzzer Pin Configuration                                                                 
 ****************************************************************************************
 */

enum buzzer_pins_ids {
    BUZZER_PIN,
};

static const pin_type_t app_buzzer_pins[] = {
    [BUZZER_PIN]  = {.port = GPIO_PORT_0, .pin = GPIO_PIN_7, .high = 0, .mode_function = OUTPUT | PID_GPIO },
};



/**
 ****************************************************************************************
 * \brief Define the maximum number of notes per melody                                     
 ****************************************************************************************
 */
#define BUZZER_MAX_MELODY_NOTES     4

/**
 ****************************************************************************************
 * \brief The size of the melody buffer is the notes plus one byte at the beginning of the
 * buffer which indicates the number of melody notes that are going to be played(do not edit)                                        
 ****************************************************************************************
 */
#define BUZZER_MAX_MELODY_LENGTH    (BUZZER_MAX_MELODY_NOTES+1)


/**
 ****************************************************************************************
 * \brief Defines the duration of each note in ticks                                      
 ****************************************************************************************
 */
#define BUZZER_NOTE_DURATION_TICKS  5

typedef enum {
    BUZZER_CONNECTED_MELODY,
    BUZZER_DISCONNECTED_MELODY,
    BUZZER_LOWBATT_MELODY,
    BUZZER_TOTAL_MELODIES
}buzzer_melody_t;

/**
 ****************************************************************************************
 * \brief A table of simple melodies. The first byte indicates the number of 
 * Notes to be played from each melody                                      
 ****************************************************************************************
 */
static const uint16_t buzzerMelodies[BUZZER_TOTAL_MELODIES][BUZZER_MAX_MELODY_LENGTH] = {
    
    [BUZZER_CONNECTED_MELODY]       = {4, 1002, 795, 668, 668 },
    [BUZZER_DISCONNECTED_MELODY]    = {4, 668 ,795, 1002, 1002 },
    [BUZZER_LOWBATT_MELODY]         = {3, 1002, 1002, 1002},
};


/**
 ****************************************************************************************
 * \brief Buzzer platform API functions                                  
 ****************************************************************************************
 */
static const app_buzzer_util_funcs_t app_buzzer_funcs = {
    .app_buzzer_start    = port_buzzer_start,
    .app_buzzer_stop     = port_buzzer_stop,
    .app_buzzer_set_note = port_buzzer_change_note
};


/**
 * \}
 * \}
 * \}
 */

#endif // _APP_BUZZER_CONFIG_H_
