/*****************************************************************************************
 *
 * \file app_buzzer_defs.h
 *
 * \brief Buzzer module definitions
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BUZZER
 * \{
 * \addtogroup APP_BUZZER
 *
 * \brief Buzzer Application Definitions
 * \{
******************************************************************************************/

#ifndef APP_BUZZER_DEFS_H
#define APP_BUZZER_DEFS_H

#include <stdint.h>

typedef void (*buzzer_note_handler_t)(void);

typedef void (*app_buzzer_start_t)(buzzer_note_handler_t handler);
typedef void (*app_buzzer_stop_t)(void);
typedef void (*app_buzzer_set_note_t)(uint32_t note, uint8_t duration);

typedef struct {
    app_buzzer_start_t  app_buzzer_start;
    app_buzzer_stop_t   app_buzzer_stop;
    app_buzzer_set_note_t app_buzzer_set_note;
}app_buzzer_util_funcs_t;

#endif // APP_BUZZER_DEFS_H

/**
 * \}
 * \}
 * \}
 */
