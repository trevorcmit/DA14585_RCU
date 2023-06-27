/*****************************************************************************************
 *
 * \file app_buzzer.c
 *
 * \brief Buzzer application module source file
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BUZZER
 * \{
 * \addtogroup APP_BUZZER
 *
 * \brief Buzzer Application Source
 * \{
 ****************************************************************************************	 
 */
 
#ifdef HAS_SOUND_INDICATION

#include "app_buzzer.h"
#include <app_buzzer_config.h>
#include "port_buzzer.h"


static uint8_t buzzerMelodyNotesRemaining;
static uint8_t buzzerNoteIndex;
static uint16_t * buzzerCurrentMelody;


/***************************************************************************************************
 * \brief Buzzer note event handler. Handles the every note completion event while playing a melody
 **************************************************************************************************
 */
static void app_buzzer_note_played_cb(void)
{
    // If the melody has not finished
    if(buzzerMelodyNotesRemaining) {
            
			// Decrease the remaining notes counter
            --buzzerMelodyNotesRemaining;
            
            // Change the current playing note to the next one
			if(app_buzzer_funcs.app_buzzer_set_note) { 
				app_buzzer_funcs.app_buzzer_set_note(buzzerCurrentMelody[buzzerNoteIndex], BUZZER_NOTE_DURATION_TICKS);
			}
			else {
				ASSERT_ERROR(0);
			}
            
            // Increment the note index in order to play the next note
            ++buzzerNoteIndex;
    }
    // Else if the melody has ended
    else {
            // Stop the buzzer
            if(app_buzzer_funcs.app_buzzer_stop) { 
				app_buzzer_funcs.app_buzzer_stop();
			}
			else {
				ASSERT_ERROR(0);
			}
    }
}


/*****************************************************************************************
 * \brief Initializes and starts the Buzzer
******************************************************************************************/
static void app_buzzer_init_and_start(void)
{
    buzzerNoteIndex = 1;
    
	if(app_buzzer_funcs.app_buzzer_start) { 
        app_buzzer_funcs.app_buzzer_start(&app_buzzer_note_played_cb);
    }
    else {
        ASSERT_ERROR(0);
    }
}


void app_buzzer_play_melody(buzzer_melody_t melody)
{
    ASSERT_ERROR( melody < BUZZER_TOTAL_MELODIES );
    
    buzzerCurrentMelody = (uint16_t*)buzzerMelodies[melody];
    
    buzzerMelodyNotesRemaining = *buzzerCurrentMelody;

    app_buzzer_init_and_start();

}
#endif // HAS_SOUND_INDICATION

/**
 * \}
 * \}
 * \}
 */
