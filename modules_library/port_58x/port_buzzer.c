/*****************************************************************************************
 * \file port_buzzer.c
 * \brief Buzzer module platform adaptation source file
*****************************************************************************************/
 
/****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BUZZER
 * \{
 * \addtogroup PORT_BUZZER
 * \{
*****************************************************************************************/

#ifdef HAS_SOUND_INDICATION
#include "port_buzzer.h"
#include <app_buzzer_config.h>
#include "timer0.h"
#include "gpio.h"
#include "port_platform.h"

// Buzzer PWM settings
#define BUZZER_TIMER_ON        1000
#define BUZZER_PWM_HIGH        500
#define BUZZER_PWM_LOW         200


uint8_t port_current_note_repeats __PORT_RETAINED;
buzzer_note_handler_t port_note_ended_handler __PORT_RETAINED;


/*****************************************************************************************
 * \brief Handles PWM events related to the buzzer
*****************************************************************************************/ 
static void port_buzzer_note_cb(void)
{
    // Time to change to the next note (current note duration expired)
    if(port_current_note_repeats == 0)
    { 
        if (port_note_ended_handler)
        {
            port_note_ended_handler();
        }
    }
    else
    {
        --port_current_note_repeats;
    }    
}


void port_buzzer_start(buzzer_note_handler_t hndl)
{    
    port_note_ended_handler = hndl;
    
    // In order to have optimal buffer performance, postpone sleep
	port_force_active_mode();
    
    // Enables TIMER0,TIMER2 clock
    set_tmr_enable(CLK_PER_REG_TMR_ENABLED);

    // Sets TIMER0,TIMER2 clock division factor to 8, so TIM0 Fclk is F = 16MHz/8 = 2Mhz
    set_tmr_div(CLK_PER_REG_TMR_DIV_8);

    // Initialize PWM with the desired settings
    timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);

    // set pwm Timer0 'On', Timer0 'high' and Timer0 'low' reload values
    timer0_set(BUZZER_TIMER_ON, BUZZER_PWM_HIGH, BUZZER_PWM_LOW);

    // register callback function for SWTIM_IRQn irq
    timer0_register_callback(port_buzzer_note_cb);

    // enable SWTIM_IRQn irq
    timer0_enable_irq();
    
     // Switch the buzzer pin to pwm0 output in order to drive the buzzer and produce melodies
    PORT_CONFIGURE_GPIO(app_buzzer_pins[BUZZER_PIN], OUTPUT, PID_PWM0, true);
    
    // start pwm0
    timer0_start();    
}


void port_buzzer_change_note(uint32_t note, uint8_t noteRepeat)
{
    port_current_note_repeats = noteRepeat;
    timer0_set_pwm_on_counter(0xFFFF);
    timer0_set_pwm_high_counter(note/3 * 2);
    timer0_set_pwm_low_counter(note/3);
}


void port_buzzer_stop()
{
    // We are stopping the buzzer, so we can now resume sleep
    port_restore_sleep_mode();
    
    // Set the buzzer pin to GPIO output low ( implicitly deactivate any buzzer activity )
    PORT_CONFIGURE_GPIO_DEFAULT(app_buzzer_pins[BUZZER_PIN]);

    // Stop the buzzer PWM timer
    timer0_stop();
    
    // Disable TIMER0, TIMER2 peripheral clocks
    set_tmr_enable(CLK_PER_REG_TMR_DISABLED);       
}


void port_buzzer_pins_declare(void)
{
    PORT_RESERVE_GPIO(app_buzzer_pins[BUZZER_PIN]);
}


void port_buzzer_pins_init(void)
{  
    PORT_CONFIGURE_GPIO_DEFAULT(app_buzzer_pins[BUZZER_PIN]);
}


#endif

/**
 * \}
 * \}
 * \}
 */
