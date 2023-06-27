/*****************************************************************************************
 *
 * \file port_leds.c
 *
 * \brief LED module platform adaptation source file
 * 
******************************************************************************************/
 
 /*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup LEDS
 * \{
 * \addtogroup PORT_LEDS
 * \{
 ****************************************************************************************	 
 */

#ifdef HAS_LED_INDICATORS

#include "port_leds.h"
#include "port_platform.h"
#include <app_leds_config.h>
#if (RWBLE_SW_VERSION_MAJOR >= 8) 
    #include "timer0.h"
    #include "timer2.h"
#else
    #include "pwm.h"
#endif   
#include "app_leds.h"

static led_ramp_handler_t port_led_ramp_handler __PORT_RETAINED;

static const led_timer_handler_t port_leds_timer1_cb = app_leds_timer_1_handler;
static const led_timer_handler_t port_leds_timer2_cb = app_leds_timer_2_handler;


void port_leds_declare_led_gpios(void)
{      
    uint8_t i;
    
    for(i=0; i < NUM_OF_LEDS; i++) {
        RESERVE_GPIO( i, app_led_pins[i].port, app_led_pins[i].pin, PID_GPIO);  
    }
}


void port_leds_init_gpios_default(uint8_t idx)
{
    PORT_CONFIGURE_GPIO_DEFAULT(app_led_pins[idx]);
}


void port_leds_init_gpios_output(uint8_t idx)
{
     PORT_CONFIGURE_GPIO(app_led_pins[idx], OUTPUT, PID_GPIO, app_led_pins[idx].high==LED_ACTIVE_HIGH);
}


#ifdef LED_USE_RAMP_FEATURE

/*****************************************************************************************
 * \brief Handles PWM events related to the ramping LEDs
******************************************************************************************/
static void port_leds_pwm_handler(void)
{
    if(port_led_ramp_handler) {
        port_led_ramp_handler();
    }
}


void port_leds_init_gpios_pwm(uint8_t idx)
{
     PORT_CONFIGURE_GPIO(app_led_pins[idx], OUTPUT, PID_PWM2, app_led_pins[idx].high==LED_ACTIVE_HIGH);
}


void port_leds_start_pwm(led_ramp_handler_t handler, uint16_t pwmTime)
{
    port_led_ramp_handler = handler;
    
    // Enable TIMER0,TIMER2 clock
    set_tmr_enable(CLK_PER_REG_TMR_ENABLED);

    // Set TIMER0,TIMER2 clock division factor to 8 -> Timer clock is 2MHz
    set_tmr_div(LED_TIMER_CLOCK_DIVIDER);

    // initilalize TIMER0 PWM with the desired settings
    timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);
    
    // register callback function for SWTIM_IRQn irq (TIMER0)
    timer0_register_callback(port_leds_pwm_handler);

    // initilalize TIMER2 PWM with the desired settings (sw paused)
    timer2_init(HW_CAN_NOT_PAUSE_PWM_2_3_4, PWM_2_3_4_SW_PAUSE_DISABLED, LED_RAMP_PWM_COUNTER_MAX);

    // set Timer0 On, Timer0 'high' and Timer0 'low' reload values
    timer0_set(LED_RAMP_STEP_PERIOD_DIVIDER, 0, 0);    
    
    // enable SWTIM_IRQn irq
    timer0_enable_irq();
    
    // Force active mode in order to perform the pwm successfully
    port_force_active_mode();
    
    // set pwm2 duty cycle
    timer2_set_pwm2_duty_cycle(pwmTime);

    // release sw pause to let pwm2,pwm3,pwm4 run
    timer2_set_sw_pause(PWM_2_3_4_SW_PAUSE_DISABLED); 
    
    // start pwm0
    timer0_start();
}


void port_leds_stop_pwm()
{
    // Stop the timers
    timer0_stop(); 
    timer2_stop();  
    
    // Disable TIMER0, TIMER2 peripheral clocks
    set_tmr_enable(CLK_PER_REG_TMR_DISABLED); 
    
    // Let the device sleep
    port_restore_sleep_mode();
}


bool port_leds_set_next_ramp_lvl(int16_t rampVal)
{
    if(rampVal >= LED_RAMP_PWM_COUNTER_MIN && rampVal < LED_RAMP_PWM_COUNTER_MAX) {
        timer2_set_pwm2_duty_cycle(rampVal);
        return true;
    }
    return false;
}

#endif // LED_USE_RAMP_FEATURE


bool port_leds_dbg_confict(uint8_t led_id)
{
    return (((led_pads[led_id].type == LED_GPIO) && 
            (((app_led_pins[led_id].port == GPIO_PORT_1) && (app_led_pins[led_id].pin == GPIO_PIN_4)) || \
             ((app_led_pins[led_id].port == GPIO_PORT_1) && (app_led_pins[led_id].pin == GPIO_PIN_5)))) &&
             (((GetWord16(SYS_STAT_REG) & DBG_IS_UP) == DBG_IS_UP)));
}


bool port_leds_boost_enabled()
{
    return (GetBits16(ANA_STATUS_REG, BOOST_SELECTED) == 0x1);
}


void port_leds_timer1_handler()
{
    if(port_leds_timer1_cb) {
        port_leds_timer1_cb();
    }
}


void port_leds_timer2_handler()
{
    if(port_leds_timer2_cb) {
        port_leds_timer2_cb();
    }
}

#endif // HAS_LED_INDICATORS


/**
 * \}
 * \}
 * \}
 */
