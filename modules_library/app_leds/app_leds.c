/*****************************************************************************************
 *
 * \file app_leds.c
 *
 * \brief LED  module source file
 * 
******************************************************************************************/
 
/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup LEDS
 * \{
 * \addtogroup APP_LEDS
 * \{
 ****************************************************************************************	 	 
 */
 
/*
 * INCLUDE FILES
******************************************************************************************/
 
#ifdef HAS_LED_INDICATORS

#include "app_leds.h"
#include "port_leds.h"
#include "port_platform.h"
    
#include "port_timer.h"

enum led_state {
    LED_INIT = 0,
    LED_OFF,
    LED_ON,
    LED_DELAY,
    LED_IDLE,
    #ifdef LED_USE_RAMP_FEATURE                            
        LED_RAMP_UP,
        LED_RAMP_DOWN,
    #endif
};

typedef struct {
	uint16_t on_time;
	uint16_t off_time;
    #ifdef LED_USE_RAMP_FEATURE                            
        uint16_t ramp_on_step;
        uint16_t ramp_off_step;
        int16_t ramp_value;
    #endif
    uint16_t count;
    uint16_t delay;
	enum led_state state;
    enum led_mode mode;
    bool high_priority;
} led_pattern_t;

led_pattern_t led_pattern[NUM_OF_LEDS]     __PORT_RETAINED;
led_id_t pwm_led                        __PORT_RETAINED;
const led_params_t *led_pending_param  __PORT_RETAINED;
uint8_t led_pending_size                   __PORT_RETAINED;
bool led_boost_mode_enabled                __PORT_RETAINED;
bool led_block_ramp                        __PORT_RETAINED;

static void app_leds_update_state(led_id_t led_id);


#ifdef LED_USE_RAMP_FEATURE

/**
 ******************************************************************************
 * \brief LED ramp event handling function
 ******************************************************************************
 */
static void app_leds_ramp_callback_function(void)
{ 
    led_pattern_t *pattern = &led_pattern[pwm_led];
    
    switch(pattern->state) {
    case LED_RAMP_UP:
        pattern->ramp_value += pattern->ramp_on_step;
        break;
    case LED_RAMP_DOWN:
        pattern->ramp_value -= pattern->ramp_off_step;
        break;
    default:
        return;
    }
    
    if (port_leds_set_next_ramp_lvl(pattern->ramp_value) == false) {
        switch(pattern->state) {
        case LED_RAMP_UP:
            pattern->ramp_value -= pattern->ramp_on_step;
            break;
        case LED_RAMP_DOWN:
            pattern->ramp_value += pattern->ramp_off_step;
            break;
        default:
            return;
        }
        app_leds_update_state(pwm_led);
    }
}


/**
 ******************************************************************************
 * \brief Makes the proper initializations and starts LED ramping
 *
 * \param[in] led_id The id of the LED that we want to ramp
 ******************************************************************************
 */
static void app_leds_ramp_start(led_id_t led_id)
{
    pwm_led = led_id;
    
    port_leds_init_gpios_pwm(led_id);
    
    port_leds_start_pwm(&app_leds_ramp_callback_function, led_pattern[led_id].ramp_value);
}


/**
 ******************************************************************************
 * \brief Makes the proper de-initializations and stops LED ramping
 *
 * \param[in] led_id The id of the LED that we want to stop ramping
 ******************************************************************************
 */
static void app_leds_ramp_stop(led_id_t led_id)
{
    if(pwm_led == led_id) {
        port_leds_stop_pwm();
        pwm_led = NUM_OF_LEDS;
    }
}

#endif // LED_USE_RAMP_FEATURE


/*****************************************************************************************
 * \brief Block sleep when MCU is in BOOST mode for the period a LED is on.
******************************************************************************************/
static void app_leds_block_sleep(void)
{
    if (led_boost_mode_enabled) {
        port_force_active_mode(); // in BOOST mode, prevent sleep only if enabled
    }
}


/*****************************************************************************************
 * \brief Restore sleep when MCU is in BOOST mode after a LED is turn off.
******************************************************************************************/
static void app_leds_restore_sleep(void)
{
    if (led_boost_mode_enabled) {
        port_restore_sleep_mode(); // restore sleep
    }
}


/*****************************************************************************************
 * \brief Turns off a LED
 *
 * \param[in] led_id The id of the LED to turn off
******************************************************************************************/
static void app_leds_turn_off_led(led_id_t led_id)
{
    if (led_pattern[led_id].state==LED_ON) {
        app_leds_restore_sleep();
        port_timer_clear(led_timer[led_id], TASK_APP);
    }
    led_pattern[led_id].state = LED_OFF;        
    app_leds_update_state(led_id);    
}


led_params_t param_on = {
        .mode           = LED_NO_BLINK,
        .off_time       = 0,
#ifdef LED_USE_RAMP_FEATURE        
        .ramp_on_time   = 0,
        .ramp_off_time  = 0,
#endif    
        .count          = 1,
};
    

led_result_t app_leds_on(led_id_t led_id, uint16_t on_time)
{

    param_on.id      = led_id;
    param_on.on_time = on_time;
    
    return LEDS_RAMP(&param_on);
}


void app_leds_init()
{
    uint8_t i;
    
    led_boost_mode_enabled = port_leds_boost_enabled();
    
    for (i=0; i<NUM_OF_LEDS; i++) {
        if (!port_leds_dbg_confict((led_id_t)i)) {
            port_leds_init_gpios_default(i);
        }
        led_pattern[i].state  = LED_IDLE;
    }
    led_pending_param = NULL;
    pwm_led = NUM_OF_LEDS;
}


void app_leds_off(led_id_t led_id)
{
    // clear pending LED
    led_pending_param = NULL;    
    app_leds_turn_off_led(led_id);
    led_pattern[led_id].high_priority = false;
}


void app_leds_off_param(const led_params_t *param, uint8_t num_of_leds)
{
    if(param == led_pending_param) {
        // If LED is pending clear it
        led_pending_param = NULL;    
    }
    else {
        for(int i=0; i<num_of_leds; i++) {
            if(led_pattern[param[i].id].high_priority==false || param[i].high_priority==true) {
                app_leds_off(param[i].id);
            }
        }
    }
}


void app_leds_all_off(void)
{
    int i;
    
    for (i=0; i< NUM_OF_LEDS; i++) {
        app_leds_off((led_id_t)i);
    }
    
}


led_result_t app_leds_ramp(const led_params_t *param, uint8_t num_of_leds)
{
    int i;
    
#ifdef LED_USE_RAMP_FEATURE  
    bool ramping_led = false;
#endif    
    
    for(i=0; i<num_of_leds; i++) {
        if(led_pattern[param[i].id].state!=LED_IDLE && led_pattern[param[i].id].high_priority == true) {
            led_pending_param = param;
            led_pending_size = num_of_leds;
            return LED_RESULT_PENDING;
        }
#ifdef LED_USE_RAMP_FEATURE            
        if(param[i].ramp_on_time != 0 || param[i].ramp_off_time != 0) {
            ramping_led = true;
        }
#endif        
    }

#ifdef LED_USE_RAMP_FEATURE        
    if(ramping_led == true) {
        if(led_block_ramp == true) {
            return LED_RESULT_FAIL;
        }        
        for(i=0; i<NUM_OF_LEDS; i++) {
            if(led_pattern[i].state!=LED_IDLE && led_pattern[i].high_priority==true && (led_pattern[i].ramp_on_step!=0 || led_pattern[i].ramp_off_step!=0)) {
               return LED_RESULT_FAIL;
            }
        }
    }
#endif
    
    for(i=0; i<num_of_leds; i++) {
#ifdef LED_USE_RAMP_FEATURE            
        uint16_t step_on, step_off;
#endif        
        led_id_t led_id=param[i].id;

        if(param[i].mode == LED_TURN_OFF) {
            app_leds_off(param[i].id);
            continue;
        }
        
        led_pattern[led_id].high_priority = param[i].high_priority;
        app_leds_turn_off_led(led_id);
        
#ifdef LED_USE_DOUBLE_BLINK_FEATURE
        led_pattern[led_id].count  = (param[i].mode==LED_DOUBLE_BLINK?2*param[i].count: param[i].count);
#else
        led_pattern[led_id].count  = param[i].count;
#endif        

#ifdef LED_USE_RAMP_FEATURE            
        if (param[i].ramp_on_time != 0) {
            step_on = LED_RAMP_STEP_MULTIPLIER/param[i].ramp_on_time;
        } else {
            step_on = 0;
        }

        if (param[i].ramp_off_time != 0) {
            step_off = LED_RAMP_STEP_MULTIPLIER/param[i].ramp_off_time;
        } else {
            step_off = 0;
        }
        if(app_led_pins[led_id].high == LED_ACTIVE_LOW) {
            step_on *= -1;
            step_off *= -1;
        }

        led_pattern[led_id].ramp_on_step  = step_on;
        led_pattern[led_id].ramp_off_step = step_off;        
        led_pattern[led_id].on_time = param[i].on_time - LED_RAMP_STEP_MULTIPLIER/step_on - LED_RAMP_STEP_MULTIPLIER/step_off;
#else
        led_pattern[led_id].on_time = param[i].on_time;        
#endif        
        led_pattern[led_id].mode = param[i].mode;    

        if(param[i].mode == LED_NO_BLINK || led_pattern[led_id].mode == LED_NO_BLINK || led_pattern[led_id].state == LED_IDLE) {
        // If LED is already blinking do not disturb its blinking pattern. Just update the parameters
            if (param[i].delay != 0) {
                led_pattern[led_id].off_time = param[i].delay;
                led_pattern[led_id].state = LED_DELAY;
            } else {
                led_pattern[led_id].state = LED_INIT;
            }
            app_leds_update_state(led_id);        
        }

        led_pattern[led_id].off_time = param[i].off_time;
    }
    led_pending_param = NULL;  // invalidate pending param
    
    return LED_RESULT_OK;
}


led_params_t param_blink;


led_result_t app_leds_blink(led_id_t led_id, enum led_mode mode, uint16_t on_time, uint16_t off_time, uint8_t count)
{
    
#ifdef LED_USE_RAMP_FEATURE
    param_blink.ramp_on_time   = 0;
    param_blink.ramp_off_time  = 0;
#endif    
    param_blink.id        = led_id;
    param_blink.mode      = mode;
    param_blink.on_time   = on_time;
    param_blink.off_time  = off_time;
    param_blink.count     = count;
    
    return LEDS_RAMP(&param_blink);
}


/*****************************************************************************************
 * \brief Sets LED on or off
 *
 * \param[in]  led_id  The ID of the LED    
 * \param[in]  led_on  true to turn the LEDon, false to turn it off
******************************************************************************************/
static void app_leds_set_led_state(led_id_t led_id, bool led_on)
{    
    if(led_on == true) {
        if(led_pattern[led_id].on_time>0 || led_pattern[led_id].mode != LED_NO_BLINK) {
            port_timer_set(led_timer[led_id], TASK_APP, led_pattern[led_id].on_time);
        }
        led_pattern[led_id].state=LED_ON;
        app_leds_block_sleep();
    } else {
        if(led_pattern[led_id].off_time!=0 && led_pattern[led_id].count > 1) {
#ifdef LED_USE_DOUBLE_BLINK_FEATURE
            if(led_pattern[led_id].mode == LED_DOUBLE_BLINK && (led_pattern[led_id].count%2) == 0) {
                port_timer_set(led_timer[led_id], TASK_APP, LED_DOUBLE_BLINK_DELAY);
            } else 
#endif            
            {
                port_timer_set(led_timer[led_id], TASK_APP, led_pattern[led_id].off_time);
            }
            led_pattern[led_id].count--;
            led_pattern[led_id].state = LED_INIT;
        } else {
            led_pattern[led_id].state = LED_IDLE;
            if(led_pads[led_id].type != LED_GPIO && led_pads[led_id].callback != NULL) {
                (led_pads[led_id].callback)(LED_COMMAND_DISABLE_CONTROL);
            }
        }
    }
    if (!port_leds_dbg_confict(led_id)) {
        // GPIOs are not being used by the debugger                        
        if(led_pads[led_id].type == LED_GPIO) {
            port_leds_init_gpios_default(led_id);
            bool set_high = (app_led_pins[led_id].high==LED_ACTIVE_HIGH && led_on==true) || (app_led_pins[led_id].high==LED_ACTIVE_LOW && led_on==false);
            port_gpio_set_pin_state(app_led_pins[led_id].port, app_led_pins[led_id].pin, set_high);       
        }
        else {
            if(led_pads[led_id].callback != NULL) {
                (led_pads[led_id].callback)(led_on?LED_COMMAND_TURN_ON:LED_COMMAND_TURN_OFF);
            }
        }
    }
}


/*****************************************************************************************
 * \brief Handles the LED state transitions when the corresponding LED timer expires
 *
 * \param[in] led_id  The led id of the LED    
******************************************************************************************/
static void app_leds_update_state(led_id_t led_id)
{
    switch(led_pattern[led_id].state) {
    case LED_INIT:
        #ifdef LED_USE_RAMP_FEATURE            
        if (led_pattern[led_id].ramp_on_step != 0 ) {
            port_timer_clear(led_timer[led_id], TASK_APP);
            led_pattern[led_id].ramp_value = (app_led_pins[led_id].high == LED_ACTIVE_HIGH? LED_RAMP_PWM_COUNTER_MIN : LED_RAMP_PWM_COUNTER_MAX+led_pattern[led_id].ramp_off_step-1);
            app_leds_ramp_start(led_id);  
            led_pattern[led_id].state=LED_RAMP_UP;
            break;
        }
        #endif // LED_USE_RAMP_FEATURE          
        
        if(led_pads[led_id].type != LED_GPIO && led_pads[led_id].callback != NULL) {
            (led_pads[led_id].callback)(LED_COMMAND_ENABLE_CONTROL);
        }
        
        app_leds_set_led_state(led_id, true);
        break;
    #ifdef LED_USE_RAMP_FEATURE                
    case LED_RAMP_UP:
        app_leds_ramp_stop(led_id);
        app_leds_set_led_state(led_id, true);
        break;
    case LED_RAMP_DOWN:
        app_leds_ramp_stop(led_id);
        app_leds_set_led_state(led_id, false);
        if (led_pattern[led_id].state == LED_IDLE) {
            led_pattern[led_id].high_priority = false;
            if(led_pending_param != NULL) {
                app_leds_ramp(led_pending_param, led_pending_size);
            }
       }
        break;
    #endif
    case LED_ON:
        app_leds_restore_sleep(); // forced when LED went ON
        #ifdef LED_USE_RAMP_FEATURE                            
        if( led_pattern[led_id].ramp_off_step != 0 ) {
            app_leds_ramp_start(led_id);  
            led_pattern[led_id].state=LED_RAMP_DOWN;
            break;
        }
        #endif
        app_leds_set_led_state(led_id, false);
        if (led_pattern[led_id].state == LED_IDLE) {
            led_pattern[led_id].high_priority = false;
            if(led_pending_param != NULL) {
                app_leds_ramp(led_pending_param, led_pending_size);
            }
        }
        break;
    case LED_OFF:
        #ifdef LED_USE_RAMP_FEATURE      
            app_leds_ramp_stop(led_id);
        #endif
        led_pattern[led_id].off_time = 0;
        app_leds_set_led_state(led_id, false);
        break;
    case LED_DELAY:
        port_timer_set(led_timer[led_id], TASK_APP, led_pattern[led_id].off_time);
        led_pattern[led_id].state=LED_INIT; 
        break;
    case LED_IDLE:
        break;
    }
}


void app_leds_timer_1_handler(void)
{
    for(int i=0; i< NUM_OF_LEDS; i++) {
        if(led_timer[i] == APP_LED_1_TIMER) {
            app_leds_update_state((led_id_t)i);
        }
    }
}


void app_leds_timer_2_handler(void)
{
    for(int i=0; i< NUM_OF_LEDS; i++) {
        if(led_timer[i] == APP_LED_2_TIMER) {
            app_leds_update_state((led_id_t)i);
        }
    }
}


void app_leds_block_ramp(bool block)
{
    led_block_ramp = block;
}


void app_leds_declare_gpios(void)
{      
    port_leds_declare_led_gpios();
}


void app_leds_init_gpios(void)
{                                       
    uint8_t i;
    for (i=0; i < NUM_OF_LEDS; i++) {
        if (led_pads[i].type == LED_GPIO) {
            if ( led_pattern[i].state == LED_IDLE || led_pattern[i].state == LED_INIT ) {
                port_leds_init_gpios_default(i);
            #ifdef LED_USE_RAMP_FEATURE    
            } else if(led_pattern[i].state == LED_RAMP_UP || led_pattern[i].state == LED_RAMP_DOWN) {
                port_leds_init_gpios_pwm(i);
            #endif
            } else {
                port_leds_init_gpios_output(i);
            }
        }
    }
}

#endif // HAS_LED_INDICATORS


/**
 * \}
 * \}
 * \}
 */
