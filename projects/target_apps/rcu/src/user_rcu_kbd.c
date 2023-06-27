/*****************************************************************************************
 *
 * \file user_rcu_kbd.c
 *
 * \brief RCU keyboard implementation.
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_APP
 * \{
 * \addtogroup APP_RCU_KBD
 *
 * \{
******************************************************************************************/

/*
 ****************************************************************************************
 * INCLUDE FILES
******************************************************************************************/
#include "rwip_config.h"
#include "user_rcu.h"
#include "user_rcu_kbd.h"
#include "user_modules.h"
#include "port_platform.h"
#include "ke_env.h"
#include "app_leds.h"
#include "port_timer.h"
/*
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

#ifdef HAS_KBD
    bool user_fn_locked __PORT_RETAINED;
    #ifdef HAS_LONG_PRESS_POWERUP_BUTTON
    bool kbd_long_press __PORT_RETAINED;
    #endif
#endif
    
#ifdef HAS_IR
    bool user_ir_mode __PORT_RETAINED;
#endif

#ifdef DEBUG_EMULATE_PACKET_LOSS
    uint8_t mute_radio_level __PORT_RETAINED;
    uint8_t mute_radio_index __PORT_RETAINED;
    const uint8_t radio_mute_pattern[] ={80, 10}; // on, off times in msec
#endif
                
#ifdef MOUSE_GENERATE_TEST_PATTERN
    bool user_start_mouse_test __PORT_RETAINED;
#endif
        
/*
 ****************************************************************************************
 * FUNCTION DEFINITIONS
******************************************************************************************/

#ifdef DEBUG_EMULATE_PACKET_LOSS
    
void user_rcu_kbd_radio_mute_timer_handler(void)
{
    if(mute_radio_level > 0) {
        if((mute_radio_index & 0x01) == 0) {
            port_enable_radio();
            port_timer_set(USER_RADIO_MUTE_TIMER, TASK_APP, radio_mute_pattern[mute_radio_index]);
        }
        else {
            port_disable_radio();
            port_timer_set(USER_RADIO_MUTE_TIMER, TASK_APP, radio_mute_pattern[mute_radio_index]*mute_radio_level);
        }
        
        mute_radio_index++;
        if(mute_radio_index >= sizeof(radio_mute_pattern)) {
            mute_radio_index = 0;
        }
    }
    else {
        port_enable_radio();
    }
}

/*****************************************************************************************
 * \brief Increment emulated packet loss. mute_radio_level is incremented by 1.
          Off times (odd indexes) in radio_mute_pattern are multiplied by 
 *        mute_radio_level.
******************************************************************************************/
static void user_packet_loss_inc(void)
{
    if(mute_radio_level < 16) {
        mute_radio_level++;
        dbg_printf(DBG_APP_LVL, "Radio mute level %d\r\n", mute_radio_level);
    }
    if(mute_radio_level == 1) {
        user_rcu_kbd_radio_mute_timer_handler();
    }
}

/*****************************************************************************************
 * \brief Decrement emulated packet loss. mute_radio_level is decremented by 1.
          Off times (odd indexes) in radio_mute_pattern are multiplied by 
 *        mute_radio_level.
******************************************************************************************/
static void user_packet_loss_dec(void)
{
    if(mute_radio_level > 0) {
        mute_radio_level--;
        dbg_printf(DBG_APP_LVL, "Radio mute level %d\r\n", mute_radio_level);
    }
}
#endif

#if defined(HAS_KBD) || defined(HAS_TOUCH)

#ifdef HAS_KBD

#ifdef HAS_LONG_PRESS_POWERUP_BUTTON

void user_long_keypress_timer_handler(void) 
{
    #ifdef HAS_CONNECTION_FSM
    app_con_fsm_request_disconnect(MULTI_BOND_REJECT_LAST, APP_CON_FSM_DC_REASON_START_PAIR);
    #endif    
    #ifdef HAS_PWR_MGR
    user_pwr_mgr_disable_inactivity();
    #endif
    kbd_long_press = true;
}
#endif

void user_kbd_notification_cb(enum kbd_notification notification)
{
    switch(notification) {
        
    case KBD_KEY_ACTION:
        user_action_triggered();
        break;
    case KBD_PASSCODE_ENTERED:
        {
    #ifdef HAS_CONNECTION_FSM
            uint32_t passcode = app_kbd_get_passcode();
            dbg_printf(DBG_APP_LVL, "MITM code:%d\r\n", passcode); 
            app_con_fsm_mitm_passcode_report(passcode);
    #endif
        }
        break;  
    case KBD_FN_LOCK_PRESSED:
        user_fn_locked = true;
    #ifdef HAS_AUDIO
        user_audio_set_config_data();
    #endif
        if (user_is_ble_connected())  {
    #ifdef HAS_MOTION      
            user_motion_start();
    #endif
            USER_LEDS_RAMP(led_motion_kbd_page_param);
        }
#if defined(HAS_LED_FN_LOCK) && defined(HAS_LED_INDICATORS)
        app_leds_on(LED_FN_LOCK,0);
    #endif
        break;
    case KBD_FN_LOCK_RELEASED:
        user_fn_locked = false;
    #ifdef HAS_AUDIO
            user_audio_set_config_data();
    #endif
    #ifdef HAS_MOTION      
        user_motion_stop();
    #endif
        USER_LEDS_OFF(led_motion_kbd_page_param);
    #if defined(HAS_LED_FN_LOCK) && defined(HAS_LED_INDICATORS)
        app_leds_off(LED_FN_LOCK);
    #endif
        break;
    default: {
        enum custom_keys key = (enum custom_keys)(KBD_NOTIFY_CODE_TO_KEY(notification) & ~KBD_NOTIFY_CODE_KEY_PRESSED_MASK);
        bool pressed = (KBD_NOTIFY_CODE_TO_KEY(notification) & KBD_NOTIFY_CODE_KEY_PRESSED_MASK) != 0;
        switch (key) {
#ifdef HAS_AUDIO       
        case CUSTOM_KEY_AUDIO:
#ifdef CFG_AUDIO_ADAPTIVE_RATE
            pressed ? user_audio_start(ADPCM_MODE_MIN, true) : user_audio_stop(false);
#else        
            pressed ? user_audio_start(ADPCM_DEFAULT_MODE, true) : user_audio_stop(false);
#endif        
            user_action_triggered();
            break;
#endif
#ifdef HAS_MOTION      
        case CUSTOM_KEY_MOTION:
            pressed ? user_motion_start() : user_motion_stop();
            user_action_triggered();
            break;
        case CUSTOM_KEY_MOTION_CLICK: 
        {
            extern bool user_motion_left_click_pressed;
            user_motion_left_click_pressed = pressed;
            user_action_triggered();
            break;
        }
#endif
#ifdef HAS_IR
        case CUSTOM_KEY_IR:
            if(pressed == true) {
                user_ir_mode=!user_ir_mode;
                if(user_ir_mode == false) {
    #ifdef HAS_LED_INDICATORS
                    app_leds_block_ramp(false);
                    LEDS_RAMP(led_ir_mode_off_param);
    #endif
                    dbg_puts(DBG_APP_LVL, "IR mode is off\r\n");
                    app_ir_stop();
                }
                else {
    #ifdef HAS_LED_INDICATORS                    
                    app_leds_all_off();
                    app_leds_block_ramp(true);
                    LEDS_RAMP(led_ir_mode_on_param);
    #endif
                    dbg_puts(DBG_APP_LVL, "IR mode is on\r\n");
                }
            }
            break;
#endif
#ifdef HAS_POWERUP_BUTTON       
        case CUSTOM_KEY_POWER:
            if(pressed) {
                if (app_con_fsm_get_state() == IDLE_ST) {
    #ifdef HAS_LONG_PRESS_POWERUP_BUTTON
                    kbd_long_press=true;  
                    user_action_triggered();
    #endif
                } else {
    #ifdef HAS_LONG_PRESS_POWERUP_BUTTON
                    port_timer_set(APP_KEYPRESS_TIMER, 0, LONG_KEYPRESS_QUOTA_IN_MS);
                    kbd_long_press=false;
    #endif
                }                        
            } else {
    #ifdef HAS_LONG_PRESS_POWERUP_BUTTON
                if (kbd_long_press == false) {
                    port_timer_clear(APP_KEYPRESS_TIMER, 0);
    #endif
                    if(force_power_off == 2) { // just woke up
                        force_power_off = 0;
                    } else {
                        app_kbd_add_full_release_reports();
                        force_power_off = 1;
                        app_con_fsm_state_update(POWEROFF_EVT);     
    #ifdef HAS_PWR_MGR
                        user_pwr_mgr_disable_inactivity();
    #endif                        
                    }
    #ifdef HAS_LONG_PRESS_POWERUP_BUTTON
                }
    #endif
            }
            break;
#endif  
        default:
            if(pressed) {
                switch (key) {
#ifdef HAS_CONNECTION_FSM
                case CUSTOM_KEY_PAIR:
    #ifdef FORCE_CONNECT_TO_HOST_ON
                    app_con_fsm_switch_to_peer(CON_FSM_PEER_ANY);
    #else                
                    app_con_fsm_request_disconnect(MULTI_BOND_REJECT_LAST, APP_CON_FSM_DC_REASON_START_PAIR);
    #endif                
                    break;
                case CUSTOM_KEY_CLEAR_BONDING_DATA:
                    app_con_fsm_reset_bonding_data();
                    break;
    #ifdef FORCE_CONNECT_TO_HOST_ON                
                case CUSTOM_KEY_PAIR_WITH_HOST1:
                    app_con_fsm_add_host_to_entry(0);
                    break;
                case CUSTOM_KEY_PAIR_WITH_HOST2:
                    app_con_fsm_add_host_to_entry(1);
                    break;
                case CUSTOM_KEY_PAIR_WITH_HOST3:
                    app_con_fsm_add_host_to_entry(2);
                    break;
                case CUSTOM_KEY_SWITCH_TO_HOST1:
                    app_con_fsm_switch_to_peer(0);
                    break;
                case CUSTOM_KEY_SWITCH_TO_HOST2:
                    app_con_fsm_switch_to_peer(1);
                    break;
                case CUSTOM_KEY_SWITCH_TO_HOST3:
                    app_con_fsm_switch_to_peer(2);
                    break;
    #endif

    #ifdef HAS_SPECIAL_ADVERTISING
                case CUSTOM_KEY_ONOFF_HOST:
                    if(user_is_ble_connected()) {
                        dbg_puts(DBG_APP_LVL, "\r\n User requested POWER-OFF Peer\r\n");
                        app_hid_report_create_extended_report(POWER_KEY_BYTE, POWER_KEY_MASK, POWER_KEY_MASK);
                        app_hid_report_create_extended_report(POWER_KEY_BYTE, POWER_KEY_MASK, 0);
                    }
                    else {
                        dbg_puts(DBG_APP_LVL, "\r\n User requested WAKE-UP Peer\r\n");
                        user_wake_up_peer();
                    }
                    break;
	#endif
#endif        
        
#ifdef AUDIO_TEST_MODE                    
                case CUSTOM_KEY_AUDIO_TEST_MODE:
                    if(user_is_ble_connected()) {
                        user_rcu_audio_start_audio_test(true);
                    }
                    break;
#endif

#ifdef MOUSE_GENERATE_TEST_PATTERN    
                case CUSTOM_KEY_MOUSE_TEST:
                    user_start_mouse_test = !user_start_mouse_test;
                    break;
#endif     
               
#ifdef DEBUG_EMULATE_PACKET_LOSS
                case CUSTOM_KEY_INC_PACKET_LOSS:
                    user_packet_loss_inc();
                    break;
                case CUSTOM_KEY_DEC_PACKET_LOSS:
                    user_packet_loss_dec();
                    break;
#endif                    
                default:
                    break;
                }
            }
            break;
            }
        } 
    }
}

void user_rcu_kbd_turn_on_leds(void)
{
    #ifdef HAS_LED_INDICATORS    
        #ifdef HAS_LED_FN_LOCK	
        if(user_fn_locked) {
                app_leds_on(LED_FN_LOCK,0);
            }
        #endif
    #endif
    
}

void user_rcu_kbd_turn_off_leds(void)
{
#ifdef HAS_LED_INDICATORS
	#ifdef HAS_LED_CAPS_LOCK
		app_leds_off(LED_CAPS_LOCK);
	#endif	
    
	#ifdef HAS_LED_NUM_LOCK
		app_leds_off(LED_NUM_LOCK);
	#endif	
	
	#ifdef HAS_LED_SCROLL_LOCK
		app_leds_off(LED_SCROLL_LOCK);
	#endif	
    
	#ifdef HAS_LED_FN_LOCK
		app_leds_off(LED_FN_LOCK);
	#endif	
#endif    
}

#if (BLE_HID_DEVICE)
void user_keyboard_led_report_callback(uint8_t conidx, struct hogpd_report_info *report_info)
{
#define NUM_LOCK_LED_BIT     0x01
#define CAPS_LOCK_LED_BIT    0x02
#define SCROLL_LOCK_LED_BIT  0x04

#ifdef HAS_LED_INDICATORS    
    #ifdef HAS_LED_NUM_LOCK				
    if(report_info->value & NUM_LOCK_LED_BIT) { 
        app_leds_on(LED_NUM_LOCK, 0);
    } else {
        app_leds_off(LED_NUM_LOCK);
    }
    #endif				
    
    #ifdef HAS_LED_CAPS_LOCK				
    if(report_info->value & CAPS_LOCK_LED_BIT) { 
        app_leds_on(LED_CAPS_LOCK, 0);
    } else {
        app_leds_off(LED_CAPS_LOCK);
    }            
    #endif				
    
    #ifdef HAS_LED_SCROLL_LOCK				
    if(report_info->value & SCROLL_LOCK_LED_BIT) { 
        app_leds_on(LED_SCROLL_LOCK, 0);
    } else {
        app_leds_off(LED_SCROLL_LOCK);
    }            
    #endif	
#endif    
}
#endif

#ifdef HAS_IR
void user_kbd_key_detected_cb(uint16_t row, uint16_t column, bool pressed)
{
    if(user_ir_mode == false) {
        return;
    }
    
    #ifdef HAS_AUDIO
    if (app_audio_is_active()) {
            return;
    }
    #endif    

    #ifdef HAS_MOTION
    if (app_motion_is_active()) {
            return;
    }
    #endif    
    if (pressed == true && app_ir_is_active() == false) {
        app_ir_init();
        if (kbd_ir_keymap[row][column] != K_CODE) {
            app_ir_send_command(kbd_ir_keymap[row][column]);
        }
    }
    else {
        app_ir_stop();
    }
}
#endif

#endif

#endif

/**
 * \}
 * \}
 * \}
 */
