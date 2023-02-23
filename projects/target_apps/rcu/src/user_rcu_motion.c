/**
 ****************************************************************************************
 *
 * \file user_rcu_motion.c
 *
 * \brief RCU motion implementation.
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
 * \addtogroup USER
 * \{
 * \addtogroup USER_APP
 * \{
 * \addtogroup APP_RCU_MOTION
 *
 * \{
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "user_rcu.h"
#include "user_rcu_motion.h"
#include "user_modules.h"

/*
 ****************************************************************************************
 * DEFINES
 ****************************************************************************************
 */

#if defined(HAS_MOTION) || defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)
    #if BLE_HID_DEVICE == 0
        #error "Mouse/trackpad module requires HOGPD profile"
    #endif
#endif

#if defined(HAS_TOUCHPAD_TRACKPAD) && defined(HAS_TOUCHPAD_SLIDER)
    #error "TRACKPAD and SLIDER cannot be used at the same time"
#endif

/*
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#ifdef HAS_MOTION
    bool user_motion_left_click_pressed __PORT_RETAINED;
	bool motion_cpt_event = false;
    uint16_t user_motion_hdl __PORT_RETAINED;
#endif

#ifdef HAS_TOUCHPAD_TRACKPAD
    bool user_touch_tracking   __PORT_RETAINED; 
    bool user_touch_left_click __PORT_RETAINED; 
#endif

#if defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)
    app_touchpad_actions_t user_touch_event __PORT_RETAINED;
#endif

#ifdef HAS_MOUSE
    bool mouse_sensor_poll __PORT_RETAINED;
#endif

#ifdef HAS_GPIO_KEYS
    #include "app_gpio_keys.h"
    uint8_t button_pressed[GPIO_NUM_OF_KEYS];
    
    #ifdef HAS_MOUSE
    bool mouse_button_event_occured = false;
    #endif
#endif
    
#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
    uint16_t user_mouse_hdl __PORT_RETAINED;
	bool mouse_cpt_event = false;
#endif

/*
 ****************************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void user_motion_get_handles(void)
{
    #ifdef HAS_MOTION
    // Get motion handle
        #if BLE_HID_DEVICE
        user_motion_hdl = app_hogpd_report_handle(HID_MOTION_DATA_IDX);
        #else
            #error "Motion module requires HOGPD profile"
        #endif
    #endif

    #if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
    // Get mouse handle
        #if BLE_HID_DEVICE
            user_mouse_hdl = app_hogpd_report_handle(HID_MOUSE_IDX);
        #else
            #error "Mouse/trackpad module requires HOGPD profile"
        #endif
    #endif
}

#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
bool user_mouse_send_report(bool full_release)
{
    uint8_t report[HID_MOUSE_REPORT_SIZE];

    int16_t count_x, count_y;
    
#ifdef HAS_MOUSE    
    burst_data_t data;

    app_mouse_get_data(&data);
    
    count_y = data.deltaY;	
    count_x = data.deltaX;        
#endif      
    
    bool send_report = true;
    if(user_is_ble_connected() == true) {
        do {
            if(full_release == true) {
                break;
            }                
        #ifdef HAS_MOUSE                
            if(count_x != 0 || count_y != 0) {
                break;
            }
        #endif    
        #ifdef HAS_TOUCHPAD_TRACKPAD
            if(user_touch_tracking == true) {
                break;
            }
            if(user_touch_left_click == true) {
                break;
            }
        #endif            
        #ifdef HAS_MOUSE_WHEEL
            if(motion_or_quadrature_decoder_event_occured_after_last_report() == true) {
                break;
            }
        #endif       
        #if defined(HAS_GPIO_KEYS) && defined(HAS_MOUSE)
            if(mouse_button_event_occured == true) {
                break;
            }
        #endif                     
            send_report = false;
        } while(0);
    }
    else {
#ifdef HAS_CONNECTION_FSM 
        switch (app_con_fsm_get_state()) { 
        case CONNECTED_ST:
            // already handled
        break;

        case CONNECTION_IN_PROGRESS_ST:
        case CONNECTED_PAIRING_ST:
            do {
                if(full_release == true) {
                    break;
                }                

            #if defined(HAS_GPIO_KEYS) && defined(HAS_MOUSE)
                if (mouse_button_event_occured && (app_hid_report_get_free_nodes() > HID_REPORT_FIFO_SIZE/2)) {
                    break;
                }
                if (mouse_button_event_occured && (app_mouse_sensor_data_accumulated_over_quota() == true))  {
                    break;
                }                     
            #endif
                send_report = false;
            } while(0);
            break;
            
        case IDLE_ST:
        case DISCONNECTED_INIT_ST:
        default:      
            send_report = false;
            break;
        }
#else
        send_report = false;   
#endif        
    }
    
    if(send_report == false) {
        return false;
    }

    
    #if (RWBLE_SW_VERSION_MAJOR >= 8)      
        struct hogpd_env_tag* hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
    
        if ((hogpd_env->svcs[0].ntf_cfg[0] & (HOGPD_CFG_REPORT_NTF_EN << HID_MOUSE_IDX)) == 0) {
            return false; // notifications disabled
        }
    
    #else
        if(hogpd_env.features[0].report_char_cfg[HID_MOUSE_IDX] & HOGPD_REPORT_NTF_CFG_MASK) == 0) {
            return false; // notifications disabled
        }
    #endif
    
#ifdef HAS_TOUCHPAD_TRACKPAD
    app_touchpad_track_data_t info;
    if(app_touchpad_get_last_track_info(&info)) {
        count_x = info.app_touch_delta.deltaX;
        count_y = info.app_touch_delta.deltaY;
    }
    else {
        count_x = 0;
        count_y = 0;
    }
#endif    
    
    port_write16(report+HID_MOUSE_REPORT_X_BYTE, count_x);
    port_write16(report+HID_MOUSE_REPORT_Y_BYTE, count_y);
    
#ifdef HAS_MOUSE_WHEEL
    int16_t mouse_wheel_value = quad_decoder_get_x_counter();
    int16_t mouse_wheel_delta = ((int32_t)mouse_wheel_value - quadrature_decoder_x_value_last_sent)/2;
    
    #if MOUSE_WHEEL_REVERSED       
        report[HID_MOUSE_REPORT_WHEEL_BYTE+1] = (-mouse_wheel_delta)>>8;
        report[HID_MOUSE_REPORT_WHEEL_BYTE]   = (-mouse_wheel_delta)&0xFF;
    #else
        report[HID_MOUSE_REPORT_WHEEL_BYTE+1] = (mouse_wheel_delta)>>8;
        report[HID_MOUSE_REPORT_WHEEL_BYTE]   = (mouse_wheel_delta)&0xFF;         
    #endif //MOUSE_WHEEL_REVERSED          

    quadrature_decoder_x_value_last_sent = mouse_wheel_value;
#else
    report[HID_MOUSE_REPORT_WHEEL_BYTE+1] = 0;
    report[HID_MOUSE_REPORT_WHEEL_BYTE]   = 0;
#endif

    report[HID_MOUSE_REPORT_BUTTON_BYTE] = 0;

    if (full_release == false) {
#if defined(HAS_GPIO_KEYS) && defined(HAS_MOUSE)    
        
        if(mouse_button_event_occured == true) {
            report[HID_MOUSE_REPORT_BUTTON_BYTE] |= (button_pressed[MOUSE_BUTTON_MIDDLE]<<2) | (button_pressed[MOUSE_BUTTON_RIGHT]<<1) | (button_pressed[MOUSE_BUTTON_LEFT]<<0);
        }
#endif        
#ifdef HAS_TOUCHPAD_TRACKPAD
        if(user_touch_left_click == true) {
            user_touch_left_click = false;
            report[HID_MOUSE_REPORT_BUTTON_BYTE] |= 1;
        }
#endif
    }
    
    report[HID_MOUSE_REPORT_H_DATA_BYTE+1] = 0;
    report[HID_MOUSE_REPORT_H_DATA_BYTE]   = 0;
#ifdef HAS_TOUCHPAD_TRACKPAD    
    if(port_send_notification(user_mouse_hdl, report, HID_MOUSE_REPORT_SIZE, false)) {
    #if defined(HAS_GPIO_KEYS) && defined(HAS_MOUSE)              
        mouse_button_event_occured = false;                  
    #endif
        return true;
    }
#endif
    return false;
}
#endif

#ifdef HAS_MOTION
void user_motion_start(void) 
{
    app_motion_start();
    motion_cpt_event = true;
}

void user_motion_stop(void) 
{
    app_motion_stop();
}

bool user_motion_on_ble_powered(void)
{
    app_motion_data_t data;
    
    bool ret = app_motion_on_ble_powered();
    
    if(motion_cpt_event == true && user_is_conn_upd_pending() == false && app_motion_read_data(&data)) {
        data.click_events = user_motion_left_click_pressed;
        port_send_notification(user_motion_hdl, (uint8_t *)&data, sizeof(app_motion_data_t), false);
        ret = true;
        motion_cpt_event = false;
    }
    
    return ret;
}
#endif

#ifdef HAS_TOUCHPAD_TRACKPAD
static void user_touchpad_handle_event(void)
{
    switch(user_touch_event) {        
        case APP_TOUCHPAD_TOUCH_AND_HOLD:
        case APP_TOUCHPAD_SWIPE_LEFT:
        case APP_TOUCHPAD_SWIPE_RIGHT:
        case APP_TOUCHPAD_SWIPE_UP:
        case APP_TOUCHPAD_SWIPE_DOWN:
        case APP_TOUCHPAD_ZOOM_IN:
        case APP_TOUCHPAD_ZOOM_OUT:
        case APP_TOUCHPAD_DOUBLE_FINGER_TAP:
        case APP_TOUCHPAD_SCROLL_UP:
        case APP_TOUCHPAD_SCROLL_DOWN:
        case APP_TOUCHPAD_SCROLL_RIGHT:
        case APP_TOUCHPAD_SCROLL_LEFT:
            // Add code for handling the other events
            break;
        default:
            break;
    }   
}
#endif

#ifdef HAS_TOUCHPAD_SLIDER
static void user_touchpad_handle_event(void)
{
    switch(user_touch_event) {
        // Rotate Right events will send Volume Up commands to the host
        case APP_TOUCHPAD_ROTATE_RIGHT:
            app_hid_report_create_extended_report(VOL_PLUS_KEY_BYTE, VOL_PLUS_KEY_MASK, VOL_PLUS_KEY_MASK);
            app_hid_report_create_extended_report(VOL_PLUS_KEY_BYTE, VOL_PLUS_KEY_MASK, 0);
            break;
        
        // Rotate Left gestures will send Volume Down commands to the host
        case APP_TOUCHPAD_ROTATE_LEFT:
            app_hid_report_create_extended_report(VOL_MINUS_KEY_BYTE, VOL_MINUS_KEY_MASK, VOL_MINUS_KEY_MASK);
            app_hid_report_create_extended_report(VOL_MINUS_KEY_BYTE, VOL_MINUS_KEY_MASK, 0);    
            break;
        
        // Flick Left gestures will send Mute commands to the host
        case APP_TOUCHPAD_FLICK_LEFT:
            app_hid_report_create_extended_report(MUTE_KEY_BYTE, MUTE_KEY_MASK, MUTE_KEY_MASK);
            app_hid_report_create_extended_report(MUTE_KEY_BYTE, MUTE_KEY_MASK, 0);
            break;

        // Flick Right gestures will send enter key to the host
        case APP_TOUCHPAD_FLICK_RIGHT:
            app_hid_report_modify_kbd_keyreport(0x28, true);
            app_hid_report_modify_kbd_keyreport(0x28, false);
            break;
        
        // Tap Up gestures will send "Up" commands to the host
        case APP_TOUCHPAD_TAP_UP:
            app_hid_report_modify_kbd_keyreport(0x52, true);
            app_hid_report_modify_kbd_keyreport(0x52, false);
            break;
        
        // Tap Down gestures will send "Down" commands to the host
        case APP_TOUCHPAD_TAP_DOWN:
            app_hid_report_modify_kbd_keyreport(0x51, true);
            app_hid_report_modify_kbd_keyreport(0x51, false);
            break;
        
        // Tap Left gestures will send "Left" commands to the host
        case APP_TOUCHPAD_TAP_LEFT:
            app_hid_report_modify_kbd_keyreport(0x50, true);
            app_hid_report_modify_kbd_keyreport(0x50, false);
            break;
        
        // Tap Right gestures will send "Right" commands to the host
        case APP_TOUCHPAD_TAP_RIGHT:
            app_hid_report_modify_kbd_keyreport(0x4F, true);
            app_hid_report_modify_kbd_keyreport(0x4F, false);
            break;
        default:
            break;    }
}
#endif

#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)
bool user_mouse_trackpad_on_ble_powered(void)
{
    bool ret = false;
    #if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)        
    if(mouse_cpt_event == true) {
    #if defined(HAS_MOUSE) && defined(MOUSE_GENERATE_TEST_PATTERN)
        extern bool user_start_mouse_test;
        if(user_start_mouse_test == true) {
            app_mouse_prepare_next_test_data_sample();
        }
    #endif     
        
        mouse_cpt_event = false;                            
        if(user_mouse_send_report(false) == true) {
            // The inactivity counter (if enabled) will be reloaded
            user_action_triggered();
            ret = true; // To send the report            
        }
    }
        #ifdef HAS_TOUCHPAD_TRACKPAD   
	if(user_touch_event != APP_TOUCHPAD_NO_EVENT) {        
	    user_touchpad_handle_event();
	    user_touch_event = APP_TOUCHPAD_NO_EVENT;
	    ret = true;
}    
#endif
    #endif

    #ifdef HAS_TOUCHPAD_SLIDER
    if(user_touch_event!=APP_TOUCHPAD_NO_EVENT) {        
        user_touchpad_handle_event();
        user_touch_event = APP_TOUCHPAD_NO_EVENT;
        return user_hid_report_on_ble_powered();
    }
    #endif    
    return ret;    
}    
#endif

#if defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)   
uint8_t user_touchpad_on_system_powered(void)
{
    uint8_t power_status = app_touchpad_poll();
    
#ifdef HAS_TOUCHPAD_TRACKPAD    
    if(user_touch_tracking == true) {
        power_status |= APP_BLE_WAKEUP;
    }        
    if(user_touch_left_click == true) {
        user_mouse_send_report(false);
        user_mouse_send_report(true); // Send a full release
        power_status |= APP_BLE_WAKEUP;
    }    
#endif
    
    // If a touch event has occured
    if(user_touch_event!= APP_TOUCHPAD_NO_EVENT) {
        power_status |= APP_BLE_WAKEUP;
        user_action_triggered();
    }    
    
    return power_status;
}
#endif

#ifdef HAS_MOUSE
uint8_t user_mouse_on_system_powered(void)
{
    uint8_t power_status = 0;
    #ifdef HAS_GPIO_KEYS
        if(mouse_button_event_occured == true) {
            if(user_mouse_send_report(false)) {
                power_status |= APP_BLE_WAKEUP;
            }
        }
    #endif
        if (mouse_sensor_poll == true) {
    #ifdef HAS_ACTION_INACTIVITY_TIMEOUT
        if (app_mouse_is_active() == true) {
            void user_initiate_activity_period(void);
            user_initiate_activity_period();
        }        
    #endif
        app_mouse_poll_sensor();
        mouse_sensor_poll = false;
    }     
    return power_status;
}


void user_systick_mouse_callback(void)
{
    mouse_sensor_poll = true;
}
#endif 
    
#ifdef HAS_TOUCHPAD_TRACKPAD
void user_touchpad_on_tracking_event(void *info)
{

	app_touchpad_evt_t *pInf = (app_touchpad_evt_t*)info;
	switch(pInf->touch_action)
	{
		case APP_TOUCHPAD_TRACK_STARTED :
			user_action_triggered();
            user_touch_tracking = true;
			break;
        
		case APP_TOUCHPAD_TRACKING : 	
			break;
        
		case APP_TOUCHPAD_TRACK_STOPPED : 
            user_touch_tracking = false;
			break;
	}

}
#endif


#ifdef HAS_TOUCHPAD_TRACKPAD
void user_touchpad_on_special_event(void *info)
{
    app_touchpad_evt_t *pInf = (app_touchpad_evt_t*)info;
    
    switch(pInf->touch_action) {
        // A single finger tap gesture
        // will send a left click to the host
        case APP_TOUCHPAD_SINGLE_FINGER_TAP:  
            user_touch_left_click = true;
            break;
        
        // All other events are copied in the user_touch_event variable
        // for further processing when ble is powered
        case APP_TOUCHPAD_RESET:
        case APP_TOUCHPAD_RELEASE:        
        case APP_TOUCHPAD_TOUCH_AND_HOLD:
        case APP_TOUCHPAD_SWIPE_LEFT:    
        case APP_TOUCHPAD_SWIPE_RIGHT:    
        case APP_TOUCHPAD_SWIPE_UP:  
        case APP_TOUCHPAD_SWIPE_DOWN:
        case APP_TOUCHPAD_ZOOM_IN:
        case APP_TOUCHPAD_ZOOM_OUT:
        case APP_TOUCHPAD_DOUBLE_FINGER_TAP:
        case APP_TOUCHPAD_SCROLL_UP:
        case APP_TOUCHPAD_SCROLL_DOWN:
        case APP_TOUCHPAD_SCROLL_RIGHT:
        case APP_TOUCHPAD_SCROLL_LEFT:
            user_touch_event = pInf->touch_action;
        break;
        default:
        break;
    }
}
#endif

#ifdef HAS_TOUCHPAD_SLIDER   
void user_touchpad_on_special_event(void *info)
{
    app_touchpad_evt_t *pInf = (app_touchpad_evt_t*)info;
    
    switch(pInf->touch_action) {  
        case APP_TOUCHPAD_TAP_UP:
        case APP_TOUCHPAD_TAP_DOWN:
        case APP_TOUCHPAD_TAP_RIGHT:
        case APP_TOUCHPAD_TAP_LEFT:
        case APP_TOUCHPAD_FLICK_LEFT:
        case APP_TOUCHPAD_FLICK_RIGHT:
        case APP_TOUCHPAD_ROTATE_LEFT:
        case APP_TOUCHPAD_ROTATE_RIGHT:
            user_touch_event = pInf->touch_action;
        break;
        default:
        break;
	}
}
#endif

#ifdef HAS_GPIO_KEYS
/**
 ****************************************************************************************
 * \brief     This callback is called when a GPIO key is pressed or released
 * \param[in] key     The key that has been pressed or released
 * \param[in] pressed true if the key is pressed, false if it is released
 ****************************************************************************************
 */
void user_gpio_keys_notification_cb(enum gpio_key key, bool pressed)
{
    #ifdef HAS_ACTION_INACTIVITY_TIMEOUT        
    user_initiate_activity_period();  
    #endif            
    button_pressed[key] = pressed;
    
    #ifdef HAS_MOUSE         
    extern bool mouse_button_event_occured;
    
    switch(key) {
    case MOUSE_BUTTON_LEFT:
    case MOUSE_BUTTON_RIGHT:            
    case MOUSE_BUTTON_MIDDLE:
        mouse_button_event_occured = true;
        break;   
    }
    #endif            
}
#endif

/**
 * \}
 * \}
 * \}
 */
