/**
 ****************************************************************************************
 *
 * \file user_rcu_kbd.h
 *
 * \brief Main application header file.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */

#ifndef _USER_RCU_KBD_H_
#define _USER_RCU_KBD_H_

/**
 ****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_APP
 * \{
 * \addtogroup APP_RCU
 *
 * \brief Mouse sensor driver
 *
 * \{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "app_kbd_defs.h"
#include "hogpd.h"
#include "hogpd_task.h"

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#ifdef HAS_KBD

/**
 ****************************************************************************************
 * \brief User keyboard function that turns on LEDs related with keyboard special functions
 ****************************************************************************************
 */
void user_rcu_kbd_turn_on_leds(void);


/**
 ****************************************************************************************
 * \brief User keyboard function that turns off LEDs related with keyboard special functions
 ****************************************************************************************
 */
void user_rcu_kbd_turn_off_leds(void);

/**
 ****************************************************************************************
 * @brief     This callback is called when a notification is send by keyboard module. 
 *            Notifications sre defined in kbd_notification. Custom key presses are also
 *            sent as notifications. Custom keys are defined in custom_keys enum and 
 *            used in kbd_keymap or multi_key_combinations
 *
 * @param[in] notification The keyboard notification or custom key press.
 ****************************************************************************************
 */
void user_kbd_notification_cb(enum kbd_notification notification);

#ifdef HAS_IR
/**
 ****************************************************************************************
 * @brief     This callback when a keyboard key press or release is detected.
 *
 * @param[in] row     the keyboard matrix row of the key
 * @param[in] column  the keyboard matrix colomn of the key
 * @param[in] pressed true if key is pressed, false if it is released
 ****************************************************************************************
 */
void user_kbd_key_detected_cb(uint16_t row, uint16_t column, bool pressed);
#endif // HAS_IR

#ifdef HAS_LONG_PRESS_POWERUP_BUTTON
/**
 ****************************************************************************************
 * @brief Long keypress timer handler
 ****************************************************************************************
 */
void user_long_keypress_timer_handler(void);

#endif // HAS_LONG_PRESS_POWERUP_BUTTON

#if (BLE_HID_DEVICE)
/**
 ****************************************************************************************
 * @brief Handle keyboard LEDs on/off. Called when keyboard LED report is received
 * 
 * param[in] conidx      The current connection index
 * param[in] report_info The report info of the LED report
 ****************************************************************************************
 */
void user_keyboard_led_report_callback(uint8_t conidx, 
                                       struct hogpd_report_info *report_info);

#endif // BLE_HID_DEVICE

#endif // HAS_KBD


#ifdef DEBUG_EMULATE_PACKET_LOSS
/**
 ****************************************************************************************
 * \brief Radio mute timer handler 
 ****************************************************************************************
 */
void user_rcu_kbd_radio_mute_timer_handler(void);

#endif // DEBUG_EMULATE_PACKET_LOSS


/**
 ****************************************************************************************
 * \brief Send HID keyboard reports to BLE. This function checks keycode_buffer to create
 *        more HID reports if possible.
 * \return KEEP_POWERED if there are reports that must be sent to BLE, otherwise
 *         GOTO_SLEEP.
 ****************************************************************************************
 */
bool user_rcu_send_key_reports(void);

/**
 * \}
 * \}
 * \}
 */

#endif // _USER_RCU_KBD_H_
