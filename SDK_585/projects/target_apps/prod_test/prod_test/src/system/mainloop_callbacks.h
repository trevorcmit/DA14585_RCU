/*****************************************************************************************
 *
 * @file mainloop_callbacks.h
 *
 * @brief Declaration of mainloop callbacks used by the production test application. 
  *
******************************************************************************************/


#ifndef _MAINLOOP_CALLBACKS_H_
#define _MAINLOOP_CALLBACKS_H_

#include "arch_api.h"

arch_main_loop_callback_ret_t app_on_ble_powered(void);

arch_main_loop_callback_ret_t app_on_full_power(void);

void app_going_to_sleep(sleep_mode_t sleep_mode);

void app_resume_from_sleep(void);

void send_tx_cmd_cmp_event(void);

#endif // _MAINLOOP_CALLBACKS_H_
