/*****************************************************************************************
 *
 * \file user_rcu.h
 *
 * \brief Main application header file.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

#ifndef _USER_RCU_H_
#define _USER_RCU_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_APP
 * \{
 * \addtogroup APP_RCU
 *
 * \brief Mouse sensor driver
 *
 * \{
******************************************************************************************/
 

/*
 * INCLUDE FILES
******************************************************************************************/

#include "rwip_config.h"
#include "rwip.h"
#include "app_api.h"

#include "app_task.h"                  // application task
#include "gapc_task.h"                 // gap functions and messages
#include "gapm_task.h"                 // gap functions and messages
#include "app.h"                       // application definitions
#include "co_error.h"                  // error code definitions

#include "app_nv_prom.h"
#include "app_con_fsm_defs.h"

/*
 * TYPE DEFINITIONS
******************************************************************************************/


/*
 * DEFINES
******************************************************************************************/

#ifdef HAS_CONNECTION_FSM        
    #define user_is_ble_connected(param) (app_con_fsm_get_state() == CONNECTED_ST)
#else
    #define user_is_ble_connected(param) (ke_state_get(TASK_APP) == APP_CONNECTED)
#endif  

#ifdef HAS_LED_INDICATORS
    #define USER_LEDS_RAMP(led_param) LEDS_RAMP(led_param)
    #define USER_LEDS_OFF(led_param) LEDS_OFF(led_param)
#else        
    #define USER_LEDS_RAMP(led_param) 
    #define USER_LEDS_OFF(led_param)
#endif


/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Handle connection request
 *
 * @param[in] connection_idx
 * @param[in] param
******************************************************************************************/
void user_on_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param);

/*****************************************************************************************
 * @brief   Configures the application when the connection is terminated.
 *
 * @param[in] param     parameters passed from the stack
******************************************************************************************/
void user_on_disconnect(struct gapc_disconnect_ind const *param);
/*****************************************************************************************
 * @brief
******************************************************************************************/
void user_on_init(void);

/*****************************************************************************************
 * @brief
******************************************************************************************/
void user_on_set_dev_config_complete(void);

/*****************************************************************************************
 * @brief   Handles what needs to be done after the completion of the initialization
 *          of all modules
******************************************************************************************/
void user_on_db_init_complete(void);

#ifdef HAS_CONNECTION_FSM
/**
****************************************************************************************
* @brief
* @param[in] status
****************************************************************************************
*/
void user_on_update_params_rejected(const uint8_t status);

/**
****************************************************************************************
* @brief
****************************************************************************************
*/
void user_on_update_params_complete(void);

/*****************************************************************************************
 * @brief
 *
 * @param[in] uuid
 * @param[in] attr_num
 * @param[in] value
******************************************************************************************/
void user_store_ccc(uint16_t uuid, int attr_num, int value);

    #if ((RWBLE_SW_VERSION_MAJOR >= 8) && BLE_BATT_SERVER) ||  BLE_BAS_SERVER
/*****************************************************************************************
 * @brief
 * @param[in] conidx
 * @param[in] ntf_cfg
******************************************************************************************/
    void user_bass_store_ccc(uint8_t conidx, uint8_t ntf_cfg);
    #endif
#else
/*****************************************************************************************
 * @brief
 * @param[in] status
******************************************************************************************/
void user_app_adv_undirect_complete(uint8_t status);
#endif // HAS_CONNECTION_FSM

#if BLE_APP_SEC
/*****************************************************************************************
 * @brief
 * @param[in] auth
******************************************************************************************/
void user_on_encrypt_ind(const uint8_t auth);
#endif // BLE_APP_SEC

#if (RWBLE_SW_VERSION_MAJOR >= 8)
/*****************************************************************************************
 * @brief
 * @param appearance
******************************************************************************************/
void user_app_on_get_dev_appearance(uint16_t* appearance);

/*****************************************************************************************
 * @brief
 * @param slv_params
******************************************************************************************/
void user_app_on_get_dev_slv_pref_params(struct gap_slv_pref* slv_params);

/*****************************************************************************************
 * @brief
 * @param ind
******************************************************************************************/
void user_on_data_length_change(struct gapc_le_pkt_size_ind *ind);
#endif

/**
****************************************************************************************
* @brief
* @param[in] msgid
* @param[in] param
* @param[in] dest_id
* @param[in] src_id
****************************************************************************************
*/
void user_process_catch_rest(
    ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id
);

/*****************************************************************************************
 * \brief   Process tasks while BLE is powered
 * \return  true device sleep must be blocked
******************************************************************************************/
arch_main_loop_callback_ret_t user_on_ble_powered(void);

/*****************************************************************************************
 * @brief
 * @return
******************************************************************************************/
arch_main_loop_callback_ret_t user_on_system_powered(void);

/*****************************************************************************************
 * @brief
******************************************************************************************/
void user_before_sleep(void);

/*****************************************************************************************
 * @brief
 * @param[in] sleep_mode
 * @return
******************************************************************************************/
sleep_mode_t user_validate_sleep(sleep_mode_t sleep_mode);

/*****************************************************************************************
 * @brief
 *
 * @param[in] sleep_mode
******************************************************************************************/
void user_going_to_sleep(sleep_mode_t sleep_mode);

/*****************************************************************************************
 * @brief
******************************************************************************************/
void user_resume_from_sleep(void);  

#if (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER)
/*****************************************************************************************
 * @brief  SUOTA status change callback
 *         Gets called when SUOTA status changes to update the application
 *
 * @param[in] suotar_event  The event of SUOTAR process
******************************************************************************************/
void user_on_suotar_status_change(const uint8_t suotar_event);
#endif

#ifdef HAS_ACTION_INACTIVITY_TIMEOUT
/*****************************************************************************************
 * @brief
******************************************************************************************/
void user_wakeup_activity_callback(void);

/*****************************************************************************************
 * @brief
******************************************************************************************/
void user_systick_callback(void);
#endif

#if (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER)
/*****************************************************************************************
 * @brief
******************************************************************************************/
void user_suota_timeout_timer_handler(void);
#endif

/*****************************************************************************************
 * @brief Perform actions required when a user action is detected
******************************************************************************************/
void user_action_triggered(void);

/*****************************************************************************************
 * @brief  Check if connection parameter update procedure has been completed
 *
 * \return true is connection parameter update is pending
******************************************************************************************/
bool user_is_conn_upd_pending(void);

/*****************************************************************************************
 * @brief     This callback is called when the connection state has changed to connected,
 *            connection in progress, disconnected, off, passcode entry started.  
 *
 * @param[in] type Type of connection FSM callback
******************************************************************************************/
void user_con_fsm_callback(enum con_fsm_state_update_callback_type type);

/*****************************************************************************************
 * @brief      This callback is called when bonding data are read from the non-volatile   
 *             memory so that the service database is updated
 *
 * @param[in]  uuid Position of the characteristic in the notification_info
 * @param[in]  attr_num Attribute number of the characteristic
 * @param[in]  value Value of the attribute
******************************************************************************************/
void user_update_attr_callback(uint16_t uuid, int attr_num, int value);

#ifdef HAS_HID_REPORT
/*****************************************************************************************
 * \brief   Process HID FIFO tasks while BLE is powered
 * \return  true device sleep must be blocked
******************************************************************************************/
bool user_hid_report_on_ble_powered(void);

#endif

#ifdef HAS_PWR_MGR
/*****************************************************************************************
 * \brief   This function is call when the inactivity timeout expires
******************************************************************************************/
void user_pwr_mgr_inactivity_callback(void);

#endif

/**
 * \}
 * \}
 * \}
 */
 
#endif // _USER_RCU_H_
