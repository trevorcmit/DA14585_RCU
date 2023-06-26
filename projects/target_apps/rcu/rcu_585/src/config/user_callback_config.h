/*****************************************************************************************
 *
 * \file user_callback_config.h
 *
 * \brief SDK Callback functions configuration file.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

#ifndef _USER_CALLBACK_CONFIG_H_
#define _USER_CALLBACK_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup APP_CONFIG
 * \{
 * \addtogroup SDK_CALLBACKS_CFG
 *
 * \brief SDK Callback functions configuration
 * \{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include "app_callback.h"
#include "app_default_handlers.h"
#include "app_bass.h"
#include "app_proxr.h"
#include "app_suotar.h"
#include "app_prf_types.h"
#include "app_hogpd.h"
#include "port_adv_fsm.h"
#include "user_rcu.h"

/*
 * LOCAL VARIABLE DEFINITIONS
******************************************************************************************/
#if (BLE_BATT_SERVER)
static const struct app_bass_cb user_app_bass_cb = {
    .on_batt_level_upd_rsp      = NULL,
    #ifdef HAS_CONNECTION_FSM
    .on_batt_level_ntf_cfg_ind  = user_bass_store_ccc,
    #else
    .on_batt_level_ntf_cfg_ind  = NULL,
    #endif
};
#endif

#if BLE_SUOTA_RECEIVER
static const struct app_suotar_cb user_app_suotar_cb = {
    .on_suotar_status_change = user_on_suotar_status_change,
};
#endif

static const struct app_callbacks user_app_callbacks = {
    .app_on_connection              = user_on_connection,
    .app_on_disconnect              = user_on_disconnect,
#ifdef HAS_CONNECTION_FSM
    .app_on_update_params_rejected  = user_on_update_params_rejected,
    .app_on_update_params_complete  = user_on_update_params_complete,
    .app_on_set_dev_config_complete = user_on_set_dev_config_complete,
    .app_on_adv_undirect_complete   = port_adv_fsm_on_undirect_complete,
    .app_on_adv_direct_complete     = port_adv_fsm_on_direct_complete,
#else
    .app_on_update_params_rejected  = NULL,
    .app_on_update_params_complete  = NULL,
    .app_on_set_dev_config_complete = default_app_on_set_dev_config_complete,
    .app_on_adv_undirect_complete   = user_app_adv_undirect_complete,
    .app_on_adv_direct_complete     = NULL,
#endif
    .app_on_adv_nonconn_complete    = NULL,
    .app_on_db_init_complete        = user_on_db_init_complete,
    .app_on_scanning_completed      = NULL,
    .app_on_adv_report_ind          = NULL, 
    .app_on_get_dev_appearance      = user_app_on_get_dev_appearance,
    .app_on_get_dev_slv_pref_params = user_app_on_get_dev_slv_pref_params,
    .app_on_set_dev_info            = default_app_on_set_dev_info,
    .app_on_data_length_change      = user_on_data_length_change,
    .app_on_update_params_request   = default_app_update_params_request,
#if BLE_APP_SEC && !defined(HAS_CONNECTION_FSM)
    // if HAS_CONNECTION_FSM is defind the DLG_SEC is excluded. 
    // No need to define the callbacks. 
    // The massages are handled in user_process_catch_rest
    .app_on_pairing_request         = default_app_on_pairing_request,
    .app_on_tk_exch_nomitm          = default_app_on_tk_exch_nomitm,
    .app_on_irk_exch                = NULL,
    .app_on_csrk_exch               = default_app_on_csrk_exch,
    .app_on_ltk_exch                = default_app_on_ltk_exch,
    .app_on_pairing_succeded        = NULL,
    .app_on_encrypt_ind             = user_on_encrypt_ind,
    .app_on_mitm_passcode_req       = NULL,
    .app_on_encrypt_req_ind         = default_app_on_encrypt_req_ind,

    .app_on_security_req_ind        = NULL,
#endif
};

// Default Handler Operations
static const struct default_app_operations user_default_app_operations = {
#ifdef HAS_CONNECTION_FSM
    .default_operation_adv = NULL,
#else    
    .default_operation_adv = default_advertise_operation,
#endif
};

static void (*const app_process_catch_rest_cb)(ke_msg_id_t const msgid, void const *param,
                                         ke_task_id_t const dest_id, ke_task_id_t const src_id) = user_process_catch_rest;

static const struct arch_main_loop_callbacks user_app_main_loop_callbacks = {
    .app_on_init            = user_on_init,

    // By default the watchdog timer is reloaded and resumed when the system wakes up.
    // The user has to take into account the watchdog timer handling (keep it running, 
    // freeze it, reload it, resume it, etc), when the app_on_ble_powered() is being 
    // called and may potentially affect the main loop.
    .app_on_ble_powered     = user_on_ble_powered,

    // By default the watchdog timer is reloaded and resumed when the system wakes up.
    // The user has to take into account the watchdog timer handling (keep it running, 
    // freeze it, reload it, resume it, etc), when the app_on_system_powered() is being 
    // called and may potentially affect the main loop.
    .app_on_system_powered  = user_on_system_powered,

    .app_before_sleep       = user_before_sleep,
    .app_validate_sleep     = user_validate_sleep,
    .app_going_to_sleep     = user_going_to_sleep,
    .app_resume_from_sleep  = user_resume_from_sleep,
};

//place in this structure the app_<profile>_db_create and app_<profile>_enable functions
//for SIG profiles that do not have this function already implemented in the SDK
//or if you want to override the functionality. Check the prf_func array in the SDK
//for your reference of which profiles are supported.
static const struct prf_func_callbacks user_prf_funcs[] =
{   
#if (BLE_DIS_SERVER)
    {TASK_ID_DISS,          app_diss_create_db, NULL},
#endif
#if (BLE_BATT_SERVER)
    {TASK_ID_BASS,          app_bass_create_db, app_bass_enable}, // Battery profile must be created before HOGPD
#endif
#if (BLE_HID_DEVICE)
    {TASK_ID_HOGPD, app_hogpd_create_db, app_hogpd_enable},
#endif
    {TASK_ID_INVALID,    NULL, NULL}   // DO NOT MOVE. Must always be last  
};

/**
 * \}
 * \}
 * \}
 */

#endif // _USER_CALLBACK_CONFIG_H_
