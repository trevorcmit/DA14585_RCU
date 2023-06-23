/**
* \file app_adv_fsm_config.h
* \brief Advertise FSM module configuration header file.
*/

#ifndef _APP_ADV_FSM_CONFIG_H_
#define _APP_ADV_FSM_CONFIG_H_

/**
* \addtogroup CONFIGURATION
* \{
* \addtogroup MODULE_CONFIG
* \{
* \addtogroup ADV_FSM_CFG
* \brief Advertise FSM module configuration
* \{
*/
 
#include "port_adv_fsm.h"


// Define the device name used in the advertisement data during advertising             
#define APP_DFLT_DEVICE_NAME    USER_DEVICE_NAME
                                                    
/**
 ****************************************************************************************
 * If defined the APP_DFLT_DEVICE_NAME will be automatically appended in advertising 
 * data. If it does not fit, a partial name will be appended in advertising data. The 
 * full name will be appended in scan response data.
 ****************************************************************************************
 */
#define AUTO_APPEND_DEVICE_NAME_IN_ADV_DATA

// Advertising data
#define APP_ADV_DATA USER_ADVERTISE_DATA

// Define HAS_SPECIAL_ADVERTISING to enable transmission of wake-up adv packets for waking up sleeping hosts
#undef HAS_SPECIAL_ADVERTISING


void app_con_fsm_handle_adv_notification(uint8_t notif);

static const adv_configuration_t adv_fsm_config = {    
    /**
    ****************************************************************************************
    * \brief This callback will be called to notify the application that the state of the FSM has changed
    ****************************************************************************************
    */
    .app_adv_notification_callback = app_con_fsm_handle_adv_notification,
    .disable_undirected_advertise  = true,  // Do not start unbonded advertise when bonded advertise has finished
    .directed_advertising_repeats  = 3,     // Number of timer to repeat directed advertising

    // DEFAULT = false, true does not seem to hurt it
    .disable_advertising_timeout   = false, // Set to true if advertising should not use timeouts (advertising timer)
  
    // Undirected advertising mode. Pairing to new hosts is allowed
    .adv_params[ADV_SETTING_UNDIRECTED].discoverable_timeout    = 120000, // Advertising time (in msec)
    .adv_params[ADV_SETTING_UNDIRECTED].adv_int_min             = 30,     // Minimum advertising interval in msec
    .adv_params[ADV_SETTING_UNDIRECTED].adv_int_max             = 30,     // Maximum advertising interval in msec
    .adv_params[ADV_SETTING_UNDIRECTED].adv_mode                = GAP_GEN_DISCOVERABLE, // Advertising mode
    .adv_params[ADV_SETTING_UNDIRECTED].adv_data                = APP_ADV_DATA,         // Advertising data
    .adv_params[ADV_SETTING_UNDIRECTED].adv_data_length         = sizeof(APP_ADV_DATA)-1,
    .adv_params[ADV_SETTING_UNDIRECTED].scan_rsp_data           = NULL,                 // Scan response data
    .adv_params[ADV_SETTING_UNDIRECTED].scan_rsp_data_length    = 0,

   // Limited Discoverable adverising mode when bonded to a host
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].discoverable_timeout  = 4000,   // Advertising time (in msec)
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].adv_int_min           = 20,     // Minimum advertising interval in msec
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].adv_int_max           = 20,     // Maximum advertising interval in msec
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].adv_mode              = GAP_LIM_DISCOVERABLE, // Advertising mode
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].adv_data              = NULL,                 // Advertising data
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].adv_data_length       = 0,
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].scan_rsp_data         = NULL,                 // Scan response data
    .adv_params[ADV_SETTING_UNDIRECTED_LIM].scan_rsp_data_length  = 0,

    /**
    ****************************************************************************************
    * Limited Discoverable advertising mode when bonded to a host after ADV_BONDED_LIM is 
    * completed. No pairing to a new host is allowed..
    ****************************************************************************************
    */
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].discoverable_timeout = 60000,  // Advertising time (in msec)
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].adv_int_min          = 30,     // Minimum advertising interval in msec
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].adv_int_max          = 40,     // Maximum advertising interval in msec
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].adv_mode             = GAP_GEN_DISCOVERABLE, // Advertising mode
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].adv_data             = NULL,                 // Advertising data
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].adv_data_length      = 0,
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].scan_rsp_data        = NULL,                 // Scan response data
    .adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].scan_rsp_data_length = 0,

    /**
    ****************************************************************************************
    * Slow undirected advertising mode after ADV_UNBONDED is completed.
    * Set to zero to skip this stage when when is_normally_connectable=false
    ****************************************************************************************
    */
    .adv_params[ADV_SETTING_SLOW].discoverable_timeout  = 0,      // Advertising time (in msec)
    .adv_params[ADV_SETTING_SLOW].adv_int_min           = 1000,   // Minimum advertising interval in msec
    .adv_params[ADV_SETTING_SLOW].adv_int_max           = 2500,   // Maximum advertising interval in msec
    .adv_params[ADV_SETTING_SLOW].adv_mode              = GAP_GEN_DISCOVERABLE, // Advertising mode
    .adv_params[ADV_SETTING_SLOW].adv_data              = NULL,                 // Advertising data
    .adv_params[ADV_SETTING_SLOW].adv_data_length       = 0,
    .adv_params[ADV_SETTING_SLOW].scan_rsp_data         = NULL,                 // Scan response data
    .adv_params[ADV_SETTING_SLOW].scan_rsp_data_length  = 0,
    
    #ifdef HAS_SPECIAL_ADVERTISING    
        /**
        ****************************************************************************************
        * Special undirected advertising mode used in order to wake up a sleeping host
        * or power off an active host
        ****************************************************************************************
        */
        .adv_params[ADV_SETTING_SPECIAL].discoverable_timeout  = 3000,      // Advertising time (in msec)
        .adv_params[ADV_SETTING_SPECIAL].adv_int_min           = 20,   // Minimum advertising interval in msec
        .adv_params[ADV_SETTING_SPECIAL].adv_int_max           = 20,   // Maximum advertising interval in msec
        .adv_params[ADV_SETTING_SPECIAL].adv_mode              = GAP_LIM_DISCOVERABLE, // Advertising mode
        .adv_params[ADV_SETTING_SPECIAL].adv_data              = NULL,                 // Advertising data - No effect here
        .adv_params[ADV_SETTING_SPECIAL].adv_data_length       = 0,                    // Advertising data length - No effect here 
        .adv_params[ADV_SETTING_SPECIAL].scan_rsp_data         = NULL,                 // Scan response data
        .adv_params[ADV_SETTING_SPECIAL].scan_rsp_data_length  = 0,
    #endif
};


/**
 ****************************************************************************************
 * Basic advertising platform functions
 * Define here which function will be used by the Advertising FSM
 ****************************************************************************************
 */
static const adv_util_funcs_t app_adv_fsm_funcs= {
    .adv_start_dir_adv = port_adv_fsm_start_adv_directed,
    .adv_start_und_adv = port_adv_fsm_start_adv_undirected,
    .adv_stop          = port_adv_fsm_adv_stop
};

/**
 * \}
 * \}
 * \}
 */

#endif // _APP_ADV_FSM_CONFIG_H_
