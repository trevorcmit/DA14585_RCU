/**
 ****************************************************************************************
 *
 * \file app_adv_fsm.c
 *
 * \brief Advertise FSM module source file
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
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup APP_ADV_FSM
 *
 * \brief Advertising FSM application
 * \{
 ****************************************************************************************          
 */

#ifdef HAS_CONNECTION_FSM

#include "app_adv_fsm.h"
#include <app_adv_fsm_config.h>
#include "port_adv_fsm.h"
#include <port_multi_bond.h>
#include "port_timer.h"

typedef enum {
    #ifdef HAS_SPECIAL_ADVERTISING
        ADV_SPECIAL,
    #endif
        ADV_UNDIRECTED_SLOW,     // do not change assigned values
        ADV_DIRECTED,
        ADV_UNDIRECTED_LIM,
        ADV_UNDIRECTED,
        ADV_UNDIRECTED_NO_PAIRING,
        ADV_FSM_EVENT_PENDING,
        ADV_IDLE,
}adv_states;


#ifdef HAS_SPECIAL_ADVERTISING
uint8_t adv_special_data[ADV_DATA_LEN - 3]                      __PORT_RETAINED;
uint8_t adv_special_data_length                                 __PORT_RETAINED;
#endif

adv_fsm_events_t adv_pending_event                              __PORT_RETAINED;

extern struct bd_addr dev_bdaddr;
extern struct bd_addr alt_dev_bdaddr;


bool bd_address_requested                                       __PORT_RETAINED;
                    
uint32_t directed_adv_repeat_counter                            __PORT_RETAINED;
                    
adv_states current_adv_state                                    __PORT_RETAINED;

                    
enum adv_filter_policy adv_und_filter_policy                    __PORT_RETAINED;
struct bd_addr app_adv_bd_address                               __PORT_RETAINED;

adv_notification_types_t adv_fsm_next_notification              __PORT_RETAINED;                   

#ifdef AUTO_APPEND_DEVICE_NAME_IN_ADV_DATA
    uint8_t app_adv_data_length                                 __PORT_RETAINED;        // Advertising data length
    uint8_t app_adv_data[ADV_DATA_LEN - 3]                      __PORT_RETAINED;        // Advertising data
    uint8_t app_scanrsp_data_length                             __PORT_RETAINED;        // Scan response data length
    uint8_t app_scanrsp_data[SCAN_RSP_DATA_LEN]                 __PORT_RETAINED;        // Scan response data
#else
    #define app_adv_data_length             adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED].adv_data_length
    #define app_adv_data                    adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED].adv_data
    #define app_scanrsp_data_length         adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED].scan_rsp_data_length
    #define app_scanrsp_data                adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED].scan_rsp_data
#endif


__attribute__((unused)) static const char adv_state_names[][18] = {
    #ifdef HAS_SPECIAL_ADVERTISING
    "ADV_SPECIAL",
    #endif
    "ADV_UNDIR_SLOW", 
    "ADV_DIRECTED", 
    "ADV_UNDIR_LIM",
    "ADV_UNDIRECTED", 
    "ADV_UNDIR_NO_PAIR", 
    "ADV_EVT_PENDING",
    "ADV_IDLE",
};

__attribute__((unused)) static const char adv_events_names[][20] = {
    "ADV_NO_EVENT", 
    "UND_ADV_COMPLETED", 
    "DIR_ADV_COMPLETED",
    "UND_ADV_TIMED_OUT", 
    "DIR_ADV_INTERRUPTED",
    "UND_ADV_INTERRUPTED", 
    "START_ADV",
    "START_ADV_LIM", 
    "START_ADV_NO_PAIR", 
    "START_ADV_DIR", 
    "STOP_ADV",
    #ifdef HAS_SPECIAL_ADVERTISING
    "START_ADV_SPECIAL"
    #endif
};

/**
 ****************************************************************************************
 * \brief Get the advertising settings index, depending on the undirected advertising state
 *
 * \param[in] state The undirected advertising state for which we need the settings index
 *
 * \return adv_settings_idx_t The settings index
 ****************************************************************************************
 */
static adv_settings_idx_t adv_fsm_get_adv_settings_idx(adv_states state)
{
    switch(state)
    {
        case ADV_UNDIRECTED_SLOW:
            return ADV_SETTING_SLOW;
        
        case ADV_UNDIRECTED_LIM:
            return ADV_SETTING_UNDIRECTED_LIM;

        case ADV_UNDIRECTED:
            return ADV_SETTING_UNDIRECTED;
        
        case ADV_UNDIRECTED_NO_PAIRING:
            return ADV_SETTING_UNDIRECTED_NO_PAIR;
        
#ifdef HAS_SPECIAL_ADVERTISING        
        case ADV_SPECIAL:
            return ADV_SETTING_SPECIAL;
#endif        
        
        default:
            ASSERT_ERROR(0);
            return ADV_SETTING_UNDIRECTED;
    }
}


/**
 ****************************************************************************************
 * \brief Set a notification to be sent by the adv fsm to the main fsm
 *
 * \param[in] notif     The type of the notification to be sent
 ****************************************************************************************
 */
static void app_adv_fsm_set_and_send_notification(adv_notification_types_t notif)
{
    adv_fsm_next_notification = notif;
}


#ifdef AUTO_APPEND_DEVICE_NAME_IN_ADV_DATA
/**
 ****************************************************************************************
 * \brief   Sets the advertising and the scan response data
 ****************************************************************************************
 */
static void app_adv_fsm_set_adv_data(void)
{
        adv_settings_idx_t adv_state_default_settings = adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED);
        
        // Per type adverising data cannot be used when AUTO_APPEND_DEVICE_NAME_IN_ADV_DATA is defined
        ASSERT_ERROR(adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED_LIM].adv_data == NULL        && adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED_LIM].scan_rsp_data == NULL &&
                 adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].adv_data == NULL && adv_fsm_config.adv_params[ADV_SETTING_UNDIRECTED_NO_PAIR].scan_rsp_data == NULL &&
                 adv_fsm_config.adv_params[ADV_SETTING_SLOW].adv_data == NULL              && adv_fsm_config.adv_params[ADV_SETTING_SLOW].scan_rsp_data == NULL)
    
        /*-----------------------------------------------------------------------------
         * Set the Advertising Data
         *-----------------------------------------------------------------------------*/
#if (NVDS_SUPPORT)
        if (nvds_get(NVDS_TAG_APP_BLE_ADV_DATA, &app_adv_data_length,
                &app_adv_data[0]) != NVDS_OK)
#endif //(NVDS_SUPPORT)
        {
                app_adv_data_length = adv_fsm_config.adv_params[adv_state_default_settings].adv_data_length;
                memcpy(&app_adv_data[0], adv_fsm_config.adv_params[adv_state_default_settings].adv_data, app_adv_data_length);
        }

        
        /*-----------------------------------------------------------------------------
         * Set the Scan Response Data
         *-----------------------------------------------------------------------------*/
#if (NVDS_SUPPORT)
        if (nvds_get(NVDS_TAG_APP_BLE_SCAN_RESP_DATA, &app_scanrsp_data_length,
                &app_scanrsp_data[0]) != NVDS_OK)
#endif //(NVDS_SUPPORT)
        {
                if (adv_fsm_config.adv_params[adv_state_default_settings].scan_rsp_data != NULL && app_scanrsp_data_length > 0) {
                    app_scanrsp_data_length = adv_fsm_config.adv_params[adv_state_default_settings].scan_rsp_data_length;
                        memcpy(&app_scanrsp_data[0], adv_fsm_config.adv_params[adv_state_default_settings].scan_rsp_data, app_scanrsp_data_length);
                }
                else {
                    app_scanrsp_data_length = 0;                    
                }
        }

#ifdef APP_DFLT_DEVICE_NAME
        
        /*-----------------------------------------------------------------------------
         * Add the Device Name in the Advertising Data
         *-----------------------------------------------------------------------------*/
        // Get available space in the Advertising Data
        int8_t device_name_length = APP_ADV_DATA_MAX_SIZE - app_adv_data_length - 2;

        // Check if data can be added to the Advertising data
        if (device_name_length > 0) {
                // Get default Device Name (No name if not enough space)
                int8_t temp_len;

                temp_len = (strlen(APP_DFLT_DEVICE_NAME) <= device_name_length) ? strlen(APP_DFLT_DEVICE_NAME) : 0;
                if (temp_len > 0) {
                        device_name_length = temp_len;
                }
                // else device_name_length shows the available space in the ADV pkt

                memcpy(&app_adv_data[app_adv_data_length + 2], APP_DFLT_DEVICE_NAME, device_name_length);
                app_adv_data[app_adv_data_length] = device_name_length + 1;                // Length

                if (temp_len > 0) {
                        app_adv_data[app_adv_data_length + 1] = '\x09';  // Complete Local Name Flag
                        app_adv_data_length += (device_name_length + 2); // Update Advertising Data Length
                }
                else {
                        app_adv_data[app_adv_data_length + 1] = '\x08'; // Shortened Local Name Flag
                        app_adv_data_length += (device_name_length + 2); // Update Advertising Data Length
                        device_name_length = 0; // To add the full name in the Scan Response data
                }
        }

        if (device_name_length > 0)
                return; // device name has been added

        /*-----------------------------------------------------------------------------
         * Add the Device Name in the Advertising Scan Response Data
         *-----------------------------------------------------------------------------*/
        // Get available space in the Advertising Data
        device_name_length = APP_ADV_DATA_MAX_SIZE - app_scanrsp_data_length - 2;

        // Check if data can be added to the Advertising data
        if (device_name_length > 0) {
                // Get default Device Name (No name if not enough space)
                device_name_length = (strlen(APP_DFLT_DEVICE_NAME) <= device_name_length) ? strlen(APP_DFLT_DEVICE_NAME) : 0;
                if (device_name_length > 0) {
                        memcpy(&app_scanrsp_data[app_scanrsp_data_length + 2], APP_DFLT_DEVICE_NAME, device_name_length);
                        app_scanrsp_data[app_scanrsp_data_length] = device_name_length + 1; // Length
                        app_scanrsp_data[app_scanrsp_data_length + 1] = '\x09'; // Device Name Flag

                        app_scanrsp_data_length += (device_name_length + 2); // Update Scan response Data Length
                }
        }
#endif // APP_DFLT_DEVICE_NAME
}

#endif


/**
 ****************************************************************************************
 * \brief Send the undirected advertising request to the BLE stack
 *
 * \param[in] undFilterP              The advertising filter policy
 * \param[in] adv_state_settings_idx  The undirected advertising settings idx
 ****************************************************************************************
 */
static void send_adv_undirected_request(uint8_t undFilterP, uint8_t adv_state_settings_idx)
{
        start_adv_data_t data;

        
        // The filter policy is defined prior to the call to this function.
        // An application may wish to advertise using various filter policies, depending
        // on its state i.e. it may use ADV_ALLOW_SCAN_ANY_CON_WLST when it wants to
        // connect to known hosts only or ADV_ALLOW_SCAN_ANY_CON_ANY when it wants to
        // pair to new hosts.
        data.filter_policy = undFilterP;
        
        if(adv_state_settings_idx == ADV_SETTING_UNDIRECTED) {      
                data.adv_data = app_adv_data;
                data.adv_data_length = app_adv_data_length;
                data.scanrsp_data = app_scanrsp_data;
                data.scanrsp_data_length = app_scanrsp_data_length;
        }
        else {
        
                if(adv_fsm_config.adv_params[adv_state_settings_idx].adv_data) {
                    data.adv_data        = adv_fsm_config.adv_params[adv_state_settings_idx].adv_data;
                    data.adv_data_length = adv_fsm_config.adv_params[adv_state_settings_idx].adv_data_length;
                }
                else {
                    data.adv_data        = app_adv_data;
                    data.adv_data_length = app_adv_data_length;
                }
                    
                if(adv_fsm_config.adv_params[adv_state_settings_idx].scan_rsp_data) {    
                    data.scanrsp_data        = adv_fsm_config.adv_params[adv_state_settings_idx].scan_rsp_data;
                    data.scanrsp_data_length = adv_fsm_config.adv_params[adv_state_settings_idx].scan_rsp_data_length;
                }
                else {
                    data.scanrsp_data = app_scanrsp_data;
                    data.scanrsp_data_length = app_scanrsp_data_length;
                }
        }

        data.adv_params = &adv_fsm_config.adv_params[adv_state_settings_idx];
        
        // Send undirected advertising request
        if(app_adv_fsm_funcs.adv_start_und_adv) {
                app_adv_fsm_funcs.adv_start_und_adv(&data);
        }
        else {
                ASSERT_ERROR(0);
        }
        
}


/**
 ****************************************************************************************
 * \brief Starts Undirected Advertising
 *
 * \param[in] filterPolicy      The advertising filter policy
 * \param[in] adv_settings_idx  The undirected advertising settings idx
 ****************************************************************************************
 */
static void start_adv_undirected(uint8_t filterPolicy, adv_settings_idx_t adv_settings_idx)
{
        send_adv_undirected_request(filterPolicy, (uint8_t)adv_settings_idx);
        
        // Start the advertising timer if advertising with timeout is supported
        if (adv_fsm_config.disable_advertising_timeout == false) {
            port_timer_set(APP_ADV_FSM_TIMER, 0, adv_fsm_config.adv_params[adv_settings_idx].discoverable_timeout);
        }    
}


#ifdef HAS_SPECIAL_ADVERTISING
/**
 ****************************************************************************************
 * \brief Starts Undirected Special Advertising, uses manufacturer specific data provided 
 * by adv_special_data
 ****************************************************************************************
 */
static void start_adv_undirected_special(void)
{
    start_adv_data_t data;

    data.filter_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;
        
    data.adv_data        = adv_special_data;
    data.adv_data_length = adv_special_data_length;
        
    if(adv_fsm_config.adv_params[ADV_SETTING_SPECIAL].scan_rsp_data) {    
        data.scanrsp_data        = adv_fsm_config.adv_params[ADV_SETTING_SPECIAL].scan_rsp_data;
        data.scanrsp_data_length = adv_fsm_config.adv_params[ADV_SETTING_SPECIAL].scan_rsp_data_length;
    }
    else {
        data.scanrsp_data = app_scanrsp_data;
        data.scanrsp_data_length = app_scanrsp_data_length;
    }
        

    data.adv_params = &adv_fsm_config.adv_params[ADV_SETTING_SPECIAL];
    
    // Send undirected advertising request
    if(app_adv_fsm_funcs.adv_start_und_adv) {
            app_adv_fsm_funcs.adv_start_und_adv(&data);
    }
    else {
            ASSERT_ERROR(0);
    }  
    
    // Start the advertising timer if advertising with timeout is supported
    if (adv_fsm_config.disable_advertising_timeout == false) {
        port_timer_set(APP_ADV_FSM_TIMER, 0, adv_fsm_config.adv_params[ADV_SETTING_SPECIAL].discoverable_timeout);
    } 
    
}
#endif

/**
 ****************************************************************************************
 * \brief Starts Directed Advertising to a given peer address
 *
 * \param[in] peerAddr  The address to which the directed advertising will be performed
 ****************************************************************************************
 */
static void start_adv_directed(struct bd_addr *peerAddr)
{
        start_adv_direct_data_t data;

        ASSERT_ERROR(adv_fsm_config.directed_advertising_repeats > 0);

        data.peer_addr_type = ADDR_PUBLIC;
        data.peer_addr = peerAddr;

        data.addr = &dev_bdaddr;

        if(app_adv_fsm_funcs.adv_start_dir_adv) {
                app_adv_fsm_funcs.adv_start_dir_adv(&data);
        }
        else {
                ASSERT_ERROR(0);
        }

} 



/************************************************************************************
 ********************* Advertise FSM State Event Handlers ****************************
 ************************************************************************************/


/**
 ****************************************************************************************
 * \brief Common event handler when being in undirected advertising states
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_undirected_common(adv_fsm_events_t adv_fsm_evt)
{
    switch(adv_fsm_evt)
    {
        // A connection request interrupted the undirected advertising
        case UND_ADV_INTERRUPTED:
            
            // Stop/Clear the undirected advertising timer, 
            // since advertsing has been interrupted
            port_timer_clear(APP_ADV_FSM_TIMER, BLE_TASK);   
        
            // Also notify the main fsm that undirected advertsing has ended
            app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_ENDED);
            
            // Set the advertising state to IDLE
            return ADV_IDLE;

        // The undirected advertising timer has timed out
        case UND_ADV_TIMED_OUT:
            
            // Stop undirected advertising
            if(app_adv_fsm_funcs.adv_stop) {
                    app_adv_fsm_funcs.adv_stop();
            }        
            else {
                    ASSERT_ERROR(0);
            }
            
            // Wait for undirected advertising to end completely
            // So stay at the same state until we receive an und completed event
            return current_adv_state;
        
        case START_ADV:
        case START_ADV_LIM:
        case START_ADV_NO_PAIR:
        case START_ADV_DIR:
        case STOP_ADV:         
    #ifdef HAS_SPECIAL_ADVERTISING
        case START_ADV_SPECIAL:
    #endif
            // Backup the pending event
            adv_pending_event = adv_fsm_evt;
        
            // Stop the advertising timer
            port_timer_clear(APP_ADV_FSM_TIMER, BLE_TASK);   
            
            // Stop undirected advertising as well
            if(app_adv_fsm_funcs.adv_stop) {
                app_adv_fsm_funcs.adv_stop();
            }
            else {
                    ASSERT_ERROR(0);
            }
            
            // The main fsm bypassed the advertising fsm normal function
            // So go to con fsm pending event state in order to wait for advertising
            // to finish completely
            return ADV_FSM_EVENT_PENDING;
        
        default:
            ASSERT_ERROR(0);
            return ADV_IDLE;    
    }
}


/**
 ****************************************************************************************
 * \brief Event handler when being in SLOW undirected advertising state (ADV_UNDIRECTED_SLOW)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_undirected_slow(adv_fsm_events_t adv_fsm_evt)
{
    switch(adv_fsm_evt)
    {
        case UND_ADV_COMPLETED:
                // Notify the main fsm that undirected advertsing has ended
                app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_ENDED);
                
                // Go to IDLE state
                return ADV_IDLE;
                    
        default:
                return adv_state_handle_undirected_common(adv_fsm_evt);
    }
}


/**
 ****************************************************************************************
 * \brief Event handler when being in default Undirected Advertising state (ADV_UNDIRECTED)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_undirected(adv_fsm_events_t adv_fsm_evt)
{
    switch(adv_fsm_evt)
    {
        case UND_ADV_COMPLETED:
                // If slow undirected advertising is supported
                if(adv_fsm_config.adv_params[adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_SLOW)].discoverable_timeout) {
                    
                    // Start slow undirected advertising
                    start_adv_undirected(adv_und_filter_policy, adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_SLOW));
                    
                    // Notify the main fsm that undirected advertising has started
                    app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_STARTED);
                    
                    // And go to slow undirected advertising state
                    return ADV_UNDIRECTED_SLOW;
                }
                else {
                    // Else notify the main fsm that undirected advertsing has ended
                    app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_ENDED);
                    
                    // And go to ADV_IDLE state
                    return ADV_IDLE;
                }
                
        default:
                return adv_state_handle_undirected_common(adv_fsm_evt);
    }
}


/**
 ****************************************************************************************
 * \brief Event handler when being in Limited Undirected Advertising state (ADV_UNDIRECTED_LIM)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_undirected_lim(adv_fsm_events_t adv_fsm_evt)
{
    switch(adv_fsm_evt)
    {
        case UND_ADV_COMPLETED:
            if(adv_fsm_config.disable_undirected_advertise == false) {
                 
                 // Start undirected advertising - NO PAIRING
                 start_adv_undirected(adv_und_filter_policy, adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_NO_PAIRING));
                 
                 // Notify the main fsm that undirected advertising has started
                 app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_STARTED);
                
                return ADV_UNDIRECTED_NO_PAIRING;
            }
            else {
                app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_LIM_ENDED);
                return ADV_IDLE;
            }
     
        // A connection request interrupted the undirected advertising
        case UND_ADV_INTERRUPTED:
            
            // Stop/Clear the undirected advertising timer, 
            // since advertsing has been interrupted
            port_timer_clear(APP_ADV_FSM_TIMER, BLE_TASK);   
        
            // Also notify the main fsm that undirected advertsing has ended
            app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_LIM_ENDED);
            
            // Set the advertising state to IDLE
            return ADV_IDLE;

        default:
                return adv_state_handle_undirected_common(adv_fsm_evt);
    }
}


/**
 ****************************************************************************************
 * \brief Event handler when being in Undirected Advertising 
 *       - no pairing state (ADV_UNDIRECTED_NO_PAIRING)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_undirected_no_pair(adv_fsm_events_t adv_fsm_evt)
{
    switch(adv_fsm_evt)
    {
        case UND_ADV_COMPLETED:
                // If slow undirected advertising is supported
                if(adv_fsm_config.adv_params[adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_SLOW)].discoverable_timeout) {
                        // Start slow undirected advertising
                        start_adv_undirected(adv_und_filter_policy, adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_SLOW));
            
                        // Notify the main fsm that undirected advertising has started
                        app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_STARTED);
        
                        // And go to slow undirected advertising state
                        return ADV_UNDIRECTED_SLOW;
                }
                else { 
                        // Notify the main fsm that undirected advertising has started
                        app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_ENDED);
                    
                        return ADV_IDLE;
                }
                
                
        default:
                return adv_state_handle_undirected_common(adv_fsm_evt);
    }
}


/**
 ****************************************************************************************
 * \brief Event handler when being in Directed Advertising state (ADV_DIRECTED)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_directed(adv_fsm_events_t adv_fsm_evt)
{
    switch(adv_fsm_evt)
    {
        // A connection request interrupted the directed advertising
        case DIR_ADV_INTERRUPTED:
            
            // Directed advertising has stopped due to connection, nothing more to do
            // Just notify the main FSM
            app_adv_fsm_set_and_send_notification(ADV_FSM_DIR_ADV_ENDED);
        
            // Set the state to IDLE
            return ADV_IDLE;
        
           
        // Directed Advertising has completed
        case DIR_ADV_COMPLETED:
            
            // If we still haven't reached the directed advertising repeats
            if (directed_adv_repeat_counter < (adv_fsm_config.directed_advertising_repeats - 1)) {
                
                // Start directed advertising again to the previous address
                dbg_printf(DBG_ADV_LVL, "(Repeating to %02x:%02x:%02x:%02x:%02x:%02x)",
                    app_adv_bd_address.addr[5], app_adv_bd_address.addr[4],
                    app_adv_bd_address.addr[3], app_adv_bd_address.addr[2],
                    app_adv_bd_address.addr[1], app_adv_bd_address.addr[0]);
                start_adv_directed(&app_adv_bd_address);
                
                // Increment the repeat counter by one
                ++directed_adv_repeat_counter;
                
                // We are still in Directed Advertising State
                return ADV_DIRECTED;
            }
            else {
                // Else if undirected advertise is not disabled, our scenario tells us to start undirected
                // advertising with no pairing
                if(adv_fsm_config.disable_undirected_advertise == false) {
                    // According to our scenario the next state will be undirected advertising
                    // with no pairing, so start undirected advertising with that setting
                    start_adv_undirected(adv_und_filter_policy, adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_NO_PAIRING));
    
                    // Notify the main fsm that undirected advertsing has started
                    app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_STARTED);
                
                    // The next state will be undirected advertising with no pairing
                    return ADV_UNDIRECTED_NO_PAIRING;
                }
                else {
                    // Notify the main fsm that directed advertsing has ended
                    app_adv_fsm_set_and_send_notification(ADV_FSM_DIR_ADV_ENDED);
                    return ADV_IDLE;
                }
            }
        
        // If a start or stop advertise event occurs while in directed advertising
        // we should enqueue that event and wait for the directed advertising
        // to complete since we cannot stop directed advertising but just wait for it to end
        case START_ADV:
        case START_ADV_LIM:
        case START_ADV_NO_PAIR:
        case START_ADV_DIR:
        case STOP_ADV:
    #ifdef HAS_SPECIAL_ADVERTISING
        case START_ADV_SPECIAL:
    #endif
            
            // Backup the pending event 
            adv_pending_event = adv_fsm_evt;
            
            // Go to adv con fsm event pending and wait until directed advertising ends completely (dir_adv_completed_evt)
            // in order to handle the pending advertising event
            return ADV_FSM_EVENT_PENDING;
        
        default:
            ASSERT_ERROR(0);
            return ADV_IDLE;    
    }
}         


/**
 ****************************************************************************************
 * \brief Event handler when being in Idle state (ADV_IDLE)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_con_fsm_evt_when_idle(adv_fsm_events_t adv_fsm_evt)
{
     switch(adv_fsm_evt)
     {
         case START_ADV:
                 // Start undirected advertising
                 dbg_puts(DBG_ADV_LVL, "(Pairing Allowed)");
                 start_adv_undirected(adv_und_filter_policy, adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED));
         
                 // Notify the main fsm that undirected advertising has started
                 app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_STARTED);
                 return ADV_UNDIRECTED;
         
         case START_ADV_DIR:
                 // Start directed advertising again to the previous address
                 dbg_printf(DBG_ADV_LVL, "(Advertising to %02x:%02x:%02x:%02x:%02x:%02x)",
                    app_adv_bd_address.addr[5], app_adv_bd_address.addr[4],
                    app_adv_bd_address.addr[3], app_adv_bd_address.addr[2],
                    app_adv_bd_address.addr[1], app_adv_bd_address.addr[0]);
                 
                 start_adv_directed(&app_adv_bd_address);
     
                 // Notify the main fsm that undirected advertising has started
                 app_adv_fsm_set_and_send_notification(ADV_FSM_DIR_ADV_STARTED);
                 return ADV_DIRECTED;
         
         case START_ADV_NO_PAIR:
                 // Start undirected advertising
                 start_adv_undirected(adv_und_filter_policy, adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_NO_PAIRING));
             
                 // Notify the main fsm that undirected advertising has started
                 app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_STARTED);
                 return ADV_UNDIRECTED_NO_PAIRING;
         
         case START_ADV_LIM:                        
                 // Start undirected limited advertising
                 start_adv_undirected(adv_und_filter_policy, adv_fsm_get_adv_settings_idx(ADV_UNDIRECTED_LIM));
         
                 // Notify the main fsm that undirected limited advertising has started
                 app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_LIM_STARTED);
                 return ADV_UNDIRECTED_LIM;
         
#ifdef HAS_SPECIAL_ADVERTISING         
         case START_ADV_SPECIAL:
                // Start undirected advertising for waking-up host
                start_adv_undirected_special();
                
                // Notify the main fsm that wake on ble advertising has started
                app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_SPECIAL_STARTED);
                return ADV_SPECIAL;
#endif         
         
         case STOP_ADV:
                 return ADV_IDLE;
         
         case ADV_NO_EVENT:
                 return ADV_IDLE;
         
         default:
                 ASSERT_ERROR(0);
                 return ADV_IDLE;
         
     }
}


/**
 ****************************************************************************************
 * \brief Event handler when being in Advertise Event Pending State (ADV_FSM_EVENT_PENDING)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_con_fsm_event_pending(adv_fsm_events_t adv_fsm_evt)
{
    adv_fsm_events_t temp_pending_evt;
    switch(adv_fsm_evt)
    {
        case UND_ADV_COMPLETED:
        case DIR_ADV_COMPLETED:   
                // Keep the pending event in order to clear it
                temp_pending_evt = adv_pending_event;
                
                // Clear it before calling the handling function
                adv_pending_event = ADV_NO_EVENT;
                
                
                if(adv_fsm_evt == DIR_ADV_COMPLETED) {
                    // Also notify the main fsm that undirected advertsing has ended
                    app_adv_fsm_set_and_send_notification(ADV_FSM_DIR_ADV_ENDED);
                }
                else {
                    // Also notify the main fsm that undirected advertsing has ended
                    app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_ENDED);
                }
                 
                // Now that advertising has completed, check and handle any pending events (if any)
                return adv_state_handle_con_fsm_evt_when_idle(temp_pending_evt);
        
        case START_ADV:
        case START_ADV_LIM:
        case START_ADV_NO_PAIR:
        case START_ADV_DIR:
        case STOP_ADV:
    #ifdef HAS_SPECIAL_ADVERTISING
        case START_ADV_SPECIAL:
    #endif
            adv_pending_event = adv_fsm_evt;
            return ADV_FSM_EVENT_PENDING;
        
        // If advertising has ended due an incoming connection then cancel any pending events
        case DIR_ADV_INTERRUPTED:
            adv_pending_event = ADV_NO_EVENT;
            // Also notify the main fsm that directed advertsing has ended
            app_adv_fsm_set_and_send_notification(ADV_FSM_DIR_ADV_ENDED);
            return ADV_IDLE;
        
        // If advertising has ended due an incoming connection then cancel any pending events
        case UND_ADV_INTERRUPTED:
            adv_pending_event = ADV_NO_EVENT;            
            // Also notify the main fsm that undirected advertsing has ended
            app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_ENDED);
            return ADV_IDLE;
        
        default:
                return ADV_FSM_EVENT_PENDING;
    }
}


#ifdef HAS_SPECIAL_ADVERTISING
/**
 ****************************************************************************************
 * \brief Event handler when being in Special Undirected Advertising State(ADV_SPECIAL)
 *
 * \param[in] adv_fsm_evt  The incoming event
 *
 * \return adv_states The resulting advertising state after handling the incoming event
 ****************************************************************************************
 */
static adv_states adv_state_handle_special(adv_fsm_events_t adv_fsm_evt)
{
    switch(adv_fsm_evt)
    {
        case UND_ADV_COMPLETED:
                // Else notify the main fsm that undirected advertsing has ended
                app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_SPECIAL_ENDED);
                
                // And go to ADV_IDLE state
                return ADV_IDLE;
        
        // A connection request interrupted the undirected advertising
        case UND_ADV_INTERRUPTED:
                // Stop/Clear the undirected advertising timer, 
                // since advertsing has been interrupted
                port_timer_clear(APP_ADV_FSM_TIMER, BLE_TASK);   
            
                // Also notify the main fsm that undirected advertsing has ended
                app_adv_fsm_set_and_send_notification(ADV_FSM_UND_ADV_SPECIAL_ENDED);
                
                // Set the advertising state to IDLE
                return ADV_IDLE;   
        
        default:
                return adv_state_handle_undirected_common(adv_fsm_evt);
    }
    
}
#endif



/************************************************************************************
 ********************* Advertise FSM Core Event Handler *****************************
 ************************************************************************************/


void app_adv_fsm_process_evt(adv_fsm_events_t adv_fsm_evt)
{    
    dbg_printf(DBG_ADV_LVL, "@@ ADV evt:%s", adv_events_names[adv_fsm_evt]);
    dbg_printf(DBG_ADV_LVL, " @ %s", adv_state_names[current_adv_state]);
    
    switch(current_adv_state)
    {
        case ADV_DIRECTED:
            current_adv_state = adv_state_handle_directed(adv_fsm_evt);
            break;
        
        case ADV_UNDIRECTED_SLOW:
            current_adv_state = adv_state_handle_undirected_slow(adv_fsm_evt);
            break;
        
        case ADV_UNDIRECTED:
            current_adv_state = adv_state_handle_undirected(adv_fsm_evt);
            break;
        
        case ADV_UNDIRECTED_LIM:
            current_adv_state = adv_state_handle_undirected_lim(adv_fsm_evt);
            break;
        
        case ADV_UNDIRECTED_NO_PAIRING:
            current_adv_state = adv_state_handle_undirected_no_pair(adv_fsm_evt);
            break;
        
        case ADV_IDLE:
            current_adv_state = adv_state_handle_con_fsm_evt_when_idle(adv_fsm_evt);
            break;
        
        case ADV_FSM_EVENT_PENDING:
            current_adv_state = adv_state_handle_con_fsm_event_pending(adv_fsm_evt);
            break;
        
#ifdef HAS_SPECIAL_ADVERTISING        
        case ADV_SPECIAL:
            current_adv_state = adv_state_handle_special(adv_fsm_evt);
            break;
#endif
        
        default:
            ASSERT_ERROR(0);
            break;
    }
    
    dbg_printf(DBG_ADV_LVL, " -> %s\r\n", adv_state_names[current_adv_state]);
    // If there is a pending notification to be sent, send it now
    // (if a notification callback is registered)
    if(adv_fsm_config.app_adv_notification_callback && adv_fsm_next_notification != ADV_FSM_NO_NOTIFICATION) {
             adv_fsm_config.app_adv_notification_callback((uint8_t)adv_fsm_next_notification);
    }
    
    // Clear the notification flag
    adv_fsm_next_notification = ADV_FSM_NO_NOTIFICATION;
}



/************************************************************************************
 ********************* Advertise FSM API functions **********************************
 ************************************************************************************/


void app_adv_fsm_adv_stop()
{
    app_adv_fsm_process_evt(STOP_ADV);
}


void app_adv_fsm_und_start_no_pair(enum adv_filter_policy undFilterP)
{
    adv_und_filter_policy = undFilterP;
    app_adv_fsm_process_evt(START_ADV_NO_PAIR);  
}


void app_adv_fsm_dir_start(struct bd_addr *address)
{
    // Clear the repeat counter
    directed_adv_repeat_counter = 0;
    
    // Copy the given peer address, else keep the previous one
    if(address) {
        memcpy(&app_adv_bd_address, address, sizeof(struct bd_addr));
    }
    else {
        // Assert a warning if the given address is NULL
        ASSERT_WARNING(0);
    }
    
    app_adv_fsm_process_evt(START_ADV_DIR);
}


void app_adv_fsm_und_start(enum adv_filter_policy undFilterP)
{
    adv_und_filter_policy = undFilterP;
    app_adv_fsm_process_evt(START_ADV);
}


void app_adv_fsm_und_start_lim(enum adv_filter_policy undFilterP)
{
    adv_und_filter_policy = undFilterP;
    app_adv_fsm_process_evt(START_ADV_LIM);    
}


#ifdef HAS_SPECIAL_ADVERTISING
void app_adv_fsm_start_special()
{
    app_adv_fsm_process_evt(START_ADV_SPECIAL);    
}


void app_adv_fsm_set_special_adv_data(uint8_t *special_data, uint8_t special_data_len)
{
    ASSERT_ERROR(special_data_len <= (ADV_DATA_LEN - 3));
    memcpy(adv_special_data, special_data, special_data_len);
    adv_special_data_length = special_data_len;
}
#endif


void app_adv_fsm_init()
{
#ifdef AUTO_APPEND_DEVICE_NAME_IN_ADV_DATA          
        app_adv_fsm_set_adv_data();         // Prepare Advertising data
#endif  
        
        // Set the adv state to IDLE
        current_adv_state = ADV_IDLE;
    
        // Clear the pending event variable (ADV_NO_EVENT)
        adv_pending_event = ADV_NO_EVENT;
}


#endif

/**
 * \}
 * \}
 * \}
 */
