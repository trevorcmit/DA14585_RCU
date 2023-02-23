/**
 *****************************************************************************************
 *
 * \file app_con_fsm.c
 *
 * \brief Connection FSM module source file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 *****************************************************************************************
 */

/**
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup APP_CON_FSM
 *
 * \brief Connection FSM application file
 * \{
 ****************************************************************************************	 	 
 */


#ifdef HAS_CONNECTION_FSM

#include "app_con_fsm.h"
#include <app_con_fsm_config.h>
#include <port_con_fsm.h>
#include "port_platform.h"
#include <port_multi_bond.h>
#include "port_security.h"
#include "app_white_list.h"
#include "app_adv_fsm.h"
#include "port_timer.h"

#define BLE_ASYNC_TASK_SWITCH_HOST                  0x01
#define BLE_ASYNC_TASK_START_PAIR                   0x02
#define BLE_ASYNC_TASK_ENTERING_PASSCODE            0x04
#define BLE_ASYNC_TASK_SEND_ADV_COMPLETED_EVT       0x08
#define BLE_ASYNC_TASK_SEND_SPECIAL_ADV_ENDED_EVT   0x10


#define ASYNC_TASK_WRITE_BONDING_DATA       0x01
#define ASYNC_TASK_ERASE_BONDING_DATA       0x02



typedef enum
{
    APP_CON_FSM_NO_SUBTIMER,
    APP_CON_FSM_PASSCODE_SUBTIMER,
    APP_CON_FSM_PARAM_UPD_SUBTIMER,
    APP_CON_FSM_POWEROFF_SUBTIMER,
    APP_CON_FSM_ALL_SUBTIMERS
}con_fsm_timer_type_t;


#if DEVELOPMENT_DEBUG
    #define FSM_LOG_DEPTH 8
    
    struct fsm_log_ {
            enum main_fsm_events evt;
            main_fsm_states state;
            uint16_t time;
    } fsm_log[FSM_LOG_DEPTH] __PORT_RETAINED;
    
    int fsm_log_ptr __PORT_RETAINED;
#endif  //DEVELOPMENT_DEBUG

typedef enum
{
    APP_CON_FSM_ADV_DEFAULT,
    APP_CON_FSM_ADV_NO_PAIRING,
    APP_CON_FSM_ADV_RECONNECT,
    #ifdef HAS_SPECIAL_ADVERTISING
    APP_CON_FSM_ADV_SPECIAL_ADV
    #endif
}app_con_fsm_adv_type_t;

main_fsm_states current_fsm_state       __PORT_RETAINED;

app_con_fsm_adv_type_t   app_con_fsm_last_queued_adv    __PORT_RETAINED;

uint8_t ble_async_tasks                 __PORT_RETAINED;
uint8_t non_ble_async_tasks             __PORT_RETAINED;

bool nv_rom_is_read                     __PORT_RETAINED;

bool conn_upd_pending                   __PORT_RETAINED;

enum multi_bond_host_rejection multi_bond_enabled __PORT_RETAINED;


#ifdef FORCE_CONNECT_TO_HOST_ON
        uint8_t force_next_store_entry  __PORT_RETAINED;
        uint8_t fallback_peer_entry     __PORT_RETAINED;
#endif


bool directed_adv_permitted                         __PORT_RETAINED;
bool app_con_fsm_advertising                        __PORT_RETAINED;
con_fsm_timer_type_t app_con_fsm_running_timer_type __PORT_RETAINED;

extern struct bd_addr dev_bdaddr;
extern struct bd_addr alt_dev_bdaddr;

__attribute__((unused)) static const char state_names[][23] = {
    "IDLE", 
    "ADVERTISING_ST", 
    "CONNECTED", 
    "CONNECTION_IN_PROGRESS", 
    "CONNECTED_PAIRING",
    "DISCONNECTED_INIT",
    "POWEROFF_ST", 
    "WAIT_DC_AFTER_POWEROFF"
};

__attribute__((unused)) static const char events_names[][21] = {
    "INIT_EVT", 
    "USER_EVT", 
    "ADV_COMPLETED_EVT", 
    "PAIRING_REQ_EVT", 
    "CONN_REQ_EVT",
    "CONN_CMP_EVT", 
    "CONN_IRK_EXCH",
    "CONN_CANCELD_EVT", 
    "DISCONNECT_CMP_EVT", 
    "CONN_UPD_RESP_EVT",
    "PASSKEY_ENTERED", 
    "START_PAIRING_EVT", 
    "SWITCH_EVT", 
    "NEW_HOST_EVT", 
    "ALT_PAIR_TMR_EXP_EVT",
    "POWEROFF_EVT",
    "SHUTDOWN_EVT",
    "POWEROFF_TO_EVT",
    "PASSCODE_TO_EVT",
    "PARAM_UPD_TO_EVT",
    #ifdef HAS_SPECIAL_ADVERTISING
    "SPECIAL_ADV_START_EVT",
    "SPECIAL_ADV_CMP_EVT"
    #endif
};

#define CON_FSM_WARNINGS


/**
 ****************************************************************************************
 * \brief       App Con Fsm callback indication API
 *
 * \details     Used to send an indication from the app_con_fsm to the user application
 *
 * \param[in]   type App Con Fsm indication type
 ****************************************************************************************
 */
static inline void app_con_fsm_call_callback(enum con_fsm_state_update_callback_type type)
{
        if (con_fsm_params.state_update_callback) {
                (*con_fsm_params.state_update_callback)(type);
        }
}


/**
 ****************************************************************************************
 * \brief       Start a general purpose subtimer.
 *
 * \details     The APP_CON_FSM_TIMER is used to run three separate timer types
 *              This means that only one subtimer can be active at a time
 *
 * \param[in]       timerType The desired subtimer to start
 ****************************************************************************************
 */
static void app_con_fsm_start_gp_timer(con_fsm_timer_type_t timerType)
{
    // Error if we are trying to start a timer while another one is running
    ASSERT_ERROR(app_con_fsm_running_timer_type == APP_CON_FSM_NO_SUBTIMER);

    // Before starting a new timer, stop/clear the previous one
    if((app_con_fsm_running_timer_type!=APP_CON_FSM_NO_SUBTIMER)) {
        port_timer_clear(APP_CON_FSM_TIMER, BLE_TASK);
    }

    switch(timerType)
    {
        case APP_CON_FSM_PASSCODE_SUBTIMER:
                    port_timer_set(APP_CON_FSM_TIMER, BLE_TASK, con_fsm_params.passcode_timeout);
            break;

        case APP_CON_FSM_PARAM_UPD_SUBTIMER:
                    port_timer_set(APP_CON_FSM_TIMER, BLE_TASK, con_fsm_params.time_to_request_param_upd);
            break;
        
       
        case APP_CON_FSM_POWEROFF_SUBTIMER:
                    port_timer_set(APP_CON_FSM_TIMER, BLE_TASK, con_fsm_params.notification_timeout);
            break;
       
        
        default:
            ASSERT_ERROR(0);
            break;
    }
    
    // Update the currenlty running timer type
    app_con_fsm_running_timer_type = timerType;
}


/**
 ****************************************************************************************
 * \brief   Requests update of connection params
 *          After connection and, optionally, pairing is completed, this function 
 *          is called to (optionally) modify the connection parameters.
 ****************************************************************************************
 */
static void app_con_fsm_param_update(void)
{
        if (con_fsm_params.use_pref_conn_params) {
                if (!con_fsm_params.has_mitm) {
                        app_con_fsm_call_callback(INDICATE_START_PARAM_UPDATE);
                }
                //Set a timer to update connection params
                app_con_fsm_start_gp_timer(APP_CON_FSM_PARAM_UPD_SUBTIMER);
                dbg_puts(DBG_CONN_LVL, "[Set param update timer]");
                conn_upd_pending = true;
        }        
}


/**
 ****************************************************************************************
 * \brief      Starts the address resolving
 ****************************************************************************************
 */
static void app_con_fsm_resolve_address(void)
{
        if(app_con_fsm_funcs.con_fsm_resolve_address) {
                app_con_fsm_funcs.con_fsm_resolve_address();
        }
        else {
                ASSERT_ERROR(0);
        }   
}


/**
 ****************************************************************************************
 * \brief      Sends a disconnect request
 ****************************************************************************************
 */
static void app_con_fsm_disconnect(void)
{
        if(app_con_fsm_funcs.con_fsm_disconnect) {
                app_con_fsm_funcs.con_fsm_disconnect();
        }
        else {
                ASSERT_ERROR(0);
        }
}


/**
 ****************************************************************************************
 * \brief      Starts the security procedure during a connection
 ****************************************************************************************
 */
static void app_con_fsm_start_security(void)
{
        if(app_con_fsm_funcs.con_fsm_start_security) {
                app_con_fsm_funcs.con_fsm_start_security();
        }
        else {
                ASSERT_ERROR(0);
        }   
}


/**
 ****************************************************************************************
 * \brief      Sends a connection confirmation
 *
 * \param[in]  auth the authorization type as seen in the enum gap_auth
 *
 * \return     uint8_t The peer's address type
 ****************************************************************************************
 */
static void app_con_fsm_connect_confirm(uint8_t auth)
{
        if(app_con_fsm_funcs.con_fsm_connect_confirm) {
                app_con_fsm_funcs.con_fsm_connect_confirm(auth);
        }
        else {
                ASSERT_ERROR(0);
        }   
}


#ifdef CON_FSM_WARNINGS
/**
 ****************************************************************************************
 * \brief      
 *
 * \param[in]  thystate
 * \param[in]  thyevent
 ****************************************************************************************
 */
static void dbg_warning_con_fsm(main_fsm_states thystate, enum main_fsm_events thyevent)
{
    dbg_printf(DBG_CONN_LVL, "[UNEXP EVT %s ON %s state]", events_names[thyevent], state_names[thystate] );
}
#endif


/**
 ****************************************************************************************
 * \brief      Asynchronous general purpose (non-ble) task setup function
 *
 * \details    Used to set up a task to be executed asynchronously
 *
 * \param[in]      taskbm Specifies which task to set up
 *             ASYNC_TASK_WRITE_BONDING_DATA: Write/Save the current bonding data
 *             ASYNC_TASK_ERASE_BONDING_DATA: Erase All bonding data
 ****************************************************************************************
 */
static void app_con_fsm_set_async_task(uint8_t taskbm)
{
    non_ble_async_tasks |= taskbm;
}


/**
 ****************************************************************************************
 * \brief      Asynchronous BLE task setup function
 *
 * \details    Sets up a BLE task to be executed asynchronously
 *
 * \param[in]      taskbm Specifies which task to set up
 *             BLE_ASYNC_TASK_SWITCH_HOST: Execute a switch host task (used when 
 *             multi-bonding is enabled for host switching)
 *             
 *             BLE_ASYNC_TASK_START_PAIR: Execute a Start Pairing task (search for a new 
 *             host to pair)
 *
 *             BLE_ASYNC_TASK_ENTERING_PASSCODE: Task to asynchronously inform the app_con_fsm
 *             about password entering (used when mitm is enabled)
 ****************************************************************************************
 */
static void app_con_fsm_set_ble_async_task(uint8_t taskbm)
{
    ble_async_tasks |= taskbm;
}


void app_con_fsm_gp_timer_expired(void)
{
    
    switch(app_con_fsm_running_timer_type)
    {
        case APP_CON_FSM_PASSCODE_SUBTIMER:
            app_con_fsm_state_update(PASSCODE_TIMEOUT_EVT);
            break;
        
        case APP_CON_FSM_PARAM_UPD_SUBTIMER:
            app_con_fsm_state_update(PARAM_UPD_TIMEOUT_EVT);
            break;

        case APP_CON_FSM_POWEROFF_SUBTIMER:
            app_con_fsm_state_update(POWEROFF_TIMEOUT_EVT);
            break;
        
        default:
            ASSERT_ERROR(0);
            break;
        
    }
    
    app_con_fsm_running_timer_type = APP_CON_FSM_NO_SUBTIMER;
    
}


/**
 ****************************************************************************************
 * \brief       Stop a general purpose subtimer.
 *
 * \param[in]       timerType The desired subtimer to stop
 ****************************************************************************************
 */
static void app_con_fsm_stop_gp_timer(con_fsm_timer_type_t timerType)
{
    // If the timer is already stopped then return immediately
    if(app_con_fsm_running_timer_type == APP_CON_FSM_NO_SUBTIMER) {
        return;
    }

    // Error if we are trying to stop another timer
    ASSERT_ERROR(timerType ==APP_CON_FSM_ALL_SUBTIMERS || app_con_fsm_running_timer_type==timerType );

    switch(timerType)
    {

        case APP_CON_FSM_ALL_SUBTIMERS:
        case APP_CON_FSM_PASSCODE_SUBTIMER:
        case APP_CON_FSM_PARAM_UPD_SUBTIMER:
        case APP_CON_FSM_POWEROFF_SUBTIMER:
            port_timer_clear(APP_CON_FSM_TIMER, BLE_TASK);
            break;  
            
        
        default:
            ASSERT_ERROR(0);
            break;
        
    }
    
    app_con_fsm_running_timer_type = APP_CON_FSM_NO_SUBTIMER;
}


/**
 ****************************************************************************************
 * \brief       Disconnect for a Host switch.
 *
 * \details     Called on a Host switch to disconnect from the current Host and
 *              start the "switching period" of con_fsm_params.alt_pair_disconn_time
 ****************************************************************************************
 */
static void app_alt_pair_disconnect(void)
{
        if (port_alt_pair_is_bonded() && (multi_bond_enabled == MULTI_BOND_REJECT_LAST ||
                (multi_bond_enabled == MULTI_BOND_REJECT_ALL_KNOWN
                #ifdef FORCE_CONNECT_TO_HOST_ON
                        && fallback_peer_entry != MAX_BOND_PEER
                #endif
                ))) {
                port_timer_set(APP_CON_FSM_ALT_PAIR_TIMER, BLE_TASK, con_fsm_params.alt_pair_disconn_time);
        }
        else {
                port_timer_clear(APP_CON_FSM_ALT_PAIR_TIMER, BLE_TASK);
        }
            
#ifdef FORCE_CONNECT_TO_HOST_ON
        fallback_peer_entry = port_alt_pair_get_active_index();
#endif
        app_con_fsm_disconnect();
}


/**
 ****************************************************************************************
 * \brief       Sends an indication from the app_con_fsm to the user application that
 *              the con fsm is entering IDLE state
 ****************************************************************************************
 */

static void indicate_disconnecting_idle()
{
    app_con_fsm_call_callback(INDICATE_REINITIALIZE);
    app_con_fsm_call_callback(INDICATE_IDLE);
}


/**
 ****************************************************************************************
 * \brief Changes/Enqueues a desired advertising type. Used in cases where the user wants to
 *        enqueue an advertising state, after an event occurs
 *
 * \param[in] advType The advertising type to be enqueued
 ****************************************************************************************
 */
static void app_con_fsm_enqueue_adv_type(app_con_fsm_adv_type_t advType)
{
    app_con_fsm_last_queued_adv = advType;
}


/**
 ****************************************************************************************
 * \brief Returns the last enqueued advertising type
 *
 * \return The last enqueued advertising type
 ****************************************************************************************
 */
static app_con_fsm_adv_type_t app_con_fsm_get_last_queued_adv_type(void)
{
    return app_con_fsm_last_queued_adv;
}


/**
 ****************************************************************************************
 * \brief Helper function that starts a desired type of advertising
 *
 * \param[in] advType The advertising type we want to start
 ****************************************************************************************
 */
static void app_con_fsm_start_advertise(app_con_fsm_adv_type_t advType)
{
    enum adv_filter_policy undFilterP = ADV_ALLOW_SCAN_ANY_CON_ANY;

#ifdef HAS_SPECIAL_ADVERTISING
    if(advType == APP_CON_FSM_ADV_SPECIAL_ADV && port_alt_pair_is_bonded()) {
            app_adv_fsm_start_special();
            // Dont backup the special advertising (dont update the last queued adv type)
            return;
    }
#endif
            
    app_con_fsm_last_queued_adv = advType;    
    
    if(port_alt_pair_is_bonded() == false || advType == APP_CON_FSM_ADV_DEFAULT) {
            app_adv_fsm_und_start(undFilterP);
            app_con_fsm_last_queued_adv = APP_CON_FSM_ADV_DEFAULT;
    }
    else {
            
            if(advType == APP_CON_FSM_ADV_NO_PAIRING) {

                    if (con_fsm_params.has_white_list && app_white_list_written()) {
                            undFilterP = ADV_ALLOW_SCAN_WLST_CON_WLST;
                    }

                    app_adv_fsm_und_start_no_pair(undFilterP);
                    return;
            }

            if(directed_adv_permitted && advType == APP_CON_FSM_ADV_RECONNECT) {
                app_adv_fsm_dir_start(port_alt_pair_get_addr());
            } 
            else {
                app_adv_fsm_und_start_lim(undFilterP);
            }
    }
}


/**
 ****************************************************************************************
 * \brief Helper function that stops advertising (if active)
 ****************************************************************************************
 */
static void app_con_fsm_stop_advertise()
{
        app_adv_fsm_adv_stop();
}

/**
 ****************************************************************************************
 * \brief Checks if host can be accepted according to the given RSSI threshold
 * \return bool true if host can be accepted, false otherwise
 ****************************************************************************************
 */
static bool app_con_fsm_smart_pairing_is_host_accepted(void)
{
        if(con_fsm_params.has_smart_rssi_pairing == true && app_con_fsm_funcs.con_fsm_get_rssi) {
                int8_t rssi = app_con_fsm_funcs.con_fsm_get_rssi();
            dbg_printf(DBG_CONN_LVL, "\r\n### RSSI: %d, threshold : %d ###\r\n", rssi, con_fsm_params.smart_pairing_rssi_threshold);
                return (rssi >= con_fsm_params.smart_pairing_rssi_threshold); 
        }
        else {
                return true;
        }
}

/**
 ****************************************************************************************
 * \brief       Idle state common event handling function
 *
 * \details 	Checks based on the current pairing information if directed advertising 
 *              is permitted
 ****************************************************************************************
 */
static void idle_state_common_handler()
{
        if (con_fsm_params.has_nv_rom) {
                if (nv_rom_is_read == false) {
                        directed_adv_permitted = false;
                }
                
                if ((directed_adv_permitted == false) && (nv_rom_is_read == false)) {
                        port_alt_pair_read_status();
                        port_alt_pair_reset_active_peer_pos();
                        
                    if(port_alt_pair_load_last_used()) {
                            directed_adv_permitted = true;
                    }
                    else {
                            directed_adv_permitted = false;
                    }

                    if (directed_adv_permitted) {
                            // Has the last connected host public /*or static random address*/?
                            if (ADDR_PUBLIC != port_alt_pair_get_addr_type()) {
                                
                                // If the host does not have public address set
                                // the directed_adv_permitted flag to false
                                directed_adv_permitted = false; // will be set to true if Privacy is enabled and the Reconnection address is written
                            }    
                    }
                    else {
                            port_alt_pair_set_default_info();
                    }
                    
                    nv_rom_is_read = true;
                }
        }
        else {
                if (!directed_adv_permitted) {
                        port_alt_pair_set_default_info();
                }
        }
}

/**
 ****************************************************************************************
 * \brief IDLE state handler
 *
 * \details Handles any incoming events while the connection FSM is in IDLE state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static main_fsm_states handle_idle_state(enum main_fsm_events evt)
{
        // Each time we enter idle state
        idle_state_common_handler();

        switch (evt) {

            case INIT_EVT:
            #ifdef NORMALLY_CONNECTABLE
                    // In a normally connectable environment, device should always start advertise
                    // trying to reconnect with its last bonded host (if any)
                    app_con_fsm_start_advertise(APP_CON_FSM_ADV_RECONNECT);
                    
                    // Notify the application that we are exiting IDLEness
                    app_con_fsm_call_callback(INDICATE_EXIT_IDLE);

                    /* Decide the next state */
                    return ADVERTISING_ST;
            #else
                    // Just indicate we are in IDLE state
                    app_con_fsm_call_callback(INDICATE_IDLE);

                    // No change in state
                    return IDLE_ST;
            #endif                      
            
                    
            case USER_EVT:
			case SWITCH_EVT:
                    // Indicate to the application that we are exiting IDLEness 
                    app_con_fsm_call_callback(INDICATE_EXIT_IDLE);
            
                    // Start advertising and try to reconnect (if there is a valid bonding)
                    app_con_fsm_start_advertise(APP_CON_FSM_ADV_RECONNECT);

                    // Go to advertising state
                    return ADVERTISING_ST;
                    
                    
            case START_PAIRING_EVT:
                    // Indicate to the application that we are exiting IDLEness 
                    app_con_fsm_call_callback(INDICATE_EXIT_IDLE);
                    
                    // Start default advertising to pair with new hosts
                    app_con_fsm_start_advertise(APP_CON_FSM_ADV_DEFAULT);

                    // Go to advertising state
                    return ADVERTISING_ST;

            
            case ADV_COMPLETED_EVT:
                    // Start advertising as lastly enqueued
                    app_con_fsm_start_advertise(app_con_fsm_get_last_queued_adv_type());
            
                    // Go to advertising state
                    return ADVERTISING_ST;
            
#ifdef HAS_SPECIAL_ADVERTISING
            case SPECIAL_ADV_START_EVT:
                    app_con_fsm_start_advertise(APP_CON_FSM_ADV_SPECIAL_ADV);
                    return ADVERTISING_ST;
#endif
            
            case POWEROFF_EVT:
                    // Ignore POWEROFF_EVT, we are already IDLE
                    return IDLE_ST;
            
            default:
                    #ifdef CON_FSM_WARNINGS
                        dbg_warning_con_fsm(current_fsm_state, evt);
                    #else
                        ASSERT_WARNING(0);
                    #endif
                    return IDLE_ST;
        }
}


/**
 ****************************************************************************************
 * \brief ADVERTISING_ST state handler 
 *
 * \details Handles any incoming events while the connection FSM is in ADVERTISING state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static main_fsm_states handle_advertising_state (enum main_fsm_events evt)
{
    switch (evt)
    {
        case USER_EVT:
        case SWITCH_EVT:
                // A switch event occurred, so start advertising for reconnection
                // to the host we are going to switch to
                app_con_fsm_start_advertise(APP_CON_FSM_ADV_RECONNECT);

                // Stay in advertising state
                return ADVERTISING_ST; 

        case START_PAIRING_EVT:
                // A pairing event has occurred, so start default advertising
                // in order to connect/pair with new hosts
                app_con_fsm_start_advertise(APP_CON_FSM_ADV_DEFAULT);

                // Stay in advertising state
                return ADVERTISING_ST;
        
        case ADV_COMPLETED_EVT:
                // Advertising has completed without any connection requests
                // Indicate "idleness" since nothing happened during advertising
                app_con_fsm_call_callback(INDICATE_IDLE);
                
                // Go to IDLE state
                return IDLE_ST;
                    
        case POWEROFF_EVT:
                // Stop advertising immediately because we are powering off!
                app_con_fsm_stop_advertise();
                                
                // Stay in advertising state and wait for the 
                // advertising to complete (ADV_COMPLETED_EVT)
                return ADVERTISING_ST;                

        case CONN_REQ_EVT:
                // Advertising was interrupted due to a connection request
        
                // Go to connection in progress state in order 
                // to handle the connection request
                return CONNECTION_IN_PROGRESS_ST;            

        case ALT_PAIR_TIMER_EXP_EVT:
                // Alt Pair Timer has expired, so 
        
                // switch to the last/fallback peer we were previously connected            
            #ifdef FORCE_CONNECT_TO_HOST_ON
                // Switch to that host
                if (fallback_peer_entry != MAX_BOND_PEER) {
                        app_con_fsm_switch_to_peer(fallback_peer_entry);
                        app_alt_pair_reset_force_next_store_entry();
                        fallback_peer_entry = MAX_BOND_PEER;
                }
            #endif

                // stop advertising
                app_con_fsm_stop_advertise();
                
                // Go to IDLE state
                return IDLE_ST;        
        
#ifdef HAS_SPECIAL_ADVERTISING                
        case SPECIAL_ADV_ENDED_EVT:
             // When special advertising has ended, start advertising for reconnection
             app_con_fsm_start_advertise(APP_CON_FSM_ADV_RECONNECT);
             return ADVERTISING_ST;
        
        
        case SPECIAL_ADV_START_EVT:
            // Start special advertising in the case of a SPECIAL_START_ADV_EVT
            app_con_fsm_start_advertise(APP_CON_FSM_ADV_SPECIAL_ADV);
            return ADVERTISING_ST;
#endif       
        
        default:
                #ifdef CON_FSM_WARNINGS
                        dbg_warning_con_fsm(current_fsm_state, evt);
                #else
                        ASSERT_WARNING(0);
                #endif
                return ADVERTISING_ST;

    }
  
}


/**
 ****************************************************************************************
 * \brief Connection in progress state common handler
 *
 * \details Common event handler while being in connection in progress state
 ****************************************************************************************
 */
static void conn_in_progr_common_evt(void)
{
        dbg_puts(DBG_CONN_LVL, "[(-) ENC timer]");
        port_timer_clear(APP_CON_FSM_ENC_TIMER, BLE_TASK); // Timer expire results in connection drop and is handled afterwards
}


/**
 ****************************************************************************************
 * \brief CONNECTION_IN_PROGRESS_ST state pairing request event handler
 *
 * \details While in CONNECTION_IN_PROGRESS_ST state this function handles any incoming
 * pairing request events (PAIRING_REQ_EVT) and checks if the request can be accepted 
 * and sends a pairing response accordingly
 *
 * \return true if pairing request was accepted, false otherwise
 ****************************************************************************************
 */
static bool connection_in_progress_is_pairing_req_accepted(void)
{
        // If the SMART RSSI PAIRING feature is enabled
        // then first check if the host can be accepted depending on its RSSI
        if(con_fsm_params.has_smart_rssi_pairing == true && 
           app_con_fsm_smart_pairing_is_host_accepted() == false) {
                dbg_puts(DBG_CONN_LVL, "\r\n##REJECTED due to RSSI threshold##\r\n");
                port_security_send_pairing_rsp(false);
                return false;
        }
                    
        if (app_con_fsm_get_last_queued_adv_type() != APP_CON_FSM_ADV_DEFAULT) {
                // if we are bonded to a host with random address and we were performing undirected advertising to that host
                // then reject pairing requests from other hosts
                dbg_puts(DBG_CONN_LVL, "    GAPC_PAIRING_REQ rejected due to ADV_BONDED\r\n");
                port_security_send_pairing_rsp(false);
                return false;
        }
        
        // Clear bond data of previous connection (if any)
        port_alt_pair_reset_bonding_data();
        port_alt_pair_set_default_info();
        port_alt_pair_reset_active_peer_pos();
        
        // Send a positive pairing response
        port_security_send_pairing_rsp(true);
        
        // If mitm is enabled and there is a defined timeout for the passcode
        // timer then start the passcode timer
        if (con_fsm_params.has_mitm && con_fsm_params.has_passcode_timeout) {
                dbg_puts(DBG_CONN_LVL, "[(+) passcode timer]");
                app_con_fsm_start_gp_timer(APP_CON_FSM_PASSCODE_SUBTIMER);
        }
        
        // entering "passcode" mode
        app_con_fsm_call_callback(INDICATE_REINITIALIZE);
        
        return true;
}


/**
 ****************************************************************************************
 * \brief Connection in progress state CONN_CMP_EVT handler
 *
 * \details While in CONNECTION_IN_PROGRESS_ST this function handles any incoming connection
 * completion events (CONN_CMP_EVT) and sends a CONNECTED indication to the user application
 *
 ****************************************************************************************
 */
static void connection_in_progress_conn_cmp_evt(void)
{
    if (con_fsm_params.has_multi_bond) {
        multi_bond_enabled = MULTI_BOND_REJECT_NONE;
        port_timer_clear(APP_CON_FSM_ALT_PAIR_TIMER, BLE_TASK);
    }
    
    app_con_fsm_param_update();
    
    // no need to store anything to the NV memory
    if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {
            int8_t entry = port_alt_pair_get_active_index();
            
            if (entry == MAX_BOND_PEER) { // If this is a new entry
                    // Find the place where it will be stored
                    entry = port_multi_bond_get_entry_to_delete();
            }
            app_white_list_add_host(app_con_fsm_get_peer_addr_type(), app_con_fsm_get_peer_addr(), entry);
    }
    
    // has the host public /* or static random address */?
    if (con_fsm_params.disable_bonding_data_storage == false
            && (ADDR_PUBLIC == app_con_fsm_get_peer_addr_type())
            /*|| ( (app_ble_get_peer_addr_type() == ADDR_RAND) && ((app_ble_get_peer_addr()->addr[5] & GAP_STATIC_ADDR) == GAP_STATIC_ADDR) )*/) {
            
            // Set the the directed_adv_permitted flag to true in order to do    
            // Directed Advertising to this host if connection is lost
            directed_adv_permitted = true; 
    }
    else {
            // Alternatively set the flag to false
            // Will be set to true if Privacy is enabled and the 
            // Reconnection address is written
            directed_adv_permitted = false; 
    }
    
    // Indicate to the application that we are connected
    app_con_fsm_call_callback(INDICATE_CONNECTED);
    
}


/**
 ****************************************************************************************
 * \brief CONNECTION_IN_PROGRESS_ST state handler 
 *
 * \details Handles any incoming events while the connection FSM is in 
 * CONNECTION_IN_PROGRESS_ST state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static main_fsm_states handle_connection_in_progress_state(enum main_fsm_events evt)
{
        if(evt != ADV_COMPLETED_EVT && evt!= CONN_UPD_RESP_EVT
#ifdef HAS_SPECIAL_ADVERTISING
        && evt!=SPECIAL_ADV_ENDED_EVT
#endif
        ) {
                conn_in_progr_common_evt();
        }

        switch(evt) {
        case ALT_PAIR_TIMER_EXP_EVT:
                // While a connection was in progress the Alt Pair Timer expired so
                
                // Set the device to switch to the previously connected/fallback host
            #ifdef FORCE_CONNECT_TO_HOST_ON
                if (fallback_peer_entry != MAX_BOND_PEER) {
                        app_con_fsm_switch_to_peer(fallback_peer_entry);
                        app_alt_pair_reset_force_next_store_entry();
                        fallback_peer_entry = MAX_BOND_PEER;
                }
            #endif
                    
                // Disconnect - End the current connection in progress
                app_con_fsm_disconnect();  
                
                // Enqueue advertising with no pairing
                app_con_fsm_enqueue_adv_type(APP_CON_FSM_ADV_NO_PAIRING);
                                
                // Go to disconnected init state to wait for disconnection completion
                return DISCONNECTED_INIT_ST;        
           
                
        case PAIRING_REQ_EVT:
                // If pairing request is accepted go to CONNECTED_PAIRING_ST state
                if(connection_in_progress_is_pairing_req_accepted()) {
                        return CONNECTED_PAIRING_ST;
                }
                
                // Else remain in the same state
                return CONNECTION_IN_PROGRESS_ST;
          
                
        case NEW_HOST_EVT:
                // Indicate to the application that it needs to reinitialize its settings
                // We have a new challenger!
                app_con_fsm_call_callback(INDICATE_REINITIALIZE);
        
                // No state change
                return CONNECTION_IN_PROGRESS_ST;
        
        
        case CONN_CANCELLED_EVT:
                // The connection was cancelled so disconnect
                app_con_fsm_disconnect();     

                // Go to disconnected init state to wait 
                // for disconnection completion event
                return DISCONNECTED_INIT_ST;

        
        case CONN_CMP_EVT:
                // Handle the Connection Completion Event
                connection_in_progress_conn_cmp_evt();

                // Go to connected state, since the connection has completed!
                return CONNECTED_ST;        
            
        
        case DISCONNECT_CMP_EVT:
                // A disconnection has occurred, so continue advertising with the same type
                app_con_fsm_start_advertise(app_con_fsm_get_last_queued_adv_type());
        
                 // Go to advertising state
                return ADVERTISING_ST;         
        
        
        case POWEROFF_EVT:
                // A power off event has occurred
                
                // Close the current connection / Disconnect
                dbg_puts(DBG_CONN_LVL, "[Disconnect due to power off]");
                app_con_fsm_disconnect();
        
                // Start the Power Off subtimer and wait for it to expire at the POWEROFF_ST(POWEROFF_TIMEOUT_EVT)
                app_con_fsm_start_gp_timer(APP_CON_FSM_POWEROFF_SUBTIMER);
        
                // Go to Poweroff State
                return POWEROFF_ST;
        
        
        case START_PAIRING_EVT:
                // A start pairing event has occurred
                dbg_puts(DBG_CONN_LVL, "Disconnect due to start pairing\r\n");
        
                // If multi-bond feature is enabled, disconnect using the alt-pair timer
                if(con_fsm_params.has_multi_bond == true)
                {
                    // Disconnect - End the current connection in progress
                    app_alt_pair_disconnect();
                }
                // Else use the simple disconnection function    
                else
                {
                    // Disconnect - End the current connection in progress
                    app_con_fsm_disconnect();    
                }

                // Set/Enqueue the next adv type to be default
                app_con_fsm_enqueue_adv_type(APP_CON_FSM_ADV_DEFAULT);
        
                // Go to disconnected init state to wait for disconnection completion
                return DISCONNECTED_INIT_ST;                    
        
                
         case SWITCH_EVT:
                // A switch host event has occurred
         
                // If multi bond feature is enabled
                if(con_fsm_params.has_multi_bond == true)
                {
                    dbg_puts(DBG_CONN_LVL, "[Disconnect due to switching]");
                    
                    // Disconnect - End the current connection in progress
                    app_alt_pair_disconnect();
                    
                    // Set/Enqueue the next adv type to be reconnect
                    app_con_fsm_enqueue_adv_type(APP_CON_FSM_ADV_RECONNECT);
                    
                    // Go to disconnected init state to wait for disconnection completion
                    return DISCONNECTED_INIT_ST;
                }

                // Else stay in the same state
                return CONNECTION_IN_PROGRESS_ST;
                
                
         // Event ignore list
#ifdef HAS_SPECIAL_ADVERTISING
        case SPECIAL_ADV_ENDED_EVT:  
        case SPECIAL_ADV_START_EVT:             
#endif             
        case ADV_COMPLETED_EVT:
		case CONN_UPD_RESP_EVT:
                return CONNECTION_IN_PROGRESS_ST;
         
        default:
                #ifdef CON_FSM_WARNINGS
                        dbg_warning_con_fsm(current_fsm_state, evt);
                #else
                        ASSERT_WARNING(0);
                #endif
                return CONNECTION_IN_PROGRESS_ST;
        }
}


/**
 ****************************************************************************************
 * \brief CONNECTED_PAIRING_ST common event handler
 *
 * \details Handles common events while in CONNECTED_PAIRING_ST state 
 ****************************************************************************************
 */
static void connected_pairing_state_common_evt(void)
{
    if (con_fsm_params.has_mitm && con_fsm_params.has_passcode_timeout) {
            dbg_puts(DBG_CONN_LVL, "[(-) passcode timer]");
            app_con_fsm_stop_gp_timer(APP_CON_FSM_PASSCODE_SUBTIMER);
    }
}


/**
 ****************************************************************************************
 * \brief CONNECTED_PAIRING_ST pass key event handler
 *
 * \details Handles a PASSKEY_ENTERED event while in CONNECTED_PAIRING_ST state 
 ****************************************************************************************
 */
static void connected_pairing_state_passkey_entered_evt(void)
{
        if (con_fsm_params.has_passcode_timeout) {
                dbg_puts(DBG_CONN_LVL, "[(+) passcode timer]");
                app_con_fsm_start_gp_timer(APP_CON_FSM_PASSCODE_SUBTIMER);
        }    
}


/**
 ****************************************************************************************
 * \brief CONNECTED_PAIRING_ST connection completion event host checking
 *
 * \details Checks a connected host when being in CONNECTED_PAIRING_ST state  
 * and a CONN_CMP_EVT event occurs
 *
 * \return true if host is accepted, false otherwise
 ****************************************************************************************
 */
static bool connected_pairing_state_conn_cmp_evt_check_host(void)
{
        app_con_fsm_call_callback(INDICATE_PAIRING_COMPLETED);
    
        // We may reach this point after getting an LL_ENC_REQ from an unbonded host
        // with EDIV and RAND set to zero. Reject the Host in case of MITM since no Pairing has been performed.
        if ((port_alt_pair_get_auth() == GAP_AUTH_NONE) && con_fsm_params.has_mitm) {
                // reset bond_info
                if (con_fsm_params.has_mitm) {
                        port_alt_pair_set_auth(GAP_AUTH_REQ_MITM_BOND);
                }
                else {
                        port_alt_pair_set_auth(GAP_AUTH_REQ_NO_MITM_BOND);
                }

                // disconnect
                app_con_fsm_disconnect();
                dbg_puts(DBG_CONN_LVL, "    No authentication. Disconnecting\r\n");
                return false;
        }

        dbg_puts(DBG_CONN_LVL, "    GAPC_PAIRING_SUCCEED\r\n");
        
        return true;
}


/**
 ****************************************************************************************
 * \brief CONNECTED_PAIRING_ST connection completion event handler
 *
 * \details Called on a CONN_CMP_EVT (while in CONNECTED_PAIRING_ST) to handle the accepted
 *  host
 ****************************************************************************************
 */
static void connected_pairing_state_conn_cmp_evt(void)
{
        dev_bdaddr = alt_dev_bdaddr;
    
        if (con_fsm_params.has_multi_bond) {
                multi_bond_enabled = MULTI_BOND_REJECT_NONE;
                port_timer_clear(APP_CON_FSM_ALT_PAIR_TIMER, BLE_TASK);
        }
    
        if (port_alt_pair_is_bonded()) {
                app_con_fsm_request_write_bonding_data();
        }
        
        if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {          
                int entry = port_alt_pair_get_active_index();
                
                if (entry == MAX_BOND_PEER) { // If this is a new entry
                        // Find the place where it will be stored
                        entry = port_multi_bond_get_entry_to_delete();
                }
                app_white_list_add_host(app_con_fsm_get_peer_addr_type(),
                        app_con_fsm_get_peer_addr(), entry);
        }
    
        // has the host public /* or static random address */?
        if (con_fsm_params.disable_bonding_data_storage == false
                && (ADDR_PUBLIC == app_con_fsm_get_peer_addr_type())
                /*|| ( (port_ble_get_peer_addr_type() == ADDR_RAND) && ((port_ble_get_peer_addr()->addr[5] & GAP_STATIC_ADDR) == GAP_STATIC_ADDR) )*/) {
                
                // Set the the directed_adv_permitted flag to true in order to do    
                // Directed Advertising to this host if connection is lost
                directed_adv_permitted = true;
        }
        else {
                // Alternatively set the flag to false
                // Will be set to true if Privacy is enabled and the Reconnection address is written
                directed_adv_permitted = false; 
        }
        app_con_fsm_call_callback(INDICATE_CONNECTED);
        
        app_con_fsm_param_update();
}


/**
 ****************************************************************************************
 * \brief CONNECTED_PAIRING_ST state handler 
 *
 * \details Handles any incoming events while the connection FSM is in 
 * CONNECTED_PAIRING_ST state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static main_fsm_states handle_connected_pairing_state(enum main_fsm_events evt)
{
        connected_pairing_state_common_evt();

        switch (evt) {
        case PASSKEY_ENTERED:
                // Handle the Pass key entered event
                connected_pairing_state_passkey_entered_evt();

                // No state change
                return CONNECTED_PAIRING_ST; 

        case CONN_IRK_EXCH:
                // IRK has been exchanged before getting GAPC_PAIRING_SUCCEED. 
                // Do not store bonging data. Wait for pairing to be completed.
        
                // Update the multi_bond_resolved_peer_pos so that the new bonding data can replace the old data
                // in case the host was bonded to the device in the past
                port_alt_pair_find_irk(port_alt_pair_get_irk(), 1);
                return CONNECTED_PAIRING_ST;
        case CONN_CMP_EVT:
                // If a connection complete event occurs

                // Check if the host can be accepted
                if(!connected_pairing_state_conn_cmp_evt_check_host()) {
                    // If the host should be rejected stay in the same state    
                    return CONNECTED_PAIRING_ST;
                }

                // Else handle the event properly (the host has not been rejected)
                connected_pairing_state_conn_cmp_evt();

                // Go to connected state
                return CONNECTED_ST;

                
        case DISCONNECT_CMP_EVT:
                // A Disconnection event has occurred, so start advertising 
                // to search for a host to pair with
                app_con_fsm_start_advertise(app_con_fsm_get_last_queued_adv_type());
                
                // Go to advertising state
                return ADVERTISING_ST;        
        
        
        case PASSCODE_TIMEOUT_EVT:
                // If a passcode timeout event occurred, then we should disconnect
                if (con_fsm_params.has_passcode_timeout) {
                        dbg_puts(DBG_CONN_LVL, "[Disconnect due to passcode timeout]");
                        app_con_fsm_disconnect();
                        return DISCONNECTED_INIT_ST;
                }
                else {
                        // A passcode timeout event occurred but we haven't enabled the passcode timeout
                        // configuration. ASSERT_WARNING!
                        ASSERT_WARNING(0);
                        return CONNECTED_PAIRING_ST;
                }
              
                
        case POWEROFF_EVT:
                // A poweroff event has occurred, so close the ongoing connection / disconnect!
                dbg_puts(DBG_CONN_LVL, "[Disconnect due to power off]");
                app_con_fsm_disconnect();
        
                // Start the power off timer and
                app_con_fsm_start_gp_timer(APP_CON_FSM_POWEROFF_SUBTIMER);
        
                // Go to poweroff state in order to wait for a shutdown event
                return POWEROFF_ST;
        
        
        case START_PAIRING_EVT:
                // A start pairing event has occurred
                dbg_puts(DBG_CONN_LVL, "[Disconnect due to start pairing]");
                
                // If multi-bond feature is enabled, disconnect using the alt-pair timer
                if(con_fsm_params.has_multi_bond == true)
                {
                    // Disconnect - End the current connection in progress
                    app_alt_pair_disconnect();
                }
                // Else use the simple disconnection function    
                else
                {
                    // Disconnect - End the current connection in progress
                    app_con_fsm_disconnect();    
                }
                
                // Set/Enqueue the next adv type to be default
                app_con_fsm_enqueue_adv_type(APP_CON_FSM_ADV_DEFAULT);
                
                // Go to disconnected init state to wait for disconnection completion
                return DISCONNECTED_INIT_ST;                    
        
                
         case SWITCH_EVT:
                // Handle the switch event
                if(con_fsm_params.has_multi_bond == true)
                {
                    dbg_puts(DBG_CONN_LVL, "[Disconnect due to switching]");
                    
                    // Disconnect - End the current connection in progress
                    app_alt_pair_disconnect();
                    
                    // When disconnection occurs, enqueue the next adv type to be reconnect
                    app_con_fsm_enqueue_adv_type(APP_CON_FSM_ADV_RECONNECT);
                    
                    // Go to disconnected init state to wait for disconnection completion
                    return DISCONNECTED_INIT_ST;
                }
         
                /* Decide the new state */
                return CONNECTED_PAIRING_ST;        
        
                
        #ifdef HAS_SPECIAL_ADVERTISING
        case SPECIAL_ADV_START_EVT:
        #endif
        case USER_EVT:
                return CONNECTED_PAIRING_ST;
        
        default:
                #ifdef CON_FSM_WARNINGS
                        dbg_warning_con_fsm(current_fsm_state, evt);
                #else
                        ASSERT_WARNING(0);
                #endif
                return CONNECTED_PAIRING_ST;
        }
}


/**
 ****************************************************************************************
 * \brief CONNECTED_ST state param update timeout event handler
 *
 * \details Handles a PARAM_UPD_TIMEOUT_EVT while in CONNECTED_ST
 ****************************************************************************************
 */
static void connected_state_param_upd_timeout_evt(void)
{
    if (con_fsm_params.use_pref_conn_params) {
            app_con_fsm_send_connection_upd_req(true);
#ifdef USE_L2CAP_CONN_UPDATE_REQ
            conn_upd_pending = false;
#endif
    }
    else {
            ASSERT_WARNING(0);
    }
}


/**
 ****************************************************************************************
 * \brief CONNECTED_ST state power off event handler
 *
 * \details Handles a POWEROFF_EVT while in CONNECTED_ST
 ****************************************************************************************
 */
static void connected_state_poweroff_evt(void)
{
        dbg_puts(DBG_CONN_LVL, "[Disconnect due to poweroff]");
    
        if (con_fsm_params.use_pref_conn_params) {
                // Clear update connection params timer
                app_con_fsm_stop_gp_timer(APP_CON_FSM_PARAM_UPD_SUBTIMER);
                dbg_puts(DBG_CONN_LVL, "[(-) params update timer]");
        }
        
        // If there is a notification timeout
        if(con_fsm_params.notification_timeout != 0) {
                
                // Start the poweroff timer
                app_con_fsm_start_gp_timer(APP_CON_FSM_POWEROFF_SUBTIMER);
        }
        else {
                // Else disconnect immediately
                app_con_fsm_disconnect();
        }
}


/**
 ****************************************************************************************
 * \brief CONNECTED_ST pairing request event handler
 *
 * \details While in CONNECTED_ST state this function handles any incoming pairing
 * request events (PAIRING_REQ_EVT) and checks if the request can be accepted and sends
 * a pairing response accordingly
 *
 * \return bool true if pairing request was accepted, false otherwise
 ****************************************************************************************
 */
static bool connected_state_pairing_req_evt_is_accepted(void)
{
    dbg_puts(DBG_CONN_LVL, "    GAPC_PAIRING_REQ ind\r\n");
    if (app_con_fsm_get_last_queued_adv_type() != APP_CON_FSM_ADV_DEFAULT) { 
            // if we are bonded to a host with random address and we are performing undirected advertising to that host
            // then reject pairing requests from other hosts
            dbg_puts(DBG_CONN_LVL, "    GAPC_PAIRING_REQ rejected due to ADV_BONDED\r\n");
            port_security_send_pairing_rsp(false);
        
            return false;
    }  
    
    // special case: got a PAIRING_REQ when already paired and connected! Do not clear bond_info
    // in this case or else the DB will be corrupted (i.e. the notifications will be disabled
    // while the host had them enabled and won't re-enable them after the 2nd Pairing...).
    
    port_security_send_pairing_rsp(true);

    if ((con_fsm_params.has_mitm) || (con_fsm_params.has_nv_rom)) {
            app_con_fsm_call_callback(INDICATE_CONNECTION_IN_PROGRESS);
    }

    if (con_fsm_params.use_pref_conn_params) {
            // Clear update connection params timer or it may hit while we are in CONNECTED_PAIRING_ST...
            app_con_fsm_stop_gp_timer(APP_CON_FSM_PARAM_UPD_SUBTIMER);
            dbg_puts(DBG_CONN_LVL, "[(-) params update timer]");
    }

    if (con_fsm_params.has_mitm && con_fsm_params.has_passcode_timeout) {
            app_con_fsm_start_gp_timer(APP_CON_FSM_PASSCODE_SUBTIMER);
            dbg_puts(DBG_CONN_LVL, "[(+) passcode timer]");
    }
    
    // entering "passcode" mode
    app_con_fsm_call_callback(INDICATE_REINITIALIZE);
    
    return true;
}


/**
 ****************************************************************************************
 * \brief CONNECTED_ST state handler 
 *
 * \details Handles any incoming events while the connection FSM is in CONNECTED_ST state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static main_fsm_states handle_connected_state(enum main_fsm_events evt)
{
        switch (evt) {
            case PAIRING_REQ_EVT:
                // If the pairing request was valid we decide to go to connected pairing state
                if(connected_state_pairing_req_evt_is_accepted()) {
                    return CONNECTED_PAIRING_ST;
                }

                // Else stay in the same state
                return CONNECTED_ST;
            
            case CONN_IRK_EXCH:
                // IRK has been exhanged after GAPC_PAIRING_SUCCEED event.
                // Write the bonding data again to store the IRK.
                app_con_fsm_request_write_bonding_data();
                return CONNECTED_ST;        
        case CONN_UPD_RESP_EVT:  
                // PARAM_UPDATE was completed!
                // set the pending flag to false
                conn_upd_pending = false;
                return CONNECTED_ST;

        
        case DISCONNECT_CMP_EVT:
                // A disconnection event has occurred, so stop the params update timer
                if (con_fsm_params.use_pref_conn_params) {
                        // Clear update connection params timer
                        app_con_fsm_stop_gp_timer(APP_CON_FSM_PARAM_UPD_SUBTIMER);
                        dbg_puts(DBG_CONN_LVL, "[(-) params update timer]");
                }        
                
                if (con_fsm_params.disable_advertise_after_disconnection == true) {
                        // If advertising after disconnection is disabled, indicate IDLEness
                        indicate_disconnecting_idle();    
                        
                        // And go to IDLE state
                        return IDLE_ST;
                }
                else {
                        // Else start advertising in order to reconnect
                        app_con_fsm_start_advertise(APP_CON_FSM_ADV_RECONNECT);
                    
                        // And go to advertising state
                        return ADVERTISING_ST;
                }
                
                
        case PARAM_UPD_TIMEOUT_EVT:
                // Handle the params update timer's timeout event
                connected_state_param_upd_timeout_evt();
        
                // Stay in the same state
                return CONNECTED_ST;        
        
        
        case START_PAIRING_EVT:
                // A start pairing event has occurred
                dbg_puts(DBG_CONN_LVL, "[Disconnect due to start pairing]");
                
                // If multi-bond feature is enabled, disconnect using the alt-pair timer
                if(con_fsm_params.has_multi_bond == true)
                {
                    // Disconnect - End the current connection in progress
                    app_alt_pair_disconnect();
                }
                // Else use the simple disconnection function    
                else
                {
                    // Disconnect - End the current connection in progress
                    app_con_fsm_disconnect();    
                }
                
                // Set/Enqueue the next adv type to be default
                app_con_fsm_enqueue_adv_type(APP_CON_FSM_ADV_DEFAULT);
                
                // Go to disconnected init state to wait for disconnection completion
                return DISCONNECTED_INIT_ST;                    
        
                
         case SWITCH_EVT:
                // If multi-bond feature is enabled, disconnect using the alt-pair timer
                if(con_fsm_params.has_multi_bond == true)
                {
                    dbg_puts(DBG_CONN_LVL, "[Disconnect due to switching]");
                    
                    // Disconnect - End the current connection in progress
                    app_alt_pair_disconnect();
                    
                    if (con_fsm_params.use_pref_conn_params) {
                                // Clear update connection params timer
                                app_con_fsm_stop_gp_timer(APP_CON_FSM_PARAM_UPD_SUBTIMER);
                                dbg_puts(DBG_CONN_LVL, "[(-) params update timer]");   
                    }
                    
                    // When disconnection occurs, enqueue the next adv type to be reconnect
                    app_con_fsm_enqueue_adv_type(APP_CON_FSM_ADV_RECONNECT);
                    
                    // Go to disconnected init state to wait for disconnection completion
                    return DISCONNECTED_INIT_ST;
                }

                /* Decide the new state */
                return CONNECTED_ST;                
               
                
        case POWEROFF_EVT:
                // Handle the PowerOff Event
                connected_state_poweroff_evt();
                
                // Go to poweroff state
                return POWEROFF_ST;

        
#ifdef HAS_SPECIAL_ADVERTISING
        case SPECIAL_ADV_ENDED_EVT:
        case SPECIAL_ADV_START_EVT:
#endif
        case ADV_COMPLETED_EVT:
                return CONNECTED_ST;        
        
        default:
                #ifdef CON_FSM_WARNINGS
                        dbg_warning_con_fsm(current_fsm_state, evt);
                #else
                        ASSERT_WARNING(0);
                #endif
                return CONNECTED_ST;
        }
}


/**
 ****************************************************************************************
 * \brief POWEROFF_ST state handler 
 *
 * \details Handles any incoming events while the connection FSM is in POWEROFF_ST state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static main_fsm_states handle_poweroff_state(enum main_fsm_events evt)
{
        switch (evt) {
                
        case CONN_REQ_EVT:
                // During poweroff state, if a connection request occurs
                // Disconnect and stay in the same state
                app_con_fsm_disconnect();
                return POWEROFF_ST;
        
        
		case POWEROFF_TIMEOUT_EVT:
                // The poweroff timer has expired so disconnect and
                app_con_fsm_disconnect();
        
                // go to "waiting disconnection after poweroff" state in order to wait for
                // disconnection completion
                return WAITING_DISCONNECTION_AFTER_POWEROFF;
		
        
		case SHUTDOWN_EVT:
                // A shutdown evt was triggered, so stop the poweroff timer (if running)
                app_con_fsm_stop_gp_timer(APP_CON_FSM_POWEROFF_SUBTIMER);
        
                // And close the existing connection
                app_con_fsm_disconnect();
        
                // Go to "waiting disconnection after poweroff" state 
                // in order to wait for disconnection completion
                return WAITING_DISCONNECTION_AFTER_POWEROFF;

        
        case DISCONNECT_CMP_EVT:
                // Disconnection completion event has occurred so
                // Indicate IDLEness and 
                indicate_disconnecting_idle();
        
                // go to IDLE state
                return IDLE_ST;
        
        
        // Event Ignore List
        case POWEROFF_EVT:
        case SWITCH_EVT:
        case START_PAIRING_EVT:
    #ifdef HAS_SPECIAL_ADVERTISING
        case SPECIAL_ADV_START_EVT:
    #endif
            return POWEROFF_ST;
             
        
        default:
                #ifdef CON_FSM_WARNINGS
                    dbg_warning_con_fsm(current_fsm_state, evt);
                #else
                    ASSERT_WARNING(0);
                #endif
                return POWEROFF_ST;
        }
        
}


/**
 ****************************************************************************************
 * \brief WAITING_DISCONNECTION_AFTER_POWEROFF state handler 
 *
 * \details Handles any incoming events while the connection FSM is in 
 * WAITING_DISCONNECTION_AFTER_POWEROFF state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static main_fsm_states handle_waiting_disconnection_after_poweroff_state(enum main_fsm_events evt)
{
    switch(evt)
    {
        
        case DISCONNECT_CMP_EVT :
                // Disconnection completion has occurred so 
        
                // Indicate IDLEness, 
                indicate_disconnecting_idle();
        
                // stop the poweroff timer(if running)
                app_con_fsm_stop_gp_timer(APP_CON_FSM_POWEROFF_SUBTIMER);
        
                // and go to IDLE state
                return IDLE_ST;
        
        
#ifdef HAS_SPECIAL_ADVERTISING        
        case SPECIAL_ADV_ENDED_EVT:
        case SPECIAL_ADV_START_EVT:
#endif
        case ADV_COMPLETED_EVT:
                return WAITING_DISCONNECTION_AFTER_POWEROFF;
        
        default:
            #ifdef CON_FSM_WARNINGS
                    dbg_warning_con_fsm(current_fsm_state, evt);
            #else
                    ASSERT_WARNING(0);
            #endif
            return WAITING_DISCONNECTION_AFTER_POWEROFF;
    }
    
}


/**
 ****************************************************************************************
 * \brief DISCONNECTED_INIT_ST state handler 
 *
 * \details Handles any incoming events while the connection FSM is in 
 * DISCONNECTED_INIT_ST state
 *
 * \param[in] evt The event to be processed
 *
 * \return main_fsm_states The next state after the event has been handled
 ****************************************************************************************
 */
static  main_fsm_states handle_disconnected_init_state(enum main_fsm_events evt)
{
        switch (evt) {
        case DISCONNECT_CMP_EVT:
                // We were waiting for this event to start advertising again
                app_con_fsm_start_advertise(app_con_fsm_get_last_queued_adv_type());
                return ADVERTISING_ST;

        
        case POWEROFF_EVT:
                // When DISCONNECT_CMP_EVT device will power off.
                return POWEROFF_ST; 
        
        
#ifdef HAS_SPECIAL_ADVERTISING        
        case SPECIAL_ADV_ENDED_EVT:
#endif
        case ADV_COMPLETED_EVT:
        case CONN_UPD_RESP_EVT:  // PARAM_UPDATE was completed!
        case START_PAIRING_EVT:
        case SWITCH_EVT:
                
                return DISCONNECTED_INIT_ST;
        
        
        default:
                #ifdef CON_FSM_WARNINGS
                    dbg_warning_con_fsm(current_fsm_state, evt);
                #else
                    ASSERT_WARNING(0);
                #endif
                return DISCONNECTED_INIT_ST;
        }
}


void app_con_fsm_state_update(enum main_fsm_events evt)
{
#if (DEVELOPMENT_DEBUG)
        fsm_log[fsm_log_ptr].state = current_fsm_state;
        fsm_log[fsm_log_ptr].evt = evt;
        fsm_log[fsm_log_ptr].time = port_get_time();
        fsm_log_ptr++;
        if (fsm_log_ptr == FSM_LOG_DEPTH) {
                fsm_log_ptr = 0;
        }
#endif
        dbg_printf(DBG_CONN_LVL, "**CON evt:%s", events_names[evt]);
        dbg_printf(DBG_CONN_LVL, " @ %s", state_names[current_fsm_state]);

        switch(evt) {            
        case USER_EVT:
            // USER_EVT is only used in IDLE state
            if(current_fsm_state != IDLE_ST) {
				dbg_printf(DBG_CONN_LVL, " -> %s\r\n", state_names[current_fsm_state]);
                return;
            }
            break;

        case DISCONNECT_CMP_EVT:
                // Stop any running timers
                app_con_fsm_stop_gp_timer(APP_CON_FSM_ALL_SUBTIMERS);
                port_timer_clear(APP_CON_FSM_ENC_TIMER, BLE_TASK);

                // Send an indication to the main application
                app_con_fsm_call_callback(INDICATE_DISCONNECTED);
                break;

        }       
        
        
        /*
            Depending on the current connection FSM state and the event that occurred,
            call the current state handler
            and get which will be the next state
        */
        
        switch (current_fsm_state) {
        
        case IDLE_ST:
                current_fsm_state = handle_idle_state(evt);
                break;
        
        case ADVERTISING_ST:
                current_fsm_state = handle_advertising_state(evt);
                break;
        
        case CONNECTION_IN_PROGRESS_ST:
                current_fsm_state = handle_connection_in_progress_state(evt);
                break;
        
        case CONNECTED_PAIRING_ST:
                current_fsm_state = handle_connected_pairing_state(evt);
                break;
        
        case CONNECTED_ST:
                current_fsm_state = handle_connected_state(evt);
                break;
                
        case POWEROFF_ST:
                current_fsm_state = handle_poweroff_state(evt);
                break;

        case DISCONNECTED_INIT_ST:
                current_fsm_state = handle_disconnected_init_state(evt);
                break;

        case WAITING_DISCONNECTION_AFTER_POWEROFF:
                current_fsm_state = handle_waiting_disconnection_after_poweroff_state(evt);
                break;
        
        default:
                ASSERT_WARNING(0);
                break;
        }
        
        dbg_printf(DBG_CONN_LVL, " -> %s\r\n", state_names[current_fsm_state]);
}


void app_con_fsm_request_write_bonding_data()
{
        app_con_fsm_set_async_task(ASYNC_TASK_WRITE_BONDING_DATA);
}


void app_con_fsm_request_erase_bonding_data(void)
{
        app_con_fsm_set_async_task(ASYNC_TASK_ERASE_BONDING_DATA);
}


void app_con_fsm_reset_bonding_data(void)
{
        // CLRP clears bonding info unconditionally
        if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {
                app_white_list_clear();
        }
        
        app_con_fsm_request_erase_bonding_data();
        
        port_alt_pair_reset_bonding_data();
        
        directed_adv_permitted = false;
        
#ifdef FORCE_CONNECT_TO_HOST_ON
        fallback_peer_entry = MAX_BOND_PEER;
        app_alt_pair_reset_force_next_store_entry();
#endif
}


/**
 ****************************************************************************************
 * \brief   Check if connection to host can be accepted.
 *
 * \param[in] param  BD address of the host
 *
 * \return true, if connection can be accepted. Otherwise return false.
 ****************************************************************************************
 */
static bool alt_pair_check_peer(const peer_addr_t *param)
{

        if (port_alt_pair_get_active_index() == MAX_BOND_PEER) {
                return true;
        }

        if (con_fsm_params.has_multi_bond) {
                switch (multi_bond_enabled) {
                case MULTI_BOND_REJECT_LAST:
                case MULTI_BOND_REJECT_ALL_KNOWN:
                    
                // Do not reject known hosts here. They will be rejected during encryption process.
                // Known hosts must be allowed to be re-bonded
                        break;
                       
                default:
                        break;
                }
        }
       return true;
}


bool app_con_fsm_connection_validation(const peer_addr_t *addr)
{
        dbg_printf(DBG_CONN_LVL, "Peer addr %02x:%02x:%02x:%02x:%02x:%02x trying to connect\r\n",
                addr->addr.addr[5], addr->addr.addr[4],
                addr->addr.addr[3], addr->addr.addr[2],
                addr->addr.addr[1], addr->addr.addr[0]);

        app_con_fsm_state_update(CONN_REQ_EVT);

        if (con_fsm_params.has_virtual_white_list && app_con_fsm_get_last_queued_adv_type() != APP_CON_FSM_ADV_DEFAULT) { 
                // Public and Static Random Addresses are checked here. Resolvable Random Addresses are checked after
                // they are resolved.
                if ((ADDR_PUBLIC == addr->addr_type)
// FIXME Due to a BLE Manager bug we cannot check for static random address.
//       BLE manager reports the resolved address (which is always static) instead of the random address.
#ifndef PLATFORM_68X
                        || ((addr->addr_type == ADDR_RAND)
                                && ((addr->addr.addr[5] & GAP_STATIC_ADDR) == GAP_STATIC_ADDR))
#endif
                                ) 
                {
                        if (!app_white_list_lookup_public_in_virtual_white_list(addr->addr_type,
                                &addr->addr)) {
                                dbg_puts(DBG_CONN_LVL, "Host disconnected at connection. Not in virtual white list.\r\n");
                                // Inform the FSM that the connection is to be cancelled    
                                app_con_fsm_state_update(CONN_CANCELLED_EVT);   
                                return false;
                        }
                }
        }

        if (con_fsm_params.has_multi_bond) {
                if (alt_pair_check_peer(addr) == false) {
                        // Inform the FSM that the connection is to be cancelled   
                        app_con_fsm_state_update(CONN_CANCELLED_EVT);    
                        return false;
                }
        }

        
        // Reaching this point on means that we have a valid connection request
        port_alt_pair_clear_resolved_pos();

        if (con_fsm_params.has_mitm || con_fsm_params.has_nv_rom || port_alt_pair_is_bonded() == false) {

                // send connection confirmation
                app_con_fsm_connect_confirm(port_alt_pair_get_auth());

                // set a timer in case pairing/encryption does not follow
                port_timer_set(APP_CON_FSM_ENC_TIMER, BLE_TASK, con_fsm_params.enc_safeguard_timeout);
                dbg_puts(DBG_CONN_LVL, "[(+) Connection FSM ENC timer]");

                app_con_fsm_call_callback(INDICATE_CONNECTION_IN_PROGRESS);

                if (con_fsm_params.has_security_request_send) {
                        if ((port_con_fsm_get_peer_addr_type() == ADDR_RAND)
                             && ((port_con_fsm_get_peer_addr()->addr[BD_ADDR_LEN - 1] & 0xC0) == GAP_RSLV_ADDR)) {
                                 app_con_fsm_resolve_address();
                        }
                        else {
                                app_con_fsm_start_security();
                        }
                        
                }
        }
        else {
                // Both public and random addresses are checked here against the stored values during pairing
                if ((app_con_fsm_get_peer_addr_type() == port_alt_pair_get_addr_type()) &&
                        (!memcmp(port_alt_pair_get_addr()->addr, app_con_fsm_get_peer_addr()->addr, BD_ADDR_LEN)) &&
                        port_alt_pair_is_bonded()) {

                        // send connection confirmation
                        app_con_fsm_connect_confirm(port_alt_pair_get_auth());

                        // no bonding --> connection setup proceeds directly
                        app_con_fsm_state_update(CONN_CMP_EVT);
                }
                else {
                        // Not the previously bonded host or, in the case of random address, the address has changed.
                        // Will disconnect
                }
        }
        
        return true;
}


void app_con_fsm_mitm_passcode_entry_func(void)
{
        dbg_puts(DBG_CONN_LVL, "    GAPC_TK_EXCH\r\n");
        app_con_fsm_call_callback(INDICATE_START_PASSCODE);
        app_con_fsm_set_ble_async_task(BLE_ASYNC_TASK_ENTERING_PASSCODE);
}


bool app_con_fsm_encrypt_req_validation(uint16_t ediv, const struct rand_nb *rand_nb)
{
        if (con_fsm_params.has_nv_rom) {
                if (port_alt_pair_is_bonded()
                        && (memcmp(port_alt_pair_get_rand(), rand_nb, RAND_NB_LEN) == 0)
                        && (port_alt_pair_get_ediv() == ediv)) {
                        if (con_fsm_params.has_multi_bond) {
                                // the connecting host is the last host we connected to
                                if (multi_bond_enabled == MULTI_BOND_REJECT_LAST || multi_bond_enabled == MULTI_BOND_REJECT_ALL_KNOWN) {
                                        dbg_puts(DBG_CONN_LVL, "Host disconnected on ecryption request. Same host\r\n");
                                        return false;
                                }
                        }
                        
                        // If it's not blocked then no NV memory access is required to load keys.
                        port_alt_pair_updatedb_from_bonding_info();
                        port_alt_pair_update_usage_count();
                }
                else {
                        if (port_alt_pair_load_bond_data(rand_nb, ediv) != 0) {
                                if (con_fsm_params.has_multi_bond) {
                                    
                                        if (multi_bond_enabled == MULTI_BOND_REJECT_ALL_KNOWN) {
                                                dbg_puts(DBG_CONN_LVL, "Host disconnected on ecryption request. Known host\r\n");
                                                return false;
                                        }
                                }
                        }
                        else {
                                app_con_fsm_state_update(NEW_HOST_EVT);
                        }
                }
        }
        else {
                if (port_alt_pair_is_bonded()
                        && (memcmp(port_alt_pair_get_rand(), rand_nb, RAND_NB_LEN) == 0)
                        && (port_alt_pair_get_ediv() == ediv)) {
                        // bond_info.info and DB are OK
                        port_alt_pair_updatedb_from_bonding_info();
                }
                else {
                        port_alt_pair_set_default_info();
                }
        }

        return true;
}


void app_con_fsm_init(void)
{
#ifdef FORCE_CONNECT_TO_HOST_ON
        
        // Force-connect number requires multi bonding
        ASSERT_ERROR(con_fsm_params.has_multi_bond);

        // Force-connect to host requires a white list or virtual white list
        ASSERT_ERROR(con_fsm_params.has_virtual_white_list || con_fsm_params.has_white_list);

        // Initialize the next (switching) host and last host variables
        fallback_peer_entry = MAX_BOND_PEER;
    
#ifndef FORCE_CONNECT_NUM_OF_HOSTS
#error "Force-connect number of hosts not defined"
#endif

        ASSERT_ERROR(FORCE_CONNECT_NUM_OF_HOSTS <= MAX_BOND_PEER);
#endif // FORCE_CONNECT_TO_HOST_ON

        // Subtimer timeouts must be non-zero
        ASSERT_ERROR( con_fsm_params.passcode_timeout && 
                        con_fsm_params.time_to_request_param_upd && 
                        con_fsm_params.notification_timeout );
                        
        ASSERT_ERROR(con_fsm_params.has_smart_rssi_pairing == false || con_fsm_params.smart_pairing_rssi_threshold != 0);

        // Clear all the pending asynchronous tasks bitmap
        ble_async_tasks      = 0;   
        non_ble_async_tasks  = 0;
    
        // Initialize Multi-Bonding (if applicable)
        port_alt_pair_init();       
    
        // Initialize the Con (Connection) FSM state to IDLE
        current_fsm_state = IDLE_ST;

        // Initialize the Adv (Advertising) FSM 
        app_adv_fsm_init();
}


void app_con_fsm_on_disconnect(void)
{
    // Send a disconnection completion event to the connection FSM
    app_con_fsm_state_update(DISCONNECT_CMP_EVT);
}


bool app_con_fsm_on_ble_powered(void)
{

    if(ble_async_tasks & BLE_ASYNC_TASK_ENTERING_PASSCODE) {
        ble_async_tasks &= ~BLE_ASYNC_TASK_ENTERING_PASSCODE;
        app_con_fsm_state_update(PASSKEY_ENTERED);
    }
    
    
    if(ble_async_tasks & BLE_ASYNC_TASK_START_PAIR) {
        ble_async_tasks &= ~BLE_ASYNC_TASK_START_PAIR;
        app_con_fsm_state_update(START_PAIRING_EVT);
    }
    
    
    if(ble_async_tasks & BLE_ASYNC_TASK_SWITCH_HOST) {
        ble_async_tasks &= ~BLE_ASYNC_TASK_SWITCH_HOST;
        if (con_fsm_params.has_multi_bond) {        
                app_con_fsm_state_update(SWITCH_EVT);
        }
    }
    
    
    if(ble_async_tasks & BLE_ASYNC_TASK_SEND_ADV_COMPLETED_EVT) {
        ble_async_tasks &= ~BLE_ASYNC_TASK_SEND_ADV_COMPLETED_EVT;
        app_con_fsm_state_update(ADV_COMPLETED_EVT);
    }
    
#ifdef HAS_SPECIAL_ADVERTISING    
    if(ble_async_tasks & BLE_ASYNC_TASK_SEND_SPECIAL_ADV_ENDED_EVT) {
        ble_async_tasks &= ~BLE_ASYNC_TASK_SEND_SPECIAL_ADV_ENDED_EVT;
        app_con_fsm_state_update(SPECIAL_ADV_ENDED_EVT);
    }
#endif    
    return false;
}


uint8_t app_con_fsm_on_system_powered(void)
{
    
    if(non_ble_async_tasks & ASYNC_TASK_WRITE_BONDING_DATA) {
        non_ble_async_tasks &= ~ASYNC_TASK_WRITE_BONDING_DATA;
        if (con_fsm_params.has_nv_rom) {
            dbg_puts(DBG_CONN_LVL, "Writing bonding data\r\n");
            port_alt_pair_store_bond_data();
        }
    }
    
    
    if(non_ble_async_tasks & ASYNC_TASK_ERASE_BONDING_DATA)
    {
        non_ble_async_tasks &= ~ASYNC_TASK_ERASE_BONDING_DATA;
        if (con_fsm_params.has_nv_rom) {
            port_alt_pair_clear_all_bond_data();
        }
        app_con_fsm_request_disconnect(MULTI_BOND_REJECT_NONE, APP_CON_FSM_DC_REASON_START_PAIR);
    }
    
    return APP_GOTO_SLEEP;
}

bool app_con_fsm_switch_to_peer(uint8_t entry)
{
#ifdef FORCE_CONNECT_TO_HOST_ON
        dbg_printf(DBG_CONN_LVL, "\r\nUser requested SWITCH TO peer %d\r\n", entry);
        ASSERT_ERROR(con_fsm_params.has_virtual_white_list || con_fsm_params.has_white_list);

        uint8_t active_index = port_alt_pair_get_active_index();

        // If there are no valid data for this entry
        if (entry != CON_FSM_PEER_ANY) {
                if(entry != active_index && port_alt_pair_load_entry(entry) == false) {
                        return false;
                }
        }

		multi_bond_enabled = MULTI_BOND_REJECT_NONE;
        app_white_list_clear();
        
        if(entry == CON_FSM_PEER_ANY) {
                for(int i = 0; i < FORCE_CONNECT_NUM_OF_HOSTS; i ++) {
                        if(i != active_index && port_alt_pair_load_entry(i) == true) {
                                app_white_list_add_host(port_alt_pair_get_addr_type(), port_alt_pair_get_addr(), i);
                        }
                }
                directed_adv_permitted = false; 
        }
        else {
                app_white_list_add_host(port_alt_pair_get_addr_type(), port_alt_pair_get_addr(), entry);
                if (port_alt_pair_get_addr_type() == ADDR_PUBLIC
                        /*|| ( (app_ble_get_peer_addr_type() == ADDR_RAND) && ((app_ble_get_peer_addr()->addr[5] & GAP_STATIC_ADDR) == GAP_STATIC_ADDR) )*/) {
                            
                        // Set the the directed_adv_permitted flag to true in order to do    
                        // Directed Advertising to this host if connection is lost    
                        directed_adv_permitted = true; 
                }
                else {
                        // Alternatively set the flag to false
                        // will be set to true if Privacy is enabled and the Reconnection address is written
                        directed_adv_permitted = false; 
                }        
        }
        

        
        if (entry != active_index) {
                // If the new host is not the current host
                app_con_fsm_request_disconnect(MULTI_BOND_REJECT_NONE, APP_CON_FSM_DC_REASON_SWITCH_HOST);  // do not reject known hosts
        }

        return true;
#else
        ASSERT_WARNING(0); // FORCE_CONNECT_TO_HOST_ON must be defined to use this function
        return false;
#endif        
}


#ifdef HAS_SPECIAL_ADVERTISING
void app_con_fsm_send_special_packet(uint8_t *data, uint8_t length)
{
    app_adv_fsm_set_special_adv_data(data, length);

    app_con_fsm_state_update(SPECIAL_ADV_START_EVT);  
}
#endif


void app_con_fsm_add_host_to_entry(uint8_t entry)
{
#ifdef FORCE_CONNECT_TO_HOST_ON
        dbg_printf(DBG_CONN_LVL, "\r\nUser requested SAVE peer to %d\r\n", entry);
        ASSERT_ERROR(con_fsm_params.has_virtual_white_list || con_fsm_params.has_white_list);
        ASSERT_ERROR(entry < MAX_BOND_PEER);
    
        app_white_list_clear();
        fallback_peer_entry = app_alt_pair_force_next_store_entry(entry);
        app_con_fsm_request_disconnect(MULTI_BOND_REJECT_ALL_KNOWN, APP_CON_FSM_DC_REASON_START_PAIR);  // reject all known hosts
#else
        ASSERT_WARNING(0); // FORCE_CONNECT_TO_HOST_ON must be defined to use this function
#endif  
}


void app_con_fsm_request_disconnect(enum multi_bond_host_rejection reject_hosts, app_con_fsm_disconnect_req_reason_t reason)
{
    
    multi_bond_enabled = reject_hosts;
    switch(reason) {
        
        case APP_CON_FSM_DC_REASON_SWITCH_HOST:
            app_con_fsm_set_ble_async_task(BLE_ASYNC_TASK_SWITCH_HOST);
            dbg_puts(DBG_CONN_LVL, "User requested disconnection to switch\r\n");
            break;
        
        case APP_CON_FSM_DC_REASON_START_PAIR:
            app_con_fsm_set_ble_async_task(BLE_ASYNC_TASK_START_PAIR);
            dbg_puts(DBG_CONN_LVL, "User requested disconnection to pair\r\n");
            break;

        default:
            ASSERT_ERROR(0);
            break;
    }
}


main_fsm_states app_con_fsm_get_state(void)
{
        return current_fsm_state;
}


void app_con_fsm_handle_cmp_evt(uint8_t operation, uint8_t status)
{
        switch (operation) {
            
        case GAPM_RESOLV_ADDR:
                if (con_fsm_params.has_nv_rom) {
                        if (status == GAP_ERR_NO_ERROR) {
                                // found!
                                // do nothing, handled in app_con_fsm_gapm_addr_solved_ind_handler()
                        }
                        else if (status != GAP_ERR_NOT_FOUND) {
                                ASSERT_WARNING(0);
                        }
                        else {
                                // Host address has not been resolved - New Host
                                if (con_fsm_params.has_virtual_white_list) {
                                        if ((app_white_list_written() != 0)) {
                                                dbg_puts(DBG_CONN_LVL, "New host not in virtual white list. Disconnect\r\n");
                                                app_con_fsm_disconnect();
                                        }
                                        else if (con_fsm_params.has_security_request_send) {
                                                app_con_fsm_start_security();
                                        }
                                }
                                else if (con_fsm_params.has_security_request_send) {
                                        app_con_fsm_start_security();
                                }
                        }
                }
                break;
                
        case GAPM_ADD_DEV_IN_WLIST:
        case GAPM_RMV_DEV_FRM_WLIST:
                if (con_fsm_params.has_white_list) {
                        app_white_list_handle_cmp_evt(operation, status);
                }
                break;
        }
}


void app_con_fsm_random_address_resolved(struct gap_sec_key *irk)
{
        uint8_t resolved_peer_pos = 0;
        // The Host has been found! We don't care about the key and the BD address. 
        // The entry will be located again when EDIV & RAND are provided.
    
        // Since we have the IRK, we can find the real address
        if (port_alt_pair_is_bonded()
                && (memcmp(port_alt_pair_get_irk(), &(irk->key[0]), KEY_LEN) == 0)
                && (con_fsm_params.has_multi_bond)
                && (multi_bond_enabled == MULTI_BOND_REJECT_LAST)) {
//                dbg_puts(DBG_CONN_LVL, "Host disconnected when address resolved. Same host\r\n");
//                app_con_fsm_disconnect();
        }
        else {
                resolved_peer_pos = port_alt_pair_find_irk(irk, 1);
                if (con_fsm_params.has_virtual_white_list) {
                        bool found = resolved_peer_pos != 0;
                        while (found) {
                            if (app_white_list_lookup_rand_in_virtual_white_list(resolved_peer_pos - 1)) {
                                    break;
                            }
                            // Continue to searh in casethe host is stored in more than one places
                            resolved_peer_pos = port_alt_pair_find_irk(irk, resolved_peer_pos + 1);
                            found = resolved_peer_pos != 0;
                        }
                        if(found == false) {
                                dbg_puts(DBG_CONN_LVL, "Host disconnected when address resolved. Not in virtual white list\r\n");
                                app_con_fsm_disconnect();
                        }
                }
        }
}


bool app_con_fsm_is_conn_upd_pending(void)
{
        return conn_upd_pending;
}


/**
 ****************************************************************************************
 * \brief Directed Advertising Ended notification handler
 *
 * \details Handles an incoming ADV_FSM_DIR_ADV_ENDED notification from the 
 * Advertising FSM
 ****************************************************************************************
 */
static void handle_dir_adv_ended_notification(void)
{
        app_con_fsm_set_ble_async_task(BLE_ASYNC_TASK_SEND_ADV_COMPLETED_EVT);
        app_con_fsm_advertising = false;
}


/**
 ****************************************************************************************
 * \brief Directed Advertising Started notification handler
 *
 * \details Handles an incoming ADV_FSM_DIR_ADV_STARTED notification from the 
 * Advertising FSM
 ****************************************************************************************
 */
static void handle_dir_adv_started_notification(void)
{
        app_con_fsm_advertising = true;
}


/**
 ****************************************************************************************
 * \brief Undirected Advertising Ended notification handler
 *
 * \details Handles an incoming ADV_FSM_UND_ADV_ENDED notification from the 
 * Advertising FSM
 ****************************************************************************************
 */
static void handle_und_adv_ended_notification(void)
{
        app_con_fsm_set_ble_async_task(BLE_ASYNC_TASK_SEND_ADV_COMPLETED_EVT);
        app_con_fsm_call_callback(INDICATE_ADVERTISING_END);
        app_con_fsm_advertising = false;
}


/**
 ****************************************************************************************
 * \brief Undirected Advertising Started notification handler
 *
 * \details Handles an incoming ADV_FSM_UND_ADV_STARTED notification from the 
 * Advertising FSM
 ****************************************************************************************
 */
static void handle_und_adv_started_notification(void)
{
        app_con_fsm_call_callback(INDICATE_ADVERTISING_START);
        app_con_fsm_advertising = true;
}


/**
 ****************************************************************************************
 * \brief Undirected Limited Advertising Started notification handler
 *
 * \details Handles an incoming ADV_FSM_UND_ADV_LIM_STARTED notification from the 
 * Advertising FSM
 ****************************************************************************************
 */
static void handle_und_adv_lim_started_notification(void)
{
        app_con_fsm_advertising = true;
}


/**
 ****************************************************************************************
 * \brief Undirected Limited Advertising Ended notification handler
 *
 * \details Handles an incoming ADV_FSM_UND_ADV_LIM_ENDED notification from the 
 * Advertising FSM
 ****************************************************************************************
 */
static void handle_und_adv_lim_ended_notification(void)
{
        app_con_fsm_set_ble_async_task(BLE_ASYNC_TASK_SEND_ADV_COMPLETED_EVT);
        app_con_fsm_advertising = false;
}


#ifdef HAS_SPECIAL_ADVERTISING
/**
 ****************************************************************************************
 * \brief Special Undirected Advertising Ended notification handler
 *
 * \details Handles an incoming ADV_FSM_UND_ADV_SPECIAL_ENDED notification from the 
 * Advertising FSM
 ****************************************************************************************
 */
static void handle_special_adv_ended_notification(void)
{
     app_con_fsm_advertising = false;
     app_con_fsm_set_ble_async_task(BLE_ASYNC_TASK_SEND_SPECIAL_ADV_ENDED_EVT);
}
#endif


void app_con_fsm_handle_adv_notification(uint8_t notif)
{
    switch(notif)
    {        
        case ADV_FSM_DIR_ADV_STARTED:
            handle_dir_adv_started_notification();
            break;
        
        case ADV_FSM_DIR_ADV_ENDED:
            handle_dir_adv_ended_notification();
            break;
        
        case ADV_FSM_UND_ADV_STARTED:
            handle_und_adv_started_notification();
            break;
        
        case ADV_FSM_UND_ADV_ENDED:
            handle_und_adv_ended_notification();
            break;
        
        case ADV_FSM_UND_ADV_LIM_STARTED:
            handle_und_adv_lim_started_notification();
            break;
        
        case ADV_FSM_UND_ADV_LIM_ENDED:
            handle_und_adv_lim_ended_notification();
            break;
    #ifdef HAS_SPECIAL_ADVERTISING        
        case ADV_FSM_UND_ADV_SPECIAL_STARTED:
            break;
        
        case ADV_FSM_UND_ADV_SPECIAL_ENDED:
            handle_special_adv_ended_notification();
            break;
    #endif     
        default:
            break;
    }
}


app_prf_srv_perm_t app_con_fsm_get_access_rights(void)
{
        if (con_fsm_params.has_mitm) {
                return SRV_PERM_AUTH;
        } else {
                if (con_fsm_params.has_nv_rom) {
                        return SRV_PERM_UNAUTH;
                } else {
                        return SRV_PERM_ENABLE;
                }            
        }    
}


void app_con_fsm_send_connection_upd_req(bool force_min_interval)
{
        if(app_con_fsm_funcs.con_fsm_conn_upd_req) {
                app_con_fsm_funcs.con_fsm_conn_upd_req(&con_fsm_params.param_update, force_min_interval);
        }
        else {
                ASSERT_ERROR(0);
        }
}


struct bd_addr* app_con_fsm_get_peer_addr(void)
{
        if(app_con_fsm_funcs.con_fsm_get_peeraddr) {
                return app_con_fsm_funcs.con_fsm_get_peeraddr();
        }

        ASSERT_ERROR(0);
        return NULL;     
}


uint8_t app_con_fsm_get_peer_addr_type(void)
{
        if(app_con_fsm_funcs.con_fsm_get_peeraddrtype) {
                return app_con_fsm_funcs.con_fsm_get_peeraddrtype();
        }

        ASSERT_ERROR(0);
        return NULL;     
}


void app_con_fsm_mitm_passcode_report(uint32_t code)
{
        dbg_printf(DBG_CONN_LVL, "Code: %d\r\n", code);   

        if(app_con_fsm_funcs.con_fsm_mitm_passcode_report) {
                app_con_fsm_funcs.con_fsm_mitm_passcode_report(code);
        }
        else {
                ASSERT_ERROR(0);
        }
}

#endif // HAS_CONNECTION_FSM

/**
 * \}
 * \}
 * \}
 */
