/****************************************************************************************
* \file user_rcu.c
* \brief RCU implementation
*****************************************************************************************/

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_APP
 * \{
 * \addtogroup APP_RCU
 *
 * \{
******************************************************************************************/

/*
 ****************************************************************************************
 * INCLUDE FILES
******************************************************************************************/

#include "user_rcu.h"
#include "user_rcu_kbd.h"
#include "user_modules.h"
#include "app_prf_perm_types.h"
#include "gattc_task.h"
#include "l2cm.h"
#include "syscntl.h"
#include "port_ble_gap.h"
#include "port_platform.h"
#include "port_timer.h"
#include "ke_env.h"

#ifdef HAS_PWR_MGR
    #include "user_pwr_mgr.h"
#endif

#ifdef HAS_WKUP
	#include "port_wkup.h"
#endif    

#if ((RWBLE_SW_VERSION_MAJOR >= 8) && BLE_BATT_SERVER) ||  BLE_BAS_SERVER
	#include "bass.h"
	#include "app_bass.h"
#endif

#ifdef HAS_CONNECTION_FSM      
    #include "port_multi_bond.h"
#endif

#if (RWBLE_SW_VERSION_MAJOR >= 8)
    #include "prf_utils.h"
#endif

/*
 ****************************************************************************************
 * DEFINES
******************************************************************************************/

#if (RWBLE_SW_VERSION_MAJOR >= 8)
    #define APP_TASK_ID_DISS   TASK_ID_DISS
    #define APP_TASK_ID_BASS   TASK_ID_BASS
    #define APP_TASK_ID_HOGPD  TASK_ID_HOGPD    
    #define APP_TASK_ID_SUOTAR TASK_ID_SUOTAR    
    #define APP_TASK_ID_CUSTS1 TASK_ID_CUSTS1
#else
    #define APP_TASK_ID_DISS   TASK_DISS
    #define APP_TASK_ID_BASS   TASK_BASS
    #define APP_TASK_ID_HOGPD  TASK_HOGPD    
    #define APP_TASK_ID_SUOTAR TASK_SPOTAR    
    #define APP_TASK_ID_CUSTS1 TASK_CUSTS1
#endif

#ifndef HAS_CONNECTION_FSM
    #ifndef CFG_APP_SECURITY
        #warning "Please define CFG_APP_SECURITY if security is needed"
    #endif
#endif

#if defined(HAS_PWR_MGR) && defined(NORMALLY_CONNECTABLE)
    #warning "It does not make sense to use inactivity timeout when the NORMALLY_CONNECTABLE is defined..."
#endif

#if defined(HAS_KBD) && defined(HAS_HID_REPORT) && (HID_REPORT_ROLL_OVER_BUF_SIZE <= KBD_DEBOUNCE_BUFFER_SIZE)
    #error "The roll-over buffer must be bigger than the debounce buffer."
#endif

/*
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/
extern bool skip_slave_latency_flag;

bool system_initialized __PORT_RETAINED;
uint8_t last_connection_idx __PORT_RETAINED;

#ifdef HAS_AUDIO
    struct gapc_param_updated_ind user_connection_params __PORT_RETAINED;
#endif

#ifdef HAS_CONNECTION_FSM 
    bool user_sync_key_press_evt __PORT_RETAINED; // flag to indicate a Key press to the high-level FSM synchronously to the BLE
#else
	bool user_conn_update_pending __PORT_RETAINED;
#endif

#ifdef HAS_ACTION_INACTIVITY_TIMEOUT
    bool user_activity_period_expired;
#endif

#if (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER)
    bool user_suota_in_progress __PORT_RETAINED;
#endif
 
#ifdef HAS_HID_REPORT
    bool user_hid_report_normal_full_release_pending   __PORT_RETAINED;
    bool user_hid_report_extended_full_release_pending __PORT_RETAINED;
#endif
 
#ifdef HAS_PWR_MGR
    bool user_reset_inactivity_triggered __PORT_RETAINED;
#endif

/*
 ****************************************************************************************
 * FUNCTION DEFINITIONS
******************************************************************************************/
#ifdef HAS_HID_REPORT
    static void user_hid_report_clear_lists(void);
    static void user_hid_report_send_full_release_reports(void);
#endif

#ifdef HAS_PWR_MGR
static void user_rcu_reset_inactivity(void)
{
    if(user_reset_inactivity_triggered == false)
    {
        user_pwr_mgr_reset_inactivity();   
        user_reset_inactivity_triggered = true;
    }
}
#endif
/*****************************************************************************************
 * @brief  Send MTU exchange command
******************************************************************************************/
static void user_exchange_mtu(void)
{
    struct gattc_exc_mtu_cmd *pkt = KE_MSG_ALLOC(
        GATTC_EXC_MTU_CMD,
        KE_BUILD_ID(TASK_GATTC, app_env[last_connection_idx].conidx),
        TASK_APP, gattc_exc_mtu_cmd
    );

    if (pkt)
    {
        #if (RWBLE_SW_VERSION_MAJOR >= 8)    
            pkt->operation = GATTC_MTU_EXCH;
        #else
            pkt->req_type = GATTC_MTU_EXCH;
        #endif
            ke_msg_send(pkt);
    }
}

#ifdef HAS_SPECIAL_ADVERTISING
/*****************************************************************************************
 * @brief  Send a special advertising packet to wake-up the host
******************************************************************************************/
static void user_wake_up_peer(void)
{
    uint8_t headerfooter[] = {'W','K','U','P','!'};
    uint8_t user_special_data[ADV_DATA_LEN - 3];
    uint8_t user_special_length;
    uint8_t *pDat;

    pDat = user_special_data;
    
    // Skip the length field for now
    ++pDat;
    
    // Put the manufacturer specific tag
    *pDat++ = GAP_DATA_TYPE_MANUFACTURER_SPEC;
    
    // Copy the sample string as preamble
    memcpy(pDat, headerfooter, sizeof(headerfooter));
    pDat = pDat + sizeof(headerfooter);
    
    // Copy the peer's mac address to the packet
    memcpy(pDat, app_con_fsm_get_peer_addr(), sizeof(struct bd_addr));
    pDat = pDat + sizeof(struct bd_addr);
    
    // Copy the sample string as postamble
    memcpy(pDat, headerfooter, sizeof(headerfooter));
    pDat = pDat + sizeof(headerfooter);
    
    // Update the wake up packet length
    user_special_length = pDat - user_special_data;

    *user_special_data = user_special_length-1;
    
    #ifdef HAS_SPECIAL_ADVERTISING  
        app_con_fsm_send_special_packet(user_special_data, user_special_length);
    #endif
    
}
#endif

bool user_is_conn_upd_pending(void)
{
#ifdef HAS_CONNECTION_FSM
    return app_con_fsm_is_conn_upd_pending();
#else
    return user_conn_update_pending;
#endif        
}


// User application callback functions

#ifdef HAS_CONNECTION_FSM
void user_on_update_params_rejected(const uint8_t status)
{
    switch (status) {
    #if (RWBLE_SW_VERSION_MAJOR >= 8)         
        case LL_ERR_UNSUPPORTED_LMP_PARAM_VALUE:
            if(app_con_fsm_is_conn_upd_pending()) {
                app_con_fsm_send_connection_upd_req(false);
                // Do this once and go to CONNECTED state
                app_con_fsm_state_update(CONN_UPD_RESP_EVT);
            }
            break;
    #endif        
            
        case GAP_ERR_REJECTED:
            // it's application specific what to do
            // when the Param Upd request is rejected

            // Go to Connected State
            app_con_fsm_state_update(CONN_UPD_RESP_EVT);
            break;
    #if (RWBLE_SW_VERSION_MAJOR >= 8) 
        case LL_ERR_LMP_COLLISION:
            break;
    #endif        
        default:
            ASSERT_INFO(0, status, APP_PARAM_UPD);
            // Disconnect
            // port_con_fsm_disconnect();
            break;
    }
}

void user_on_update_params_complete(void)
{
    app_con_fsm_state_update(CONN_UPD_RESP_EVT);
}

void user_on_set_dev_config_complete(void)
{
    // Add the first required service in the database
    if (app_db_init_start())
    {
        app_con_fsm_state_update(INIT_EVT);
    }
}

void user_store_ccc(uint16_t uuid, int attr_num, int value)
{
    port_alt_pair_store_ccc(uuid, attr_num, value);
}

    #if ((RWBLE_SW_VERSION_MAJOR >= 8) && BLE_BATT_SERVER) ||  BLE_BAS_SERVER
    void user_bass_store_ccc(uint8_t conidx, uint8_t ntf_cfg)
    {
        port_alt_pair_store_ccc(ATT_CHAR_BATTERY_LEVEL, 0, ntf_cfg);
    }
    #endif

#else // HAS_CONNECTION_FSM

void user_update_params_timer_cb(timer_hnd hnd)
{
    // When CONNECTION_FSM is not used this function will send the connection
    // parameter request command
    app_easy_gap_param_update_start(0);
    user_conn_update_pending = false;
}

void user_app_adv_undirect_complete(uint8_t status)
{
    // If advertising was canceled then update advertising data and start advertising again
    if (status==GAP_ERR_CANCELED)
    {
        app_easy_gap_undirected_advertise_start();
    }
}

#endif // HAS_CONNECTION_FSM

#if BLE_APP_SEC || defined(HAS_CONNECTION_FSM)
void user_on_encrypt_ind(const uint8_t auth)
{   
    #ifdef HAS_KBD               
    app_kbd_start_reporting(); // start sending notifications    
    #endif    
}
#endif

void user_on_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
    last_connection_idx = connection_idx;
    
#ifdef HAS_CONNECTION_FSM
	peer_addr_t addr;
	
	addr.addr_type = param->peer_addr_type;
	addr.addr = param->peer_addr;

    app_env[connection_idx].connection_active = true;
           
    // Retrieve the connection info from the parameters (MUST always be done!)
    app_env[connection_idx].conhdl = param->conhdl;
    app_env[connection_idx].peer_addr_type = param->peer_addr_type;
    app_env[connection_idx].peer_addr = param->peer_addr;
    
    if (app_env[connection_idx].conidx == GAP_INVALID_CONIDX) {    
        return;
    }
    
    ke_state_set(TASK_APP, APP_CONNECTED); // Update TASK_APP state (MUST always be done!)
    
    // connection confirmation is handled by app_con_fsm_connection_validation    
    if (app_con_fsm_connection_validation(&addr) == false) {
        return;
    }  

    app_prf_enable(connection_idx);
#else
    default_app_on_connection(connection_idx, param);
    app_easy_timer(500, user_update_params_timer_cb);
    user_conn_update_pending = true;
#endif
    
    // Send an MTU exchange command. This is needed for some hosts which do not
    // initialte the MTU exchange procedure
    user_exchange_mtu();

#if ((RWBLE_SW_VERSION_MAJOR >= 8) && BLE_BATT_SERVER) ||  BLE_BAS_SERVER
    app_batt_poll_start(BATTERY_LEVEL_POLLING_PERIOD/10); // Start polling
#endif
        
#ifdef HAS_AUDIO
    // Store the connection parameters
    user_connection_params.con_interval = param->con_interval;
    user_connection_params.con_latency  = param->con_latency;
    user_connection_params.sup_to       = param->sup_to;
    // Calculete the audio packet size for the active connection interval
    user_audio_set_packet_size();
#endif
}

void user_on_disconnect( struct gapc_disconnect_ind const *param )
{
    uint8_t state = ke_state_get(TASK_APP);
    
#if (RWBLE_SW_VERSION_MAJOR >= 8)
    if (state==APP_CONNECTED)
#else
    if ((state==APP_CONNECTED) || (state==APP_PARAM_UPD) || (state==APP_SECURITY))
#endif        
    {
        skip_slave_latency_flag = false;  
        
        // Perform actions required by modules on disconnection. Set the corresponding 
        // mudule functions in user_module_config structure.
        user_modules_on_disconnect();     
        
#ifdef AUDIO_TEST_MODE
        user_rcu_audio_start_audio_test(false);
#endif
        
#ifndef HAS_CONNECTION_FSM
        default_app_on_disconnect(param);
    #ifdef HAS_PWR_MGR
        user_pwr_mgr_disable_inactivity();
    #endif        
#endif

        ke_state_set(TASK_APP, APP_CONNECTABLE); // APP_CONNECTABLE means "Idle"

#if ((RWBLE_SW_VERSION_MAJOR >= 8) && BLE_BATT_SERVER) ||  BLE_BAS_SERVER
        app_batt_poll_stop(); // stop battery polling
#endif

#if (BLE_SUOTA_RECEIVER) || (BLE_SPOTA_RECEIVER)
        user_suota_in_progress = false;
        // Issue a platform reset when it is requested by the suotar procedure
        if (suota_state.reboot_requested) {
            // Reboot request will be served
            suota_state.reboot_requested = 0;

            // Platform reset
            platform_reset(RESET_AFTER_SUOTA_UPDATE);
        }
#endif
    }
    // There is an extreme case where this message is received twice. This happens when 
    // both the device and the host decide to terminate the connection at the same time.
    // In this case, when the device sends the LL_TERMINATE_IND, it also gets the same
    // message from the Host. This is not an erroneous situation provided that the 
    // device has already cleared its state.
    //    else
    //        ASSERT_ERROR(0);
}
 
void user_on_db_init_complete(void)
{
#ifdef HAS_AUDIO
    // Get the handles of the corresponding HID reports or custom audio data characteristic 
    // value. These handles will be used for sending notifications directly to L2CAP, 
    // bypassing ATT and HOGPD or custom audio profile layers
    user_audio_get_handles();
    user_audio_set_config_data();
#endif
    
#if defined(HAS_MOTION) || defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
    // Get the handles of the corresponding HID reports. These handles will be used
    // for sending notifications directly to L2CAP, bypassing HOGPD and ATT layers
    user_motion_get_handles();
#endif
    
#ifdef HAS_CONNECTION_FSM
    app_con_fsm_state_update(INIT_EVT);
//    app_con_fsm_state_update(USER_EVT); // Start advertising
#else
    default_app_on_db_init_complete();
#endif
}

#if (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER)
    // SUOTA Functions

    /**
     ****************************************************************************************
    * @brief Prepare system for SUOTA
    ****************************************************************************************
    */
    static void user_suota_started(void)
    {
        USER_LEDS_RAMP(led_suota_start_param);
    #ifdef HAS_PWR_MGR    
        user_pwr_mgr_disable_inactivity();
    #endif
        
        skip_slave_latency_flag = true;
        user_suota_in_progress = true;
        // Configure a timeout timer. If there is no SUOTA action during this time SUOTA will 
        // be stopped.
        port_timer_set(APP_SUOTA_TIMEOUT_TIMER, TASK_APP, 2000);
    }

/*****************************************************************************************
 * @brief Perform actions after SUOTA has ended
******************************************************************************************/
static void user_suota_ended(void)
{
    USER_LEDS_OFF(led_suota_start_param);
#ifdef HAS_PWR_MGR
    user_rcu_reset_inactivity();
#endif
    
    skip_slave_latency_flag = false;  
    user_suota_in_progress = false;
    
    // Set the timer to request connection parameter update 
    port_timer_set(APP_SUOTA_TIMEOUT_TIMER, TASK_APP, 2000);
}

void user_suota_timeout_timer_handler(void)
{
    if(user_suota_in_progress == true) {
        // Timeout has expired. Stop SUOTA
        user_suota_ended();
    }
#ifdef HAS_CONNECTION_FSM
    else {
        // restore connection parameters in case they have been modified during SUOTA
        app_con_fsm_send_connection_upd_req(true);    
    }
#endif
}


void user_on_suotar_status_change(const uint8_t suotar_event)
{
    switch(suotar_event) {
    case SUOTAR_START:
        user_suota_started();
        break;
    case SUOTAR_END:
        user_suota_ended();
        break;
    case SUOTAR_IN_PROGRESS:
        // SUOTA in progress. Reset timeout timer
        port_timer_set(APP_SUOTA_TIMEOUT_TIMER, TASK_APP, 2000);
        break;
    default:
        ASSERT_WARNING(0);
    }
}
#endif // (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER)

#if (RWBLE_SW_VERSION_MAJOR >= 8)
void user_app_on_get_dev_appearance(uint16_t* appearance)
{
    *appearance = BLE_GAP_APPEARANCE_GENERIC_REMOTE_CONTROL; // Remote Control
}

void user_app_on_get_dev_slv_pref_params(struct gap_slv_pref* slv_params)
{   
    slv_params->con_intv_min  = user_connection_param_conf.intv_min;
    slv_params->con_intv_max  = user_connection_param_conf.intv_max;
    slv_params->slave_latency = user_connection_param_conf.latency;
    slv_params->conn_timeout  = user_connection_param_conf.time_out;
}

void user_on_data_length_change(struct gapc_le_pkt_size_ind *ind)
{
    #ifdef HAS_AUDIO
    // Recalculate the audio packet size using the new packet length
    user_audio_set_packet_size();
    #endif        
}
#endif

void user_process_catch_rest(
    ke_msg_id_t const msgid, void const *param,
    ke_task_id_t const dest_id, ke_task_id_t const src_id
) 
{
#if (BLE_HID_DEVICE)
    // Process the messages sent to HOGPD
    if(app_hogpd_process_handler(msgid, param, dest_id, src_id) == PR_EVENT_HANDLED) {
        return;
    }
#endif
    
#ifdef HAS_AUDIO    
    #ifdef AUDIO_USE_CUSTOM_PROFILE
    // Process the messages sent to Custom audio profile
    if(user_audio_process_custom_profile_handler(msgid, param) == PR_EVENT_HANDLED) {
        return;
    }
    #endif

    if(((enum gapc_msg_id)msgid) == GAPC_PARAM_UPDATED_IND) {
        user_connection_params = *((struct gapc_param_updated_ind const *) param);
        // Connection parameters have changed. 
        // Recalculate the audio packet size using the new connection interval
        user_audio_set_packet_size();
        // Report the new connection parameters to the host
        user_audio_send_conn_params_report();
        return;
    }

    #if (RWBLE_SW_VERSION_MAJOR >= 8)
    if(((enum gattc_msg_id)msgid) == GATTC_MTU_CHANGED_IND) {
        // MTU has changed. 
        // Recalculate the audio packet size using the new MTU
        user_audio_set_packet_size();
        return;
    }
    #else
        #warning "GATTC_MTU_CHANGED_IND indication implementation for 580 is pending"
    #endif
#endif    
    
#ifdef HAS_CONNECTION_FSM        
    if(msgid == GAPC_ENCRYPT_IND)
    {
        user_on_encrypt_ind(((struct gapc_encrypt_ind *)param)->auth);
        // Do not return here. Let port_security_process_handler() handle this message as well
    }
    
    // Process the messages exchanged during pairing
    if(port_security_process_handler(msgid, param, KE_IDX_GET(src_id)) == PR_EVENT_HANDLED)
    {
        return;
    }

    // Process the messages needed by Connection FSM
    if(port_con_fsm_process_handler(msgid, param) == PR_EVENT_HANDLED)
    {
        return;
    }
#endif    
}

void user_action_triggered(void) 
{
    // A user action such as a key press or pointer movement has been detected
#ifdef HAS_CONNECTION_FSM    
    // Notify the connection FSM to start advertise if required
    if (app_con_fsm_get_state() == IDLE_ST) {
        app_con_fsm_state_update(USER_EVT);
    } 
    else {
        user_sync_key_press_evt = true; // synchronize the call to app_con_fsm_state_update(USER_EVT);
    }
#endif
    
#ifdef HAS_PWR_MGR
    // Reset inactivity timeout
    user_rcu_reset_inactivity();
#endif
}

/*
 ****************************************************************************************
 * Main loop callback functions
******************************************************************************************/

void user_on_init(void)
{    
    dbg_puts(DBG_APP_LVL, "\r\n\n\n");
#ifdef HAS_MOTION
    port_delay_usec(4000); // wait for BMI to power up

    #if 0   
    // print the chip ID of the BMI to verify that the sensor has been powered-up properly
    extern struct bmi160_t s_bmi160;
    port_motion_wakeup();
    dbg_printf(DBG_APP_LVL, "BMI chip ID is %02X\r\n",s_bmi160.chip_id);
    #endif  
#endif

#ifdef HAS_PWR_MGR
    user_pwr_mgr_init();
#endif
  
#ifdef HAS_WKUP
     port_wkup_init();
#endif
    
    // Initialize user modules. Set mudule initialization functions in 
    // user_module_config structure 
    user_modules_init();
    
    default_app_on_init();

    system_initialized = true;
    
#ifdef HAS_CONNECTION_FSM
    app_prf_srv_perm_t access_rights = app_con_fsm_get_access_rights();
#else
    const app_prf_srv_perm_t access_rights = SRV_PERM_ENABLE;
#endif

    // Set perimssion for services
#if (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER) || !defined(HAS_CONNECTION_FSM)
    // If SUOTA is used then enable the profile without security
    app_set_prf_srv_perm(APP_TASK_ID_DISS, SRV_PERM_ENABLE);
#else
    app_set_prf_srv_perm(APP_TASK_ID_DISS, access_rights);     
#endif
    
#if (((RWBLE_SW_VERSION_MAJOR >= 8) && BLE_BATT_SERVER) ||  BLE_BAS_SERVER)
    app_set_prf_srv_perm(APP_TASK_ID_BASS, access_rights);
#endif

#if BLE_HID_DEVICE 
    app_set_prf_srv_perm(APP_TASK_ID_HOGPD, access_rights); 
#endif    

#if BLE_CUSTOM1_SERVER
    app_set_prf_srv_perm(APP_TASK_ID_CUSTS1, access_rights); 
#endif

#if (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER)
    app_set_prf_srv_perm(APP_TASK_ID_SUOTAR, access_rights); 
#endif
}

arch_main_loop_callback_ret_t user_on_ble_powered(void)
{
    // Synchronize with the BLE here! The time window of requesting a packet trm at the upcoming
    // anchor point is from the CSCNT event until the FINEGTIM event. If you pass the FINEGTIM
    // event then the packet will be sent at the next anchor point!
    // Note that this synchronization is only possible in sleep modes!

	arch_main_loop_callback_ret_t ret;
    
    // Perform actions needed by modules while BLE is powered.
    // Set mudule functions in user_module_config structure.
    // Modules can block sleep by returning true
    ret = user_modules_on_ble_powered() ? KEEP_POWERED : GOTO_SLEEP;
                
#ifdef HAS_CONNECTION_FSM
    if (user_sync_key_press_evt == true) {
        app_con_fsm_state_update(USER_EVT);
        user_sync_key_press_evt = false;
    }
        
    if(app_con_fsm_get_state() == POWEROFF_ST) {
    #ifdef HAS_HID_REPORT        
        if(app_hid_report_is_report_list_empty()) 
    #endif        
        {
            app_con_fsm_state_update(SHUTDOWN_EVT);
    #ifdef HAS_PWR_MGR
            user_pwr_mgr_disable_inactivity();
    #endif		
        }
    }
#endif
            
    if (ret == KEEP_POWERED) {
        wdg_reload(WATCHDOG_DEFAULT_PERIOD);
    }
    
    user_reset_inactivity_triggered = false;
    
	return ret;
}
        
arch_main_loop_callback_ret_t user_on_system_powered(void)
{
	arch_main_loop_callback_ret_t ret;
    uint8_t power_status;

    // Perform actions needed by modules while BLE is powered.
    // Set mudule functions in user_module_config structure
    // Modules can block sleep by returning APP_GOTO_SLEEP or 
    // force BLE to wake up by returning APP_BLE_WAKEUP
    power_status = user_modules_on_system_powered();
    
    ret = (power_status == APP_KEEP_POWERED) ? KEEP_POWERED : GOTO_SLEEP;
    
    if ((power_status & APP_BLE_WAKEUP) != 0 && user_is_ble_connected()) {
        if(GetBits16(CLK_RADIO_REG, BLE_ENABLE) == 0) {
            // If BLE is sleeping, wake it up!
            ret = arch_ble_force_wakeup() ? KEEP_POWERED : GOTO_SLEEP;
#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)                        
            if(ret == KEEP_POWERED) {
                user_mouse_send_report(false); // send the first report
            }
#endif       
        }
        else {
            ret = KEEP_POWERED;
        }
    }
       
    if(ret == KEEP_POWERED) {
        wdg_reload(WATCHDOG_DEFAULT_PERIOD);
    }
    
    return ret;
}

void user_before_sleep(void)
{
#ifdef HAS_KBD    
    port_kbd_scan_before_sleep();
#endif    
}

sleep_mode_t user_validate_sleep(sleep_mode_t sleep_mode)
{
#ifdef HAS_AUDIO
    extern bool user_audio_samples_available;
    if(user_audio_samples_available == true) {
        // block power-off while there are audio samples to process
        // Make sure that the samples are processed before going to idle
        return mode_active;
    }
#endif 

#ifdef HAS_KBD        
    if ( app_kbd_scan_is_active() ) {
        // block power-off while keyboard scanning is active
        return mode_idle;
    }
#endif    
    
#ifdef HAS_IR
    if(app_ir_is_active() == true) {
        // block power-off while IR is active
        return mode_idle; 
    }
#endif    

#ifdef HAS_ACTION_INACTIVITY_TIMEOUT
    if(user_activity_period_expired == false) {
        // block power-off while there is user activity
        return mode_idle;
    }    
#endif     
    return sleep_mode;
}

void user_going_to_sleep(sleep_mode_t sleep_mode)
{
    if ( sleep_mode == mode_idle)
    {
        syscntl_use_lowest_amba_clocks();
    }
}

void user_resume_from_sleep(void)
{
    syscntl_use_highest_amba_clocks();    
}    

/*****************************************************************************************
 * Callback functions
******************************************************************************************/
void user_update_attr_callback(uint16_t uuid, int attr_num, int value)
{
    // Attributes have been read from the NV PROM.
    // The database must be updated.
    switch (uuid) {
#if (RWBLE_SW_VERSION_MAJOR >= 8)
    #if (BLE_BATT_SERVER)        
    case ATT_CHAR_BATTERY_LEVEL: 
        {
            struct bass_env_tag* bass_env = PRF_ENV_GET(BASS, bass);

            // Update Battery Level
            if (value) {
                bass_env->ntf_cfg[0] |= 0x01;
            } else {
                bass_env->ntf_cfg[0] &= ~0x01;
            }
        }
        break;
    #endif        
#elif (BLE_BAS_SERVER)
    case ATT_CHAR_BATTERY_LEVEL: 
        if (value) {
            bass_env.features[0] |= BASS_FLAG_NTF_CFG_BIT;
        }   
        else {
            bass_env.features[0] &= ~BASS_FLAG_NTF_CFG_BIT;
        }
        break;
#endif     
#if (BLE_HID_DEVICE)
    case ATT_CHAR_PROTOCOL_MODE:
    // Update Protocol mode
    {
    #if (RWBLE_SW_VERSION_MAJOR >= 8)      
        struct hogpd_env_tag* hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
        hogpd_env->svcs[0].proto_mode = value ? HOGP_REPORT_PROTOCOL_MODE : HOGP_BOOT_PROTOCOL_MODE;
    #else
        hogpd_env.proto_mode[0] = value ? HOGP_REPORT_PROTOCOL_MODE
                                        : HOGP_BOOT_PROTOCOL_MODE;
    #endif    
        break;
    }
    case ATT_CHAR_BOOT_KB_IN_REPORT:
    // Update Boot report
    {
    #if (RWBLE_SW_VERSION_MAJOR >= 8)      
        struct hogpd_env_tag* hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
        if (value) {
            hogpd_env->svcs[0].ntf_cfg[0] |= HOGPD_CFG_KEYBOARD;
        } else {
            hogpd_env->svcs[0].ntf_cfg[0] &= ~HOGPD_CFG_KEYBOARD;
        }
    #else
        if (value) {
            hogpd_env.features[0].svc_features |= HOGPD_REPORT_NTF_CFG_MASK;
        }   
        else {
            hogpd_env.features[0].svc_features &= ~HOGPD_REPORT_NTF_CFG_MASK;
        }
    #endif
        break;
    }
    case ATT_CHAR_REPORT:
    {
    #if (RWBLE_SW_VERSION_MAJOR >= 8)      
        struct hogpd_env_tag* hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
        ASSERT_ERROR(attr_num < HOGPD_NB_REPORT_INST_MAX);
        if (value) {
            hogpd_env->svcs[0].ntf_cfg[0] |= (HOGPD_CFG_REPORT_NTF_EN << attr_num);
        }
        else {
            hogpd_env->svcs[0].ntf_cfg[0] &= ~(HOGPD_CFG_REPORT_NTF_EN << attr_num);
        }
    #else
        if (value) {
            hogpd_env.features[0].report_char_cfg[attr_num] |= HOGPD_REPORT_NTF_CFG_MASK;
        }
        else {
            hogpd_env.features[0].report_char_cfg[attr_num] &= ~HOGPD_REPORT_NTF_CFG_MASK;
        }
    #endif
        break;
    }
    case ATT_CHAR_HID_CTNL_PT:      
#endif
    case ATT_CHAR_SERVICE_CHANGED:  
    default:
        break;
    }
}

#ifdef HAS_CONNECTION_FSM
void user_con_fsm_callback(enum con_fsm_state_update_callback_type type)
{
    switch (type) {
        
    case INDICATE_IDLE:
    #ifdef HAS_LED_INDICATORS            
        app_leds_init();
    #endif    
    #ifdef HAS_PWR_MGR
        user_pwr_mgr_disable_inactivity();
    #endif
    #ifdef HAS_DEEPSLEEP        
        #if (RWBLE_SW_VERSION_MAJOR >= 8) 
        arch_set_deep_sleep(false);
        #else            
        arch_set_deep_sleep();
        #endif            
    #endif    
    #ifdef HAS_KBD    
        app_kbd_scan_enable_delayed_scanning(true);
    #endif        
        arch_ble_ext_wakeup_on();
        break;
    
    case INDICATE_EXIT_IDLE:
        arch_ble_force_wakeup();
        arch_ble_ext_wakeup_off();
    #ifdef HAS_DEEPSLEEP  
        arch_set_sleep_mode(ARCH_EXT_SLEEP_ON);
    #endif        
        break;
    
    case INDICATE_REINITIALIZE:
    #ifdef HAS_KBD
        app_kbd_flush_buffer();
    #endif
    #ifdef HAS_HID_REPORT
        user_hid_report_clear_lists();
    #endif    
        break;

    case INDICATE_START_PARAM_UPDATE:
    #ifdef HAS_KBD        
        app_kbd_start_reporting(); // start sending notifications
    #endif    
        // Clear all buffers if reporting "old" keys is not wanted
    #if defined(HAS_HID_REPORT) && !defined(HID_REPORT_HISTORY)
        #ifdef HAS_KBD        
            app_kbd_flush_buffer();
        #endif    
            user_hid_report_clear_lists();
    #endif
        break;
        
    case INDICATE_START_PASSCODE:
    #ifdef HAS_PWR_MGR
        user_rcu_reset_inactivity();
    #endif        
    #ifdef HAS_KBD        
        app_kbd_start_passcode(); // Set to 'Passcode' mode until the connection is established        
    #endif    
        break;
    
    case INDICATE_CONNECTION_IN_PROGRESS:
        USER_LEDS_RAMP(led_connection_in_progress_param);
        break;
    
    case INDICATE_DISCONNECTED:
    #ifdef HAS_SOUND_INDICATION
        app_buzzer_play_melody(BUZZER_DISCONNECTED_MELODY);
    #endif
    #ifdef HAS_KBD    
        user_rcu_kbd_turn_off_leds();
    #endif    
        USER_LEDS_OFF(led_connection_in_progress_param);
        USER_LEDS_OFF(led_advertise_param);
        USER_LEDS_OFF(led_motion_kbd_page_param);
    #if (BLE_SPOTA_RECEIVER) || (BLE_SUOTA_RECEIVER)
        USER_LEDS_OFF(led_suota_start_param);
    #endif        
        USER_LEDS_RAMP(led_disconnected_param);
        break;
    
    case INDICATE_CONNECTED:
    #ifdef HAS_PWR_MGR
        user_rcu_reset_inactivity();
    #endif        
    #ifdef HAS_SOUND_INDICATION
        app_buzzer_play_melody(BUZZER_CONNECTED_MELODY);
    #endif
        USER_LEDS_OFF(led_advertise_param);
        USER_LEDS_OFF(led_connection_in_progress_param);
        USER_LEDS_RAMP(led_connected_param);        
    #ifdef HAS_KBD
        user_rcu_kbd_turn_on_leds();
        extern bool user_fn_locked; 
        if(user_fn_locked) {
        #ifdef HAS_MOTION
            user_motion_start();
        #endif    
            USER_LEDS_RAMP(led_motion_kbd_page_param);
        }
    #endif     
        // Send a connection parameter update request very early 
        // to allow motion devices to work smoothly. 
        // Another request will be sent later by CON_FSM.
        app_con_fsm_send_connection_upd_req(false);
        break;
        
    case INDICATE_ADVERTISING_START:
        USER_LEDS_RAMP(led_advertise_param);
        break;
    
    case INDICATE_ADVERTISING_END:
        USER_LEDS_OFF(led_advertise_param);
        break;
    
    default:
        break;
    }
}
#endif

void user_flash_erase_callback(void) 
{
    if(system_initialized == true && GetBits16(CLK_RADIO_REG, BLE_ENABLE) == 1 &&
       GetBits32(BLE_DEEPSLCNTL_REG, DEEP_SLEEP_STAT) == 0 &&
       !(rwip_prevent_sleep_get() & RW_WAKE_UP_ONGOING)) {
        rwip_schedule();  
    }    
}

#ifdef HAS_PWR_MGR
void user_pwr_mgr_inactivity_callback(void)
{
    // Inactivity timeout has expired.
    // Stop everything and and go to sleep
    #ifdef HAS_KBD        
    app_kbd_stop();
    #endif       
    
    #ifdef HAS_HID_REPORT
    user_hid_report_send_full_release_reports();
    #endif
    
    #ifdef HAS_AUDIO
    user_audio_stop(false);
    #endif    
    
    #ifdef HAS_MOTION
    user_motion_stop();
    #endif    

    #ifdef HAS_CONNECTION_FSM    
    app_con_fsm_state_update(POWEROFF_EVT); 
    #endif    
}
#endif

#ifdef HAS_ACTION_INACTIVITY_TIMEOUT
/*
 ****************************************************************************************
 * Action inactivity functions
******************************************************************************************/

void user_systick_callback(void)
{
    port_systick_stop(SYSTICK_ACTION_INACTIVITY_CHANNEL);
    user_activity_period_expired = true;
#ifdef HAS_GPIO_KEYS    
    port_gpio_keys_enable_irq();
#endif    
}

void user_initiate_activity_period(void)
{
    port_systick_restart(SYSTICK_ACTION_INACTIVITY_CHANNEL, ACTION_INACTIVITY_TIMEOUT_IN_US); 
    user_activity_period_expired = false;
    user_action_triggered();    
}

void user_wakeup_activity_callback(void)
{
    user_initiate_activity_period();    
}
#endif

#ifdef HAS_HID_REPORT

    #ifdef HAS_KBD

/*****************************************************************************************
 * \brief Send HID keyboard reports to BLE. This function checks keycode_buffer to create
 *        more HID reports if possible.
 * \return KEEP_POWERED if there are reports that must be sent to BLE, otherwise
 *         GOTO_SLEEP.
******************************************************************************************/
static bool user_hid_report_prepare_keyreports(void)
{
    // The report will be placed in the HID FIFO. It's the application's task to clear the
    // list in case of disconnection with this host that is followed by a connection
    // establishment to a new host.
    
    bool ret = false;
    uint32_t keycode;
    
    while(app_hid_report_is_report_list_full() == false) {
        
        keycode = app_kbd_get_key_code();
        
        if(keycode == KBD_KEY_BUFFER_EMPTY) {
            break;
        }
        if(keycode == KBD_KEY_FULL_RELEASE) {
            user_hid_report_send_full_release_reports();
            ret = true;
        }
        
        bool pressed = keycode & KBD_KEY_PRESSED_FLAG;
        
        if((keycode & 0xFC00) == 0xF400) {
            uint8_t byte = HOGP_GET_BYTE_FROM_CODE(keycode);
            uint8_t mask = HOGP_GET_MASK_FROM_CODE(keycode);
            
            app_hid_report_create_extended_report(byte, mask, pressed ? mask : 0);
        }
        else {
            app_hid_report_modify_kbd_keyreport(keycode & 0xFFFF, pressed);
        }
        
    }            
    return ret;
}
    #endif

/*****************************************************************************************
 * \brief   Clear HID FIFO lists and pending full release reports
******************************************************************************************/
static void user_hid_report_clear_lists(void)
{
        app_hid_report_clear_lists();
        user_hid_report_normal_full_release_pending = false;
        user_hid_report_extended_full_release_pending = false;
}

bool user_hid_report_on_ble_powered(void)
{   
    // Send HID keyboard reports to BLE. This function checks keycode_buffer to create
    // more HID reports if possible.
    bool ret = false;

    if ( !ke_event_get(KE_EVENT_KE_MESSAGE) ) {
        // Since pkt reqs can be silently discarded if no Tx bufs are available, check first!
        if (user_is_ble_connected() && l2cm_get_nb_buffer_available())  {
            // prepare a new report if possible

    #ifdef HAS_KBD
            user_hid_report_prepare_keyreports();
    #endif
            
            // add a "full release" report in the HID FIFO list.
            if(user_hid_report_normal_full_release_pending == true) {
                if(app_hid_report_add_full_release_report()) {
                    user_hid_report_normal_full_release_pending = false;
                }
            }

            // Add an "full release" extended key report
            if(user_hid_report_extended_full_release_pending == true) {
                if(app_hid_report_add_report(HID_REPORT_EXTENDED_REPORT_IDX, EXTENDED, NULL, HID_REPORT_EXTENDED_REPORT_SIZE)) {
                    user_hid_report_extended_full_release_pending = false;
                }
            }

            switch(app_hid_report_send_report()) {
            case HID_REPORT_FAILED:
                ret = true;
                break;
            case HID_REPORTS_AVAILABLE:
                // One HID report is removed from the HID FIFO. Check if other HID reports are to be
                // prepared because of unread data in the keycode_buffer now that the free list is not NULL.
    #ifdef HAS_KBD
                user_hid_report_prepare_keyreports();
    #endif
                ret = true; // because list is not empty
                break;
            case HID_LAST_REPORT_SENT:
                // One HID report is removed from the HID FIFO. Check if other HID reports are to be
                // prepared because of unread data in the keycode_buffer now that the free list is not NULL.
    #ifdef HAS_KBD    
                if(user_hid_report_prepare_keyreports() == true) { 
                    // if no new report has been created do not keep the system powered on 
                    // because there are no more reports in the list
                    ret = true;
                }
    #endif
                break;
            case HID_REPORT_EMPTY: // list is empty. Go to sleep
                break;
            }
        }
    }   
    return ret;    
}

/*****************************************************************************************
 * \brief   Send full release reports for both normal and extended reports
******************************************************************************************/
static void user_hid_report_send_full_release_reports(void)
{   
    user_hid_report_normal_full_release_pending = true;
    user_hid_report_extended_full_release_pending = true;
}

#endif
/*
 ****************************************************************************************
 * Functions called from rwble.c
******************************************************************************************/

void user_event_isr(void)
{
#ifdef HAS_BLE_STREAM
    extern uint8_t stream_cpt_event;
    stream_cpt_event = 1;        
#endif    

#ifdef HAS_MOTION 
    extern bool motion_cpt_event;
    motion_cpt_event = true;
#endif      

#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
    extern bool mouse_cpt_event;
    mouse_cpt_event = true;
#endif
    
#ifdef DEBUG_EMULATE_PACKET_LOSS
    port_handle_radio_ctrl();
#endif       
}

void user_cscnt_isr(void) 
{
#ifdef DEBUG_EMULATE_PACKET_LOSS
    port_handle_radio_ctrl();
#endif    
}

void user_rf_diag(void)
{
#ifdef HAS_BLE_STREAM
    extern bool transmitting_data;
    transmitting_data = true;
#endif  
}

/**
 * \}
 * \}
 * \}
 */
