/******************************************************************************************
 *
 * \file app_con_fsm_defs.h
 *
 * \brief Connection FSM module definitions
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 *****************************************************************************************
 */

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup APP_CON_FSM
 *
 * \brief Connection FSM definitions
 * \{
 ****************************************************************************************	 	 
 */

#ifndef APP_CON_FSM_DEFS_H_
#define APP_CON_FSM_DEFS_H_

#include "stdint.h"
#include "co_bt.h"

enum con_fsm_state_update_callback_type {
        INDICATE_REINITIALIZE,
        INDICATE_CONNECTED,
        INDICATE_START_PARAM_UPDATE,
        INDICATE_CONNECTION_IN_PROGRESS,
        INDICATE_DISCONNECTED,
        INDICATE_ADVERTISING_START,
        INDICATE_ADVERTISING_END,
        INDICATE_PAIRING_COMPLETED,
        INDICATE_ON,
        INDICATE_IDLE,
        INDICATE_EXIT_IDLE,
        INDICATE_START_PASSCODE,
    INDICATE_PAIR_REQUEST_ACCEPTED
};

typedef void (con_fsm_state_update_callback_t)(enum con_fsm_state_update_callback_type type);
typedef void (con_fsm_attr_update_callback_t)(uint16_t uuid, int attr_num, int value);


enum notification_type {
        ATTR_TYPE,
        CCC_TYPE,
};

typedef struct notification_info_s {
        uint8_t position;             // bit position in the 32-bit word
        uint8_t num_of_bits;          // Number of bits used for the attribute in the 32-bit word
        uint16_t uuid;                // Attribute uuid
        enum notification_type type;  // type, CCC_TYPE or ATTR_TYPE
        uint8_t num_of_atts;          // Number of attributes  
        uint8_t length;               // Attribute length
        uint8_t default_value;        // Default value of the attribute. It must fit in num_of_bits.
} notification_info_t;

enum multi_bond_host_rejection {
        MULTI_BOND_REJECT_NONE,
        MULTI_BOND_REJECT_LAST,
        MULTI_BOND_REJECT_ALL_KNOWN
};

typedef enum {
        IDLE_ST,   
        ADVERTISING_ST,    
        CONNECTED_ST,              
        CONNECTION_IN_PROGRESS_ST,
        CONNECTED_PAIRING_ST,
        DISCONNECTED_INIT_ST,
        POWEROFF_ST,
        WAITING_DISCONNECTION_AFTER_POWEROFF
} main_fsm_states;

enum main_fsm_events {
        INIT_EVT,
        USER_EVT,
        ADV_COMPLETED_EVT,
        PAIRING_REQ_EVT,
        CONN_REQ_EVT,
        CONN_CMP_EVT,
        CONN_IRK_EXCH,
        CONN_CANCELLED_EVT,
        DISCONNECT_CMP_EVT,
        CONN_UPD_RESP_EVT,
        PASSKEY_ENTERED,
        START_PAIRING_EVT,
        SWITCH_EVT,
        NEW_HOST_EVT,
        ALT_PAIR_TIMER_EXP_EVT,
        POWEROFF_EVT,
        SHUTDOWN_EVT,
        POWEROFF_TIMEOUT_EVT,
        PASSCODE_TIMEOUT_EVT,
        PARAM_UPD_TIMEOUT_EVT,
        SPECIAL_ADV_START_EVT,
        SPECIAL_ADV_ENDED_EVT
};

typedef struct peer_addr_s {
        uint8_t addr_type;
        struct bd_addr addr;
} peer_addr_t;
    
typedef enum
{
    APP_CON_FSM_DC_REASON_START_PAIR,
    APP_CON_FSM_DC_REASON_SWITCH_HOST,
} app_con_fsm_disconnect_req_reason_t;

typedef struct {
    uint16_t preferred_conn_interval_min;              
    uint16_t preferred_conn_interval_max;              
    uint16_t preferred_conn_latency;                 
    uint16_t preferred_conn_timeout;                   
} con_fsm_param_update_t;

typedef struct {
    bool disable_bonding_data_storage;
    bool has_multi_bond;
    bool has_privacy;
    bool use_pref_conn_params;
    bool disable_advertise_after_disconnection;
    bool has_passcode_timeout;
    bool has_mitm;
    bool has_white_list;
    bool has_virtual_white_list;
    bool has_security_request_send;
    bool has_usage_counters;
    bool has_nv_rom;
    bool has_smart_rssi_pairing;
    int8_t smart_pairing_rssi_threshold;
    uint32_t enc_safeguard_timeout;            
    uint32_t passcode_timeout;             
    uint32_t notification_timeout;
    uint16_t time_to_request_param_upd;        
    uint16_t alt_pair_disconn_time;            
    con_fsm_param_update_t param_update;                   
    con_fsm_state_update_callback_t *state_update_callback; 
    con_fsm_attr_update_callback_t *attr_update_callback;
} con_fsm_params_t;

typedef void (*app_con_fsm_disconnect_t)(void);
typedef struct bd_addr* (*app_con_fsm_get_peer_addr_t)(void);
typedef uint8_t (*app_con_fsm_get_peer_addrType_t)(void);
typedef void (*app_con_fsm_conn_upd_req_t)(const con_fsm_param_update_t *params, bool force_min_interval);
typedef void (*app_con_fsm_resolve_address_t)(void);
typedef void (*app_con_fsm_start_security_t)(void);
typedef void (*app_con_fsm_mitm_passcode_report_t)(uint32_t code);
typedef void (*app_con_fsm_connect_confirm_t)(uint8_t auth);
typedef int8_t (*app_con_fsm_get_rssi_t)(void);

typedef struct {
    app_con_fsm_disconnect_t           con_fsm_disconnect;
    app_con_fsm_get_peer_addr_t        con_fsm_get_peeraddr;
    app_con_fsm_get_peer_addrType_t    con_fsm_get_peeraddrtype;
    app_con_fsm_conn_upd_req_t         con_fsm_conn_upd_req;  
    app_con_fsm_resolve_address_t      con_fsm_resolve_address;
    app_con_fsm_start_security_t       con_fsm_start_security;  
    app_con_fsm_mitm_passcode_report_t con_fsm_mitm_passcode_report;
    app_con_fsm_connect_confirm_t      con_fsm_connect_confirm;
    app_con_fsm_get_rssi_t             con_fsm_get_rssi;
}con_util_funcs_t;

#endif // APP_CON_FSM_DEFS_H_

/**
 * \}
 * \}
 * \}
 */
