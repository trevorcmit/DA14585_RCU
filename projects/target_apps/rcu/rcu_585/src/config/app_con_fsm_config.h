/**
 ****************************************************************************************
 *
 * \file app_con_fsm_config.h
 *
 * \brief Connection FSM module configuration header file.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */

#ifndef _APP_CON_FSM_CONFIG_H_
#define _APP_CON_FSM_CONFIG_H_

/**
 ****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup CON_FSM_CFG
 *
 * \brief Connection FSM module configuration
 * \{
 ****************************************************************************************
 */

#include "app_con_fsm_defs.h"
#ifdef CFG_PRF_HOGPD
    #include <user_hogpd_config.h>
#endif
#include <app_adv_fsm_config.h>
#include "port_con_fsm.h"
#include "user_rcu.h"

/**
 ****************************************************************************************
 * Define USE_L2CAP_CONN_UPDATE_REQ if L2CAP_CONN_PARAM_UPDATE_REQ must be sent instead
 * of LL_CONNECTION_PARAM_REQ  
 ****************************************************************************************
 */
//#define USE_L2CAP_CONN_UPDATE_REQ

/**
 ****************************************************************************************
 * \brief Load all bond info into RetRAM at power-up to eliminate subsequent                   
 * Reads and reduce power consumption
 * 0 -> Do not load
 * 1 -> Do load 
 ****************************************************************************************
*/
#define MBOND_LOAD_INFO_AT_INIT 1                   // 0, do not load - 1, do load

/**
 ****************************************************************************************
 * \brief Maximum number of hosts that can be handled by bonding mechanism.
 * The NV memory size required for storing the bonding data is equal to 
 * (16 + no_of_bonds * 60)  
 * If SPI FLASH memory is used then the bonding data must fit in a 256-byte page.
 ****************************************************************************************
 */
#define MAX_BOND_PEER		   3

/**
 ****************************************************************************************
 * \brief When FORCE_CONNECT_TO_HOST_ON is defined force-connect to specific
 *        host is enabled. FORCE_CONNECT_NUM_OF_HOSTS must be defined as well
 ****************************************************************************************
 */
#undef FORCE_CONNECT_TO_HOST_ON

/**
 ****************************************************************************************
 * \brief When FORCE_CONNECT_TO_HOST_ON is defined, FORCE_CONNECT_NUM_OF_HOSTS defines
 *        the maximum number of hosts to which connection can be forced.
 ****************************************************************************************
 */
#define FORCE_CONNECT_NUM_OF_HOSTS  3

/**
 ****************************************************************************************
 * \brief Attribute and CCC values. These values are stored for each bonded host in a 
 * 32-bit word together with the bonding data. These values are restored when the device
 * is connected to the host. Refer to notification_info_t documentation for details.
 ****************************************************************************************
 */
static const notification_info_t notification_info[] = {
#ifdef CFG_PRF_BASS
    {0, 2, ATT_CHAR_SERVICE_CHANGED,    CCC_TYPE,   1,                  2, 0},
#endif                                                                      
    {2, 1, ATT_CHAR_BATTERY_LEVEL,      CCC_TYPE,   1,                  2, 0},
#ifdef CFG_PRF_HOGPD                                                    
    {3, 1, ATT_CHAR_PROTOCOL_MODE,      ATTR_TYPE,  1,                  1, 1},
    {4, 1, ATT_CHAR_BOOT_KB_IN_REPORT,  CCC_TYPE,   1,                  2, 0},
    {5, 1, ATT_CHAR_HID_CTNL_PT,        ATTR_TYPE,  1,                  1, 0},
    {6, 1, ATT_CHAR_REPORT,             CCC_TYPE,   HID_NUM_OF_REPORTS, 2, 0},
#endif    
};

/**
 ****************************************************************************************
 * \brief Connection FSM Configuration Parameters
 ****************************************************************************************
 */
static const con_fsm_params_t con_fsm_params = {
/**
 ****************************************************************************************
 * Do not store bonding data in the NV PROM
 ****************************************************************************************
 */
    .disable_bonding_data_storage          = false,

/**
 ****************************************************************************************
 * Bond to multiple hosts
 ****************************************************************************************
 */
    .has_multi_bond                        = true,

/**
 ****************************************************************************************
 * Use Privacy (implementation pending) 
 ****************************************************************************************
 */
    .has_privacy                           = false,

/**
 ****************************************************************************************
 * Send a ConnUpdateParam request after connection completion 
 ****************************************************************************************
 */
    .use_pref_conn_params                  = true,

/**
 ****************************************************************************************
 * After disconnection do not start advertise
 ****************************************************************************************
 */
    .disable_advertise_after_disconnection = false,

/**
 ****************************************************************************************
 * Enable timeout checking during PassCode entry
 ****************************************************************************************
 */
    .has_passcode_timeout                  = true,

/**
 ****************************************************************************************
 * Use MITM authentication mode
 ****************************************************************************************
 */
    .has_mitm                              = false,

/**
 ****************************************************************************************
 * Use Non volatile ROM to store bonding data
 ****************************************************************************************
 */
    .has_nv_rom                            = true,
        
/**
 ****************************************************************************************
 * Use White List when exiting DIRECTED_ADV_ST unsuccessfully.
 *     
 * Note 1: if White List is used, then the Device will be able to bond only to 1 Master!
 *         This can be easily modified according to application's usage scenario.       
 * Note 2: VIRTUAL_WHITE_LIST_ON must NOT be defined! Only 1 White List can exist at a  
 *         time!                                                                        
 ****************************************************************************************
 */
    .has_white_list               = false,
        
/**
 ****************************************************************************************
 * Use a Virtual White List to support also hosts with Resolvable Random Addresses.     
 * All addresses are filtered by SW and not by HW!                                      
 * Note 1: if White List is used, then the Device will be able to bond only to 1 Master!
 *         This can be easily modified according to application's usage scenario.       
 * Note 2: WHITE_LIST_ON MUST NOT be defined! Only 1 White List can exist at a time!    
 * Note 3: Bonding and Encryption MUST be enabled (bonding info is stored in a          
 *         non-volatile storage area).                                                  
 ****************************************************************************************
 */
#ifdef FORCE_CONNECT_TO_HOST_ON    
    .has_virtual_white_list       = true,        
#else    
    .has_virtual_white_list       = false,    
#endif    
/**
 ****************************************************************************************
 * Send a SECURITY_REQUEST when connecting to a Host  
 ****************************************************************************************
 */
    .has_security_request_send    = true,
        
/**
 ****************************************************************************************
 * Use usage counters                                                                   
 * Usage counters are used during the bonding of a new host in order to determine       
 * the oldest entry in the NV RAM that will be used for storing the bonding data        
 * of the new host.                                                                     
 * If usage counters are not used then the next new host will be stored in the next     
 * empty position in the NV RAM. If all positions are used then the new host will       
 * be stored in the last position of the NV RAM                                         
 ****************************************************************************************
 */
    .has_usage_counters           = false,

/**
 ****************************************************************************************
 * \brief Set has_smart_rssi_pairing to true to enable smart RSSI pairing.
 * The smart RSSI pairing feature will enable the device to pair ONLY with hosts
 * that are in close range.
 ****************************************************************************************
 */
    .has_smart_rssi_pairing       = false,
        
/**
 ****************************************************************************************
 * If the SMART RSSI PAIRING feature is enabled, this variable configures the minimum
 * RSSI (threshold) that is required for a host to be accepted when it tries to pair
 ****************************************************************************************
 */
    .smart_pairing_rssi_threshold = -45,
    
/**
 ****************************************************************************************
 * Timeouts                                                                             
 ****************************************************************************************
 */
    
/**
 ****************************************************************************************
 * Time to wait for a PAIRING_REQ or ENC_REQ from the Host when BONDING is enabled in msec
 ****************************************************************************************
 */
    .enc_safeguard_timeout        = 10000,  
                                              
/**
 ****************************************************************************************
 * Time in CONNECTED_NO_PAIR_ST until passcode is entered in msec 
 * (when has_passcode_timeout=true)
 ****************************************************************************************
 */
    .passcode_timeout             = 60000,  
                                              
/**
 ****************************************************************************************
 * Time to wait for the last notifications to be sent to the host before Disconnecting
 * Set to 0 if device can disconnect immediately.
 ****************************************************************************************
 */
    .notification_timeout         = 2000,  
    
/**
 ****************************************************************************************
 * Time to request update of connection parameters in msec 
 * (when use_pref_conn_params=true)
 ****************************************************************************************
 */
    .time_to_request_param_upd    = 3000, 
                                            
/**
 ****************************************************************************************
 * Time to block previous host during a "host-switch" in msec
 ****************************************************************************************
 */
    .alt_pair_disconn_time        = 15000,   

    
/**
 ****************************************************************************************
 * Prefered connection parameters                                                       
 ****************************************************************************************
 */
    .param_update.preferred_conn_interval_min = PARAM_UPDATE_INV_MIN,
    .param_update.preferred_conn_interval_max = PARAM_UPDATE_INV_MAX,
    .param_update.preferred_conn_latency      = PARAM_UPDATE_LATENCY,
    .param_update.preferred_conn_timeout      = PARAM_UPDATE_SUP_TIMEOUT,

/**
 ****************************************************************************************
 * Callback functions                                                                   
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * The callback function that is called when the connection state has changed to 
 * connected, connection in progress, disconnected, off, passcode entry started
 ****************************************************************************************
 */
    .state_update_callback = user_con_fsm_callback,
    
/**
 ****************************************************************************************
 * The callback function that is called when bonding data are read from the non-volatile  
 * memory so that the service database is updated
 ****************************************************************************************
 */
    .attr_update_callback = user_update_attr_callback
};


/**
 ****************************************************************************************
 * \brief Connection FSM Platform API Functions Configuration
 ****************************************************************************************
 */
static const con_util_funcs_t app_con_fsm_funcs = {
    .con_fsm_disconnect           = port_con_fsm_disconnect,
    .con_fsm_get_peeraddr         = port_con_fsm_get_peer_addr,
    .con_fsm_get_peeraddrtype     = port_con_fsm_get_peer_addr_type,
    .con_fsm_conn_upd_req         = port_con_fsm_send_connection_upd_req,
    .con_fsm_resolve_address      = port_con_fsm_resolve_address,
    .con_fsm_start_security       = port_con_fsm_start_security,  
    .con_fsm_mitm_passcode_report = port_con_fsm_mitm_passcode_report,
    .con_fsm_connect_confirm      = port_con_fsm_connect_confirm,
    .con_fsm_get_rssi             = port_con_fsm_get_rssi,
};

/**
 * \}
 * \}
 * \}
 */

#endif // _APP_COM_FSM_CONFIG_H_
