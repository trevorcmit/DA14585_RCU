/*****************************************************************************************
 *
 * \file port_con_fsm.h
 *
 * \brief The port_con_fsm module provides an abstraction layer between the platform specific
 * connection handling functions and the application.
 * 
******************************************************************************************/
 
/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup PORT_CON_FSM
 * \{
 ****************************************************************************************	 
 */
 
#ifndef PORT_CON_FSM_H_
#define PORT_CON_FSM_H_


/*
 * INCLUDE FILES
******************************************************************************************/

#include "rwip_config.h"             // SW configuration
#include "app_con_fsm.h"
#include <port_platform.h>

#if (BLE_SUOTA_RECEIVER)
#include "app_suotar.h"
#endif 

#if (BLE_SPOTA_RECEIVER)
#include "app_spotar.h"
#endif 


/*****************************************************************************************
 * \brief Send a disconnection request
******************************************************************************************/
void port_con_fsm_disconnect(void);

/*****************************************************************************************
 * \brief Returns the address of the connected peer
 * \return struct bd_addr * The address of the connected peer
******************************************************************************************/
struct bd_addr *port_con_fsm_get_peer_addr(void);

/*****************************************************************************************
 * \brief Returns the address type of the connected peer
 * \return uint8_t The address type of the connected peer
******************************************************************************************/
uint8_t port_con_fsm_get_peer_addr_type(void);

/*****************************************************************************************
 * \brief Sends connection update request
 *
 * \param[in] params Param update request parameters 
 * \param[in] force_min_interval If true set min/max interval to minimum value.
******************************************************************************************/
void port_con_fsm_send_connection_upd_req(const con_fsm_param_update_t *params, bool force_min_interval);

/*****************************************************************************************
 * \brief Starts address resolution process
******************************************************************************************/
void port_con_fsm_resolve_address(void);

/*****************************************************************************************
 * \brief Used during a connection to start the security mechanism
******************************************************************************************/
void port_con_fsm_start_security(void);

/*****************************************************************************************
 * \brief   Sends the passcode that the user entered to the Host
 *
 * \param[in] code  The code to report to the Host
******************************************************************************************/
void port_con_fsm_mitm_passcode_report(uint32_t code);

      
/*****************************************************************************************
 * \brief Called to confirm an active connection
 *
 * \param[in] auth The authentication type
******************************************************************************************/
__INLINE void port_con_fsm_connect_confirm(uint8_t auth)
{
#if (RWBLE_SW_VERSION_MAJOR >= 8)
    app_easy_gap_confirm(app_env[0].conidx, (enum gap_auth) auth, 1);
#else    
    app_easy_gap_confirm(app_env[0].conidx, (enum gap_auth) auth, GAP_AUTHZ_NOT_SET);
#endif    
}
 
/*****************************************************************************************
 * \brief   Get the RSSI of the active connection
 *
 * \return int8_t The rssi value
******************************************************************************************/
int8_t port_con_fsm_get_rssi(void);

#endif // PORT_CON_FSM_H_

/**
 * \}
 * \}
 * \}
 */
