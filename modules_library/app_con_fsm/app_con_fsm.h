/******************************************************************************************
 *
 * \file app_con_fsm.h
 *
 * \brief The connection FSM provides a mechanism for handling procedures related to 
 * connection/disconnection, bonding, pairing, advertising(by using the Advertising FSM).
 * The Finite State Machine has a variety of states and reacts/handles any connection related
 * events produced by the application and provides a variety of notifications back to it.
 *
 * Define symbol HAS_CONNECTION_FSM to include this module in the application.
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
 * \brief Connection FSM header file
 * \{
 ****************************************************************************************	 	 
 */

#ifndef APP_CON_FSM_H_
#define APP_CON_FSM_H_

#include "app_con_fsm_defs.h"
#include "gap.h"
#include "app_prf_perm_types.h"

#define CON_FSM_PEER_ANY 0xFF

/*****************************************************************************************
 * \brief Connection FSM Core Event Handling Function 
 *
 * \details Handles any incoming events depending on the current FSM state
 *
 * \param[in] evt The event to be processed by the Connection FSM
******************************************************************************************/
void app_con_fsm_state_update(enum main_fsm_events evt);

/*****************************************************************************************
 * \brief Resets the bonding data and the white list entries
******************************************************************************************/
void app_con_fsm_reset_bonding_data(void);

/*****************************************************************************************
 * \brief   Configures connection FSM when connection is established.
 *          
 * \param[in]  addr   The BD address of the host
 *
 * \return  true if connection is accepted. False otherwise
******************************************************************************************/
bool app_con_fsm_connection_validation(const peer_addr_t *addr);    

/*****************************************************************************************
 * \brief   Initialize the FSM
******************************************************************************************/
void app_con_fsm_init(void);

/*****************************************************************************************
 * \brief   Handle disconnection
******************************************************************************************/
void app_con_fsm_on_disconnect(void);

/*****************************************************************************************
 * \brief   Starts the passcode entry
******************************************************************************************/
void app_con_fsm_mitm_passcode_entry_func(void);

/*****************************************************************************************
 * \brief   Sends the passcode that the user entered to the Host
 *
 * \param[in] code  The code to report to the Host
******************************************************************************************/
void app_con_fsm_mitm_passcode_report(uint32_t code);
    
/*****************************************************************************************
 * \brief   Check if encryption will be accepted (LL_ENC_REQ received but not when
 *          in PAIRING mode) for this link
 *
 * \param[in] ediv     The EDIV of the host
 * \param[in] rand_nb  The RAND of the host
 *
 * \return  true, if everything is ok. false, if the request is rejected
******************************************************************************************/
bool app_con_fsm_encrypt_req_validation(uint16_t ediv, const struct rand_nb *rand_nb);

/*****************************************************************************************
 * \brief     Connect to the host the bonding info of which are stored
 *            in the specified position 
 *          
 * \param[in] entry  The position in the storage memory that contains
 *                   the host's bonding information
 *
 * \return    true if host is known and switch is successful.
 *            false if the specified position does not contain valid host information.
******************************************************************************************/
bool app_con_fsm_switch_to_peer(uint8_t entry);

/*****************************************************************************************
 * \brief     Store the bonding info of the next host that will be bonded
 *            to the specified position 
 *          
 * \param[in] entry  The position in the storage memory where the bonding data
 *                   will be stored
******************************************************************************************/
void app_con_fsm_add_host_to_entry(uint8_t entry);

/*****************************************************************************************
 * \brief   Synchronize asynchronously generated requests for transmission
 *          of messages to OS tasks. Must be called from user_on_ble_powered()
 *
 * \return  true device sleep must be blocked
******************************************************************************************/
bool app_con_fsm_on_ble_powered(void);

/*****************************************************************************************
 * \brief   Synchronize asynchronously generated requests. 
 *          Must be called from user_on_system_powered()
 *
 * \return  APP_KEEP_POWERED device sleep must be blocked, APP_BLE_WAKEUP if BLE must
 *          wake-up, otherwise APP_GOTO_SLEEP
******************************************************************************************/
uint8_t app_con_fsm_on_system_powered(void);

/*****************************************************************************************
 * \brief   Saves the current bonding information to the NV. Will trigger an async task to do it 
******************************************************************************************/
void app_con_fsm_request_write_bonding_data(void);

/*****************************************************************************************
 * \brief   Deletes all the bonding information from the NV. Will trigger an async task to do it 
******************************************************************************************/
void app_con_fsm_request_erase_bonding_data(void);

/*****************************************************************************************
 * \brief      Request asynchronously a disconnection from the currently connected host
 *             The request will be synchronized with OS in
 *             app_con_fsm_on_ble_powered()
 *          
 * \param[in]  reject_hosts Specify how the hosts that will try to connect will be treated
 *             MULTI_BOND_REJECT_NONE: all hosts will be allowed to connect
 *             MULTI_BOND_REJECT_LAST: Last connected host will be rejected
 *             MULTI_BOND_REJECT_ALL_KNOWN: All known hosts stored in the memory will
 *             be rejected
 * \param[in]  dcreason The disconnection reason
******************************************************************************************/
void app_con_fsm_request_disconnect(enum multi_bond_host_rejection reject_hosts, app_con_fsm_disconnect_req_reason_t dcreason);

/*****************************************************************************************
 * \brief get the current fsm state
 *
 * \return  current fsm state
******************************************************************************************/
main_fsm_states app_con_fsm_get_state(void);

/*****************************************************************************************
 * \brief Handles GAP manager command complete events
 *
 * \param[in] operation  The GAP operation to be handled
 * \param[in] status     The status of the GAP operation
******************************************************************************************/
void app_con_fsm_handle_cmp_evt(uint8_t operation, uint8_t status);

/*****************************************************************************************
 * \brief   Handles what needs to be done when random address has been resolved
 *
 * \param[in]  irk The irk used for resolving the address
******************************************************************************************/
void app_con_fsm_random_address_resolved(struct gap_sec_key *irk);

/*****************************************************************************************
 * \brief   Return the status of connection update request
 *
 * \return  True if connection update request is pending. False otherwise
******************************************************************************************/
bool app_con_fsm_is_conn_upd_pending(void);
    
/*****************************************************************************************
 * \brief       General purpose subtimer expiration handler
 *
 * \details     Handles the expiration of each of the subtimers and depending on the timer
 *              type generates an event to the app_con_fsm
******************************************************************************************/    
void app_con_fsm_gp_timer_expired(void);  
  
/*****************************************************************************************
 * \brief   Returns the access rights
 *
 * \return The access rights
******************************************************************************************/  
app_prf_srv_perm_t app_con_fsm_get_access_rights(void);

/*****************************************************************************************
 * \brief      Sends a connection update request
 *
 * \param[in]  force_min_interval force the minimum interval
******************************************************************************************/
void app_con_fsm_send_connection_upd_req(bool force_min_interval);

/*****************************************************************************************
 * \brief      Returns the current peer's address
 *
 * \return     bd_addr A pointer to the peers address
******************************************************************************************/
struct bd_addr* app_con_fsm_get_peer_addr(void);

/*****************************************************************************************
 * \brief      Returns the current peer's address type
 *
 * \return     uint8_t The peer's address type
******************************************************************************************/
uint8_t app_con_fsm_get_peer_addr_type(void);

#ifdef HAS_SPECIAL_ADVERTISING
/*****************************************************************************************
 * \brief      
 *
 * \param[in]  data
 * \param[in]  length
******************************************************************************************/
void app_con_fsm_send_special_packet(uint8_t *data, uint8_t length);
#endif

#endif // APP_CON_FSM_H_

/**
 * \}
 * \}
 * \}
 */
