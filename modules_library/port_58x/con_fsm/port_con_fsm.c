/*****************************************************************************************
 *
 * \file port_con_fsm.c
 *
 * \brief Connection FSM module platform adaptation source file
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
 
#ifdef HAS_CONNECTION_FSM

/*
 * INCLUDE FILES
******************************************************************************************/
#include "app_easy_security.h"
#include "app_mid.h"
#include <port_con_fsm.h>
#include <port_multi_bond.h>
#include "llc.h"

#ifdef CFG_APP_SECURITY
    #error "CFG_APP_SECURITY cannot be defiend when CON FSM is used"
#endif

ke_msg_id_t port_con_fsm_disconnect_msg;

struct bd_addr *port_con_fsm_get_peer_addr(void)
{
	return &app_env[0].peer_addr;
}

uint8_t port_con_fsm_get_peer_addr_type(void)
{
	return app_env[0].peer_addr_type;
}

void port_con_fsm_send_connection_upd_req(const con_fsm_param_update_t *params, bool force_min_interval)
{
    ke_state_t app_state = ke_state_get(TASK_APP);

    // Modify Conn Params
#if (RWBLE_SW_VERSION_MAJOR >= 8)    
    if (app_state == APP_CONNECTED) {
#else
    if (app_state == APP_SECURITY || app_state == APP_PARAM_UPD || app_state == APP_CONNECTED) {
#endif  
        
#ifdef USE_L2CAP_CONN_UPDATE_REQ
        #include "l2cc_task.h"

        struct l2cc_pdu_send_req *req = KE_MSG_ALLOC(L2CC_PDU_SEND_REQ,
                KE_BUILD_ID(TASK_L2CC, 0), TASK_APP,
                l2cc_pdu_send_req);

        // generate packet identifier

        /* fill up the parameters */
        req->pdu.chan_id                  = L2C_CID_LE_SIGNALING;
        req->pdu.data.update_req.code     = L2C_CODE_CONN_PARAM_UPD_REQ;
        req->pdu.data.update_req.intv_max = (force_min_interval == true) ? params->preferred_conn_interval_min :
                                                       params->preferred_conn_interval_max;
        req->pdu.data.update_req.intv_min = params->preferred_conn_interval_min;
        req->pdu.data.update_req.latency  = params->preferred_conn_latency; // Conn Events skipped
        req->pdu.data.update_req.timeout  = params->preferred_conn_timeout;   
        req->pdu.data.update_req.pkt_id   = 1;
#else        
        struct gapc_param_update_cmd * req = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD, TASK_GAPC,
                TASK_APP, gapc_param_update_cmd);

        // Fill in the parameter structure
        req->operation = GAPC_UPDATE_PARAMS;
    #if (RWBLE_SW_VERSION_MAJOR >= 8)           
        req->intv_min = params->preferred_conn_interval_min;
        req->intv_max = (force_min_interval == true) ? params->preferred_conn_interval_min :
                                                       params->preferred_conn_interval_max; 
        req->latency = params->preferred_conn_latency; // Conn Events skipped
        req->time_out = params->preferred_conn_timeout;   
        req->ce_len_min = 0xFFFF;
        req->ce_len_max = 0xFFFF;
    #else
        req->params.intv_min = params->preferred_conn_interval_min;
        req->params.intv_max = (force_min_interval == true) ? params->preferred_conn_interval_min :
                                                       params->preferred_conn_interval_max; 
        req->params.latency = params->preferred_conn_latency; // Conn Events skipped
        req->params.time_out = params->preferred_conn_timeout;     
    #endif          

#endif
        dbg_puts(DBG_CONN_LVL, "[Send GAP_PARAM_UPDATE_REQ]");
        ke_msg_send(req);
#if (RWBLE_SW_VERSION_MAJOR >= 8)   
        ke_state_set(TASK_APP, APP_CONNECTED);
#else
        ke_state_set(TASK_APP, APP_PARAM_UPD);
#endif        
    }
}

void port_con_fsm_disconnect(void)
{
    app_easy_gap_disconnect(0);
}

void port_con_fsm_resolve_address(void)
{
        uint8_t num_of_peers = port_alt_pair_get_num_of_peers();
    
		//Resolve address
		struct gapm_resolv_addr_cmd *cmd =
				(struct gapm_resolv_addr_cmd *)KE_MSG_ALLOC_DYN(
						GAPM_RESOLV_ADDR_CMD,
						TASK_GAPM, TASK_APP, gapm_resolv_addr_cmd,
						num_of_peers * sizeof(struct gap_sec_key));

		cmd->operation = GAPM_RESOLV_ADDR; // GAPM requested operation
		cmd->nb_key = num_of_peers; // Number of provided IRK
		cmd->addr = *port_con_fsm_get_peer_addr(); // Resolvable random address to solve
		port_alt_pair_get_irk_list(cmd->irk);

		ke_msg_send(cmd);
}

void port_con_fsm_start_security(void)
{
        uint8_t num_of_peers = port_alt_pair_get_num_of_peers();
    
        struct gapc_security_cmd* sec_cmd = app_easy_security_request_get_active(0);

        sec_cmd->auth = GAP_AUTH_REQ_NO_MITM_BOND; // TODO: Check this
        
        app_easy_security_request(0);

#if (RWBLE_SW_VERSION_MAJOR >= 8)   
        ke_state_set(TASK_APP, APP_CONNECTED);
#else
        // Go to security State
        ke_state_set(TASK_APP, APP_SECURITY);
#endif          
}

void port_con_fsm_mitm_passcode_report(uint32_t code)
{
    struct gapc_bond_cfm* cfm = app_gapc_bond_cfm_tk_exch_msg_create(0);
        
    memset(cfm->data.tk.key, 0, KEY_LEN);

    cfm->data.tk.key[3] = (uint8_t)((code & 0xFF000000) >> 24);
    cfm->data.tk.key[2] = (uint8_t)((code & 0x00FF0000) >> 16);
    cfm->data.tk.key[1] = (uint8_t)((code & 0x0000FF00) >>  8);
    cfm->data.tk.key[0] = (uint8_t)((code & 0x000000FF) >>  0);

    app_gapc_bond_cfm_tk_exch_msg_send(cfm);
}

int8_t port_con_fsm_get_rssi(void)
{
    return llc_env[0]->rssi;
}

#endif // HAS_CONNECTION_FSM

/**
 * \}
 * \}
 * \}
 */
