/**
 ****************************************************************************************
 *
 * \file port_security.c
 *
 * \brief Implementation of security.
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
 * \addtogroup PORT_SECURITY
 * \{
 ****************************************************************************************	 
 */
 
#ifdef HAS_CONNECTION_FSM

#if BLE_APP_SEC
    #error "Connection FSM is used. Disable BLE security functionality in TASK_APP by undefining CFG_APP_SECURITY"
#endif

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
    #include "app_easy_security.h"
    #include "app_pairing.h"
    #include "port_multi_bond.h"
    #include "port_security.h"
    #include "port_con_fsm.h"
    #include "app_mid.h"
    #include "app_entry_point.h"

void port_security_send_ltk_exch_rsp(uint8_t conidx, struct app_pairing_env_tag *env)
{
//    app_easy_security_set_ltk_exch_from_sec_env(conidx);
    struct gapc_bond_cfm* cfm = app_gapc_bond_cfm_ltk_exch_msg_create(conidx);

    cfm->data.ltk.key_size = env->key_size;
    cfm->data.ltk.ediv = env->ediv;
    memcpy(&(cfm->data.ltk.randnb), &(env->rand_nb) , RAND_NB_LEN);
    memcpy(&(cfm->data.ltk.ltk), &(env->ltk) , KEY_LEN);
    
    // Send the message
    ke_msg_send(cfm);
}

void port_security_send_encrypt_cfm(uint8_t conidx, struct app_pairing_env_tag *env, bool accept)
{
    struct gapc_encrypt_cfm* cfm = app_gapc_encrypt_cfm_msg_create(conidx);
    
    if (accept == true) {
        cfm->found = true;
        cfm->key_size = env->key_size;
        memcpy(&(cfm->ltk), &(env->ltk), KEY_LEN);
        port_con_fsm_connect_confirm(env->auth);
    } else {
        cfm->found = false;
    }
    ke_msg_send(cfm);
}

void port_security_send_pairing_rsp(bool accepted)
{
    dbg_puts(DBG_CONN_LVL, "    GAPC_PAIRING_REQ ind\r\n");

    struct gapc_bond_cfm* cfm;
    cfm = app_easy_security_pairing_rsp_get_active(0);

    // Feature check cannot be performed by the application. For example, the Host may have sent the
    // LL_CONNECT_REQ with No MITM and Bond. Although we require MITM and Bond we should reply. It is
    // left to the Host to check whether the requirements can be met and the Pairing procedure should
    // proceed or not.

    if (accepted == false) {
        // if we are bonded to a host with random address and we are performing undirected advertising to that host
        // then reject pairing requests from other hosts
        cfm->accept = false;
    }
    else {
        cfm->accept = true;

        if (app_con_fsm_get_access_rights() == SRV_PERM_AUTH) {
            // Authentication requirements
            // We do not return NO_BOND even if an NV memory is not present. The reason is that
            // the Host may drop the Pairing procedure in case it requires bonding (i.e. Windows do that).
            // Of course, since an NV memory is not present we fake we are bonding. We expect this to be used
            // in test or controlled environments only though.
            cfm->data.pairing_feat.auth = GAP_AUTH_REQ_MITM_BOND;
            // IO capabilities
            cfm->data.pairing_feat.iocap = GAP_IO_CAP_KB_ONLY;
            //Security requirements
            cfm->data.pairing_feat.sec_req = GAP_SEC1_AUTH_PAIR_ENC;
        }
        // if MITM is not used, then settings in user_config.h are applied

        //Initiator key distribution
        if ((port_con_fsm_get_peer_addr_type() == ADDR_RAND)
            && ((port_con_fsm_get_peer_addr()->addr[BD_ADDR_LEN - 1] & 0xC0) == GAP_RSLV_ADDR)) {
            cfm->data.pairing_feat.ikey_dist = GAP_KDIST_IDKEY;
        }
    }
    app_easy_security_send_pairing_rsp(0);
}

/**
 ****************************************************************************************
 * \brief
 *
 * \param       param
 * \param[in]   conidx
 *
 * \return
 ****************************************************************************************
 */
static int port_security_bond_req_ind_handler(struct gapc_bond_req_ind *param, uint8_t conidx)
{
    switch(param->request)
    {
    // Bond Pairing request
    case GAPC_PAIRING_REQ:
        app_con_fsm_state_update(PAIRING_REQ_EVT);
        break;

    // Retrieve Temporary Key (TK)
    case GAPC_TK_EXCH:
        if(param->data.tk_type == GAP_TK_DISPLAY || param->data.tk_type == GAP_TK_OOB) {
            // No special handling is needed
        }
        else if (param->data.tk_type == GAP_TK_KEY_ENTRY) {
            app_con_fsm_mitm_passcode_entry_func();
        }
        else {
            ASSERT_ERR(0);
        }
        break;

    // Retrieve Identity Resolving Key (IRK)
    case GAPC_IRK_EXCH:
        break;

    // Retrieve Connection Signature Resolving Key (CSRK)
    case GAPC_CSRK_EXCH:
        break;

    // Retrieve Long Term Key (LTK)
    case GAPC_LTK_EXCH:
        app_pairing_ltk_exch(conidx, port_alt_pair_get_env(), param->data.key_size);
        break;

    default:
        ASSERT_ERR(0);
        break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * \brief       
 *
 * \param       param
 * \param[in]   conidx
 *
 * \return
 ****************************************************************************************
 */
static int port_security_bond_ind_handler(struct gapc_bond_ind *param, uint8_t conidx)
{    
    switch(param->info)
    {
        // Bond Pairing request
    case GAPC_PAIRING_SUCCEED:
        // Save Authentication level
        port_alt_pair_set_auth(param->data.auth & GAP_AUTH_REQ_MITM_BOND);
        
        if (port_alt_pair_get_auth() & GAP_AUTH_BOND)
        {
            ASSERT_WARNING(conidx < APP_EASY_MAX_ACTIVE_CONNECTION);
            port_alt_pair_set_addr(app_env[conidx].peer_addr_type, 
                                   &app_env[conidx].peer_addr);
        }
        app_con_fsm_state_update(CONN_CMP_EVT);
        break;

    case GAPC_PAIRING_FAILED:
        // Disconnect
        app_easy_gap_disconnect(conidx);

        // No need to clear bonding data
        break;

    case GAPC_IRK_EXCH:
        // Save Identity Resolving Key
        dbg_puts(DBG_CONN_LVL, "IRK:");
        for(int i=0; i < KEY_LEN; i++) {
            dbg_printf(DBG_CONN_LVL, " %X", param->data.irk.irk.key[i]);
        }
        dbg_puts(DBG_CONN_LVL, "\r\n");
        port_alt_pair_set_irk(&param->data.irk);
        app_con_fsm_state_update(CONN_IRK_EXCH);
        break;
        
    case GAPC_LTK_EXCH:
    case GAPC_CSRK_EXCH:
    default:
        break;
    }
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * \brief       
 *
 * \param       param
 * \param[in]   conidx
 ****************************************************************************************
 */
static void port_security_encrypt_req_ind(struct gapc_encrypt_req_ind *param, uint8_t conidx)
{
    struct smpc_mst_id_info mst_id_info;
    mst_id_info.ediv = param->ediv;    
    memcpy(mst_id_info.randnb, param->rand_nb.nb, RAND_NB_LEN);
    
    app_pairing_encrypt_req_ind(conidx, port_alt_pair_get_env(), &mst_id_info);
}

/**
 ****************************************************************************************
 * \brief       
 *
 * \param       param
 * \param[in]   conidx
 ****************************************************************************************
 */
static void port_security_encrypt_ind(struct gapc_encrypt_req_ind *param, uint8_t conidx)
{
    app_con_fsm_state_update(CONN_CMP_EVT);
}

enum process_event_response port_security_process_handler(ke_msg_id_t const msgid, void const *param, uint8_t conidx)
{    
    switch(msgid) {
    case GAPC_BOND_REQ_IND:
        port_security_bond_req_ind_handler((struct gapc_bond_req_ind *)param, conidx);
        break;
    case GAPC_BOND_IND:
        port_security_bond_ind_handler((struct gapc_bond_ind *)param, conidx);
        break;
    case GAPC_ENCRYPT_REQ_IND:
        port_security_encrypt_req_ind((struct gapc_encrypt_req_ind *)param, conidx);
        break;
    case GAPC_ENCRYPT_IND:
        port_security_encrypt_ind((struct gapc_encrypt_req_ind *)param, conidx);
        break;
    default:
        return PR_EVENT_UNHANDLED;
    }
    
    return PR_EVENT_HANDLED;
}

#endif // HAS_CONNECTION_FSM

/**
 * \}
 * \}
 * \}
 */
