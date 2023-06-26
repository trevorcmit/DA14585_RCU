/*****************************************************************************************
 *
 * \file port_con_fsm_task.c
 *
 * \brief Connection FSM module platform adaptation source file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

 /*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup PORT_CON_FSM
 * \brief Connection FSM HAL implementation
 * \{
 ****************************************************************************************	 
 */
 
#ifdef HAS_CONNECTION_FSM

/*
 * INCLUDE FILES
******************************************************************************************/

#include <port_con_fsm.h>
#include <app_con_fsm_config.h>
#include <port_con_fsm_task.h>
#include <port_multi_bond.h>
#include "gattc_task.h"
#include "app_entry_point.h"

extern enum adv_states current_adv_state;
extern bool bd_address_requested;
extern enum multi_bond_host_rejection multi_bond_enabled;
extern struct bd_addr dev_bdaddr;
extern struct bd_addr alt_dev_bdaddr;
extern bool send_undirected_adv_request_pending;

void port_con_fsm_enc_timer_handler(void)
{
    dbg_puts(DBG_CONN_LVL, "ENC timer exp\r\n");    
    port_con_fsm_disconnect();
}

void port_con_fsm_timer_handler(void)
{
    dbg_puts(DBG_CONN_LVL, "Connection FSM timer exp\r\n");
     
    // Timer elapsed!
    app_con_fsm_gp_timer_expired();
}

void port_con_fsm_alt_pair_timer_handler(void)
{
    dbg_puts(DBG_CONN_LVL, "ALT Pair Timer Exp\r\n");
        // set the multi bond "filter" to accept all hosts
    if (con_fsm_params.has_multi_bond) {
            multi_bond_enabled = MULTI_BOND_REJECT_NONE;
    }

    app_con_fsm_state_update(ALT_PAIR_TIMER_EXP_EVT);
}

#if (RWBLE_SW_VERSION_MAJOR >= 8)
    // Privacy is not handled yet
#else
/*****************************************************************************************
 * \brief  The Privacy flag has been altered by the remote host
 *
 * \param[in] param
******************************************************************************************/                                    
static void port_con_fsm_updated_privacy_ind_handler(void const *param)
{
    // 1. If disabled, use the public address in advertising
    // 2. If enabled generate a reconnection address (if 0:0:0:0:0:0) and write it.
    
    if (con_fsm_params.has_privacy) {
        #warning "Full feature implementation (Privacy) is pending..."
        
        struct gapm_updated_privacy_ind *privacy_ind = (struct gapm_updated_privacy_ind *)param;
        if (privacy_ind->privacy) {
            ASSERT_WARNING(0);
        }
    }
}    

/*****************************************************************************************
 * \brief  The host updated the reconnection address
 *
 * \param[in] param
******************************************************************************************/   
static void port_con_fsm_updated_recon_addr_ind_handler(void const *param)
{
    if (con_fsm_params.has_privacy) {
        ASSERT_WARNING(0);
    }
}
#endif

/*****************************************************************************************
 * \brief  The Client Char Config of Service Changed has been updated
 *
 * \param[in] param
******************************************************************************************/ 
static void port_con_fsm_service_changed_cfg_ind_handler(void const *param)
{
    struct gattc_svc_changed_cfg *ind = (struct gattc_svc_changed_cfg *)param;
        
    // ind->ind_cfg holds the value
    port_alt_pair_store_ccc(ATT_CHAR_SERVICE_CHANGED, 0, ind->ind_cfg);
}

/*****************************************************************************************
 * \brief   Called when the Service Changed indication has been successfully received by the Host
 *          
 * \param[in] param
 *****************************************************************************************
 */
static void port_con_fsm_svc_chng_cmp_evt_handler(void const *param)
{
    struct gattc_cmp_evt *ind = (struct gattc_cmp_evt *)param;
    
#if (RWBLE_SW_VERSION_MAJOR >= 8)    
    if(ind->operation == GATTC_SVC_CHANGED) {
#else
    if(ind->req_type == GATTC_SVC_CHANGED) {
#endif        
        // Clear bond_info handles so that we do not re-send the Service Changed Indication to this Host again
        port_alt_pair_clear_service_changes_hdls();

        // Update NV memory
        app_con_fsm_request_write_bonding_data();
    }
}

enum process_event_response port_con_fsm_process_handler(ke_msg_id_t const msgid, void const *param)
{
    enum gapm_msg_id gapm_msgid = (enum gapm_msg_id)msgid;
    switch(gapm_msgid) {
    case GAPM_ADDR_SOLVED_IND:
        app_con_fsm_random_address_resolved(&(((struct gapm_addr_solved_ind *)param)->irk));
        break;
    #if (RWBLE_SW_VERSION_MAJOR >= 8)        
    // Not handled yet
    #else
    case GAPM_UPDATED_PRIVACY_IND:
        port_con_fsm_updated_privacy_ind_handler(param);
        break;
    case GAPM_UPDATED_RECON_ADDR_IND:
        port_con_fsm_updated_recon_addr_ind_handler(param);
        break;
    #endif    
    case GAPM_CMP_EVT:
        app_con_fsm_handle_cmp_evt(((struct gapm_cmp_evt *)param)->operation, ((struct gapm_cmp_evt *)param)->status);
        break;
    default: 
        {
            enum gattc_msg_id gattc_msgid = (enum gattc_msg_id)msgid;
            switch(gattc_msgid) {
                case GATTC_SVC_CHANGED_CFG_IND:
                    port_con_fsm_service_changed_cfg_ind_handler(param);
                break;
                case GATTC_CMP_EVT:
                    port_con_fsm_svc_chng_cmp_evt_handler(param);
                break;
                default:
                    return PR_EVENT_UNHANDLED;
            }
        } 
        break;
    }    
    return PR_EVENT_HANDLED;
}

#endif // HAS_CONNECTION_FSM

/**
 * \}
 * \}
 * \}
 */
