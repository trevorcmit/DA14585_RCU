/*****************************************************************************************
 *
 * @file app_default_handlers.c
 *
 * @brief Default helper handlers implementing a primitive peripheral.
 *
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include "rwip_config.h"             // SW configuration
#include "arch_api.h"
#include "app_prf_types.h"
#include "app_prf_perm_types.h"
#include "app_easy_security.h"
#include "app.h"
#include "app_callback.h"
#include "app_default_handlers.h"
#include "app_task.h"
#include <user_profiles_config.h>
#include <user_callback_config.h>
#include "gap.h"
#include "gapc_task.h"

#if (BLE_CUSTOM_SERVER)
#include "user_custs_config.h"
#endif


/*
 * FUNCTION DEFINITIONS
******************************************************************************************/

void default_advertise_operation(void)
{
    if (user_default_hnd_conf.adv_scenario == DEF_ADV_FOREVER)
    {
        app_easy_gap_undirected_advertise_start();
    }
    else if (user_default_hnd_conf.adv_scenario == DEF_ADV_WITH_TIMEOUT)
    {
        app_easy_gap_undirected_advertise_with_timeout_start(user_default_hnd_conf.advertise_period, NULL);
    }
}

void default_app_on_init(void)
{
#if BLE_PROX_REPORTER
    app_proxr_init();
#endif

#if BLE_FINDME_TARGET
    app_findt_init();
#endif

#if BLE_FINDME_LOCATOR
    app_findl_init();
#endif

#if BLE_BATT_SERVER
    app_batt_init();
#endif

#if BLE_DIS_SERVER
    app_dis_init();
#endif

#if BLE_SUOTA_RECEIVER
    app_suotar_init();
#endif

    // Initialize service access write permissions for all the included profiles
    prf_init_srv_perm();

    // Set sleep mode
    arch_set_sleep_mode(app_default_sleep_mode);
}

void default_app_on_connection(uint8_t conidx, struct gapc_connection_req_ind const *param)
{
    if (app_env[conidx].conidx != GAP_INVALID_CONIDX)
    {
        app_env[conidx].connection_active = true;
        ke_state_set(TASK_APP, APP_CONNECTED);
        // Retrieve the connection info from the parameters
        app_env[conidx].conhdl = param->conhdl;
        app_env[conidx].peer_addr_type = param->peer_addr_type;
        memcpy(app_env[conidx].peer_addr.addr, param->peer_addr.addr, BD_ADDR_LEN);
#if (BLE_APP_SEC)
        // send connection confirmation
        app_easy_gap_confirm(conidx, (enum gap_auth) app_sec_env[conidx].auth, 1);
#else // (BLE_APP_SEC)
        app_easy_gap_confirm(conidx, GAP_AUTH_REQ_NO_MITM_NO_BOND, 1);
#endif

        if (user_default_hnd_conf.adv_scenario == DEF_ADV_WITH_TIMEOUT)
        {
            app_easy_gap_advertise_with_timeout_stop();
        }

        // Enable the created profiles/services
        app_prf_enable(conidx);
        
        if ((user_default_hnd_conf.security_request_scenario == DEF_SEC_REQ_ON_CONNECT) && (BLE_APP_SEC))
        {
            app_easy_security_request(conidx);
        }
    }
    else
    {
       // No connection has been established, restart advertising
       CALLBACK_ARGS_0(user_default_app_operations.default_operation_adv)
    }
}

void default_app_on_disconnect( struct gapc_disconnect_ind const *param ){
    // Restart Advertising
    CALLBACK_ARGS_0(user_default_app_operations.default_operation_adv)
}

void default_app_on_set_dev_config_complete(void)
{
    // Add the first required service in the database
    if (app_db_init_start())
    {
        // No more service to add, start advertising
        CALLBACK_ARGS_0(user_default_app_operations.default_operation_adv)
    }
}

void default_app_on_db_init_complete( void )
{
    CALLBACK_ARGS_0(user_default_app_operations.default_operation_adv)
}

void default_app_on_pairing_request(uint8_t conidx, struct gapc_bond_req_ind const *param)
{
    app_easy_security_send_pairing_rsp(conidx);
}

void default_app_on_tk_exch_nomitm(uint8_t conidx, struct gapc_bond_req_ind const *param)
{
    // Generate the pass key
    uint32_t pass_key = app_sec_gen_tk();
    
    // Store 32-bit number to local buffer
    uint8_t buf[sizeof(uint32_t)];
    for (uint8_t i = 0; i < sizeof(uint32_t); i++)
    {
        buf[i] = pass_key & 0xFF;
        pass_key = pass_key >> 8;
    }
    
    // Provide the TK to the host
    app_easy_security_tk_exch(conidx, buf, sizeof(uint32_t));
}

void default_app_on_csrk_exch(uint8_t conidx, struct gapc_bond_req_ind const *param)
{
    // Provide the CSRK to the host
    app_easy_security_csrk_exch(conidx);
}

void default_app_on_ltk_exch(uint8_t conidx, struct gapc_bond_req_ind const *param)
{
    // generate ltk and store it to sec_env
    app_sec_gen_ltk(conidx, param->data.key_size);
    //copy the parameters in the message
    app_easy_security_set_ltk_exch_from_sec_env(conidx);
    //send the message
    app_easy_security_ltk_exch(conidx);
    
}

void default_app_on_encrypt_req_ind(uint8_t conidx, struct gapc_encrypt_req_ind const *param)
{

    if (app_easy_security_validate_encrypt_req_against_env(conidx, param))
    {
        // update connection auth
        app_easy_gap_confirm(conidx, (enum gap_auth) app_sec_env[conidx].auth, 1);
        
        app_easy_security_set_encrypt_req_valid(conidx);
        
        app_easy_security_encrypt_cfm(conidx);
    }
    else
    {
        app_easy_security_set_encrypt_req_invalid(conidx);
    
        app_easy_security_encrypt_cfm(conidx);
    
        app_easy_gap_disconnect(conidx);
    }
}

void default_app_on_get_dev_appearance(uint16_t* appearance)
{
    *appearance = 0;
}

void default_app_on_get_dev_slv_pref_params(struct gap_slv_pref* slv_params)
{
    slv_params->con_intv_min = MS_TO_DOUBLESLOTS(10);
    slv_params->con_intv_max = MS_TO_DOUBLESLOTS(20);
    slv_params->slave_latency = 0;
    slv_params->conn_timeout = MS_TO_TIMERUNITS(1250);
}

void default_app_on_set_dev_info(uint8_t req, uint8_t* status)
{
    switch (req)
    {
        case GAPC_DEV_NAME:
        {
            *status = GAP_ERR_REJECTED;
        }            
        break;
        case GAPC_DEV_APPEARANCE:
        {
            *status = GAP_ERR_REJECTED;
        }            
        break;
        default: /* Do Nothing */ break;
    }
}

void default_app_update_params_request(struct gapc_param_update_req_ind const *param, struct gapc_param_update_cfm *cfm)
{
    // by default, the request is being accepted no matter what the param values are
    cfm->accept = true;
    cfm->ce_len_min = MS_TO_DOUBLESLOTS(0);
    cfm->ce_len_max = MS_TO_DOUBLESLOTS(0);
}
