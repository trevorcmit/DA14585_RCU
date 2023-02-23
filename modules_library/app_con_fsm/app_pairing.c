/**
 *****************************************************************************************
 *
 * \file app_pairing.c
 *
 * \brief Pairing implementation file
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
 * \addtogroup APP_PAIRING
 * \brief Pairing application
 * \{
 ****************************************************************************************
 */


#ifdef HAS_CONNECTION_FSM

#include "app_pairing.h"
#include "port_con_fsm.h"
#include "port_multi_bond.h"
#include "port_security.h"
#include <stdlib.h>
#include "port_timer.h"

void app_pairing_ltk_exch(uint8_t conidx, struct app_pairing_env_tag *env, uint8_t key_size)
{
    env->key_size = key_size;
    // Generate the Random Number
    for (uint8_t i = 0; i < RAND_NB_LEN; i++) {
        env->rand_nb.nb[i] = rand() % 256;
    }

    // Randomly generate the LTK
    for (uint8_t i = 0; i < KEY_LEN; i++) {
        env->ltk.key[i] = (key_size < 16-i ? 0 : rand() % 256);
    }

    // Randomly generate the EDIV
    env->ediv = rand() % 65536;
    
    port_security_send_ltk_exch_rsp(conidx, env);
}

void app_pairing_encrypt_req_ind(uint8_t conidx, struct app_pairing_env_tag *env, struct smpc_mst_id_info *mst_id_info)
{
    if (app_con_fsm_encrypt_req_validation(mst_id_info->ediv, (struct rand_nb *)mst_id_info->randnb) == false) {
        port_con_fsm_disconnect();
        port_security_send_encrypt_cfm(conidx, env, false);
    }
    else {
        // Read the Authentication level, Random number and EDIV from sec_env
        if((env->auth & GAP_AUTH_BOND) &&
           !memcmp(env->rand_nb.nb, mst_id_info->randnb, RAND_NB_LEN) &&
           env->ediv == mst_id_info->ediv) {
            port_security_send_encrypt_cfm(conidx, env, true);
            // update connection auth
        } else {
            // An LL_REJECT_IND with reason "Pin or Key missing" will be sent by the stack.
            port_security_send_encrypt_cfm(conidx, env, false);
        }
    }        
}

#endif // HAS_CONNECTION_FSM

/**
 * \}
 * \}
 * \}
 */
