/*****************************************************************************************
 *
 * @file app_callback.h
 *
 * @brief Application callbacks definitions. 
 *
 * Copyright (C) 2015 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
******************************************************************************************/

#ifndef _APP_CALLBACK_H_
#define _APP_CALLBACK_H_

/*
 * INCLUDE FILES
******************************************************************************************/

#include <stdint.h>
#include "gapm.h"
#include "attm.h"
#include "gapc_task.h"

/*
 * TYPE DEFINITIONS
******************************************************************************************/

struct app_callbacks
{
    void (*app_on_connection)(const uint8_t, struct gapc_connection_req_ind const *);
    void (*app_on_disconnect)(struct gapc_disconnect_ind const *);
    void (*app_on_update_params_rejected)(const uint8_t);
    void (*app_on_update_params_complete)(void);
    void (*app_on_set_dev_config_complete)(void);
    void (*app_on_adv_nonconn_complete)(const uint8_t);
    void (*app_on_adv_undirect_complete)(const uint8_t);
    void (*app_on_adv_direct_complete)(const uint8_t);
    void (*app_on_db_init_complete)(void);
    void (*app_on_scanning_completed)(const uint8_t);
    void (*app_on_adv_report_ind)(struct gapm_adv_report_ind const *);
    void (*app_on_connect_failed)(void);
    void (*app_on_get_dev_appearance)(uint16_t*);
    void (*app_on_get_dev_slv_pref_params)(struct gap_slv_pref*);
    void (*app_on_set_dev_info)(uint8_t, uint8_t*);
    void (*app_on_data_length_change)(struct gapc_le_pkt_size_ind *);
    void (*app_on_update_params_request)(struct gapc_param_update_req_ind const *, struct gapc_param_update_cfm *);
#if (BLE_APP_SEC)
    void (*app_on_pairing_request)(const uint8_t, struct gapc_bond_req_ind const *);
    void (*app_on_tk_exch_nomitm)(const uint8_t, struct gapc_bond_req_ind const *);
    void (*app_on_irk_exch)(struct gapc_bond_req_ind const *);
    void (*app_on_csrk_exch)(const uint8_t, struct gapc_bond_req_ind const *);
    void (*app_on_ltk_exch)(const uint8_t, struct gapc_bond_req_ind const *);
    void (*app_on_pairing_succeded)(void);
    void (*app_on_encrypt_ind)(const uint8_t);
    void (*app_on_mitm_passcode_req)(const uint8_t);
    void (*app_on_encrypt_req_ind)(const uint8_t, struct gapc_encrypt_req_ind const *);
    void (*app_on_security_req_ind)(const uint8_t);
#endif
};

/*
 * DEFINES
******************************************************************************************/

#define CALLBACK_ARGS_0(cb)                      {if (cb != NULL) cb();}
#define CALLBACK_ARGS_1(cb, arg1)                {if (cb != NULL) cb(arg1);}
#define CALLBACK_ARGS_2(cb, arg1, arg2)          {if (cb != NULL) cb(arg1, arg2);}
#define CALLBACK_ARGS_3(cb, arg1, arg2, arg3)    {if (cb != NULL) cb(arg1, arg2, arg3);}

#endif // _APP_CALLBACK_H_
