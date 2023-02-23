/**
 ****************************************************************************************
 *
 * \file app_hogpd.c
 *
 * \brief Keyboard (HID) over GATT Application entry point.
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
 * \addtogroup USER
 * \{
 * \addtogroup PROFILE
 * \{
 * \addtogroup APP_HOGPD
 *
 * \{
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#if (BLE_HID_DEVICE)

#include <user_hogpd_config.h>
#include "app_hogpd.h"
#include "prf_utils.h"
#include "port_platform.h"
#include "app_prf_perm_types.h"

uint16_t report_ntf  __PORT_RETAINED;
uint8_t hogpd_conidx __PORT_RETAINED;

#define REPORT_TO_MASK(index) (HOGPD_CFG_REPORT_NTF_EN << index)

#if HID_NUM_OF_REPORTS > HOGPD_NB_REPORT_INST_MAX
    #error "Maximum munber of HID reports exceeded. Please increase HOGPD_NB_REPORT_INST_MAX"
#endif

/**
 ****************************************************************************************
 * @brief Enables the HOGPD profile 
 *
 * @param   None
 *
 * @return  void
 ****************************************************************************************
 */
void app_hogpd_enable(uint8_t conidx)
{   
    // Allocate the message
    struct hogpd_enable_req * req = KE_MSG_ALLOC(HOGPD_ENABLE_REQ, prf_get_task_from_id(TASK_ID_HOGPD), 
                                                 TASK_APP,
                                                 hogpd_enable_req);
    
    hogpd_conidx = conidx;
    // Fill in the parameter structure
    req->conidx = hogpd_conidx;

    report_ntf = 0;

#ifdef ALWAYS_ENALBE_REPORTS   
    int i;
    for (i=0; i<HID_NUM_OF_REPORTS; i++) {
        if(hogpd_reports[i].cfg & HOGPD_CFG_REPORT_IN == HOGPD_CFG_REPORT_IN) {
            report_ntf |= REPORT_TO_MASK(i); 
        }
    }        
#endif    

    req->ntf_cfg[0] = report_ntf;

    // Send the message
    ke_msg_send(req);    
}

/**
 ****************************************************************************************
 * \brief Creates the HID over GATT database
 *
 * \param   None
 ****************************************************************************************
 */
void app_hogpd_create_db(void)
{   
    struct hogpd_db_cfg* db_cfg;

    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC_DYN(GAPM_PROFILE_TASK_ADD_CMD,
                                                             TASK_GAPM, 
                                                             TASK_APP,
                                                             gapm_profile_task_add_cmd, 
                                                             sizeof(struct hogpd_db_cfg));
 
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
    req->prf_task_id = TASK_ID_HOGPD;
    req->app_task = TASK_APP;
    req->sec_lvl = get_user_prf_srv_perm(TASK_ID_HOGPD);
    req->start_hdl = 0;

    // Set parameters
    db_cfg = (struct hogpd_db_cfg*) req->param;
    struct hogpd_hids_cfg *cfg = &db_cfg->cfg[0];              

    db_cfg->hids_nb = 1;
    
    struct hids_hid_info *hid_info = &cfg->hid_info;        

    cfg->svc_features = HOGPD_CFG_KEYBOARD | HOGPD_CFG_PROTO_MODE;
    if (hogpd_params.batt_external_report) {
        cfg->svc_features |= HOGPD_CFG_MAP_EXT_REF;
    }
    if (hogpd_params.boot_protocol_mode) {
        cfg->svc_features |= HOGPD_CFG_BOOT_KB_WR;
    } 
 
    cfg->report_nb = HID_NUM_OF_REPORTS;

    uint8_t i;
    for(i = 0; i < HID_NUM_OF_REPORTS; i++) {
        cfg->report_id[i]=hogpd_reports[i].id;
        cfg->report_char_cfg[i]=hogpd_reports[i].cfg;
    }
    
    hid_info->bcdHID = 0x100;
    hid_info->bCountryCode = 0;
    if (hogpd_params.remote_wakeup) {
        hid_info->flags = HIDS_REMOTE_WAKE_CAPABLE;        
#ifdef HAS_PWR_MGR        
        ASSERT_WARNING(0);
#endif        
    }
    if (hogpd_params.normally_connectable) {
        hid_info->flags |= HIDS_NORM_CONNECTABLE;
    }
    
    if (hogpd_params.batt_external_report) {
        if(BLE_BATT_SERVER == 0) {
            ASSERT_WARNING(0); // Battery service must be enabled
        }
        struct hogpd_ext_ref *ext_rep_ref = &cfg->ext_ref;  // table 5.7 external report reference = included service value
        uint16_t start_hdl = 0;
        uint16_t uuid = ATT_SVC_BATTERY_SERVICE;
        uint8_t ret;
        volatile att_size_t value_size;
        
        ret = port_attmdb_find_by_uuid(0, &start_hdl, 0xFFFF, ATT_UUID_16_LEN, (uint8_t *)&uuid, true);
    
        if (ret == ATT_ERR_NO_ERROR) {
            ext_rep_ref->inc_svc_hdl = start_hdl;
            ext_rep_ref->rep_ref_uuid = ATT_SVC_BATTERY_SERVICE;
        }        
    }    

    // Send the message
    ke_msg_send(req);      
}

bool app_hogpd_send_report(uint8_t report_idx, uint8_t *data, uint16_t length, enum hogpd_report_type type)
{
    struct hogpd_report_upd_req *req;
    
    // Allocate the message
    req = KE_MSG_ALLOC_DYN(HOGPD_REPORT_UPD_REQ, 
                           prf_get_task_from_id(TASK_ID_HOGPD), 
                           TASK_APP, 
                           hogpd_report_upd_req, 
                           length);
    
    
    if (!req) {
        return false;
    }

    req->conidx = hogpd_conidx;

    struct hogpd_report_info *report = &req->report;
    
    // Fill in the parameter structure
    // TODO: find the index from the report number
    report->hid_idx = 0;
    
    ASSERT_ERROR((type != HOGPD_BOOT_KEYBOARD_INPUT_REPORT && type != HOGPD_BOOT_MOUSE_INPUT_REPORT) || report_idx == 0 );
    ASSERT_ERROR((type != HOGPD_BOOT_KEYBOARD_INPUT_REPORT && type != HOGPD_BOOT_MOUSE_INPUT_REPORT) || length <= HOGPD_BOOT_REPORT_MAX_LEN);
    ASSERT_ERROR((type == HOGPD_BOOT_KEYBOARD_INPUT_REPORT || type == HOGPD_BOOT_MOUSE_INPUT_REPORT) || length <= HOGPD_REPORT_MAX_LEN);

    report->type = type;
    report->idx =  report_idx;
    
    report->length = length;
    memcpy(report->value, data, length);            
    
    ke_msg_send(req);    
    
    return true;
}

uint8_t app_hogpd_get_protocol_mode(void)
{
    struct hogpd_env_tag* hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
    return hogpd_env->svcs[0].proto_mode;
}

uint16_t app_hogpd_report_handle(uint8_t report_nb)
{
    ASSERT_WARNING(report_nb<HOGPD_NB_REPORT_INST_MAX);
    struct hogpd_env_tag* hogpd_env = PRF_ENV_GET(HOGPD, hogpd);
    return hogpd_get_att_handle(hogpd_env, 0, HOGPD_IDX_REPORT_VAL, report_nb);    
}

#endif

/**
 * \}
 * \}
 * \}
 */
