/*****************************************************************************************
 *
 * \file port_white_list.c
 *
 * \brief White List management API
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
 * \addtogroup PORT_WHITELIST
 * \{
 ****************************************************************************************	 
 */

/*
 * INCLUDE FILES
******************************************************************************************/

#include <port_white_list.h>

void port_white_list_clear_list(void)
{
        struct gapm_white_list_mgt_cmd * req = KE_MSG_ALLOC(GAPM_WHITE_LIST_MGT_CMD,
                TASK_GAPM, TASK_APP,
                gapm_white_list_mgt_cmd);

        // Fill in the parameter structure
        req->operation = GAPM_CLEAR_WLIST;
        req->nb = 0;
        ke_msg_send(req);
}

void port_white_list_send_mgt_cmd(bool add, uint8_t peer_addr_type, struct bd_addr const *peer_addr)
{
    struct gapm_white_list_mgt_cmd * req = KE_MSG_ALLOC_DYN(GAPM_WHITE_LIST_MGT_CMD, 
                                                            TASK_GAPM, TASK_APP, 
                                                            gapm_white_list_mgt_cmd, 
                                                            sizeof(struct gap_bdaddr));

    // Fill in the parameter structure
    if (add) {
        req->operation = GAPM_ADD_DEV_IN_WLIST;
    }
    else {
        req->operation = GAPM_RMV_DEV_FRM_WLIST;
    }
        
    req->nb = 1;
    req->devices[0].addr_type = peer_addr_type;
    memcpy(req->devices[0].addr.addr, peer_addr->addr, BD_ADDR_LEN);
    
    ke_msg_send(req);
}

/**
 * \}
 * \}
 * \}
 */
