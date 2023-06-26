/*****************************************************************************************
 *
 * \file app_pairing.h
 *
 * \brief Pairing header file
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
 * \addtogroup APP_PAIRING
 * \brief Pairing header
 * \{
******************************************************************************************/
 
#ifndef APP_PAIRING_H_
#define APP_PAIRING_H_

#include "app_con_fsm.h"
#include "gap.h"
#include "smpc.h"

struct app_pairing_env_tag
{
    // CSRK
//    struct gap_sec_key csrk; // This is not used
    
    // LTK
    struct gap_sec_key ltk;
    
    // Random Number
    struct rand_nb rand_nb;
    
    // EDIV
    uint16_t ediv;
    
    // Remote IRK
    struct gap_sec_key irk;

    // LTK key size
    uint8_t key_size;

    // Last paired peer address type
    uint8_t peer_addr_type;
    
    // Last paired peer address
    struct bd_addr peer_addr;

    // authentication level
    uint8_t auth;

    // Current Peer Device NVDS Tag
    uint8_t nvds_tag;
};

/*****************************************************************************************
 * \brief Function called on long term key exchange event
 *
 * \param[in] conidx         Connection Id number
 * \param[in] env            Pairing auth tag
 * \param[in] key_size       LTK size
******************************************************************************************/
void app_pairing_ltk_exch(uint8_t conidx, struct app_pairing_env_tag *env, uint8_t key_size);

/*****************************************************************************************
 * \brief Function called on encrypt request event
 *
 * \param[in] conidx         Connection Id number
 * \param[in] env            Pairing auth tag
 * \param[in] mst_id_info    Pointer to Master Identifier structure (EDIV + Random number)
******************************************************************************************/
void app_pairing_encrypt_req_ind(uint8_t conidx, struct app_pairing_env_tag *env, struct smpc_mst_id_info *mst_id_info);

#endif // APP_PAIRING_H_

/**
 * \}
 * \}
 * \}
 */
