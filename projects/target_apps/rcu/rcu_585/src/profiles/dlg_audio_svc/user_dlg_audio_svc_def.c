/**
 ****************************************************************************************
 *
 * \file user_dlg_audio_svc_def.c
 *
 * \brief Custom1 Server (CUSTS1) profile database definitions.
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
 * \addtogroup DLG_AUDIO_SVC
 *
 * \{
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "prf_types.h"
#include "attm_db_128.h"
#include <user_custs1_def.h>

/*
 * LOCAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

static att_svc_desc128_t custs1_svc                             = DLG_AUDIO_SVC_UUID_128;

static uint8_t DLG_AUDIO_CTRL_UUID_128[ATT_UUID_128_LEN]        = DEF_DLG_AUDIO_CTRL_UUID_128;
static uint8_t DLG_AUDIO_CONFIG_UUID_128[ATT_UUID_128_LEN]      = DEF_DLG_AUDIO_CONFIG_UUID_128;
static uint8_t DLG_AUDIO_AUDIO_DATA_UUID_128[ATT_UUID_128_LEN]  = DEF_DLG_AUDIO_AUDIO_DATA_UUID_128;

static uint16_t att_decl_svc       = ATT_DECL_PRIMARY_SERVICE;
static uint16_t att_decl_char      = ATT_DECL_CHARACTERISTIC;
static uint16_t att_decl_cfg       = ATT_DESC_CLIENT_CHAR_CFG;
static uint16_t att_decl_user_desc = ATT_DESC_CHAR_USER_DESCRIPTION;

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Full CUSTOM1 Database Description - Used to add attributes into the database
struct attm_desc_128 custs1_att_db[CUST1_IDX_NB] =
{
    // Dialog Audio Service Declaration
    [DLG_AUDIO_IDX_SVC]                  = {(uint8_t*)&att_decl_svc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(custs1_svc), sizeof(custs1_svc), (uint8_t*)&custs1_svc},

    // Control Characteristic Declaration
    [DLG_AUDIO_IDX_CTRL_CHAR]            = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // Control Characteristic Value
    [DLG_AUDIO_IDX_CTRL_VAL]             = {DLG_AUDIO_CTRL_UUID_128, ATT_UUID_128_LEN, PERM(NTF, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE) | PERM(WRITE_COMMAND, ENABLE),
                                            DLG_AUDIO_CTRL_CHAR_LEN, 0, NULL},

     // Control Client Characteristic Configuration Descriptor
    [DLG_AUDIO_IDX_CTRL_NTF_CFG]         = {(uint8_t*)&att_decl_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            sizeof(uint16_t), 0, NULL},

    // Control Characteristic User Description
    [DLG_AUDIO_IDX_CTRL_USER_DESC]       = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(DLG_AUDIO_CTRL_USER_DESC) - 1, sizeof(DLG_AUDIO_CTRL_USER_DESC) - 1, DLG_AUDIO_CTRL_USER_DESC},

    // Configuration Characteristic Declaration
    [DLG_AUDIO_IDX_CONFIG_CHAR]          = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // Configuration Characteristic Value
    [DLG_AUDIO_IDX_CONFIG_VAL]           = {DLG_AUDIO_CONFIG_UUID_128, ATT_UUID_128_LEN, PERM(RD, ENABLE),
                                            DLG_AUDIO_CONFIG_CHAR_LEN, 0, NULL},

    // Configuration Characteristic User Description
    [DLG_AUDIO_IDX_CONFIG_USER_DESC]     = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(DLG_AUDIO_CONFIG_USER_DESC) - 1, sizeof(DLG_AUDIO_CONFIG_USER_DESC) - 1, DLG_AUDIO_CONFIG_USER_DESC},

                                        
    // Data Characteristic Declaration
    [DLG_AUDIO_IDX_AUDIO_DATA_CHAR]      = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // Data Characteristic Value
    [DLG_AUDIO_IDX_AUDIO_DATA_VAL]       = {DLG_AUDIO_AUDIO_DATA_UUID_128, ATT_UUID_128_LEN, PERM(NTF, ENABLE),
                                            DLG_AUDIO_AUDIO_DATA_CHAR_LEN, 0, NULL},

    // Data Client Characteristic Configuration Descriptor
    [DLG_AUDIO_IDX_AUDIO_DATA_NTF_CFG]   = {(uint8_t*)&att_decl_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            sizeof(uint16_t), 0, NULL},

    // Data Characteristic User Description
    [DLG_AUDIO_IDX_AUDIO_DATA_USER_DESC] = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(DLG_AUDIO_AUDIO_DATA_USER_DESC) - 1, sizeof(DLG_AUDIO_AUDIO_DATA_USER_DESC) - 1, DLG_AUDIO_AUDIO_DATA_USER_DESC},
};

/**
 * \}
 * \}
 * \}
 */
