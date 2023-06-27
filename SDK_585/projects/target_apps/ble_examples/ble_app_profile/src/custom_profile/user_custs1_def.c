/*****************************************************************************************
 *
 * @file user_custs1_def.c
 *
 * @brief Custom1 Server (CUSTS1) profile database definitions.
  *
******************************************************************************************/

/*****************************************************************************************
 * @defgroup USER_CONFIG
 * @ingroup USER
 * @brief Custom1 Server (CUSTS1) profile database definitions.
 *
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include <stdint.h>
#include "prf_types.h"
#include "attm_db_128.h"
#include "user_custs1_def.h"

/*
 * LOCAL VARIABLE DEFINITIONS
******************************************************************************************/

static att_svc_desc128_t custs1_svc                              = DEF_CUST1_SVC_UUID_128;

static uint8_t CUST1_CTRL_POINT_UUID_128[ATT_UUID_128_LEN]       = DEF_CUST1_CTRL_POINT_UUID_128;
static uint8_t CUST1_LED_STATE_UUID_128[ATT_UUID_128_LEN]        = DEF_CUST1_LED_STATE_UUID_128;
static uint8_t CUST1_ADC_VAL_1_UUID_128[ATT_UUID_128_LEN]        = DEF_CUST1_ADC_VAL_1_UUID_128;
static uint8_t CUST1_ADC_VAL_2_UUID_128[ATT_UUID_128_LEN]        = DEF_CUST1_ADC_VAL_2_UUID_128;
static uint8_t CUST1_BUTTON_STATE_UUID_128[ATT_UUID_128_LEN]     = DEF_CUST1_BUTTON_STATE_UUID_128;
static uint8_t CUST1_INDICATEABLE_UUID_128[ATT_UUID_128_LEN]     = DEF_CUST1_INDICATEABLE_UUID_128;
static uint8_t CUST1_LONG_VALUE_UUID_128[ATT_UUID_128_LEN]       = DEF_CUST1_LONG_VALUE_UUID_128;

static uint16_t att_decl_svc       = ATT_DECL_PRIMARY_SERVICE;
static uint16_t att_decl_char      = ATT_DECL_CHARACTERISTIC;
static uint16_t att_decl_cfg       = ATT_DESC_CLIENT_CHAR_CFG;
static uint16_t att_decl_user_desc = ATT_DESC_CHAR_USER_DESCRIPTION;

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

/// Full CUSTOM1 Database Description - Used to add attributes into the database
struct attm_desc_128 custs1_att_db[CUST1_IDX_NB] =
{
    // CUSTOM1 Service Declaration
    [CUST1_IDX_SVC]                     = {(uint8_t*)&att_decl_svc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(custs1_svc), sizeof(custs1_svc), (uint8_t*)&custs1_svc},

    // Control Point Characteristic Declaration
    [CUST1_IDX_CONTROL_POINT_CHAR]      = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // Control Point Characteristic Value
    [CUST1_IDX_CONTROL_POINT_VAL]       = {CUST1_CTRL_POINT_UUID_128, ATT_UUID_128_LEN, PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            DEF_CUST1_CTRL_POINT_CHAR_LEN, 0, NULL},

    // Control Point Characteristic User Description
    [CUST1_IDX_CONTROL_POINT_USER_DESC] = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_CONTROL_POINT_USER_DESC) - 1, sizeof(CUST1_CONTROL_POINT_USER_DESC) - 1, CUST1_CONTROL_POINT_USER_DESC},

    // LED State Characteristic Declaration
    [CUST1_IDX_LED_STATE_CHAR]          = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // LED State Characteristic Value
    [CUST1_IDX_LED_STATE_VAL]           = {CUST1_LED_STATE_UUID_128, ATT_UUID_128_LEN, PERM(WR, ENABLE) | PERM(WRITE_COMMAND, ENABLE),
                                            DEF_CUST1_LED_STATE_CHAR_LEN, 0, NULL},

    // LED State Characteristic User Description
    [CUST1_IDX_LED_STATE_USER_DESC]     = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_LED_STATE_USER_DESC) - 1, sizeof(CUST1_LED_STATE_USER_DESC) - 1, CUST1_LED_STATE_USER_DESC},

    // ADC Value 1 Characteristic Declaration
    [CUST1_IDX_ADC_VAL_1_CHAR]          = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // ADC Value 1 Characteristic Value
    [CUST1_IDX_ADC_VAL_1_VAL]           = {CUST1_ADC_VAL_1_UUID_128, ATT_UUID_128_LEN, PERM(RD, ENABLE) | PERM(NTF, ENABLE),
                                            DEF_CUST1_ADC_VAL_1_CHAR_LEN, 0, NULL},

    // ADC Value 1 Client Characteristic Configuration Descriptor
    [CUST1_IDX_ADC_VAL_1_NTF_CFG]       = {(uint8_t*)&att_decl_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            sizeof(uint16_t), 0, NULL},

    // ADC Value 1 Characteristic User Description
    [CUST1_IDX_ADC_VAL_1_USER_DESC]     = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_ADC_VAL_1_USER_DESC) - 1, sizeof(CUST1_ADC_VAL_1_USER_DESC) - 1, CUST1_ADC_VAL_1_USER_DESC},

    // ADC Value 2 Characteristic Declaration
    [CUST1_IDX_ADC_VAL_2_CHAR]          = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // ADC Value 2 Characteristic Value
    [CUST1_IDX_ADC_VAL_2_VAL]           = {CUST1_ADC_VAL_2_UUID_128, ATT_UUID_128_LEN, PERM(RD, ENABLE),
                                            DEF_CUST1_ADC_VAL_2_CHAR_LEN, 0, NULL},

    // ADC Value 2 Characteristic User Description
    [CUST1_IDX_ADC_VAL_2_USER_DESC]     = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_ADC_VAL_2_USER_DESC) - 1, sizeof(CUST1_ADC_VAL_2_USER_DESC) - 1, CUST1_ADC_VAL_2_USER_DESC},

    // Button State Characteristic Declaration
    [CUST1_IDX_BUTTON_STATE_CHAR]       = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // Button State Characteristic Value
    [CUST1_IDX_BUTTON_STATE_VAL]        = {CUST1_BUTTON_STATE_UUID_128, ATT_UUID_128_LEN, PERM(RD, ENABLE) | PERM(NTF, ENABLE),
                                            DEF_CUST1_BUTTON_STATE_CHAR_LEN, 0, NULL},

    // Button State Client Characteristic Configuration Descriptor
    [CUST1_IDX_BUTTON_STATE_NTF_CFG]    = {(uint8_t*)&att_decl_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            sizeof(uint16_t), 0, NULL},

    // Button State Characteristic User Description
    [CUST1_IDX_BUTTON_STATE_USER_DESC]  = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_BUTTON_STATE_USER_DESC) - 1, sizeof(CUST1_BUTTON_STATE_USER_DESC) - 1, CUST1_BUTTON_STATE_USER_DESC},

    // Indicateable Characteristic Declaration
    [CUST1_IDX_INDICATEABLE_CHAR]       = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // Indicateable Characteristic Value
    [CUST1_IDX_INDICATEABLE_VAL]        = {CUST1_INDICATEABLE_UUID_128, ATT_UUID_128_LEN, PERM(RD, ENABLE) | PERM(IND, ENABLE),
                                            DEF_CUST1_INDICATEABLE_CHAR_LEN, 0, NULL},

    // Indicateable Client Characteristic Configuration Descriptor
    [CUST1_IDX_INDICATEABLE_IND_CFG]    = {(uint8_t*)&att_decl_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            sizeof(uint16_t), 0, NULL},

    // Indicateable Characteristic User Description
    [CUST1_IDX_INDICATEABLE_USER_DESC]  = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_INDICATEABLE_USER_DESC) - 1, sizeof(CUST1_INDICATEABLE_USER_DESC) - 1, CUST1_INDICATEABLE_USER_DESC},

    // Long Value Characteristic Declaration
    [CUST1_IDX_LONG_VALUE_CHAR]         = {(uint8_t*)&att_decl_char, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            0, 0, NULL},

    // Long Value Characteristic Value
    [CUST1_IDX_LONG_VALUE_VAL]          = {CUST1_LONG_VALUE_UUID_128, ATT_UUID_128_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(NTF, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            DEF_CUST1_LONG_VALUE_CHAR_LEN, 0, NULL},

    // Long Value Client Characteristic Configuration Descriptor
    [CUST1_IDX_LONG_VALUE_NTF_CFG]      = {(uint8_t*)&att_decl_cfg, ATT_UUID_16_LEN, PERM(RD, ENABLE) | PERM(WR, ENABLE) | PERM(WRITE_REQ, ENABLE),
                                            sizeof(uint16_t), 0, NULL},

    // Long Value Characteristic User Description
    [CUST1_IDX_LONG_VALUE_USER_DESC]    = {(uint8_t*)&att_decl_user_desc, ATT_UUID_16_LEN, PERM(RD, ENABLE),
                                            sizeof(CUST1_LONG_VALUE_CHAR_USER_DESC) - 1, sizeof(CUST1_LONG_VALUE_CHAR_USER_DESC) - 1, CUST1_LONG_VALUE_CHAR_USER_DESC},
};

/// @} USER_CONFIG
