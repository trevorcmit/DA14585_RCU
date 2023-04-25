/**
****************************************************************************************
* \file user_custs1_def.h
* \brief Dialog audio service database declarations.
****************************************************************************************
*/
#ifndef _USER_CUSTS1_DEF_H_
#define _USER_CUSTS1_DEF_H_

/**
****************************************************************************************
* \addtogroup CONFIGURATION
* \{
* \addtogroup PROFILE_CONFIG
* \{
* \addtogroup DLG_AUDIO_SVC_CFG
* \brief Dialog Audio Service implementation
* \{
****************************************************************************************
*/

// INCLUDE FILES
#include "attm_db_128.h"

// DEFINES
#define DLG_AUDIO_SVC_UUID_128            {0xd8, 0x03, 0x99, 0x2d, 0x91, 0x3d, 0xb0, 0xb6, 0x03, 0x4c, 0x94, 0x50, 0x8b, 0x10, 0x1d, 0xbc}

#define DEF_DLG_AUDIO_CTRL_UUID_128       {0x6a, 0xbc, 0x6f, 0x0f, 0x0b, 0xdc, 0xc2, 0xb2, 0x71, 0x47, 0xe8, 0xef, 0xae, 0x41, 0x05, 0x0c}
#define DEF_DLG_AUDIO_CONFIG_UUID_128     {0x46, 0x95, 0xac, 0x87, 0x3d, 0x0e, 0xc6, 0xbf, 0xd7, 0x4d, 0x21, 0x9c, 0x9f, 0x28, 0xf9, 0xfd}
#define DEF_DLG_AUDIO_AUDIO_DATA_UUID_128 {0xd1, 0x3d, 0x07, 0x24, 0x32, 0x61, 0x87, 0x8e, 0xe4, 0x4c, 0x20, 0x0d, 0x91, 0x09, 0x2c, 0x8d}

#define DLG_AUDIO_CTRL_CHAR_LEN         20
#define DLG_AUDIO_CONFIG_CHAR_LEN       20
#define DLG_AUDIO_AUDIO_DATA_CHAR_LEN   (MAX_MTU_SIZE-3)

#define DLG_AUDIO_CTRL_USER_DESC        "Dialog Audio Control"
#define DLG_AUDIO_CONFIG_USER_DESC      "Dialog Audio Configuration"
#define DLG_AUDIO_AUDIO_DATA_USER_DESC  "Dialog Audio data"

/// Dialog Audio Service Data Base Characteristic enum
enum
{
    DLG_AUDIO_IDX_SVC = 0,

    DLG_AUDIO_IDX_CTRL_CHAR,
    DLG_AUDIO_IDX_CTRL_VAL,
    DLG_AUDIO_IDX_CTRL_NTF_CFG,
    DLG_AUDIO_IDX_CTRL_USER_DESC,

    DLG_AUDIO_IDX_CONFIG_CHAR,
    DLG_AUDIO_IDX_CONFIG_VAL,
    DLG_AUDIO_IDX_CONFIG_USER_DESC,

    DLG_AUDIO_IDX_AUDIO_DATA_CHAR,
    DLG_AUDIO_IDX_AUDIO_DATA_VAL,
    DLG_AUDIO_IDX_AUDIO_DATA_NTF_CFG,
    DLG_AUDIO_IDX_AUDIO_DATA_USER_DESC,

    CUST1_IDX_NB
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

extern struct attm_desc_128 custs1_att_db[CUST1_IDX_NB];

/**
 * \}
 * \}
 * \}
 */

#endif // _USER_CUSTS1_DEF_H_
