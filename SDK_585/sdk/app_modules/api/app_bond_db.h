/******************************************************************************************
 *
 * @file app_bond_db.h
 *
 * @brief Bond database header file.
 *
 * Copyright (C) 2012-2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 *****************************************************************************************
 */

#ifndef _APP_BOND_DB_H_
#define _APP_BOND_DB_H_

/*****************************************************************************************
 * @addtogroup APP_BOND_DB
 *
 * @brief
 *
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include "gapc_task.h"
#include "gap.h"
#include "co_bt.h"
#include <user_periph_setup.h>
#include <user_config.h>

/*
 * DEFINES
******************************************************************************************/

// SPI FLASH and I2C EEPROM data offset
#ifndef USER_CFG_BOND_DB_DATA_OFFSET
#if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
    #define APP_BOND_DB_DATA_OFFSET     (0x1E000)
#elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)
    #define APP_BOND_DB_DATA_OFFSET     (0x8000)
#endif
#else
    #define APP_BOND_DB_DATA_OFFSET     (USER_CFG_BOND_DB_DATA_OFFSET)
#endif // USER_CFG_BOND_DB_DATA_OFFSET

// Max number of bonded peers
#ifndef USER_CFG_BOND_DB_MAX_BONDED_PEERS
#define APP_BOND_DB_MAX_BONDED_PEERS    (5)
#else
#define APP_BOND_DB_MAX_BONDED_PEERS    (USER_CFG_BOND_DB_MAX_BONDED_PEERS)
#endif // USER_CFG_BOND_DB_MAX_BONDED_PEERS

// Database version
#define BOND_DB_VERSION                 (0x0001)

// Start and end header used to mark the bond data in memory
#define BOND_DB_HEADER_START            ((0x1234) + BOND_DB_VERSION)
#define BOND_DB_HEADER_END              ((0x4321) + BOND_DB_VERSION)

#define BOND_DB_VALID_ENTRY             (0xAA)

/*
 * TYPE DEFINITIONS
******************************************************************************************/
enum bond_db_entry_flags
{
    BOND_DB_ENTRY_IRK_PRESENT       = (1 << 0),
    BOND_DB_ENTRY_LCSRK_PRESENT     = (1 << 1),
    BOND_DB_ENTRY_RCSRK_PRESENT     = (1 << 2),
};

struct bond_db_data
{
    uint8_t valid;              // Valid Entry
    uint8_t flags;              // Present Keys
    uint8_t lru;                // Least Recently Used (stored) value

    struct gapc_ltk ltk;        // LTK (LTK, EDIV, RANDNB, Key size)
    struct gapc_irk irk;        // IRK (IRK, Remote Identity (Public or Static Address), Remote Address type)
    struct gap_sec_key lcsrk;   // Local CSRK
    struct gap_sec_key rcsrk;   // Remote CSRK
    struct gap_bdaddr bdaddr;   // Remote BD address, Remote BD address type
    uint8_t auth;               // Authentication level
};

/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Do initial fetch of stored bond data.
 * @return void
******************************************************************************************/
void bond_db_init(void);

/*****************************************************************************************
 * @brief Store bond data. Searches the slot to write the new bond data by using the
 *        following criteria.
 *        1) If there is a slot with the same IRK or BD address, replace that slot
 *        2) Else if there is an empty slot, store bond data to the first empty slot
 *        3) Else store bond data to the oldest written slot
 * @param[in] data    Pointer to the data to be stored.
 * @return void
******************************************************************************************/
void bond_db_store(struct bond_db_data *data);

/*****************************************************************************************
 * @brief Look up bond data by EDIV, RAND_NB. This is used when security is requested
 *        in order to retrieve the LTK.
 * @param[in] rand_nb    RAND_NB
 * @param[in] ediv       EDIV
 * @return Pointer to the bond data if they were found. Otherwise null.
******************************************************************************************/
const struct bond_db_data* bond_db_lookup_by_ediv(const struct rand_nb *rand_nb,
                                                  uint16_t ediv);

/*****************************************************************************************
 * @brief Clear bond data.
 * @return void
******************************************************************************************/
void bond_db_clear(void);

/// @} APP_BOND_DB

#endif // _APP_BOND_DB_H_
