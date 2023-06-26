/*****************************************************************************************
 *
 * \file app_white_list.h
 *
 * \brief White List management header file.
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
 * \addtogroup APP_WHITE_LIST
 * \brief White and virtual white list header
 *
 * \{
******************************************************************************************/

#ifndef APP_WHITE_LIST_H_
#define APP_WHITE_LIST_H_

#include <stdint.h>
#include <stdbool.h>
#include "co_bt.h"

enum wl_entry_status {
    UNUSED = 0,
    USED_ADDR_PUBLIC_or_PRIVATE,
    USED_ADDR_RAND
};

struct virtual_wl_entry {
    enum wl_entry_status status;
    union {
        struct bd_addr addr;
        int8_t entry;
    } info;
};


/*****************************************************************************************
 * \brief       Adds a host to the normal or virtual White List.
 *
 * \details     Adds a host to the White List. If the host has Public Address,
 *              it is added to the normal White List. If it has a Resolvable
 *              Random Address, it is added to the virtual White List.
 *
 * \param[in]   peer_addr_type   The BD_ADDR type of the host.
 * \param[in]   peer_addr        The BD_ADDR of the host.
 * \param[in]   bond_info_entry  The index to the storage area for this bonding info.
 *
 * \return      bool
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> true if no error occurred; white_list_written will be increased by 1 upon successful completion
 *                  <li> false if the host could not be added for some reason
 *              </ul>
******************************************************************************************/
bool app_white_list_add_host(uint8_t peer_addr_type, struct bd_addr const *peer_addr, uint8_t bond_info_entry);

/*****************************************************************************************
 * \brief       Removes a host to the normal or virtual White List.
 *
 * \details     Removes a host from the White List. If the host has Public Address,
 *              it is removed from the normal White List. If it has a Resolvable
 *              Random Address, it is removed from the virtual White List.
 *
 * \param[in]   peer_addr_type   The BD_ADDR type of the host.
 * \param[in]   peer_addr        The BD_ADDR of the host.
 * \param[in]   bond_info_entry  The index to the storage area for this bonding info.
 *
 * \return      bool
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> true if no error occurred; white_list_written will be decreased by 1 upon successful completion
 *                  <li> false if the host could not be removed for some reason
 *              </ul>
******************************************************************************************/
bool app_white_list_remove_host(uint8_t peer_addr_type, struct bd_addr const *peer_addr, uint8_t bond_info_entry);

/*****************************************************************************************
 * \brief       Looks up a host with Resolvable Random address in virtual White List.
 *
 * \details     Looks for a host with Resolvable Random address into the virtual White List.
 *
 * \param       entry  The index to the storage area for this bonding info.
 *
 * \return      bool
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> true if the host has been found or the policy is ADV_ALLOW_SCAN_ANY_CON_ANY
 *                  <li> false if the host wasn't found
 *              </ul>
 *
 * @warning     The caller should only call this function for Resolvable Random addresses.
******************************************************************************************/
bool app_white_list_lookup_rand_in_virtual_white_list(uint8_t entry);

/*****************************************************************************************
 * \brief       Looks up a host with Public or Static Random address in virtual White List.
 *
 * \details     Looks for a host with Public or Static Random address into the virtual White List.
 *
 * \param[in]   peer_addr_type   The BD_ADDR type of the host.
 * \param[in]   peer_addr        The BD_ADDR of the host.
 *
 * \return      bool
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> true if the host has been found or the policy is ADV_ALLOW_SCAN_ANY_CON_ANY
 *                  <li> false if the host wasn't found
 *              </ul>
 *
 * @warning     The caller should only call this function for Public or Static Random addresses.
******************************************************************************************/
bool app_white_list_lookup_public_in_virtual_white_list(uint8_t peer_addr_type, struct bd_addr const *peer_addr);

/*****************************************************************************************
 * \brief       Clears both the normal and the virtual White Lists.
******************************************************************************************/
void app_white_list_clear(void);

/*****************************************************************************************
 * \brief       Check the status of the white list
 *
 * \return      true, if the white list contains at least one host, otherwise false
******************************************************************************************/
bool app_white_list_written(void);

/*****************************************************************************************
 * \brief Handles GAP manager command complete events
 *
 * \param[in] operation  The GAP operation to be handled
 * \param[in] status     The status of te GAP operation
******************************************************************************************/
void app_white_list_handle_cmp_evt(uint8_t operation, uint8_t status);

#endif // APP_WHITE_LIST_H_

/**
 * \}
 * \}
 * \}
 */
