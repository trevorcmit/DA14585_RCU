/*****************************************************************************************
 *
 * \file app_white_list.c
 *
 * \brief White List management API.
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
 * \brief White and virtual white list implementation
 *
 * \{
******************************************************************************************/
 
#ifdef HAS_CONNECTION_FSM
#include "app_white_list.h"
#include <port_white_list.h>
#include <port_platform.h>

#include <app_con_fsm_config.h>

uint8_t white_list_written __PORT_RETAINED;
struct virtual_wl_entry virtual_white_list[MAX_BOND_PEER] __PORT_RETAINED;

/***
 * NOTE
 * ----
 * virtual_white_list has a 1-1 correspondence with the indexes to the volatile storage bond info of the 
 * entries that have been entered into it. Since it is initialized to 0, the value of 0 is used to mark
 * "unused" or "empty". A "used" virtual white list entry has a value of (index + 1).
 */
enum adv_filter_policy virtual_wlist_policy __PORT_RETAINED;
/***
 * NOTE
 * ----
 * The Virtual White List does not support Scan policies. So, below are listed the possible values of 
 * virtual_wlist_policy and those that are not supported are marked explicitly.
 *
 * ADV_ALLOW_SCAN_ANY_CON_ANY   : Allow both scan and connection requests from anyone. (default setting)
 * ADV_ALLOW_SCAN_WLST_CON_ANY  : Allow scan req from Virtual White List devices only and connection req from anyone. (NOT SUPPORTED)
 * ADV_ALLOW_SCAN_ANY_CON_WLST  : Allow scan req from anyone and connection req from Virtual White List devices only.
 * ADV_ALLOW_SCAN_WLST_CON_WLST : Allow scan and connection requests from Virtual White List devices only. (NOT SUPPORTED)
 */

bool app_white_list_add_host(uint8_t peer_addr_type, struct bd_addr const *peer_addr, uint8_t bond_info_entry)
{
        ASSERT_ERROR(con_fsm_params.has_virtual_white_list!=true || con_fsm_params.has_white_list!=true);

        int i;
        bool ret = false;

        if (con_fsm_params.has_white_list) {
                if (ADDR_PUBLIC == peer_addr_type) {
                        port_white_list_send_mgt_cmd(true, peer_addr_type, peer_addr);
                        ret = true;
                }
        }
        else {
                if (con_fsm_params.has_virtual_white_list) {
                        // Public or Static Private addresses
                        if ((ADDR_PUBLIC == peer_addr_type)
                                || ((peer_addr_type == ADDR_RAND) && ((peer_addr->addr[5] & GAP_STATIC_ADDR) == GAP_STATIC_ADDR))) {
                                for (i = 0; i < MAX_BOND_PEER; i++) {
                                        if ((virtual_white_list[i].status == USED_ADDR_PUBLIC_or_PRIVATE)
                                                && (!memcmp(&virtual_white_list[i].info.addr, peer_addr, BD_ADDR_LEN))) {
                                                break;  // already in there...
                                        }
                                }
                                if (i == MAX_BOND_PEER) {
                                        for (i = 0; i < MAX_BOND_PEER; i++) {
                                                if (virtual_white_list[i].status == UNUSED) {
                                                        dbg_printf(DBG_CONN_LVL, "Virtual list: Host %02x:%02x:%02x:%02x:%02x:%02x added\r\n",
                                                                peer_addr->addr[5], peer_addr->addr[4], peer_addr->addr[3], peer_addr->addr[2], peer_addr->addr[1], peer_addr->addr[0]);
                                                        memcpy(&virtual_white_list[i].info.addr, peer_addr, BD_ADDR_LEN);    // add entry
                                                        virtual_white_list[i].status = USED_ADDR_PUBLIC_or_PRIVATE;
                                                        white_list_written++;
                                                        virtual_wlist_policy = ADV_ALLOW_SCAN_ANY_CON_WLST;
                                                        ret = true;
                                                        break;
                                                }
                                        }
                                }
                        }
                        else {
                                if ((peer_addr_type == ADDR_RAND) && ((peer_addr->addr[BD_ADDR_LEN - 1] & 0xC0) == GAP_RSLV_ADDR)) {
                                        // Resolvable Random addresses
                                        for (i = 0; i < MAX_BOND_PEER; i++) {
                                                if ((virtual_white_list[i].status == USED_ADDR_RAND)
                                                        && (virtual_white_list[i].info.entry == (bond_info_entry + 1))) {
                                                        break;  // already in there...
                                                }
                                        }
                                        if (i == MAX_BOND_PEER) {
                                                for (i = 0; i < MAX_BOND_PEER; i++) {
                                                        if (virtual_white_list[i].status == UNUSED) {
                                                                dbg_printf(DBG_CONN_LVL, "Virtual list: Host with index %d added\r\n", bond_info_entry);
                                                                virtual_white_list[i].info.entry = bond_info_entry + 1;    // add entry
                                                                virtual_white_list[i].status = USED_ADDR_RAND;
                                                                white_list_written++;
                                                                virtual_wlist_policy = ADV_ALLOW_SCAN_ANY_CON_WLST;
                                                                ret = true;
                                                                break;
                                                        }
                                                }
                                        }
                                }
                        }
                }
        }

        return ret;
}

bool app_white_list_remove_host(uint8_t peer_addr_type, struct bd_addr const *peer_addr, uint8_t bond_info_entry)
{
        int i;
        bool ret = false;

        if (con_fsm_params.has_white_list) {
                if (ADDR_PUBLIC == peer_addr_type) {
                        port_white_list_send_mgt_cmd(false, peer_addr_type, peer_addr);
                        ret = true;
                }
        }
        else if (con_fsm_params.has_virtual_white_list) {
                // Public or Static Private addresses
                if ((ADDR_PUBLIC == peer_addr_type)
                        || ((peer_addr_type == ADDR_RAND) && ((peer_addr->addr[5] & GAP_STATIC_ADDR) == GAP_STATIC_ADDR))) {
                        for (i = 0; i < MAX_BOND_PEER; i++) {
                                if ((virtual_white_list[i].status == USED_ADDR_PUBLIC_or_PRIVATE)
                                        && (!memcmp(&virtual_white_list[i].info.addr, peer_addr, BD_ADDR_LEN))) {
                                        virtual_white_list[i].status = UNUSED;      // remove entry
                                        white_list_written--;
                                        if (white_list_written == 0) {
                                                virtual_wlist_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;
                                        }
                                        ret = true;
                                        break;
                                }
                        }
                }
                else if ((peer_addr_type == ADDR_RAND) && ((peer_addr->addr[BD_ADDR_LEN - 1] & 0xC0) == GAP_RSLV_ADDR)) {
                        // Resolvable Random addresses
                        for (i = 0; i < MAX_BOND_PEER; i++) {
                                if ((virtual_white_list[i].status == USED_ADDR_RAND) && (virtual_white_list[i].info.entry == (bond_info_entry + 1))) {
                                        virtual_white_list[i].status = UNUSED;      // remove entry
                                        white_list_written--;
                                        if (white_list_written == 0) {
                                                virtual_wlist_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;
                                        }
                                        ret = true;
                                        break;
                                }
                        }
                }
        }

        return ret;
}

bool app_white_list_lookup_rand_in_virtual_white_list(uint8_t bond_info_entry)
{
        bool ret = false;
        int i;

        if (con_fsm_params.has_virtual_white_list) {
                if (virtual_wlist_policy == ADV_ALLOW_SCAN_ANY_CON_ANY) {
                        ret = true;
                }
                else {
                        for (i = 0; i < MAX_BOND_PEER; i++) {
                                if ((virtual_white_list[i].status == USED_ADDR_RAND) && (virtual_white_list[i].info.entry == (bond_info_entry + 1))) {
                                        dbg_puts(DBG_CONN_LVL, "Virtual list: Host found\r\n");
                                        ret = true;
                                        break;
                                }
                        }
                }
        }

        return ret;
}

bool app_white_list_lookup_public_in_virtual_white_list(uint8_t peer_addr_type, struct bd_addr const *peer_addr)
{
        bool ret = false;
        int i;

        if (con_fsm_params.has_virtual_white_list) {
                if (virtual_wlist_policy == ADV_ALLOW_SCAN_ANY_CON_ANY) {
                        ret = true;
                }
                else {
                        for (i = 0; i < MAX_BOND_PEER; i++) {
                                if ((virtual_white_list[i].status == USED_ADDR_PUBLIC_or_PRIVATE)
                                        && (!memcmp(&virtual_white_list[i].info.addr, peer_addr, BD_ADDR_LEN))) {
                                        dbg_puts(DBG_CONN_LVL, "Virtual list: Host found\r\n");
                                        ret = true;
                                        break;
                                }
                        }
                }
        }

        return ret;
}

void app_white_list_clear(void)
{
        int i;

        if (con_fsm_params.has_white_list) {
                port_white_list_clear_list();
        }
        else if (con_fsm_params.has_virtual_white_list) {
                for (i = 0; i < MAX_BOND_PEER; i++) {
                        virtual_white_list[i].status = UNUSED;
                }
                virtual_wlist_policy = ADV_ALLOW_SCAN_ANY_CON_ANY;
        }
        white_list_written = 0;
}

bool app_white_list_written(void)
{
        return white_list_written != 0;
}

void app_white_list_handle_cmp_evt(uint8_t operation, uint8_t status)
{
        if (con_fsm_params.has_white_list) {
                switch (operation) {
                case GAPM_ADD_DEV_IN_WLIST:
                        if (status == GAP_ERR_NO_ERROR) {
                                white_list_written++;
                        }
                        // else something went wrong, i.e. the host may
                        // already be in the White List or the White List
                        // is empty...
                        break;
                case GAPM_RMV_DEV_FRM_WLIST:
                        if (white_list_written == 0) {
                                ASSERT_WARNING(0);
                        }

                        if (status == GAP_ERR_NO_ERROR) {
                                white_list_written--;
                        }
                        // else something went wrong, i.e. the host may
                        // not be in the White List or the White List
                        // is empty...
                        break;
                default:
                        break;
                }
        }
}

#endif

/**
 * \}
 * \}
 * \}
 */
