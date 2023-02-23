/**
 ****************************************************************************************
 *
 * \file port_multi_bond.c
 *
 * \brief Special (multi) bonding procedure
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
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup PORT_MULTI_BOND
 * \brief Special (multi) bonding procedure.
 * \{
 ****************************************************************************************	 
 */

#ifdef HAS_CONNECTION_FSM

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#include "app_nv_prom.h"
#include "app_con_fsm.h"
#include "port_con_fsm.h"
#include "port_multi_bond.h"
#include "app_white_list.h"
#include "attm_db.h"
#include "gattc_task.h"
#include <app_con_fsm_config.h>

struct usage_array_s {
    uint8_t pos[MAX_BOND_PEER];
};

struct irk_array_s {
    struct gap_sec_key irk[MAX_BOND_PEER];
};

extern bool bd_address_requested;

extern struct bd_addr dev_bdaddr;
extern enum multi_bond_host_rejection multi_bond_enabled; 

#if (BLE_APP_PRESENT)

#define ATTR_TYPE   0
#define CCC_TYPE    1

// Load IRKs in irk_array. Depends on the MBOND_LOAD_INFO_AT_INIT setting.
// IRKs won't be loaded if MBOND_LOAD_INFO_AT_INIT is 1 since the info is already available.
// They will be loaded if MBOND_LOAD_INFO_AT_INIT is 0 though.
// Do not modify!
#if (MBOND_LOAD_INFO_AT_INIT)
    #define MBOND_LOAD_IRKS_AT_INIT     (0)
#else
    #define MBOND_LOAD_IRKS_AT_INIT     (1)
#endif

#define NV_STORAGE_MAGIC_ADDR           (NV_STORAGE_BASE_ADDR + 0x04)
#define NV_STORAGE_STATUS_ADDR          (NV_STORAGE_BASE_ADDR + 0x08)
#define NV_STORAGE_USAGE_ADDR           (NV_STORAGE_BASE_ADDR + 0x09)
#define NV_STORAGE_BOND_DATA_ADDR       (NV_STORAGE_BASE_ADDR + 0x10)   // Up to 7 bonds (0x10 - 0x09 = 7)

/*
 * Retained variables
 ****************************************************************************************
 */
uint8_t multi_bond_status                       __PORT_RETAINED;
uint8_t multi_bond_active_peer_pos              __PORT_RETAINED; 
uint8_t multi_bond_next_peer_pos                __PORT_RETAINED;
uint8_t multi_bond_resolved_peer_pos            __PORT_RETAINED;
struct usage_array_s bond_usage                 __PORT_RETAINED;
struct irk_array_s irk_array                    __PORT_RETAINED; // stored in RetRAM for power saving reasons
struct bonding_info_s bond_array[MAX_BOND_PEER]  __PORT_RETAINED; // stored in RetRAM for power saving reasons
struct bonding_info_s bond_info                  __PORT_RETAINED; // Bonding info for current host
struct bd_addr alt_dev_bdaddr                   __PORT_RETAINED;

// Only one of irk_array and bond_array will be used eventually.
#if (MBOND_LOAD_INFO_AT_INIT && MBOND_LOAD_IRKS_AT_INIT)
#error "Either the IRKs will be loaded into the RetRAM or the whole bonding info. Not both!"
#endif

/*
 * Local variables
 ****************************************************************************************
 */
int attribute_handle;
	
/**
 ****************************************************************************************
 * \brief       Clear the NV memory.
 *
 * \warning     nv_prom_init() must be called before calling this function.  
 *              nv_prom_release() must be called after this function exits.
 ****************************************************************************************
 */
static void multi_bond_clear_eeprom(void)
{
    int i;
    int magic;
    const int info_size=sizeof(struct bonding_info_s);
    
    dbg_puts(DBG_CONN_LVL, "\r\n\r\n ***Erase all bonding data!!!!***\r\n");

    uint8_t zero_data[info_size];
    uint32_t addr = NV_STORAGE_BOND_DATA_ADDR;
      
    memset(zero_data, 0, info_size);
        
    for (i = 0; i < MAX_BOND_PEER; i++) {
        nv_prom_write_data( zero_data, addr, info_size);                               
        addr += info_size;
    }
    
    nv_prom_write_data( zero_data, NV_STORAGE_STATUS_ADDR, 1 );
    nv_prom_write_data( zero_data, NV_STORAGE_USAGE_ADDR,  NV_STORAGE_BOND_DATA_ADDR - NV_STORAGE_USAGE_ADDR);
    
    multi_bond_status = 0;

    if (con_fsm_params.has_usage_counters) {
        for (i = 0; i < MAX_BOND_PEER; i++) {
            bond_usage.pos[i] = 0;
        }
    }

    if (DEVELOPMENT_DEBUG) {
        addr = 0;
        int i;
        uint8_t read_data[4];

        for (i = 0; i < (info_size*MAX_BOND_PEER)/4; i++)
        {
            
            *((uint32_t *)read_data) = 0x55555555;
            
            nv_prom_read_data(read_data, NV_STORAGE_BOND_DATA_ADDR+addr, 4);
            addr += 4;
            if (memcmp(zero_data, read_data, 4))
                __asm("BKPT #0\n"); 
        }
    }
            
    magic = NV_STORAGE_MAGIC_NUMBER;
    nv_prom_write_data((uint8_t *)&magic, NV_STORAGE_MAGIC_ADDR, sizeof(int));
    if (con_fsm_params.has_usage_counters) {
        memset(&bond_usage, 0, sizeof(struct usage_array_s));
    }
    if (MBOND_LOAD_INFO_AT_INIT == false) {
        memset(&irk_array, 0, sizeof(struct irk_array_s));
    }

    if (MBOND_LOAD_INFO_AT_INIT) {
        memset(bond_array, 0, (MAX_BOND_PEER * sizeof(struct bonding_info_s)));
    }
    
    if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {
        app_white_list_clear();
    }
}

/**
 ****************************************************************************************
 * \brief       Update the index to the last (or current) used NV memory entry.
 *
 * \param[in]   index
 ****************************************************************************************
 */
static void multi_bond_update_active_peer_pos(int index)
{
    ASSERT_WARNING((index >= 0) && (index < MAX_BOND_PEER));
    
    multi_bond_active_peer_pos = index;
    multi_bond_next_peer_pos = ((index + 1) % MAX_BOND_PEER);
}

/**
 ****************************************************************************************
 * \brief       Check if bond info must be stored in the NV memory due to a DataBase change.
 *
 * \param[in]   pos             The offset of the field in the info
 * \param[in]   num_of_bits
 * \param[in]   value           The value of this field in the DataBase.
 * \param[in]   request_write
 ****************************************************************************************
 */
static void multi_bond_update_bond_data(uint8_t pos, uint8_t num_of_bits, uint8_t value, bool request_write)
{
    ASSERT_ERROR(num_of_bits <= 8);
    
    int old_info = bond_info.info;
    uint8_t mask = (1 << num_of_bits)-1;
    
    bond_info.info &= ~(mask << pos);
    bond_info.info |= ((value & mask) << pos);
    
    if (request_write == true && bond_info.info != old_info) {
        app_con_fsm_request_write_bonding_data();
    }
}

/**
 ****************************************************************************************
 * \brief       Reads bonding data fro the NV memory
 *
 * \param[out]  info    A pointer to where the bonding info will be placed
 * \param[in]   entry   The entry for which we want to get the bonding information
 *
 * \return      bool    true if the reading of bonding data was successful
 ****************************************************************************************
 */
static bool multi_bond_read_bond_data_from_nv(struct bonding_info_s *info, uint8_t entry)
{
    nv_prom_read_data( (uint8_t *) info, 
                       NV_STORAGE_BOND_DATA_ADDR + entry*sizeof(struct bonding_info_s), 
                       sizeof(struct bonding_info_s));

    return ((info->env.nvds_tag >> 4) == 0x5);    
}

/**
 ****************************************************************************************
 * \brief
 *
 * \param[in]   uuid
 *
 * \return
 ****************************************************************************************
 */
static uint8_t multi_bond_get_notification_info_index(uint16_t uuid) 
{
    uint8_t i=0;
    while(notification_info[i].uuid != uuid) {
        i++;
    }
    return i;
}

/**
 ****************************************************************************************
 * \brief       Refresh usage counters.
 *
 * \details     Refresh usage counters (decrement) for all entries apart from idx.
 *              idx is the current entry being used. Thus, it gets the maximum value
 *              currently available while all other entries are decremented by 1.
 *
 * \param[in]   idx     index of currently connected host
 ****************************************************************************************
 */
static void multi_bond_update_usage_count_values(int idx)
{
    int i;

    if (con_fsm_params.has_usage_counters) {
        for (i = 0; i < MAX_BOND_PEER; i++) {
            if ( (i == idx) || (bond_usage.pos[i] < bond_usage.pos[idx]) ) {
                continue;
            }
            if (bond_usage.pos[i]) {
                bond_usage.pos[i]--;
            }
        }
    }
}

/**
 ****************************************************************************************
 * \brief      Called to update the usage counter of a peer with a specific index
 *
 * \param[in]   idx The index of the peer
 *
 * \return      int 
 *
 * \retval      Status of update usage counters operation.
 *              <ul>
 *                  <li> 0 if usage counters are not supported
 *                  or the peer idx is invalid
 *                  <li> 1 if the update was successful
 *              </ul>
 ****************************************************************************************
 */
static int multi_bond_update_usage_count(int idx)
{
    if (con_fsm_params.has_usage_counters) {
        int i;
        int max = 0;
        int max_idx = -1;
        
        // find maximum
        for (i = 0; i < MAX_BOND_PEER; i++) {
            if (bond_usage.pos[i] > max) {
                max = bond_usage.pos[i];
                max_idx = i;
            }
        }

        if (max_idx == idx) {
            return 0;
        }
        // max > MAX?
        if (max < (MAX_BOND_PEER-1))
        {
            if (bond_usage.pos[idx] != 0) {
                multi_bond_update_usage_count_values(idx);
            }
            if (bond_usage.pos[idx] == 0) {
                bond_usage.pos[idx] = max + 1;
            } else {
                bond_usage.pos[idx] = max;
            }
        } else {
            multi_bond_update_usage_count_values(idx);
            bond_usage.pos[idx] = (MAX_BOND_PEER-1);
        }    
        ASSERT_ERROR(NV_STORAGE_BOND_DATA_ADDR - NV_STORAGE_USAGE_ADDR >= sizeof(struct usage_array_s));
        
        nv_prom_write_data((uint8_t *)&bond_usage, NV_STORAGE_USAGE_ADDR, sizeof(struct usage_array_s));
        return 1;
    }
    
    return 0;
}

#ifdef FORCE_CONNECT_TO_HOST_ON
extern uint8_t force_next_store_entry;

/**
 ****************************************************************************************
 * \brief     Force storing of the bonding info of the next host that will be bonded
 *            to the specified position
 *          
 * \param[in] entry  The position in the storage memory where the bonding data
 *                   will be stored
 ****************************************************************************************
 */
uint8_t app_alt_pair_force_next_store_entry(uint8_t entry)
{
        force_next_store_entry = entry;
        return port_alt_pair_get_active_index();
}

void app_alt_pair_reset_force_next_store_entry(void)
{
        force_next_store_entry = MAX_BOND_PEER;                
}

#endif

uint8_t port_multi_bond_get_entry_to_delete(void)
{
    uint8_t i;
#ifdef FORCE_CONNECT_TO_HOST_ON    
    if (force_next_store_entry != MAX_BOND_PEER) {
        uint8_t ret = force_next_store_entry;
        return ret;
    }
#endif
    
    if (MBOND_LOAD_INFO_AT_INIT) {
        if (port_con_fsm_get_peer_addr_type() == ADDR_PUBLIC) {
            // Look if there's an older entry for this host (use Public address)
            for (i = 0; i < MAX_BOND_PEER; i++) {
                if ((bond_array[i].env.peer_addr_type == ADDR_PUBLIC) && !memcmp(&bond_array[i].env.peer_addr, &app_env[0].peer_addr, BD_ADDR_LEN)) {
                    return i;
                }
            }
        } else {
            if (multi_bond_resolved_peer_pos != 0) { //resolved addresses (MBOND_LOAD_IRKS_AT_INIT case)
                return (multi_bond_resolved_peer_pos - 1);
            }
        }
    }
    else { //resolved addresses 
        if (port_con_fsm_get_peer_addr_type() == ADDR_PUBLIC) {
            // Look if there's an older entry for this host (use Public address)            
            struct bonding_info_s info;
            for (i = 0; i < MAX_BOND_PEER; i++) {
                multi_bond_read_bond_data_from_nv(&info, i);
                if ( (info.env.peer_addr_type == ADDR_PUBLIC) 
                   && (!memcmp(&info.env.peer_addr, &app_env[0].peer_addr, BD_ADDR_LEN)) ) { // no need to check if the entry is valid
                    break;            
                }
            }
                        
            if (i != MAX_BOND_PEER) {
                return i;
            }
        }
        else { 
            if (multi_bond_resolved_peer_pos != 0) {
                return (multi_bond_resolved_peer_pos - 1);
            }
        }
    }

    if (con_fsm_params.has_usage_counters) {
        // Find oldest entry
        for (i = 0; i < MAX_BOND_PEER; i++) {
            if (bond_usage.pos[i] == 0) {
                return i;
            }
        }
    }
    else {
       // if an empty entry is found return this entry. Otherwise return the last entry of the array.
       if (MBOND_LOAD_INFO_AT_INIT) {
            for (i = 0; i < MAX_BOND_PEER; i++) {
                if ((bond_array[i].env.nvds_tag >> 4) != 0x5) {
                    return i;
                }
            }
        }
        else {
            struct bonding_info_s info;
            for (i = 0; i < MAX_BOND_PEER; i++) {
                multi_bond_read_bond_data_from_nv(&info, i);
                if ((info.env.nvds_tag >> 4) != 0x5) { 
                    return i;
                }
            }                            
        }
    }
    
    return 0;
}

/**
 ****************************************************************************************
 * \brief       Get the last used entry (if any).
 *
 * \details     Gets the position of the last used bonding entry.
 *
 * \return      int
 *
 * \retval      The index of the last used bonding entry or MAX_BOND_PEER if none if found.
 ****************************************************************************************
 */
static int multi_bond_get_last_used_entry(void)
{
    int i;
    int ret = 0;
    int max = 0;

    if (con_fsm_params.has_usage_counters) {
        ret = MAX_BOND_PEER;
        
        for (i = 0; i < MAX_BOND_PEER; i++) {
            if (bond_usage.pos[i] > max) {
                max = bond_usage.pos[i];
                ret = i;
            }
        }
    }
    else {
        if (multi_bond_status == 0) {    // nothing has been stored in the NV memory
            ret = MAX_BOND_PEER;
        }
        // else return the 1st entry
    }

    return ret;
}

void port_alt_pair_init(void)
{
    if(con_fsm_params.has_multi_bond) {
// Multiple bonding support requires NV memory!        
        ASSERT_ERROR(con_fsm_params.has_nv_rom);
    }

#if !(defined(HAS_SPI_FLASH_STORAGE) || defined(HAS_I2C_EEPROM_STORAGE))
    if (con_fsm_params.has_nv_rom) {
// has_nv_rom is true. Neither HAS_SPI_FLASH_STORAGE nor HAS_I2C_EEPROM_STORAGE is defined.
        ASSERT_ERROR(0);
    }
#endif

#if !defined(NV_STORAGE_BASE_ADDR)
    if (con_fsm_params.has_nv_rom) {
// NV storage base address must be defined
        ASSERT_ERROR(0);
    }    
#endif

#ifdef FORCE_CONNECT_TO_HOST_ON    
    force_next_store_entry = MAX_BOND_PEER;
#endif
    
    if (con_fsm_params.has_nv_rom) {
        int i;
        int magic;
        /*volatile*/ int usage_mask = (1 << MAX_BOND_PEER) - 1; // if not volatile the compiler mess things up due to optimizations!
        bool flush = false;

        /*** Sanity tests ***/
        
        // MAGIC number first
        nv_prom_init();
#if (DEVELOPMENT_DEBUG)
        volatile int security;
        nv_prom_read_data((uint8_t *)&security, NV_STORAGE_BASE_ADDR, sizeof(int));        
#endif        
        nv_prom_read_data((uint8_t *)&magic, NV_STORAGE_MAGIC_ADDR, sizeof(int));
        if (magic != NV_STORAGE_MAGIC_NUMBER) {
            flush = true;
        } else {
            // MAGIC number is OK
            
            if (con_fsm_params.has_usage_counters) {
                nv_prom_read_data((uint8_t *)&bond_usage, NV_STORAGE_USAGE_ADDR, sizeof(struct usage_array_s));
                for (i = 0; i < MAX_BOND_PEER; i++) {
                    // usage must be within limits
                    if (bond_usage.pos[i] > MAX_BOND_PEER) {
                        flush = true;
                        break;
                    }
                    
                    // there can't be two entries that have the same usage count
                    // unless the count is zero
                    if (bond_usage.pos[i]) {
                        if ( usage_mask & (1 << bond_usage.pos[i]) ) {
                            usage_mask &= ~(1 << bond_usage.pos[i]);
                        } else {
                            flush = true;
                            break;
                        }
                    }
                }
            }
            
            if (!MBOND_LOAD_INFO_AT_INIT) {
                struct bonding_info_s info;

                for (i = 0; i < MAX_BOND_PEER; i++) {
                    if(multi_bond_read_bond_data_from_nv(&info, i)) {
                        if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {
                            app_white_list_add_host(info.env.peer_addr_type, &info.env.peer_addr, i);
                        }
                        if (info.ext_info & EXT_INFO_IRK_MASK) {
                            irk_array.irk[i] = info.env.irk;
                        }
                    }                
                }
            }
            
            if (MBOND_LOAD_INFO_AT_INIT) {
                struct bonding_info_s info;
                for (i = 0; i < MAX_BOND_PEER; i++) {
                    if (multi_bond_read_bond_data_from_nv(&info, i)) {
                        bond_array[i] = info;
                        if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {
                            app_white_list_add_host(info.env.peer_addr_type, &info.env.peer_addr, i);
                        }
                    }                
                }
            }
        }           
        if (flush) {
            multi_bond_clear_eeprom();
        }
        nv_prom_release();
    }
}

void port_alt_pair_read_status(void)
{
    if (con_fsm_params.has_nv_rom) {
        nv_prom_init_read_data(&multi_bond_status, NV_STORAGE_STATUS_ADDR, sizeof(uint8_t));
    }
}

void port_alt_pair_store_status(void)
{
    if (con_fsm_params.has_nv_rom) {
        nv_prom_init();
        nv_prom_write_byte(NV_STORAGE_STATUS_ADDR, multi_bond_status);
        nv_prom_release();
    }
}
     
/**
 ****************************************************************************************
 * \brief       Reads the value of a UUID in the DataBase.
 *
 * \details     Finds the given UUID descriptor in the DB and reads its value.
 *
 * \param[out]  handle      The handle of the UUID being processed
 * \param[in]   info        notification_info to be processed
 * \param[out]  value       The value to be read.
 *
 * \return      int
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> 0 if the UUID does not exist in the DB.
 *                  <li> 1 if the UUID was found and the read/write operation was successful
 *              </ul>
 ****************************************************************************************
 */
static bool multi_bond_read_value_of_uuid(uint16_t *handle, const notification_info_t *info, uint8_t **value)
{
    att_size_t length;
    
    if (port_atts_find_value_by_uuid(0, handle, 0xFFFF, ATT_UUID_16_LEN, (uint8_t *)&(info->uuid)) == ATT_ERR_NO_ERROR) {
#if (RWBLE_SW_VERSION_MAJOR >= 8)        
        if(info->type == CCC_TYPE) { // Cannot read CCC. They are no longer in the attdb.
            return false;
        }
            
        uint8_t ret = attmdb_get_value(*handle, &length, value);
#else
        extern uint8_t attmdb_att_get_value(uint16_t handle, att_size_t* length, uint8_t** value);

        if (info->type == CCC_TYPE) {                
            int tmp_uuid = ATT_DESC_CLIENT_CHAR_CFG;
                
            if (port_atts_find_value_by_uuid(0, handle, 0xFFFF, ATT_UUID_16_LEN, (uint8_t *)&tmp_uuid) != ATT_ERR_NO_ERROR) {
                ASSERT_WARNING(0);
                return false;
            }
        }

        uint8_t ret = attmdb_att_get_value(*handle, &length, value);
#endif        
        if (ret == ATT_ERR_NO_ERROR) {
            return true;
        }
    }
    return false;
}

/**
 ****************************************************************************************
 * \brief       Writes the value of a UUID in the DataBase.
 *
 * \details     Finds the given UUID or a Client Characteristic
 *              Configuration descriptor in the DB and writes its value.
 *
 * \param[out]  handle      The handle of the CCC UUID being processed
 * \param[in]   info        notification_info to be processed
 * \param[in]   value       The value to be read.
 *
 * \return      int
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> 0 if the UUID does not exist in the DB.
 *                  <li> 1 if the UUID was found and the read/write operation was successful
 *              </ul>
 ****************************************************************************************
 */
static bool multi_bond_write_value_of_uuid(uint16_t *handle, const notification_info_t *info, int value)
{
    if (port_atts_find_value_by_uuid(0, handle, 0xFFFF, ATT_UUID_16_LEN, (uint8_t *)&(info->uuid)) == ATT_ERR_NO_ERROR) {
#if (RWBLE_SW_VERSION_MAJOR >= 8)         
        if(info->type == CCC_TYPE) { // Cannot set CCC. They are no longer in the attdb.
            return false;
        }
        
        if (attmdb_att_set_value(*handle, info->length, 0, (uint8_t*) &value) == ATT_ERR_NO_ERROR) {
            return true;
        }
#else            
        if (info->type == CCC_TYPE) {                
            int tmp_uuid = ATT_DESC_CLIENT_CHAR_CFG;
                
            if (port_atts_find_value_by_uuid(0, handle, 0xFFFF, ATT_UUID_16_LEN, (uint8_t *)&tmp_uuid) != ATT_ERR_NO_ERROR) {
                ASSERT_WARNING(0);
                return 0;
            }
        }
        if (attmdb_att_set_value(*handle, info->length, (uint8_t*) &value) == ATT_ERR_NO_ERROR) {
            return true;
        }
#endif            
        }
    return false;
}

/**
 ****************************************************************************************
 * \brief       Update bonding information based on the current configuration of the DataBase.
 *
 * \param[in]   inf     The bonding information
 ****************************************************************************************
 */
static void prepare_bonding_info(struct bonding_info_s *inf)
{
    bool ret;
    int i, j;
    uint8_t *value;
    uint16_t handle;
            
       
    for (i=0; i < sizeof(notification_info)/sizeof(notification_info_t); i++) {
            handle = 0;
            for (j = 0; j < notification_info[i].num_of_atts; j++) {
                ret=multi_bond_read_value_of_uuid(&handle, &notification_info[i], &value);
                
                if (ret == true) {
                    multi_bond_update_bond_data(notification_info[i].position + j*notification_info[i].num_of_bits, 
                                                notification_info[i].num_of_bits, *value, false);
                }
            }
        }
}        

/**
 ****************************************************************************************
 * \brief       Write bonding information to a specific entry of the NV memory.
 *
 * \param[in]   entry     The index to to NV memory bonding entry.
 ****************************************************************************************
 */
static void write_bonding_info(int8_t entry)
{
    int addr = NV_STORAGE_BOND_DATA_ADDR;
    
    addr += entry * sizeof(struct bonding_info_s); // offset
   
    nv_prom_write_byte((addr + offsetof(struct bonding_info_s, env.nvds_tag)), 0); //invalidate
    nv_prom_write_data((uint8_t *)&bond_info, addr, sizeof(struct bonding_info_s));
    multi_bond_update_usage_count(entry);
    
    // Update the IRK array
    if (MBOND_LOAD_INFO_AT_INIT == false) {
//        memcpy(&irk_array.irk[multi_bond_active_peer_pos].key[0], &bond_info.irk.key[0], KEY_LEN);
        irk_array.irk[multi_bond_active_peer_pos] = bond_info.env.irk;
    }

    // or, update the buffer in RetRAM
    if (MBOND_LOAD_INFO_AT_INIT) {
        bond_array[entry] = bond_info;
    }
}

void port_alt_pair_store_bond_data(void)
{
    if (con_fsm_params.has_nv_rom) {
        if (con_fsm_params.disable_bonding_data_storage == false &&
	         (app_con_fsm_get_state() == CONNECTED_PAIRING_ST ||
             app_con_fsm_get_state() == CONNECTED_ST)) {
            nv_prom_init();
            if (multi_bond_active_peer_pos != MAX_BOND_PEER) { // entry for peer exists. Overwrite!
                prepare_bonding_info(&bond_info);
                write_bonding_info(multi_bond_active_peer_pos);
            }
            else {       // entry for peer does not exist. Write to an empty or the one that hasn't been used the longest.
                int8_t entry;

                ASSERT_WARNING(multi_bond_active_peer_pos == MAX_BOND_PEER);
                entry = port_multi_bond_get_entry_to_delete();
#ifdef FORCE_CONNECT_TO_HOST_ON                
                app_alt_pair_reset_force_next_store_entry();
#endif                
                multi_bond_update_active_peer_pos(entry);
            
                prepare_bonding_info(&bond_info);
                bond_info.env.nvds_tag = 0x50 + entry; // validity flag
                write_bonding_info(entry);
            
                if ( !(multi_bond_status & (1 << entry)) ) {
                    multi_bond_status |= (1 << entry); // update status
                    nv_prom_write_byte(NV_STORAGE_STATUS_ADDR, multi_bond_status);
                }
            }

            nv_prom_release();
        }
        // else ignore any write. It will fail with "Insufficient Authentication" anyway...
    }
    else {
        prepare_bonding_info(&bond_info);
    }
}

/**
 ****************************************************************************************
 * \brief       Updates the next attribute of a specific type in the DataBase
 *              based on the bonding information.
 *
 * \details     Updates a specific attribute in the DataBase based on the bonding information,
 *              starting from the current value of the "read" pointer of the DataBase. 
 *
 * \param[in]   inf      The bonding information.
 * \param[in]   pos      The position of the attribute in the inf array.
 * \param[in]   attr_num The number of the attribute at the specific position.
 *
 * \return      int
 *
 * \retval      The value of the attribute.
 ****************************************************************************************
 */
static int multi_bond_process_next_attribute(struct bonding_info_s *inf, int pos, int attr_num)
{
    const notification_info_t *p_info=&notification_info[pos];
    int value;
    
    value = (inf->info >> (p_info->position + attr_num*p_info->num_of_bits)) & ((1 << p_info->num_of_bits) - 1);  
        multi_bond_write_value_of_uuid((uint16_t *)&attribute_handle, &notification_info[pos], value);

    return value;
}

/**
 ****************************************************************************************
 * \brief       Updates the first attribute of a specific type in the DataBase
 *              based on the bonding information.
 *
 * \details     Updates a specific attribute in the DataBase based on the bonding information,
 *              starting from the beginning of the DataBase. The "read" pointer is reset.
 *
 * \param[in]   inf     The bonding information.
 * \param[in]   pos     The position of the attribute in the inf array.
 *
 * \return      int
 *
 * \retval      The value of the attribute.
 ****************************************************************************************
 */
static int multi_bond_process_attribute(struct bonding_info_s *inf, int pos)
{
    attribute_handle = 0;
        
    return multi_bond_process_next_attribute(inf, pos, 0);
}

void port_alt_pair_store_ccc(uint16_t uuid, int attr_num, int value)
{
    uint8_t pos = multi_bond_get_notification_info_index(uuid);
    
    multi_bond_update_bond_data(notification_info[pos].position+(attr_num*notification_info[pos].num_of_bits), notification_info[pos].num_of_bits, value, true);
}

void port_alt_pair_updatedb_from_bonding_info(void)
{
    int value;
    int i,j;
    
    for (i=0; i < sizeof(notification_info)/sizeof(notification_info_t); i++) {
        value = multi_bond_process_attribute(&bond_info, i);
        if (con_fsm_params.attr_update_callback) {
            (*con_fsm_params.attr_update_callback)(notification_info[i].uuid,0,value);
        } 
        for (j=1; j<notification_info[i].num_of_atts; j++) {
            value = multi_bond_process_next_attribute(&bond_info, i, j);
            if (con_fsm_params.attr_update_callback) {
                (*con_fsm_params.attr_update_callback)(notification_info[i].uuid,j,value);  
            }                
        }
    }
    
    // Check if there's any affected attribute range at this point.
    if (bond_info.ext_info & (EXT_INFO_SRV_CNG_START_HDL_MASK | EXT_INFO_SRV_CNG_END_HDL_MASK)) {
        struct gattc_send_svc_changed_cmd *req = KE_MSG_ALLOC(GATTC_SEND_SVC_CHANGED_CMD,
                                                      KE_BUILD_ID(TASK_GATTC, app_env[0].conidx), TASK_APP,
                                                      gattc_send_svc_changed_cmd);
        req->svc_shdl = (bond_info.ext_info & EXT_INFO_SRV_CNG_START_HDL_MASK) >> EXT_INFO_SRV_CNG_START_HDL_POS;
        req->svc_ehdl = (bond_info.ext_info & EXT_INFO_SRV_CNG_END_HDL_MASK  ) >> EXT_INFO_SRV_CNG_END_HDL_POS;
        
        // Send the event
        ke_msg_send(req);
    }
}

int port_alt_pair_load_bond_data(const struct rand_nb *rand_nb, uint16_t ediv)
{
    if (con_fsm_params.has_nv_rom) {
        uint32_t i;
        int retval = 0;

        if (MBOND_LOAD_INFO_AT_INIT) {
            //Read buffer in RetRAM
            nv_prom_init();
        
            for (i = 0; i < MAX_BOND_PEER; i++) {   
                if ( ( (bond_array[i].env.nvds_tag >> 4) == 0x5) && (bond_array[i].env.ediv == ediv) && (!memcmp(rand_nb, &bond_array[i].env.rand_nb, RAND_NB_LEN))
                            && (bond_array[i].env.auth & GAP_AUTH_BOND)) {
                    if ( (bond_info.env.ediv == ediv) && (!memcmp(rand_nb, &bond_info.env.rand_nb, RAND_NB_LEN))
                          && (bond_info.env.auth & GAP_AUTH_BOND) ) {
                        retval = 2;
                    } else {
                        retval = 1;
                    }
                    bond_info = bond_array[i];  // use memcpy instead???
               
                    multi_bond_update_active_peer_pos(i);
                    multi_bond_update_usage_count(i);
                    port_alt_pair_updatedb_from_bonding_info();
                    break; 
                }
            }
            nv_prom_release();

            return retval;
        }
        else {
            // Read NV memory
            struct bonding_info_s info;

            nv_prom_init();
            for (i = 0; i < MAX_BOND_PEER; i++) {
                if ( multi_bond_read_bond_data_from_nv(&info, i) && (info.env.ediv == ediv) && (!memcmp(rand_nb, &info.env.rand_nb, RAND_NB_LEN))
                     && (info.env.auth & GAP_AUTH_BOND)) {
                    if ( (bond_info.env.ediv == ediv) && (!memcmp(rand_nb, &bond_info.env.rand_nb, RAND_NB_LEN))
                          && (bond_info.env.auth & GAP_AUTH_BOND) ) {
                        retval = 2;
                    } else {
                        retval = 1;
                    }
                    bond_info = info;
                    multi_bond_update_active_peer_pos(i);
                    multi_bond_update_usage_count(i);
                    port_alt_pair_updatedb_from_bonding_info();
                    break; 
                }
            }
            nv_prom_release();
            return retval;
        }
    }
    else {
        return 0;
    }
}

bool port_alt_pair_get_next_bond_data(bool init)
{
    if (con_fsm_params.has_nv_rom) {
        uint32_t i;
        bool status = false;

        if (con_fsm_params.disable_bonding_data_storage == true) {
            return false;
        }

        if (MBOND_LOAD_INFO_AT_INIT) {
            // Read buffer in RetRAM after reset 'multi_bond_next_peer_pos' is 0.
            // If it's not 0 then this is the position we should use
            i = multi_bond_next_peer_pos;
            
            ASSERT_ERROR(i <= MAX_BOND_PEER);
            
            if (init) {
                multi_bond_active_peer_pos = MAX_BOND_PEER;
            }

            do {
                if (i == MAX_BOND_PEER) {
                    i = 0;
                }
                if ( ( (bond_array[i].env.nvds_tag >> 4) == 0x5) && (bond_array[i].env.peer_addr_type == ADDR_PUBLIC) && (bond_array[i].env.auth & GAP_AUTH_BOND) ) { 
                //used entry (public address) 
                    bond_info = bond_array[i];
                    port_alt_pair_updatedb_from_bonding_info();
                    multi_bond_update_active_peer_pos(i);
                    status = true;
                    break; 
                }
                i++;
                if (i == multi_bond_active_peer_pos) {
                    if (multi_bond_active_peer_pos == MAX_BOND_PEER) { // special case
                        multi_bond_next_peer_pos = 0;
                    }
                    else {
                        multi_bond_next_peer_pos = i + 1;
                    }
                    break;
                }
            } while(1);
            
            if (!status) {
                if (init){
                    port_alt_pair_set_default_info();
                    if (con_fsm_params.has_mitm) {
                            bond_info.env.auth = GAP_AUTH_REQ_MITM_BOND;
                    }
                    else {
                        bond_info.env.auth = GAP_AUTH_REQ_NO_MITM_BOND;
                    }
                }
                else {
                    bond_info = bond_array[multi_bond_active_peer_pos];
                }
            }
                
            return status;
        } else {
            // Read NV memory
            struct bonding_info_s info;

            nv_prom_init();
            // after reset 'multi_bond_next_peer_pos' is 0.
            // if it's not 0 then this is the position we should use
            i = multi_bond_next_peer_pos;

            if (init) {
                multi_bond_active_peer_pos = MAX_BOND_PEER;
            }

            do {
                if (i == MAX_BOND_PEER) {
                    i = 0;
                }
                if (multi_bond_read_bond_data_from_nv(&info, i) && (info.env.peer_addr_type == ADDR_PUBLIC) && (info.env.auth & GAP_AUTH_BOND) ) {
                //used entry (public address)
                    bond_info = info;
                    port_alt_pair_updatedb_from_bonding_info();
                    multi_bond_update_active_peer_pos(i);
                    status = true;
                    break; 
                }
                i++;

                if (i == multi_bond_active_peer_pos) {
                    if (multi_bond_active_peer_pos == MAX_BOND_PEER) { // special case
                        multi_bond_next_peer_pos = 0;
                    }
                    else {
                        multi_bond_next_peer_pos = i + 1;
                    }
                    break;
                }

            } while(1);

            if (!status) {
                if (init) {
                    port_alt_pair_set_default_info();
                    if (con_fsm_params.has_mitm) {
                        bond_info.env.auth = GAP_AUTH_REQ_MITM_BOND;
                    } else {
                        bond_info.env.auth = GAP_AUTH_REQ_NO_MITM_BOND;
                    }
                } else {
                    multi_bond_read_bond_data_from_nv(&bond_info, multi_bond_active_peer_pos);
                }
            }
                    
            nv_prom_release();

            return status;
        }
    }
    else {
        return false;
    }
}

bool port_alt_pair_load_last_used(void)
{
    bool status = false;
    uint32_t i;
    bool ret;
    
    if (con_fsm_params.has_nv_rom) {
        if (MBOND_LOAD_INFO_AT_INIT) {
            // Read buffer in RetRAM            
            i = multi_bond_get_last_used_entry();
                                    
            if (i != MAX_BOND_PEER) {   
                ASSERT_WARNING((bond_array[i].env.nvds_tag >> 4) == 0x5);
                if ( (bond_array[i].env.nvds_tag >> 4) == 0x5 ) {
                    bond_info = bond_array[i];
                    port_alt_pair_updatedb_from_bonding_info();
                    multi_bond_update_active_peer_pos(i);
                    status = true;
                } else {
                    // Data are corrupted!
                    // bond_usage or multi_bond_status reported as used an invalid entry!
                    ASSERT_ERROR(0);
                }
            }
        }
        else {
            // Read NV memory
            struct bonding_info_s info;
 
            i = multi_bond_get_last_used_entry();

            nv_prom_init();
 
            if (i != MAX_BOND_PEER) {

                ret = multi_bond_read_bond_data_from_nv(&info, i);
//                ASSERT_WARNING(ret);
                if (ret) {
					bond_info = info;
					port_alt_pair_updatedb_from_bonding_info();
					multi_bond_update_active_peer_pos(i);
					status = true;
                } else {
                    // NV memory is corrupted!
                    // bond_usage or multi_bond_status reported as used an invalid entry!
                    dbg_puts(DBG_CONN_LVL, "Failed to read last used entry. Bond data corrupted\r\n");
                    multi_bond_clear_eeprom();
                }
            }
            else {
                // If EEPROM is empty
            }
            nv_prom_release();
        }
    }

    return status;
}

bool port_alt_pair_load_entry(int8_t entry)
{
    bool status = false;
    
    ASSERT_WARNING( ((entry >= 0) && (entry < MAX_BOND_PEER)) );
    ASSERT_WARNING(entry != multi_bond_active_peer_pos);
    
    if (con_fsm_params.has_nv_rom) {
        if (MBOND_LOAD_INFO_AT_INIT) {
            // Read buffer in RetRAM
            
            if ( (bond_array[entry].env.nvds_tag >> 4) == 0x5 ) {
                bond_info = bond_array[entry];
                port_alt_pair_updatedb_from_bonding_info();
                multi_bond_update_active_peer_pos(entry);
                nv_prom_init();
                multi_bond_update_usage_count(entry);
                nv_prom_release();
                status = true;
            }
            else {
                // The entry is not valid!
            }
        }
        else {
            // Read NV memory
            struct bonding_info_s info;

            nv_prom_init();
     
            if ( multi_bond_read_bond_data_from_nv(&info, entry) ) {
                bond_info = info;
                port_alt_pair_updatedb_from_bonding_info();
                multi_bond_update_active_peer_pos(entry);
                multi_bond_update_usage_count(entry);
                status = true;
            }
            else {
                // The entry is not valid!
            }
            nv_prom_release();
        }
    }
    return status;
}

void port_alt_pair_delete_entry(int8_t entry)
{
    ASSERT_WARNING( ((entry >= 0) && (entry < MAX_BOND_PEER)) );
    
    if (con_fsm_params.has_nv_rom) {
        int addr;
        
        // Update usage counters
        if (con_fsm_params.has_usage_counters) {
            if (bond_usage.pos[entry] != 0) {
                int i;
                
                for (i = 0; i < MAX_BOND_PEER; i++) {
                    if (bond_usage.pos[i] > bond_usage.pos[entry]) {
                        bond_usage.pos[i]--;
                    }
                }
                bond_usage.pos[entry] = 0;
            }
        }
        
        nv_prom_init();
        
        if (MBOND_LOAD_INFO_AT_INIT == false) {
            struct bonding_info_s info;
            
            if ( multi_bond_read_bond_data_from_nv(&info, entry) ) {
                if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {
                    app_white_list_remove_host(info.env.peer_addr_type, &info.env.peer_addr, entry);
                }
            }
        }
        
        // Delete the entry
        addr = (entry * sizeof(struct bonding_info_s)) + NV_STORAGE_BOND_DATA_ADDR;
        nv_prom_write_byte((addr + offsetof(struct bonding_info_s, env.nvds_tag)), 0); //invalidate
       
        if (con_fsm_params.has_usage_counters) {
            // Write the usage counters
            nv_prom_write_data((uint8_t *)&bond_usage, NV_STORAGE_USAGE_ADDR, sizeof(struct usage_array_s));
        }
        
        // Update the multi_bond_status
        multi_bond_status &= ~(1 << entry);
        nv_prom_write_byte(NV_STORAGE_STATUS_ADDR, multi_bond_status);

        nv_prom_release();

        // Update the IRK array
        if (MBOND_LOAD_INFO_AT_INIT == false) {
            memset(&irk_array.irk[entry].key[0], 0, KEY_LEN);
        }
        
        // or, update the buffer in RetRAM
        if (MBOND_LOAD_INFO_AT_INIT) {
            if (con_fsm_params.has_white_list || con_fsm_params.has_virtual_white_list) {
                app_white_list_remove_host(bond_array[entry].env.peer_addr_type, &bond_array[entry].env.peer_addr, entry);
            }
            memset(&bond_array[entry], 0, sizeof(struct bonding_info_s));
        }
    }
}

void port_alt_pair_clear_all_bond_data(void)
{
    if (con_fsm_params.has_nv_rom) {
        nv_prom_init();
        multi_bond_clear_eeprom();
        nv_prom_release();
    }
    multi_bond_active_peer_pos = MAX_BOND_PEER;
}

void port_alt_pair_reset_active_peer_pos(void)
{
    multi_bond_active_peer_pos = MAX_BOND_PEER;
    multi_bond_next_peer_pos = 0;
}

uint8_t port_alt_pair_get_active_index(void)
{
    return multi_bond_active_peer_pos;
}

void port_alt_pair_update_usage_count(void)
{
    nv_prom_init();
    multi_bond_update_usage_count(bond_info.env.nvds_tag & 0xF);
    nv_prom_release();
}
   
bool port_alt_pair_is_bonded(void)
{
	return ((bond_info.env.auth & GAP_AUTH_BOND) != 0);
}

uint8_t port_alt_pair_get_auth(void)
{
	return bond_info.env.auth;
}

void port_alt_pair_set_auth(uint8_t auth)
{
	bond_info.env.auth = auth;	
}

struct bd_addr * port_alt_pair_get_addr(void)
{
	return &bond_info.env.peer_addr;
}

uint8_t port_alt_pair_get_addr_type(void)
{
	return bond_info.env.peer_addr_type;
}

void port_alt_pair_set_addr(uint8_t addr_type, struct bd_addr *bd_addr)
{
	bond_info.env.peer_addr_type = addr_type;
    memcpy(bond_info.env.peer_addr.addr, bd_addr->addr, BD_ADDR_LEN);
}

struct rand_nb * port_alt_pair_get_rand(void)
{
	return &(bond_info.env.rand_nb);
}

uint16_t port_alt_pair_get_ediv(void)
{
	return bond_info.env.ediv;
}

struct gap_sec_key * port_alt_pair_get_irk(void)
{
	return &(bond_info.env.irk);
}

void port_alt_pair_set_irk(struct gapc_irk *irk)
{
    bond_info.env.irk = irk->irk;
    bond_info.env.peer_addr = irk->addr.addr; // real BD addr
    bond_info.ext_info |= EXT_INFO_IRK_MASK; // set IRK flag to indicate that an IRK is included
}

struct app_pairing_env_tag * port_alt_pair_get_env(void)
{
	return &bond_info.env;
}

void port_alt_pair_set_default_info(void)
{
    bond_info.info = 0;
    for(int i = 0; i < sizeof(notification_info)/sizeof(notification_info_t); i++) {
        bond_info.info |= notification_info[i].default_value << notification_info[i].position;
    }
	port_alt_pair_updatedb_from_bonding_info();
	bond_info.env.auth = GAP_AUTH_NONE;
}

void port_alt_pair_reset_bonding_data(void)
{
	memset( (uint8_t *)&bond_info.env, 0, sizeof(struct app_pairing_env_tag) );	
}

void port_alt_pair_clear_service_changes_hdls(void)
{
    bond_info.ext_info &= ~(EXT_INFO_SRV_CNG_START_HDL_MASK | EXT_INFO_SRV_CNG_END_HDL_MASK);
}

uint8_t port_alt_pair_get_num_of_bonded_hosts(void)
{
	return __builtin_popcount(multi_bond_status);
}

/**
 ****************************************************************************************
 * \brief       Gets the IRK of a specific peer
 *
 * \param[in]   peer    The peer for which we want to get the IRK
 *
 * \return      struct gap_sec_key * The IRK
 ****************************************************************************************
 */
__INLINE struct gap_sec_key * multi_bond_get_irk_from_list(int peer)
{
	if (MBOND_LOAD_INFO_AT_INIT) {
		return &(bond_array[peer].env.irk);
	}
	else {
		return &(irk_array.irk[peer]);
	}
}

uint8_t port_alt_pair_get_num_of_peers(void)
{
    return MAX_BOND_PEER;
}

void port_alt_pair_get_irk_list(struct gap_sec_key irk[])
{
	if (con_fsm_params.has_nv_rom) {
		if (MBOND_LOAD_INFO_AT_INIT == false) {
			memcpy(irk, irk_array.irk, MAX_BOND_PEER * sizeof(struct gap_sec_key)); // Array of IRK used for address resolution (MSB -> LSB)
		}
		
		if (MBOND_LOAD_INFO_AT_INIT) {
			for (int i = 0; i < MAX_BOND_PEER; i++) {
				irk[i] = bond_array[i].env.irk;
			}
		}
	} else {
		irk[0] = *port_alt_pair_get_irk(); // Only one member in the "array", the "previous" host, if any.
	}
}	

uint8_t port_alt_pair_find_irk(struct gap_sec_key *irk, uint8_t start_pos)
{
    ASSERT_ERROR(start_pos > 0);
    
	for(int i = start_pos-1; i < MAX_BOND_PEER; i++) {
		if (!memcmp(&(multi_bond_get_irk_from_list(i)->key[0]), &irk->key[0], KEY_LEN)) {
			multi_bond_resolved_peer_pos = i+1;
			return multi_bond_resolved_peer_pos;
		}
	}
	
	return 0;
}

void port_alt_pair_clear_resolved_pos(void)
{
	multi_bond_resolved_peer_pos = 0;
}

#endif //(BLE_APP_PRESENT)

#endif // HAS_CONNECTION_FSM

/**
 * \}
 * \}
 * \}
 */
