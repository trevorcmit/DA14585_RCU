/*****************************************************************************************
 *
 * \file port_multi_bond.h
 *
 * \brief Special (multi) bonding procedure header file.
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup PORT_MULTI_BOND
 * \brief Special (multi) bonding procedure.
 * \{
 ****************************************************************************************	 
 */

#ifndef APP_MULTI_BOND_H_
#define APP_MULTI_BOND_H_

/*****************************************************************************************
 * USAGE
 *
 * To use this module the following must be properly defined in your project and include the header file in app_multi_bond.c:
 *     ALT_PAIR_DISCONN_TIME : Time to block previous host during a "host-switch" (in 10th of msec) 
 *
 * and the following must be defined as true if used or false if not used:
 * con_fsm_params.has_nv_rom : if there's no storage memory then using this module is useless.
 *
******************************************************************************************/


/*
 * INCLUDE FILES
******************************************************************************************/

#include "app_pairing.h"
#include "app_con_fsm_defs.h"


/*
 * DEFINES
******************************************************************************************/

#define NV_STORAGE_GUARD_NUMBER         (0xA55A5AA5)
#define NV_STORAGE_MAGIC_NUMBER         (0xDEADBEEF)

/*
 * Nono-volatile memory Structure
 ****************************************************************************************
 *
 *                            7         6         5         4         3         2         1         0
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 * NV_STORAGE_STATUS_ADDR    |                                   STATUS                                     |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                                                                  ...
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 * NV_STORAGE_USAGE_ADDR     |                                   Usage #0                                    |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 *                           |                                   Usage #1                                    |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 *                           |                                      ...                                      |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 *                           |                               Usage #N (< = 15)                               |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                                                                  ...
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 * NV_STORAGE_BOND_DATA_ADDR |                                Bonding info #0                                |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 *                           |                                      ...                                      |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 *                           |                                Bonding info #1                                |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 *                           |                                      ...                                      |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                           |         |         |         |         |         |         |         |         |
 *                           |                                Bonding info #2                                |
 *                           |         |         |         |         |         |         |         |         |
 *                           |---------|---------|---------|---------|---------|---------|---------|---------|
 *                                                              ...
 */

/*
 * BONDING INFO
 ****************************************************************************************
 *
 * Byte 0-3: CCC and attribute info as defined in notification_info
 *
 * Byte 4
 * ------
 * Set after any SW update that results in a modification of the DB (at all entries).
 * Cleared (in a specific entry) after informing the corresponding Host.
 *
 *      7         6         5         4         3         2         1         0
 * |---------|---------|---------|---------|---------|---------|---------|---------|
 * |         |         |         |         |         |         |         |         |
 * |                      Start of affected handle attr range                      |
 * |         |         |         |         |         |         |         |         |
 * |---------|---------|---------|---------|---------|---------|---------|---------|
 *
 *
 *
 * Byte 5
 * ------
 * Set after any SW update that results in a modification of the DB (at all entries).
 * Cleared (in a specific entry) after informing the corresponding Host.
 *
 *      7         6         5         4         3         2         1         0
 * |---------|---------|---------|---------|---------|---------|---------|---------|
 * |         |         |         |         |         |         |         |         |
 * |                       End of affected handle attr range                       |
 * |         |         |         |         |         |         |         |         |
 * |---------|---------|---------|---------|---------|---------|---------|---------|
 *
 *
 * Bytes 6 : RESERVED
 * ------
 *
 *
 * Byte 7
 * ------
 *      7         6         5         4         3         2         1         0
 * |---------|---------|---------|---------|---------|---------|---------|---------|
 * |         |         |         |         |         |         |         |         |
 * | IRK flg |                          RESERVED                                   |
 * |         |         |         |         |         |         |         |         |
 * |---------|---------|---------|---------|---------|---------|---------|---------|
 *
 *
 * Bytes 8 - ..: app_pairing_env_tag
 * ------
 */

#define EXT_INFO_SRV_CNG_START_HDL_POS   0
#define EXT_INFO_SRV_CNG_START_HDL_MASK  (0xFF << EXT_INFO_SRV_CNG_START_HDL_POS)

#define EXT_INFO_SRV_CNG_END_HDL_POS     8
#define EXT_INFO_SRV_CNG_END_HDL_MASK    (0xFF << EXT_INFO_SRV_CNG_END_HDL_POS)

#define EXT_INFO_IRK_POS                 31
#define EXT_INFO_IRK_MASK                (((uint32_t)1) << EXT_INFO_IRK_POS)

struct bonding_info_s
{
    uint32_t info;
    uint32_t ext_info;
    struct app_pairing_env_tag env;  
};


/*
 * VARIABLES
******************************************************************************************/

extern struct bonding_info_s bond_info;


/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * \brief       Initialize NV memory.
 *
 * \details     Checks the existence of the MAGIC number and the sanity of the usage counters.
 *              If any of the checks fails, it deletes the area of the NV memory which is used
 *              to store the bonding info. 
 *              It initializes bond_usage (usage counters) from the data stored in the NV memory.
******************************************************************************************/
void port_alt_pair_init(void);

/*****************************************************************************************
 * \brief       Read Multi-Bond status.
 *
 * \details     Initializes multi_bond_status from the data stored in the NV memory.
 *              multi_bond_status is a a flag which indicates the validity of host data 
 *              in the available slots. A '1' at a bit position indicates that the corresponding
 *              slot has valid host data. This field also informs about the number of the used slots
 *              (and, consequently, the number of bonded hosts).
 *              If selective delete is added then this field must be updated properly.
******************************************************************************************/
void port_alt_pair_read_status(void);

/*****************************************************************************************
 * \brief       Write Multi-Bond status.
 *
 * \details     Writes the multi_bond_status to the NV memory.
 *              See port_alt_pair_read_status() of info about Multi-Bond status.
******************************************************************************************/
void port_alt_pair_store_status(void);

/*****************************************************************************************
 * \brief       Store the bonding information of the connected Host to the NV memory.
 *
 * \details     First, it looks for an entry for this Host in the NV memory. If one is found,
 *              it updates it only if the Host is using a Random address or the write is
 *              forced because of a DB change (bond_info.info).
 *              If no entry is found in the NV memory for this host then it gets one via
 *              get_entry_to_delete() and writes the data there.
 *              The usage counters are updated in both cases.
******************************************************************************************/
void port_alt_pair_store_bond_data(void);

/*****************************************************************************************
 * \brief       Update CCC notification status on the bonding information.
 *
 * \param[in]   uuid      uuid of the CCC
 * \param[in]   attr_num  Attribute number of the CCC
 * \param[in]   value     value of the attribute
******************************************************************************************/    
void port_alt_pair_store_ccc(uint16_t uuid, int attr_num, int value);

/*****************************************************************************************
 * \brief       Update the DataBase based on the bonding information.
 *
 * \details     Updates the DB with the values of inf. If a modified attribute range
 *              is written in the NV memory then a Service Changed indication (if enabled)
 *              is sent to the Host.
******************************************************************************************/
void port_alt_pair_updatedb_from_bonding_info(void);

/*****************************************************************************************
 * \brief       Find the keys for the connecting host, if available.
 *
 * \details     Search if there's an entry in the NV memory for the connecting host. It is 
 *              called upon the reception of an LL_ENC_REQ message from the host.
 *
 * \param[in]   rand_nb     The RAND_NB value of the LL_ENC_REQ message.
 * \param[in]   ediv        The EDIV value of the LL_ENC_REQ message.
 *
 * \return      int
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> 0 if there's no entry in the NV memory for this host
 *                  <li> 1 if the new host is different from the last host we connected to
 *                  <li> 2 if the new host is the same with the last host we connected to
 *              </ul>
******************************************************************************************/
int port_alt_pair_load_bond_data(const struct rand_nb *rand_nb, uint16_t ediv);

/*****************************************************************************************
 * \brief       Find the first or next entry in the NV memory with valid keys.
 *
 * \details     If init is true then find the first valid entry starting from the beginning.
 *              Used when no connection exists and advertising is about to start!
 *              If init is false then find the next valid entry (if any) starting just 
 *              after the last position used. This can be the position of the current bonded 
 *              host or, if directed advertising has started, the position of the last host 
 *              we did directed advertising to. Used when connected and intend to start 
 *              directed advertising to other hosts for whom bonded data exist or while a 
 *              cycle of directed advertising to all known bonded hosts, until a connection 
 *              is established, is being executed.
 *
 * \warning     Active and Next pointers will be reset when init is true.
 *
 * \param[in]   init    Controls which entry to find; first, if true or next, if false.
 *
 * \return      bool
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> true if a valid entry was found
 *                  <li> false if no valid entry was found. bond_info is reset.
 *              </ul>
******************************************************************************************/
bool port_alt_pair_get_next_bond_data(bool init);

/*****************************************************************************************
 * \brief       Find the last used entry in the NV memory and load keys.
 *
 * \details     Find the last used entry in the NV memory and load the bonding info.
 *
 * \return      bool
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> true if a valid entry was found
 *                  <li> false if the NV memory is empty. bond_info is left unchanged.
 *              </ul>
******************************************************************************************/
bool port_alt_pair_load_last_used(void);

/*****************************************************************************************
 * \brief       Load the keys from a specific entry in the NV memory.
 *
 * \details     Gets the bonding info from a specific entry in the NV memory and, if it's valid,
 *              it copies it to the bond_info and updates the DataBase according to it.
 *
 * \param[in]   entry  The index to the entry in the NV memory.
 *
 * \return      bool
 *
 * \retval      Status of operation.
 *              <ul>
 *                  <li> true if a valid entry was found
 *                  <li> false if there is no valid info at "entry". bond_info is left unchanged.
 *              </ul>
******************************************************************************************/
bool port_alt_pair_load_entry(int8_t entry);

/*****************************************************************************************
 * \brief       Delete a specific entry in the NV memory.
 *
 * \param       entry  The index to the entry in the NV memory to be deleted.
******************************************************************************************/
void port_alt_pair_delete_entry(int8_t entry);

/*****************************************************************************************
 * \brief       Clear the NV memory.
******************************************************************************************/
void port_alt_pair_clear_all_bond_data(void);

/*****************************************************************************************
 * \brief       Initialize the index to the last (or current) used NV memory entry.
******************************************************************************************/
void port_alt_pair_reset_active_peer_pos(void);

/*****************************************************************************************
 * \brief       Get the index to the current (used) NV memory entry.
 *
 * \return      int
 *
 * \retval      The index to the NV memory entry being used. If no entry has been loaded, MAX_BOND_PEER is returned.
******************************************************************************************/
uint8_t port_alt_pair_get_active_index(void);
    
/*****************************************************************************************
 * \brief       Refresh usage counters.
 *
 * \details     Refresh usage counters. idx is the current entry being used.
 *              Thus, it gets the maximum value currently available while all
 *              other entries are decremented by 1.
******************************************************************************************/
void port_alt_pair_update_usage_count(void);

/*****************************************************************************************
 * \brief       Get an entry to use (write).
 *
 * \details     Gets the next available bonding entry for storing the bonding info.
 *              The one that hasn't been used for the longest period is selected, or
 *              an empty one (if available). 
 *
 * \warning     If (MBOND_LOAD_INFO_AT_INIT) is not used (defined as 0), this function accesses the EEPROM.
 *              In this case:
 *              i2c_eeprom_init() must be called before calling this function and   
 *              i2c_eeprom_release() must be called after this function exits.
 *
 * \return      uint8_t
 *
 * \retval      The index of the usage entry to be deleted / written.
******************************************************************************************/
uint8_t port_multi_bond_get_entry_to_delete(void);
    
/*****************************************************************************************
 * \brief Called to check if the device is bonded
 *
 * \return true if the device is bonded, false otherwise 
******************************************************************************************/    
bool port_alt_pair_is_bonded(void);

/*****************************************************************************************
 * \brief Returns the authentication type set for a peer
 *
 * \return uint8_t the authentication type 
******************************************************************************************/
uint8_t port_alt_pair_get_auth(void);

/*****************************************************************************************
 * \brief Sets the authentication type for a connecting peer
 *
 * \param[in] auth The authentication type 
******************************************************************************************/
void port_alt_pair_set_auth(uint8_t auth);

/*****************************************************************************************
 * \brief Returns the current host's address
 *
 * \return struct bd_addr* a pointer to the hosts address
******************************************************************************************/
struct bd_addr * port_alt_pair_get_addr(void);

/*****************************************************************************************
 * \brief  Returns the host's address type
 *
 * \return uint8_t the host's address type
 *
 * \retval ADDR_PUBLIC: public address
 * \retval ADDR_RAND: random address
 * \retval ADDR_RPA_PUBLIC: RPA or public address
 * \retval ADDR_RPA_RAND: RPA or random address
******************************************************************************************/
uint8_t port_alt_pair_get_addr_type(void);

/*****************************************************************************************
 * \brief Sets the address and address type of the active connecting peer
 *
 * \param[in]  addr_type The address type of the incoming peer
 * \param[in]  addr      A pointer to the address of the incoming peer
******************************************************************************************/
void port_alt_pair_set_addr(uint8_t addr_type, struct bd_addr *addr);

/*****************************************************************************************
 * \brief Returns the Random Number of the active connection
 *
 * \return  struct rand_nb * a pointer to the Random Number
******************************************************************************************/
struct rand_nb * port_alt_pair_get_rand(void);

/*****************************************************************************************
 * \brief  Returns the Encrypted Diversifier of the active connection
 *
 * \return uint16_t The Encrypted Diversifier
******************************************************************************************/
uint16_t port_alt_pair_get_ediv(void);

/*****************************************************************************************
 * \brief Returns the Identity Resolution Key of the current connection
 *
 * \return  gap_sec_key *  A pointer to the IRK
******************************************************************************************/
struct gap_sec_key * port_alt_pair_get_irk(void);

/*****************************************************************************************
 * \brief Sets the Identity Resolution Key acquired from the current connection
 *
 * \param[in] irk A pointer to the IRK
******************************************************************************************/
void port_alt_pair_set_irk(struct gapc_irk *irk);

/*****************************************************************************************
 * \brief
 *
 * \return
******************************************************************************************/
struct app_pairing_env_tag * port_alt_pair_get_env(void);

/*****************************************************************************************
 * \brief  Sets default pairing info (clears current pairing information)     
******************************************************************************************/
void port_alt_pair_set_default_info(void);

/*****************************************************************************************
 * \brief Resets the bonding data      
******************************************************************************************/
void port_alt_pair_reset_bonding_data(void);

/*****************************************************************************************
 * \brief       
******************************************************************************************/
void port_alt_pair_clear_service_changes_hdls(void);

/*****************************************************************************************
 * \brief Returns the numbers of bonded hosts
 *
 * \return uint8_t the number of bonded hosts 
******************************************************************************************/
uint8_t port_alt_pair_get_num_of_bonded_hosts(void);

/*****************************************************************************************
 * \brief Returns the numbers of the current active peers
 *
 * \return uint8_t the number of current active peers
******************************************************************************************/
uint8_t port_alt_pair_get_num_of_peers(void);

/*****************************************************************************************
 * \brief Gets the Identity Resolution Key list
 *
 * \param[out] irk A pointer to where the IRK list is going to be placed 
******************************************************************************************/
void port_alt_pair_get_irk_list(struct gap_sec_key irk[]);

/*****************************************************************************************
 * \brief  Searches for a specific IRK and returns the index to which it is placed
 *
 * \param[in] irk A pointer to the IRK we want to search 
 * \param[in] start_pos the index to start the search
 *
 * \return uint8_t The index inside the bonding info that the given IRK is found 
******************************************************************************************/
uint8_t port_alt_pair_find_irk(struct gap_sec_key *irk, uint8_t start_pos);

/*****************************************************************************************
 * \brief Clears the resolved hosts index      
******************************************************************************************/
void port_alt_pair_clear_resolved_pos(void);

#ifdef FORCE_CONNECT_TO_HOST_ON
/*****************************************************************************************
 * \brief     Force storing of the bonding info of the next host that will be bonded
 *            to the specified position
 *          
 * \param[in] entry  The position in the storage memory where the bonding data
 *                   will be stored
 * 
 * \return    previous entry in use
******************************************************************************************/
uint8_t app_alt_pair_force_next_store_entry(uint8_t entry);

/*****************************************************************************************
 * \brief     Reset force storing of the bonding info
******************************************************************************************/
void app_alt_pair_reset_force_next_store_entry(void);

#endif
    
#endif // APP_MULTI_BOND_H_

/**
 * \}
 * \}
 * \}
 */
