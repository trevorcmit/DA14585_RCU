/******************************************************************************************
 * @file app_bond_db.c
 * @brief Bond database code file.
******************************************************************************************/

/*****************************************************************************************
 * @addtogroup APP_BOND_DB
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include "rwip_config.h"             // SW configuration
#include "rwip.h"

#if (BLE_APP_PRESENT)

    #include "app_bond_db.h"

    #if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
    #include "spi_flash.h"
    #elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)
    #include "i2c_eeprom.h"
    #endif

/*
 * DEFINES
******************************************************************************************/

#define BOND_DB_SLOT_NOT_FOUND          (-1)

/*
 * TYPE DEFINITIONS
******************************************************************************************/

struct bond_db
{
    uint16_t start_hdr;
    struct bond_db_data data[APP_BOND_DB_MAX_BONDED_PEERS];
    uint8_t next_slot;
    uint16_t end_hdr;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

static struct bond_db bdb __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY

/*
 * FUCTION DEFINITIONS
******************************************************************************************/

#if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)

static void bond_db_spi_flash_init(void)
{
    static int8_t dev_id;

    dev_id = spi_flash_enable(SPI_EN_GPIO_PORT, SPI_EN_GPIO_PIN);
    if (dev_id == SPI_FLASH_AUTO_DETECT_NOT_DETECTED)
    {
        // The device was not identified. The default parameters are used.
        // Alternatively, an error can be asserted here.
        spi_flash_init(SPI_FLASH_DEFAULT_SIZE, SPI_FLASH_DEFAULT_PAGE);
    }
}

static void bond_db_load_flash(void)
{
    bond_db_spi_flash_init();

    spi_flash_read_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db));

    // Power down flash
    spi_flash_power_down();

    spi_release();
}

/*****************************************************************************************
 * @brief Erase Flash sectors where bond database is stored
 * @param[in] scheduler_en  True: Enable rwip_scheduler while Flash is being erased
 *                          False: Do not enable rwip_scheduler. Blocking mode
 * @return ret              Error code or success (ERR_OK)
******************************************************************************************/
static int8_t bond_db_erase_flash_sectors(bool scheduler_en)
{
    uint32_t sector_nb;
    uint32_t offset;
    int8_t ret;
    int i;
    uint16_t timeout_cnt;

    // Calculate the starting sector offset
    offset = (APP_BOND_DB_DATA_OFFSET / SPI_SECTOR_SIZE) * SPI_SECTOR_SIZE;

    // Calculate the numbers of sectors to erase
    sector_nb = (sizeof(bdb) / SPI_SECTOR_SIZE);
    if (sizeof(bdb) % SPI_SECTOR_SIZE)
        sector_nb++;

    for (i = 0; i < sector_nb; i++)
    {
        if (scheduler_en)
        {
            // Non-Blocking Erase of a Flash sector
            ret = spi_flash_block_erase_no_wait(offset, SECTOR_ERASE);
            if (ret != ERR_OK)
                break;

            timeout_cnt = 0;

            while ((spi_flash_read_status_reg() & STATUS_BUSY) != 0)
            {
                // Check if BLE is on and not in deep sleep and call rwip_schedule()
                if ((GetBits16(CLK_RADIO_REG, BLE_ENABLE) == 1) &&
                   (GetBits32(BLE_DEEPSLCNTL_REG, DEEP_SLEEP_STAT) == 0))
                {
                    // Assuming that the WDG is not active, timeout will be reached in case of a Flash erase error.
                    // NOTE: In case the WDG is active, the WDG timer will expire (much) earlier than the timeout
                    // is reached and therefore an NMI will be triggered.
                    if (++timeout_cnt > MAX_READY_WAIT_COUNT)
                    {
                        return ERR_TIMEOUT;
                    }
                    rwip_schedule();
                }
            }
        }
        else
        {
            // Blocking Erase of a Flash sector
            ret = spi_flash_block_erase(offset, SECTOR_ERASE);
            if (ret != ERR_OK)
                break;
        }
        offset += SPI_SECTOR_SIZE;
    }

    return ret;
}


/*****************************************************************************************
 * @brief Store Bond Database to Flash memory
 * @param[in] scheduler_en  True: Enable rwip_scheduler while Flash is being erased
 *                          False: Do not enable rwip_scheduler. Blocking mode
 * @return none
******************************************************************************************/
static void bond_db_store_flash(bool scheduler_en)
{
    int8_t ret;

    bond_db_spi_flash_init();

    ret = bond_db_erase_flash_sectors(scheduler_en);
    if (ret == ERR_OK)
    {
        spi_flash_write_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db));
    }

    // Power down flash
    spi_flash_power_down();

    spi_release();
}

#elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)

static void bond_db_load_eeprom(void)
{
    uint32_t bytes_read;

    i2c_eeprom_init(I2C_SLAVE_ADDRESS, I2C_SPEED_MODE, I2C_ADDRESS_MODE, I2C_ADDRESS_SIZE);

    i2c_eeprom_read_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db), &bytes_read);
    ASSERT_ERROR(bytes_read == sizeof(struct bond_db));

    i2c_eeprom_release();
}

static void bond_db_store_eeprom(void)
{
    uint32_t bytes_written;

    i2c_eeprom_init(I2C_SLAVE_ADDRESS, I2C_SPEED_MODE, I2C_ADDRESS_MODE, I2C_ADDRESS_SIZE);

    i2c_eeprom_write_data((uint8_t *)&bdb, APP_BOND_DB_DATA_OFFSET, sizeof(struct bond_db), &bytes_written);
    ASSERT_ERROR(bytes_written == sizeof(struct bond_db));

    i2c_eeprom_release();
}

#endif

/*****************************************************************************************
 * @brief Load Bond Database from external memory
 * @return none
******************************************************************************************/
static inline void bond_db_load_ext(void)
{
    #if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
    bond_db_load_flash();
    #elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)
    bond_db_load_eeprom();
    #endif
}

/*****************************************************************************************
 * @brief Store Bond Database to external memory
 * @param[in] scheduler_en  Only used if external memory is Flash
                            True: Enable rwip_scheduler while Flash is being erased
 *                          False: Do not enable rwip_scheduler. Blocking mode
 * @return none
******************************************************************************************/
static inline void bond_db_store_ext(bool scheduler_en)
{
    #if defined (USER_CFG_APP_BOND_DB_USE_SPI_FLASH)
    bond_db_store_flash(scheduler_en);
    #elif defined (USER_CFG_APP_BOND_DB_USE_I2C_EEPROM)
    bond_db_store_eeprom();
    #endif
}

/*****************************************************************************************
 * @brief Store Bond data entry to external memory
 * @param[in] *data  Data to be stored
 * @param[in] *idx   Entry in the database
 * @return none
******************************************************************************************/
static void bond_db_store_at_idx(struct bond_db_data *data, int idx)
{
    // Update the cache
    memcpy(&bdb.data[idx], data, sizeof(struct bond_db_data));
    // Store new bond data to external memory
    // In case of Flash (erase then write) enable the scheduler
    bond_db_store_ext(true);
}

void bond_db_init(void)
{
    // Load bond data from the external memory resource
    bond_db_load_ext();

    // Simple check for garbage in memory (this also catches the 0xFF of cleared memory)
    if ((bdb.next_slot > APP_BOND_DB_MAX_BONDED_PEERS) ||
        (bdb.start_hdr != BOND_DB_HEADER_START) || (bdb.end_hdr != BOND_DB_HEADER_END))
    {
        bond_db_clear();
    }
}

/*****************************************************************************************
 * @brief Updates the least recently used ID of each slot, using the following criteria
 *        * Least recently used: LRU ID = APP_BOND_DB_MAX_BONDED_PEERS-1
 *        * Most recently used: LRU ID = 0
 *        * If slot_to_replace_found is false, all non-empty entries increment their LRU ID
 *        * If slot_to_replace_found is true, all non-empty entries, with LRU ID less than
 *          the slot_to_write LRU ID, increment their LRU ID
 *        * LRU of slot_to_write takes zero value
 * @param[in] slot_to_write          Slot to write
 * @param[in] slot_to_replace_found  True if slot_to_write is not the oldest written slot
 * @return void
******************************************************************************************/
static void bond_db_update_lru(int slot_to_write, bool slot_to_replace_found)
{
    uint8_t idx = 0;
    uint8_t lru_to_stop_increment = APP_BOND_DB_MAX_BONDED_PEERS;

    // If the slot that have been written was not the oldest one increment valid entries
    // LRU IDs only if current LRU ID is less than the slot_to_write LRU ID
    if (slot_to_replace_found)
    {
        lru_to_stop_increment = bdb.data[slot_to_write].lru;
    }

    // For every entry in bond DB
    for(idx = 0; idx < APP_BOND_DB_MAX_BONDED_PEERS; idx++)
    {
        // If the entry is valid (not empty)
        if (bdb.data[idx].valid == BOND_DB_VALID_ENTRY)
        {
            if (bdb.data[idx].lru < lru_to_stop_increment)
            {
                // Increase the lru flag of the entry
                bdb.data[idx].lru++;
                bdb.data[idx].lru = bdb.data[idx].lru % APP_BOND_DB_MAX_BONDED_PEERS;
            }
        }
    }
    // LRU of slot_to_write takes zero value
    bdb.data[slot_to_write].lru = 0;
}

void bond_db_store(struct bond_db_data *data)
{
    bool first_empty_slot_found = false;
    bool slot_to_replace_found = false;
    uint8_t idx = 0;
    int slot_to_write = BOND_DB_SLOT_NOT_FOUND;

    for(idx = 0; idx < APP_BOND_DB_MAX_BONDED_PEERS; idx++)
    {
        // If current slot is not valid (is empty)
        if (bdb.data[idx].valid != BOND_DB_VALID_ENTRY)
        {
            // Check if an empty slot has already been found
            if (first_empty_slot_found == false)
            {
                // First empty slot has been found
                first_empty_slot_found = true;
                // Store empty slot to write
                slot_to_write = idx;
            }
        }
        // If current slot is not empty
        else
        {
            // Check if IRK is present
            if (bdb.data[idx].flags & BOND_DB_ENTRY_IRK_PRESENT)
            {
                // Check if stored IRK matches with new IRK
                if (memcmp(&data->irk.irk, &bdb.data[idx].irk.irk, sizeof(struct gap_sec_key)) == 0)
                {
                    // IRK matches, store this slot to be replaced and exit
                    slot_to_write = idx;
                    slot_to_replace_found = true;
                    break;
                }
            }
            // If IRK is not present
            else
            {
                // Check if stored BD address matches with new BD address
                if (memcmp(&data->bdaddr, &bdb.data[idx].bdaddr, sizeof(struct gap_bdaddr)) == 0)
                {
                    // BD address matches, store this slot to be replaced and exit
                    slot_to_write = idx;
                    slot_to_replace_found = true;
                    break;
                }
            }
        }
    }

    // If there is no available slot, find the least recently written slot to replace
    if (slot_to_write == BOND_DB_SLOT_NOT_FOUND)
    {
        for(idx = 0; idx < APP_BOND_DB_MAX_BONDED_PEERS; idx++)
        {
            // The least recently written slot will have LRU ID = APP_BOND_DB_MAX_BONDED_PEERS-1
            if (bdb.data[idx].lru == APP_BOND_DB_MAX_BONDED_PEERS-1)
            {
                slot_to_write = idx;
                break;
            }
        }
    }

    // Update LRU values of bond db entries
    bond_db_update_lru(slot_to_write, slot_to_replace_found);

    // Store bond db entry
    bond_db_store_at_idx(data, slot_to_write);
}

const struct bond_db_data* bond_db_lookup_by_ediv(const struct rand_nb *rand_nb,
                                                  uint16_t ediv)
{
    int idx = 0;
    struct bond_db_data *found_data = 0;

    // search if it already exists in db
    for(idx = 0; idx < APP_BOND_DB_MAX_BONDED_PEERS; ++idx)
    {
        // match rand_nd and ediv
        if ( 0 == memcmp(rand_nb, &bdb.data[idx].ltk.randnb, RAND_NB_LEN)
            && ediv == bdb.data[idx].ltk.ediv )
        {
            found_data = &bdb.data[idx];
            break;
        }
    }

    return found_data;
}

void bond_db_clear(void)
{
    memset((void *)&bdb, 0, sizeof(struct bond_db) ); // zero bond data
    bdb.start_hdr = BOND_DB_HEADER_START;
    bdb.end_hdr = BOND_DB_HEADER_END;
    // Store zero bond data to external memory
    // In case of Flash (erase then write) do not enable the scheduler
    bond_db_store_ext(false);
}

#endif  // BLE_APP_PRESENT

/// @} APP_BOND_DB
