/*****************************************************************************************
 *
 * \file app_nv_prom.h
 *
 * \brief Provide simplified access to various types of NV PROM devices.
 *
 * Define symbol HAS_SPI_FLASH_STORAGE, HAS_SPI_EEPROM_STORAGE or HAS_I2C_EEPROM_STORAGE 
 * to include this module in the application.
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup NV_PROM
 * \{
 * \addtogroup APP_NV_PROM
 *
 * \brief Simplified access to various types of NV PROM devices.
 * \{
 ****************************************************************************************	 	 
 */

#ifndef APP_NV_PROM_H_
#define APP_NV_PROM_H_

#include <stdint.h>

enum prom_type {
   PROM_SPI_FLASH,
   PROM_SPI_EEPROM,
   PROM_I2C_EEPROM,
   PROM_NONE
};

#ifdef SPI_FLASH_EEPROM_AUTOCONFIG        
    extern enum prom_type spi_prom_type;
#endif

#ifdef HAS_I2C_EEPROM_STORAGE
    #include "i2c_eeprom.h"

    #ifndef HAS_I2C
        #define HAS_I2C
    #endif
//    #include "app_i2c_eeprom_config.h"
#endif

#ifdef HAS_SPI_FLASH_STORAGE
    #include "app_flash.h"
    #include <app_flash_config.h>

    #ifndef HAS_SPI
        #define HAS_SPI
    #endif
#endif

#ifdef HAS_SPI_EEPROM_STORAGE
    #include "spi_eeprom.h"
    #include "app_spi_eeprom_config.h"    
#endif

#ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    extern uint32_t NV_STORAGE_BASE_ADDR;
    extern uint32_t CONFIGURATION_BASE_ADDR;
#elif defined(HAS_SPI_FLASH_STORAGE)
    #define NV_STORAGE_BASE_ADDR      SPI_FLASH_BONDING_INFO_BASE_ADDR	
#elif defined(HAS_SPI_EEPROM_STORAGE)
    #define NV_STORAGE_BASE_ADDR      SPI_EEPROM_BONDING_INFO_BASE_ADDR				
#elif defined(HAS_I2C_EEPROM_STORAGE)
    #define NV_STORAGE_BASE_ADDR      I2C_EEPROM_BONDING_INFO_BASE_ADDR
#else    
    #define NV_STORAGE_BASE_ADDR      0
#endif

/*****************************************************************************************
 * \brief       Initialize the storage memory for read or write operation
******************************************************************************************/
void nv_prom_init(void);
 
 
/*****************************************************************************************
 * \brief       Release the storage memory after read or write operation
******************************************************************************************/
void nv_prom_release(void);

/*****************************************************************************************
 * \brief       Get the NV PROM type
 *
 * \return      NV PROM type
******************************************************************************************/
__forceinline static enum prom_type nv_prom_get_type(void) 
{
#if defined(SPI_FLASH_EEPROM_AUTOCONFIG)
    return spi_prom_type;
#elif defined(HAS_SPI_FLASH_STORAGE)
    return PROM_SPI_FLASH;
#elif defined(HAS_SPI_EEPROM_STORAGE)
    return PROM_SPI_EEPROM;
#elif defined(HAS_I2C_EEPROM_STORAGE)
    return PROM_I2C_EEPROM;
#else    
    return PROM_NONE;
#endif   
}

/*****************************************************************************************
 * \brief Simplified write data to a single page in storage memory including device 
 *        initialize and release
 *
 * \param[in]   data: pointer to the data
 * \param[in]   address: the address in the storage memory
 * \param[in]   size: the amount of data to write
 *
 * \return      Amount of data written
******************************************************************************************/
uint32_t nv_prom_init_write_data(uint8_t *data, uint32_t address, uint32_t size);

/*****************************************************************************************
 * \brief Simplified write data to a single page in storage memory
 *
 * \param[in]   data: pointer to the data
 * \param[in]   address: the address in the storage memory
 * \param[in]   size: the amount of data to write
 *
 * \return      Amount of data written
******************************************************************************************/
uint32_t nv_prom_write_data(uint8_t *data, uint32_t address, uint32_t size);

/*****************************************************************************************
 * \brief Simplified write byte 
 *
 * \param[in]   address: the address in the storage memory
 * \param[in]   data: byte to be written
******************************************************************************************/
void nv_prom_write_byte(uint32_t address, uint8_t data);

/*****************************************************************************************
 * \brief Simplified read data from storage memory including device initialize and 
 *        release
 *
 * \param[in]   data: pointer to the data
 * \param[in]   address: the address in the storage memory
 * \param[in]   size: the amount of data to read
 *
 * \return      Amount of data read
******************************************************************************************/
uint32_t nv_prom_init_read_data(uint8_t *data, uint32_t address, uint32_t size);

/*****************************************************************************************
 * \brief Simplified read data from storage memory
 *
 * \param[in]   data: pointer to the data
 * \param[in]   address: the address in the storage memory
 * \param[in]   size: the amount of data to read
 *
 * \return      Amount of data read
******************************************************************************************/
uint32_t nv_prom_read_data(uint8_t *data, uint32_t address, uint32_t size); 

/*****************************************************************************************
 * \brief       Autodetect SPI device (EEPROM or Flash)
******************************************************************************************/
void app_nv_prom_spi_autodetect(void);


#endif // APP_NV_PROM_H_

/**
 * \}
 * \}
 * \}
 */
