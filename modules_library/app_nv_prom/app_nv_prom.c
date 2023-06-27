/*****************************************************************************************
 *
 * \file app_nv_prom.c
 *
 * \brief Provide simplified access to various types of NV PROM devices.
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

#if !defined(SPI_FLASH_EEPROM_AUTOCONFIG) && !defined(HAS_SPI_EEPROM_STORAGE) && \
    !defined(HAS_I2C_EEPROM_STORAGE) && !defined(HAS_SPI_FLASH_STORAGE)
    #warning "No NV memory type has been selected"
#else
/*
 * INCLUDE FILES
******************************************************************************************/
    #include "app_nv_prom.h"

    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
enum prom_type spi_prom_type     __PORT_RETAINED;
uint32_t NV_STORAGE_BASE_ADDR    __PORT_RETAINED;
    #endif


void nv_prom_init(void)
{    
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    if(spi_prom_type == PROM_SPI_FLASH)
    #endif
    {
    #ifdef HAS_SPI_FLASH_STORAGE
        app_spi_flash_peripheral_init(SPI_XTAL_DIV_2);
    #endif
    }
        
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    else
    #endif
    {        
    #ifdef HAS_SPI_EEPROM_STORAGE
       spi_eeprom_init();
    #endif
    } 
    
    #ifdef HAS_I2C_EEPROM_STORAGE
    i2c_eeprom_init(I2C_SLAVE_ADDRESS, I2C_SPEED_MODE, I2C_ADDRESS_MODE,
                    I2C_ADRESS_BYTES_CNT);
    #endif
}


void nv_prom_release(void)
{
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    if(spi_prom_type == PROM_SPI_FLASH)
    #endif
    {
    #ifdef HAS_SPI_FLASH_STORAGE
        app_spi_flash_peripheral_release();
    #endif
    }  
    
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    else
    #endif
    {
    #ifdef HAS_SPI_EEPROM_STORAGE
        spi_eeprom_release();
    #endif
    }
    
    #ifdef HAS_I2C_EEPROM_STORAGE
    i2c_eeprom_release();
    #endif
}


uint32_t nv_prom_init_read_data(uint8_t *data, uint32_t address, uint32_t size) 
{
    uint32_t ret;
    
    nv_prom_init();
    ret=nv_prom_read_data(data, address, size);  
    nv_prom_release();    
    return ret;
}


uint32_t nv_prom_read_data(uint8_t *data, uint32_t address, uint32_t size) 
{
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    if(spi_prom_type == PROM_SPI_FLASH)
    #endif
    {
    #ifdef HAS_SPI_FLASH_STORAGE
        return spi_flash_read_data(data, address, size);
    #endif
    }
    
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    else
    #endif
    {
    #ifdef HAS_SPI_EEPROM_STORAGE
        return spi_eeprom_read_data(data, address, size);
    #endif
    }
    
    #ifdef HAS_I2C_EEPROM_STORAGE
    uint32_t byte_read;
    i2c_eeprom_read_data(data, address, size, &byte_read);
    return byte_read;
    #endif
        
    #if !defined(HAS_SPI_FLASH_STORAGE) && !defined(HAS_SPI_EEPROM_STORAGE) && !defined(HAS_I2C_EEPROM_STORAGE)
    return 0;
    #endif    
}


void nv_prom_write_byte(uint32_t address, uint8_t data)
{
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    if(spi_prom_type == PROM_SPI_FLASH)
    #endif
    {
    #ifdef HAS_SPI_FLASH_STORAGE
        app_spi_flash_write_random_page_data(&data, address, sizeof(uint8_t));
    #endif
    }
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    else
    #endif
    {
    #ifdef HAS_SPI_EEPROM_STORAGE
        spi_eeprom_write_byte(data, address);
    #endif
    }

    #ifdef HAS_I2C_EEPROM_STORAGE
    i2c_eeprom_write_byte(address, data);
    #endif
}


uint32_t nv_prom_init_write_data(uint8_t *data, uint32_t address, uint32_t size) 
{
    uint32_t ret;
    
    nv_prom_init();
    ret=nv_prom_write_data(data, address, size);
    nv_prom_release();
    return ret;
}


uint32_t nv_prom_write_data(uint8_t *data, uint32_t address, uint32_t size)
{
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    if(spi_prom_type == PROM_SPI_FLASH)
    #endif
    {
    #ifdef HAS_SPI_FLASH_STORAGE
        return app_spi_flash_write_random_page_data(data, address, size);
    #endif
    }
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
    else
    #endif
    {        
    #ifdef HAS_SPI_EEPROM_STORAGE
        return spi_eeprom_write_data(data, address, size);
    #endif
    }
    
    #ifdef HAS_I2C_EEPROM_STORAGE
    uint32_t byte_read;
    i2c_eeprom_write_data(data, address, size, &byte_read);
    return byte_read;
    #endif
    
    #if !defined(HAS_SPI_FLASH_STORAGE) && !defined(HAS_SPI_EEPROM_STORAGE) && !defined(HAS_I2C_EEPROM_STORAGE)
    return 0;
    #endif    
}


void app_nv_prom_spi_autodetect(void)
{
    #ifdef SPI_FLASH_EEPROM_AUTOCONFIG
//    int8_t detected_spi_flash_device_index;
    
    // assume that an SPI Flash is used
    spi_prom_type = PROM_SPI_FLASH;
    NV_STORAGE_BASE_ADDR    = SPI_FLASH_BONDING_INFO_BASE_ADDR;

    // detect SPI flash IC
    app_spi_flash_peripheral_init(SPI_XTAL_DIV_14);
//    detected_spi_flash_device_index = spi_flash_auto_detect();
    uint32_t jedec_id;
	uint8_t i;
	
	jedec_id = spi_read_flash_jedec_id();
//    detected_spi_flash_device_index = SPI_FLASH_AUTO_DETECT_NOT_DETECTED;
    
	for (i=0; i<SPI_FLASH_DEVICES_SUPPORTED_COUNT; i++) {
		if ( (jedec_id & SPI_FLASH_KNOWN_DEVICES_PARAMETERS_LIST[i].jedec_id_matching_bitmask) ==
		     SPI_FLASH_KNOWN_DEVICES_PARAMETERS_LIST[i].jedec_id )
		{
//			detected_spi_flash_device_index = i;
            break;
		}	
	}

    app_spi_flash_peripheral_release();

    // if the SPI Flash is not detected then an SPI EEPROM is used    
//    if(detected_spi_flash_device_index == SPI_FLASH_AUTO_DETECT_NOT_DETECTED) {
    if(i == SPI_FLASH_DEVICES_SUPPORTED_COUNT) {
        spi_prom_type = PROM_SPI_EEPROM;        
        NV_STORAGE_BASE_ADDR    = SPI_EEPROM_BONDING_INFO_BASE_ADDR;
    }    
    #endif    
}

#endif

/**
 * \}
 * \}
 * \}
 */
