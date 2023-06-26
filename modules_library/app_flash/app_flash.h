/*****************************************************************************************
 *
 * \file app_flash.h
 *
 * \brief SPI flash handling functions.
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
 * \addtogroup FLASH
 * \{
 * \addtogroup APP_FLASH
 * \{
 ****************************************************************************************	 	 
 */
 
#ifndef _APP_FLASH_
#define _APP_FLASH_
 
#ifdef HAS_SPI_FLASH_STORAGE

typedef void (*flash_erase_callback_t)(void);

#include <app_flash_config.h>
    
#include "spi_flash.h"
#include "gpio.h"
#include <string.h>
#include "app_dbg.h"

/*****************************************************************************************
 * \brief
******************************************************************************************/
void app_spi_flash_power_down(void);

/*****************************************************************************************
 * \brief Initializes the flash pins and the SPI interface.
 *
 * \param[in]   freq: The frequency of the SPI bus block
******************************************************************************************/
void app_spi_flash_peripheral_init(SPI_XTAL_Freq_t freq);
    
/*****************************************************************************************
 * \brief Releases the flash pins and the SPI interface.
******************************************************************************************/
void app_spi_flash_peripheral_release(void);

/*****************************************************************************************
 * \brief Simplified write data to a single page in flash. Used to write bonding data.
 *
 * \param[in]   data: pointer to the data.
 * \param[in]   address: the address in the flash memory.
 * \param[in]   size: the amount of data to write.
 *
 * \return      Amount of data written.
******************************************************************************************/
size_t app_spi_flash_write_random_page_data(const void *data, uint32_t address, size_t size);

/*****************************************************************************************
 * \brief Write debug registers to flash.
 *
 * \param[in]   fault: type of fault that occurred.
 *
 * \return      APP_DBG_FAULT_NONE if writing succeeded, APP_DBG_FAULT_FLASH else.
******************************************************************************************/
app_dbg_fault_t app_spi_flash_write_dbg_regs(app_dbg_fault_t fault);

/*****************************************************************************************
 * \brief Read debug registers from flash.
 *
 * \return      type of fault that occurred, or APP_DBG_FAULT_FLASH if reading failed.
******************************************************************************************/
app_dbg_fault_t app_spi_flash_read_dbg_regs(void);

/*****************************************************************************************
 * \brief Erase debug registers from flash.
 *
 * \return      APP_DBG_FAULT_NONE if erasing succeeded, APP_DBG_FAULT_FLASH else.
******************************************************************************************/
app_dbg_fault_t app_spi_flash_erase_dbg_regs(void);

/*****************************************************************************************
 * \brief Write debug message to flash.
 *
 * \param[in]   msg: pointer to the message.
 *
 * \return      APP_DBG_FAULT_NONE if writing succeeded, APP_DBG_FAULT_FLASH else.
******************************************************************************************/
app_dbg_fault_t app_spi_flash_write_dbg_msg(const char *msg);

/*****************************************************************************************
 * \brief Read debug message from flash.
 *
 * \param[in]   msg: pointer to the message bufer
 * \param[in]   len: message buffer size
 *
 * \return      APP_DBG_FAULT_NONE if reading succeeded, APP_DBG_FAULT_FLASH else.
******************************************************************************************/
app_dbg_fault_t app_spi_flash_read_dbg_msg(char *msg, size_t len);

/*****************************************************************************************
 * \brief Erase debug message from flash.
 *
 * \return      APP_DBG_FAULT_NONE if erasing succeeded, APP_DBG_FAULT_FLASH else.
******************************************************************************************/
app_dbg_fault_t app_spi_flash_erase_dbg_msg(void);


#ifdef HAS_FLASH_SPI_POWER_DOWN
/*****************************************************************************************
 * \brief Declare SPI flash power-down GPIO
******************************************************************************************/
__INLINE void declare_flash_spi_power_down_gpios(void)
{
    PORT_RESERVE_GPIO(app_flash_pins[FLASH_POWER_DOWN_PIN]);
}

/*****************************************************************************************
 * \brief Power-down SPI flash using external circuit.
 *
 * \remarks An external circuit controled by pin defined as FLASH_POWER_DOWN_PIN
 *          is used to control the device power.
******************************************************************************************/
__INLINE void flash_spi_power_down(void)
{
    PORT_CONFIGURE_GPIO(app_flash_pins[FLASH_POWER_DOWN_PIN], OUTPUT, PID_GPIO, true);
}

/*****************************************************************************************
 * \brief Power-up SPI flash using external circuit.
 *
 * \remarks An external circuit controled by pin definedas FLASH_POWER_DOWN_PIN
 *          is used to control the device power.
******************************************************************************************/
__INLINE void flash_spi_power_up(void)
{
    PORT_CONFIGURE_GPIO(app_flash_pins[FLASH_POWER_DOWN_PIN], OUTPUT, PID_GPIO, false);
}

#endif

/*****************************************************************************************
 * \brief Declare SPI flash GPIOs
******************************************************************************************/
__INLINE void flash_declare_spi_gpios(void)
{
#ifdef HAS_FLASH_SPI_POWER_DOWN
    declare_flash_spi_power_down_gpios();
#endif    
    PORT_RESERVE_GPIO(app_flash_spi_pins[FLASH_SPI_CLK_PIN]); 
    PORT_RESERVE_GPIO(app_flash_spi_pins[FLASH_SPI_DO_PIN]); 
    PORT_RESERVE_GPIO(app_flash_spi_pins[FLASH_SPI_DI_PIN]); 
    PORT_RESERVE_GPIO(app_flash_pins[FLASH_SPI_CS_PIN]); 
}

/*****************************************************************************************
 * \brief Declare SPI flash CS
******************************************************************************************/
__INLINE void flash_declare_spi_cs_gpio(void)
{
#ifdef HAS_FLASH_SPI_POWER_DOWN
    declare_flash_spi_power_down_gpios();
#endif    
    PORT_RESERVE_GPIO(app_flash_pins[FLASH_SPI_CS_PIN]); 
}

/*****************************************************************************************
 * \brief Activate SPI flash GPIOs
******************************************************************************************/
__INLINE void activate_spi_flash_gpios(void)
{
#ifdef HAS_FLASH_SPI_POWER_DOWN
    flash_spi_power_up();
#endif
    PORT_CONFIGURE_GPIO(app_flash_pins[FLASH_SPI_CS_PIN], OUTPUT, PID_SPI_EN, true);
    PORT_CONFIGURE_GPIO(app_flash_spi_pins[FLASH_SPI_CLK_PIN], OUTPUT, PID_SPI_CLK, false );
    PORT_CONFIGURE_GPIO(app_flash_spi_pins[FLASH_SPI_DO_PIN],  OUTPUT, PID_SPI_DO,  false );
    PORT_CONFIGURE_GPIO(app_flash_spi_pins[FLASH_SPI_DI_PIN],  INPUT,  PID_SPI_DI,  false );
}

/*****************************************************************************************
 * \brief Deactivate SPI flash GPIOs
******************************************************************************************/
__INLINE void deactivate_spi_flash_gpios(void)
{
#ifdef HAS_FLASH_SPI_POWER_DOWN
    flash_spi_power_down();
#endif    
    PORT_SET_PIN_FUNCTION_DEFAULT(app_flash_pins[FLASH_SPI_CS_PIN]);
    PORT_SET_PIN_FUNCTION_DEFAULT(app_flash_spi_pins[FLASH_SPI_CLK_PIN]);
    PORT_SET_PIN_FUNCTION_DEFAULT(app_flash_spi_pins[FLASH_SPI_DO_PIN]); 
    PORT_SET_PIN_FUNCTION_DEFAULT(app_flash_spi_pins[FLASH_SPI_DI_PIN]); 
}

/*****************************************************************************************
 * \brief Dectivate SPI flash CS
******************************************************************************************/
__INLINE void deactivate_spi_flash_cs_gpio(void)
{
#ifdef HAS_FLASH_SPI_POWER_DOWN
    flash_spi_power_down();
#endif    
    PORT_SET_PIN_FUNCTION_DEFAULT(app_flash_pins[FLASH_SPI_CS_PIN]);
}

#endif // HAS_SPI_FLASH_STORAGE

#endif // _APP_FLASH_

/**
 * \}
 * \}
 * \}
 */
