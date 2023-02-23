/**
 ****************************************************************************************
 *
 * \file app_flash.h
 *
 * \brief Implementation of SPI flash handling functions.
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
 * \addtogroup FLASH
 * \{
 * \addtogroup APP_FLASH
 * \{
 ****************************************************************************************	 	 
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#ifdef HAS_SPI_FLASH_STORAGE

#include "app_flash.h"

#ifdef NV_DEBUG_BASE_ADDR
    #define APP_FLASH_BASE      NV_DEBUG_BASE_ADDR
#else
    #define APP_FLASH_BASE      0x3E000
#endif

#define APP_FLASH_MSG_SIZE  SPI_FLASH_SECTOR - APP_DBG_REGS_SIZE
#define APP_FLASH_REGS_BASE APP_FLASH_BASE + APP_FLASH_MSG_SIZE

bool power_up_delay;

SPI_Pad_t spi_FLASH_CS_Pad;

void app_spi_flash_power_down(void)
{
    app_spi_flash_peripheral_init(SPI_XTAL_DIV_2);
    app_spi_flash_peripheral_release();
}


void app_spi_flash_peripheral_init(SPI_XTAL_Freq_t freq)
{
#ifdef HAS_FLASH_SPI_POWER_DOWN
    // power_up_delay is used to delay the first write operation
    // until SPI Flash is ready after power up.
    power_up_delay=true; 
#endif

    //Initialize the SPI interface and power up the SPI flash if needed
    //in case it is shared with another device on different pins.
    activate_spi_flash_gpios();
        
	spi_FLASH_CS_Pad.pin  = (GPIO_PIN)app_flash_pins[FLASH_SPI_CS_PIN].pin;
    spi_FLASH_CS_Pad.port = (GPIO_PORT)app_flash_pins[FLASH_SPI_CS_PIN].port;
    // Enable SPI & SPI FLASH
    spi_flash_init(SPI_FLASH_SIZE, SPI_FLASH_PAGE);
    spi_init(&spi_FLASH_CS_Pad, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW,
             SPI_PHA_MODE_0, SPI_MINT_DISABLE, freq);

    //The Flash is kept in power_down so we need to power it up
    spi_flash_release_from_power_down();
}

void app_spi_flash_peripheral_release(void)
{
#ifndef FLASH_SPI_DISABLE_SW_POWERDOWN
    spi_flash_power_down();
#endif
    deactivate_spi_flash_gpios();
}

size_t app_spi_flash_write_random_page_data(const void *data, uint32_t address, size_t size)
{
    uint32_t statusReadCount;
    
    uint8_t cpdata[SPI_FLASH_VIRTUAL_PAGE_SIZE];
    int32_t bytes_read;
    
// Do nothing if new data is not different from the data already stored in the NV ROM
    bytes_read=spi_flash_read_data (cpdata, address, size);
    if ( bytes_read < size) {
        return 0; //flash error
    }
    if (memcmp(cpdata, data, size)==0) {    
        return size; //data identical
    }    
    
#ifdef HAS_FLASH_SPI_POWER_DOWN
    if (power_up_delay) {
// The flash was off so wait until is ready for write operations        
        power_up_delay=false;
        port_delay_usec(8000);
    }
#endif
      
    int start_address = address & ~(SPI_FLASH_VIRTUAL_PAGE_SIZE-1);
    
    // Assumption that the address refers to a single VIRTUAL PAGE
    ASSERT_ERROR(start_address == ((address+size-1) & ~(SPI_FLASH_VIRTUAL_PAGE_SIZE-1)));
    //attempt to write size more than one page
    
    int stop_address = address  | (SPI_FLASH_VIRTUAL_PAGE_SIZE-1);  
    int size_to_write = (address+size < stop_address) ? size : stop_address-address+1;
    
//read the whole page
    if (spi_flash_read_data (cpdata, start_address, SPI_FLASH_VIRTUAL_PAGE_SIZE) < SPI_FLASH_VIRTUAL_PAGE_SIZE) {
        return 0;
    }
    memcpy (&cpdata[address-start_address], data, size_to_write);
    
    if (spi_flash_set_write_enable( )!= ERR_OK) {
        return 0;
    }

    // erase a whole sector. Only the first page of the sector is used
    if(spi_flash_block_erase_no_wait(start_address& ~(SPI_FLASH_SECTOR-1),SECTOR_ERASE)!= ERR_OK) {
        return 0;
    }

    for (statusReadCount = 0; statusReadCount < MAX_READY_WAIT_COUNT; statusReadCount++) {
        if ((spi_flash_read_status_reg() & STATUS_BUSY) == 0) {
            break;
        }
        if(flash_erase_callback != NULL) {
            flash_erase_callback();
        }
    }
    
	if(statusReadCount == MAX_READY_WAIT_COUNT) {
		return 0;
	}

    if (spi_flash_page_program(cpdata, start_address, SPI_FLASH_VIRTUAL_PAGE_SIZE) != ERR_OK) {
        return 0;
    }

    if (spi_flash_set_write_disable() != ERR_OK) {
        return 0;
    }

    return size_to_write;
}    

#if (DEBUG_WRITE_TO_SPI)

app_dbg_fault_t app_spi_flash_write_dbg_regs(app_dbg_fault_t fault)
{
    const size_t    flag_size = sizeof(app_dbg_fault_t),
                    uint32_size = sizeof(uint32_t);
    uint8_t page[APP_DBG_REGS_SIZE],
            *address = page;
    
    memcpy(address, &fault, flag_size);
    address += flag_size;
    
    for (int i=0; i < APP_DBG_TOTAL_REGS; i++) {
        memcpy(address, app_dbg_registers + i, uint32_size);
        address += uint32_size;
    }
    
    app_spi_flash_peripheral_init(SPI_XTAL_DIV_14);
    if (app_spi_flash_write_random_page_data(page, APP_FLASH_REGS_BASE, APP_DBG_REGS_SIZE)
        < APP_DBG_REGS_SIZE) {
        return APP_DBG_FAULT_FLASH;
    }
    app_spi_flash_peripheral_release();
    
    return APP_DBG_FAULT_NONE;
}

app_dbg_fault_t app_spi_flash_read_dbg_regs(void)
{
    const size_t flag_size = sizeof(app_dbg_fault_t);
    const size_t uint32_size = sizeof(uint32_t);

    uint8_t page[APP_DBG_REGS_SIZE], *address = page;

    app_spi_flash_peripheral_init(SPI_XTAL_DIV_14);
    if (spi_flash_read_data(page, APP_FLASH_REGS_BASE, APP_DBG_REGS_SIZE)
        < APP_DBG_REGS_SIZE) {
            return APP_DBG_FAULT_FLASH;
    }
    app_spi_flash_peripheral_release();

    app_dbg_fault_t fault;
    memcpy(&fault, address, flag_size);
    address += flag_size;    
    if (fault == APP_DBG_FAULT_NONE) {
        return APP_DBG_FAULT_NONE;
    }
    
    for (int i=0; i < APP_DBG_TOTAL_REGS; i++) {
        memcpy(app_dbg_registers + i, address, uint32_size);
        address += uint32_size;
    }
    return fault;
}

app_dbg_fault_t app_spi_flash_erase_dbg_regs(void)
{
    const size_t flag_size = sizeof(app_dbg_fault_t);
    const app_dbg_fault_t fault = APP_DBG_FAULT_NONE;
    
    app_spi_flash_peripheral_init(SPI_XTAL_DIV_14);
    if (app_spi_flash_write_random_page_data(&fault, APP_FLASH_REGS_BASE, flag_size)
        < flag_size) {
        return APP_DBG_FAULT_FLASH;
    }
    app_spi_flash_peripheral_release();
    
    return APP_DBG_FAULT_NONE;
}

app_dbg_fault_t app_spi_flash_write_dbg_msg(const char *msg)
{
    int msg_len;
    const size_t len_size = sizeof(int);

    app_spi_flash_peripheral_init(SPI_XTAL_DIV_14);
    if (msg == NULL)
        msg_len = -1;
    else {
        size_t msg_size = strlen(msg) + len_size;
        if (msg_size + APP_DBG_REGS_SIZE > SPI_FLASH_SECTOR) {
            msg_size = SPI_FLASH_SECTOR - APP_DBG_REGS_SIZE;
        }
        uint32_t address = APP_FLASH_BASE;
        msg_len = msg_size;

        while (msg_size > SPI_FLASH_PAGE) {
            if (app_spi_flash_write_random_page_data(msg, address, SPI_FLASH_PAGE)
                < SPI_FLASH_PAGE) {
                return APP_DBG_FAULT_FLASH;
            }
            address += SPI_FLASH_PAGE;
            msg += SPI_FLASH_PAGE;
            msg_size -= SPI_FLASH_PAGE;
        }
        if (app_spi_flash_write_random_page_data(msg, address, msg_size) < msg_size) {
            return APP_DBG_FAULT_FLASH;
        }
    }
    if (app_spi_flash_write_random_page_data(&msg_len, APP_FLASH_REGS_BASE-len_size,
                                             len_size) < len_size, NULL) {
        return APP_DBG_FAULT_FLASH;        
    }
    app_spi_flash_peripheral_release();
    
    return APP_DBG_FAULT_NONE;
}

app_dbg_fault_t app_spi_flash_read_dbg_msg(char *msg, size_t len)
{
    int msg_len;
    const size_t    len_size = sizeof(int),
                    char_size = sizeof(char);

    app_spi_flash_peripheral_init(SPI_XTAL_DIV_14);
    if (spi_flash_read_data((uint8_t*)&msg_len, APP_FLASH_REGS_BASE-len_size, len_size)
        < len_size) {
        return APP_DBG_FAULT_FLASH;
    }
    if (msg_len == -1) { // Flash contains 0xFF
        *msg = '\0';
    } else {
        if (msg_len + char_size > len) {
            msg_len = len - char_size;
        }
        if (spi_flash_read_data((uint8_t*)msg, APP_FLASH_BASE, msg_len) < msg_len) {
            return APP_DBG_FAULT_FLASH;
        }
        msg[msg_len] = '\0';
    }
    app_spi_flash_peripheral_release();
    
    return APP_DBG_FAULT_NONE;
}

app_dbg_fault_t app_spi_flash_erase_dbg_msg(void)
{
    return app_spi_flash_write_dbg_msg(NULL);
}

    #endif // (DEBUG_WRITE_TO_SPI)

#endif // HAS_SPI_FLASH_STORAGE

/**
 * \}
 * \}
 * \}
 */
