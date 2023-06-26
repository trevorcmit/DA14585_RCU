/*****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
 * Copyright (C) 2015 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
******************************************************************************************/

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
******************************************************************************************/

#include "uart.h"
#include "gpio.h"

/*
 * DEFINES
******************************************************************************************/

// Select UART settings

#define UART2_BAUDRATE      UART_BAUDRATE_115K2       // Baudrate in bits/s: {9K6, 14K4, 19K2, 28K8, 38K4, 57K6, 115K2}
#define UART2_FRAC_BAUDRATE UART_FRAC_BAUDRATE_115K2  // Baudrate fractional part: {9K6, 14K4, 19K2, 28K8, 38K4, 57K6, 115K2}
#define UART2_DATALENGTH    UART_CHARFORMAT_8         // Datalength in bits: {5, 6, 7, 8}
#define UART2_PARITY        UART_PARITY_NONE          // Parity: {UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD}
#define UART2_STOPBITS      UART_STOPBITS_1           // Stop bits: {UART_STOPBITS_1, UART_STOPBITS_2}
#define UART2_FLOWCONTROL   UART_FLOWCONTROL_DISABLED // Flow control: {UART_FLOWCONTROL_DISABLED, UART_FLOWCONTROL_ENABLED}

#define UART2_TX_GPIO_PORT  GPIO_PORT_0
#define UART2_TX_GPIO_PIN   GPIO_PIN_4
#define UART2_RX_GPIO_PORT  GPIO_PORT_0
#define UART2_RX_GPIO_PIN   GPIO_PIN_5

#define UART_ENABLED

// Select EEPROM characteristics
#define I2C_EEPROM_SIZE   0x20000         // EEPROM size in bytes
#define I2C_EEPROM_PAGE   256             // EEPROM page size in bytes
#define I2C_SLAVE_ADDRESS 0x50            // Set slave device address
#define I2C_SPEED_MODE    I2C_FAST        // Speed mode: I2C_STANDARD (100 kbits/s), I2C_FAST (400 kbits/s)
#define I2C_ADDRESS_MODE  I2C_7BIT_ADDR   // Addressing mode: {I2C_7BIT_ADDR, I2C_10BIT_ADDR}
#define I2C_ADDRESS_SIZE  I2C_2BYTES_ADDR // Address width: {I2C_1BYTE_ADDR, I2C_2BYTES_ADDR, I2C_3BYTES_ADDR}

// Select I2C settings
#define I2C_GPIO_PORT     GPIO_PORT_0
#define I2C_SCL_PIN       GPIO_PIN_2
#define I2C_SDA_PIN       GPIO_PIN_3

/****************************************************************************************************************/
/* I2C_SS_FREQ_TRIM: Enable trimming of SCL clock low and high period count for standard speed.                 */
/*    -defined      Setup I2C_HS_SCL_HCNT_REG and I2C_HS_SCL_LCNT_REG registers with user-defined values        */
/*    -undefined    Setup I2C_HS_SCL_HCNT_REG and I2C_HS_SCL_LCNT_REG registers with default values             */
/*    Default values have been evaluated for one slave on the I2C bus with 4.3k pull-up resistors on the SCL    */
/*    and SDA lines, respectively.                                                                              */
/****************************************************************************************************************/
#undef I2C_SS_FREQ_TRIM
#ifdef I2C_SS_FREQ_TRIM
    #define I2C_SS_SCL_HCNT_VAL     0x48
    #define I2C_SS_SCL_LCNT_VAL     0x4F
#endif // I2C_SS_FREQ_TRIM

/****************************************************************************************************************/
/* I2C_FS_FREQ_TRIM: Enable trimming of SCL clock low and high period count for fast speed.                     */
/*    -defined      Setup I2C_HS_SCL_HCNT_REG and I2C_HS_SCL_LCNT_REG registers with user-defined values        */
/*    -undefined    Setup I2C_HS_SCL_HCNT_REG and I2C_HS_SCL_LCNT_REG registers with default values             */
/*    Default values have been evaluated for one slave on the I2C bus with 4.3k pull-up resistors on the SCL    */
/*    and SDA lines, respectively.                                                                              */
/****************************************************************************************************************/
#undef I2C_FS_FREQ_TRIM
#ifdef I2C_FS_FREQ_TRIM
    #define I2C_FS_SCL_HCNT_VAL     0x08
    #define I2C_FS_SCL_LCNT_VAL     0x17
#endif // I2C_FS_FREQ_TRIM

/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
 * @return void
******************************************************************************************/
void periph_init(void);

#endif // _USER_PERIPH_SETUP_H_
