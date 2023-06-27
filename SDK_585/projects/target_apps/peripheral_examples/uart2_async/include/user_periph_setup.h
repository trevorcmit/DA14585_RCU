/*****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
******************************************************************************************/

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
******************************************************************************************/

#include "gpio.h"
#include "uart.h"

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
