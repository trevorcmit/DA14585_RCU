/**
 ****************************************************************************************
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
 ****************************************************************************************
 */

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "arch.h"
#include "da1458x_periph_setup.h"
#include "dialog_prod.h"

/*
 * VARIABLE DEFINITIONS
 ****************************************************************************************
 */
 
extern uint8_t port_sel;

typedef struct __uart_sel_pins_t
{
    uint8_t uart_port_tx;
    uint8_t uart_tx_pin;
    uint8_t uart_port_rx;
    uint8_t uart_rx_pin;
}_uart_sel_pins_t;

extern _uart_sel_pins_t uart_sel_pins;
extern  uint8_t baud_rate_sel;
extern  uint8_t baud_rate_frac_sel;


/*
 * DEFINES
 ****************************************************************************
 */

/****************************************************************************/
/* UART pin configuration                                                   */
/****************************************************************************/

/****************************************************************************/
/* CONFIG_UART_GPIOS                                                        */
/*    -defined      Uart Port/Pins are defined by external tool             */
/*    -undefined    Uart Port/Pins are defined in the current project       */
/****************************************************************************/
#undef CONFIG_UART_GPIOS

/*****************************************************************************/
/* UART pin configuration                                                    */
/* Supported Port/Pin Combinations:                                          */
/* Tx: P04, Rx: P05                                                          */
/* Tx: P00, Rx: P01                                                          */
/* Tx: P14, Rx: P15                                                          */
/* Tx: P04, Rx: P13                                                          */
/*****************************************************************************/

#define UART1_TX_GPIO_PORT  GPIO_PORT_0
#define UART1_TX_GPIO_PIN   GPIO_PIN_4
#define UART1_RX_GPIO_PORT  GPIO_PORT_0
#define UART1_RX_GPIO_PIN   GPIO_PIN_5

enum
{
    P0_0_AND_P0_1_INITIALIZED_FROM_EXT_TOOL = 0x00,
    P0_2_AND_P0_3_INITIALIZED_FROM_EXT_TOOL = 0x02,
    P0_4_AND_P0_5_INITIALIZED_FROM_EXT_TOOL = 0x04,
    P0_6_AND_P0_7_INITIALIZED_FROM_EXT_TOOL = 0x06,
};

/***************************************************************************************/
/* Production debug output configuration                                               */
/***************************************************************************************/
#if PRODUCTION_DEBUG_OUTPUT
    #define PRODUCTION_DEBUG_PORT GPIO_PORT_2
    #define PRODUCTION_DEBUG_PIN GPIO_PIN_5
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
 * @return void
 ****************************************************************************************
 */
void periph_init(void);

/**
 ****************************************************************************************
 * @brief Map port pins. The UART and SPI port pins and GPIO ports are mapped.
 * @return void
 ****************************************************************************************
 */
void set_pad_functions(void);

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 * @return void
 ****************************************************************************************
 */
void GPIO_reservations(void);

/**
 ****************************************************************************************
 * @brief Map port pins. The UART pins are mapped.
 * @return void
 ****************************************************************************************
 */
void set_pad_uart(void);

/**
 ****************************************************************************************
 * @brief Update port pins. The UART pins are stored to retention memory.
 * @param[in] tx_port Port for UART TX
 * @param[in] tx_pin Pin for UART TX
 * @param[in] rx_port Port for UART RX
 * @param[in] rx_pin Pin for UART RX
 * @return void
 ****************************************************************************************
 */
void update_uart_pads(GPIO_PORT tx_port, GPIO_PIN tx_pin, GPIO_PORT rx_port, GPIO_PIN rx_pin);

/**
 ****************************************************************************************
 * @brief Initialize TXEN and RXEN.
 * @return void
 ****************************************************************************************
 */
void init_TXEN_RXEN_irqs(void);

#endif // _USER_PERIPH_SETUP_H_
