/*****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief Peripherals setup and initialization.
 *
 * Copyright (C) 2012 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
******************************************************************************************/
/*
 * INCLUDE FILES
******************************************************************************************/
#include "rwip_config.h"             // SW configuration
#include "user_periph_setup.h"            // peripheral configuration
#include "gpio.h"
#include "uart.h"                    // UART initialization

/*****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 * @return void
******************************************************************************************/

#if DEVELOPMENT_DEBUG

void GPIO_reservations(void)
{
/*
* Globally reserved GPIOs reservation
*/

/*
* Application specific GPIOs reservation. Used only in Development mode (#if DEVELOPMENT_DEBUG)

i.e.
    RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);    //Reserve P_01 as Generic Purpose I/O
*/

    RESERVE_GPIO(UART1_TX,        UART1_TX_GPIO_PORT,   UART1_TX_GPIO_PIN,   PID_UART1_TX);
    RESERVE_GPIO(UART1_RX,        UART1_RX_GPIO_PORT,   UART1_RX_GPIO_PIN,   PID_UART1_RX);
    #if !HW_CONFIG_USB_DONGLE
    RESERVE_GPIO(UART1_RTS,       UART1_RTSN_GPIO_PORT, UART1_RTSN_GPIO_PIN, PID_UART1_RTSN);
    RESERVE_GPIO(UART1_CTS,       UART1_CTSN_GPIO_PORT, UART1_CTSN_GPIO_PIN, PID_UART1_CTSN);
    #endif
    #ifdef CFG_PRINTF_UART2
    RESERVE_GPIO(UART2_TX,        UART2_TX_GPIO_PORT,   UART2_TX_GPIO_PIN,   PID_UART2_TX);
    RESERVE_GPIO(UART2_RX,        UART2_RX_GPIO_PORT,   UART2_RX_GPIO_PIN,   PID_UART2_RX);
    #endif
    #ifdef CFG_WAKEUP_EXT_PROCESSOR
    // external MCU wakeup GPIO
    RESERVE_GPIO(EXT_WAKEUP_GPIO, EXT_WAKEUP_PORT,      EXT_WAKEUP_PIN,      PID_GPIO);
    #endif

}
#endif //DEVELOPMENT_DEBUG

/*****************************************************************************************
 * @brief Map port pins. The UART and SPI port pins and GPIO ports are mapped.
******************************************************************************************/
void set_pad_functions(void)        // set gpio port function mode
{
/*
* Configure application ports.
i.e.
    GPIO_ConfigurePin( GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false ); // Set P_01 as Generic purpose Output
*/

#ifdef __DA14586__
    // disallow spontaneous flash wake-up
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_3, OUTPUT, PID_GPIO, true);
#endif

    GPIO_ConfigurePin(UART1_TX_GPIO_PORT,   UART1_TX_GPIO_PIN,   OUTPUT, PID_UART1_TX,   false);
    GPIO_ConfigurePin(UART1_RX_GPIO_PORT,   UART1_RX_GPIO_PIN,   INPUT,  PID_UART1_RX,   false);
#if !HW_CONFIG_USB_DONGLE
    GPIO_ConfigurePin(UART1_RTSN_GPIO_PORT, UART1_RTSN_GPIO_PIN, OUTPUT, PID_UART1_RTSN, false);
    GPIO_ConfigurePin(UART1_CTSN_GPIO_PORT, UART1_CTSN_GPIO_PIN, INPUT,  PID_UART1_CTSN, false);
#endif

#ifdef CFG_PRINTF_UART2
    GPIO_ConfigurePin(UART2_TX_GPIO_PORT,   UART2_TX_GPIO_PIN,   OUTPUT, PID_UART2_TX,   false);
    GPIO_ConfigurePin(UART2_RX_GPIO_PORT,   UART2_RX_GPIO_PIN,   INPUT,  PID_UART2_RX,   false);
#endif

#ifdef CFG_WAKEUP_EXT_PROCESSOR
    // external MCU wakeup GPIO
    GPIO_ConfigurePin(EXT_WAKEUP_PORT,      EXT_WAKEUP_PIN,      OUTPUT, PID_GPIO,       false); // initialized to low
#endif
}


/*****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
 * @return void
******************************************************************************************/
void periph_init(void)
{
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP)) ;

    SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 1);

    //rom patch
    patch_func();

    //Init pads
    set_pad_functions();

    // (Re)Initialize peripherals
    // i.e.
    // uart_init(UART_BAUDRATE_115K2, UART_FRAC_BAUDRATE_115K2, UART_CHARFORMAT_8);

    // Initialize UART component
    // mode=3-> no parity, 1 stop bit 8 data length
    uart_init(UART_BAUDRATE_115K2, UART_FRAC_BAUDRATE_115K2, UART_CHARFORMAT_8);

#ifdef CFG_PRINTF_UART2
    SetBits16(CLK_PER_REG, UART2_ENABLE, 1);
    uart2_init(UART_BAUDRATE_115K2, UART_FRAC_BAUDRATE_115K2, UART_CHARFORMAT_8);
#endif

   // Enable the pads
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
