/*****************************************************************************************
 *
 * @file periph_setup.c
 *
 * @brief Peripherals setup and initialization.
 *
******************************************************************************************/


/*
 * INCLUDE FILES
******************************************************************************************/

#include "rwip_config.h"        // SW configuration
#include "user_periph_setup.h"  // periphera configuration
#include "gpio.h"
#include "uart.h"               // UART initialization

/*
 * FUNCTION DEFINITIONS
******************************************************************************************/

#if DEVELOPMENT_DEBUG
/*****************************************************************************************
 * @brief GPIO_reservations. Globally reserved GPIOs
 * @return void
 ****************************************************************************************
*/
void GPIO_reservations(void)
{
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
    RESERVE_GPIO(EXT_WAKEUP_GPIO, EXT_WAKEUP_PORT,      EXT_WAKEUP_PIN,      PID_GPIO );
    #endif
}
#endif

/*****************************************************************************************
 * @brief Map port pins. The UART and SPI port pins and GPIO ports
 *        (for debugging) are mapped.
******************************************************************************************/
void set_pad_functions(void)        // set gpio port function mode
{
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
 *        is down.
******************************************************************************************/
void periph_init(void)
{
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));

    SetBits16(CLK_16M_REG,XTAL16_BIAS_SH_ENABLE, 1);


    // Initialize UART controller
    uart_init(UART_BAUDRATE_115K2, UART_FRAC_BAUDRATE_115K2, UART_CHARFORMAT_8);

    // Initialize UART2 controller for debug print console
#ifdef CFG_PRINTF_UART2
    SetBits16(CLK_PER_REG, UART2_ENABLE, 1);
    uart2_init(UART_BAUDRATE_115K2, UART_FRAC_BAUDRATE_115K2, UART_CHARFORMAT_8);
#endif

    // ROM patch
    patch_func();

    // Init pads
    set_pad_functions();

    // Enable the pads
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
