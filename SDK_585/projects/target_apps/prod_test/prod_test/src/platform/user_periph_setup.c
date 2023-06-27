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
#include "user_periph_setup.h"  // peripheral configuration
#include "gpio.h"
#include "uart.h"               // UART initialization
 
/****************************************************************************************/
/* UART pin configuration                                                               */
/****************************************************************************************/
uint8_t port_sel                           __attribute__((section("prodtest_uninit"), zero_init));
bool device_wake_up                        __attribute__((section("retention_mem_area0"), zero_init));
_uart_sel_pins_t uart_sel_pins             __attribute__((section("retention_mem_area0"), zero_init));
uint8_t baud_rate_sel                      __attribute__((section("retention_mem_area0"), zero_init));
uint8_t baud_rate_frac_sel                 __attribute__((section("retention_mem_area0"), zero_init));

_uart_sel_pins_t uart_sel_pins;

#if DEVELOPMENT_DEBUG
/*****************************************************************************************
 * @brief Reserve the GPIOs used by the application.
 * @return void 
 ****************************************************************************************
*/
void GPIO_reservations(void)
{
#ifdef CONFIG_UART_GPIOS
    switch (port_sel)
    {
    case P0_0_AND_P0_1_INITIALIZED_FROM_EXT_TOOL:
        RESERVE_GPIO(UART1_TX, GPIO_PORT_0, GPIO_PIN_0, PID_UART1_TX);
        RESERVE_GPIO(UART1_RX, GPIO_PORT_0, GPIO_PIN_1, PID_UART1_RX);
        break;
    case P0_2_AND_P0_3_INITIALIZED_FROM_EXT_TOOL:
        RESERVE_GPIO(UART1_TX, GPIO_PORT_0, GPIO_PIN_2, PID_UART1_TX);
        RESERVE_GPIO(UART1_RX, GPIO_PORT_0, GPIO_PIN_3, PID_UART1_RX);
        break;
    case P0_4_AND_P0_5_INITIALIZED_FROM_EXT_TOOL:
        RESERVE_GPIO(UART1_TX, GPIO_PORT_0, GPIO_PIN_4, PID_UART1_TX);
        RESERVE_GPIO(UART1_RX, GPIO_PORT_0, GPIO_PIN_5, PID_UART1_RX);
        break;
    case P0_6_AND_P0_7_INITIALIZED_FROM_EXT_TOOL:
        RESERVE_GPIO(UART1_TX, GPIO_PORT_0, GPIO_PIN_6, PID_UART1_TX);
        RESERVE_GPIO(UART1_RX, GPIO_PORT_0, GPIO_PIN_7, PID_UART1_RX);
        break;
    default:
        RESERVE_GPIO(UART1_TX, GPIO_PORT_0, GPIO_PIN_4, PID_UART1_TX);
        RESERVE_GPIO(UART1_RX, GPIO_PORT_0, GPIO_PIN_5, PID_UART1_RX);
        break;
    }
#elif (defined(UART1_TX_GPIO_PORT) && defined(UART1_TX_GPIO_PIN) && defined(UART1_RX_GPIO_PORT) && defined(UART1_RX_GPIO_PIN))
    RESERVE_GPIO( UART1_TX, UART1_TX_GPIO_PORT,  UART1_TX_GPIO_PIN, PID_UART1_TX);
    RESERVE_GPIO( UART1_RX, UART1_RX_GPIO_PORT,  UART1_RX_GPIO_PIN, PID_UART1_RX);
#else
    #error "=== No UART pin configuration selected in periph_setup.h ==="
#endif // CONFIG_UART_GPIOS

#if HAS_AUDIO
    declare_audio439_gpios();
#endif
}

#endif //DEVELOPMENT_DEBUG

/*****************************************************************************************
 * @brief Set function mode for GPIO pins.
 *        If DEVELOPMENT_DEBUG is defined then each pin must have been
 *        previously reserved in GPIO_reservations().
******************************************************************************************/
void set_pad_functions(void)
{
#ifdef __DA14586__
    // disallow spontaneous flash wake-up
    GPIO_ConfigurePin(GPIO_PORT_2, GPIO_PIN_3, OUTPUT, PID_GPIO, true);
#endif

#if HAS_AUDIO
    init_audio439_gpios(app_audio439_timer_started);
#endif

if (!device_wake_up)
{
    #if defined(CONFIG_UART_GPIOS) // Ports for UART are initialized from external tool
        switch (port_sel)
        {
            case P0_0_AND_P0_1_INITIALIZED_FROM_EXT_TOOL: // Ports for UART P0_0 & P0_1 are initialized from external tool
                update_uart_pads(GPIO_PORT_0, GPIO_PIN_0, GPIO_PORT_0, GPIO_PIN_1);
                break;
            case P0_2_AND_P0_3_INITIALIZED_FROM_EXT_TOOL: // Ports for UART P0_2 & P0_3 are initialized from external tool
                update_uart_pads(GPIO_PORT_0, GPIO_PIN_2, GPIO_PORT_0,GPIO_PIN_3);
                break;
            case P0_4_AND_P0_5_INITIALIZED_FROM_EXT_TOOL: // Ports for UART P0_4 & P0_5 are initialized from external tool
                update_uart_pads(GPIO_PORT_0, GPIO_PIN_4, GPIO_PORT_0, GPIO_PIN_5);
                break;
            case P0_6_AND_P0_7_INITIALIZED_FROM_EXT_TOOL: // Ports for UART P0_6 & P0_7 are initialized from external tool
                update_uart_pads(GPIO_PORT_0, GPIO_PIN_6, GPIO_PORT_0, GPIO_PIN_7);
                break;
            default:
                break;
        }
    #elif (defined(UART1_TX_GPIO_PORT) && defined(UART1_TX_GPIO_PIN) && defined(UART1_RX_GPIO_PORT) && defined(UART1_RX_GPIO_PIN))
        update_uart_pads(UART1_TX_GPIO_PORT, UART1_TX_GPIO_PIN, UART1_RX_GPIO_PORT, UART1_RX_GPIO_PIN);
        
    #else
        #error "=== No UART pin configuration selected in periph_setup.h ==="
    #endif
    device_wake_up = true;
}

set_pad_uart();
}

void set_pad_uart(void)
{
    GPIO_ConfigurePin((GPIO_PORT) uart_sel_pins.uart_port_tx, (GPIO_PIN) uart_sel_pins.uart_tx_pin, OUTPUT, PID_UART1_TX, false);
    GPIO_ConfigurePin((GPIO_PORT) uart_sel_pins.uart_port_rx, (GPIO_PIN) uart_sel_pins.uart_rx_pin, INPUT, PID_UART1_RX, false);
}

void update_uart_pads(GPIO_PORT tx_port, GPIO_PIN tx_pin, GPIO_PORT rx_port, GPIO_PIN rx_pin)
{
    if ((tx_port == GPIO_PORT_1) && (tx_pin == GPIO_PIN_4) && (rx_port == GPIO_PORT_1) && (rx_pin == GPIO_PIN_5))
    {
        // Wait until debugger is disconnected and then disable debuging
        while ((GetWord16(SYS_STAT_REG) & DBG_IS_UP) == DBG_IS_UP) {};
        SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 0);    // Close debugger
    }
    uart_sel_pins.uart_port_tx = tx_port;
    uart_sel_pins.uart_tx_pin = tx_pin;
    uart_sel_pins.uart_port_rx = rx_port;
    uart_sel_pins.uart_rx_pin = rx_pin;
}

void init_TXEN_RXEN_irqs(void)
{
    //init for TXEN
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_WSEL_0, 2); //SO SELECT RADIO_DIAG0
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_MASK_0, 1); //ENABLE IRQ
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_BSEL_0, 7); //BIT7 OF DIAG0 BUS, SO TXEN
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_EDGE_0, 0); //SELECT POS EDGE
    
    //init for RXEN
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_WSEL_1, 3); //SO SELECT RADIO_DIAG1
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_MASK_1, 1); //ENABLE IRQ
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_BSEL_1, 7); //BIT7 OF DIAG1 BUS, SO RXEN
    SetBits16(RF_DIAGIRQ01_REG, DIAGIRQ_EDGE_1, 0); //SELECT POS EDGE
    
    NVIC_EnableIRQ(BLE_RF_DIAG_IRQn); 
    NVIC_SetPriority(BLE_RF_DIAG_IRQn,4);     
    NVIC_ClearPendingIRQ(BLE_RF_DIAG_IRQn); //clear eventual pending bit, but not necessary becasuse this is already cleared automatically in HW
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
    if (baud_rate_sel == 0)
    {
        baud_rate_sel = UART_BAUDRATE_115K2;
        baud_rate_frac_sel = UART_FRAC_BAUDRATE_115K2;
    }
    uart_init(baud_rate_sel, baud_rate_frac_sel, UART_CHARFORMAT_8);


    // ROM patch
    patch_func();

    // Init pads
    set_pad_functions();

    // Enable the pads
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);
}
