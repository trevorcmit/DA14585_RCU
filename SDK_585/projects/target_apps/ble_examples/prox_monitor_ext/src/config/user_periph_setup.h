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

/*
 * DEFINES
 ****************************************************************************************
 */

#undef PROGRAM_ALTERNATE_UART_PINS

//*** <<< Use Configuration Wizard in Context Menu >>> ***

// <o> DK selection <0=> As in da1458x_periph_setup.h <1=> Basic <2=> Pro <3=> Expert <4=> USB Dongle
#define HW_CONFIG (2)

#define HW_CONFIG_BASIC_DK      ((HW_CONFIG==0 && SDK_CONFIG==1) || HW_CONFIG==1)
#define HW_CONFIG_PRO_DK        ((HW_CONFIG==0 && SDK_CONFIG==2) || HW_CONFIG==2)
#define HW_CONFIG_EXPERT_DK     ((HW_CONFIG==0 && SDK_CONFIG==3) || HW_CONFIG==3)
#define HW_CONFIG_USB_DONGLE    (HW_CONFIG==4)

//*** <<< end of configuration section >>>    ***

/****************************************************************************************/
/* UART pin configuration                                                               */
/****************************************************************************************/

#if HW_CONFIG_BASIC_DK || HW_CONFIG_PRO_DK
    #define UART1_TX_GPIO_PORT  GPIO_PORT_0
    #define UART1_TX_GPIO_PIN   GPIO_PIN_4

    #define UART1_RX_GPIO_PORT  GPIO_PORT_0
    #define UART1_RX_GPIO_PIN   GPIO_PIN_5

    #define UART1_RTSN_GPIO_PORT GPIO_PORT_0
    #define UART1_RTSN_GPIO_PIN  GPIO_PIN_6

    #define UART1_CTSN_GPIO_PORT GPIO_PORT_0
    #define UART1_CTSN_GPIO_PIN  GPIO_PIN_7
#elif HW_CONFIG_EXPERT_DK
    #define UART1_TX_GPIO_PORT  GPIO_PORT_0
    #define UART1_TX_GPIO_PIN   GPIO_PIN_4

    #define UART1_RX_GPIO_PORT  GPIO_PORT_0
    #define UART1_RX_GPIO_PIN   GPIO_PIN_5

    #ifdef PROGRAM_ALTERNATE_UART_PINS
        #define UART1_RTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_RTSN_GPIO_PIN  GPIO_PIN_7

        #define UART1_CTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_CTSN_GPIO_PIN  GPIO_PIN_6
    #else
        #define UART1_RTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_RTSN_GPIO_PIN  GPIO_PIN_3

        #define UART1_CTSN_GPIO_PORT GPIO_PORT_0
        #define UART1_CTSN_GPIO_PIN  GPIO_PIN_2
    #endif // PROGRAM_ALTERNATE_UART_PINS
#else // HW_CONFIG_USB_DONGLE
    #define UART1_TX_GPIO_PORT  GPIO_PORT_0
    #define UART1_TX_GPIO_PIN   GPIO_PIN_4

    #define UART1_RX_GPIO_PORT  GPIO_PORT_0
    #define UART1_RX_GPIO_PIN   GPIO_PIN_5
#endif

/****************************************************************************************/
/* UART2 pin configuration (debug print console)                                        */
/****************************************************************************************/

#ifdef CFG_PRINTF_UART2
    #if HW_CONFIG_BASIC_DK
        #define UART2_TX_GPIO_PORT  GPIO_PORT_2
        #define UART2_TX_GPIO_PIN   GPIO_PIN_6

        #define UART2_RX_GPIO_PORT  GPIO_PORT_2
        #define UART2_RX_GPIO_PIN   GPIO_PIN_7

    #elif HW_CONFIG_PRO_DK
        #define UART2_TX_GPIO_PORT  GPIO_PORT_2
        #define UART2_TX_GPIO_PIN   GPIO_PIN_6

        #define UART2_RX_GPIO_PORT  GPIO_PORT_2
        #define UART2_RX_GPIO_PIN   GPIO_PIN_7

    #elif HW_CONFIG_EXPERT_DK
        #define UART2_TX_GPIO_PORT  GPIO_PORT_2
        #define UART2_TX_GPIO_PIN   GPIO_PIN_6

        #define UART2_RX_GPIO_PORT  GPIO_PORT_2
        #define UART2_RX_GPIO_PIN   GPIO_PIN_7

    #else // (e.g. HW_CONFIG_USB_DONGLE)
        #define UART2_TX_GPIO_PORT  GPIO_PORT_2
        #define UART2_TX_GPIO_PIN   GPIO_PIN_6

        #define UART2_RX_GPIO_PORT  GPIO_PORT_2
        #define UART2_RX_GPIO_PIN   GPIO_PIN_7

    #endif
#endif // CFG_PRINTF_UART2

/****************************************************************************************/
/* External CPU to DA1458x wake-up pin selection                                        */
/****************************************************************************************/

 #ifdef CFG_EXTERNAL_WAKEUP
    /*Auto select the external GPIO wakeup signal according to the HCI_EIF_SELECT_PORT/HCI_EIF_SELECT_PIN configuration*/
    #define EIF_WAKEUP_GPIO                                     (1)
    #ifdef EIF_WAKEUP_GPIO
        #ifdef CFG_HCI_BOTH_EIF
            #define EXTERNAL_WAKEUP_GPIO_PORT           (GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)?UART1_CTSN_GPIO_PORT:SPI_CLK_PORT
            #define EXTERNAL_WAKEUP_GPIO_PIN            (GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)?UART1_CTSN_GPIO_PIN:SPI_CS_PIN
            #define EXTERNAL_WAKEUP_GPIO_POLARITY       (GPIO_GetPinStatus(HCI_EIF_SELECT_PORT, HCI_EIF_SELECT_PIN) == 1)?1:0
        #else
            #if defined(CFG_HCI_SPI) || defined(CFG_GTL_SPI)
                #define EXTERNAL_WAKEUP_GPIO_PORT       SPI_GPIO_PORT
                #define EXTERNAL_WAKEUP_GPIO_PIN        SPI_CS_PIN
                #define EXTERNAL_WAKEUP_GPIO_POLARITY   (0)
            #else // UART
                #define EXTERNAL_WAKEUP_GPIO_PORT       UART1_CTSN_GPIO_PORT
                #define EXTERNAL_WAKEUP_GPIO_PIN        UART1_CTSN_GPIO_PIN
                #define EXTERNAL_WAKEUP_GPIO_POLARITY   (1)
            #endif
        #endif
    #else
        #define EXTERNAL_WAKEUP_GPIO_PORT               GPIO_PORT_1
        #define EXTERNAL_WAKEUP_GPIO_PIN                GPIO_PORT_7
        #define EXTERNAL_WAKEUP_GPIO_POLARITY           (1)
    #endif
#endif

/****************************************************************************************/
/* DA1458x to external CPU wake-up pin selection                                        */
/****************************************************************************************/

#ifdef CFG_WAKEUP_EXT_PROCESSOR
    #define EXT_WAKEUP_PORT GPIO_PORT_1
    #define EXT_WAKEUP_PIN  GPIO_PIN_2
#endif // #ifdef CFG_WAKEUP_EXT_PROCESSOR


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

#endif // _USER_PERIPH_SETUP_H_
