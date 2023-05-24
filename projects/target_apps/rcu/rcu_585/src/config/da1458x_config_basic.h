/**
****************************************************************************************
* \file da1458x_config_basic.h
* \brief Basic compile configuration file.
****************************************************************************************
*/
#ifndef _DA1458X_CONFIG_BASIC_H_
#define _DA1458X_CONFIG_BASIC_H_

#include "user_profiles_config.h"

/**
 ****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup APP_CONFIG
 * \{
 * \addtogroup SDK_BASIC_CFG
 * \brief SDK basic configuration
 * \{
 ****************************************************************************************
 */

/***************************************************************************************************************/
/* Integrated or external processor configuration                                                              */
/*    -defined      Integrated processor mode. Host application runs in DA14585 processor. Host application    */
/*                  is the TASK_APP kernel task.                                                               */
/*    -undefined    External processor mode. Host application runs on an external processor. Communicates with */
/*                  BLE application through GTL protocol over a signalling interface (UART, SPI etc)           */
/***************************************************************************************************************/
#define CFG_APP

/****************************************************************************************************************/
/* Enables the BLE security functionality in TASK_APP. If not defined BLE security related code is compiled out.*/
/****************************************************************************************************************/
#undef CFG_APP_SECURITY

/****************************************************************************************************************/
/* Enables WatchDog timer.                                                                                      */
/****************************************************************************************************************/
// #define CFG_WDOG // original
#undef CFG_WDOG

/****************************************************************************************************************/
/* Determines maximum concurrent connections supported by application. It configures the heap memory allocated  */
/* to service multiple connections. It is used for GAP central role applications. For GAP peripheral role it    */
/* should be set to 1 for optimising memory utilisation.                                                        */
/*      - MAX value for DA14585: 8                                                                              */
/****************************************************************************************************************/
#define CFG_MAX_CONNECTIONS     (1)

/****************************************************************************************************************/
/* Enables development/debug mode. For production mode builds it must be disabled.                              */
/* When enabled the following debugging features are enabled                                                    */
/*      -   Allows the emulation of the OTP mirroring to System RAM. No actual writing to RAM is done, but the  */
/*          exact same amount of time is spend as if the mirroring would take place. This is to mimic the       */
/*          behavior as if the System Code is already in OTP, and the mirroring takes place after waking up,    */
/*          but the (development) code still resides in an external source.                                     */
/*      -   Validation of GPIOs reservations.                                                                   */
/*      -   Enables Debug module and sets code execution in breakpoint in Hardfault and NMI (Watchdog) handlers.*/
/*          It allows developer to hot attach debugger and get debug information                                */
/****************************************************************************************************************/
// #undef CFG_DEVELOPMENT_DEBUG // original
#define CFG_DEVELOPMENT_DEBUG

/****************************************************************************************************************/
/* UART Console Print. Enables serial interface logging mechanism. If CFG_PRINTF is defined CFG_PRINTF_UART2    */
/* controls the uart module used. If it is defined UART2 is used. If not, UART is used. uart or uart2 driver    */
/* must be included in project respectively.                                                                    */
/****************************************************************************************************************/
#ifdef CFG_DEVELOPMENT_DEBUG
    #define CFG_PRINTF
    #ifdef CFG_PRINTF
        #define CFG_PRINTF_UART2
        #define UART2_BAUDRATE 500K
    #endif
#endif    

/**
 * \}
 * \}
 * \}
 */

#endif // _DA1458X_CONFIG_BASIC_H_
