/*****************************************************************************************
 *
 * @file da1458x_config_advanced.h
 *
 * @brief Advanced compile configuration file.
  *
******************************************************************************************/

#ifndef _DA1458X_CONFIG_ADVANCED_H_
#define _DA1458X_CONFIG_ADVANCED_H_

#include "da1458x_stack_config.h"

/****************************************************************************************************************/
/* Low Power clock selection.                                                                                   */
/*      -LP_CLK_XTAL32      External XTAL32 oscillator                                                          */
/*      -LP_CLK_RCX20       Internal RCX20 clock                                                                */
/*      -LP_CLK_FROM_OTP    Use the selection in the corresponding field of OTP Header                          */
/****************************************************************************************************************/
#define CFG_LP_CLK              LP_CLK_RCX20

/****************************************************************************************************************/
/* If defined the application uses a hardcoded value for XTAL16M trimming. Should be disabled for devices       */
/* where XTAL16M is calibrated and trim value is stored in OTP.                                                 */
/* Important note. The hardcoded value is the average value of the trimming values giving the optimal results   */
/* for DA14585 DK devices. May not be applicable in other designs                                               */
/****************************************************************************************************************/
#undef CFG_USE_DEFAULT_XTAL16M_TRIM_VALUE_IF_NOT_CALIBRATED

/****************************************************************************************************************/
/* Periodic wakeup period to poll GTL iface. Time in msec.                                                      */
/****************************************************************************************************************/
#define CFG_MAX_SLEEP_DURATION_PERIODIC_WAKEUP_MS                  500  // 0.5s

/****************************************************************************************************************/
/* Periodic wakeup period if GTL iface is not enabled. Time in msec.                                            */
/****************************************************************************************************************/
#define CFG_MAX_SLEEP_DURATION_EXTERNAL_WAKEUP_MS                  10000  // 10s

/****************************************************************************************************************/
/* Wakeup from external processor running host application.                                                     */
/****************************************************************************************************************/
#undef CFG_EXTERNAL_WAKEUP

/****************************************************************************************************************/
/* Wakeup external processor when a message is sent to GTL                                                      */
/****************************************************************************************************************/
#undef CFG_WAKEUP_EXT_PROCESSOR

/****************************************************************************************************************/
/* Enables True Random number Generator. A random number is generated at system initialization and used to seed */
/* the C standard library random number generator.                                                              */
/****************************************************************************************************************/
#undef CFG_TRNG

/****************************************************************************************************************/
/* Custom heap sizes                                                                                            */
/****************************************************************************************************************/
// #define DB_HEAP_SZ              1024
// #define ENV_HEAP_SZ             4928
// #define MSG_HEAP_SZ             6880
// #define NON_RET_HEAP_SZ         2048

/****************************************************************************************************************/
/* NVDS configuration                                                                                           */
/* - CFG_NVDS_TAG_BD_ADDRESS            Default bdaddress. If bdaddress is written in OTP header this value is  */
/*                                      ignored                                                                 */
/* - CFG_NVDS_TAG_LPCLK_DRIFT           Low power clock drift. Permitted values in ppm are:                     */
/*      + DRIFT_20PPM                                                                                           */
/*      + DRIFT_30PPM                                                                                           */
/*      + DRIFT_50PPM                                                                                           */
/*      + DRIFT_75PPM                                                                                           */
/*      + DRIFT_100PPM                                                                                          */
/*      + DRIFT_150PPM                                                                                          */
/*      + DRIFT_250PPM                                                                                          */
/*      + DRIFT_500PPM                  Default value (500 ppm)                                                 */
/* - CFG_NVDS_TAG_BLE_CA_TIMER_DUR      Channel Assessment Timer duration (Multiple of 10ms)                    */
/* - CFG_NVDS_TAG_BLE_CRA_TIMER_DUR     Channel Reassessment Timer duration (Multiple of CA timer duration)     */
/* - CFG_NVDS_TAG_BLE_CA_MIN_RSSI       Minimal RSSI Threshold                                                  */
/* - CFG_NVDS_TAG_BLE_CA_NB_PKT         Number of packets to receive for statistics                             */
/* - CFG_NVDS_TAG_BLE_CA_NB_BAD_PKT     Number  of bad packets needed to remove a channel                       */
/****************************************************************************************************************/
#define CFG_NVDS_TAG_BD_ADDRESS             {0x22, 0x22, 0x22, 0xCA, 0xEA, 0x80}

#define CFG_NVDS_TAG_LPCLK_DRIFT            DRIFT_500PPM
#define CFG_NVDS_TAG_BLE_CA_TIMER_DUR       2000
#define CFG_NVDS_TAG_BLE_CRA_TIMER_DUR      6
#define CFG_NVDS_TAG_BLE_CA_MIN_RSSI        0x40
#define CFG_NVDS_TAG_BLE_CA_NB_PKT          100
#define CFG_NVDS_TAG_BLE_CA_NB_BAD_PKT      50

/****************************************************************************************************************/
/* Enables the BLE statistics measurement feature.                                                              */
/****************************************************************************************************************/
#define CFG_BLE_METRICS

/****************************************************************************************************************/
/* Output the Hardfault arguments to serial/UART interface.                                                     */
/****************************************************************************************************************/
#undef CFG_PRODUCTION_DEBUG_OUTPUT

/****************************************************************************************************************/
/* To add comments                                                                                              */
/****************************************************************************************************************/
#define CFG_PRODUCTION_TEST

/****************************************************************************************************************/
/* Maximum supported TX data packet length (supportedMaxTxOctets value, as defined in 4.2 Specification).       */
/* Range: 27 - 251 octets.                                                                                      */
/* NOTE 1: Even number of octets are not supported. A selected even number will be automatically converted to   */
/*         the next odd one.                                                                                    */
/* NOTE 2: The supportedMaxTxTime value is automatically calculated by the ROM code, according to the following */
/*         equation:                                                                                            */
/*             supportedMaxTxTime = (supportedMaxTxOctets + 11 + 3 ) * 8                                        */
/*         Range: 328 - 2120 usec.                                                                              */
/****************************************************************************************************************/
#define CFG_MAX_TX_PACKET_LENGTH        (251)

/****************************************************************************************************************/
/* Maximum supported RX data packet length (supportedMaxRxOctets value, as defined in 4.2 Specification).       */
/* Range: 27 - 251 octets.                                                                                      */
/* NOTE 1: Even number of octets are not supported. A selected even number will be automatically converted to   */
/*         the next odd one.                                                                                    */
/* NOTE 2: The supportedMaxRxTime value is automatically calculated by the ROM code, according to the following */
/*         equation:                                                                                            */
/*             supportedMaxRxTime = (supportedMaxRxOctets + 11 + 3 ) * 8                                        */
/*         Range: 328 - 2120 usec.                                                                              */
/****************************************************************************************************************/
#define CFG_MAX_RX_PACKET_LENGTH        (251)

/****************************************************************************************************************/
/* Select external application/host transport layer:                                                            */
/*     - 0 = GTL (auto)                                                                                         */
/*     - 1 = HCI (auto)                                                                                         */
/*     - 8 = GTL (fixed)                                                                                        */
/*     - 9 = HCI (fixed)                                                                                        */
/****************************************************************************************************************/
#define CFG_USE_H4TL                    (1)

/****************************************************************************************************************/
/* Duplicate filter max value for the scan report list. The maximum value shall be 100.                         */
/****************************************************************************************************************/
#define CFG_BLE_DUPLICATE_FILTER_MAX    (10)

/****************************************************************************************************************/
/* Duplicate filter flag for the scan report list. This flag controls what will be reported if the              */
/* CFG_BLE_DUPLICATE_FILTER_MAX number is exceeded.                                                             */
/*     - If the flag is defined, the extra devices are considered to be in the list and will not be reported.   */
/****************************************************************************************************************/
#undef CFG_BLE_DUPLICATE_FILTER_FOUND

/****************************************************************************************************************/
/* Resolving list maximum size.                                                                                 */
/****************************************************************************************************************/
#define CFG_LLM_RESOLVING_LIST_MAX      LLM_RESOLVING_LIST_MAX

/****************************************************************************************************************/
/* Enables automatic data packet length negotiation.                                                            */
/****************************************************************************************************************/
#undef AUTO_DATA_LENGTH_NEGOTIATION_UPON_NEW_CONNECTION

/****************************************************************************************************************/
/* Maximum retention memory in bytes. The base address of the retention data is calculated from the selected    */
/* size.                                                                                                        */
/****************************************************************************************************************/
#define CFG_RET_DATA_SIZE    (2048)

/****************************************************************************************************************/
/* Maximum uninitialized retained data required by the application.                                             */
/****************************************************************************************************************/
#define CFG_RET_DATA_UNINIT_SIZE (0)

/****************************************************************************************************************/
/* The Keil scatter file may be provided by the user. If the user provides his own scatter file, the system has */
/* to be aware which RAM blocks has to retain. The 4th RAM block is always retained, since it contains the ROM  */
/* data.                                                                                                        */
/*     - CFG_RETAIN_RAM_1_BLOCK: if defined, the 1st RAM block must be retained.                                */
/*     - CFG_RETAIN_RAM_2_BLOCK: if defined, the 2nd RAM block must be retained.                                */
/*     - CFG_RETAIN_RAM_3_BLOCK: if defined, the 3rd RAM block must be retained.                                */
/*                                                                                                              */
/* If the CFG_CUSTOM_SCATTER_FILE flag is undefined, the system knows which blocks to retain based on the       */
/* default SDK scatter file.                                                                                    */
/****************************************************************************************************************/
#undef CFG_CUSTOM_SCATTER_FILE
#ifdef CFG_CUSTOM_SCATTER_FILE
    #define CFG_RETAIN_RAM_1_BLOCK
    #define CFG_RETAIN_RAM_2_BLOCK
    #define CFG_RETAIN_RAM_3_BLOCK
#endif

/****************************************************************************************************************/
/* Code location selection.                                                                                     */
/*     - CFG_CODE_LOCATION_EXT: Code is loaded from SPI flash / I2C EEPROM / UART                               */
/*     - CFG_CODE_LOCATION_OTP: Code is burned in the OTP                                                       */
/* The above options are mutually exclusive and exactly one of them must be enabled.                            */
/****************************************************************************************************************/
#define CFG_CODE_LOCATION_EXT
#undef CFG_CODE_LOCATION_OTP

/****************************************************************************************************************/
/* Production test configures GPIOs dynamically through host application commands.                              */
/* Therefore, the pre GPIO allocation is disabled.                                                              */
/****************************************************************************************************************/
#define GPIO_DRV_PIN_ALLOC_MON_DISABLED

#endif // _DA1458X_CONFIG_ADVANCED_H_
