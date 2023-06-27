/*****************************************************************************************
 *
 * @file user_config.h
 *
 * @brief User configuration file.
 *
******************************************************************************************/

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

/*
 * INCLUDE FILES
******************************************************************************************/

#include "arch_api.h"

/*
 * VARIABLES
******************************************************************************************/

/******************************************
 * Default sleep mode. Possible values are:
 *
 * - ARCH_SLEEP_OFF
 * - ARCH_EXT_SLEEP_ON
 * - ARCH_EXT_SLEEP_OTP_COPY_ON
 *
 ******************************************
 */
const static sleep_state_t app_default_sleep_mode = ARCH_EXT_SLEEP_ON;


#endif // _USER_CONFIG_H_
