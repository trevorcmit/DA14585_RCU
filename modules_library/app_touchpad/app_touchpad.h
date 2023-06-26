/*****************************************************************************************
 * \file app_touchpad.h
 * \brief This module provides an API for getting tracking and gesture events from a touchpad controller.
*****************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup TOUCHPAD
 * \{
 * \addtogroup APP_TOUCHPAD
 * \{
****************************************************************************************	 */ 

#ifndef APP_TOUCHPAD_H
#define APP_TOUCHPAD_H

#include "app_touchpad_defs.h"
#include <stdint.h>

#ifndef HAS_I2C
    #define HAS_I2C
#endif


/*****************************************************************************************
 * \brief Used to poll the trackpad driver for new events and data
 * \return  APP_KEEP_POWERED device sleep must be blocked, APP_BLE_WAKEUP if BLE must
 *          wake-up, otherwise APP_GOTO_SLEEP
*****************************************************************************************/
uint8_t app_touchpad_poll(void);


/*****************************************************************************************
 * \brief Used to get the last tracking info as it was acquired from the poll function
 * \param[out] pInfo A pointer where the acquired data will be placed
 * \return bool True if there was unread tracking info, false otherwise
*****************************************************************************************/
bool app_touchpad_get_last_track_info(app_touchpad_track_data_t * pInfo);


/*****************************************************************************************
 * \brief Touchpad Platform init function
*****************************************************************************************/
void app_touchpad_init(void);


/*****************************************************************************************
 * \brief Touchpad Platform de-init function
*****************************************************************************************/
void app_touchpad_deinit(void);


#endif // APP_TOUCHPAD_H

/**
 * \}
 * \}
 * \}
 */
