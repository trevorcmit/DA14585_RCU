/**
 ****************************************************************************************
 *
 * \file app_motion.h
 *
 * \brief This module provides an API for acquiring motion data from accelerometer and
 * gyro sensors.
 *
 * Define symbol HAS_MOTION to include this module in the application.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */
 
/**
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup MOTION
 * \{
 * \addtogroup APP_MOTION
 * \{
 ****************************************************************************************
 */
 
#ifndef APP_MOTION_H_
#define APP_MOTION_H_

#include "app_motion_defs.h"

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

typedef  struct app_motion_data_s
{  
    int32_t timestamp;
    int16_t temperature;
    int16_t acc_motion[3];
    int16_t rot_motion[3];
    int16_t click_events;
} app_motion_data_t;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * \brief Reads motion data to a structure
 *
 * \param[out] data  Pointer to the structure to be filled
 *
 * \return true if sensor is active
 ****************************************************************************************
 */
bool app_motion_read_data(app_motion_data_t *data);

/**
 ****************************************************************************************
 * \brief Initialize motion module
 ****************************************************************************************
 */
void app_motion_init(void);

/**
 ****************************************************************************************
 * \brief Handler for the motion timer expiration event
 ****************************************************************************************
 */
void app_motion_timer_handler(void);

/**
 ****************************************************************************************
 * \brief Handler for the motion deactivation timer expiration event
 ****************************************************************************************
 */
void app_motion_deactivation_timer_handler(void);

/**
 ****************************************************************************************
 * \brief Start motion tracking
 ****************************************************************************************
 */
void app_motion_start(void);

/**
 ****************************************************************************************
 * \brief Stop motion tracking
 ****************************************************************************************
 */
void app_motion_stop(void);

/**
 ****************************************************************************************
 * \brief Process tasks while BLE is powered
 *
 * \return  true if system must remain active
 ****************************************************************************************
 */
bool app_motion_on_ble_powered(void);

/**
 ****************************************************************************************
 * \brief Process tasks while system is powered
 *
 * \return  APP_GOTO_SLEEP if the system can go to sleep
 *          APP_KEEP_POWERED if the system must remain active
 *          APP_BLE_WAKEUP if the BLE must be woken up
 ****************************************************************************************
 */
uint8_t app_motion_on_system_powered(void);

/**
 ****************************************************************************************
 * \brief
 *
 * \return true if motion tracking is active
 ****************************************************************************************
 */
bool app_motion_is_active(void);

#endif // APP_MOTION_H_

/**
 * \}
 * \}
 * \}
 */
