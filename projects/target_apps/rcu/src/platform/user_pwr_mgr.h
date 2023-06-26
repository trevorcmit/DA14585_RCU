/*****************************************************************************************
 *
 * \file user_pwr_mgr.h
 *
 * \brief Power management functions
 *
 * Define symbol HAS_PWR_MGR to include this module in the application.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

#ifndef _USER_PWR_MGR_H_
#define _USER_PWR_MGR_H_

/*****************************************************************************************
 * @addtogroup APP
 * @ingroup PWR_MGR
 *
 * @brief Power management function implementation.
 *
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/
typedef void (pwr_mgr_callback_t)(void);

#include <user_pwr_mgr_config.h>

/*****************************************************************************************
 * \brief Initializes the power manager
******************************************************************************************/
void user_pwr_mgr_init(void);

/*****************************************************************************************
 * \brief Resets the inactivity timer
******************************************************************************************/
void user_pwr_mgr_reset_inactivity(void);

/*****************************************************************************************
 * \brief Disables inactivity timer
******************************************************************************************/
void user_pwr_mgr_disable_inactivity(void);

/*****************************************************************************************
 * \brief Handles the inactivity timer expiration
******************************************************************************************/
void user_pwr_mgr_inactivity_timer_handler(void);

/// @} APP

#endif // _USER_PWR_MGR_H_
