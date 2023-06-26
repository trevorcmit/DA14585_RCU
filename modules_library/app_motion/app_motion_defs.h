/*****************************************************************************************
 *
 * \file app_motion_defs.h
 *
 * \brief Motion application definitions.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/
 
/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup MOTION
 * \{
 * \addtogroup APP_MOTION
 * \{
******************************************************************************************/
 
#ifndef APP_MOTION_DEFS_H_
#define APP_MOTION_DEFS_H_

#include "stdint.h"
#include "stdbool.h"

typedef void (*app_motion_wakeup_t)(void);
typedef int8_t (*app_motion_sleep_t)(void);
typedef void (*app_motion_config_t)(void);
typedef int8_t (*app_motion_read_t)(int16_t* temp, int16_t* acc, int16_t* rot);
typedef void (*app_motion_issue_bist_t)(void);
typedef bool (*app_motion_test_bist_t)(void);

typedef struct {
    app_motion_wakeup_t     wakeup;
    app_motion_sleep_t      sleep;
    app_motion_config_t     config;
    app_motion_read_t       read;
    app_motion_issue_bist_t issue_bist;
    app_motion_test_bist_t  test_bist;
} motion_util_funcs_t;

#endif // APP_MOTION_DEFS_H_

/**
 * \}
 * \}
 * \}
 */
