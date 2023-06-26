/*****************************************************************************************
 *
 * \file app_motion.c
 *
 * \brief Motion module source file
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

#ifdef HAS_MOTION

#include "app_motion.h"
#include <app_motion_config.h>
#include "port_platform.h"
#include "port_timer.h"

#define MOTION_CALL_CALBACK_VOID(func)       if(app_motion_funcs.func!=NULL) app_motion_funcs.func()
#define MOTION_CALL_CALBACK_VOID_RET(func)   app_motion_funcs.func()

enum motion_state {
    MOTION_STATE_IDLE               = 0,
    MOTION_STATE_WAITING_TO_WAKEUP  = 1,
    MOTION_STATE_START_BIST         = 2,
    MOTION_STATE_WAITING_FOR_BIST   = 3,
    MOTION_STATE_CHECK_BIST_STATUS  = 4,
    MOTION_STATE_WAITING_FOR_CONFIG = 5,
    MOTION_STATE_ACTIVE             = 6
};

enum motion_released_state {
    MOTION_RELEASED_ENABLED            = 0,
    MOTION_RELEASED_WAITING_FOR_CURSOR = 1,
    MOTION_RELEASED_DEACTIVATE         = 2,
    MOTION_RELEASED_IDLE               = 3
};

static int cnt;

static enum motion_state state_motion_pressed;
static enum motion_released_state state_motion_released;
static bool motion_key_pressed;

bool app_motion_read_data(app_motion_data_t *data)
{
    if(state_motion_pressed != MOTION_STATE_ACTIVE) {
        return false;
    }
    
    memset(data, 0, sizeof(app_motion_data_t));

    if (app_motion_funcs.read(&data->temperature, data->acc_motion, data->rot_motion) == 0) {
        cnt++;
        data->timestamp = cnt;
    }
    return true;
}

/*****************************************************************************************
 * \brief
******************************************************************************************/
static void app_motion_sleep(void)
{
    cnt = 0;
    MOTION_CALL_CALBACK_VOID(sleep);
}

/*****************************************************************************************
 * \brief
******************************************************************************************/
static void app_motion_wakeup(void)
{
    MOTION_CALL_CALBACK_VOID(wakeup);
    cnt = 0;
}

/*****************************************************************************************
 * \brief
******************************************************************************************/
static void app_motion_config(void)
{
    MOTION_CALL_CALBACK_VOID(config);
    cnt = 0;
}

/*****************************************************************************************
 * \brief
******************************************************************************************/
static void app_motion_state_machine(void)
{
    if (motion_key_pressed) {
        if (state_motion_released == 1) { //active waiting to shutdown
#if defined(MOTION_DEACTIVATION_TIMEOUT_IN_MS) && (MOTION_DEACTIVATION_TIMEOUT_IN_MS > 0)
            port_timer_clear(APP_MOT_DEACT_TIMER, 0);
#endif            
        }
        
        state_motion_released = MOTION_RELEASED_ENABLED;
     
        switch(state_motion_pressed) {
            case MOTION_STATE_IDLE:
                app_motion_wakeup();
                dbg_puts(DBG_MOT_LVL, "MOTION: Activated. Waiting to wakeup\r\n");
                state_motion_pressed = MOTION_STATE_WAITING_TO_WAKEUP;
                port_timer_set(APP_MOT_TIMER, 0, 60);
                break;
            case MOTION_STATE_START_BIST:
                MOTION_CALL_CALBACK_VOID(issue_bist);
                dbg_puts(DBG_MOT_LVL, "MOTION: Start BIST\r\n");
                state_motion_pressed = MOTION_STATE_WAITING_FOR_BIST;
                port_timer_set(APP_MOT_TIMER, 0, 20);
                break;
            case MOTION_STATE_CHECK_BIST_STATUS:
                if (MOTION_CALL_CALBACK_VOID_RET(test_bist)) {
                    dbg_puts(DBG_MOT_LVL, "MOTION: BIST OK\r\n");
                    app_motion_config();
                    state_motion_pressed = MOTION_STATE_WAITING_FOR_CONFIG;
                    port_timer_set(APP_MOT_TIMER, 0, 10);
                } else {
                    app_motion_wakeup();
                    dbg_puts(DBG_MOT_LVL, "MOTION: BIST not OK. Wait to wakeup\r\n");
                    state_motion_pressed = MOTION_STATE_WAITING_TO_WAKEUP;
                    port_timer_set(APP_MOT_TIMER, 0, 10);
                }
                break;
            case MOTION_STATE_ACTIVE:
            case MOTION_STATE_WAITING_TO_WAKEUP:
            case MOTION_STATE_WAITING_FOR_BIST:
            case MOTION_STATE_WAITING_FOR_CONFIG:
                break;
        }
    } else {
        switch(state_motion_released) {
            case MOTION_RELEASED_ENABLED:
#if defined(MOTION_DEACTIVATION_TIMEOUT_IN_MS) && (MOTION_DEACTIVATION_TIMEOUT_IN_MS > 0)
                port_timer_set(APP_MOT_DEACT_TIMER, 0, MOTION_DEACTIVATION_TIMEOUT_IN_MS);  // delay to get the cursor back
                state_motion_released = MOTION_RELEASED_WAITING_FOR_CURSOR;
#else
                state_motion_released = MOTION_RELEASED_DEACTIVATE;
#endif            
                break;
            case MOTION_RELEASED_DEACTIVATE:
                dbg_puts(DBG_MOT_LVL, "MOTION: Deactivated\r\n");
                app_motion_init();
                break;
            case MOTION_RELEASED_WAITING_FOR_CURSOR:
            case MOTION_RELEASED_IDLE:
                break;
        }
    }
}

void app_motion_init(void)
{
    state_motion_pressed = MOTION_STATE_IDLE;
    state_motion_released = MOTION_RELEASED_IDLE;
    cnt = 0;
    motion_key_pressed = false;
    port_timer_clear(APP_MOT_TIMER, 0);
#if defined(MOTION_DEACTIVATION_TIMEOUT_IN_MS) && (MOTION_DEACTIVATION_TIMEOUT_IN_MS > 0)    
    port_timer_clear(APP_MOT_DEACT_TIMER, 0);
#endif    
    app_motion_sleep();
}

void app_motion_timer_handler(void)
{
    if (state_motion_pressed < MOTION_STATE_ACTIVE) {
        state_motion_pressed++;
    }
}

void app_motion_deactivation_timer_handler(void)
{
    if (state_motion_released < MOTION_RELEASED_DEACTIVATE) {
        state_motion_released++;
    }
}

void app_motion_start(void)
{
    motion_key_pressed = true;
}

void app_motion_stop(void)
{
    motion_key_pressed = false;
}

bool app_motion_on_ble_powered(void)
{
    app_motion_state_machine();
    
    return false;
}
   
uint8_t app_motion_on_system_powered(void)
{
    if(motion_key_pressed == true && state_motion_pressed == MOTION_STATE_IDLE) {
        return APP_BLE_WAKEUP;
    }
    return APP_GOTO_SLEEP;
}

bool app_motion_is_active(void)
{
    return state_motion_pressed != MOTION_STATE_IDLE;
}

#endif // HAS_MOTION

/**
 * \}
 * \}
 * \}
 */
