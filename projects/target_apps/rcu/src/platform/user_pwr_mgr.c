/*****************************************************************************************
 *
 * \file user_pwr_mgr.c
 *
 * \brief Power management functions
 * 
******************************************************************************************/

/*****************************************************************************************
 * @addtogroup APP
 * @{
******************************************************************************************/


#ifdef HAS_PWR_MGR        


/*
 * INCLUDE FILES
******************************************************************************************/

#include "user_pwr_mgr.h"
#include "port_platform.h"
#include "port_timer.h"

uint32_t conn_timer_remaining                __PORT_RETAINED;

void user_pwr_mgr_init(void)
{ 
    if(pwr_mgr_params.inactivity_timeout != 0) {
        conn_timer_remaining = pwr_mgr_params.inactivity_timeout; // reset inactivity timer here    
    }
}


void user_pwr_mgr_reset_inactivity(void)
{
//    dbg_puts(DBG_PWR_MGR_LVL, "  (!) inactivity timer\r\n");
    if(pwr_mgr_params.inactivity_timeout != 0) {
        port_timer_clear(PWR_MGR_INACTIVITY_TIMER, BLE_TASK);
        conn_timer_remaining = pwr_mgr_params.inactivity_timeout;
        port_timer_set(PWR_MGR_INACTIVITY_TIMER, BLE_TASK, conn_timer_remaining); 
    }
}

void user_pwr_mgr_disable_inactivity(void)
{
    if(pwr_mgr_params.inactivity_timeout != 0) {
        dbg_puts(DBG_PWR_MGR_LVL, "  (-) inactivity timer\r\n");
        port_timer_clear(PWR_MGR_INACTIVITY_TIMER, BLE_TASK);
    }
    // in case of EXTENDED_TIMERS, extended_timer_cnt holds its value and can be used
    // when the timer is reset later...
}

void user_pwr_mgr_inactivity_timer_handler(void)
{
    if(pwr_mgr_params.inactivity_timeout != 0) {
        dbg_puts(DBG_PWR_MGR_LVL, "Inactivity timer exp\r\n");
        
        // Timer elapsed!
        if(pwr_mgr_params.inactivity_callback != NULL) {
            pwr_mgr_params.inactivity_callback();
        }
    }
}

#endif // HAS_PWR_MGR        

/// @} APP
