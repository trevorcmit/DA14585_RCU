/*****************************************************************************************
 *
 * \file port_systick.c
 *
 * \brief Systick controller functions
 *
 * Define symbol HAS_SYSTICK to include this module in the application.
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
 * \addtogroup SYSTICK
 * \{
 * \addtogroup PORT_SYSTICK
 * \{
 ****************************************************************************************	 
 */ 

/*
 * INCLUDE FILES
******************************************************************************************/

#ifdef HAS_SYSTICK

#include "port_systick.h"
#include "systick.h"

uint32_t systick_counter[NUM_OF_SYSTICK_CHANNELS];
uint32_t systick_period[NUM_OF_SYSTICK_CHANNELS];

void port_systick_handler(void)
{
    for(uint8_t i = 0; i < NUM_OF_SYSTICK_CHANNELS; i++) {            
        systick_counter[i]--;
        if (systick_counter[i] == 0) {
            systick_counter[i] = systick_period[i];
            systick_config[i].callback();
        }
    }
}

void port_systick_init(void)
{
    for(uint8_t i = 0; i < NUM_OF_SYSTICK_CHANNELS; i++) {
        ASSERT_WARNING(systick_config[i].callback != NULL);
        systick_counter[i] = 0;
        systick_period[i] = 0;
    }
    systick_register_callback(port_systick_handler);
}

void port_systick_restart(enum port_systick_channel channel, uint32_t period)
{
    systick_period[channel] = 0; // Stop it
    
    port_systick_start(channel, period);
}
    
void port_systick_start(enum port_systick_channel channel, uint32_t period)
{
    if (GetBits32(&SysTick->CTRL, SysTick_CTRL_ENABLE_Msk) == 0) {
        port_systick_init();
        
        // Start systick if its not already started
        systick_start(SYSTICK_PERIOD_IN_US, 1);
        
        // Force active mode until systick is not active
        port_force_active_mode();
    }
    
    if(systick_period[channel] == 0) {                          // if channels is stopped
        systick_counter[channel] = period/SYSTICK_PERIOD_IN_US; // reset counter
        ASSERT_WANRING((period%SYSTICK_PERIOD_IN_US) == 0);     // Exact period cannot be achieved.
    }
    
    systick_period[channel] = period/SYSTICK_PERIOD_IN_US; // Update period
}    
    
void port_systick_stop(enum port_systick_channel channel)
{
    systick_period[channel] = 0;

    for(uint8_t i = 0; i < NUM_OF_SYSTICK_CHANNELS; i++) {
        if(systick_period[i] != 0) {
            return;
        }
    }    
    // If all channels are stopped, stop the systick
    systick_stop();
    
    // Also restore sleep mode since systick is not active anymore
    port_restore_sleep_mode();
}
#endif

/**
 * \}
 * \}
 * \}
 */
