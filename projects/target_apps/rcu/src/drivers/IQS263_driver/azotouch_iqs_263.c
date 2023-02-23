/**
 ****************************************************************************************
 *
 * \file azotouch_iqs_263.c
 *
 * \brief Azoteq IQS 263 module library sourcecode
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
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IQS263_I2C_DRV
 *
 * \{
 ****************************************************************************************
 */

#include "azotouch_iqs_263.h"
#include "i2c_azotouch.h"
#include "port_platform.h"



/**
 ***************************************************************************************
 * I2C Configuration for the IQS263 module                                                
 ***************************************************************************************
 */
#define I2C_AZOTOUCH_IQS263_SLAVE_ADDRESS       0x44                    // The IQS263 I2C device address is 0x44  
#define I2C_AZOTOUCH_IQS263_SPEED_MODE          I2C_AZOTOUCH_FAST       // Selected Speed mode: I2C_FAST (400 kbits/s)
#define I2C_AZOTOUCH_IQS263_ADDRESS_MODE        I2C_AZOTOUCH_7BIT_ADDR  // Addressing mode: I2C_7BIT_ADDR
#define I2C_AZOTOUCH_IQS263_ADDRESS_SIZE        1                       // The internal address(register address) size is 1 byte



/**
 ***************************************************************************************
 * IQS 263 initialization register values                                                    
 ***************************************************************************************
 */
 
// Used to switch Projected mode & set Global Filter Halt (0x01 Byte1)
#define SYSTEM_FLAGS_VAL					0x00

// Enable / Disable system events (0x01 Byte2)
#define SYSTEM_EVENTS_VAL					0x01

// Change the Multipliers & Base value (0x07 in this order)
#define MULTIPLIERS_CH0_VAL						(2<<4 | 5 ) // 0x08// - default
#define MULTIPLIERS_CH1_VAL						(0<<4 | 15) // 0x08// - default
#define MULTIPLIERS_CH2_VAL						(0<<4 | 14) // 0x08// - default
#define MULTIPLIERS_CH3_VAL						(1<<4 | 5 ) // 0x08// - default
#define BASE_VAL						        (1<<6 | 2 ) // 0x08// - default

// Change the Compensation for each channel (0x08 in this order)
#define COMPENSATION_CH0_VAL					0x51 
#define COMPENSATION_CH1_VAL					0x49 
#define COMPENSATION_CH2_VAL					0x4A 
#define COMPENSATION_CH3_VAL					0x49 

// Change the Prox Settings or setup of the IQS263 (0x09 in this order)
#define PROXSETTINGS0_VAL					PROX_SET0_REDO_ATI
#define PROXSETTINGS1_VAL					(PROX_SET1_EVT_MODE | PROX_SET1_LTA_BETA_2_8 | \
										PROX_SET1_SLIDER_WHEEL | PROX_SET1_CF_BETA01) // 0x59 
#define PROXSETTINGS2_VAL					0x00
#define PROXSETTINGS3_VAL					0x00
#define EVENT_MASK_VAL						( EVENT_MASK_TAP  | EVENT_MASK_SLIDE | EVENT_MASK_FLICK_R | EVENT_MASK_FLICK_L | EVENT_MASK_TOUCH)
							
// Change the Thresholds for each channel (0x0A in this order)
#define PROX_THRESHOLD_VAL					0x04
#define TOUCH_THRESHOLD_CH1_VAL				0x0C
#define TOUCH_THRESHOLD_CH2_VAL				0x0C
#define TOUCH_THRESHOLD_CH3_VAL				0x0C
#define MOVEMENT_THRESHOLD_VAL				0x03
#define RESEED_BLOCK_VAL					0x00
#define HALT_TIME_VAL						0x14
#define I2C_TIMEOUT_VAL						0x04

// Change the Timing settings (0x0B in this order)
#define LOW_POWER_VAL						0x10
#define ATI_TARGET_TOUCH_VAL				0x40
#define ATI_TARGET_PROX_VAL					0x80

// Change the Gesture Timing settings (0x0C in this order)
#define TAP_TIMER_VAL						0x20
#define FLICK_TIMER_VAL						0x14
#define FLICK_THRESHOLD_VAL					0xB4

// Set Active Channels (0x0D) */
#define ACTIVE_CHS_VAL						0x0F


#define AZOTOUCH_SETUP_REGS                 8


/*
* TYPEDEFS
*/

typedef struct
{
    uint8_t regAddr;
    uint8_t *regSettings;
}azo_iqs_263_regs_t;


/*
* LOCAL VARIABLES
*/

static const uint8_t iqs_263_sys_flags_reg[] = {1, SYSTEM_EVENTS_VAL};

static const uint8_t iqs_263_active_chan_reg[] = {1, ACTIVE_CHS_VAL};

static const uint8_t iqs_263_threshold_reg[] = {8, PROX_THRESHOLD_VAL, 
                                TOUCH_THRESHOLD_CH1_VAL, TOUCH_THRESHOLD_CH2_VAL, 
                                TOUCH_THRESHOLD_CH3_VAL, MOVEMENT_THRESHOLD_VAL,
                                RESEED_BLOCK_VAL, HALT_TIME_VAL, I2C_TIMEOUT_VAL};

static const uint8_t iqs_263_timing_targ_reg[] = {3, LOW_POWER_VAL, ATI_TARGET_TOUCH_VAL, ATI_TARGET_PROX_VAL};

static const uint8_t iqs_263_mult_reg[] = {4, MULTIPLIERS_CH0_VAL, MULTIPLIERS_CH1_VAL, MULTIPLIERS_CH2_VAL, MULTIPLIERS_CH3_VAL};
static const uint8_t iqs_263_prox_reg[] = {5, PROXSETTINGS0_VAL, PROXSETTINGS1_VAL,PROXSETTINGS2_VAL,PROXSETTINGS3_VAL,EVENT_MASK_VAL};
static const uint8_t iqs_263_comp_reg[] = {4, COMPENSATION_CH0_VAL, COMPENSATION_CH1_VAL, COMPENSATION_CH2_VAL, COMPENSATION_CH3_VAL};
static const uint8_t iqs_263_gesture_timers_reg[] = {3, TAP_TIMER_VAL, FLICK_TIMER_VAL, FLICK_THRESHOLD_VAL};

static uint8_t azotouchInitCounter      __PORT_RETAINED;

azo_iqs_263_regs_t azotouchRegSetup[AZOTOUCH_SETUP_REGS] ={
    
    {SYS_FLAGS_EVENTS_REG,      (uint8_t*)iqs_263_sys_flags_reg     },
    {MULTIPLIERS_REG,           (uint8_t*)iqs_263_mult_reg          },
    {COMPENSATION_REG,          (uint8_t*)iqs_263_comp_reg          },
    {THRESHOLDS_REG,            (uint8_t*)iqs_263_threshold_reg     },
    {TIMINGS_AND_TARGETS_REG,   (uint8_t*)iqs_263_timing_targ_reg   },
    {GESTURE_TIMERS_REG,        (uint8_t*)iqs_263_gesture_timers_reg},
    {ACTIVE_CHANNELS_REG,       (uint8_t*)iqs_263_active_chan_reg   },
    {PROX_SETTINGS_REG,         (uint8_t*)iqs_263_prox_reg          }
};

    
    
    
bool azotouch_iqs_263_init_module()
{
    
    azotouchInitCounter = 0;
    
    return true;
}


void azotouch_iqs_263_process_event(azoteq_iqs_263_evt_t *el)
{
    uint8_t azoBuff[5];
    
    // Init the I2C for start of communication
	i2c_azotouch_init(I2C_AZOTOUCH_IQS263_SLAVE_ADDRESS, I2C_AZOTOUCH_IQS263_SPEED_MODE,
                          I2C_AZOTOUCH_IQS263_ADDRESS_MODE, I2C_AZOTOUCH_IQS263_ADDRESS_SIZE);
    
    // If the touch setup hasnt been written yet
    // Write any remaining settings and return
    if(azotouchInitCounter < AZOTOUCH_SETUP_REGS) {
        
        i2c_azotouch_data_write(azotouchRegSetup[azotouchInitCounter].regAddr, azotouchRegSetup[azotouchInitCounter].regSettings+1,
                *((uint8_t*)azotouchRegSetup[azotouchInitCounter].regSettings));
        ++azotouchInitCounter;
        return;
    }
    
    // Read the event data
    i2c_azotouch_repeat_read_data(SYS_FLAGS_EVENTS_REG, COORDINATES_REG, azoBuff, 2 , 3);

    // Extract the events as they were read from the 
    // touch sensor module
    el->actions.ebyte = azoBuff[1];
    el->relCoord = azoBuff[2] | (((uint16_t)azoBuff[3])<<8);
    el->wheelCoord = azoBuff[4];
}


void azotouch_iqs_263_deinit_module()
{
        // De-init Azoteq Touchpad module
        // i2c_azotouch_release();
}

/**
 * \}
 * \}
 * \}
 */
