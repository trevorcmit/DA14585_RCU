/*****************************************************************************************
 *
 * \file azotouch_iqs_263.h
 *
 * \brief Azoteq IQS263 module library provides an API with the touch module. The library
 * uses a local FIFO that is used to push/pop incoming touch events. The azotouch_iqs_263_handle_events
 * is meant to be used whenever there is a touch event(i.e inside the RDY pin's interrupt handler).
 * The poll function is meant to be used to check if the touch event FIFO has any events (not empty)
 * and pop these events for further processing by the user application. The events supported by 
 * IQS263 touch module are touch, proximity, tap, wheel sliding and flick left/right gestures.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

#ifndef _AZOTOUCH_IQS_263_H_
#define _AZOTOUCH_IQS_263_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IQS263_I2C_DRV
 *
 * \{
******************************************************************************************/

#include <stdint.h>

/*********************** IQS263 REGISTERS *************************************/

#define DEVICE_INFO_REG             0x00

// System Flags/Events Register and bitfields

// Register
#define SYS_FLAGS_EVENTS_REG    0x01

// Bitfields Byte0
#define SYS_FLAGS_SHOW_RST      0x80
#define SYS_FLAGS_MOVMENT       0x40
#define SYS_FLAGS_ATI_ERR       0x20
#define SYS_FLAGS_PROJ_MODE     0x10
#define SYS_FLAGS_FILTER_HALT   0x08
#define SYS_FLAGS_ATI_BUSY      0x04
#define SYS_FLAGS_IND_HALT      0x02
#define SYS_FLAGS_LP_ACTIVE     0x01

// Bitfields Byte1
#define SYS_EVENT_FLICK_L   0x80
#define SYS_EVENT_FLICK_R   0x40
#define SYS_EVENT_TAP       0x20
#define SYS_EVENT_MOVEMENT  0x10
#define SYS_EVENT_ATI       0x08
#define SYS_EVENT_SLIDE     0x04
#define SYS_EVENT_TOUCH     0x02
#define SYS_EVENT_PROX      0x01


// Coordinates Register
#define COORDINATES_REG             0x02

// Touch Bytes Register
#define TOUCH_BYTES_REG             0x03
#define TOUCHBYTE0_CH0              0x01
#define TOUCHBYTE0_CH1              0x02
#define TOUCHBYTE0_CH2              0x04
#define TOUCHBYTE0_CH3              0x08
    
#define TOUCHBYTE1_CH0              0x01
#define TOUCHBYTE1_CH1              0x02
#define TOUCHBYTE1_CH2              0x04
#define TOUCHBYTE1_CH3              0x08

// Counts Register
#define COUNTS_REG                  0x04

// LTA Register
#define LTA_REG                     0x05

// Deltas Register
#define DELTAS_REG                  0x06

// Multipliers Register
#define MULTIPLIERS_REG             0x07

// Compensation Register
#define COMPENSATION_REG            0x08

// Proximity Settings Register & Bitfields
#define PROX_SETTINGS_REG            0x09

#define PROX_SET0_ATI_OFF            0x80
#define PROX_SET0_ATI_PART           0x40
#define PROX_SET0_ATI_BAND           0x20
#define PROX_SET0_REDO_ATI           0x10
#define PROX_SET0_RESEED             0x08
#define PROX_SET0_STREAM_ATI         0x04
#define PROX_SET0_4MHZ_OSC           0x02
#define PROX_SET0_FORCE_HALT         0x01


#define PROX_SET1_WDT_OFF            0x80
#define PROX_SET1_EVT_MODE           0x40

#define PROX_SET1_LTA_BETA_2_9       0x00
#define PROX_SET1_LTA_BETA_2_8       0x10
#define PROX_SET1_LTA_BETA_2_7       0x20
#define PROX_SET1_LTA_BETA_2_6       0x30

#define PROX_SET1_SLIDER_DISABLE     0x00
#define PROX_SET1_SLIDER_2CH         0x04
#define PROX_SET1_SLIDER_WHEEL       0x08
#define PROX_SET1_SLIDER_3CH         0x0C

#define PROX_SET1_CF_OFF             0x00
#define PROX_SET1_CF_BETA01          0x01
#define PROX_SET1_CF_BETA02          0x02
#define PROX_SET1_CF_BETA03          0x03

#define PROX_SET2_SLEEP_HALT         0x80
#define PROX_SET2_FORCE_SLEEP        0x40
#define PROX_SET2_WAKE_RELEASE2      0x20
#define PROX_SET2_WHEEL_FILTER       0x10

#define PROX_SET2_MOVE_DIS           0x00
#define PROX_SET2_MOVE_CH0           0x08
#define PROX_SET2_MOVE_CH3           0x04

#define PROX_SET2_PROX               0x00
#define PROX_SET2_SYNC               0x02
#define PROX_SET2_TOUCH_CH1          0x01
#define PROX_SET2_MOV_OUT            0x03

#define PROX_SET3_TOUCH_DEBOUNCE_SAMPLE4    0x80
#define PROX_SET3_TOUCH_SMALL_CAP           0x40
#define PROX_SET3_TOUCH_20UA                0x20
#define PROX_SET3_TOUCH_FLOATING            0x10
#define PROX_SET3_TOUCH_AUTOADJUST          0x08
#define PROX_SET3_TOUCH_TURBODISABLED       0x04
#define PROX_SET3_TOUCH_DETECT_FILTERED     0x02
#define PROX_SET3_TOUCH_SLOW_CHARGE         0x01

#define EVENT_MASK_FLICK_R                  0x80
#define EVENT_MASK_FLICK_L                  0x40
#define EVENT_MASK_TAP                      0x20
#define EVENT_MASK_MOVEMENT                 0x10
#define EVENT_MASK_ATI                      0x08
#define EVENT_MASK_SLIDE                    0x04
#define EVENT_MASK_TOUCH                    0x02
#define EVENT_MASK_PROX                     0x01

// Thresholds Register
#define THRESHOLDS_REG              0x0A

// Timings & Targets Register
#define TIMINGS_AND_TARGETS_REG     0x0B

// Gesture Timers Register
#define GESTURE_TIMERS_REG          0x0C

// Active Channels Register
#define ACTIVE_CHANNELS_REG         0x0D

#define AZOTOUCH_IQS_263_REGS       13

/*
* TYPEDEFS
*/
typedef struct
{
    uint8_t prox:1;
    uint8_t touch:1;
    uint8_t slide:1;
    uint8_t ati:1;
    uint8_t movement:1;
    uint8_t tap:1;
    uint8_t flick_left:1;
    uint8_t flick_right:1;
}azoteq_iqs263_actions_t;

typedef union
{
    azoteq_iqs263_actions_t eBitMask;
    uint8_t ebyte;
}azoteq_iqs263_actions_u;


typedef struct
{
    azoteq_iqs263_actions_u actions;
    int16_t relCoord;
    uint8_t wheelCoord;
} azoteq_iqs_263_evt_t;


/*****************************************************************************************
 * \brief Touch module initialization function
 * \return bool true on successful initialization of the module, false otherwise
******************************************************************************************/
bool azotouch_iqs_263_init_module(void);


/*****************************************************************************************
 * \brief Processing function used as a handler for Azoteq Touch events
******************************************************************************************/
void azotouch_iqs_263_process_event(azoteq_iqs_263_evt_t *el);


/*****************************************************************************************
 * \brief Touch module DE-initialization function
******************************************************************************************/
void azotouch_iqs_263_deinit_module(void);
/**
 * \}
 * \}
 * \}
 */

 #endif // _AZOTOUCH_IQS_263_H_
