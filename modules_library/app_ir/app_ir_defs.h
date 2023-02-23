/**
 ****************************************************************************************
 *
 * \file app_ir_defs.h
 *
 * \brief IR module definitions
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
 * \addtogroup IR
 * \{
 * \addtogroup APP_IR
 *
 * \brief IR protocol module.
 *
 * \{
 ****************************************************************************************
 */
 
#ifndef APP_IR_DEFS_H_
#define APP_IR_DEFS_H_

#include "gpio.h"

typedef struct
{
    uint16_t carrier_period;   // in number of clock periods
    uint16_t carrier_on_time;  // in number of clock periods
    uint16_t repeat_time;      // in number of clock cycles
    uint8_t  repeat_type;
    uint8_t  timer_freq;       // timer frequency    
    uint8_t  logic_one_mark;
    uint8_t  logic_one_space;
    uint8_t  logic_zero_mark;
    uint8_t  logic_zero_space;
} ir_protocol_params_t;

typedef void (*app_ir_send_command_callback_t)(uint16_t key, uint8_t toggle_bit);

typedef struct
{
    uint16_t max_repeat;
    bool use_ble_sync;
    const ir_protocol_params_t *protocol_params;
    app_ir_send_command_callback_t send_command_callback;
} ir_params_t;

#endif // APP_IR_DEFS_H_

/**
 * \}
 * \}
 * \}
 */
