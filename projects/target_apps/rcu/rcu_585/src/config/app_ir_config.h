/*****************************************************************************************
 *
 * \file app_ir_config.h
 *
 * \brief  IR module configuration header file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/ 
 
#ifndef _APP_IR_CONFIG_H_
#define _APP_IR_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup IR_CFG
 *
 * \brief IR module configuration
 * \{
******************************************************************************************/

#include "app_ir_defs.h"
#include "port_platform.h"
#include "port_ir.h"
#include "ir_driver.h"

/*****************************************************************************************
 * pin configuration
******************************************************************************************/ 
enum ir_pin_ids {
    IR_PIN,
};

static const pin_type_t app_ir_pins[] = {
    [IR_PIN]  = {.port = GPIO_PORT_2, .pin = GPIO_PIN_0, .high = 0, .mode_function = INPUT_PULLDOWN | PID_GPIO },
};

/*****************************************************************************************
 * Philips RC-5 protocol parameters
******************************************************************************************/ 
static const ir_protocol_params_t ir_rc5_params = {  
/*****************************************************************************************
 * Timer frequency in MHz. Choose between 2, 4, 8 and 16MHz
******************************************************************************************/    
    .timer_freq       = 4,
    
/*****************************************************************************************
 * IR carrier clock period in timer clock cycles
******************************************************************************************/    
    .carrier_period   = 111,  // Carrier frequency is 36.036KHz

/*****************************************************************************************
 * IR carrier on time in timer clock cycles
******************************************************************************************/    
    .carrier_on_time  = 28,   // Carrier duty cycle is 25%

/*****************************************************************************************
 * IR logic "one" mark in carrier clock cycles
******************************************************************************************/    
    .logic_one_mark   = 32,

/*****************************************************************************************
 * IR logic "one" space in carrier clock cycles
******************************************************************************************/    
    .logic_one_space  = 32,

/*****************************************************************************************
 * IR logic "zero" mark in carrier clock cycles
******************************************************************************************/    
    .logic_zero_mark  = 32,

/*****************************************************************************************
 * IR logic "zero" space in carrier clock cycles
******************************************************************************************/    
    .logic_zero_space = 32,
    
/*****************************************************************************************
 * Repeat type, either IR_REPEAT_FROM_CODE_FIFO or IR_REPEAT_FROM_REPEAT_FIFO
******************************************************************************************/    
    .repeat_type      = IR_REPEAT_FROM_CODE_FIFO,

/*****************************************************************************************
 * Message repeat time in carrier clock cycles
******************************************************************************************/    
    .repeat_time      = 4100, // 113.775msec
};

/*****************************************************************************************
 * Sony SIRC protocol parameters. *** ONLY FOR REFERENCE ***
******************************************************************************************/ 
static const ir_protocol_params_t ir_sirc_params = {  
    .timer_freq       = 4,
    .carrier_period   = 100,  // Carrier frequency is 40KHz
    .carrier_on_time  = 25,   // Carrier duty cycle is 25%
    .logic_one_mark   = 48,
    .logic_one_space  = 24,
    .logic_zero_mark  = 24,
    .logic_zero_space = 24,
    .repeat_type      = IR_REPEAT_FROM_CODE_FIFO,
    .repeat_time      = 1800, // 45msec
};

/*****************************************************************************************
 * NEC protocol parameters. *** ONLY FOR REFERENCE ***
******************************************************************************************/ 
static const ir_protocol_params_t ir_nec_params = {  
    .timer_freq       = 16,
    .carrier_period   = 421,  // Carrier frequency is 38KHz
    .carrier_on_time  = 105,  // Carrier duty cycle is 25%
    .logic_one_mark   = 21,
    .logic_one_space  = 64,
    .logic_zero_mark  = 21,
    .logic_zero_space = 21,
    .repeat_type      = IR_REPEAT_FROM_REPEAT_FIFO,
    .repeat_time      = 4180, // 110msec
};

static const ir_params_t ir_params = {
/*****************************************************************************************
 * The maximum number of repeat messages
******************************************************************************************/    
    .max_repeat      = 500,

/*****************************************************************************************
 * Use synchronization between IR and BLE. Using synchronization can reduce power 
 * consumption
******************************************************************************************/
    .use_ble_sync    = false,

/*****************************************************************************************
 * This callback function will be called to send the command. It must be implemented 
 * according to the IR protocol used.
******************************************************************************************/    
    .send_command_callback = port_ir_rc5_send_command,
//    .send_command_callback = port_ir_sirc_send_command,
//    .send_command_callback = port_ir_nec_send_command,
    
/*****************************************************************************************
 * The IR protocol parameters
******************************************************************************************/    
    .protocol_params = &ir_rc5_params,
//    .protocol_params = &ir_sirc_params,
//    .protocol_params = &ir_nec_params,
};

/**
 * \}
 * \}
 * \}
 */

#endif	// _APP_IR_CONFIG_H_
