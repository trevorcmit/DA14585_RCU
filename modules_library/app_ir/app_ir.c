/*****************************************************************************************
 *
 * \file app_ir.c
 *
 * \brief IR module source file
 * 
******************************************************************************************/

 /*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup IR
 * \{
 * \addtogroup APP_IR
 *
 * \brief IR protocol module.
 *
 * \{
******************************************************************************************/

#ifdef HAS_IR


/*
 * INCLUDE FILES
******************************************************************************************/

#include <app_ir_config.h>
#include "app_ir.h"
#include "ir_driver.h"
#include "port_ir.h"
#include "port_platform.h"


/*
 * RETAINED VARIABLE DECLARATIONS
******************************************************************************************/

uint8_t ir_toggle_bit __PORT_RETAINED;


/*
 * FUNCTION DEFINITIONS
******************************************************************************************/

void app_ir_init(void)
{
    ir_init();
    
    CLK_PER_REG_TMR_DIV_t trm_div;
    
    const ir_protocol_params_t *params = ir_params.protocol_params;
    
    switch(params->timer_freq) {
    case 16:
        trm_div = CLK_PER_REG_TMR_DIV_1;
        break;
    case 8:
        trm_div = CLK_PER_REG_TMR_DIV_2;
        break;        
    case 4:
        trm_div = CLK_PER_REG_TMR_DIV_4;
        break;        
    case 2:
        trm_div = CLK_PER_REG_TMR_DIV_8;
        break;        
    default:
        ASSERT_ERROR(0); // Timer frequency not supported
    }
        
    ir_set_carrier_freq(params->carrier_on_time, params->carrier_period, trm_div);
    ir_set_logic_one_time(IR_LOGIC_ONE_STARTS_SPACE, params->logic_one_mark, params->logic_one_space);
    ir_set_logic_zero_time(IR_LOGIC_ZERO_STARTS_MARK, params->logic_zero_mark, params->logic_zero_space);
    ir_set_repeat_type(params->repeat_type);
    ir_set_repeat_time(params->repeat_time);
}

void app_ir_send_command(uint16_t key)
{
    port_ir_send_command(key, ir_toggle_bit);
}

void app_ir_stop(void)
{
    ir_stop();
    ir_toggle_bit ^= 0x01;
    //while(ir_is_busy()){};
}

bool app_ir_is_active(void)
{
    return ir_is_busy();
}

#endif // HAS_IR

/**
 * \}
 * \}
 * \}
 */
