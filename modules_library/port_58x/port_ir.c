/*****************************************************************************************
 *
 * \file port_ir.c
 *
 * \brief IR module platform adaptation source file
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup IR
 * \{
 * \addtogroup PORT_IR
 * \{
 ****************************************************************************************	 
 */ 
 

/*
 * INCLUDE FILES
******************************************************************************************/
 
#include "ir_driver.h"
#include "port_platform.h"
#include "app_ir_config.h"


/*
 * EXTERNAL VARIABLE DECLARATIONS
******************************************************************************************/


/*
 * RETAINED VARIABLE DECLARATIONS
******************************************************************************************/

uint16_t    port_ir_key        __PORT_RETAINED;
uint8_t     port_ir_toggle_bit __PORT_RETAINED;


/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

void port_ir_rc5_send_command(uint16_t key, uint8_t toggle_bit) 
{
    //Insert a new digital message into code fifo: 2 bits length start message (always 0x03)
    ir_insert_digital_message(IR_CODE_FIFO, 2, 0x03);
    
    //Insert toggle bit value into code fifo: value of this bit will be toggled for each 
    // button press/release event
    ir_insert_digital_message(IR_CODE_FIFO, 1, toggle_bit);
    
    //Insert remote device address into code fifo: address is 5 bit length value. 
    // The ir_reverse_bit_order() function was used to reverse bits order required by 
    // RC5 protocol.
    ir_insert_digital_message(IR_CODE_FIFO, 5, ir_reverse_bit_order((key >> 8) & 0x00FF, 5));
    
    //Insert remote device command into code fifo: command value is 6 bit length value. 
    // The ir_reverse_bit_order() function was used to reverse bits order required by 
    // RC5 protocol.
    ir_insert_digital_message(IR_CODE_FIFO, 6, ir_reverse_bit_order(key & 0x00FF,        6));
}

void port_ir_sirc_send_command(uint16_t key, uint8_t toggle_bit) 
{
    ASSERT_ERROR(0); // Code provided as reference. 
/*    
    //Insert paint message into code fifo: this is the first part of the message header
    ir_insert_paint_message(IR_CODE_FIFO, IR_PAINT_SYMBOL_TYPE_MARK, 96);
    
    //Insert paint message into code fifo: this is the second part of the message header
    ir_insert_paint_message(IR_CODE_FIFO, IR_PAINT_SYMBOL_TYPE_SPACE, 24);
    
    //Insert remote device command into code fifo: command is 7 bit length value.
    ir_insert_digital_message(IR_CODE_FIFO, 7, key & 0x00FF);
    
    //Insert remote device address into code fifo: address is 5 bit length value.
    ir_insert_digital_message(IR_CODE_FIFO, 5, (key >> 8) & 0x00FF);
*/    
}

void port_ir_nec_send_command(uint16_t key, uint8_t toggle_bit) 
{
    ASSERT_ERROR(0); // Code provided as reference. 
/*    
    //Insert paint mark message into repeat fifo: this is the first part of the special repeat message
    ir_insert_paint_message(IR_REPEAT_FIFO, IR_PAINT_SYMBOL_TYPE_MARK, 342);
    //Insert paint space message into repeat fifo: this is the second part of the special repeat message
    ir_insert_paint_message(IR_REPEAT_FIFO, IR_PAINT_SYMBOL_TYPE_SPACE, 86);
    //Insert paint mark message into repeat fifo: this is the third part of the special repeat message
    ir_insert_paint_message(IR_REPEAT_FIFO, IR_PAINT_SYMBOL_TYPE_MARK, 21);
    
    //Insert paint message into code fifo: this is the first part of the message header
    ir_insert_paint_message(IR_CODE_FIFO, IR_PAINT_SYMBOL_TYPE_MARK, 342);
    //Insert paint message into code fifo: this is the second part of the message header
    ir_insert_paint_message(IR_CODE_FIFO, IR_PAINT_SYMBOL_TYPE_SPACE, 171);
    
    //Insert remote device address into code fifo: address is 8 bit length value.
    ir_insert_digital_message(IR_CODE_FIFO, 8, (key >> 8) & 0x00FF);
    //Insert remote device inverted address into code fifo: address is 8 bit length value.
    ir_insert_digital_message(IR_CODE_FIFO, 8, 0xFF & (~((key >> 8) & 0x00FF)));
    //Insert remote device command into code fifo: command is 8 bit length value.
    ir_insert_digital_message(IR_CODE_FIFO, 8, key & 0x00FF);
    //Insert remote device inverted command into code fifo: command is 8 bit length value.
    ir_insert_digital_message(IR_CODE_FIFO, 8, 0xFF & (~(key & 0x00FF)));
    //Insert paint message into code fifo: this is the message end indicator
    ir_insert_paint_message(IR_CODE_FIFO, IR_PAINT_SYMBOL_TYPE_MARK, 21);
*/    
}

/*****************************************************************************************
 * \brief   Handler of a dummy TASK_APP msg sent to trigger sending the command
******************************************************************************************/                                    
static void port_ir_msg_handler(void)
{
    ir_flush_code_fifo();
    ir_flush_repeat_fifo();
    
    if(ir_params.send_command_callback != NULL) {
        (ir_params.send_command_callback)(port_ir_key, port_ir_toggle_bit);
    }
    else {
        ASSERT_WARNING(0); // IR protocol not implemented
    }   
    
    ir_start();
}

void port_ir_send_command(uint16_t key, uint8_t toggle_bit)
{
    port_ir_toggle_bit = toggle_bit;
    port_ir_key = key;

    // Put a message to the queue for the BLE to handle. The message
    // is handled by the app and sends the command.
    ke_msg_id_t ir_msg = app_easy_msg_set(port_ir_msg_handler);
    ASSERT_WARNING(ir_msg != 0xFFFF);
    if(ir_msg != 0xFFFF) {
        ke_msg_send_basic(ir_msg, TASK_APP, 0);
    }
    
    arch_ble_force_wakeup();
    arch_ble_ext_wakeup_off();
}

void port_ir_reserve_gpios(void)
{
    PORT_RESERVE_GPIO(app_ir_pins[IR_PIN]); 
}

/**
 * \}
 * \}
 * \}
 */
