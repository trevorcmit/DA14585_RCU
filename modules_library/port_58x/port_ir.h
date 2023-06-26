/*****************************************************************************************
 *
 * \file port_ir.h
 *
 * \brief port_ir provides a hardware abstraction layer for initializing the
 * MCU peripherals in order to drive an infrared transmitter for sending IR commands.
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
 * \addtogroup IR
 * \{
 * \addtogroup PORT_IR
 * \{
 ****************************************************************************************	 
 */ 
 
#ifndef PORT_IR_H_
#define PORT_IR_H_
    
/*****************************************************************************************
 * \brief Send an IR command using RC-5 protocol
 *
 * \param[in] key:          RC5 command to be sent
 * \param[in] toggle_bit:   RC5 toggle bit
******************************************************************************************/        
void port_ir_rc5_send_command(uint16_t key, uint8_t toggle_bit);

/*****************************************************************************************
 * \brief
******************************************************************************************/        
void port_ir_reserve_gpios(void);

/*****************************************************************************************
 * \brief Start sending specified command using IR interface.
 *
 * \param[in] key:          command to be sent
 * \param[in] toggle_bit:   toggle bit
******************************************************************************************/
void port_ir_send_command(uint16_t key, uint8_t toggle_bit);

#endif // PORT_IR_H_

/**
 * \}
 * \}
 * \}
 */
