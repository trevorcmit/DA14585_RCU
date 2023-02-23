/**
 ****************************************************************************************
 *
 * \file app_ir.h
 *
 * \brief This module provides an API for initializing a pin for driving an IR transmitter
 * as well as sending commands.
 *
 * Define symbol HAS_IR to include this module in the application.
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
 
#ifndef APP_IR_H_
#define APP_IR_H_


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>          // standard integer definition
#include <co_bt.h>


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * \brief  Initialize IR interface.
 ****************************************************************************************
 */
void app_ir_init(void);

/**
 ****************************************************************************************
 * \brief  Start sending specified command using IR interface.
 *
 * \param[in] command: command which will be sent.
 ****************************************************************************************
 */
void app_ir_send_command(uint16_t command);

/**
 ****************************************************************************************
 * \brief  Stop sending IR command.
 ****************************************************************************************
 */
void app_ir_stop(void);

/**
 ****************************************************************************************
 * \brief  Check active status.
 *
 * \return True if active or false if not.
 ****************************************************************************************
 */
bool app_ir_is_active(void);


#endif // APP_IR_H_

/**
 * \}
 * \}
 * \}
 */
