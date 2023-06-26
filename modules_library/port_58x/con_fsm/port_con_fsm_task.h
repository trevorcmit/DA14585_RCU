/*****************************************************************************************
 *
 * \file port_con_fsm_task.h
 *
 * \brief Connection FSM handlers header file.
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
 * \addtogroup BONDING
 * \{
 * \addtogroup PORT_CON_FSM
 * \brief Connection FSM HAL implementation
 * \{
 ****************************************************************************************	 
 */
 
#ifndef PORT_CON_FSM_TASK_H_
#define PORT_CON_FSM_TASK_H_

#include "ke_msg.h"


/*
 * FUNCTION DECLARATIONS
******************************************************************************************/
                                    
/*****************************************************************************************
 * \brief   Handler of the Enc Timer
 *
 * \remarks In case encryption is not activated by the remote host and the connection
 *           is still alive (if it wasn't then the timer would have been cleared),
 *           the handler will drop the connection. This situation appears in certain
 *           cases when pairing fails.
******************************************************************************************/
void port_con_fsm_enc_timer_handler(void);		

/*****************************************************************************************
 * \brief   Handler of the Connection FSM Timer - Action depends on the app state
******************************************************************************************/
void port_con_fsm_timer_handler(void);                                    

/*****************************************************************************************
 * \brief       Handler of the "Host switch" timer.
 *
 * \details     Called when the "switching period" of 60 sec expires.
******************************************************************************************/
void port_con_fsm_alt_pair_timer_handler(void);
          
/*****************************************************************************************
 * \brief
 *
 * \param[in]   msgid
 * \param[in]   param
 *
 * \return
******************************************************************************************/
enum process_event_response port_con_fsm_process_handler(ke_msg_id_t const msgid, void const *param);
    
#endif // PORT_CON_FSM_TASK_H_

/**
 * \}
 * \}
 * \}
 */
