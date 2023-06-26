/*****************************************************************************************
 *
 * \file port_adv_fsm.h
 *
 * \brief The port_adv_fsm module provides an abstraction layer to platform specific 
 * advertising related functions such as starting/stopping advertising, as well as 
 * handling of advertising timers.
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
 * \addtogroup PORT_ADV_FSM
 * \brief Advertising FSM HAL implementation
 * \{
 ****************************************************************************************	 
 */
 
#ifndef PORT_ADV_FSM_H
#define PORT_ADV_FSM_H

#include "app_adv_fsm_defs.h"


/*****************************************************************************************
 * \brief Starts undirected advertising timer
 *
 * \param[in] adv_timer_remaining   The timeout value of the timer
******************************************************************************************/
void port_con_fsm_start_adv_timer(uint32_t adv_timer_remaining);

/*****************************************************************************************
 * \brief Handles the undirected advertising timer's expiration event
******************************************************************************************/
void port_adv_fsm_adv_timer_handler(void);

/*****************************************************************************************
 * \brief Starts undirected advertising
 *
 * \param[in] data    The undirected advertising data / parameters 
******************************************************************************************/
void port_adv_fsm_start_adv_undirected(start_adv_data_t *data);

/*****************************************************************************************
 * \brief Starts directed advertising
 *
 * \param[in] data    The directed advertising data / parameters 
******************************************************************************************/ 
void port_adv_fsm_start_adv_directed(start_adv_direct_data_t *data);

/*****************************************************************************************
 * \brief Stops UNDIRECTED advertising
******************************************************************************************/ 
void port_adv_fsm_adv_stop(void);


/* 
 * Advertise Completion Event Callback handlers 
 */

/*****************************************************************************************
 * \brief Used to handle the completion of an undirected advertising
 *
 * \param[in] status    The status of the undirected advertising completion
 *              GAP_ERR_CANCELED  : Advertising completed because a connection request interrupted it
 *              GAP_ERR_NO_ERROR  : Advertising completed because there was a request to stop it 
******************************************************************************************/
void port_adv_fsm_on_undirect_complete(uint8_t status);

/*****************************************************************************************
 * \brief Used to handle the completion of a direct advertising
 *
 * \param[in] status    The status of the directed advertising completion
 *              GAP_ERR_CANCELED  : Advertising completed because a connection request interrupted it
 *              GAP_ERR_NO_ERROR  : Advertising ended naturally
******************************************************************************************/
void port_adv_fsm_on_direct_complete(uint8_t status);

#endif // PORT_ADV_FSM_H

/**
 * \}
 * \}
 * \}
 */
