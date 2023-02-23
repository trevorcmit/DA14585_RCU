/**
 ****************************************************************************************
 *
 * \file app_adv_fsm.h
 *
 * \brief The Advertise FSM lib provides a mechanism for handling requests related
 * to starting/stopping advertising procedures. 
 * The lib provides an API for initializing the Advertisting Finite State Machine 
 * as well as starting various types of advertising procedures and gives also the ability 
 * to the Connection FSM to get notified about advertising related events.
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
 * \addtogroup BONDING
 * \{
 * \addtogroup APP_ADV_FSM
 *
 * \brief Advertising FSM header
 * \{
 ****************************************************************************************	 	 
 */
 
#ifndef APP_ADV_FSM_H
#define APP_ADV_FSM_H

#include "app_adv_fsm_defs.h"


/**
 ****************************************************************************************
 * \brief Advertise FSM event handler. The core function of the Advertise FSM
 *        This function actually "feeds" the advertising FSM with events that occured
 *        and need to be handled/processed
 *
 * \param[in] adv_fsm_evt   The event to be processed by the Advertise FSM
 ****************************************************************************************
 */
void app_adv_fsm_process_evt(adv_fsm_events_t adv_fsm_evt);


/**
 ****************************************************************************************
 * \brief Initializes the Advertise FSM
 ****************************************************************************************
 */
void app_adv_fsm_init(void);


/**
 ****************************************************************************************
 * \brief Stops advertising
 *        In case of ongoing directed advertising the FSM will wait for the advertising
 *        to end and then set the state to ADV_IDLE (when the DIR_ADV_COMPLETED event occurs)
 *        In case of ongoing undirected advertising it will immediately stop the undirected
 *        advertising and set the state to ADV_IDLE (when the UND_ADV_COMPLETED event occurs)
 ****************************************************************************************
 */
void app_adv_fsm_adv_stop(void);


/**
 ****************************************************************************************
 * \brief Starts undirected advertising with a given filter policy
 *        Uses ADV_SETTING_UNDIRECTED as the advertising parametes 
 *        (see adv_fsm_config.adv_params array in app_adv_fsm_config.h)
 *        It is actually undirected advertising with pairing allowed
 *
 * \param[in] undFilterP the advertising filter policy
 ****************************************************************************************
 */
void app_adv_fsm_und_start(enum adv_filter_policy undFilterP);


/**
 ****************************************************************************************
 * \brief Starts undirected advertising(no pairing allowed) with a given filter policy
 *        Uses ADV_SETTING_UNDIRECTED_NO_PAIR as the advertising parametes 
 *        (see adv_fsm_config.adv_params array in app_adv_fsm_config.h)
 *        It is actually undirected advertising with NO pairing allowed
 *
 * \param[in] undFilterP the advertising filter policy
 ****************************************************************************************
 */
void app_adv_fsm_und_start_no_pair(enum adv_filter_policy undFilterP);


/**
 ****************************************************************************************
 * \brief Starts directed advertising to a specific peer
 *
 * \param[in] address A pointer to the address of the peer to which the device will advertise
 ****************************************************************************************
 */
void app_adv_fsm_dir_start(struct bd_addr *address);


/**
 ****************************************************************************************
 * \brief Starts limited undirected advertising
 *        Uses ADV_SETTING_UNDIRECTED_LIM as the advertising parametes 
 *        (see adv_fsm_config.adv_params array in app_adv_fsm_config.h) 
 *
 * \param[in] undFilterP The filter policy
 ****************************************************************************************
 */
void app_adv_fsm_und_start_lim(enum adv_filter_policy undFilterP);


#ifdef HAS_SPECIAL_ADVERTISING
/**
 ****************************************************************************************
 * \brief Starts special undirected advertising
 *        Uses ADV_SETTING_SPECIAL as the advertising parametes 
 *        (see adv_fsm_config.adv_params array in app_adv_fsm_config.h) 
 *        Use app_adv_fsm_set_special_adv_data function before starting "special"
 *        advertising in order to set the special advertising payload and payload length
 ****************************************************************************************
 */
void app_adv_fsm_start_special(void);


/**
 ****************************************************************************************
 * \brief Sets the special advertising data
 *
 * \param[in] specialData the special advertising data
 * \param[in] special_data_len the special advertising data length
 ****************************************************************************************
 */
void app_adv_fsm_set_special_adv_data(uint8_t *special_data, uint8_t special_data_len);
#endif

#endif // APP_ADV_FSM_H

/**
 * \}
 * \}
 * \}
 */
