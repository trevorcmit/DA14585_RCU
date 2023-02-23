/**
 ****************************************************************************************
 *
 * \file port_security.h
 *
 * \brief Security platform include file.
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
 * \addtogroup PORT_SECURITY
 * \{
 ****************************************************************************************	 
 */
 
#ifndef PORT_SECURITY_H_
#define PORT_SECURITY_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"             // SW configuration
#include "app_pairing.h"
#include "smpc.h"
#include "ke_msg.h"

/**
 ****************************************************************************************
 * \brief
 *
 * \param[in]   conidx
 * \param       env
 ****************************************************************************************
 */
void port_security_send_ltk_exch_rsp(uint8_t conidx, struct app_pairing_env_tag *env);

/**
 ****************************************************************************************
 * \brief Provides the encryption confirmation to the host
 *
 * \param[in] conidx        Connection Id number
 * \param[in] env           Authentication requirements (@see gap_auth)
 * \param[in] accept        Accept(true) or reject(false) the encryption request
 ****************************************************************************************
 */
void port_security_send_encrypt_cfm(uint8_t conidx, struct app_pairing_env_tag *env, bool accept);

/**
 ****************************************************************************************
 * \brief   Prepares and sends the reply to the GAPC_PAIRING_REQ msg
 *
 * \param[in] accepted  true if pairing request is accepted
 ****************************************************************************************
 */
void port_security_send_pairing_rsp(bool accepted);

/**
 ****************************************************************************************
 * \brief
 *
 * \param[in]   msgid
 * \param[in]   param
 * \param[in]   conidx
 *
 * \return
 ****************************************************************************************
 */
enum process_event_response port_security_process_handler(ke_msg_id_t const msgid, void const *param, uint8_t conidx);

#endif // PORT_SECURITY_H_

/**
 * \}
 * \}
 * \}
 */
