/*****************************************************************************************
 *
 * @file app_default_handlers.h
 *
 * @brief Application default handlers header file.
 *
******************************************************************************************/

#ifndef _APP_DEFAULT_HANDLERS_H_
#define _APP_DEFAULT_HANDLERS_H_

/*****************************************************************************************
 * @addtogroup APP
 * @ingroup
 *
 * @brief
 *
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include <stdio.h>
#include "gapc_task.h"

/*****************************************************************************************
 * @brief Possible advertise scenarios.
******************************************************************************************/
enum default_advertise_scenario{
    DEF_ADV_FOREVER,
    DEF_ADV_WITH_TIMEOUT};

/*****************************************************************************************
 * @brief Possible security request scenarios.
******************************************************************************************/
enum default_security_request_scenario{
    DEF_SEC_REQ_NEVER,
    DEF_SEC_REQ_ON_CONNECT};

/*****************************************************************************************
 * @brief Configuration options for the default_handlers.
******************************************************************************************/
struct default_handlers_configuration
{
    //Configure the advertise operation used by the default handlers
    enum default_advertise_scenario adv_scenario;

    //Configure the advertise period in case of DEF_ADV_WITH_TIMEOUT.
    //It is measured in timer units (10ms). Use MS_TO_TIMERUNITS macro to convert
    //from milliseconds (ms) to timer units.
    int16_t advertise_period;

    //Configure the security start operation of the default handlers
    //if the security is enabled (CFG_APP_SECURITY)
    enum default_security_request_scenario security_request_scenario;
};


/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Default function called on initialization event.
 * @return void
******************************************************************************************/
void default_app_on_init(void);

/*****************************************************************************************
 * @brief Default function called on connection event.
 * @param[in] conidx Connection Id number
 * @param[in] param          Pointer to GAPC_CONNECTION_REQ_IND message
 * @return void
******************************************************************************************/
void default_app_on_connection(uint8_t conidx, struct gapc_connection_req_ind const *param);

/*****************************************************************************************
 * @brief Default function called on disconnection event.
 * @param[in] param          Pointer to GAPC_DISCONNECT_IND message
 * @return void
******************************************************************************************/
void default_app_on_disconnect(struct gapc_disconnect_ind const *param);

/*****************************************************************************************
 * @brief Default function called on device configuration completion event.
 * @return void
******************************************************************************************/
void default_app_on_set_dev_config_complete(void);

/*****************************************************************************************
 * @brief Default function called on database initialization completion event.
 * @return void
******************************************************************************************/
void default_app_on_db_init_complete(void);

/*****************************************************************************************
 * @brief Default function called on pairing request event.
 * @param[in] conidx         Connection Id number
 * @param[in] param          Pointer to GAPC_BOND_REQ_IND message
 * @return void
******************************************************************************************/
void default_app_on_pairing_request(uint8_t conidx, struct gapc_bond_req_ind const *param);

/*****************************************************************************************
 * @brief Default function called on no-man-in-the-middle TK exchange event.
 * @param[in] conidx         Connection Id number
 * @param[in] param          Pointer to GAPC_BOND_REQ_IND message
 * @return void
******************************************************************************************/
void default_app_on_tk_exch_nomitm(uint8_t conidx, struct gapc_bond_req_ind const *param);

/*****************************************************************************************
 * @brief Default function called on CSRK exchange event.
 * @param[in] conidx         Connection Id number
 * @param[in] param          Pointer to GAPC_BOND_REQ_IND message
 * @return void
******************************************************************************************/
void default_app_on_csrk_exch(uint8_t conidx, struct gapc_bond_req_ind const *param);

/*****************************************************************************************
 * @brief Default function called on long term key exchange event.
 * @param[in] conidx         Connection Id number
 * @param[in] param          Pointer to GAPC_BOND_REQ_IND message
 * @return void
******************************************************************************************/
void default_app_on_ltk_exch(uint8_t conidx, struct gapc_bond_req_ind const *param);

/*****************************************************************************************
 * @brief Default function called on encryption request event.
 * @param[in] conidx         Connection Id number
 * @param[in] param          Pointer to GAPC_ENCRYPT_REQ_IND message
 * @return void
******************************************************************************************/
void default_app_on_encrypt_req_ind(uint8_t conidx, struct gapc_encrypt_req_ind const *param);

/*****************************************************************************************
 * @brief Default function called on advertising operation.
 * @return void
******************************************************************************************/
void default_advertise_operation(void);

/*****************************************************************************************
 * @brief Structure containing the operations used by the default handlers.
 * @return void
******************************************************************************************/
struct default_app_operations {
    void (*default_operation_adv)(void);
};

/*****************************************************************************************
 * @brief Default function called on device appearance read request from peer.
 * @param[in|out] appearance         The appearance value returned.
 * @return void
******************************************************************************************/
void default_app_on_get_dev_appearance(uint16_t* appearance);

/*****************************************************************************************
 * @brief Default function called on slave preferred connection parameters read request 
 *        from peer.
 * @param[in|out] slv_params         The slave preferred connection parameters.
 * @return void
******************************************************************************************/
void default_app_on_get_dev_slv_pref_params(struct gap_slv_pref* slv_params);

/*****************************************************************************************
 * @brief Default function called on device info write request from peer.
 * @param[in] req                Requested information: 
 *                                  - GAPC_DEV_NAME: Device Name
 *                                  - GAPC_DEV_APPEARANCE: Device Appearance Icon
 * @param[in|out] status         Status code used to know if requested has been accepted 
 *                               or not.
 * @return void
******************************************************************************************/
void default_app_on_set_dev_info(uint8_t req, uint8_t* status);

/*****************************************************************************************
 * @brief Default function called on parameter update request indication.
 * @param[in] param             Pointer to a @gapc_param_update_req_ind message struct
 * @param[in|out] cfm           Pointer to a @gapc_param_update_cfm message struct
 * @return void
 * @note The application may accept or reject the received parameter update request, 
 *       depending on the received param values.
******************************************************************************************/
void default_app_update_params_request(struct gapc_param_update_req_ind const *param,
                                       struct gapc_param_update_cfm *cfm);

/// @} APP

#endif // _APP_DEFAULT_HANDLERS_H_
