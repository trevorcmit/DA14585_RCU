/*****************************************************************************************
 *
 * @file app_easy_security.c
 *
 * @brief Application Security Entry Point
 *
******************************************************************************************/

/*****************************************************************************************
 * @addtogroup APP_SECURITY
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/
 
#include <stdlib.h>
#include "co_bt.h"
#include "gapc_task.h"          // GAP Controller Task API Definition
#include "app_api.h"            // Application task Definition
#include "app_easy_security.h"

/*
 * DEFINES
******************************************************************************************/

#define APP_EASY_SECURITY_MAX_CONNECTION APP_EASY_MAX_ACTIVE_CONNECTION

/*
 * LOCAL VARIABLE DEFINITIONS
******************************************************************************************/

static struct gapc_bond_cfm *gapc_bond_cfm_pairing_rsp[APP_EASY_SECURITY_MAX_CONNECTION] __attribute__((section("retention_mem_area0"),zero_init));
static struct gapc_bond_cfm *gapc_bond_cfm_tk_exch[APP_EASY_SECURITY_MAX_CONNECTION]     __attribute__((section("retention_mem_area0"),zero_init));
static struct gapc_bond_cfm *gapc_bond_cfm_csrk_exch[APP_EASY_SECURITY_MAX_CONNECTION]   __attribute__((section("retention_mem_area0"),zero_init));
static struct gapc_bond_cfm *gapc_bond_cfm_ltk_exch[APP_EASY_SECURITY_MAX_CONNECTION]    __attribute__((section("retention_mem_area0"),zero_init));
static struct gapc_encrypt_cfm *gapc_encrypt_cfm[APP_EASY_SECURITY_MAX_CONNECTION]       __attribute__((section("retention_mem_area0"),zero_init));
static struct gapc_security_cmd *gapc_security_req[APP_EASY_SECURITY_MAX_CONNECTION]     __attribute__((section("retention_mem_area0"),zero_init));

/*
 * FUNCTION DEFINITIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Creates a GAPC_BOND_CFM pairing feature response message and stores it into
 *        retention memory, if the message does not already exist.
 * @param[in] conidx        Connection index.
 * @return The pointer to the message.
******************************************************************************************/
static inline struct gapc_bond_cfm* pairing_rsp_create_msg(uint8_t conidx)
{
    // Allocate a message for GAP
    if (gapc_bond_cfm_pairing_rsp[conidx] == NULL)
    {
        struct gapc_bond_cfm* cfm = app_gapc_bond_cfm_pairing_rsp_msg_create(conidx);
        gapc_bond_cfm_pairing_rsp[conidx] = cfm;

        cfm->data.pairing_feat.iocap = user_security_conf.iocap;
        cfm->data.pairing_feat.oob = user_security_conf.oob;
        cfm->data.pairing_feat.auth = user_security_conf.auth;
        cfm->data.pairing_feat.sec_req = user_security_conf.sec_req;
        cfm->data.pairing_feat.key_size = user_security_conf.key_size;
        cfm->data.pairing_feat.ikey_dist = user_security_conf.ikey_dist;
        cfm->data.pairing_feat.rkey_dist = user_security_conf.rkey_dist;
    }
    return gapc_bond_cfm_pairing_rsp[conidx];
}

struct gapc_bond_cfm* app_easy_security_pairing_rsp_get_active(uint8_t conidx)
{
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    return pairing_rsp_create_msg(conidx);
}

void app_easy_security_send_pairing_rsp(uint8_t conidx)
{
    struct gapc_bond_cfm* cmd;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    cmd = pairing_rsp_create_msg(conidx);
    ke_msg_send(cmd);
    gapc_bond_cfm_pairing_rsp[conidx] = NULL;
}

/*****************************************************************************************
 * @brief Creates a GAPC_BOND_CFM temporary key (TK) exchange message and stores it into
 *        retention memory, if hte message does not already exist.
 * @param[in] conidx        Connection index.
 * @return The pointer to the message.
******************************************************************************************/
static inline struct gapc_bond_cfm* tk_exch_create_msg(uint8_t conidx)
{
    // Allocate a message for GAP
    if (gapc_bond_cfm_tk_exch[conidx] == NULL)
    {
        gapc_bond_cfm_tk_exch[conidx] = app_gapc_bond_cfm_tk_exch_msg_create(conidx);
    }
     return gapc_bond_cfm_tk_exch[conidx];
}

struct gapc_bond_cfm* app_easy_security_tk_get_active(uint8_t conidx)
{
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    return tk_exch_create_msg(conidx);
}

void app_easy_security_tk_exch(uint8_t conidx, uint8_t *key, uint8_t length)
{
    struct gapc_bond_cfm* cmd;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    cmd = tk_exch_create_msg(conidx);

    // Load the pass key or the OOB provided key to the TK member of the created GAPC_BOND_CFM message
    memset((void*)gapc_bond_cfm_tk_exch[conidx]->data.tk.key, 0, KEY_LEN);
    memcpy(gapc_bond_cfm_tk_exch[conidx]->data.tk.key, key, length * sizeof(uint8_t));

    ke_msg_send(cmd);
    gapc_bond_cfm_tk_exch[conidx] = NULL;
}

/*****************************************************************************************
 * @brief Creates a GAPC_BOND_CFM connection signature resolving key (CSRK) exchange
 *        message and stores it into retention memory, if the message does not already exist.
 * @param[in] connection_idx        Connection index.
 * @return The pointer to the message.
******************************************************************************************/
static inline struct gapc_bond_cfm* csrk_exch_create_msg(uint8_t conidx)
{
    // Allocate a message for GAP
    if (gapc_bond_cfm_csrk_exch[conidx] == NULL)
    {
        gapc_bond_cfm_csrk_exch[conidx] = app_gapc_bond_cfm_csrk_exch_msg_create(conidx);
        app_sec_gen_csrk(conidx);
        memcpy((void*)gapc_bond_cfm_csrk_exch[conidx]->data.csrk.key, app_sec_env[conidx].csrk.key, KEY_LEN);
    }
     return gapc_bond_cfm_csrk_exch[conidx];
}

struct gapc_bond_cfm* app_easy_security_csrk_get_active(uint8_t conidx)
{
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    return csrk_exch_create_msg(conidx);
}

void app_easy_security_csrk_exch(uint8_t conidx)
{
    struct gapc_bond_cfm* cfm;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    cfm = csrk_exch_create_msg(conidx);
    ke_msg_send(cfm);
    gapc_bond_cfm_csrk_exch[conidx] = NULL;
}

/*****************************************************************************************
 * @brief Creates a GAPC_BOND_CFM long term key (LTK) exchange message and stores it into
 *        retention memory, if the message does not already exist.
 * @param[in] connection_idx        Connection index.
 * @return The pointer to the message.
******************************************************************************************/
static inline struct gapc_bond_cfm* ltk_exch_create_msg(uint8_t conidx)
{
    // Allocate a message for GAP
    if (gapc_bond_cfm_ltk_exch[conidx] == NULL)
    {
        gapc_bond_cfm_ltk_exch[conidx] = app_gapc_bond_cfm_ltk_exch_msg_create(conidx);
    }
    return gapc_bond_cfm_ltk_exch[conidx];
}

struct gapc_bond_cfm* app_easy_security_ltk_exch_get_active(uint8_t conidx)
{
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    return ltk_exch_create_msg(conidx);
}

void app_easy_security_ltk_exch(uint8_t conidx)
{
    struct gapc_bond_cfm* cfm;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    cfm = ltk_exch_create_msg(conidx);
    ke_msg_send(cfm);
    gapc_bond_cfm_ltk_exch[conidx] = NULL;
}

void app_easy_security_set_ltk_exch_from_sec_env(uint8_t conidx)
{
    struct gapc_bond_cfm* cfm;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    cfm = ltk_exch_create_msg(conidx);
    cfm->data.ltk.key_size = app_sec_env[conidx].key_size;
    cfm->data.ltk.ediv = app_sec_env[conidx].ediv;

    memcpy(&(cfm->data.ltk.randnb), &(app_sec_env[conidx].rand_nb) , RAND_NB_LEN);
    memcpy(&(cfm->data.ltk.ltk), &(app_sec_env[conidx].ltk) , KEY_LEN);
}

void app_easy_security_set_ltk_exch(uint8_t conidx, uint8_t* long_term_key, uint8_t encryption_key_size,
                                            uint8_t* random_number, uint16_t encryption_diversifier)
{
    struct gapc_bond_cfm* cfm;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    cfm = ltk_exch_create_msg(conidx);
    cfm->data.ltk.key_size = encryption_key_size;
    cfm->data.ltk.ediv = encryption_diversifier;

    memcpy(&(cfm->data.ltk.randnb), random_number , RAND_NB_LEN);
    memcpy(&(cfm->data.ltk.ltk), long_term_key , KEY_LEN);
}

/*****************************************************************************************
 * @brief Creates a GAPC_ENCRYPT_CFM encryption information message and stores it into
          retention memory, if the message does not already exist.
 * @param[in] connection_idx Connection index.
 * @return The pointer to the message.
******************************************************************************************/
static inline struct gapc_encrypt_cfm* encrypt_cfm_create_msg(uint8_t conidx)
{
    // Allocate a message for GAP
    if (gapc_encrypt_cfm[conidx] == NULL)
    {
        gapc_encrypt_cfm[conidx] = app_gapc_encrypt_cfm_msg_create(conidx);
    }
    return gapc_encrypt_cfm[conidx];
}

void app_easy_security_encrypt_cfm(uint8_t conidx)
{
    struct gapc_encrypt_cfm* cfm;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    cfm = encrypt_cfm_create_msg(conidx);
    ke_msg_send(cfm);
    gapc_encrypt_cfm[conidx] = NULL;
}

struct gapc_encrypt_cfm* app_easy_security_encrypt_cfm_get_active( uint8_t conidx )
{
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    return encrypt_cfm_create_msg(conidx);
}

bool app_easy_security_validate_encrypt_req_against_env(uint8_t conidx, struct gapc_encrypt_req_ind const *param)
{
    if(((app_sec_env[conidx].auth & GAP_AUTH_BOND) != 0) &&
        (memcmp(&(app_sec_env[conidx].rand_nb), &(param->rand_nb), RAND_NB_LEN) == 0) &&
        (app_sec_env[conidx].ediv == param->ediv))
    {
        return true;
    }
    return false;
}

void app_easy_security_set_encrypt_req_valid(uint8_t conidx)
{
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    struct gapc_encrypt_cfm* cfm = encrypt_cfm_create_msg(conidx);
    cfm->found = true;
    cfm->key_size = app_sec_env[conidx].key_size;
    memcpy(&(cfm->ltk), &(app_sec_env[conidx].ltk), KEY_LEN);
}

void app_easy_security_set_encrypt_req_invalid(uint8_t conidx)
{
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    struct gapc_encrypt_cfm* cfm = encrypt_cfm_create_msg(conidx);
    cfm->found = false;
}

/*****************************************************************************************
 * @brief Creates a GAPC_SECURITY_CMD security request command message and stores it into
          retention memory, if the message does not already exist.
 * @param[in] connection_idx        Connection index.
 * @return The pointer to the message.
******************************************************************************************/
static inline struct gapc_security_cmd* security_request_create_msg(uint8_t conidx)
{
    // Allocate a message for GAP
    if (gapc_security_req[conidx] == NULL)
    {
        gapc_security_req[conidx] = app_gapc_security_request_msg_create(conidx, user_security_conf.auth);
    }
    return gapc_security_req[conidx];
}

struct gapc_security_cmd* app_easy_security_request_get_active(uint8_t conidx)
{
     ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
     return security_request_create_msg(conidx);
}

void app_easy_security_request(uint8_t conidx)
{
    struct gapc_security_cmd* req;
    ASSERT_WARNING(conidx < APP_EASY_SECURITY_MAX_CONNECTION);
    req = app_easy_security_request_get_active(conidx);
    ke_msg_send(req);
    gapc_security_req[conidx] = NULL;
}

/// @} APP_SECURITY
