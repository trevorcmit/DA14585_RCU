/*****************************************************************************************
 *
 * @file rscpc.h
 *
 * @brief Header file - Running Speed and Cadence Status Profile - Collector Role.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 * $ Rev $
 *
******************************************************************************************/

#ifndef _RSCPC_H_
#define _RSCPC_H_

/*****************************************************************************************
 * @addtogroup RSCPC Running Speed and Cadence Profile Collector
 * @ingroup RSCP
 * @brief Running Speed and Cadence Profile Collector
 * @{
******************************************************************************************/

/*
 * DEFINES
******************************************************************************************/

/// Maximum number of Running Speed and Cadence Collector task instances
#define RSCPC_IDX_MAX        (BLE_CONNECTION_MAX)

/*
 * INCLUDE FILES
******************************************************************************************/

#include "rscp_common.h"

#if (BLE_RSC_COLLECTOR)

#include "ke_task.h"
#include "prf_types.h"
#include "prf_utils.h"

/*
 * ENUMERATIONS
******************************************************************************************/

/// Running Speed and Cadence Service Characteristic Descriptors
enum rscpc_rscs_descs
{
    /// RSC Measurement Char. - Client Characteristic Configuration
    RSCPC_DESC_RSC_MEAS_CL_CFG,
    /// SC Control Point Char. - Client Characteristic Configuration
    RSCPC_DESC_SC_CTNL_PT_CL_CFG,

    RSCPC_DESC_MAX,

    RSCPC_DESC_MASK = 0x10,
};

/// Internal codes for reading/writing a RSCS characteristic with one single request
enum rscpc_codes
{
    /// Notified RSC Measurement
    RSCPC_NTF_RSC_MEAS          = RSCP_RSCS_RSC_MEAS_CHAR,
    /// Read RSC Feature
    RSCPC_RD_RSC_FEAT           = RSCP_RSCS_RSC_FEAT_CHAR,
    /// Read Sensor Location
    RSCPC_RD_SENSOR_LOC         = RSCP_RSCS_SENSOR_LOC_CHAR,
    /// Indicated SC Control Point
    RSCPC_IND_SC_CTNL_PT        = RSCP_RSCS_SC_CTNL_PT_CHAR,

    /// Read/Write RSC Measurement Client Char. Configuration Descriptor
    RSCPC_RD_WR_RSC_MEAS_CFG    = (RSCPC_DESC_RSC_MEAS_CL_CFG   | RSCPC_DESC_MASK),
    /// Read SC Control Point Client Char. Configuration Descriptor
    RSCPC_RD_WR_SC_CTNL_PT_CFG  = (RSCPC_DESC_SC_CTNL_PT_CL_CFG | RSCPC_DESC_MASK),
};



/*
 * STRUCTURES
******************************************************************************************/

/**
 * Structure containing the characteristics handles, value handles and descriptors for
 * the Running Speed and Cadence Service
 */
struct rscpc_rscs_content
{
    /// Service info
    struct prf_svc svc;

    /// Characteristic info:
    ///  - RSC Measurement
    ///  - RSC Feature
    ///  - Sensor Location
    ///  - SC Control Point
    struct prf_char_inf chars[RSCP_RSCS_CHAR_MAX];

    /// Descriptor handles:
    ///  - RSC Measurement Client Cfg
    ///  - SC Control Point Client Cfg
    struct prf_char_desc_inf descs[RSCPC_DESC_MAX];
};

struct rscpc_cnx_env
{
    /// Current Operation
    void *operation;
    ///Last requested UUID(to keep track of the two services and char)
    uint16_t last_uuid_req;
    /// Counter used to check service uniqueness
    uint8_t nb_svc;
    /// Running Speed and Cadence Service Characteristics
    struct rscpc_rscs_content rscs;
};

/// Running Speed and Cadence Profile Collector environment variable
struct rscpc_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// Environment variable pointer for each connections
    struct rscpc_cnx_env* env[RSCPC_IDX_MAX];
    /// State of different task instances
    ke_state_t state[RSCPC_IDX_MAX];
};

/// Command Message Basic Structure
struct rscpc_cmd
{
    /// Operation Code
    uint8_t operation;

    /// MORE DATA
};

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

/// Pool of Running Speed and Cadence Profile Collector task environments.
extern struct rscpc_env_tag **rscpc_envs;

/*
 * GLOBAL FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Retrieve CSCP client profile interface
 *
 * @return CSCP client profile interface
******************************************************************************************/
const struct prf_task_cbs* rscpc_prf_itf_get(void);

/*****************************************************************************************
 * @brief Send Running Speed and Cadence ATT DB discovery results to RSCPC host.
******************************************************************************************/
void rscpc_enable_rsp_send(struct rscpc_env_tag *cscpc_env, uint8_t conidx, uint8_t status);

/*****************************************************************************************
 * @brief Send a RSCPC_CMP_EVT message when no connection exists (no environment)
******************************************************************************************/
void rscps_send_no_conn_cmp_evt(uint8_t src_id, uint8_t dest_id, uint8_t operation);

/*****************************************************************************************
 * @brief Send a RSCPC_CMP_EVT message to the task which enabled the profile
******************************************************************************************/
void rscpc_send_cmp_evt(struct rscpc_env_tag *rscpc_env, uint8_t conidx, uint8_t operation, uint8_t status);

#endif //(BLE_RSC_COLLECTOR)

/// @} RSCPC

#endif //(_RSCPC_H_)
