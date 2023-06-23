/**
* \file  user_custs_config.c
* \brief Custom1/2 Server (CUSTS1/2) profile database structure and initialization.
*/

/**
* \addtogroup USER
* \{
* \addtogroup PROFILE
* \{
* \addtogroup APP_CUSTS1
* \{
*/

// INCLUDE FILES
#include "app_prf_types.h"
#include "app_customs.h"
#include <user_custs1_def.h>


// GLOBAL VARIABLE DEFINTIONS

/// Custom1 server function callback table
const struct cust_prf_func_callbacks cust_prf_funcs[] =
{
#if (BLE_CUSTOM1_SERVER)
    {   TASK_ID_CUSTS1,
        custs1_att_db,
        CUST1_IDX_NB,
        #if (BLE_APP_PRESENT)
        app_custs1_create_db,
        NULL, //app_custs1_enable
        #else
        NULL, NULL,
        #endif
        NULL, NULL
    },
#endif
    {TASK_ID_INVALID, NULL, 0, NULL, NULL, NULL, NULL},   // DO NOT MOVE. Must always be last
};

/**
 * \}
 * \}
 * \}
 */
