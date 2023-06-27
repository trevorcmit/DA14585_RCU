/*****************************************************************************************
 *
 * @file user_custs1_def.c
 *
 * @brief Custom1 Server (CUSTS1) profile database definitions.
  *
******************************************************************************************/

/*****************************************************************************************
 * @defgroup USER_CONFIG
 * @ingroup USER
 * @brief Custom1 Server (CUSTS1) profile database definitions.
 *
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include <stdint.h>
#include "prf_types.h"
#include "attm_db_128.h"
#include "user_custs1_def.h"

/*
 * LOCAL VARIABLE DEFINITIONS
******************************************************************************************/

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

/// Full CUSTOM1 Database Description - Used to add attributes into the database
struct attm_desc_128 custs1_att_db[CUST1_IDX_NB] = {NULL};

/// @} USER_CONFIG
