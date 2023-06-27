/*****************************************************************************************
 *
 * \file user_pwr_mgr_config.h
 *
 * \brief Power management configuration file.
 * 
******************************************************************************************/

#ifndef _USER_PWR_MGR_CONFIG_H_
#define _USER_PWR_MGR_CONFIG_H_

#include "user_rcu.h"

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup APP_CONFIG
 * \{
 * \addtogroup PWR_MGR_CFG
 *
 * \brief Power manager configuration
 * \{
******************************************************************************************/

typedef struct {
    uint32_t inactivity_timeout;
    pwr_mgr_callback_t *inactivity_callback; 
} pwr_mgr_params_t;

static const pwr_mgr_params_t pwr_mgr_params={
/*****************************************************************************************
 * Idle time until action is requested in msec. Set to 0 to disable inactivity timeout
******************************************************************************************/
    .inactivity_timeout   = 300000,   
    .inactivity_callback  = user_pwr_mgr_inactivity_callback,
};

/**
 * \}
 * \}
 * \}
 */

#endif // _USER_PWR_MGR_CONFIG_H_
