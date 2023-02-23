/**
 *****************************************************************************************
 *
 * \file app_adv_fsm_defs.h
 *
 * \brief Advertising FSM module definitions
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 *****************************************************************************************
 */

 /**
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup BONDING
 * \{
 * \addtogroup APP_ADV_FSM
 *
 * \brief Advertising FSM definitions
 * \{
 ****************************************************************************************	 	 
 */
 
#ifndef APP_ADV_FSM_DEFS_H_
#define APP_ADV_FSM_DEFS_H_

#include <stdint.h>
#include <stdbool.h>

typedef void (*app_adv_fsm_notification_cb_t)( uint8_t notif );

typedef enum {
        ADV_SETTING_SPECIAL,
        ADV_SETTING_SLOW,     // do not change assigned values
        ADV_SETTING_UNDIRECTED_LIM,
        ADV_SETTING_UNDIRECTED,
        ADV_SETTING_UNDIRECTED_NO_PAIR,
        MAX_ADV_SETTINGS
} adv_settings_idx_t;

typedef struct {
        uint32_t discoverable_timeout;
        uint16_t adv_int_min;
        uint16_t adv_int_max;
        uint8_t  adv_mode;
        const uint8_t *adv_data;
        const uint8_t *scan_rsp_data;
        const uint8_t adv_data_length;
        const uint8_t scan_rsp_data_length;
} adv_params_t;

typedef enum {
    ADV_FSM_NO_NOTIFICATION,
	ADV_FSM_DIR_ADV_ENDED ,
	ADV_FSM_UND_ADV_ENDED  ,
    ADV_FSM_UND_ADV_LIM_ENDED,
    ADV_FSM_UND_ADV_STARTED,
    ADV_FSM_DIR_ADV_STARTED,
    ADV_FSM_UND_ADV_LIM_STARTED,
    ADV_FSM_UND_ADV_SPECIAL_STARTED,
    ADV_FSM_UND_ADV_SPECIAL_ENDED
} adv_notification_types_t;

typedef enum
{
   ADV_NO_EVENT,
   UND_ADV_COMPLETED,
   DIR_ADV_COMPLETED,
   UND_ADV_TIMED_OUT,
   DIR_ADV_INTERRUPTED, 
   UND_ADV_INTERRUPTED,
   START_ADV,
   START_ADV_LIM,
   START_ADV_NO_PAIR,
   START_ADV_DIR,
   STOP_ADV,
   START_ADV_SPECIAL
} adv_fsm_events_t;

typedef struct start_adv_data_s {
	uint8_t filter_policy;
	uint8_t adv_data_length;	
	uint8_t scanrsp_data_length;
	const adv_params_t *adv_params; 
	struct bd_addr *addr;
	const uint8_t *adv_data;
	const uint8_t *scanrsp_data;
} start_adv_data_t;

typedef struct start_adv_direct_data_s {
	struct bd_addr *addr;
	uint8_t peer_addr_type;
	struct bd_addr *peer_addr;
} start_adv_direct_data_t;

typedef struct {
    app_adv_fsm_notification_cb_t app_adv_notification_callback;
    bool disable_undirected_advertise;
    uint8_t directed_advertising_repeats;
    bool disable_advertising_timeout;
    bool force_new_advertising_start;
    adv_params_t adv_params[MAX_ADV_SETTINGS];
} adv_configuration_t;

typedef void (*app_adv_fsm_start_undir_adv_t)(start_adv_data_t *dat);
typedef void (*app_adv_fsm_start_dir_adv_t)(start_adv_direct_data_t *dat);
typedef void (*app_adv_fsm_stop_adv)(void);

typedef struct {
    app_adv_fsm_start_undir_adv_t adv_start_und_adv;
    app_adv_fsm_start_dir_adv_t adv_start_dir_adv;
    app_adv_fsm_stop_adv  adv_stop;
} adv_util_funcs_t;

#endif // APP_ADV_FSM_DEFS_H_

/**
 * \}
 * \}
 * \}
 */
