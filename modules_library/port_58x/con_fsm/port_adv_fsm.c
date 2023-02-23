/**
 ****************************************************************************************
 *
 * \file port_adv_fsm.c
 *
 * \brief Advertise FSM module platform adaptation source file
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
 * \addtogroup PORT_ADV_FSM
 * \brief Advertising FSM HAL implementation
 * \{
 ****************************************************************************************	 
 */
 
#include "port_adv_fsm.h"
#include "app_adv_fsm.h"
#include "port_timer.h"

#ifdef HAS_CONNECTION_FSM

void port_adv_fsm_start_adv_undirected(start_adv_data_t *data)
{   
    // Allocate a message for GAP
    struct gapm_start_advertise_cmd *cmd = KE_MSG_ALLOC(GAPM_START_ADVERTISE_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_start_advertise_cmd);
    #if (RWBLE_SW_VERSION_MAJOR >= 8)     
	cmd->op.addr_src = GAPM_STATIC_ADDR;
    #else
	cmd->op.addr_src = GAPM_PUBLIC_ADDR;
    #endif    

	
    cmd->op.code = GAPM_ADV_UNDIRECT;
    cmd->channel_map = ADV_ALL_CHNLS_EN;   
 
	cmd->info.host.mode = data->adv_params->adv_mode;
	cmd->intv_min = US_TO_BLESLOTS(data->adv_params->adv_int_min*1000);
	cmd->intv_max = US_TO_BLESLOTS(data->adv_params->adv_int_max*1000);

   	cmd->info.host.adv_filt_policy = data->filter_policy;
     
    /*-----------------------------------------------------------------------------------
     * Set the Advertising Data and the Scan Response Data
     *---------------------------------------------------------------------------------*/
    if (data->adv_data_length != 0) {
        memcpy(&cmd->info.host.adv_data[0], data->adv_data, data->adv_data_length);
        cmd->info.host.adv_data_len = data->adv_data_length;
    }
    if (data->scanrsp_data_length != 0) {
        memcpy(&cmd->info.host.scan_rsp_data[0], data->scanrsp_data, data->scanrsp_data_length);
        cmd->info.host.scan_rsp_data_len = data->scanrsp_data_length;
    }
    
    // Send the message
    ke_msg_send(cmd);

    // We are now connectable
    ke_state_set(TASK_APP, APP_CONNECTABLE);
}

void port_adv_fsm_start_adv_directed(start_adv_direct_data_t *data)
{
    // Allocate a message for GAP
    struct gapm_start_advertise_cmd *cmd = KE_MSG_ALLOC(GAPM_START_ADVERTISE_CMD,
                                                TASK_GAPM, TASK_APP,
                                                gapm_start_advertise_cmd);

    cmd->op.code = GAPM_ADV_DIRECT;
 
    #if (RWBLE_SW_VERSION_MAJOR >= 8)     
    cmd->op.addr_src = GAPM_STATIC_ADDR;
    #else
    cmd->op.addr_src = GAPM_PUBLIC_ADDR;
    #endif      
	
    cmd->channel_map = 7;
    cmd->intv_min = 1100;
    cmd->intv_max = 1100;
    cmd->info.direct.addr_type = data->peer_addr_type;
    cmd->info.direct.addr= *data->peer_addr;
    
    // Send the message
    ke_msg_send(cmd);

	// We are now connectable
	ke_state_set(TASK_APP, APP_CONNECTABLE);
}

void port_adv_fsm_adv_stop(void)
{
        app_easy_gap_advertise_stop();
}

void port_adv_fsm_adv_timer_handler(void)
{    
    dbg_puts(DBG_ADV_LVL, "  @*Undirected Advertising Timed-Out \r\n");
    app_adv_fsm_process_evt(UND_ADV_TIMED_OUT);
}

void port_adv_fsm_on_direct_complete(uint8_t status)
{
    if (status == GAP_ERR_TIMEOUT) {
            dbg_puts(DBG_ADV_LVL, "  @*Directed Advertising Completed \r\n");
            app_adv_fsm_process_evt(DIR_ADV_COMPLETED);
    }
    else if(status == GAP_ERR_NO_ERROR) {
            dbg_puts(DBG_ADV_LVL, "  @*Directed Advertising Interrupted \r\n");
            app_adv_fsm_process_evt(DIR_ADV_INTERRUPTED);
    }        
    else {
            ASSERT_ERROR(0); // unexpected error
    }
}

void port_adv_fsm_on_undirect_complete(uint8_t status)
{
        if (status == GAP_ERR_CANCELED) {
                dbg_puts(DBG_ADV_LVL, "  @*Undirected Advertising Completed \r\n");
                app_adv_fsm_process_evt(UND_ADV_COMPLETED);
        }
        else if(status == GAP_ERR_NO_ERROR) {
                dbg_puts(DBG_ADV_LVL, "  @*Undirected Advertising Interrupted \r\n");
                app_adv_fsm_process_evt(UND_ADV_INTERRUPTED);
        }
        else {
                ASSERT_ERROR(0); // unexpected error
        }
}

#endif

/**
 * \}
 * \}
 * \}
 */
