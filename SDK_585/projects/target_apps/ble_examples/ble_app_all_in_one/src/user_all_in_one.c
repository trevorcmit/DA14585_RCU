/*****************************************************************************************
 *
 * @file user_all_in_one.c
 *
 * @brief All in one project source code.
 *
 * Copyright (C) 2015-2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
******************************************************************************************/

/*****************************************************************************************
 * @addtogroup APP
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include "rwip_config.h"
#include "gap.h"
#include "app_easy_timer.h"
#include "app_bond_db.h"
#include "app_prf_perm_types.h"
#include "gpio.h"
#include "user_all_in_one.h"
#include "user_custs1_def.h"
#include "user_custs1_impl.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"
#include "app_easy_security.h"
#include "app_task.h"
#include "co_bt.h"

#if (BLE_SUOTA_RECEIVER)
#include "app_suotar.h"
#if (!SUOTAR_SPI_DISABLE)
#include "spi_flash.h"
#endif
#endif

/*
 * TYPE DEFINITIONS
******************************************************************************************/

// Manufacturer Specific Data ADV structure type
struct mnf_specific_data_ad_structure
{
    uint8_t ad_structure_size;
    uint8_t ad_structure_type;
    uint8_t company_id[APP_AD_MSD_COMPANY_ID_LEN];
    uint8_t proprietary_data[APP_AD_MSD_DATA_LEN];
};

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

uint8_t app_connection_idx                    __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY
timer_hnd app_adv_data_update_timer_used      __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY
timer_hnd app_param_update_request_timer_used __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY

// Retained variables
// Manufacturer Specific Data
struct mnf_specific_data_ad_structure mnf_data  __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
// Index of manufacturer data in advertising data or scan response data (when MSB is 1)
uint8_t mnf_data_index                          __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
uint8_t stored_adv_data_len                     __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data_len                __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
uint8_t stored_adv_data[ADV_DATA_LEN]           __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY
uint8_t stored_scan_rsp_data[SCAN_RSP_DATA_LEN] __attribute__((section("retention_mem_area0"), zero_init)); //@RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/

/*****************************************************************************************
 * @brief Initialize Manufacturer Specific Data
 * @return void
******************************************************************************************/
static void mnf_data_init()
{
    mnf_data.ad_structure_size = sizeof(struct mnf_specific_data_ad_structure ) - sizeof(uint8_t); // minus the size of the ad_structure_size field
    mnf_data.ad_structure_type = GAP_AD_TYPE_MANU_SPECIFIC_DATA;
    mnf_data.company_id[0] = APP_AD_MSD_COMPANY_ID & 0xFF; // LSB
    mnf_data.company_id[1] = (APP_AD_MSD_COMPANY_ID >> 8 )& 0xFF; // MSB
    mnf_data.proprietary_data[0] = 0;
    mnf_data.proprietary_data[1] = 0;
}

/*****************************************************************************************
 * @brief Update Manufacturer Specific Data
 * @return void
******************************************************************************************/
static void mnf_data_update()
{
    uint16_t data;

    data = mnf_data.proprietary_data[0] | (mnf_data.proprietary_data[1] << 8);
    data += 1;
    mnf_data.proprietary_data[0] = data & 0xFF;
    mnf_data.proprietary_data[1] = (data >> 8) & 0xFF;

    if (data == 0xFFFF) {
         mnf_data.proprietary_data[0] = 0;
         mnf_data.proprietary_data[1] = 0;
    }
}

/*****************************************************************************************
 * @brief Add an AD structure in the Advertising or Scan Response Data of the 
  *       GAPM_START_ADVERTISE_CMD parameter struct.
 * @param[in] cmd               GAPM_START_ADVERTISE_CMD parameter struct
 * @param[in] ad_struct_data    AD structure buffer
 * @param[in] ad_struct_len     AD structure length
 * @param[in] adv_connectable   Connectable advertising event or not. It controls whether 
 *                              the advertising data use the full 31 bytes length or only 
 *                              28 bytes (Document CCSv6 - Part 1.3 Flags). 
 * @return void
 */
static void app_add_ad_struct(struct gapm_start_advertise_cmd *cmd, void *ad_struct_data, uint8_t ad_struct_len, uint8_t adv_connectable)
{
    uint8_t adv_data_max_size = (adv_connectable) ? (ADV_DATA_LEN - 3) : (ADV_DATA_LEN);
    
    if ((adv_data_max_size - cmd->info.host.adv_data_len) >= ad_struct_len)
    {
        // Append manufacturer data to advertising data
        memcpy(&cmd->info.host.adv_data[cmd->info.host.adv_data_len], ad_struct_data, ad_struct_len);

        // Update Advertising Data Length
        cmd->info.host.adv_data_len += ad_struct_len;
        
        // Store index of manufacturer data which are included in the advertising data
        mnf_data_index = cmd->info.host.adv_data_len - sizeof(struct mnf_specific_data_ad_structure);
    }
    else if ((SCAN_RSP_DATA_LEN - cmd->info.host.scan_rsp_data_len) >= ad_struct_len)
    {
        // Append manufacturer data to scan response data
        memcpy(&cmd->info.host.scan_rsp_data[cmd->info.host.scan_rsp_data_len], ad_struct_data, ad_struct_len);

        // Update Scan Response Data Length
        cmd->info.host.scan_rsp_data_len += ad_struct_len;
        
        // Store index of manufacturer data which are included in the scan response data
        mnf_data_index = cmd->info.host.scan_rsp_data_len - sizeof(struct mnf_specific_data_ad_structure);
        // Mark that manufacturer data is in scan response and not advertising data
        mnf_data_index |= 0x80;
    }
    else
    {
        // Manufacturer Specific Data do not fit in either Advertising Data or Scan Response Data
        ASSERT_WARNING(0);
    }
    // Store advertising data length
    stored_adv_data_len = cmd->info.host.adv_data_len;
    // Store advertising data
    memcpy(stored_adv_data, cmd->info.host.adv_data, stored_adv_data_len);
    // Store scan response data length
    stored_scan_rsp_data_len = cmd->info.host.scan_rsp_data_len;
    // Store scan_response data
    memcpy(stored_scan_rsp_data, cmd->info.host.scan_rsp_data, stored_scan_rsp_data_len);
}

/*****************************************************************************************
 * @brief Advertisement data update timer callback function.
 * @return void
 ****************************************************************************************
*/
static void adv_data_update_timer_cb()
{
    // If mnd_data_index has MSB set, manufacturer data is stored in scan response
    uint8_t *mnf_data_storage = mnf_data_index & 0x80 ? stored_scan_rsp_data : stored_adv_data;

    // Update manufacturer data
    mnf_data_update();

    // Update the selected fields of the scan response data (manufacturer data)
    memcpy(mnf_data_storage + (mnf_data_index & 0x7F), &mnf_data, sizeof(struct mnf_specific_data_ad_structure));

    // Update advertising data on the fly
    app_easy_gap_update_adv_data(stored_adv_data, stored_adv_data_len, stored_scan_rsp_data, stored_scan_rsp_data_len);
    
    // Stop advertising air operation - a button press will wake-up the system
    app_easy_gap_advertise_stop();
}

/*****************************************************************************************
 * @brief Parameter update request timer callback function.
 * @return void
 ****************************************************************************************
*/
static void param_update_request_timer_cb()
{
    app_easy_gap_param_update_start(app_connection_idx);
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
}

/*****************************************************************************************
 * @brief Application wakeup callback function. Registered in API message utility.
 * @return void
 ****************************************************************************************
*/
static void app_wakeup_cb(void)
{
    // If state is not idle, ignore the message
    if (ke_state_get(TASK_APP) == APP_CONNECTABLE)
    {
        user_app_adv_start();
    }
}

/*****************************************************************************************
 * @brief Button press callback function. Registered in WKUPCT driver.
 * @return void
******************************************************************************************/
static void app_button_press_cb(void)
{
    if (GetBits16(SYS_STAT_REG, PER_IS_DOWN))
    {
        periph_init();
    }

    if (arch_ble_ext_wakeup_get())
    {
        arch_set_sleep_mode(app_default_sleep_mode);
        arch_ble_force_wakeup();
        arch_ble_ext_wakeup_off();
        app_easy_wakeup();
    }
}

void user_app_init(void)
{
    app_prf_srv_perm_t svc_perm = SRV_PERM_ENABLE;
    app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    
    // Initialize Manufacturer Specific Data
    mnf_data_init();
    
    // Initialize Advertising and Scan Response Data
    memcpy(stored_adv_data, USER_ADVERTISE_DATA, USER_ADVERTISE_DATA_LEN);
    stored_adv_data_len = USER_ADVERTISE_DATA_LEN;
    memcpy(stored_scan_rsp_data, USER_ADVERTISE_SCAN_RESPONSE_DATA, USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN);
    stored_scan_rsp_data_len = USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN;
    
    default_app_on_init();

    // Set custom service permission depending on the currently selected preset
    #if defined (USER_CFG_PAIR_METHOD_JUST_WORKS)
    svc_perm = SRV_PERM_UNAUTH;

    #elif defined (USER_CFG_PAIR_METHOD_PASSKEY)
    svc_perm = SRV_PERM_AUTH;

    #elif defined (USER_CFG_PAIR_METHOD_OOB)
    svc_perm = SRV_PERM_AUTH;

    #else
    svc_perm = SRV_PERM_ENABLE;
    #endif

    app_set_prf_srv_perm(TASK_ID_CUSTS1, svc_perm);
#if (BLE_SUOTA_RECEIVER)
    app_set_prf_srv_perm(TASK_ID_SUOTAR, svc_perm);
#endif

    // Fetch bond data from the external memory
    bond_db_init();
}

void user_app_adv_start(void)
{
    // Schedule the next advertising data update
    app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);
    
    struct gapm_start_advertise_cmd* cmd;
    cmd = app_easy_gap_undirected_advertise_get_active();
    
    // Add manufacturer data to initial advertising or scan response data, if there is enough space
    app_add_ad_struct(cmd, &mnf_data, sizeof(struct mnf_specific_data_ad_structure), 1);

    // Set extended sleep with OTP copy during advertising
    arch_set_extended_sleep(true);

    app_easy_gap_undirected_advertise_start();
}

#if (BLE_SUOTA_RECEIVER)
void on_suotar_status_change( const uint8_t suotar_event)
{
#if (!SUOTAR_SPI_DISABLE)
    int8_t man_dev_id = 0;

    man_dev_id = spi_flash_enable(SPI_EN_GPIO_PORT, SPI_EN_GPIO_PIN);
    if (man_dev_id == SPI_FLASH_AUTO_DETECT_NOT_DETECTED)
    {
        // The device was not identified. The default parameters are used.
        // Alternatively, an error can be asserted here.
        spi_flash_init(SPI_FLASH_DEFAULT_SIZE, SPI_FLASH_DEFAULT_PAGE);
    }

    if( suotar_event == SUOTAR_END )
    {
        // Power down SPI Flash
        spi_flash_power_down();
    }
#endif
}
#endif // (BLE_SUOTA_RECEIVER)

void user_app_connection(uint8_t connection_idx, struct gapc_connection_req_ind const *param)
{
    default_app_on_connection(connection_idx, param);

    if (app_env[connection_idx].conidx != GAP_INVALID_CONIDX)
    {
        app_connection_idx = connection_idx;

        // Stop the advertising data update timer
        app_easy_timer_cancel(app_adv_data_update_timer_used);

        // Check if the parameters of the established connection are the preferred ones.
        // If not then schedule a connection parameter update request.
        if ((param->con_interval < user_connection_param_conf.intv_min) ||
            (param->con_interval > user_connection_param_conf.intv_max) ||
            (param->con_latency != user_connection_param_conf.latency) ||
            (param->sup_to != user_connection_param_conf.time_out))
        {
            // Connection params are not these that we expect
            app_param_update_request_timer_used = app_easy_timer(APP_PARAM_UPDATE_REQUEST_TO, param_update_request_timer_cb);
        }
        user_app_enable_periphs();

        // Set extended sleep without OTP copy during connection
        arch_set_extended_sleep(false);
    }
}

void user_app_disconnect(struct gapc_disconnect_ind const *param)
{
    // Cancel the parameter update request timer
    if (app_param_update_request_timer_used != EASY_TIMER_INVALID_TIMER)
    {
        app_easy_timer_cancel(app_param_update_request_timer_used);
        app_param_update_request_timer_used = EASY_TIMER_INVALID_TIMER;
    }

    user_app_disable_periphs();
    // Update manufacturer data for the next advertsing event
    mnf_data_update();
    user_app_adv_start();

#if (BLE_SUOTA_RECEIVER)
    // Issue a platform reset when it is requested by the suotar procedure
    if (suota_state.reboot_requested)
    {
        // Reboot request will be served
        suota_state.reboot_requested = 0;

        // Platform reset
        platform_reset(RESET_AFTER_SUOTA_UPDATE);
    }
#endif
}

#if (BLE_APP_SEC)
void user_app_on_tk_exch_nomitm(uint8_t connection_idx,
                                    struct gapc_bond_req_ind const * param)
{
#if defined (USER_CFG_PAIR_METHOD_JUST_WORKS) || defined (USER_CFG_PAIR_METHOD_PASSKEY) || defined (USER_CFG_PAIR_METHOD_OOB)
    if (param->data.tk_type == GAP_TK_DISPLAY)
    {
        // By default we send hardcoded passkey
        uint32_t passkey = APP_SECURITY_MITM_PASSKEY_VAL;

        app_easy_security_tk_exch(connection_idx, (uint8_t*) &passkey, 4);
    }
    else if (param->data.tk_type == GAP_TK_OOB)
    {
        // By default we send hardcoded oob data
        uint8_t oob_tk[KEY_LEN] = APP_SECURITY_OOB_TK_VAL;

        app_easy_security_tk_exch(connection_idx, (uint8_t*) oob_tk, KEY_LEN);
    }
#else
    default_app_on_tk_exch_nomitm(connection_idx, param);
#endif
}

void user_app_on_encrypt_req_ind(uint8_t connection_idx,
                                    struct gapc_encrypt_req_ind const *param)
{
#if defined (USER_CFG_PAIR_METHOD_JUST_WORKS) || defined (USER_CFG_PAIR_METHOD_PASSKEY) || defined (USER_CFG_PAIR_METHOD_OOB)
    const struct bond_db_data *pbd;

    // Retrieve bond data and put it inside the app_sec_env
    pbd = bond_db_lookup_by_ediv(&param->rand_nb, param->ediv);
    if (pbd)
    {
        app_sec_env[connection_idx].auth = pbd->auth;                                               /// Authentication
        memcpy(&app_sec_env[connection_idx].ltk, &pbd->ltk.ltk, sizeof(struct gap_sec_key));        /// Long Term Key
        app_sec_env[connection_idx].ediv = pbd->ltk.ediv;                                           /// Encryption Diversifier
        memcpy(&app_sec_env[connection_idx].rand_nb, &pbd->ltk.randnb, RAND_NB_LEN);                /// Random Number
        app_sec_env[connection_idx].key_size = pbd->ltk.key_size;                                   /// Encryption key size (7 to 16)
        memcpy(&app_sec_env[connection_idx].irk, &pbd->irk, sizeof(struct gapc_irk));               /// IRK
    }
#endif
    // Validate bond data in app_sec_env
    default_app_on_encrypt_req_ind(connection_idx, param);
}

void user_app_on_pairing_succeeded(void)
{
    struct bond_db_data bd;

    if (app_sec_env[app_connection_idx].auth & GAP_AUTH_BOND)
    {
        memset(&bd, 0, sizeof(bd));

        // store bond data
        bd.valid = BOND_DB_VALID_ENTRY;

        memcpy(&bd.ltk.ltk, &app_sec_env[app_connection_idx].ltk, sizeof(struct gap_sec_key));  /// Long Term Key
        bd.ltk.ediv = app_sec_env[app_connection_idx].ediv;                                     /// Encryption Diversifier
        memcpy(&bd.ltk.randnb, &app_sec_env[app_connection_idx].rand_nb, RAND_NB_LEN);          /// Random Number
        bd.ltk.key_size = app_sec_env[app_connection_idx].key_size;                             /// Encryption key size (7 to 16)

        // Check bits for resolvable address type
        if ((app_sec_env[app_connection_idx].peer_addr_type == ADDR_RAND) &&
            (app_sec_env[app_connection_idx].peer_addr.addr[BD_ADDR_LEN-1] & GAP_RSLV_ADDR))
        {
            memcpy(&bd.irk, &app_sec_env[app_connection_idx].irk, sizeof(struct gapc_irk));     /// IRK
            bd.flags |= BOND_DB_ENTRY_IRK_PRESENT;
        }

        // Remote address
        bd.bdaddr.addr_type = app_sec_env[app_connection_idx].peer_addr_type;
        memcpy(&bd.bdaddr.addr, &app_sec_env[app_connection_idx].peer_addr, sizeof(struct bd_addr));

        bd.auth = app_sec_env[app_connection_idx].auth;                                         // authentication level

        bond_db_store(&bd);
    }
}
#endif // BLE_APP_SEC

void user_catch_rest_hndl(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case CUSTS1_VAL_WRITE_IND:
        {
            struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)(param);

            switch (msg_param->handle)
            {
                case CUST1_IDX_CONTROL_POINT_VAL:
                   user_custs1_ctrl_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                   break;

                case CUST1_IDX_LED_STATE_VAL:
                    user_custs1_led_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_ADC_VAL_1_NTF_CFG:
                    user_custs1_adc_val_1_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_BUTTON_STATE_NTF_CFG:
                    user_custs1_button_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_INDICATEABLE_IND_CFG:
                    user_custs1_long_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_LONG_VALUE_NTF_CFG:
                    user_custs1_long_val_cfg_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                case CUST1_IDX_LONG_VALUE_VAL:
                    user_custs1_long_val_wr_ind_handler(msgid, msg_param, dest_id, src_id);
                    break;

                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_NTF_CFM:
        {
            struct custs1_val_ntf_cfm const *msg_param = (struct custs1_val_ntf_cfm const *)(param);

            switch (msg_param->handle)
            {
                case CUST1_IDX_ADC_VAL_1_VAL:
                    break;

                case CUST1_IDX_BUTTON_STATE_VAL:
                    break;

                case CUST1_IDX_LONG_VALUE_VAL:
                    break;

                default:
                    break;
            }
        } break;

        case CUSTS1_VAL_IND_CFM:
        {
            struct custs1_val_ind_cfm const *msg_param = (struct custs1_val_ind_cfm const *)(param);

            switch (msg_param->handle)
            {
                case CUST1_IDX_INDICATEABLE_VAL:
                    break;

                default:
                    break;
             }
         } break;

        case GAPC_PARAM_UPDATED_IND:
        {
            // Cast the "param" pointer to the appropriate message structure
            struct gapc_param_updated_ind const *msg_param = (struct gapc_param_updated_ind const *)(param);

            // Check if updated Conn Params filled to preferred ones
            if ((msg_param->con_interval >= user_connection_param_conf.intv_min) &&
                (msg_param->con_interval <= user_connection_param_conf.intv_max) &&
                (msg_param->con_latency == user_connection_param_conf.latency) &&
                (msg_param->sup_to == user_connection_param_conf.time_out))
            {
            }
        } break;

        default:
            break;
    }
}

/*****************************************************************************************
 * @brief Sets button as wakeup trigger
 * @return void
 ****************************************************************************************
*/
static void app_button_enable(void)
{
    app_easy_wakeup_set(app_wakeup_cb);
    wkupct_register_callback(app_button_press_cb);
    wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // select pin (GPIO_BUTTON_PORT, GPIO_BUTTON_PIN)
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, WKUPCT_PIN_POLARITY_LOW), // polarity low
                                          1, // 1 event
                                          40); // debouncing time = 0
}

void user_app_adv_undirect_complete(const uint8_t status)
{
    // Disable wakeup for BLE and timer events. Only external (GPIO) wakeup events can wakeup processor.
    if (status == GAP_ERR_CANCELED)
    {
        arch_ble_ext_wakeup_on();

        // Configure wakeup button
        app_button_enable();
    }
}

/// @} APP
