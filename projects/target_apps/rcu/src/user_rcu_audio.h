/****************************************************************************************
 * @file user_rcu_audio.h
 * @brief RCU Audio function implementation header file.
*****************************************************************************************/
#ifndef _USER_RCU_AUDIO_H_
#define _USER_RCU_AUDIO_H_

/****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_APP
 * \{
 * \addtogroup APP_RCU_AUDIO
 * \brief Audio application
 * \{
*****************************************************************************************/
 
/***************************************************************************************
 * INCLUDE FILES
****************************************************************************************/
#include "ke_msg.h"
#include "app_audio_defs.h"

#ifndef AUDIO_USE_CUSTOM_PROFILE
    #include "hogp_common.h"
    #include "hogpd_task.h"
#endif

/****************************************************************************************
 * FUNCTION DECLARATIONS
*****************************************************************************************/

/****************************************************************************************
 * \brief Process tasks when device is disconnected from host
*****************************************************************************************/
void user_audio_on_disconnect(void);


/****************************************************************************************
 * \brief Process tasks while BLE is powered
 * \return  true if system must remain active
*****************************************************************************************/
bool user_audio_on_ble_powered(void);


/*****************************************************************************************
 * \brief Process tasks while system is powered
 *
 * \return  APP_GOTO_SLEEP if the system can go to sleep
 *          APP_KEEP_POWERED if the system must remain active
 *          APP_BLE_WAKEUP if the BLE must be woken up
******************************************************************************************/
uint8_t user_audio_on_system_powered(void);

/*****************************************************************************************
 * \brief Set the audio packet size according to connection parameters, MTU and 
 *        BLE packet length
******************************************************************************************/
void user_audio_set_packet_size(void);

/*****************************************************************************************
 * \brief Get the handles of the characteristics used for audio notifications. This 
 *        Function mustbe called from user_on_db_init_complete().
******************************************************************************************/
void user_audio_get_handles(void);

/*****************************************************************************************
 * \brief Process the custom audio profile messages
 *
 * \param[in] msgid   Id of the message received
 * \param[in] param   Pointer to the parameters of the message
 * \return Returns if the message is handled by the process handler

******************************************************************************************/

enum process_event_response user_audio_process_custom_profile_handler(ke_msg_id_t 
                                                                      const msgid, 
                                                                      void const *param);

/*****************************************************************************************
 * \brief Send a CTRL_IN_CONN_PARAMS_REPORT packet notification
******************************************************************************************/  
void user_audio_send_conn_params_report(void);

/*****************************************************************************************
 * \brief
 *
 * \param[in] mode
 * \param[in] button_pressed
******************************************************************************************/
void user_audio_start(app_audio_adpcm_mode_t mode, bool button_pressed);

/*****************************************************************************************
 * \brief
 *
 * \param[in] force_stop
******************************************************************************************/
void user_audio_stop(bool force_stop);

/*****************************************************************************************
 * \brief Set the configuration data in the custom audio profile database
******************************************************************************************/
void user_audio_set_config_data(void);

/*****************************************************************************************
 * \brief Called when HID_REPORT has sent an HID keyboard report 
******************************************************************************************/
void user_audio_hid_send_report_cb(uint8_t report_idx, uint8_t *data, uint16_t length);

/*****************************************************************************************
 * \brief Called when audio samples are available
******************************************************************************************/
void user_audio_callback(void);

#ifndef AUDIO_USE_CUSTOM_PROFILE
void user_audio_write_ctrl_out_callback(uint8_t conidx, struct hogpd_report_info *report_info);
void user_audio_read_ctrl_in_callback(uint8_t conidx, uint8_t *report_data, uint16_t *report_length);
#endif

#ifdef AUDIO_TEST_MODE
void user_rcu_audio_start_audio_test(bool start);
void user_rcu_audio_test_mode_timer_handler(void);
#endif    

#if defined(HAS_AUDIO) && defined(ENABLE_OTA_DEBUG) && defined(HAS_KBD)
void user_rcu_audio_kbd_emul_timer_handler(void);
#endif

/**
 * \}
 * \}
 * \}
 */
 
#endif // _USER_RCU_AUDIO_H_
