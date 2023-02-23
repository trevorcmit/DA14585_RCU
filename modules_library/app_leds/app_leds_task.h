/**
 ****************************************************************************************
 *
 * \file app_leds_task.h
 *
 * \brief Header file - LED task handlers.
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
 * \addtogroup LEDS
 * \{
 * \addtogroup APP_LEDS
 * \brief Definitions of LED task handlers
 * \{
 ****************************************************************************************	 	 
 */
#ifndef APP_LEDS_TASK_H_
#define APP_LEDS_TASK_H_ 
#ifdef HAS_LED_INDICATORS

/**
 ****************************************************************************************
 * \brief  Handler of the LED Timer
 *
 * \param[in]   msgid
 * \param[in]   param 
 * \param[in]   dest_id
 * \param[in]   src_id 
 *
 * \return  KE_MSG_CONSUMED
 ****************************************************************************************
 */
int app_led_timer_handler(ke_msg_id_t const msgid,
                          void const *param,
                          ke_task_id_t const dest_id,
                          ke_task_id_t const src_id);

/**
 ****************************************************************************************
 * \brief   Handler of a dummy TASK_APP msg sent to trigger the timer
 *
 * \param[in] msgid 
 * \param[in] param
 * \param[in] dest_id
 * \param[in] src_id
 *
 * \return  KE_MSG_CONSUMED
 *
 * \remarks A dummy message is sent to the TASK_APP to start the LED timers.
 *          This msg is put in the queue when the BLE is woken up. When the
 *          handler is called, it is certain that the BLE is running and 
 *          the timer may start.
 ****************************************************************************************
 */                                    
int app_led_msg_handler(ke_msg_id_t const msgid,
                                   void const *param,
                                   ke_task_id_t const dest_id,
                                   ke_task_id_t const src_id);

#endif // HAS_LED_INDICATORS
                                   
#endif // APP_LEDS_TASK_H_


/**
 * \}
 * \}
 * \}
 */