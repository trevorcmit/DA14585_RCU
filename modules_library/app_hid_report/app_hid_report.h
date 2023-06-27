/*****************************************************************************************
 *
 * \file app_hid_report.h
 *
 * \brief This module provides an API for creating, modifying and sending HID reports 
 * using HOGP.
 *
 * Define symbol HAS_HID_REPORT to include this module in the application.
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup HID
 * \{
 * \addtogroup APP_HID
 * \{
******************************************************************************************/

#ifndef APP_HID_REPORT_H_
#define APP_HID_REPORT_H_

#include "stdint.h"
#include "stdbool.h"
#if (BLE_HID_DEVICE)
    #include <user_hogpd_config.h>
#endif

#include <app_hid_report_config.h>

enum KEY_BUFF_TYPE {
	FREE,
	PRESS,
	RELEASE,
	EXTENDED,
    CUSTOM
};

enum hid_report_status {
    HID_REPORT_FAILED,
    HID_REPORTS_AVAILABLE,                 
    HID_LAST_REPORT_SENT, 
    HID_REPORT_EMPTY       
};

/*****************************************************************************************
 * \brief Initializes the lists
******************************************************************************************/
void app_hid_report_init(void);

/*****************************************************************************************
 * \brief Process tasks while system is powered
 *
 * \return  APP_GOTO_SLEEP if the system can go to sleep
 *          APP_KEEP_POWERED if the system must remain active
 *          APP_BLE_WAKEUP if the BLE must be woken up
******************************************************************************************/
uint8_t app_hid_report_on_system_powered(void);

/*****************************************************************************************
 * \brief Clear the lists
******************************************************************************************/
void app_hid_report_clear_lists(void);

/*****************************************************************************************
 * \brief Check if report list contains any reports
 *
 * \return  true if the report list is empty.
******************************************************************************************/
bool app_hid_report_is_report_list_empty(void);

/*****************************************************************************************
 * \brief Check if report list is full
 *
 * \return  true if the report list is full
******************************************************************************************/
bool app_hid_report_is_report_list_full(void);

/*****************************************************************************************
 * \brief Get the Number of free report nodes in the FIFO.
 *
 * \return  The number of free report nodes
******************************************************************************************/
uint16_t app_hid_report_get_free_nodes(void);
    
/*****************************************************************************************
 * \brief Add a HID report in the FIFO
 *
 * \param   report_id The HID report id
 * \param   type      The report type (PRESS, RELEASE, or EXTENDED)
 * \param   data      Pointer to the report data
 * \param   size      report data size
 *
 * \return  A pointer to the report data if the report has been successfully added
 *          Otherwise NULL
******************************************************************************************/   
uint8_t* app_hid_report_add_report(uint8_t report_id, enum KEY_BUFF_TYPE type, uint8_t *data, uint16_t size);

/*****************************************************************************************
 * \brief Sends a Key Report Notification to the Host
 *
 * \return  HID_REPORT_FAILED     if HID report sending failed. There is at least one report in the buffer
 *          HID_REPORTS_AVAILABLE if HID report has been sent successfully and
 *                                there are more reports in the buffer
 *          HID_LAST_REPORT_SENT  if the last HID report in the buffer has been sent successfully
 *          HID_REPORT_EMPTY        if HID report buffer is empty.
******************************************************************************************/
enum hid_report_status app_hid_report_send_report(void);

/*****************************************************************************************
 * \brief Based on the type and the status od the key determine whether 
 *        1. a new press report needs to be allocated to report a key press
 *        2. a new release report needs to be allocated to report a key release or 
 *        3. a pending release report can be used for this purpose.
 *        Prepare the report and add it to the corresponding trm list. 
 *
 * \param[in]	keycode
 * \param[in] 	pressed
 *
 * \return  false, when full
 *          true, when done
******************************************************************************************/ 
bool app_hid_report_modify_kbd_keyreport(const uint16_t keycode, uint8_t pressed);

/*****************************************************************************************
 * \brief 
 *
 * \return
******************************************************************************************/ 
bool app_hid_report_add_full_release_report(void);  
  
  
/*****************************************************************************************
 * \brief Add a HID report in the FIFO
 *
 * \param   byte   The byte of the extended report to be modified
 * \param   mask   The mask of the bits that will be modified
 * \param   value  The value that will be assigned to the bits defined by mask
 *
 * \return  true if the report has been successfully added
******************************************************************************************/ 
bool app_hid_report_create_extended_report(uint8_t byte, uint8_t mask, uint8_t value);

#endif // APP_HID_REPORT_H_

/**
 * \}
 * \}
 * \}
 */
