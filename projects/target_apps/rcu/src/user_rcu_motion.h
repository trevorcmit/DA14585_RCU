/*****************************************************************************************
 *
 * \file user_rcu_motion.h
 *
 * \brief RCU motion function implementation header file.
 * 
******************************************************************************************/

#ifndef _USER_RCU_MOTION_H_
#define _USER_RCU_MOTION_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_APP
 * \{
 * \addtogroup APP_RCU_MOTION
 *
 * \brief Motion application
 *
 * \{
******************************************************************************************/

/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * \brief Get the handles of the characteristics used for motion notifications. This 
 *        Function must be called from user_on_db_init_complete().
******************************************************************************************/
void user_motion_get_handles(void);

#ifdef HAS_MOTION

/*****************************************************************************************
 * \brief Start motion
******************************************************************************************/
void user_motion_start(void);

/*****************************************************************************************
 * \brief Stop motion
******************************************************************************************/
void user_motion_stop(void);

/*****************************************************************************************
 * \brief   Process motion tasks while BLE is powered
 * \return  true device sleep must be blocked
******************************************************************************************/
bool user_motion_on_ble_powered(void);

#endif
    
#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)
/*****************************************************************************************
 * \brief   Process mouse and trackpad while BLE is powered
 * \return  true device sleep must be blocked
******************************************************************************************/
bool user_mouse_trackpad_on_ble_powered(void);

/*****************************************************************************************
 * \brief Create and send a mouse or trackpad report. The report includes delta X and Y
 *        cursor position data and mouse key status or trackpad tap information. 
 *
 * \param[in] full_release If true send a full release report having all keys declared
 *            as not pressed
 *
 * \return    True if report has been sent successfully
******************************************************************************************/
bool user_mouse_send_report(bool full_release);

#endif    

#if defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)
/*****************************************************************************************
 * \brief Process touchpad tasks while system is powered
 *
 * \return  APP_GOTO_SLEEP if the system can go to sleep
 *          APP_KEEP_POWERED if the system must remain active
 *          APP_BLE_WAKEUP if the BLE must be woken up
******************************************************************************************/
uint8_t user_touchpad_on_system_powered(void);


/*****************************************************************************************
 * \brief This function will be called to notify the application that a special event
 *        occurred in the trackpad
******************************************************************************************/
void user_touchpad_on_special_event(void *info);
#endif

#ifdef HAS_TOUCHPAD_TRACKPAD
/*****************************************************************************************
 * \brief This function will be called to notify the application that a tracking event
 *        occurred in the trackpad
******************************************************************************************/
void user_touchpad_on_tracking_event(void *info);
#endif

#ifdef HAS_MOUSE
/*****************************************************************************************
 * \brief Process mouse tasks while system is powered
 *
 * \return  APP_GOTO_SLEEP if the system can go to sleep
 *          APP_KEEP_POWERED if the system must remain active
 *          APP_BLE_WAKEUP if the BLE must be woken up
******************************************************************************************/
uint8_t user_mouse_on_system_powered(void);

/*****************************************************************************************
 * \brief This callback function is called from the systick timer to perform the mouse
 *        sensor polling
******************************************************************************************/
void user_systick_mouse_callback(void);

#endif    

/**
 * \}
 * \}
 * \}
 */

#endif // _USER_RCU_MOTION_H_
