/*****************************************************************************************
 *
 * \file user_hogpd_config.h
 *
 * \brief HOGPDD configuration file.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

#ifndef _USER_HOGPD_CONFIG_H_
#define _USER_HOGPD_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup PROFILE_CONFIG
 * \{
 * \addtogroup HOGPD_CFG
 *
 * \brief BLE HOGPD profile configuration
 * \{
******************************************************************************************/
 
#include "app_hogpd_defs.h"

#ifdef HAS_KBD
    #include "user_rcu_kbd.h"
#endif

#if defined(HAS_AUDIO) && !defined(AUDIO_USE_CUSTOM_PROFILE)
    #include "user_rcu_audio.h"
#endif

#ifdef HAS_CONNECTION_FSM
    #include "user_rcu.h"
#endif

static const hogpd_params_t hogpd_params = {
/*****************************************************************************************
 * Use BOOT MODE                                                                        
******************************************************************************************/
    .boot_protocol_mode   = false,
        
/*****************************************************************************************
 * Include BATT in HID                                                                  
******************************************************************************************/
    .batt_external_report = false,
        
/*****************************************************************************************
 * Set RemoteWakeup mode. Remote Host may not handle properly remote wakeup when the 
 * inactivity timeout is on. Some Hosts do not expect to receive LL_TERMINATE_IND from 
 * Wakeup capable devices while they are sleeping.
******************************************************************************************/
    .remote_wakeup        = false,
        
/*****************************************************************************************
 * Set normally connectable mode                                                                
******************************************************************************************/
#ifdef NORMALLY_CONNECTABLE    
    .normally_connectable = true,
#else
    .normally_connectable = false,
#endif  
  
/*****************************************************************************************
 * \brief Callback for storing CCC and attributes
******************************************************************************************/
#ifdef HAS_CONNECTION_FSM
    .store_attribute_callback = user_store_ccc,
#else
    .store_attribute_callback = NULL,
#endif
};

/*****************************************************************************************
 * HID report IDs and sizes
******************************************************************************************/
#if defined(HAS_KBD) || defined(HAS_TOUCHPAD_SLIDER)
    #define HID_KBD_NORMAL_REPORT_ID       0x01 // Normal keyboard report
    #define HID_KBD_NORMAL_REPORT_SIZE     8    // The size of the normal keyboard report. Must be 8
                                         
    #define HID_KBD_LED_REPORT_ID          0x02 // Keyboard LED report
    #define HID_KBD_LED_REPORT_SIZE        1    // The size of the keyboard LED report. Must be 1
                                         
    #define HID_KBD_EXTENDED_REPORT_ID     0x03 // Extended keyboard report
    #define HID_KBD_EXTENDED_REPORT_SIZE   3    // The size of the extended keyboard report
#endif    

#define HID_VENDOR_DEFINED_REPORT_SIZE     20   // The size of the vendor defined reports.

#if defined(HAS_AUDIO) && !defined(AUDIO_USE_CUSTOM_PROFILE)
	#define HID_STREAM_CTRL_OUT_REPORT_ID  0x04
	#define HID_STREAM_CTRL_IN_REPORT_ID   0x05
    #define HID_AUDIO_DATA_1_REPORT_ID     0x06
    #ifdef AUDIO_USE_THREE_HID_REPORTS
        #define HID_AUDIO_DATA_2_REPORT_ID 0x07
        #define HID_AUDIO_DATA_3_REPORT_ID 0x08
    #endif
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
        #define HID_VENDOR_DEFINED_AUDIO_REPORT_SIZE APP_STREAM_PACKET_SIZE
    #else
        #define HID_VENDOR_DEFINED_AUDIO_REPORT_SIZE (MAX_MTU_SIZE-3)
    #endif
#endif                                     
#ifdef HAS_MOTION                          
	#define HID_MOTION_DATA_REPORT_ID      0x09
#endif

#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
	#define HID_MOUSE_REPORT_ID            0x1A
    #define HID_MOUSE_REPORT_SIZE          9
#endif

/*****************************************************************************************
 * HID report indexes
******************************************************************************************/
enum hogpd_indexes {
#if defined(HAS_KBD) || defined(HAS_TOUCHPAD_SLIDER)
    HID_KBD_NORMAL_REPORT_IDX = 0,
    HID_KBD_LED_REPORT_IDX,
    HID_KBD_EXTENDED_REPORT_IDX,
#endif
#if defined(HAS_AUDIO) && !defined(AUDIO_USE_CUSTOM_PROFILE)
    HID_STREAM_CTRL_OUT_IDX,
	HID_STREAM_CTRL_IN_IDX,
	HID_AUDIO_DATA_1_IDX,
    #ifdef AUDIO_USE_THREE_HID_REPORTS
        HID_AUDIO_DATA_2_IDX,
        HID_AUDIO_DATA_3_IDX,
    #endif
#endif
#ifdef HAS_MOTION 
	HID_MOTION_DATA_IDX,
#endif
#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
    HID_MOUSE_IDX,
#endif    
    HID_NUM_OF_REPORTS // Don't remove this.
};

static const hogpd_reports_t hogpd_reports[HID_NUM_OF_REPORTS] = 
{
#if defined(HAS_KBD) || defined(HAS_TOUCHPAD_SLIDER)
    [HID_KBD_NORMAL_REPORT_IDX]  = {.id             = HID_KBD_NORMAL_REPORT_ID,           
                                    .size           = HID_KBD_NORMAL_REPORT_SIZE,     
                                    .cfg            = HOGPD_CFG_REPORT_IN | HOGPD_CFG_REPORT_WR, 
                                    .read_callback  = NULL,
                                    .write_callback = NULL},
                                  
    [HID_KBD_LED_REPORT_IDX]     = {.id             = HID_KBD_LED_REPORT_ID,           
                                    .size           = HID_KBD_LED_REPORT_SIZE,                              
                                    .cfg            = HOGPD_CFG_REPORT_OUT,   
                                    .read_callback  = NULL,
    #ifdef HAS_KBD
                                    .write_callback = user_keyboard_led_report_callback,
    #else
                                    .write_callback = NULL,
    #endif
                                   },

    [HID_KBD_EXTENDED_REPORT_IDX]= {.id             = HID_KBD_EXTENDED_REPORT_ID,           
                                    .size           = HID_KBD_EXTENDED_REPORT_SIZE,   
                                    .cfg            = HOGPD_CFG_REPORT_IN | HOGPD_CFG_REPORT_WR,                   
                                    .read_callback  = NULL,
                                    .write_callback = NULL},
#endif    
#if defined(HAS_AUDIO) && !defined(AUDIO_USE_CUSTOM_PROFILE)
    [HID_STREAM_CTRL_OUT_IDX]    = {.id             = HID_STREAM_CTRL_OUT_REPORT_ID, 
                                    .size           = HID_VENDOR_DEFINED_REPORT_SIZE, 
                                    .cfg            = HOGPD_CFG_REPORT_OUT,                      
                                    .read_callback  = NULL,
                                    .write_callback = user_audio_write_ctrl_out_callback
                                   },
    
    [HID_STREAM_CTRL_IN_IDX]     = {.id             = HID_STREAM_CTRL_IN_REPORT_ID,  
                                    .size           = HID_VENDOR_DEFINED_REPORT_SIZE,
                                    .cfg            = HOGPD_CFG_REPORT_IN, 
                                    .read_callback  = user_audio_read_ctrl_in_callback,
                                    .write_callback = NULL
                                   },
    [HID_AUDIO_DATA_1_IDX]       = {.id             = HID_AUDIO_DATA_1_REPORT_ID,    
                                    .size           = HID_VENDOR_DEFINED_AUDIO_REPORT_SIZE, 
                                    .cfg            = HOGPD_CFG_REPORT_IN, 
                                    .read_callback  = NULL,
                                    .write_callback = NULL
                                   },
    #ifdef AUDIO_USE_THREE_HID_REPORTS
    [HID_AUDIO_DATA_2_IDX]       = {.id             = HID_AUDIO_DATA_2_REPORT_ID,    
                                    .size           = HID_VENDOR_DEFINED_AUDIO_REPORT_SIZE, 
                                    .cfg            = HOGPD_CFG_REPORT_IN, 
                                    .read_callback  = NULL,
                                    .write_callback = NULL
                                   },
    
    [HID_AUDIO_DATA_3_IDX]       = {.id             = HID_AUDIO_DATA_3_REPORT_ID,    
                                    .size           = HID_VENDOR_DEFINED_AUDIO_REPORT_SIZE, 
                                    .cfg            = HOGPD_CFG_REPORT_IN, 
                                    .read_callback  = NULL,
                                    .write_callback = NULL
                                   },
   #endif
#endif    
#ifdef HAS_MOTION 
    [HID_MOTION_DATA_IDX]        = {.id             = HID_MOTION_DATA_REPORT_ID,     
                                    .size           = HID_VENDOR_DEFINED_REPORT_SIZE,  
                                    .cfg            = HOGPD_CFG_REPORT_IN, 
                                    .read_callback  = NULL,
                                    .write_callback = NULL
                                   },
#endif
#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
    [HID_MOUSE_IDX]              = {.id             = HID_MOUSE_REPORT_ID,     
                                    .size           = HID_MOUSE_REPORT_SIZE,  
                                    .cfg            = HOGPD_CFG_REPORT_IN | HOGPD_CFG_REPORT_WR, 
                                    .read_callback  = NULL,
                                    .write_callback = NULL
                                   },
#endif                                   
};

/*****************************************************************************************
 * Macros for keycode from/to byte/mask conversion
******************************************************************************************/
#define HOGP_CREATE_KEY_CODE(byte, bit) ((byte << 4) | bit)
#define HOGP_GET_BYTE_FROM_CODE(code)   ((code >> 4) & 0x0F)
#define HOGP_GET_MASK_FROM_CODE(code)   (1 << (code & 0x0F))

/*****************************************************************************************
 * Cunsumer key code definition
******************************************************************************************/
#define MUTE_KEY_BYTE      0
#define MUTE_KEY_BIT       5
#define MUTE_KEY_MASK      (1 << MUTE_KEY_BIT)
#define MUTE_KEY_CODE      HOGP_CREATE_KEY_CODE(MUTE_KEY_BYTE, MUTE_KEY_BIT)

#define VOL_PLUS_KEY_BYTE  0
#define VOL_PLUS_KEY_BIT   6
#define VOL_PLUS_KEY_MASK  (1 << VOL_PLUS_KEY_BIT)
#define VOL_PLUS_KEY_CODE  HOGP_CREATE_KEY_CODE(VOL_PLUS_KEY_BYTE, VOL_PLUS_KEY_BIT)

#define VOL_MINUS_KEY_BYTE 0
#define VOL_MINUS_KEY_BIT  7
#define VOL_MINUS_KEY_MASK (1 << VOL_MINUS_KEY_BIT)
#define VOL_MINUS_KEY_CODE HOGP_CREATE_KEY_CODE(VOL_MINUS_KEY_BYTE, VOL_MINUS_KEY_BIT)

#define BACK_KEY_BYTE      1
#define BACK_KEY_BIT       6
#define BACK_KEY_MASK      (1 << BACK_KEY_BIT)
#define BACK_KEY_CODE      HOGP_CREATE_KEY_CODE(BACK_KEY_BYTE, BACK_KEY_BIT)

#define POWER_KEY_BYTE     2
#define POWER_KEY_BIT      4
#define POWER_KEY_MASK     (1 << POWER_KEY_BIT)
#define POWER_KEY_CODE     HOGP_CREATE_KEY_CODE(POWER_KEY_BYTE, POWER_KEY_BIT)

/*****************************************************************************************
 * Definition of byte positions in mouse report
******************************************************************************************/
#define HID_MOUSE_REPORT_BUTTON_BYTE  0
#define HID_MOUSE_REPORT_X_BYTE       1
#define HID_MOUSE_REPORT_Y_BYTE       3
#define HID_MOUSE_REPORT_WHEEL_BYTE   5
#define HID_MOUSE_REPORT_H_DATA_BYTE  7

// Report Descriptor == Report Map (Universal Serial Bus - Device Class Definition 
// for Human Interface section E.6)
static const uint8_t report_map[] =
{
#if defined(HAS_KBD) || defined(HAS_TOUCHPAD_SLIDER)
    HID_USAGE_PAGE    (HID_USAGE_PAGE_GENERIC_DESKTOP), 
    HID_USAGE         (HID_GEN_DESKTOP_USAGE_KEYBOARD), 
    HID_COLLECTION    (HID_APPLICATION),                
        HID_REPORT_ID     (HID_KBD_NORMAL_REPORT_ID),                           
        HID_USAGE_PAGE    (HID_USAGE_PAGE_KEY_CODES),   
        HID_USAGE_MIN_8   (0xE0),                       
        HID_USAGE_MAX_8   (0xE7),                       
        HID_LOGICAL_MIN_8 (0),                       
        HID_LOGICAL_MAX_8 (1),                       
        HID_REPORT_SIZE   (1),                       
        HID_REPORT_COUNT  (8),                       
        HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_ABS_BIT),
        HID_REPORT_COUNT  (1),                       
        HID_REPORT_SIZE   (8),                       
        HID_INPUT         (HID_CONST_BIT), 
    //LED DEFINITION - Kept for compatibility with keyboard reference design
    //        HID_REPORT_ID     (HID_KBD_REPORT_2_ID),  // Report ID for the output LED report is confusing the host
        HID_REPORT_COUNT  (5),                      
        HID_REPORT_SIZE   (1),                      
        HID_USAGE_PAGE    (HID_USAGE_PAGE_LEDS),       
        HID_USAGE_MIN_8   (1),                      
        HID_USAGE_MAX_8   (5),                      
        HID_OUTPUT        (HID_DATA_BIT | HID_VAR_BIT | HID_ABS_BIT), 
        HID_REPORT_COUNT  (1),                       
        HID_REPORT_SIZE   (3),                       
        HID_OUTPUT        (HID_CONST_BIT), 
    //        HID_REPORT_ID     (HID_KBD_REPORT_1_ID),   // Repeating report ID is confusing the host        
        HID_REPORT_COUNT  (6),                       
        HID_REPORT_SIZE   (8),                       
        HID_LOGICAL_MIN_8 (0),                       
        HID_LOGICAL_MAX_8 (0x65),                       
        HID_USAGE_PAGE    (HID_USAGE_PAGE_KEY_CODES),   
        HID_USAGE_MIN_8   (0),                       
        HID_USAGE_MAX_8   (0x65),                       
        HID_INPUT         (HID_DATA_BIT | HID_ARY_BIT),
    HID_END_COLLECTION,   
    HID_USAGE_PAGE    (HID_USAGE_PAGE_CONSUMER),         
    HID_USAGE         (HID_CONSUMER_USAGE_CONSUMER_CONTROL), 
    HID_COLLECTION    (HID_APPLICATION),    
        HID_REPORT_ID     (HID_KBD_EXTENDED_REPORT_ID),               
        HID_LOGICAL_MIN_8 (0),               
        HID_LOGICAL_MAX_8 (1),
        HID_REPORT_SIZE   (1),               
        HID_REPORT_COUNT  (8),         
        HID_USAGE         (0xB5),         // Usage (Scan Next Track)
        HID_USAGE         (0xB6),         // Usage (Scan Previous Track)
        HID_USAGE         (0xB7),         // Usage (Stop)
        HID_USAGE         (0xB8),         // Usage (Eject)
        HID_USAGE         (0xCD),         // Usage (Play/Pause)
        HID_USAGE         (0xE2),         // Usage (Mute)
        HID_USAGE         (0xE9),         // Usage (Volume Increment)
        HID_USAGE         (0xEA),         // Usage (Volume Decrement)
        HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_ABS_BIT |
                           HID_NWRP_BIT | HID_LIN_BIT | HID_PREF_BIT |
                           HID_NNUL_BIT), // Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)
        HID_USAGE16       (0x83, 0x01),   // Usage (AL Consumer Control Configuration)
        HID_USAGE16       (0x8A, 0x01),   // Usage (AL Email Reader)
        HID_USAGE16       (0x92, 0x01),   // Usage (AL Calculator)
        HID_USAGE16       (0x94, 0x01),   // Usage (AL Local Machine Browser)
        HID_USAGE16       (0x21, 0x02),   // Usage (AC Search)
        HID_USAGE_MIN_16  (0x23, 0x02),   // Usage Minimum (AC Home)
        HID_USAGE_MAX_16  (0x25, 0x02),   // Usage Maximum (AC Forward). AC Back is 0x0224
        HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_ABS_BIT |
                           HID_NWRP_BIT | HID_LIN_BIT | HID_PREF_BIT |
                           HID_NNUL_BIT), // Input (Data,Var,Abs,NWrp,Lin,Pref,NNul,Bit)
        HID_REPORT_COUNT  (0x07),         
        HID_USAGE16       (0x26, 0x02),   // Usage (AC Stop)
        HID_USAGE16       (0x27, 0x02),   // Usage (AC Refresh)
        HID_USAGE16       (0x2A, 0x02),   // Usage (AC Bookmarks)
        HID_USAGE         (0x40),         // Usage (Menu)          
        HID_USAGE         (0x30),         // Usage (Power)         
        HID_USAGE         (0x9A),         // Usage (Media select home)
        HID_USAGE         (0x46),         // Usage (Menu escape)      
        HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_ABS_BIT |
                           HID_NWRP_BIT | HID_LIN_BIT | HID_PREF_BIT | HID_NNUL_BIT), 
        HID_REPORT_COUNT  (1),         
        HID_INPUT         (HID_CONST_BIT | HID_ARY_BIT | HID_ABS_BIT), 
    HID_END_COLLECTION,
#endif        
#if (defined(HAS_AUDIO) && !defined(AUDIO_USE_CUSTOM_PROFILE)) || defined(HAS_MOTION)
    HID_USAGE_PAGE_VENDOR_DEFINED,               
    HID_USAGE         (0x01),                    
    HID_COLLECTION    (HID_LOGICAL),  
#endif    
#if defined(HAS_AUDIO) && !defined(AUDIO_USE_CUSTOM_PROFILE)
        HID_REPORT_ID     (HID_STREAM_CTRL_OUT_REPORT_ID),                    
        HID_USAGE_PAGE_VENDOR_DEFINED,               
        HID_USAGE_MIN_8   (0),                    
        HID_USAGE_MAX_8   (0),                    
        HID_REPORT_COUNT  (HID_VENDOR_DEFINED_REPORT_SIZE),                    
        HID_REPORT_SIZE   (8),                    
        HID_LOGICAL_MIN_8 (0),                    
        HID_LOGICAL_MAX_16(0xff, 0x00),              
        HID_OUTPUT        (HID_DATA_BIT | HID_ARY_BIT | HID_ABS_BIT | HID_NPREF_BIT),  
        HID_REPORT_ID     (HID_STREAM_CTRL_IN_REPORT_ID),                   
        HID_USAGE_PAGE_VENDOR_DEFINED,              
        HID_USAGE_MIN_8   (0),                   
        HID_USAGE_MAX_8   (0),                   
        HID_REPORT_COUNT  (HID_VENDOR_DEFINED_REPORT_SIZE),                   
        HID_REPORT_SIZE   (8),                   
        HID_LOGICAL_MIN_8 (0),                   
        HID_LOGICAL_MAX_16(0xff, 0x00),             
        HID_INPUT         (HID_DATA_BIT | HID_ARY_BIT | HID_ABS_BIT | HID_NPREF_BIT),  
        HID_REPORT_ID     (HID_AUDIO_DATA_1_REPORT_ID),                    
        HID_USAGE_PAGE_VENDOR_DEFINED,               
        HID_USAGE_MIN_8   (0),                    
        HID_USAGE_MAX_8   (0),                    
        HID_REPORT_COUNT  (HID_VENDOR_DEFINED_REPORT_SIZE),                    
        HID_REPORT_SIZE   (8),                    
        HID_LOGICAL_MIN_8 (0),                    
        HID_LOGICAL_MAX_16(0xff, 0x00),              
        HID_INPUT         (HID_DATA_BIT | HID_ARY_BIT | HID_ABS_BIT | HID_NPREF_BIT),  
    #ifdef AUDIO_USE_THREE_HID_REPORTS
        HID_REPORT_ID     (HID_AUDIO_DATA_2_REPORT_ID),                   
        HID_USAGE_PAGE_VENDOR_DEFINED,              
        HID_USAGE_MIN_8   (0),                   
        HID_USAGE_MAX_8   (0),                   
        HID_REPORT_SIZE   (8),                   
        HID_REPORT_COUNT  (HID_VENDOR_DEFINED_REPORT_SIZE),                   
        HID_LOGICAL_MIN_8 (0),                   
        HID_LOGICAL_MAX_16(0xff, 0x00),             
        HID_INPUT         (HID_DATA_BIT | HID_ARY_BIT | HID_ABS_BIT | HID_NPREF_BIT), 

        HID_REPORT_ID     (HID_AUDIO_DATA_3_REPORT_ID),                    
        HID_USAGE_PAGE_VENDOR_DEFINED,               
        HID_USAGE_MIN_8   (0),                    
        HID_USAGE_MAX_8   (0),                    
        HID_REPORT_COUNT  (HID_VENDOR_DEFINED_REPORT_SIZE),                    
        HID_REPORT_SIZE   (8),                    
        HID_LOGICAL_MIN_8 (0),                    
        HID_LOGICAL_MAX_16(0xff, 0x00),              
        HID_INPUT         (HID_DATA_BIT | HID_ARY_BIT | HID_ABS_BIT | HID_NPREF_BIT),  
    #endif
#endif
#ifdef HAS_MOTION
        HID_REPORT_ID     (HID_MOTION_DATA_REPORT_ID),                    
        HID_USAGE_PAGE_VENDOR_DEFINED,               
        HID_USAGE_MIN_8   (0),                    
        HID_USAGE_MAX_8   (0),                    
        HID_REPORT_COUNT  (HID_VENDOR_DEFINED_REPORT_SIZE),                    
        HID_REPORT_SIZE   (8),                    
        HID_LOGICAL_MIN_8 (0),                    
        HID_LOGICAL_MAX_16(0xff, 0x00),              
        HID_INPUT         (HID_DATA_BIT | HID_ARY_BIT | HID_ABS_BIT | HID_NPREF_BIT),  
#endif     
#if (defined(HAS_AUDIO) && !defined(AUDIO_USE_CUSTOM_PROFILE)) || defined(HAS_MOTION)
    HID_END_COLLECTION,
#endif   
#if defined(HAS_MOUSE) || defined(HAS_TOUCHPAD_TRACKPAD)
    HID_USAGE_PAGE    (HID_USAGE_PAGE_GENERIC_DESKTOP),   
    HID_USAGE         (HID_GEN_DESKTOP_USAGE_MOUSE), 
    HID_COLLECTION    (HID_APPLICATION),                
        HID_USAGE_PAGE    (HID_USAGE_PAGE_GENERIC_DESKTOP),   
        HID_USAGE         (HID_GEN_DESKTOP_USAGE_MOUSE), 
        HID_COLLECTION    (HID_LOGICAL),                       
            HID_REPORT_ID     (HID_MOUSE_REPORT_ID),
            HID_USAGE         (HID_GEN_DESKTOP_USAGE_POINTER),   
            HID_COLLECTION    (HID_PHYSICAL),                
// BUTTONS        
                HID_USAGE_PAGE    (HID_USAGE_PAGE_BUTTONS), 
                HID_USAGE_MIN_8   (1),                       
                HID_USAGE_MAX_8   (5),                       
                HID_REPORT_COUNT  (5),                       
                HID_REPORT_SIZE   (1),                       
                HID_LOGICAL_MIN_8 (0),                       
                HID_LOGICAL_MAX_8 (1),                       
                HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_ABS_BIT),
                HID_REPORT_COUNT  (1),                       
                HID_REPORT_SIZE   (3),                       
                HID_INPUT         (HID_CONST_BIT),
// HORIZONTAL (X)
                HID_USAGE_PAGE    (HID_USAGE_PAGE_GENERIC_DESKTOP), 
                HID_USAGE         (HID_GEN_DESKTOP_USAGE_X), 
// VERTICAL (Y)          
                HID_USAGE         (HID_GEN_DESKTOP_USAGE_Y), 
                HID_REPORT_COUNT  (2),                       
                HID_REPORT_SIZE   (16),                       
                HID_LOGICAL_MIN_16(0x00,0x80),      //LOGICAL MINIMUM (-32767)                      
                HID_LOGICAL_MAX_16(0xFF,0x7F),      //LOGICAL MAXIMUM (32767)                      
                HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_REL_BIT),
// WHEEL (Z)             
                HID_COLLECTION    (HID_LOGICAL),  
                    HID_USAGE         (HID_GEN_DESKTOP_USAGE_WHEEL),   
                    0x35, 0x00,                     // PHYSICAL MINIMUM (0)
                    0x45, 0x00,                     // PHYSICAL MAXIMUM (0)                    
                    HID_REPORT_COUNT  (1),  
                    HID_REPORT_SIZE   (16),                       
                    HID_LOGICAL_MIN_16(0x00,0x80),  // LOGICAL MINIMUM (-32767)                      
                    HID_LOGICAL_MAX_16(0xFF,0x7F),  // LOGICAL MAXIMUM (32767)                      
                    HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_REL_BIT),
                HID_END_COLLECTION,  // LOGICAL            
// H DATA     
                HID_COLLECTION    (HID_LOGICAL),  
                    HID_USAGE_PAGE    (HID_USAGE_PAGE_CONSUMER),   
                    HID_REPORT_COUNT  (1),  
                    HID_REPORT_SIZE   (16),                       
                    HID_LOGICAL_MIN_16(0x00,0x80),  // LOGICAL MINIMUM (-32767)                      
                    HID_LOGICAL_MAX_16(0xFF,0x7F),  // LOGICAL MAXIMUM (32767)                      
                    HID_USAGE16       (0x38,0x02),  // USAGE (AC Pan)  
                    HID_INPUT         (HID_DATA_BIT | HID_VAR_BIT | HID_REL_BIT),
                HID_END_COLLECTION, // LOGICAL    
            HID_END_COLLECTION,     // PHYSICAL
        HID_END_COLLECTION,         // LOGICAL            
    HID_END_COLLECTION,             // APPLICATION            
#endif
};

/**
 * \}
 * \}
 * \}
 */

#endif // _USER_HOGPD_CONFIG_H_
