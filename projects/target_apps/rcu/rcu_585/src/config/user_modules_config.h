/*****************************************************************************************
 *
 * \file user_modules_config.h
 *
 * \brief User modules configuration file.
 * 
******************************************************************************************/

#ifndef _USER_MODULES_CONFIG_H_
#define _USER_MODULES_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup APP_CONFIG
 * \{
 * \addtogroup MODULES_CFG
 *
 * \brief User modules configuration
 * \{
******************************************************************************************/

/*
 * DEFINES
******************************************************************************************/

/***************************************************************************************/
/* Exclude or not a module in user's application code.                                 */
/*                                                                                     */
/* (0) - The module is included. The module's messages are handled by the SDK.         */
/*                                                                                     */
/* (1) - The module is excluded. The user must handle the module's messages.           */
/*                                                                                     */
/* Note:                                                                               */
/*      This setting has no effect if the respective module is a BLE Profile           */
/*      that is not used included in the user's application.                           */
/***************************************************************************************/
#define EXCLUDE_DLG_GAP             (0)
#define EXCLUDE_DLG_TIMER           (0)
#define EXCLUDE_DLG_MSG             (0)
#define EXCLUDE_DLG_SEC             (1)
#define EXCLUDE_DLG_DISS            (0)
#define EXCLUDE_DLG_PROXR           (1)
#define EXCLUDE_DLG_BASS            (0)
#define EXCLUDE_DLG_SUOTAR          (0)
#define EXCLUDE_DLG_CUSTS1          (0)
#define EXCLUDE_DLG_CUSTS2          (1)

#include "rwip_config.h"

#ifdef HAS_KBD
    #include "port_kbd.h"
	#include "app_kbd.h"
	#include <app_kbd_config.h>
    #include "user_rcu_kbd.h"
#endif 

#ifdef HAS_HID_REPORT 
	#include "app_hid_report.h"
	#include <app_hid_report_config.h>
    #include "user_rcu.h"
#endif

#ifdef HAS_GPIO_KEYS 
    #include "app_gpio_keys.h"
    #include "port_gpio_keys.h"
#endif   

#ifdef HAS_CONNECTION_FSM
	#include "app_con_fsm.h"
    #include "port_con_fsm.h"
	#include "port_con_fsm_task.h"
    #include "app_pairing.h"
    #include "port_security.h"
#endif

#if (BLE_HID_DEVICE)
	#include "app_hogpd.h"
    #include <user_hogpd_config.h>
	#include "app_hogpd_task.h"
#endif

#ifdef HAS_SYSTICK
	#include "port_systick.h"
#endif    

#ifdef HAS_BLE_STREAM
	#include "app_stream.h"
	#include <app_stream_config.h>
#endif

#ifdef HAS_AUDIO
	#include "app_audio.h"
	#include <app_audio_config.h>
	#include "port_audio.h"
    #include "user_rcu_audio.h"
#endif

#ifdef HAS_SPI_FLASH_STORAGE
    #include "app_flash.h"
#endif

#ifdef HAS_MOTION
	#include "app_motion.h"
	#include <app_motion_config.h>
    #include "user_rcu_motion.h"
#endif

#ifdef CONFIGURE_MOTION_SENSOR_PINS
	#include <app_motion_config.h>
#endif

#ifdef HAS_MOUSE
    #include "app_mouse.h"
#endif

#ifdef HAS_LED_INDICATORS
	#include "app_leds.h"
#endif

#if defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)
    #include "app_touchpad.h"
    #include <app_touchpad_config.h>
    #include "user_rcu_motion.h"
#endif

#ifdef HAS_IR
    #include "app_ir.h"
    #include <app_ir_config.h>
#endif

#ifdef HAS_SOUND_INDICATION
    #include "app_buzzer.h"
    #include "port_buzzer.h"
#endif

static const module_config_t user_module_config[] = {      
#ifdef HAS_CONNECTION_FSM
    {.init_gpios        = NULL,
     .init              = app_con_fsm_init,
     .on_disconnect     = app_con_fsm_on_disconnect,
     .on_ble_powered    = app_con_fsm_on_ble_powered,
     .on_system_powered = app_con_fsm_on_system_powered,
     .pins_config       = NULL,
    }, 
#endif    
    
#ifdef HAS_KBD 
    {.init_gpios        = app_kbd_init_scan_gpios,
     .init              = app_kbd_init,
     .on_disconnect     = app_kbd_stop_reporting,
     .on_ble_powered    = app_kbd_on_ble_powered,
     .on_system_powered = app_kbd_on_system_powered,
     .pins_config       = SET_PIN_CONFIGURATION(app_kbd_row_pins),
    },
#endif    

#ifdef HAS_GPIO_KEYS
    {.init_gpios        = NULL,
     .init              = port_gpio_keys_enable_irq,
     .on_disconnect     = NULL,
     .on_ble_powered    = NULL,
     .on_system_powered = NULL,
     .pins_config       = SET_PIN_CONFIGURATION(app_gpio_keys_pins),
    },
#endif
        
#ifdef HAS_HID_REPORT    
    {.init_gpios        = NULL,
     .init              = app_hid_report_init,
     .on_disconnect     = NULL,
     .on_ble_powered    = user_hid_report_on_ble_powered,
     .on_system_powered = app_hid_report_on_system_powered,
     .pins_config       = NULL,
    }, 
#endif
    
#ifdef HAS_AUDIO
    {.init_gpios        = NULL,
     .init              = app_audio_init,
     .on_disconnect     = user_audio_on_disconnect,
     .on_ble_powered    = user_audio_on_ble_powered,
     .on_system_powered = user_audio_on_system_powered,
     .pins_config       = SET_PIN_CONFIGURATION(app_audio_pins),
    },
#endif    
    
#ifdef HAS_BLE_STREAM
    {.init_gpios        = NULL,
     .init              = app_stream_init,
     .on_disconnect     = app_stream_stop,
     .on_ble_powered    = app_stream_queue_data,
     .on_system_powered = NULL,
     .pins_config       = NULL,
    }, 
#endif
    
#ifdef HAS_SPI_FLASH_STORAGE
    {.init_gpios        = NULL,
     .init              = app_spi_flash_power_down,
     .on_disconnect     = NULL,
     .on_ble_powered    = NULL,
     .on_system_powered = NULL,
     .pins_config       = SET_PIN_CONFIGURATION(app_flash_pins),
    },
#endif  

#ifdef HAS_MOTION
    {
     .init_gpios        = NULL,
     .init              = app_motion_init,
     .on_disconnect     = app_motion_stop,
     .on_ble_powered    = user_motion_on_ble_powered,
     .on_system_powered = app_motion_on_system_powered,
    #if MOTION_IF == SPI
     .pins_config       = SET_PIN_CONFIGURATION(app_motion_cs_pin),
     #else   
     .pins_config       = NULL,
     #endif
    },
#endif

#if defined(CONFIGURE_MOTION_SENSOR_PINS) && (MOTION_IF == SPI)
    {
     .init_gpios        = NULL,
     .init              = NULL,
     .on_disconnect     = NULL,
     .on_ble_powered    = NULL,
     .on_system_powered = NULL,
     .pins_config       = SET_PIN_CONFIGURATION(app_motion_cs_pin),
    },
#endif
    
#if defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)
	{.init_gpios        = NULL,
     .init              = app_touchpad_init,
     .on_disconnect     = NULL,
     .on_ble_powered    = user_mouse_trackpad_on_ble_powered,
     .on_system_powered = user_touchpad_on_system_powered,
     .pins_config       = SET_PIN_CONFIGURATION(app_touchpad_int_pin),
    },
#endif    

#ifdef HAS_MOUSE
    {.init_gpios        = NULL,
     .init              = NULL,
     .on_disconnect     = NULL,
     .on_ble_powered    = user_mouse_trackpad_on_ble_powered,
     .on_system_powered = user_mouse_on_system_powered,
     .pins_config       = NULL
    },
#endif    
    
#ifdef HAS_LED_INDICATORS
    {.init_gpios        = app_leds_init_gpios,
     .init              = app_leds_init,
     .on_disconnect     = NULL,
     .on_ble_powered    = NULL,
     .on_system_powered = NULL,
     .pins_config       = SET_PIN_CONFIGURATION(app_led_pins),
    },
#endif
    
#ifdef HAS_IR
    {.init_gpios        = NULL,
     .init              = NULL,
     .on_disconnect     = NULL,
     .on_ble_powered    = NULL,
     .on_system_powered = NULL,
     .pins_config       = SET_PIN_CONFIGURATION(app_ir_pins),
    },
#endif

#ifdef HAS_SOUND_INDICATION
    {.init_gpios        = NULL,
     .init              = NULL,
     .on_disconnect     = NULL,
     .on_ble_powered    = NULL,
     .on_system_powered = NULL,
     .pins_config       = SET_PIN_CONFIGURATION(app_buzzer_pins),
    },
#endif   
};

/**
 * \}
 * \}
 * \}
 */
#endif // _USER_MODULES_CONFIG_H_
