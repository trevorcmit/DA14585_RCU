/*****************************************************************************************
 * \file app_touchpad_defs.h
 * \brief Touchpad module definitions
*****************************************************************************************/

/***************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup TOUCHPAD
 * \{
 * \addtogroup APP_TOUCHPAD
 * \{
****************************************************************************************/ 

#ifndef	APP_TOUCHPAD_DEFS_H
#define APP_TOUCHPAD_DEFS_H

#include <stdint.h>

/* 
 * Touchpad Event Masks. Used when registering the touchpad events 
 * we want the application to get notified about
 */
 
#define APP_TOUCHPAD_EVT_MASK_RESET             0x00000001
#define APP_TOUCHPAD_EVT_MASK_RELEASE           0x00000002
#define APP_TOUCHPAD_EVT_MASK_TRACK_STARTED     0x00000004
#define APP_TOUCHPAD_EVT_MASK_TRACKING          0x00000008
#define APP_TOUCHPAD_EVT_MASK_TRACK_STOPPED     0x00000010
#define APP_TOUCHPAD_EVT_MASK_TAP_UP            0x00000020
#define APP_TOUCHPAD_EVT_MASK_TAP_DOWN          0x00000040
#define APP_TOUCHPAD_EVT_MASK_TAP_RIGHT         0x00000080
#define APP_TOUCHPAD_EVT_MASK_TAP_LEFT          0x00000100
#define APP_TOUCHPAD_EVT_MASK_SINGLE_FINGER_TAP 0x00000200
#define APP_TOUCHPAD_EVT_MASK_TOUCH_AND_HOLD    0x00000400
#define APP_TOUCHPAD_EVT_MASK_SWIPE_LEFT        0x00000800
#define APP_TOUCHPAD_EVT_MASK_SWIPE_RIGHT       0x00001000
#define APP_TOUCHPAD_EVT_MASK_SWIPE_UP          0x00002000
#define APP_TOUCHPAD_EVT_MASK_SWIPE_DOWN        0x00004000
#define APP_TOUCHPAD_EVT_MASK_ZOOM_IN           0x00008000
#define APP_TOUCHPAD_EVT_MASK_ZOOM_OUT          0x00010000
#define APP_TOUCHPAD_EVT_MASK_DOUBLE_FINGER_TAP 0x00020000
#define APP_TOUCHPAD_EVT_MASK_SCROLL_UP         0x00040000
#define APP_TOUCHPAD_EVT_MASK_SCROLL_DOWN       0x00080000
#define APP_TOUCHPAD_EVT_MASK_SCROLL_RIGHT      0x00100000
#define APP_TOUCHPAD_EVT_MASK_SCROLL_LEFT       0x00200000
#define APP_TOUCHPAD_EVT_MASK_FLICK_LEFT        0x00400000
#define APP_TOUCHPAD_EVT_MASK_FLICK_RIGHT       0x00800000
#define APP_TOUCHPAD_EVT_MASK_ROTATE_LEFT       0x01000000
#define APP_TOUCHPAD_EVT_MASK_ROTATE_RIGHT      0x02000000


/*
*   TYPEDEFS
*/


typedef struct
{
        int16_t deltaX;
        int16_t deltaY;
} app_touchpad_deltaCoords_t;


typedef struct
{
        uint16_t X;
        uint16_t Y;
} app_touchpad_Coords_t;


typedef struct
{
        app_touchpad_Coords_t app_touch_coords;
        app_touchpad_deltaCoords_t app_touch_delta;
} app_touchpad_track_data_t;


typedef enum
{
    APP_TOUCHPAD_RESET,
    APP_TOUCHPAD_RELEASE,
    APP_TOUCHPAD_TRACK_STARTED,
    APP_TOUCHPAD_TRACKING,
    APP_TOUCHPAD_TRACK_STOPPED,
    APP_TOUCHPAD_TAP_UP,
    APP_TOUCHPAD_TAP_DOWN,
    APP_TOUCHPAD_TAP_RIGHT,
    APP_TOUCHPAD_TAP_LEFT,
    APP_TOUCHPAD_SINGLE_FINGER_TAP,
    APP_TOUCHPAD_TOUCH_AND_HOLD,
    APP_TOUCHPAD_SWIPE_LEFT,    
    APP_TOUCHPAD_SWIPE_RIGHT,    
    APP_TOUCHPAD_SWIPE_UP,  
    APP_TOUCHPAD_SWIPE_DOWN,
    APP_TOUCHPAD_ZOOM_IN,
    APP_TOUCHPAD_ZOOM_OUT,
    APP_TOUCHPAD_DOUBLE_FINGER_TAP,
    APP_TOUCHPAD_SCROLL_UP,
    APP_TOUCHPAD_SCROLL_DOWN,
    APP_TOUCHPAD_SCROLL_RIGHT,
    APP_TOUCHPAD_SCROLL_LEFT,
    APP_TOUCHPAD_FLICK_LEFT,
    APP_TOUCHPAD_FLICK_RIGHT,
    APP_TOUCHPAD_ROTATE_LEFT,
    APP_TOUCHPAD_ROTATE_RIGHT,
    APP_TOUCHPAD_NO_EVENT
} app_touchpad_actions_t;



typedef struct
{
    app_touchpad_actions_t touch_action;
    app_touchpad_Coords_t app_touchpad_coords;
}app_touchpad_evt_t;

typedef struct
{
    app_touchpad_actions_t touch_action;
    app_touchpad_track_data_t app_touchpad_track_data;
} app_touchpad_track_info_t;


typedef void (*app_touchpad_cb_t) (app_touchpad_evt_t * info);

typedef void (*touchpad_evt_handler_t) (void);

typedef void (*app_touchpad_reg_handler_t)(touchpad_evt_handler_t handler);

typedef void (*app_touchpad_init_t)(void);
typedef void (*app_touchpad_deinit_t)(void);
typedef bool (*app_touchpad_poll_t)(app_touchpad_evt_t *);

typedef struct {
    app_touchpad_init_t  app_tpad_init;
    app_touchpad_deinit_t app_tpad_deinit;
    app_touchpad_poll_t app_tpad_poll;
} app_touchpad_util_funcs_t;


#endif // APP_TOUCHPAD_DEFS_H

/**
 * \}
 * \}
 * \}
 */
