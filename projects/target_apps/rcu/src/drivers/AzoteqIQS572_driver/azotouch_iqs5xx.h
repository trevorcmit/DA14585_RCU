/*****************************************************************************************
 *
 * \file azotouch_iqs5xx.h
 *
 * \brief Azoteq IQS572 module library provides an API with the touchpad module. The library
 * uses a local FIFO that is used to push/pop incoming touch events. The azotouch_iqs5xxx_process_xy
 * is meant to be used whenever there is a touch event(i.e inside the RDY pin's interrupt handler).
 * The poll function is meant to be used to check if the touch event FIFO has any events (not empty)
 * and pop these events for further processing by the user application. The IQS572 module supports
 * tracking, single and double finger tap, 2-finger scrolling, swipe gestures, zoom gestures and touch
 * and hold recognition.
 * 
******************************************************************************************/


#ifndef _AZOTOUCH_IQS5XX_H_
#define _AZOTOUCH_IQS5XX_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IQS572_I2C_DRV
 *
 * \{
******************************************************************************************/

#include <stdint.h>


#define AZOTOUCH_IQS5XX_USE_TOUCH_STR		0
#define AZOTOUCH_IQS5XX_USE_FINGER_AREA		0


#define AZOTOUCH_IQS5XX_TX_CHANNEL_BYTES	8
#define AZOTOUCH_IQS5XX_RX_CHANNEL_BYTES	9

#define AZOTOUCH_MAX_FINGERS				1

#define AZOTOUCH_LOCAL_FIFO_MAX_SIZE		8

/*
*   TYPEDEFS
*/
typedef enum
{
        azotouch_iqs5xx_noevent,
        azotouch_iqs5xx_reset,
        azotouch_iqs5xx_release,
        azotouch_iqs5xx_track,
        azotouch_iqs5xx_track_tap,
        azotouch_iqs5xx_tap_and_hold,
        azotouch_iqs5xx_swipe_x_pos,    // x+
        azotouch_iqs5xx_swipe_x_neg,    // x-
        azotouch_iqs5xx_swipe_y_pos,    // y+
        azotouch_iqs5xx_swipe_y_neg,    // y-
        azotouch_iqs5xx_zoom,
        azotouch_iqs5xx_2_finger_tap,
        azotouch_iqs5xx_scroll
} azotouch_iqs5xx_actions_enum_t;


typedef struct
{
        azotouch_iqs5xx_actions_enum_t action;
        int16_t RelX;
        int16_t RelY;
        uint16_t AbsX;
        uint16_t AbsY;
} azoteq_iqs_5xx_evt_t;



typedef struct
{
        uint8_t specialAction;
        uint8_t numOfFingers;
        int16_t RelativeXCoord;
        int16_t RelativeYCoord;
        uint16_t XCoord[AZOTOUCH_MAX_FINGERS];
        uint16_t YCoord[AZOTOUCH_MAX_FINGERS];
#if AZOTOUCH_IQS5XX_USE_FINGER_AREA
        uint8_t fingerArea[AZOTOUCH_MAX_FINGERS];
#endif
#if AZOTOUCH_IQS5XX_USE_TOUCH_STR
        uint16_t touchStrength[AZOTOUCH_MAX_FINGERS];
#endif
} azotouch_iqs5xxB_info_t;


/*****************************************************************************************
 * \brief Touchpad module initialization function
 * \return bool true on successful initialization of the module, false otherwise
******************************************************************************************/
bool azotouch_iqs5xx_init_module(void);


/*****************************************************************************************
 * \brief Touchpad module DE-initialization function
******************************************************************************************/
void azotouch_iqs5xx_deinit_module(void);


/*****************************************************************************************
 * \brief Processing function used as a handler for Azoteq events
******************************************************************************************/
void azotouch_iqs5xx_process_xy(azoteq_iqs_5xx_evt_t *evt);

/**
 * \}
 * \}
 * \}
 */

#endif // _AZOTOUCH_IQS5XX_H_
