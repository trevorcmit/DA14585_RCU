/*****************************************************************************************
 *
 * \file port_touchpad.c
 *
 * \brief Touchpad module platform adaptation source file
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup TOUCHPAD
 * \{
 * \addtogroup PORT_TOUCHPAD
 * \{
 ****************************************************************************************     
 */

#if defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)

#include "port_touchpad.h"
#include <app_touchpad_config.h>
#include "port_platform.h"
#include <user_periph_setup.h>
#include "port_wkup.h"

#ifndef TOUCH_PAD_MODULE
#error "No touchpad module selected. Please define TOUCH_PAD_MODULE"
#endif

#if (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_5XX)
#include "azotouch_iqs5xx.h"
#elif (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_263)
#include "azotouch_iqs_263.h"
#else
// placeholder for user defined touch controller
#endif

#if(APP_TOUCH_COORDS_UP_MIN>=APP_TOUCH_COORDS_UP_MAX)
#define APP_TOUCH_TAP_LOCATION_UP(x)   ((x>=APP_TOUCH_COORDS_UP_MIN && x<=APP_TOUCH_COORDS_MAX) || (x>=0 && x<=APP_TOUCH_COORDS_UP_MAX)) 
#else
#define APP_TOUCH_TAP_LOCATION_UP(x)   (x>=APP_TOUCH_COORDS_UP_MIN && x<=APP_TOUCH_COORDS_UP_MAX) 
#endif


#if(APP_TOUCH_COORDS_DOWN_MIN>=APP_TOUCH_COORDS_DOWN_MAX)
#define APP_TOUCH_TAP_LOCATION_DOWN(x)   ((x>=APP_TOUCH_COORDS_DOWN_MIN && x<=APP_TOUCH_COORDS_MAX) || (x>=0 && x<=APP_TOUCH_COORDS_DOWN_MAX)) 
#else
#define APP_TOUCH_TAP_LOCATION_DOWN(x)   (x>=APP_TOUCH_COORDS_DOWN_MIN && x<=APP_TOUCH_COORDS_DOWN_MAX) 
#endif

#if(APP_TOUCH_COORDS_RIGHT_MIN>=APP_TOUCH_COORDS_RIGHT_MAX)
#define APP_TOUCH_TAP_LOCATION_RIGHT(x)   ((x>=APP_TOUCH_COORDS_RIGHT_MIN && x<=APP_TOUCH_COORDS_MAX) || (x>=0 && x<=APP_TOUCH_COORDS_RIGHT_MAX)) 
#else
#define APP_TOUCH_TAP_LOCATION_RIGHT(x)   (x>=APP_TOUCH_COORDS_RIGHT_MIN && x<=APP_TOUCH_COORDS_RIGHT_MAX) 
#endif


#if(APP_TOUCH_COORDS_LEFT_MIN>=APP_TOUCH_COORDS_LEFT_MAX)
#define APP_TOUCH_TAP_LOCATION_LEFT(x)   ((x>=APP_TOUCH_COORDS_LEFT_MIN && x<=APP_TOUCH_COORDS_MAX) || (x>=0 && x<=APP_TOUCH_COORDS_LEFT_MAX)) 
#else
#define APP_TOUCH_TAP_LOCATION_LEFT(x)   (x>=APP_TOUCH_COORDS_LEFT_MIN && x<=APP_TOUCH_COORDS_LEFT_MAX) 
#endif


typedef struct
{
        uint8_t tail;
        uint8_t head;
        uint8_t nelems;
        app_touchpad_evt_t data[TOUCHPAD_LOCAL_FIFO_MAX_SIZE];
} port_touchpad_LocalFIFO_t;


static port_touchpad_LocalFIFO_t     _touchLocalFifo     __PORT_RETAINED;


static void port_touchpad_localFifoPush(app_touchpad_evt_t *touchpad_element)
{
    if (_touchLocalFifo.nelems == TOUCHPAD_LOCAL_FIFO_MAX_SIZE || !touchpad_element) {
        return;
    }

    // Push the bytes into the FIFO
    memcpy(&_touchLocalFifo.data[_touchLocalFifo.tail], touchpad_element, sizeof(app_touchpad_evt_t));

    _touchLocalFifo.tail = (_touchLocalFifo.tail + 1) % TOUCHPAD_LOCAL_FIFO_MAX_SIZE;
    ++_touchLocalFifo.nelems;
}


static void port_touchpad_localFifoPop(app_touchpad_evt_t *touchpad_element)
{
    // If there is nothing to pop, return immediately
    if (!_touchLocalFifo.nelems || !touchpad_element) {
            return;
    }

    // Push the bytes into the FIFO
    memcpy(touchpad_element, &_touchLocalFifo.data[_touchLocalFifo.head], sizeof(app_touchpad_evt_t));

    // Update the FIFO's attributes ( head and total bytes )
    _touchLocalFifo.head = (_touchLocalFifo.head + 1) % TOUCHPAD_LOCAL_FIFO_MAX_SIZE;
    --_touchLocalFifo.nelems;
}


bool port_touchpad_poll(app_touchpad_evt_t * touchpad_element)
{
    // If there any pending events in our local FIFO
    if (_touchLocalFifo.nelems) {
        if (touchpad_element) {
            port_touchpad_localFifoPop(touchpad_element);
            return true;
        }
    }
    return false;
}


void port_touchpad_enable_irq()
{
    port_wkup_enable_irq(WKUP_TOUCHPAD_CHANNEL, TOUCHPAD_PIN_WAKEUP_PIN_POLARITY);
}


void port_touchpad_disable_irq()
{
    port_wkup_disable_irq(WKUP_TOUCHPAD_CHANNEL);
}


void port_touchpad_reserve_gpios(void)
{
    RESERVE_GPIO(TOUCHPAD_INTERRUPT_PIN, TOUCHPAD_INT_PORT, TOUCHPAD_INT_PIN, PID_GPIO);
}

void port_touchpad_init_gpios(void)
{
    GPIO_ConfigurePin(TOUCHPAD_INT_PORT, TOUCHPAD_INT_PIN, INPUT_PULLDOWN, PID_GPIO, false);
}

#if (TOUCH_PAD_MODULE==AZOTEQ_TOUCH_IQS_5XX)
static void port_touchpad_iqs_5xx_evt_handler()
{
    azoteq_iqs_5xx_evt_t iqsEvt;
    app_touchpad_evt_t touchEvtElement;
    azotouch_iqs5xx_process_xy(&iqsEvt);
    
    touchEvtElement.app_touchpad_coords.X = iqsEvt.AbsX;
    touchEvtElement.app_touchpad_coords.Y = iqsEvt.AbsY;
    
    switch(iqsEvt.action) {
        case azotouch_iqs5xx_reset:
            touchEvtElement.touch_action = APP_TOUCHPAD_RESET;
            dbg_puts(DBG_TOUCH_LVL,"TOUCH Reset\r\n");
            break;
        
        case azotouch_iqs5xx_release:
            touchEvtElement.touch_action = APP_TOUCHPAD_RELEASE;
            dbg_puts(DBG_TOUCH_LVL,"TOUCH Released\r\n");
            break;

        case azotouch_iqs5xx_track:
            touchEvtElement.touch_action = APP_TOUCHPAD_TRACKING;
            dbg_puts(DBG_TOUCH_LVL,"TRACKING\r\n");
            break;
            
        case azotouch_iqs5xx_track_tap:
            touchEvtElement.touch_action = APP_TOUCHPAD_SINGLE_FINGER_TAP;
            dbg_puts(DBG_TOUCH_LVL,"SF TAP\r\n");
            break;
        
        case azotouch_iqs5xx_tap_and_hold:
            touchEvtElement.touch_action = APP_TOUCHPAD_TOUCH_AND_HOLD;
            dbg_puts(DBG_TOUCH_LVL,"TOUCH & HOLD\r\n");
            break;
            
        case azotouch_iqs5xx_swipe_x_pos:
            touchEvtElement.touch_action = APP_TOUCHPAD_SWIPE_RIGHT;
            dbg_puts(DBG_TOUCH_LVL,"SWIPE Right\r\n");
            break;
            
        case azotouch_iqs5xx_swipe_x_neg:  
            touchEvtElement.touch_action = APP_TOUCHPAD_SWIPE_LEFT;
            dbg_puts(DBG_TOUCH_LVL,"SWIPE Left\r\n");
            break;
            
        case azotouch_iqs5xx_swipe_y_pos: 
            touchEvtElement.touch_action = APP_TOUCHPAD_SWIPE_DOWN;
            dbg_puts(DBG_TOUCH_LVL,"SWIPE Down\r\n");
            break;
            
        case azotouch_iqs5xx_swipe_y_neg: 
            touchEvtElement.touch_action = APP_TOUCHPAD_SWIPE_UP;
            dbg_puts(DBG_TOUCH_LVL,"SWIPE Up\r\n");
            break;
            
        case azotouch_iqs5xx_zoom:
            if(iqsEvt.RelX<0){
                touchEvtElement.touch_action = APP_TOUCHPAD_ZOOM_OUT;
                dbg_puts(DBG_TOUCH_LVL,"ZOOM Out\r\n");
            }
            else {
                touchEvtElement.touch_action = APP_TOUCHPAD_ZOOM_IN;
                dbg_puts(DBG_TOUCH_LVL,"ZOOM In\r\n");
            }
            break;
            
        case azotouch_iqs5xx_2_finger_tap:
            touchEvtElement.touch_action = APP_TOUCHPAD_DOUBLE_FINGER_TAP;
            dbg_puts(DBG_TOUCH_LVL,"2F Tap\r\n");
            break;
        
        case azotouch_iqs5xx_scroll:
            if(iqsEvt.RelX==0) {
                if(iqsEvt.RelY>0) {
                    touchEvtElement.touch_action = APP_TOUCHPAD_SCROLL_DOWN;
                    dbg_puts(DBG_TOUCH_LVL,"SCROLL Down\r\n");
                }
                else {
                    touchEvtElement.touch_action = APP_TOUCHPAD_SCROLL_UP;
                    dbg_puts(DBG_TOUCH_LVL,"SCROLL Up\r\n");
                }
            }
            else if (iqsEvt.RelY==0) {
                if(iqsEvt.RelX>0) {
                    touchEvtElement.touch_action = APP_TOUCHPAD_SCROLL_RIGHT;
                    dbg_puts(DBG_TOUCH_LVL,"SCROLL Right\r\n");
                }
                else {
                    touchEvtElement.touch_action = APP_TOUCHPAD_SCROLL_LEFT;
                    dbg_puts(DBG_TOUCH_LVL,"SCROLL Left\r\n");
                }
            }
            break;
    }
    
    // Push the acquired data into our Local Fifo
    port_touchpad_localFifoPush(&touchEvtElement);
    
}
#endif

#if (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_263)
uint8_t accumulated_slide_left_events    __PORT_RETAINED;
uint8_t accumulated_slide_right_events   __PORT_RETAINED;

static void port_touchpad_iqs_263_evt_handler(void)
{
    bool slideLeft;
    bool slideRight;
    azoteq_iqs_263_evt_t iqsEvt;
    app_touchpad_evt_t touchEvtElement;
    
    azotouch_iqs_263_process_event(&iqsEvt);
    
    slideLeft = (iqsEvt.actions.eBitMask.slide) && (iqsEvt.actions.eBitMask.touch) && (iqsEvt.relCoord > 0);
    slideRight = (iqsEvt.actions.eBitMask.slide) && (iqsEvt.actions.eBitMask.touch) && (iqsEvt.relCoord < 0);
    
    touchEvtElement.app_touchpad_coords.X = iqsEvt.relCoord;
    touchEvtElement.app_touchpad_coords.Y = iqsEvt.wheelCoord;
    
    // If a slide event happened
    if(slideLeft) {
        accumulated_slide_right_events =0;
        if(++accumulated_slide_left_events >= APP_TOUCH_MAX_SLIDE_ACCUMULATED_EVENTS) {
            accumulated_slide_left_events = 0;
            touchEvtElement.touch_action = APP_TOUCHPAD_ROTATE_LEFT;
            dbg_puts(DBG_TOUCH_LVL,"ROTATE Left\r\n");
            // Push the acquired data into our Local Fifo
            port_touchpad_localFifoPush(&touchEvtElement);
        }
        return;
    }
    
    if(slideRight) {
        accumulated_slide_left_events =0;
        if(++accumulated_slide_right_events >= APP_TOUCH_MAX_SLIDE_ACCUMULATED_EVENTS) {
            accumulated_slide_right_events = 0;
            touchEvtElement.touch_action = APP_TOUCHPAD_ROTATE_RIGHT;
            dbg_puts(DBG_TOUCH_LVL,"ROTATE Right\r\n");
            // Push the acquired data into our Local Fifo
            port_touchpad_localFifoPush(&touchEvtElement);
        }
        return;
    }

    if(iqsEvt.actions.eBitMask.flick_right) {
        accumulated_slide_left_events =0;
        accumulated_slide_right_events =0;
        touchEvtElement.touch_action = APP_TOUCHPAD_FLICK_RIGHT;
        dbg_puts(DBG_TOUCH_LVL,"FLICK Right\r\n");
        // Push the acquired data into our Local Fifo
        port_touchpad_localFifoPush(&touchEvtElement);
        return;
    }
    
    if(iqsEvt.actions.eBitMask.flick_left) {
        accumulated_slide_left_events =0;
        accumulated_slide_right_events =0;
        touchEvtElement.touch_action = APP_TOUCHPAD_FLICK_LEFT;
        dbg_puts(DBG_TOUCH_LVL,"FLICK Left\r\n");
        // Push the acquired data into our Local Fifo
        port_touchpad_localFifoPush(&touchEvtElement);
        return;
    }
    
    if(iqsEvt.actions.eBitMask.tap) {
        accumulated_slide_left_events =0;
        accumulated_slide_right_events =0;
        iqsEvt.relCoord  = (iqsEvt.relCoord>128)?(((iqsEvt.relCoord-128)/2) + 128): iqsEvt.relCoord;
        
        if(APP_TOUCH_TAP_LOCATION_UP(iqsEvt.relCoord)) {
            touchEvtElement.touch_action = APP_TOUCHPAD_TAP_UP;
             dbg_puts(DBG_TOUCH_LVL,"TAP Up\r\n");
        }
        else if(APP_TOUCH_TAP_LOCATION_DOWN(iqsEvt.relCoord)) {
            touchEvtElement.touch_action = APP_TOUCHPAD_TAP_DOWN;
            dbg_puts(DBG_TOUCH_LVL,"TAP Down\r\n");
        }                
        else if(APP_TOUCH_TAP_LOCATION_RIGHT(iqsEvt.relCoord)) {
            touchEvtElement.touch_action = APP_TOUCHPAD_TAP_RIGHT;
            dbg_puts(DBG_TOUCH_LVL,"TAP Right\r\n");
        }
        else if(APP_TOUCH_TAP_LOCATION_LEFT(iqsEvt.relCoord)) {
            touchEvtElement.touch_action = APP_TOUCHPAD_TAP_LEFT;
            dbg_puts(DBG_TOUCH_LVL,"TAP Left\r\n");
        }
 
        if(touchEvtElement.touch_action >= APP_TOUCHPAD_TAP_UP && touchEvtElement.touch_action <= APP_TOUCHPAD_TAP_LEFT) {
            // Push the acquired data into our Local Fifo
            port_touchpad_localFifoPush(&touchEvtElement);
        }
    }
}
#endif

void port_touchpad_init()
{
    bool stat;
    // Init the touchpad-related peripherals
#if (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_5XX)
    stat = azotouch_iqs5xx_init_module();
#elif (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_263)
    stat = azotouch_iqs_263_init_module();
#endif
    
    // If module initialization was successful, also enable its interrupt
    if (stat) {
        port_touchpad_enable_irq();
    }
}

void port_touchpad_deinit()
{
    // Init the touchpad-related peripherals
#if (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_5XX)
    azotouch_iqs5xx_deinit_module();
#elif (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_263)
    azotouch_iqs_263_deinit_module();
#endif
    
    port_touchpad_disable_irq();

}

void port_touchpad_handler(void)
{
#if (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_5XX)
    port_touchpad_iqs_5xx_evt_handler();
#elif (TOUCH_PAD_MODULE==AZOTEQ_TOUCHPAD_IQS_263)
    port_touchpad_iqs_263_evt_handler();
#endif    

    port_touchpad_enable_irq();
}

#endif // HAS_TOUCHPAD_TRACKPAD || HAS_TOUCHPAD_SLIDER

/**
 * \}
 * \}
 * \}
 */
