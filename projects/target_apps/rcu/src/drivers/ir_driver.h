/*****************************************************************************************
 *
 * \file ir_driver.h
 *
 * \brief IR low level driver.
 * 
******************************************************************************************/

#ifndef _IR_DRIVER_H_
#define _IR_DRIVER_H_

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IR_DRV
 *
 * \brief IR driver
 *
 * \{
******************************************************************************************/
  
#include "gpio.h"

#if (RWBLE_SW_VERSION_MAJOR >= 8)
    #include "timer0.h"
    #include "timer.h"
#else
    #include "pwm.h"
#endif

#define MESSAGE_MAX_FIFO_LEVEL        8
#define REPEAT_MAX_FIFO_LEVEL         4

#define IR_TIMER_WAIT_FOR_BLE_END     0
#define IR_TIMER_COMMAND              1
#define IR_TIMER_WAIT_FOR_REPEAT      2

#define IR_ENABLE_DEBUG_GPIO          0

#if IR_ENABLE_DEBUG_GPIO == 1
    #define IR_IRQ_SIG_GPIO_PORT        GPIO_PORT_0
    #define IR_IRQ_SIG_GPIO_PIN         GPIO_PIN_7
    #define IR_OUT_SIG_GPIO_PORT        GPIO_PORT_0
    #define IR_OUT_SIG_GPIO_PIN         GPIO_PIN_6
    #define IR_BIT_SIG_GPIO_PORT        GPIO_PORT_3
    #define IR_BIT_SIG_GPIO_PIN         GPIO_PIN_7
    #define IR_WAIT_SIG_GPIO_PORT       GPIO_PORT_3
    #define IR_WAIT_SIG_GPIO_PIN        GPIO_PIN_6
#endif


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

enum {
    IR_MESSAGE_TYPE_PAINT = 0,
    IR_MESSAGE_TYPE_DIGITAL
};

enum {
    IR_PAINT_SYMBOL_TYPE_SPACE = 0,
    IR_PAINT_SYMBOL_TYPE_MARK
};

enum {
    IR_LOGIC_ONE_STARTS_MARK = 0,
    IR_LOGIC_ONE_STARTS_SPACE
};

enum {
    IR_LOGIC_ZERO_STARTS_MARK = 0,
    IR_LOGIC_ZERO_STARTS_SPACE
};

enum {
    IR_BIT_STATE_FIRST_PART = 0,
    IR_BIT_STATE_SECOND_PART
};

enum {
    IR_REPEAT_FROM_CODE_FIFO = 0,
    IR_REPEAT_FROM_REPEAT_FIFO
};

/*
 * TYPES DEFINITIONS
 *****************************************************************************************
 */

typedef enum {
    IR_CODE_FIFO,
    IR_REPEAT_FIFO
} ir_fifo_t;

typedef void (ir_handler_function_t)(void);

typedef struct {
    uint16_t payload : 11;
    uint16_t length : 4;
    uint16_t type: 1;
} digital_message_t;

typedef struct {
    uint16_t duration : 14;
    uint16_t symbol_type : 1;
    uint16_t type: 1;
} paint_message_t;

typedef union {
    digital_message_t digital;
    paint_message_t paint;
    uint16_t raw;
} ir_message_t;

typedef struct {
    uint16_t ir_freq_carrier_period;
    uint16_t ir_logic_one_mark;
    uint16_t ir_logic_one_space;
    uint16_t ir_logic_zero_mark;
    uint16_t ir_logic_zero_space;
    uint16_t ir_repeat_time;
    ir_message_t message_fifo[MESSAGE_MAX_FIFO_LEVEL];
    ir_message_t repeat_fifo[REPEAT_MAX_FIFO_LEVEL];
    uint8_t fifo_level;
    uint8_t repeat_fifo_level;
    ir_handler_function_t *ir_irq_handle;
    union {
        struct {
            uint8_t IR_IRQ_EN : 1;
            uint8_t IR_LOGIC_ONE_FORMAT : 1;
            uint8_t IR_LOGIC_ZERO_FORMAT : 1;
            uint8_t IR_REPEAT_TYPE : 1;
            uint8_t IR_BUSY : 1;
            uint8_t IR_TX_REPEAT : 1;
            uint8_t IR_END : 1;
        } bits;
        uint8_t reg;
    } control;
} ir_driver_t;


/*
 * FUNCTION DECLARATIONS
******************************************************************************************/


/*****************************************************************************************
 * \brief IR driver initialization
******************************************************************************************/
void ir_init(void);

/*****************************************************************************************
 * \brief This function is used to reverse bit order. Required by RC5 protocol.
 *
 * \param[in] n: Value to reverse.
 * \param[in] bits: Define the number of value bits.
******************************************************************************************/
uint16_t ir_reverse_bit_order(uint16_t n, uint8_t bits);

/*****************************************************************************************
 * \brief Set carrier frequency
 *
 * \param[in] on_time:  Defines the carrier signal high duration in IR_clk cycles. 
 *                      0 is not allowed as a value.
 * \param[in]  period:  Defines the carrier signal period in IR_clk cycles. 0 is not 
 *                      allowed as a value.
 * \param[in]  trm_div: timer clock divider
******************************************************************************************/
void ir_set_carrier_freq(uint16_t on_time, uint16_t period, CLK_PER_REG_TMR_DIV_t trm_div);

/*****************************************************************************************
 * \brief Set logic one duration
 *
 * \param[in] logic_one_format: Defines the logic one format. IR_LOGIC_ONE_STARTS_MARK, 
 *                               IR_LOGIC_ONE_STARTS_SPACE
 * \param[in] mark:    Defines the mark duration in carrier clock cycles. Must be 
 *                     greater than 0.
 * \param[in] space:   Defines the space duration in carrier clock cycles. Must be 
 *                     greater than 0.
******************************************************************************************/
void ir_set_logic_one_time(uint8_t logic_one_format, uint8_t mark, uint8_t space);

/*****************************************************************************************
 * \brief Set logic zero duration
 *
 * \param[in] logic_zero_format:   Defines the logic zero format. 
 *                                 IR_LOGIC_ZERO_STARTS_MARK, IR_LOGIC_ZERO_STARTS_SPACE
 * \param[in] mark:   Defines the mark duration in carrier clock cycles. Must be 
 *                    greater than 0.
 * \param[in] space:  Defines the space duration in carrier clock cycles. Must be 
 *                    greater than 0.
******************************************************************************************/
void ir_set_logic_zero_time(uint8_t logic_zero_format, uint8_t mark, uint8_t space);

/*****************************************************************************************
 * \brief Set repeat time
 *
 * \param[in] time: Defines the repeat time in carrier clock cycles.
******************************************************************************************/
void ir_set_repeat_time(uint16_t time);

/*****************************************************************************************
 * \brief Set repeat command source
 *
 * \param[in] type: Defines the source of repeat command:
 *                    IR_REPEAT_FROM_CODE_FIFO, IR_REPEAT_FROM_REPEAT_FIFO
******************************************************************************************/
void ir_set_repeat_type(uint8_t type);

/*****************************************************************************************
 * \brief Insert paint message into main FIFO
 *
 * \param[in] fifo: Used to select FIFO. IR_CODE_FIFO, IR_REPEAT_FIFO
 * \param[in] symbol: Defines the message symbol. IR_PAINT_SYMBOL_TYPE_SPACE, IR_PAINT_SYMBOL_TYPE_MARK
 * \param[in] duration: Defines the mark/space duration in carrier clock cycles.
******************************************************************************************/
void ir_insert_paint_message(ir_fifo_t fifo, uint8_t symbol, uint16_t duration);

/*****************************************************************************************
 * \brief Insert digital message into main FIFO
 *
 * \param[in] fifo: Used to select FIFO. IR_CODE_FIFO, IR_REPEAT_FIFO
 * \param[in] length: Defines the number of valid bits minus 1. Range from 0 to 10.
 * \param[in] payload: Defines the digital message content.
******************************************************************************************/
void ir_insert_digital_message(ir_fifo_t fifo, uint8_t length, uint16_t payload);

/*****************************************************************************************
 * \brief  Enables ir_driver.control.bits.IR_IRQ
******************************************************************************************/
void ir_enable_irq(void);

/*****************************************************************************************
 * \brief  Disables ir_driver.control.bits.IR_IRQ
******************************************************************************************/
void ir_disable_irq(void);

/*****************************************************************************************
 * \brief  Start sending IR data
******************************************************************************************/
void ir_start(void);

/*****************************************************************************************
 * \brief  Stop sending IR data
******************************************************************************************/
void ir_stop(void);

/*****************************************************************************************
 * \brief  Flush code message FIFO
******************************************************************************************/
void ir_flush_code_fifo(void);

/*****************************************************************************************
 * \brief  Flush repeat message FIFO
******************************************************************************************/
void ir_flush_repeat_fifo(void);

/*****************************************************************************************
 * \brief  Registers a callback function to be called after TX completion.
 *
 * \param[in] handler: Handle to user function.
******************************************************************************************/
void ir_irq_register_callback(ir_handler_function_t *handler);

/*****************************************************************************************
 * \brief  Check busy status.
 *
 * \return true if busy or false if not.
******************************************************************************************/
bool ir_is_busy(void);

/**
 * \}
 * \}
 * \}
 */

#endif // _IR_DRIVER_H_
