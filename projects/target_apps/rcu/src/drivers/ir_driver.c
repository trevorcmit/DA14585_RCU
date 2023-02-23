/**
 ****************************************************************************************
 *
 * \file    ir_driver.c
 *
 * \brief   IR Driver functions.
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
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup IR_DRV
 *
 * \{
 ****************************************************************************************
 */
 
#include <app_ir_config.h>
#include "ir_driver.h"
#include "uart.h"

#define __RETAINED __attribute__((section("retention_mem_area0"), zero_init))

/*
 * LOCAL VARIABLES
 ****************************************************************************************
 */

ir_driver_t ir_driver __RETAINED;
uint8_t timer_state __RETAINED;
uint8_t msg_cnt, bits_cnt, bit_state __RETAINED;
uint16_t msg_time __RETAINED;
uint16_t msg_repeat_cnt __RETAINED;

uint8_t new_ble_evt_end_detected __RETAINED;
uint8_t last_rwble_evt __RETAINED;
uint8_t one_before_last_rwble_evt __RETAINED;

#if IR_ENABLE_DEBUG_GPIO == 1
/**
 ****************************************************************************************
 * \brief Toggle pin. Use only for debug purposes.
 * \param[in] port
 * \param[in] pin
 ****************************************************************************************
 */
static void GPIO_PinToggle(GPIO_PORT port, GPIO_PIN pin)
{
    if (GPIO_GetPinStatus(port, pin))
		{
        GPIO_SetInactive(port, pin);
		}
    else
		{
        GPIO_SetActive(port, pin);
		}
}
#endif


uint16_t ir_reverse_bit_order(uint16_t n, uint8_t bits)
{
    uint16_t nrev, N;
    uint8_t count;
    N = 1 << bits;

    count = bits - 1;
    nrev = n;
    for (n >>= 1; n; n >>= 1)
    {
        nrev <<= 1;
        nrev |= n & 1;
        count--;
    }

    nrev <<= count;
    nrev &= N - 1;

    return nrev;
}


/**
 ****************************************************************************************
 * \brief Disable PWM output.
 ****************************************************************************************
 */
static void ir_disable_output(void)
{
    PORT_CONFIGURE_GPIO(app_ir_pins[IR_PIN], OUTPUT, PID_GPIO, false);

#if IR_ENABLE_DEBUG_GPIO == 1
    GPIO_SetInactive(IR_OUT_SIG_GPIO_PORT, IR_OUT_SIG_GPIO_PIN);
#endif
}


/**
 ****************************************************************************************
 * \brief Enable PWM output.
 ****************************************************************************************
 */
static void ir_enable_output(void)
{
    PORT_SET_PIN_FUNCTION(app_ir_pins[IR_PIN], OUTPUT, PID_PWM0);

#if IR_ENABLE_DEBUG_GPIO == 1
    GPIO_SetActive(IR_OUT_SIG_GPIO_PORT, IR_OUT_SIG_GPIO_PIN);
#endif
}


/**
 ****************************************************************************************
 * \brief Set timer0 time for the next message.
 * \param[in] cnt: Define the message position in FIFO.
 * \param[in] shift: Define the number of bit shift for message data.
 ****************************************************************************************
 */
static void ir_set_irq_next_time(uint8_t cnt, uint8_t shift)
{
    ir_message_t message;

    if (ir_driver.control.bits.IR_TX_REPEAT && ir_driver.control.bits.IR_REPEAT_TYPE == IR_REPEAT_FROM_REPEAT_FIFO)
    {
        if (cnt >= ir_driver.repeat_fifo_level) return;
        message = ir_driver.repeat_fifo[cnt];
    }
    else
    {
        if (cnt >= ir_driver.fifo_level) return;
        message = ir_driver.message_fifo[cnt];
    }

    if (ir_driver.message_fifo[cnt].digital.type == IR_MESSAGE_TYPE_PAINT)
    {
        timer0_set_pwm_on_counter(message.paint.duration);
    }
    else
    {
        if (ir_driver.message_fifo[cnt].digital.payload >> shift & 0x01)
        {
            //Logic 1
            if (ir_driver.control.bits.IR_LOGIC_ONE_FORMAT == IR_LOGIC_ONE_STARTS_MARK)
                timer0_set_pwm_on_counter(ir_driver.ir_logic_one_mark);
            else
                timer0_set_pwm_on_counter(ir_driver.ir_logic_one_space);
        }
        else
        {
            //Logic 0
            if (ir_driver.control.bits.IR_LOGIC_ZERO_FORMAT == IR_LOGIC_ZERO_STARTS_MARK)
                timer0_set_pwm_on_counter(ir_driver.ir_logic_zero_mark);
            else
                timer0_set_pwm_on_counter(ir_driver.ir_logic_zero_space);
        }
    }
}


/**
 ****************************************************************************************
 * \brief Set timer0 time for next data bit.
 * \param[in] message: Define the code or repeat message
 ****************************************************************************************
 */
static void ir_set_irq_next_bit_time(ir_message_t message)
{
    if (message.digital.payload & 0x01)
    {
        //Logic 1
        if (ir_driver.control.bits.IR_LOGIC_ONE_FORMAT == IR_LOGIC_ONE_STARTS_MARK)
            timer0_set_pwm_on_counter(ir_driver.ir_logic_one_space);
        else
            timer0_set_pwm_on_counter(ir_driver.ir_logic_one_mark);
    }
    else
    {
        //Logic 0
        if (ir_driver.control.bits.IR_LOGIC_ZERO_FORMAT == IR_LOGIC_ZERO_STARTS_MARK)
            timer0_set_pwm_on_counter(ir_driver.ir_logic_zero_space);
        else
            timer0_set_pwm_on_counter(ir_driver.ir_logic_zero_mark);
    }
}


/**
 ****************************************************************************************
 * \brief Stop Timer0 and disable interrupt.
 ****************************************************************************************
 */
static void ir_stop_timer(void)
{
    timer0_stop();
    timer0_disable_irq();
    ir_disable_output();
    NVIC_ClearPendingIRQ(SWTIM_IRQn);
    set_tmr_enable(CLK_PER_REG_TMR_DISABLED);
    ir_driver.control.bits.IR_BUSY = 0;
    arch_restore_sleep_mode();
}


/**
 ****************************************************************************************
 * \brief Timer0 callback function.
 ****************************************************************************************
 */
void ir_timer_callback(void)
{
    ir_message_t message;
    uint8_t msg_type, fifo_level;

#if IR_ENABLE_DEBUG_GPIO == 1
    GPIO_SetActive(IR_IRQ_SIG_GPIO_PORT, IR_IRQ_SIG_GPIO_PIN);
    GPIO_SetInactive(IR_IRQ_SIG_GPIO_PORT, IR_IRQ_SIG_GPIO_PIN);
#endif

    if(ir_params.use_ble_sync) {
        one_before_last_rwble_evt = last_rwble_evt;
        //Get last BLE event
        last_rwble_evt = port_get_last_ble_event();

        if ( (one_before_last_rwble_evt != BLE_EVT_END) &&
                 (last_rwble_evt == BLE_EVT_END) )
        {
                new_ble_evt_end_detected = true;
        }
        else
        {
                new_ble_evt_end_detected = false;
        }
    }
    //Timer state machine
    switch (timer_state)
    {
        //Waiting for the BLE event end
        case IR_TIMER_WAIT_FOR_BLE_END:
            if(ir_params.use_ble_sync) {
                //If end of BLE event was detected, set timer state machine to send command
                if (new_ble_evt_end_detected)
                    timer_state = IR_TIMER_COMMAND;
                //Check for user request to stop the IR
                if (ir_driver.control.bits.IR_END == 1)
                {
                    //Stop IR timer and return
                    ir_stop_timer();
                    return;
                }
            }
            break;
        //Sending IR data
        case IR_TIMER_COMMAND:
            #if IR_ENABLE_DEBUG_GPIO == 1
            GPIO_SetInactive(IR_WAIT_SIG_GPIO_PORT, IR_WAIT_SIG_GPIO_PIN);
            #endif
            //Get message type and message data
            //If a repeat message is to be sent, check the message source: code or repeat FIFO.
            if (ir_driver.control.bits.IR_TX_REPEAT && ir_driver.control.bits.IR_REPEAT_TYPE == IR_REPEAT_FROM_REPEAT_FIFO)
            {
                message = ir_driver.repeat_fifo[msg_cnt];
                msg_type = ir_driver.repeat_fifo[msg_cnt].digital.type;
                fifo_level = ir_driver.repeat_fifo_level;
            }
            else
            {
                message = ir_driver.message_fifo[msg_cnt];
                msg_type = ir_driver.message_fifo[msg_cnt].digital.type;
                fifo_level = ir_driver.fifo_level;
            }
            //Add current timer value to calculate total message time.
            //Required for calculating repeat message timeout.
            msg_time += GetWord16(TIMER0_ON_REG);

            //If message type is "paint", send the paint message.
            if (msg_type == IR_MESSAGE_TYPE_PAINT)
            {
                if (message.paint.symbol_type == IR_PAINT_SYMBOL_TYPE_MARK)
                    ir_enable_output();
                else
                    ir_disable_output();

                //Load new value for the timer.
                ir_set_irq_next_time(msg_cnt + 1, 0);
                //Increment message counter
                msg_cnt++;
            }
            else
            {
                //Shift message data left to get the bit which will be sent.
                message.digital.payload >>= bits_cnt;

                //Each bit is sent in two parts
                switch (bit_state)
                {
                    //Sending first part of bit
                    case IR_BIT_STATE_FIRST_PART:
                        #if IR_ENABLE_DEBUG_GPIO == 1
                            GPIO_PinToggle(IR_BIT_SIG_GPIO_PORT, IR_BIT_SIG_GPIO_PIN);
                        #endif
                        //Check bit value
                        if (message.digital.payload & 0x01)
                        {
                            //If bit is set to 1 send it
                            if (ir_driver.control.bits.IR_LOGIC_ONE_FORMAT == IR_LOGIC_ONE_STARTS_MARK)
                                ir_enable_output();
                            else
                                ir_disable_output();
                        }
                        else
                        {
                            //If bit is set to 0 send it
                            if (ir_driver.control.bits.IR_LOGIC_ZERO_FORMAT == IR_LOGIC_ZERO_STARTS_MARK)
                                ir_enable_output();
                            else
                                ir_disable_output();
                        }

                        bit_state = IR_BIT_STATE_SECOND_PART;
                        //Load new bit time value for the timer
                        ir_set_irq_next_bit_time(message);
                        break;
                    //Sending second part of bit
                    case IR_BIT_STATE_SECOND_PART:
                        //Check bit value
                        if (message.digital.payload & 0x01)
                        {
                            //If bit is set to 1 send it
                            if (ir_driver.control.bits.IR_LOGIC_ONE_FORMAT == IR_LOGIC_ONE_STARTS_MARK)
                                ir_disable_output();
                            else
                                ir_enable_output();
                        }
                        else
                        {
                            //If bit is set to 0 send it
                            if (ir_driver.control.bits.IR_LOGIC_ZERO_FORMAT == IR_LOGIC_ZERO_STARTS_MARK)
                                ir_disable_output();
                            else
                                ir_enable_output();
                        }
                        //Increment bit counter
                        bits_cnt++;
                        bit_state = IR_BIT_STATE_FIRST_PART;
                        //Load new value for the timer.
                        ir_set_irq_next_time(msg_cnt, bits_cnt);
                        break;
                }
                //Check whether all message bits have been sent.
                if (bits_cnt >= message.digital.length)
                {
                    //If all bits are sent, clear bits counter
                    bits_cnt = 0;
                    //Load new value for the timer.
                    ir_set_irq_next_time(msg_cnt + 1, 0);
                    //Increment message counter.
                    msg_cnt++;
                }
            }
            //Check if all messages in the FIFO are transmitted
            if (msg_cnt >= fifo_level)
            {
                //Check if user would like to stop the IR
                if (ir_driver.control.bits.IR_END == 1)
                {
                    //Stop IR timer and return
                    ir_stop_timer();
                    return;
                }
                //Set state machine to wait for repeat state
                timer_state = IR_TIMER_WAIT_FOR_REPEAT;
                //Calculate the repeat delay and set the timer
                timer0_set_pwm_on_counter(ir_driver.ir_repeat_time - msg_time);
                //Set the IR_TX_REPEAT control bit to inform the driver that the next message will be 'repeat' message
                ir_driver.control.bits.IR_TX_REPEAT = 1;
                //Clear counter
                msg_time = 0;
                msg_cnt = 0;
                bits_cnt = 0;
                bit_state = 0;
                //Check wheather the user has registred and enabled the send complete interrupt
                if (ir_driver.control.bits.IR_IRQ_EN == 1 && ir_driver.ir_irq_handle != NULL)
								{
                    //Call the registered function
                    ir_driver.ir_irq_handle();
								}
                return;
            }
            break;
        //Repeat delay expired
        case IR_TIMER_WAIT_FOR_REPEAT:
            //Increment repeat message conter
            msg_repeat_cnt++;
            //Check that the maximum number of repeat messages has been sent or user has requested to stop sending
            if (msg_repeat_cnt >= ir_params.max_repeat || ir_driver.control.bits.IR_END == 1)
            {
                //Stop the IR timer and return
                ir_stop_timer();
                return;
            }
            ir_disable_output();
            #if IR_ENABLE_DEBUG_GPIO == 1
            GPIO_SetActive(IR_WAIT_SIG_GPIO_PORT, IR_WAIT_SIG_GPIO_PIN);
            #endif
            if(ir_params.use_ble_sync) {
                timer_state = IR_TIMER_WAIT_FOR_BLE_END;
            }
            else {
                timer_state = IR_TIMER_COMMAND;
            }
            msg_time = 0;
            //Load new value for the timer
            ir_set_irq_next_time(0, 0);
            break;
    }
}


void ir_init(void)
{
    ir_disable_output();
    memset(&ir_driver, 0, sizeof(ir_driver));

#if IR_ENABLE_DEBUG_GPIO == 1
    GPIO_ConfigurePin(IR_IRQ_SIG_GPIO_PORT, IR_IRQ_SIG_GPIO_PIN, OUTPUT, PID_GPIO, false);
    GPIO_ConfigurePin(IR_OUT_SIG_GPIO_PORT, IR_OUT_SIG_GPIO_PIN, OUTPUT, PID_GPIO, false);
    GPIO_ConfigurePin(IR_BIT_SIG_GPIO_PORT, IR_BIT_SIG_GPIO_PIN, OUTPUT, PID_GPIO, false);
    GPIO_ConfigurePin(IR_WAIT_SIG_GPIO_PORT, IR_WAIT_SIG_GPIO_PIN, OUTPUT, PID_GPIO, false);
#endif
}


void ir_set_carrier_freq(uint16_t on_time, uint16_t period, CLK_PER_REG_TMR_DIV_t trm_div)
{
    ASSERT_WARNING(on_time > 0);
    ASSERT_WARNING((period - on_time) > 0);
    
    ir_driver.ir_freq_carrier_period = period;
    set_tmr_div(trm_div);
    timer0_set_pwm_high_counter(on_time - 1);
    timer0_set_pwm_low_counter(period - on_time - 1);
}


void ir_set_logic_one_time(uint8_t logic_one_format, uint8_t mark, uint8_t space)
{
    if (mark == 0) mark = 1;
    if (space == 0) space= 1;

    ir_driver.ir_logic_one_mark = (ir_driver.ir_freq_carrier_period * mark - 5) / 10;   // +5 must be added before the division by 10 to round the 
                                                                                        // result to the nearest integer. (a+5)/10-1 => (a-5)/10
    ir_driver.ir_logic_one_space = (ir_driver.ir_freq_carrier_period * space - 5) / 10;
    ir_driver.control.bits.IR_LOGIC_ONE_FORMAT = logic_one_format;
}


void ir_set_logic_zero_time(uint8_t logic_zero_format, uint8_t mark, uint8_t space)
{
    if (mark == 0) mark = 1;
    if (space == 0) space= 1;

    ir_driver.ir_logic_zero_mark = (ir_driver.ir_freq_carrier_period * mark - 5) / 10;
    ir_driver.ir_logic_zero_space = (ir_driver.ir_freq_carrier_period * space - 5) / 10;
    ir_driver.control.bits.IR_LOGIC_ZERO_FORMAT = logic_zero_format;
}


void ir_set_repeat_time(uint16_t time)
{
    ir_driver.ir_repeat_time = (ir_driver.ir_freq_carrier_period * time - 5) / 10;
}


void ir_set_repeat_type(uint8_t type)
{
    ir_driver.control.bits.IR_REPEAT_TYPE = type;
}


void ir_insert_paint_message(ir_fifo_t fifo, uint8_t symbol, uint16_t duration)
{
    if (fifo == IR_CODE_FIFO)
    {
        if (ir_driver.fifo_level >= MESSAGE_MAX_FIFO_LEVEL) return;
        ir_driver.message_fifo[ir_driver.fifo_level].paint.type = IR_MESSAGE_TYPE_PAINT;
        ir_driver.message_fifo[ir_driver.fifo_level].paint.duration = (ir_driver.ir_freq_carrier_period * duration - 5) / 10;
        ir_driver.message_fifo[ir_driver.fifo_level].paint.symbol_type = symbol;
        ir_driver.fifo_level++;
    }
    else
    {
        if (ir_driver.repeat_fifo_level >= REPEAT_MAX_FIFO_LEVEL) return;
        ir_driver.repeat_fifo[ir_driver.repeat_fifo_level].paint.type = IR_MESSAGE_TYPE_PAINT;
        ir_driver.repeat_fifo[ir_driver.repeat_fifo_level].paint.duration = (ir_driver.ir_freq_carrier_period * duration - 5) / 10;
        ir_driver.repeat_fifo[ir_driver.repeat_fifo_level].paint.symbol_type = symbol;
        ir_driver.repeat_fifo_level++;
    }
}


void ir_insert_digital_message(ir_fifo_t fifo, uint8_t length, uint16_t payload)
{
    if (fifo == IR_CODE_FIFO)
    {
        if (ir_driver.fifo_level >= MESSAGE_MAX_FIFO_LEVEL) return;
        ir_driver.message_fifo[ir_driver.fifo_level].digital.type = IR_MESSAGE_TYPE_DIGITAL;
        ir_driver.message_fifo[ir_driver.fifo_level].digital.length = length;
        ir_driver.message_fifo[ir_driver.fifo_level].digital.payload = payload;
        ir_driver.fifo_level++;
    }
    else
    {
        if (ir_driver.repeat_fifo_level >= REPEAT_MAX_FIFO_LEVEL) return;
        ir_driver.repeat_fifo[ir_driver.repeat_fifo_level].digital.type = IR_MESSAGE_TYPE_DIGITAL;
        ir_driver.repeat_fifo[ir_driver.repeat_fifo_level].digital.length = length;
        ir_driver.repeat_fifo[ir_driver.repeat_fifo_level].digital.payload = payload;
        ir_driver.repeat_fifo_level++;
    }
}


void r_enable_irq(void)
{
    ir_driver.control.bits.IR_IRQ_EN = 1;
}


void ir_disable_irq(void)
{
    ir_driver.control.bits.IR_IRQ_EN = 0;
}


void ir_start(void)
{
    if (ir_driver.fifo_level == 0) {
        return; // Nothing to transmit
    }
    
    set_tmr_enable(CLK_PER_REG_TMR_ENABLED);
    timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_DIV_BY_10);
    timer0_register_callback(ir_timer_callback);
    timer0_enable_irq();

    ir_driver.control.bits.IR_TX_REPEAT = 0;
    ir_driver.control.bits.IR_END = 0;

    msg_repeat_cnt = 0;

    // config timer for first message
    if (ir_driver.message_fifo[0].digital.type == IR_MESSAGE_TYPE_PAINT)
    {
        timer0_set_pwm_on_counter(ir_driver.message_fifo[0].paint.duration);
    }
    else
    {
        if (ir_driver.message_fifo[0].digital.payload & 0x01)
        {
            timer0_set_pwm_on_counter(ir_driver.ir_logic_one_mark);
        }
        else
        {
            timer0_set_pwm_on_counter(ir_driver.ir_logic_zero_mark);
        }
    }
    if(ir_params.use_ble_sync) {
        timer_state = IR_TIMER_WAIT_FOR_BLE_END;
    }
    else {
        timer_state = IR_TIMER_COMMAND;
    }
    
    ir_driver.control.bits.IR_BUSY = 1;
    timer0_start();
}


void ir_stop(void)
{
    ir_driver.control.bits.IR_END = 1;
}


void ir_flush_code_fifo(void)
{
    ir_driver.fifo_level = 0;
    memset(&ir_driver.message_fifo, 0, sizeof(ir_message_t) * MESSAGE_MAX_FIFO_LEVEL);
}


void ir_flush_repeat_fifo(void)
{
    ir_driver.repeat_fifo_level = 0;
    memset(&ir_driver.repeat_fifo, 0, sizeof(ir_message_t) * REPEAT_MAX_FIFO_LEVEL);
}


void ir_irq_register_callback(ir_handler_function_t *handler)
{
    if (ir_driver.ir_irq_handle == NULL)
        ir_driver.ir_irq_handle = handler;
}


bool ir_is_busy(void)
{
    return ir_driver.control.bits.IR_BUSY;
}

/**
 * \}
 * \}
 * \}
 */
