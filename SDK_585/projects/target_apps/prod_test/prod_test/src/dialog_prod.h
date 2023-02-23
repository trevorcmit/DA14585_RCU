/**
 ****************************************************************************************
 *
 * @file Dialog_prod.h
 *
 * @brief Dialog production header file.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
 ****************************************************************************************
 */
 
#ifndef _DIALOG_PROD_H_
#define _DIALOG_PROD_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "gpio.h"

extern volatile uint8_t test_state;
extern volatile uint8_t test_freq;
extern volatile uint8_t test_data_pattern;
extern volatile uint16_t test_tx_packet_nr;
extern volatile uint16_t text_tx_nr_of_packets;
extern volatile uint16_t test_rx_irq_cnt;

enum
{
    ///NO_TEST_RUNNING
    STATE_IDLE = 0x00,
    ///START_TX_TEST
    STATE_START_TX,            //1
    ///START_RX_TEST
    STATE_START_RX,            //2
    ///DIRECT_TX_TEST
    STATE_DIRECT_TX_TEST,      //3 activated via default hci command
    ///DIRECT_RX_TEST
    STATE_DIRECT_RX_TEST,      //4 activated via default hci command
    ///CONTINUE_TX
    STATE_START_CONTINUE_TX,   //5
    ///UNMODULATED_ON 
    STATE_UNMODULATED_ON,      //6
};

void app_init(void);
void otp_read(uint32_t otp_pos, uint8_t *val, uint8_t len);
int otp_write_words(uint32_t otp_pos, uint32_t *val_addr, uint8_t bytes_count);
void set_state_start_continue_tx(void);
void set_state_stop(void);
void set_state_start_tx(void);
void set_state_start_rx(void);
uint8_t check_uart_pins_cmd(uint8_t *ptr_data);
void change_uart_pins_cmd(uint8_t *ptr_data);
void sensor_spi_write_byte(uint8_t address, uint8_t wr_data, GPIO_PORT cs_port, GPIO_PIN cs_pin);
uint8_t sensor_spi_read_byte(uint8_t address, GPIO_PORT cs_port, GPIO_PIN cs_pin);
void sensor_i2c_write_byte(uint32_t address, uint8_t wr_data);
uint8_t sensor_i2c_read_byte(uint8_t address);
uint32_t find_mode_register(uint8_t port_number);
#endif // _DIALOG_PROD_H_
