/*****************************************************************************************
 *
 * @file peripherals.h
 *
 * @brief Peripheral initialization function prototypes
 *
******************************************************************************************/

#include <stdint.h>
#include "gpio.h"

#ifndef PERIPHERAL_H_INCLUDED
#define PERIPHERAL_H_INCLUDED

void update_uart_pads(GPIO_PORT port, GPIO_PIN tx_pin, GPIO_PIN rx_pin);
void set_pad_uart(void);
void set_pad_spi(void);     
void set_pad_eeprom(void);  

void periph_init(void);
 
#endif
