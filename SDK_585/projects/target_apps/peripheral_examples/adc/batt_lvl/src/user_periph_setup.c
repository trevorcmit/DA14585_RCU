/*****************************************************************************************
 *
 * @file periph_setup.c
 *
 * @brief Peripherals initialization functions
 *
 * Copyright (C) 2012 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include "uart.h"
#include "gpio.h"
#include "user_periph_setup.h"

 /*****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
******************************************************************************************/
void periph_init(void)
{
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, PERIPH_SLEEP_ENABLE);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));

    // Init pads
    GPIO_ConfigurePin(UART2_TX_GPIO_PORT, UART2_TX_GPIO_PIN, OUTPUT, PID_UART2_TX, false);
    GPIO_ConfigurePin(UART2_RX_GPIO_PORT, UART2_RX_GPIO_PIN, INPUT, PID_UART2_RX, false);
    SetBits16(CLK_PER_REG, UART2_ENABLE, 1);

    // Initialize UART component
    uart2_init(UART2_BAUDRATE, UART2_FRAC_BAUDRATE, UART2_DATALENGTH);
}
