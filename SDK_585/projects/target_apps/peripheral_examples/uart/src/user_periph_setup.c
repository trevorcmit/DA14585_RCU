/**
 ****************************************************************************************
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
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "uart.h"
#include "gpio.h"
#include "user_periph_setup.h"

 /**
 ****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
 ****************************************************************************************
 */
void periph_init(void)
{
    // system init
    SetWord16(CLK_AMBA_REG, 0x00);                  // set clocks (hclk and pclk) 16MHz
    SetWord16(SET_FREEZE_REG, FRZ_WDOG);            // stop watch dog
    SetBits16(SYS_CTRL_REG, PAD_LATCH_EN, 1);       // open pads
    SetBits16(SYS_CTRL_REG, DEBUGGER_ENABLE, 1);    // open debugger
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);       // exit peripheral power down

    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));

    //Init pads
    GPIO_ConfigurePin(UART2_TX_GPIO_PORT, UART2_TX_GPIO_PIN, OUTPUT, PID_UART2_TX, false);
    GPIO_ConfigurePin(UART2_RX_GPIO_PORT, UART2_RX_GPIO_PIN, INPUT, PID_UART2_RX, false);

    // Initialize UART component
    SetBits16(CLK_PER_REG, UART2_ENABLE, 1);        // enable  clock for UART2
    uart2_init(UART2_BAUDRATE, UART2_FRAC_BAUDRATE, UART2_DATALENGTH);
}
