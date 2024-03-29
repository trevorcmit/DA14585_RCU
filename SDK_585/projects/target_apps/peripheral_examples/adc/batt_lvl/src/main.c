/*****************************************************************************************
 *
 * @file main.c
 *
 * @brief Battery level example
 *
******************************************************************************************/
#include <stdio.h>
#include "common_uart.h"
#include "user_periph_setup.h"
#include "battery.h"
#include "gpio.h"

void system_init(void);
void batt_test(void);

/*****************************************************************************************
 * @brief Main routine of the battery level example
 *
******************************************************************************************/
int main (void)
{
    system_init();
    periph_init();
    batt_test();
    while(1);
}

 /*****************************************************************************************
 * @brief System Initialization
 *
******************************************************************************************/

void system_init(void)
{
    SetWord16(CLK_AMBA_REG, 0x00);                 // set clocks (hclk and pclk ) 16MHz
    SetWord16(SET_FREEZE_REG,FRZ_WDOG);            // stop watch dog
    SetBits16(SYS_CTRL_REG,PAD_LATCH_EN,1);        // open pads
    SetBits16(SYS_CTRL_REG,DEBUGGER_ENABLE,1);     // open debugger
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP,0);       // exit peripheral power down
}

/*****************************************************************************************
 * @brief Battery Level Indication example function
 *
******************************************************************************************/
void batt_test(void)
{
    printf_string("\n\r\n\r");
    printf_string("*******************\n\r");
    printf_string("* 3V BATTERY TEST *\n\r");
    printf_string("*******************\n\r");

#if BATT_CR2032
    printf_string("\n\rBattery type: CR2032");
    printf_string("\n\rCurrent battery level (%): ");
    printf_byte_dec(battery_get_lvl(BATT_CR2032));
    printf_string("% left");
#else
    printf_string("\n\rBattery type unknown");
#endif
    printf_string("\n\rEnd of test\n\r");
}
