/*****************************************************************************************
 *
 * @file main.c
 *
 * @brief UART example
 *
******************************************************************************************/

#include <stdio.h>
#include "common_uart.h"
#include "user_periph_setup.h"
#include "gpio.h"

void system_init(void);
void uart_test(void);

/*****************************************************************************************
 * @brief Main routine of the UART example
 *
******************************************************************************************/
int main (void)
{
    system_init();
    periph_init();
    uart_test();
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
 * @brief UART test function
 *
******************************************************************************************/
void uart_test(void)
{  
    printf_string("\n\r\n\r*************");
    printf_string("\n\r* UART TEST *\n\r");
    printf_string("*************\n\r");
    printf_string("\n\rHello World! == UART printf_string()\n\r");
    printf_string("\n\rUART print_hword() = 0x");
    print_hword(0xAABB);
    printf_string("\n\rUART print_word() = 0x");
    print_word(0x11223344);
    printf_string("\n\rEnd of test\n\r");
}
