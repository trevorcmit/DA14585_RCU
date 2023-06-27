/*****************************************************************************************
 *
 * @file rdtest_support.h
 *
 * @brief RD test support functions header file.
 *
******************************************************************************************/

#ifndef _RDTEST_SUPPORT_H_
#define _RDTEST_SUPPORT_H_ 

/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Delay
 *
 * This function count down from delay down to 0. Core clock is XTAL16 and HCLK_DIV=0.
 * So clock Cortex-M0 is 16Mhz AND 1 clock cycle lasts 1/16 usec = 0.0625 usec
 *
 * @param[in] delay
 *
 * @return void
 *****************************************************************************************
 */
void delay_systick_us(unsigned long int delay);

#endif // _RDTEST_SUPPORT_H_
