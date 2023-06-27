/*****************************************************************************************
 *
 * @file trng.h
 *
 * @brief True Random Number Generator header file.
  *
******************************************************************************************/

#ifndef _TRNG_H_
#define _TRNG_H_

/*
 * FUNCTION DECLARATIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief  Acquires 128 bits, saves them in trng_bits[] and restores the modified regs.
 * @param[in] trng_bits_ptr Pointer to number
 * @return void
******************************************************************************************/
void trng_acquire(uint8_t *trng_bits_ptr);

#endif // _TRNG_H_
