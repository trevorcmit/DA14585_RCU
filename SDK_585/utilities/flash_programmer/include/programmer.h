/*****************************************************************************************
 *
 * @file programmer.h
 *
 * @brief Programmer functions
  *
******************************************************************************************/

#include <stdint.h>

#ifndef __PROGRAMMER_INCLUDED__
#define __PROGRAMMER_INCLUDED__

void print_menu(void);
void print_input(void);
void endtest_bridge(short int *idx);
void exit_test(void);
uint32_t crc32(uint32_t crc, const void *buf, size_t size);
#endif
