/*****************************************************************************************
 *
 * @file stdbool.h
 *
 * @brief stdbool.h implementation for C compilers that do not support this header.
 *
******************************************************************************************/

#ifndef STDBOOL_H_
#define STDBOOL_H_

#ifndef __cplusplus
    #define bool char
    #define true 1
    #define false 0
#endif

#endif // STDBOOL_H_
