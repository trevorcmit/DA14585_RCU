/*****************************************************************************************
 *
 * @file app_security.c
 *
 * @brief Application Security Entry Point
 *
 * Copyright (C) RivieraWaves 2009-2013
 * Copyright (C) 2017 Modified by Dialog Semiconductor
 *
******************************************************************************************/

/*****************************************************************************************
 * @addtogroup APP_SECURITY
 * @{
******************************************************************************************/

/*
 * INCLUDE FILES
******************************************************************************************/

#include "co_bt.h"

#include "app_api.h"            // Application task Definition
#include "app_security.h"       // Application Security API Definition
#include <stdlib.h>

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

/// Application Security Environment Structure
struct app_sec_env_tag app_sec_env[APP_EASY_MAX_ACTIVE_CONNECTION] __attribute__((section("retention_mem_area0"),zero_init)); // @RETENTION MEMORY

/*
 * FUNCTION DEFINITIONS
******************************************************************************************/

/*****************************************************************************************
 * @brief Fills an array with random bytes (starting from the end of the array)
 *          Assuming an array of array_size and a dynamic key size, so that key_size = M*4+N:
 *          Calls rand() M times and appends the 4 bytes of each 32 bit return value to the output array
 *          Calls rand() once and appends the N most significant bytes of the 32 bit return value to the output array
 *          Fills the rest bytes of the array with zeroes
 *
 * @param[in] *dst              The address of the array
 * @param[in] key_size          Number of bytes that shall be randomized
 * @param[in] array_size        Total size of the array
 *
 * @return void
******************************************************************************************/
static void fill_random_byte_array(uint8_t *dst, uint8_t key_size, uint8_t array_size)
{
    uint32_t rand_val;
    uint8_t rand_bytes_cnt = 0;
    uint8_t remaining_bytes;
    uint8_t i;

    // key_size must not be greater than array_size
    ASSERT_ERROR(array_size>=key_size);

    // key_size = M*4+N
    // Randomize the M most significant bytes of the array
    for (i = 0; i < key_size-3; i+=4)
    {
        // Calculate a random 32 bit value
        rand_val = rand();
        // Assign each of the 4 rand bytes to the byte array
        dst[array_size - (i+0)-1] = (rand_val >> 24) & 0xFF;
        dst[array_size - (i+1)-1] = (rand_val >> 16) & 0xFF;
        dst[array_size - (i+2)-1] = (rand_val >> 8) & 0xFF;
        dst[array_size - (i+3)-1] = (rand_val >> 0) & 0xFF;
        // Count randomized bytes
        rand_bytes_cnt += 4;
    }

    // Randomize the remaining N most significant bytes of the array. (Max N = 3)
    remaining_bytes = key_size - rand_bytes_cnt;
    if (remaining_bytes)
    {
        rand_val = rand();
        for (i = 0; i < remaining_bytes; i++)
        {
            dst[array_size -(rand_bytes_cnt+i)-1] = (rand_val >> ((3-i)<<3)) & 0xFF;
        }
    }

    // Fill the least significant bytes of the array with zeroes
    remaining_bytes = array_size - key_size;
    for (i = 0; i < remaining_bytes; i++)
    {
        dst[array_size - (key_size + i)-1] = 0;
    }
}

uint32_t app_sec_gen_tk(void)
{
    // Generate a PIN Code (Between 100000 and 999999)
    return (100000 + (rand()%900000));
}

void app_sec_gen_ltk(uint8_t conidx, uint8_t key_size)
{
    app_sec_env[conidx].key_size = key_size;

    // Randomly generate the LTK and the Random Number
    fill_random_byte_array(app_sec_env[conidx].rand_nb.nb, RAND_NB_LEN, RAND_NB_LEN);

    // Randomly generate the end of the LTK
    fill_random_byte_array(app_sec_env[conidx].ltk.key, key_size, KEY_LEN);

    // Randomly generate the EDIV
    app_sec_env[conidx].ediv = rand()%65536;
}

void app_sec_gen_csrk(uint8_t conidx)
{
    // Randomly generate the CSRK
    fill_random_byte_array(app_sec_env[conidx].csrk.key, KEY_LEN, KEY_LEN);
}

/// @} APP_SECURITY
