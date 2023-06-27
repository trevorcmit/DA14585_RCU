/*****************************************************************************************
 * \file app_audio_defs.h
 * \brief Audio Application definitions
*****************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup APP_AUDIO
 * \brief Audio implementation
 * \{
******************************************************************************************/

#ifndef APP_AUDIO_DEFS_H_
#define APP_AUDIO_DEFS_H_

#include "stdbool.h"

typedef enum audio_sampling_rate {
        SAMPLING_RATE_16KHz = 0,
        SAMPLING_RATE_8KHz,
} audio_sampling_rate_t;
 
typedef enum app_audio_adpcm_mode {
        ADPCM_MODE_64KBPS_4_16KHZ = 0, // Do not change this value
        ADPCM_MODE_48KBPS_3_16KHZ = 1, // Do not change this value
        ADPCM_MODE_32KBPS_4_8KHZ  = 2, // Do not change this value
        ADPCM_MODE_24KBPS_3_8KHZ  = 3, // Do not change this value
        ADPCM_MODE_MIN,
} app_audio_adpcm_mode_t;

typedef enum app_audio_inband_cmd {
        AUDIO_CONTROL_RESET_CMD          = 0x00,
        AUDIO_CONTROL_SET_ADPCM_MODE_CMD = 0x10,
} app_audio_inband_cmd_t;

typedef void (*app_audio_start_sampling_t)(audio_sampling_rate_t sampling_rate);
typedef void (*app_audio_stop_sampling_t)(void);
typedef uint8_t (*app_audio_get_errors_t)(void);
typedef void (*app_audio_next_package_t)(void);
typedef void *(*app_audio_get_data_t)(void);
typedef bool (*app_audio_has_data_t)(void);

typedef struct {
    app_audio_start_sampling_t start_sampling;
    app_audio_stop_sampling_t  stop_sampling;
    app_audio_get_errors_t     get_errors;
    app_audio_next_package_t   next_package;
    app_audio_get_data_t       get_data;
    app_audio_has_data_t       has_data;
} audio_util_funcs_t;

#endif // APP_AUDIO_DEFS_H_

/**
 * \}
 * \}
 * \}
 */
