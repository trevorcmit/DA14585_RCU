/*****************************************************************************************
 * \file app_audio_config.h
 * \brief  Audio application configuration header file
*****************************************************************************************/
#ifndef _APP_AUDIO_CONFIG_H_
#define _APP_AUDIO_CONFIG_H_

#include "app_audio_defs.h"
#include "port_audio.h"
#include "port_platform.h"
#include "user_rcu_audio.h"


/*****************************************************************************************
 * Define CFG_AUDIO_ADAPTIVE_RATE to enable dynamic sampling rate and ADPCM configuration changing.
 * Define CFG_AUDIO_IMA_ADPCM to use IMA ADPCM encoding. If not defined aLaw encoding will be used.
 * When CFG_AUDIO_DC_BLOCK is define, DC blocking filter is enabled.                             
******************************************************************************************/
#define CFG_AUDIO_ADAPTIVE_RATE
#define CFG_AUDIO_IMA_ADPCM
#define CFG_AUDIO_DC_BLOCK


/*****************************************************************************************
 * 0: 64 Kbit/s = ima 4Bps, 16 Khz.                                                  
 * 1: 48 Kbit/s = ima 3Bps, 16 Khz.                                                  
 * 2: 32 Kbit/s = ima 4Bps, 8 Khz (downsample).                                      
 * 3: 24 Kbit/s = ima 3Bps, 8 Khz (downsample).                                      
 * Do not use enum. Enums are not recognized by the preprocessor                     
******************************************************************************************/
#define ADPCM_DEFAULT_MODE 0      


/**************************************************************************************************
 * \brief When CFG_AUDIO_ENABLE_DC_BLOCK_FADING is defined fading in DC blocking filter is enabled
**************************************************************************************************/
// #define CFG_AUDIO_ENABLE_DC_BLOCK_FADING


/*****************************************************************************************
 * \brief When CFG_AUDIO_CLICK_STARTUP_CLEAN is defined the DC_BLOCK filter is enabled
 *        a number of packages is dropped at the beginning of the audio sampling
******************************************************************************************/
// #define CFG_AUDIO_CLICK_STARTUP_CLEAN


/****************************************************************************************
 * \brief CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE is defined if port_audio supports
 *        configurable 8KHz or 16KHz sampling rate. If not defined then 16KHz samples
 *        will be downsampled when ADPCM_MODE_32KBPS_4_8KHZ or ADPCM_MODE_24KBPS_3_8KHZ 
 *        is used.
*****************************************************************************************/
#define CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE


/*****************************************************************************************
 * \brief CFG_AUDIO_USE_32BIT_SAMPLING is defined when audio samples are 32bit instead
 *        of 16bit long. CFG_AUDIO_SAMPLING_OFFSET will be used to define which bits of 
 *        the 32bit word will be used to convert the sample from 32 to 16 bits.
******************************************************************************************/
#define CFG_AUDIO_USE_32BIT_SAMPLING


/****************************************************************************************
* \brief CFG_AUDIO_USE_32BIT_SAMPLING is defined when audio samples are 32bit instead
*        of 16bit long. AUDIO_SAMPLING_OFFSET and will be used to define which bits of 
*        the 32bit word will be used to convert the sample from 32 to 16 bits.
*****************************************************************************************/
#define CFG_AUDIO_USE_32BIT_SAMPLING


/*****************************************************************************************
 * \brief When CFG_AUDIO_USE_32BIT_SAMPLING is defined, AUDIO_SAMPLING_OFFSET defines
 *        the position of the sample in the 32bit word. e.g. if audio data are located
 *        at bit31..8 then AUDIO_SAMPLING_OFFSET must be defined to 8.
 *        Only 16 bits of the samples will be used. If a number of LSB must be discarded 
 *        to attenuate the signal then this number must be added to AUDIO_SAMPLING_OFFSET
*****************************************************************************************/
#define AUDIO_SAMPLING_OFFSET  (8+6)  // 8: 24bit samples are left aligned
                                      // 6: 6 LSB are discarded to attenuate the 
                                      //    signal. Bit21..6 are used


/****************************************************************************************
 * \brief Number of samples to get from the audio hardware peripheral. All samples are 
 *        read at once and form an audio slot which is stored in the audio buffer. 
 *        CPU will be interrupted every AUDIO_NR_SAMP_PER_SLOT*[sampling time] to get 
 *        the audio samples from the hardware peripheral. e.g. If AUDIO_NR_SAMP_PER_SLOT
 *        is 40 and sampling rate is 16KHz then the CPU will be interrupted every 40*1/16000 = 2.5msec
*****************************************************************************************/
#define AUDIO_NR_SAMP_PER_SLOT  40


/****************************************************************************************
 * \brief Number of audio slots that can be stored in the audio buffer. The size of the
 *        audio buffer is AUDIO_BUFFER_NR_SLOTS*AUDIO_NR_SAMP_PER_SLOT.
 *        The size of the buffer must be fine tuned according to the maximum time needed
 *        by the system to get and process the audio samples of the slot.
*****************************************************************************************/
#define AUDIO_BUFFER_NR_SLOTS   3


/****************************************************************************************
*  \brief Local buffer size for storing 16-bit audio samples. This buffer must be large 
*          enough to hold all the samples needed for one pass of the audio encoder. 
*****************************************************************************************/
#define AUDIO_SBUF_SIZE 88
//#define AUDIO_SBUF_SIZE 380 // This is the size when CFG_APP_STREAM_PACKET_BASED is defined 


/**************************************************************************
* \brief Emulate waveforms. This is used to test the audio interface.                                                            
**************************************************************************/
// #define CFG_AUDIO_EMULATE_PDM_MIC_TRIANGULAR // 1 Hz triangular waveform
#undef CFG_AUDIO_EMULATE_PDM_MIC // 222 Hz sine waveform


/***************************************
* \brief Audio pin name definition                                                                 
***************************************/
enum audio_pin_ids {
    AUDIO_CLK_PIN,
    AUDIO_DATA_PIN,
};


/***********************************
* \brief Audio pin configuration                                                                 
***********************************/
static const pin_type_t app_audio_pins[] = {
    [AUDIO_CLK_PIN]  = {.port = GPIO_PORT_2, .pin = GPIO_PIN_1, .high = 0, .mode_function = OUTPUT | PID_PDM_CLK },
    [AUDIO_DATA_PIN] = {.port = GPIO_PORT_2, .pin = GPIO_PIN_0, .high = 0, .mode_function = INPUT  | PID_PDM_DATA},
};


/****************************************************************************************************
* \brief This callback will be called to notify the application that new audio data are available.
****************************************************************************************************/
#define AUDIO_NOTIFICATION_CB user_audio_callback


static const audio_util_funcs_t app_audio_funcs = {
    .start_sampling = port_audio_start_sampling,
    .stop_sampling  = port_audio_stop_sampling, 
    .get_errors     = port_audio_get_errors,    
    .next_package   = port_audio_next_package, 
    .get_data       = port_audio_get_data,      
    .has_data       = port_audio_has_data,      
};  

#endif	// _APP_AUDIO_CONFIG_H_
