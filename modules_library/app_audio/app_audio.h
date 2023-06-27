/*****************************************************************************************
 * \file app_audio.h
 * \brief The audio module provides an API to capture, process and encode audio from analog or PDM microphones
 * Define symbol HAS_AUDIO to include this module in the application.
*****************************************************************************************/

/********************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup APP_AUDIO
 * \brief Audio implementation
 * \{
********************************/

#ifndef APP_AUDIO_H_
#define APP_AUDIO_H_


#ifdef HAS_AUDIO

    #include <port_platform.h>
    #include "app_audio_defs.h"

    /*****************************************************************************************
     * \brief Convert ADPCM mode to bit rate. 
     *        20% more bandwidth is used (28800 instead of 24000 and 19200 instead of 16000)
     *        to allow recovery in noise environment
     * \param[in] mode
     * \return
    ******************************************************************************************/
    #define ADPM_MODE_TO_RATE_120(mode) ((mode) == 3 ? 28800 : (4-(mode))*19200) 


    /****************************************************************************
     * \brief Convert ADPCM mode to bit rate. 
     * \param[in] mode
     * \return
    *****************************************************************************/
    #define ADPM_MODE_TO_RATE(mode) ((mode) == 3 ? 24000 : (4-(mode))*16000) 


    /*******************************************************************
     * \brief Initialize the audio state variable.
     * This function is called at every start of new Audio Command.
    ********************************************************************/
    void app_audio_init(void);


    /************************************
     * \brief Set IMA ADPCM mode
     * \param[in] mode IMA ADPCM mode
    ************************************/
    void app_audio_set_adpcm_mode(app_audio_adpcm_mode_t mode);


    #ifdef CFG_AUDIO_DEBUG_ENC_AUDIO_TO_UART
        /*****************************************************************************************
         * \brief Test Function to sent Audio Samples to UART
         * This test function will process all incoming Audio Packets (40 Samples) and 
         * encode them with selected coder (IMA, ALAW, LIN).
         * This function does not return, it runs indefinitely.
        ******************************************************************************************/
        void app_audio_to_uart(void);
    #endif


    /*****************************************************************************************
     * \brief Encode the audio, read one packet from buffer, encode and store in stream buffer
     * This function is the interface between the raw sample packets (stored in
     * spi439 fifo in groups of 40 samples) and streaming packets (with compressed audio) which
     * are currently 20 bytes (but this could change).
     * The function uses an internal working buffer (app_audio_env.sbuffer) for this purpose.
     * - if sbuffer has enough space, add spi439 sample block.
     * - if sbuffer has enough samples, compress a block of samples to make one HID stream packet.
     *   The amount of samples needed for one HID packet depends on the IMA compression mode
     *   selected. Default 4 bits/sample = 40 samples makes 20 bytes. For 3 bits/sample, we
     *   need 50 samples (5 IMA codes will occupy 2 bytes).
     * \param[in] enc_buffer pointer to the output buffer
     * \param[in] size output buffer size
     * \return number of encoded bytes in enc_buffer. Zero if no encoding has been performed
    ******************************************************************************************/
    uint16_t app_audio_encode(uint8_t *enc_buffer, uint16_t size);


    /*****************************************************************************************
     * \brief Start the Audio processing
    ******************************************************************************************/
    void app_audio_start(void);


    /*****************************************************************************************
     * \brief Stop the Audio processing
    ******************************************************************************************/
    void app_audio_stop(void);


    /*****************************************************************************************
     * \brief Stop the Audio processing
     * \return true if audio is active
    ******************************************************************************************/
    bool app_audio_is_active(void);


    /*****************************************************************************************
     * \brief Stop the Audio processing
     *
     * \return number of audio errors
    ******************************************************************************************/
    uint16_t app_audio_get_errors(void);


#endif


#endif // APP_AUDIO_H_

/**
 * \}
 * \}
 * \}
 */
