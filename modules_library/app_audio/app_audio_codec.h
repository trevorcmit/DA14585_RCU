/**
 ****************************************************************************************
 *
 * \file app_audio_codec.h
 *
 * \brief Functions for compressing Audio samples
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 *
 *  IMA ADPCM
 *  This IMA ADPCM has been implemented based on the paper spec of 
 *  IMA Digital Audio Focus and Technical Working Groups,
 *  Recommended Practices for Enhancing Digital Audio Compatibility 
 *  Revision 3.00, 21 October 1992. 
 *  http://www.cs.columbia.edu/~hgs/audio/dvi/IMA_ADPCM.pdf
 * 
 *  There are some enhancement/modifications.
 *  1) Encoder: The divisor is calculated with full precision by upshifting
 *     both the stepSize and Difference by 3.
 *  2) The new predicted difference is calculated with full precision by
 *     doing the mult/shift-add 3 bits more. 
 *  3) The exact same prediction is used in Encoder and Decoder. Note that
 *     some implementations of IMA-ADPCM do not do this.
 *  4) There is an alternative (but bit-true) calculation of the prediction
 *     difference using a multiplier instead of shift add. May be faster.
 *
 ****************************************************************************************
 */

/**
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup AUDIO_CODEC
 *
 * \brief Audio codec
 *
 * \{
 */
 
#ifndef __APP_AUDIO_CODEC_H__
#define __APP_AUDIO_CODEC_H__

#include "stdint.h"
#include <app_audio_config.h>

#define FILTER_LENGTH 38

#ifdef CFG_AUDIO_USE_32BIT_SAMPLING
        typedef int32_t audio_sample_t;
#else
        typedef int16_t audio_sample_t;
#endif

typedef struct IMAData_s {
    int16_t  *inp;
    uint8_t  *out;
    int16_t  len;
    int16_t  index;
    int16_t  predictedSample;
    int16_t  imaSize;
    int32_t  imaAnd;
    int32_t  imaOr;
    bool     use_byte_stuffing;
    uint8_t  escape_byte_value;
} IMAData_t;

typedef struct DcBlockData_s {
    int32_t yyn1;
    int16_t beta;
    int16_t *out;
    int16_t  len;
    uint8_t init;
    /* States */
#ifdef REMOVE_PACKETS_START
    int16_t fcnt;
#endif    
#ifdef CFG_AUDIO_ENABLE_DC_BLOCK_FADING
    int16_t fade;
    int16_t fade_step;
#endif
    audio_sample_t *inp;
    audio_sample_t xn1;
} DCBLOCKData_t;

#ifdef CFG_AUDIO_IMA_ADPCM
/**
 ****************************************************************************************
 * \brief IMA Adpcm Encoding, block based
 *
 * \param[inout] state: encoding state with input/output pointers and states
 *
 * \return Number of bytes encoded in output buffer
 ****************************************************************************************
 */
uint16_t app_audio_ima_enc(IMAData_t *state);
#endif

/**
 ****************************************************************************************
 * \brief ALaw Encoding, sample based
 *
 * \param[in] pcm_sample: input sample
 *
 * \return Alaw sample
 ****************************************************************************************
 */
uint8_t app_audio_aLaw_encode(int16_t pcm_sample);

/**
 ****************************************************************************************
 * \brief DC Blocking Filter
 *
 * Remove DC with differentiator and leaky integrator 
 * See http://www.dspguru.com/dsp/tricks/fixed-point-dc-blocking-filter-with-noise-shaping
 *
 * The DC high pass filter should have cutt of freq of 200Hz. 
 * With Fc=1/2*PI*RC, and alpha = RC/(RC+dt), an dt=1/Fs, (and PI=22/7)
 * then alpha = calc "1-(1/(200*2*22/7))/((1/(200*2*22/7))+(1/16000))"
 * = 0.0728
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * \brief DC Blocking Filter function
 *
 * This function will do DC Blocking on a block of len samples.
 * Note that input and output pointer may point to the same array (inplace).
 * The DC high pass filter should have cutt of freq of 200Hz. 
 * With Fc=1/2*PI*RC, and alpha = RC/(RC+dt), an dt=1/Fs, (and PI=22/7)
 * then alpha = calc "1-(1/(200*2*22/7))/((1/(200*2*22/7))+(1/16000))"
 * = 0.0728
 *
 * \param[inout] pDcBlockData Structure containing the data
 ****************************************************************************************
 */
void app_audio_dcblock(DCBLOCKData_t *pDcBlockData);


/**
 ****************************************************************************************
 * \brief downSample
 *
 * \param[in] len:        number of input samples
 * \param[in] inpSamples: buffer for len input samples
 * \param[in] outSamples: buffer for len/2 output samples
 * \param[in] taps:       Filter taps
 ****************************************************************************************
 */
void app_audio_downSample(int len, int16_t *inpSamples, int16_t *outSamples, int16_t *taps);

#endif // __APP_AUDIO_CODEC_H__

/**
 * \}
 * \}
 * \}
 */
