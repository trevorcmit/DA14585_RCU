/*****************************************************************************************
 *
 * \file app_audio_codec.c
 *
 * \brief Functions for compressing Audio samples
* 
 *****************************************************************************************
 */

/**
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup AUDIO_CODEC
 * \{
 */
 
#include "app_audio_codec.h"

#ifndef CFG_AUDIO_USE_32BIT_SAMPLING
        #undef AUDIO_SAMPLING_OFFSET
        #define AUDIO_SAMPLING_OFFSET          0  // number of bits that the sample must be shifted to the right to discard unused bits
#endif

#define FILTER_LENGTH 38

#ifdef CFG_AUDIO_IMA_ADPCM
    static const signed char indexTable[16] = {
            -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8
    };
                      
static const short stepSizeTable[89] = {
        7,     8,     9,    10,    11,    12,    13,    14,    16,    17,    
       19,    21,    23,    25,    28,    31,    34,    37,    41,    45,  
       50,    55,    60,    66,    73,    80,    88,    97,   107,   118,   
      130,   143,   157,   173,   190,   209,   230,   253,   279,   307,   
      337,   371,   408,   449,   494,   544,   598,   658,   724,   796,   
      876,   963,  1060,  1166,  1282,  1411,  1552,  1707,  1878,  2066,  
     2272,  2499,  2749,  3024,  3327,  3660,  4026,  4428,  4871,  5358,
     5894,  6484,  7132,  7845,  8630,  9493, 10442, 11487, 12635, 13899, 
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

uint16_t app_audio_ima_enc(IMAData_t *state)
{
    int32_t   i, j;
    int16_t   *ptr            = state->inp;
    int32_t   predictedSample = (int32_t)state->predictedSample;
    int32_t   index           = state->index;
    int32_t   stepSize        = stepSizeTable[index];
    uint8_t   *optr  = state -> out;
    uint16_t  outBuf = 0;
    int32_t   size   = state -> imaSize;
    int32_t   outMsb = 16-size;
    int32_t   imaAnd = state->imaAnd;
    int32_t   imaOr  = state->imaOr;
    int32_t   shift  = 4-size;
    uint16_t  sample_deduction = 0;
    
    for (i=0; i < state->len; i++) {
        int inp = *ptr++;

        /* Find the difference with predicted sample */
        int32_t diff = inp - predictedSample;
        int sign = diff;

        /* Set the sign of new IMA nibble and find absolute value of difference */
        uint8_t newIma = 0;
        if (sign < 0)
        {
            diff = -diff;
        }

        /* Quantize the difference to four bits */
        /* Also compute new sample estimate predictedSample */
        int mask = 4;
        /* Using the mult may be faster, depending on architecture  */
        /* should give same bit-true result as above..              */
        /* NOTE we do and extra shiftup of stepSize and diff to get */
        /* better/closer prediction                                 */
        int32_t tempStepSize = stepSize << 3;
        diff <<= 3;
        for (j=0;j<3;j++) {
            if (diff > tempStepSize) {
                newIma |= mask;   /* Perform Division trough repeated subtraction */
                diff -= tempStepSize;
            }
            tempStepSize >>= 1;  /* Adjust comparator for next iteration */
            mask >>= 1;          /* Adjust bit-set for next iteration */
        }

        /*
        ** We have a new IMA code, pack the ima-code in 2 bytes
        */
        int storeIma = newIma;

        if (sign < 0) {
                storeIma |= 8;
        }
        storeIma = storeIma >> shift;

        outBuf |= (storeIma << outMsb);
        if (outMsb < 8) {
            *optr = (uint8_t)(outBuf >> 8);
            if(state->use_byte_stuffing == true) {
                if(*optr == state->escape_byte_value) { 
                // If the value is equal to the escape byte then add byte stuffing
                    optr++;
                    *optr = state->escape_byte_value;
                    sample_deduction += (8+(size-1)) / size; // deduct a number of samples because their space 
                                                             // in state->out is occupied by the byte stuffing.
                }
            }
            optr++;
            
            outMsb += 8;
            outBuf = (uint16_t)(outBuf << 8);
        }
        outMsb -= size;
        /* Now fix ima word as will be used in Decoder */
        newIma = newIma & imaAnd;
        newIma |= imaOr;

        /* make sure the mult is mapped on 16x16 -> 32 bits mult as result may be larger than 16 bits */
        int32_t predictedDiff = (newIma * (int32_t)stepSize) + (stepSize >> 1) ;
        predictedDiff >>= 2;
          
        /* Adjust predictedSample based on calculated difference */
        predictedSample += (sign < 0) ? -predictedDiff
                                      : predictedDiff;
         
        /* Saturate if there is overflow */
        if (predictedSample > 32767) {
            predictedSample = 32767;
        }
        if (predictedSample < -32768) {
            predictedSample = -32768;
        }

        /* Compute new stepsize */
        /* Adjust index into stepSizeTable using newIMA */
        index += indexTable[newIma];
        if (index < 0) {
            index = 0;
        } else if (index > 88) {
            index = 88;
        }
        stepSize = stepSizeTable[index];   /* Find new quantizer stepsize */
        if((outMsb == (8 - size)) && ((sample_deduction + i) >= state->len)) {
            // Stop when an ecoded byte has been fully filled with IMA samples.
            // Output buffer must be at least 1 to 3 bytes larger because stuffing
            // may accur at the last 1 to 3 bytes depending on the IMA size.
            break;
        }
    }    
    /* Store residual bits*/
    *optr = (uint8_t)(outBuf >> 8);
    if(state->use_byte_stuffing == true) {
        if(*optr == state->escape_byte_value) { 
        // If the value is equal to the escape byte then add byte stuffing
            optr++;
            *optr = state->escape_byte_value;
        }
    }
    optr++;
    state->index = index;
    state->predictedSample = predictedSample;
    
    state->len = i; // Store the number of samples actually encoded
    
    return (optr - state->out);
}
#endif

void app_audio_dcblock(DCBLOCKData_t *pDcBlockData)
{
    int i;
    audio_sample_t *inpptr = pDcBlockData->inp;
    int16_t *outptr = pDcBlockData->out;

    volatile audio_sample_t xn1  = pDcBlockData->xn1;
    int32_t yyn1 = pDcBlockData->yyn1;
    int32_t y2;
    audio_sample_t diffx;
    audio_sample_t sample;

    #ifdef REMOVE_PACKETS_START
    // set the output to 0 for first fcnt frames...
    // NOTE initialize fcnt 
    // Delete this if you do not want to sent the frames out...
    if (pDcBlockData->fcnt > 0) {
        pDcBlockData->xn1 = pDcBlockData->inp[pDcBlockData->len-1];
        pDcBlockData->yyn1 = 0;
        for (i=0; i<pDcBlockData->len; i++) {
            *outptr++ = (int16_t)0;
        }
        pDcBlockData->fcnt--;
        #ifdef CFG_AUDIO_ENABLE_DC_BLOCK_FADING
        pDcBlockData->fade = 0;
        #endif
        return;
    }
    #endif

    
    for (i=0; i<pDcBlockData->len; i++) {
        sample = (*inpptr);
        if(pDcBlockData->init == 1) {
            pDcBlockData->init = 0;
            xn1 = sample;
        }
        diffx = (sample - xn1);
        xn1 = sample;
        inpptr++;
        y2 = yyn1 + ((uint32_t)diffx << (16 - AUDIO_SAMPLING_OFFSET)) & 0xFFFF0000;
        yyn1 = y2 - (((yyn1 >> 16) * (int32_t)pDcBlockData->beta)<<1);  // multfrac(extract_high(yyn1),pDcBlockData->beta));
                                                                        // Note that optimization is possible with msub..
    #ifdef CFG_AUDIO_ENABLE_DC_BLOCK_FADING
        // Fade 
        // TODO Amplify
        int16_t dcbout = (int16_t)(yyn1 >> 16);

        int16_t fadeout = ((int32_t)dcbout * pDcBlockData->fade)>>15;
        int32_t fade = (int32_t)pDcBlockData->fade + pDcBlockData->fade_step;

        if (fade > 32767) {
            fade = 32767;   // saturate..
        }
        pDcBlockData->fade = (int16_t)fade;
        *outptr++  = fadeout;                        // For optimization, variable OUT is not needed...
    #else
        *outptr++  = (int16_t)(yyn1 >> 16);                             
    #endif
    }
    pDcBlockData->xn1 = xn1;
    pDcBlockData->yyn1 = yyn1;
}    

uint8_t app_audio_aLaw_encode(int16_t pcm_sample)
{
    int p = pcm_sample;
    int a = 0;
    if (pcm_sample < 0) {
        p = ~p;
    } else {
        a = 0x80;
    }

    // Calculate segment and interval numbers
    p >>= 4;
    if (p >= 0x20) {
        if(p >= 0x100) {
            p >>= 4;
            a += 0x40;
        }
        if(p >= 0x40) {
            p >>= 2;
            a += 0x20;
        }
        if(p >= 0x20) {
            p >>= 1;
            a += 0x10;
        }
    }
    // a&0x70 now holds segment value and 'p' the interval number

    a += p;  // a now equal to encoded A-law value
    return (uint8_t)(a^0x55);    // A-law has alternate bits inverted for transmission
}

#define TOINT(a) (a*32768)

const int16_t FilterCoefs[FILTER_LENGTH] = {
        TOINT(-0.006550968445693665),
        TOINT(-0.007203184032532347),
        TOINT( 0.009945665954828096),
        TOINT( 0.037705923946649726),
        TOINT( 0.044282962988788518),
        TOINT( 0.014386511654397057),
        TOINT(-0.018622015733493397),
        TOINT(-0.011376412377131022),
        TOINT( 0.022192290761063299),
        TOINT( 0.023499142893664525),
        TOINT(-0.018292465495386833),
        TOINT(-0.036284806590194230),
        TOINT( 0.011605459484924431),
        TOINT( 0.055421803211650633),
        TOINT( 0.004590134692339709),
        TOINT(-0.087168057180386477),
        TOINT(-0.047507381887440345),
        TOINT( 0.180848351177550850),
        TOINT( 0.412677836954337680),
        TOINT( 0.412677836954337680),
        TOINT( 0.180848351177550850),
        TOINT(-0.047507381887440345),
        TOINT(-0.087168057180386477),
        TOINT( 0.004590134692339709),
        TOINT( 0.055421803211650633),
        TOINT( 0.011605459484924431),
        TOINT(-0.036284806590194230),
        TOINT(-0.018292465495386833),
        TOINT( 0.023499142893664525),
        TOINT( 0.022192290761063299),
        TOINT(-0.011376412377131022),
        TOINT(-0.018622015733493397),
        TOINT( 0.014386511654397057),
        TOINT( 0.044282962988788518),
        TOINT( 0.037705923946649726),
        TOINT( 0.009945665954828096),
        TOINT(-0.007203184032532347),
        TOINT(-0.006550968445693665)
};

void app_audio_downSample(int len, int16_t *inpSamples, int16_t *outSamples, int16_t *taps)
{
    int     i,j;
    int16_t *iptr = inpSamples;
    int16_t *optr = outSamples;

    for (i=0; i<len; i+=2) {
        int acc = 0;
        for (j=0; j<FILTER_LENGTH-3; j+=2) {
            acc += (int)FilterCoefs[j] *   (int)taps[j];
            acc += (int)FilterCoefs[j+1] * (int)taps[j+1];
            taps[j]   = taps[j+2];
            taps[j+1] = taps[j+3];
        }
        taps[FILTER_LENGTH-2] = *iptr++;
        taps[FILTER_LENGTH-1] = *iptr++;
        acc += (int)FilterCoefs[FILTER_LENGTH-2] * (int)taps[FILTER_LENGTH-2];
        acc += (int)FilterCoefs[FILTER_LENGTH-1] * (int)taps[FILTER_LENGTH-2];
        /* Shift down with saturation */
        int accshift = (acc >> 15);
        if (accshift > 32767) {
            accshift = 32767;
        } else if (accshift < -32768) {
            accshift =  -32768;
        }
        *optr++ = accshift; // (acc >> 15);
    }
}

/**
 * \}
 * \}
 * \}
 */
