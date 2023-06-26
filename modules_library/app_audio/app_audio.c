/*****************************************************************************************
 * \file app_audio.c
 * \brief Audio module source file
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup APP_AUDIO
 * \brief Audio implementation
 * \{
******************************************************************************************/

#ifdef HAS_AUDIO
        #include "app_audio.h"
        #include "app_audio_codec.h"
        #include <string.h>

        #ifdef CFG_AUDIO_DEBUG_ENC_AUDIO_TO_UART
                #include "uart.h"
        #endif

        #include <app_audio_config.h>

        #if !defined(CFG_AUDIO_ADAPTIVE_RATE) && !defined(ADPCM_DEFAULT_MODE)
                #error "ADPCM_DEFAULT_MODE must be defined"
        #endif

        #if defined(CFG_AUDIO_USE_32BIT_SAMPLING) && !defined(CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE)
                #error "CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE must be defined"
        #endif

        #ifdef CFG_AUDIO_SAMPLING_ATTENUATION
                #error "CFG_AUDIO_SAMPLING_ATTENUATION has been deprecated. Calclulate offset in AUDIO_SAMPLING_OFFSET instead."
        #endif

        #if defined(CFG_AUDIO_DC_BLOCK) && defined(CFG_AUDIO_EMULATE_PDM_MIC_TRIANGULAR)
                #undef CFG_AUDIO_DC_BLOCK
                #warning "Audio DC blocking has been disabled"
        #endif    

        #define APP_AUDIO_DCB_BETA (32768*0.0728)

        #define AUDIO_CALL_CALBACK(func, param)   if(app_audio_funcs.func!=NULL) app_audio_funcs.func(param)
        #define AUDIO_CALL_CALBACK_VOID(func)     if(app_audio_funcs.func!=NULL) app_audio_funcs.func()
        #define AUDIO_CALL_CALBACK_VOID_RET(func) app_audio_funcs.func()

        #if AUDIO_SBUF_SIZE > 0xFFFF
                #error "AUDIO_SBUF_SIZE must fit in 16 bits"
        #endif

        typedef  struct app_audio_env_s
        {
                #ifdef CFG_AUDIO_IMA_ADPCM
                        IMAData_t imaState;
                #endif // CFG_AUDIO_IMA_ADPCM
                unsigned int errors_send;

                #ifdef CFG_AUDIO_DC_BLOCK
                        DCBLOCKData_t dcBlock;
                #endif

                uint32_t buffer_errors;
                /* States for internal working buffer, to deal with changing rates */
                uint16_t sbuf_len;
                uint16_t sbuf_avail;
                int16_t  sbuffer[AUDIO_SBUF_SIZE];

                #if !defined(CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE) && (defined(CFG_AUDIO_ADAPTIVE_RATE) || (ADPCM_DEFAULT_MODE)==2 || (ADPCM_DEFAULT_MODE)==3)
                        int16_t FilterTaps[FILTER_LENGTH];  
                #endif    
                #ifdef CFG_AUDIO_ADAPTIVE_RATE    
                        audio_sampling_rate_t sampling_mode;
                #endif    
        } app_audio_env_t;

        app_audio_env_t app_audio_env;
    
        bool app_audio_started __PORT_RETAINED;
        #if defined(CFG_AUDIO_IMA_ADPCM) && defined(CFG_AUDIO_ADAPTIVE_RATE)
                app_audio_adpcm_mode_t adpcm_mode __PORT_RETAINED;
        #endif

        #ifdef CFG_AUDIO_CLICK_STARTUP_CLEAN
                int click_packages;
                #define DROP_PACKAGES_NO         25      //how many packages to drop
                #define DC_BLOCK_PACKAGES_START  10      //when to start DC_BLOCK calculation
                #define DC_BLOCK_PACKAGES_STOP  150      //when to stop DC_BLOCK calculation
        #endif

        #ifdef CFG_AUDIO_IMA_ADPCM
                /*****************************************************************************************
                 * \brief Set the correct IMA encoding parameters
                 * The ima-mode is stored in global retention value app_audio_adpcm_mode;
                 * The Data Rate will be:
                 * 0: 64 Kbit/s = ima 4Bps, 16 KHz.
                 * 1: 48 Kbit/s = ima 3Bps, 16 KHz.
                 * 2: 32 Kbit/s = ima 4Bps, 8 KHz (downsample if hardware does not support 8KHz sampling).
                 * 3: 24 Kbit/s = ima 3Bps, 8 KHz (downsample if hardware does not support 8KHz sampling).
                *****************************************************************************************/
                static void app_audio_set_adpcm_mode_params(void)
                {
                        int AUDIO_IMA_SIZE;

                #ifdef CFG_AUDIO_ADAPTIVE_RATE
                        switch(adpcm_mode)
                #else
                        switch(ADPCM_DEFAULT_MODE)
                #endif
                        {
                        case ADPCM_MODE_24KBPS_3_8KHZ:
                        case ADPCM_MODE_48KBPS_3_16KHZ:
                                AUDIO_IMA_SIZE = 3;
                                break;
                        case ADPCM_MODE_64KBPS_4_16KHZ:
                        case ADPCM_MODE_32KBPS_4_8KHZ:
                                AUDIO_IMA_SIZE = 4;
                                break;
                        default:
                                ASSERT_ERROR(0);
                                break;
                        }

                        app_audio_env.imaState.imaSize  = AUDIO_IMA_SIZE;
                        app_audio_env.imaState.imaAnd   = 0xF - ((1 << (4 - AUDIO_IMA_SIZE)) - 1);
                        app_audio_env.imaState.imaOr    = (1 << (4 - AUDIO_IMA_SIZE)) - 1;
                }
                #endif // CFG_AUDIO_IMA_ADPCM

void app_audio_init(void)
{
        app_audio_env.sbuf_len          = 0;
        app_audio_env.sbuf_avail        = AUDIO_SBUF_SIZE; // Number of bytes available
     
#ifdef CFG_AUDIO_DC_BLOCK
        app_audio_env.dcBlock.init      = 1;
        app_audio_env.dcBlock.len       = AUDIO_NR_SAMP_PER_SLOT;
        app_audio_env.dcBlock.beta      = APP_AUDIO_DCB_BETA;
        app_audio_env.dcBlock.xn1       = 0;
        app_audio_env.dcBlock.yyn1      = 0;
    #ifdef CFG_AUDIO_ENABLE_DC_BLOCK_FADING
        app_audio_env.dcBlock.fade_step = 16;   // about 1000 samples fade-in
    #endif    
    #ifdef REMOVE_PACKETS_START
        app_audio_env.dcBlock.fcnt      = 25;   // block input for first 25 frames of 40 samples
    #endif       
#endif
        app_audio_env.buffer_errors     = 0;
        app_audio_env.errors_send       = 100;        
    
#ifdef CFG_AUDIO_IMA_ADPCM    
    #ifdef AUDIO_CONTROL_ESCAPE_VALUE
        app_audio_env.imaState.use_byte_stuffing = true;
        app_audio_env.imaState.escape_byte_value = AUDIO_CONTROL_ESCAPE_VALUE;
    #else
        app_audio_env.imaState.use_byte_stuffing = false;
    #endif
        app_audio_env.imaState.index    = 0;
        app_audio_env.imaState.predictedSample = 0;
        app_audio_set_adpcm_mode_params();  // set IMA ADPCM encoding parameters
#endif    
}

#if defined(CFG_AUDIO_IMA_ADPCM) && defined(CFG_AUDIO_ADAPTIVE_RATE)
void app_audio_set_adpcm_mode(app_audio_adpcm_mode_t mode)
{
        #if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
                if(mode > adpcm_mode) {
                dbg_putc('-');
                }
                if(mode < adpcm_mode) {
                dbg_putc('+');
                }
        #endif
                adpcm_mode = mode;
                app_audio_set_adpcm_mode_params();
}
#endif

#ifdef CFG_AUDIO_DEBUG_ENC_AUDIO_TO_UART

void app_audio_to_uart(void)
{
        int i;
        char msg[] = "AudioData:\r\n";
        uart_write((uint8_t*)msg,strlen(msg),NULL);
    #ifdef CFG_AUDIO_IMA_ADPCM
        uint8_t ima_bytes[AUDIO_NR_SAMP_PER_SLOT];
        t_IMAData imaState;
        imaState.len = AUDIO_NR_SAMP_PER_SLOT;
        imaState.out = ima_bytes;
        imaState.index = 0;
        imaState.predictedSample = 0;
    #endif    
        app_audio_start();

        while (true) {
                if (AUDIO_CALL_CALBACK_VOID(has_data)) {
    #ifdef CFG_AUDIO_IMA_ADPCM
                        imaState.inp = (int16_t *)&((audio_sample_t *)AUDIO_CALL_CALBACK_VOID(get_data))[2];

                        app_audio_ima_enc(&imaState);
                        for (i=0;i<AUDIO_NR_SAMP_PER_SLOT/2;i++) {
                                dbg_putc(ima_bytes[i]);
                        }
    #else
                        for (i=0;i<AUDIO_NR_SAMP_PER_SLOT;i++) {
                                /* Send out all the byte one by one.. */
                                int16_t s = ((audio_sample_t *)AUDIO_CALL_CALBACK_VOID(get_data))[i+2]; // SKIP two samples !!
                                uint8_t alaw = audio_aLaw_encode(s);
                                dbg_putc(alaw);
                        }
    #endif
                        AUDIO_CALL_CALBACK_VOID(next_package);
                }
        }
}
#endif


#ifndef CFG_AUDIO_USE_32BIT_SAMPLING
/****************************************************************************************
 * \brief Fill work buffer with new packet  with optional downsampling
 * This function will add a packet to the work buffer. If downsampling is selective
 * then the a 2x downsampling is performed using FIR filter.
 * \param[in] ptr: pointer to the data in packet.
*****************************************************************************************/
static void app_audio_fill_buffer(int16_t *ptr)
{
        int16_t *dst = &app_audio_env.sbuffer[app_audio_env.sbuf_len];
        int tot = AUDIO_NR_SAMP_PER_SLOT;

    #ifndef CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE
        #ifdef CFG_AUDIO_ADAPTIVE_RATE
        if (app_audio_env.sample_mode == SAMPLING_RATE_8KHz) {
                app_audio_downSample(AUDIO_NR_SAMP_PER_SLOT, ptr, dst, app_audio_env.FilterTaps);
                tot = AUDIO_NR_SAMP_PER_SLOT / 2;
        }
        #elif ((ADPCM_DEFAULT_MODE)==2 || (ADPCM_DEFAULT_MODE)==3)
        app_audio_downSample(AUDIO_NR_SAMP_PER_SLOT, ptr, dst, app_audio_env.FilterTaps);
        tot = AUDIO_NR_SAMP_PER_SLOT / 2;
        #endif
    #endif

        app_audio_env.sbuf_len += tot;
        ASSERT_ERROR(app_audio_env.sbuf_len <= AUDIO_SBUF_SIZE);
                
        app_audio_env.sbuf_avail -= tot;

        if (tot == AUDIO_NR_SAMP_PER_SLOT)
        {
                for (int i = 0; i < tot; i++)
                {
                        *dst++ = *ptr++;
                }
        }
}
#endif

/*****************************************************************************************
 * \brief Remove len samples from the work buffer.
 * The len samples to be removed are always at the start of the buffer. Shift the complete
 * buffer, and update the sbuf_len and sbuf_avail states.
 *
 * \param[in] len: number of samples to remove
******************************************************************************************/
static void app_audio_empty_buffer(int len)
{
        app_audio_env.sbuf_len -= len;
        app_audio_env.sbuf_avail += len;

        int16_t *src = app_audio_env.sbuffer + len;
        int16_t *dst = app_audio_env.sbuffer;
        /* Move the buffer.. */
        for (int i = 0; i < app_audio_env.sbuf_len; i++) {
                *dst++ = *src++;
        }
}

#ifdef CFG_AUDIO_USE_32BIT_SAMPLING
static int16_t* app_audio_get_buffer_ptr(uint16_t length)
{
        ASSERT_ERROR(app_audio_env.sbuf_len+length <= AUDIO_SBUF_SIZE);                        
        return &app_audio_env.sbuffer[app_audio_env.sbuf_len];
}

static void app_audio_commit_buffer(uint16_t length)
{
        app_audio_env.sbuf_len += length;
        ASSERT_ERROR(app_audio_env.sbuf_len <= AUDIO_SBUF_SIZE);                        
        app_audio_env.sbuf_avail -= length;
}
#endif

uint16_t app_audio_encode(uint8_t *enc_buffer, uint16_t size)
{   
        uint16_t length;
#ifdef CFG_AUDIO_IMA_ADPCM
        app_audio_env.imaState.len = size * 8 / app_audio_env.imaState.imaSize; // Number of samples needed to encode one output packet
    
        // sbuffer must be large enough to hold the encoder input data    
        ASSERT_ERROR(AUDIO_SBUF_SIZE >= (app_audio_env.imaState.len+(AUDIO_NR_SAMP_PER_SLOT-1)));
    
        length = app_audio_env.imaState.len;
#else
        length = AUDIO_NR_SAMP_PER_SLOT;
#endif    
        // First check if there is enough space in our Sbuffer
        //  to put in one sample block 
        while ((app_audio_env.sbuf_avail >= AUDIO_NR_SAMP_PER_SLOT) && AUDIO_CALL_CALBACK_VOID_RET(has_data)
                && app_audio_env.sbuf_len < length) {
#ifdef CFG_AUDIO_DC_BLOCK
        // Additional DC Blocking Filter
        // It will do the whole block and stores output inplace (same array as input).
        #if defined(CFG_AUDIO_CLICK_STARTUP_CLEAN) && !defined(CFG_AUDIO_USE_32BIT_SAMPLING)
                if ((click_packages < DC_BLOCK_PACKAGES_STOP) &&
                        (click_packages > DC_BLOCK_PACKAGES_START))
        #endif
                {
                        app_audio_env.dcBlock.inp = AUDIO_CALL_CALBACK_VOID_RET(get_data);
        #ifdef CFG_AUDIO_USE_32BIT_SAMPLING
                        app_audio_env.dcBlock.out = app_audio_get_buffer_ptr(AUDIO_NR_SAMP_PER_SLOT);
        #else
                        app_audio_env.dcBlock.out = app_audio_env.dcBlock.inp; /* IN-PLACE operation !! */
        #endif
                        app_audio_dcblock(&app_audio_env.dcBlock);
                }
#elif defined(CFG_AUDIO_USE_32BIT_SAMPLING)
                audio_sample_t *data = AUDIO_CALL_CALBACK_VOID_RET(get_data);
                int16_t *buffer = app_audio_get_buffer_ptr(AUDIO_NR_SAMP_PER_SLOT);

                for(int i=0; i < AUDIO_NR_SAMP_PER_SLOT; i++) {
                        buffer[i]= data[i] >> AUDIO_SAMPLING_OFFSET;
                }
#endif
                
                bool commit_packet = true;
#ifdef CFG_AUDIO_CLICK_STARTUP_CLEAN
                if (click_packages < DROP_PACKAGES_NO) {
                        //if DROP_PACKAGES_NO>DC_BLOCK_PACKAGES_START DC-blocking updated but not used
                        click_packages++;
                        commit_packet = false;
                }
                else if (click_packages < DC_BLOCK_PACKAGES_STOP) {
                        click_packages++;
                }
#endif
                /* Now add the sample block to our SBuffer */
                if(commit_packet == true) {
#ifdef CFG_AUDIO_USE_32BIT_SAMPLING
                    app_audio_commit_buffer(AUDIO_NR_SAMP_PER_SLOT);
#else
                    app_audio_fill_buffer(AUDIO_CALL_CALBACK_VOID_RET(get_data));
#endif
                }
                
                AUDIO_CALL_CALBACK_VOID(next_package);
        }

        /*
         ** Check if we have enough samples in the Sbuffer.
         ** sbuf_min is number of samples needed for encoding 20 output bytes (one ble packet).
         ** For IMA-ADPCM @ 8/16 KHz, this value is 40. For IMA-ADPCM 3 bits, this is 53.
         */
        if (app_audio_env.sbuf_len >= length) {
                if (enc_buffer == NULL) {
                        // drop the packet
                        app_audio_env.buffer_errors++;
                        app_audio_empty_buffer(app_audio_env.sbuf_len);
#if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
                        dbg_putc('.');
#endif
                        return 0;
                }

#if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
                dbg_putc('E');
#endif
        
#ifdef CFG_AUDIO_IMA_ADPCM
                /*
                 ** IMA Adpcm, compress 160/AUDIO_IMA_SIZE samples
                 */
                app_audio_env.imaState.inp = app_audio_env.sbuffer;
                app_audio_env.imaState.out = enc_buffer;
                
                uint16_t num_of_encoded_bytes = app_audio_ima_enc(&app_audio_env.imaState);
                
                // imaState.len has been updated by app_audio_ima_enc with the number of samples actually encoded
                app_audio_empty_buffer(app_audio_env.imaState.len);
                return num_of_encoded_bytes;
#else
                /*
                 ** For ALAW, we encode 40 samples into 2 packets of 20 bytes.
                 */
                int16_t *src = app_audio_env.sbuffer; 
                uint8_t *dst = enc_buffer;
                uint32_t i;
                for (i=0;i<AUDIO_NR_SAMP_PER_SLOT/2;i++) {
                        *dst++ = app_audio_aLaw_encode(*src++);
                }
                for (i=0;i<AUDIO_NR_SAMP_PER_SLOT/2;i++) {
                        *dst++ = app_audio_aLaw_encode(*src++);
                }
                app_audio_empty_buffer(AUDIO_NR_SAMP_PER_SLOT);
                return AUDIO_NR_SAMP_PER_SLOT;
#endif

        }
        else {
                // The intermediate buffer is empty, go to next iteration so it can be filled up...
                return 0;
        }
}

void app_audio_start(void)
{
#if defined(CFG_AUDIO_IMA_ADPCM) && defined(CFG_AUDIO_ADAPTIVE_RATE)
        audio_sampling_rate_t new_sampling_rate;

        switch(adpcm_mode) {
        case ADPCM_MODE_24KBPS_3_8KHZ:
        case ADPCM_MODE_32KBPS_4_8KHZ:
                new_sampling_rate = SAMPLING_RATE_8KHz;
                break;
        case ADPCM_MODE_48KBPS_3_16KHZ:
        case ADPCM_MODE_64KBPS_4_16KHZ:
                new_sampling_rate = SAMPLING_RATE_16KHz;
                break;
        default:
                ASSERT_ERROR(0);
                break;
        }
#endif

        if (app_audio_started == false) {
            app_audio_init();    //clear the state data before starting
#ifdef CFG_AUDIO_CLICK_STARTUP_CLEAN
            click_packages = 0;   //changing logic
#endif
        }
        else {
#if defined(CFG_AUDIO_IMA_ADPCM) && defined(CFG_AUDIO_ADAPTIVE_RATE)
                if(new_sampling_rate != app_audio_env.sampling_mode) {
                        AUDIO_CALL_CALBACK_VOID(stop_sampling);
                        app_audio_started = false; // To force restarting audio sampling with new rate
                }
#endif                    
        }

        if(app_audio_started == false) {
#ifdef CFG_AUDIO_IMA_ADPCM            
    #ifdef CFG_AUDIO_ADAPTIVE_RATE
            app_audio_env.sampling_mode = new_sampling_rate;
            AUDIO_CALL_CALBACK(start_sampling, app_audio_env.sampling_mode);
    #elif defined(CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE) && ((ADPCM_DEFAULT_MODE == 2) || (ADPCM_DEFAULT_MODE == 3))
            AUDIO_CALL_CALBACK(start_sampling,SAMPLING_RATE_8KHz);
    #else
            AUDIO_CALL_CALBACK(start_sampling,SAMPLING_RATE_16KHz);
    #endif
#else
            AUDIO_CALL_CALBACK(start_sampling,SAMPLING_RATE_16KHz);
#endif            
        }

        app_audio_started = true;

}

void app_audio_stop(void)
{
        if (app_audio_started == true) {
                AUDIO_CALL_CALBACK_VOID(stop_sampling);
                app_audio_started = false;
        }
}


bool app_audio_is_active(void)
{
        return app_audio_started;
}

uint16_t app_audio_get_errors(void)
{
        uint16_t ret = app_audio_env.buffer_errors | ((uint16_t)AUDIO_CALL_CALBACK_VOID_RET(get_errors)) << 8;
        app_audio_env.buffer_errors = 0;
        return ret;
}

#endif // (HAS_AUDIO)

/**
 * \}
 * \}
 * \}
 */
