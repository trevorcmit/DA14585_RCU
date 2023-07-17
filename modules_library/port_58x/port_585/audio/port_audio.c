/*****************************************************************************************
 * \file port_audio.c
 * \brief Audio module platform adaptation source file
******************************************************************************************/

/***************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup PORT_AUDIO
 * \{
****************************************************************************************/


#ifdef HAS_AUDIO
    #include <stdio.h>
    #include "port_platform.h"
    #include "pdm_mic.h"
    #include <app_audio_config.h>
    #include "port_audio.h"
    #include "app_audio_codec.h"

    #if !defined(CFG_AUDIO_EMULATE_PDM_MIC) && !defined(CFG_AUDIO_USE_32BIT_SAMPLING)
        #error "32bit sampling must be used when using PDM as audio input"
    #endif

    #ifdef CFG_AUDIO_SAMPLING_ATTENUATION
        #error "CFG_AUDIO_SAMPLING_ATTENUATION has been deprecated. Calclulate offset in AUDIO_SAMPLING_OFFSET instead."
    #endif

    #if !defined(CFG_AUDIO_EMULATE_PDM_MIC) && defined(CFG_AUDIO_EMULATE_PDM_MIC_TRIANGULAR)
        #define CFG_AUDIO_EMULATE_PDM_MIC
    #endif

    #ifdef CFG_AUDIO_EMULATE_PDM_MIC
        #include "timer0.h"
    #endif

    #define USE_CIRCULAR_BUFFER

    #ifndef USE_CIRCULAR_BUFFER
        typedef struct s_audio {
            audio_sample_t samples[AUDIO_NR_SAMP_PER_SLOT];
            bool hasData;
        } t_audio_slot;
    #endif

    typedef struct s_port_audio_env
    {
        #ifdef USE_CIRCULAR_BUFFER    
            #define AUDIO_BUFFER_SIZE (AUDIO_BUFFER_NR_SLOTS*AUDIO_NR_SAMP_PER_SLOT)
            
            audio_sample_t buffer[AUDIO_BUFFER_SIZE];
            uint16_t write_index;
            uint16_t read_index;
            bool buffer_full;
        #else    
            t_audio_slot audioSlots[AUDIO_BUFFER_NR_SLOTS];
            uint8_t audioSlotWrNr;
            uint8_t audioSlotRdNr;
            #ifdef CFG_AUDIO_ADAPTIVE_RATE    
                uint8_t audioSlotSize;
            #endif    
        #endif

        uint8_t errors;
        audio_sampling_rate_t sampling_rate;
    } t_port_audio_env;

    t_port_audio_env port_audio_env;

    #ifdef USE_CIRCULAR_BUFFER
        uint16_t buffer_length_requested;
    #else
        t_audio_slot *audio_slot_to_be_committed;
    #endif
                
    typedef void (*audio_notification_cb_t)(void);   

    #ifdef AUDIO_NOTIFICATION_CB 
        static const audio_notification_cb_t app_audio_notification_cb = AUDIO_NOTIFICATION_CB;
    #else
        static const audio_notification_cb_t app_audio_notification_cb = NULL;
    #endif


    static void send_audio_notification(void)
    {
        if (app_audio_notification_cb != NULL)
        {
            (*app_audio_notification_cb)();
        }
    }                    


    #if defined(CFG_AUDIO_EMULATE_PDM_MIC) || !defined(USE_CIRCULAR_BUFFER)
        /****************************************************************************************
         * \brief 
         * \param[in]   length
         * \return
        *****************************************************************************************/ 
    static audio_sample_t *get_next_audio_buffer(uint16_t length)
    {
        #ifdef USE_CIRCULAR_BUFFER
            // In order to simplify circular buffer management, provided buffer must not cross the end of the buffer
            ASSERT_ERROR((port_audio_env.write_index + length) <= AUDIO_BUFFER_SIZE);
            
            buffer_length_requested = length;
            if(port_audio_env.buffer_full == true || 
            ((port_audio_env.write_index < port_audio_env.read_index) && ((port_audio_env.read_index - port_audio_env.write_index) < length))) {
                port_audio_env.errors += 1;
                port_audio_env.buffer_full = false;
            #if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
                dbg_putc('!'); // The target audio slot is occupied
            #endif        
            }        
            return &port_audio_env.buffer[port_audio_env.write_index];

        #else // USE_CIRCULAR_BUFFER
            ASSERT_ERROR(length == AUDIO_NR_SAMP_PER_SLOT); // When audio slots are used length can only be equal to slot size

            port_audio_env.errors += port_audio_env.audioSlots[port_audio_env.audioSlotWrNr].hasData == true? 1 : 0; // To monitor possible buffer Overflows...
            #if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
                if (port_audio_env.audioSlots[port_audio_env.audioSlotWrNr].hasData == true)
                {
                    dbg_putc('!'); // The target audio slot is occupied
                }
            #endif        
            audio_slot_to_be_committed = &port_audio_env.audioSlots[port_audio_env.audioSlotWrNr];

            port_audio_env.audioSlotWrNr++;
            port_audio_env.audioSlotSize++;

            if (port_audio_env.audioSlotWrNr == AUDIO_BUFFER_NR_SLOTS) { port_audio_env.audioSlotWrNr = 0; }
            return audio_slot_to_be_committed->samples;
        #endif
    }
    #endif


    /**************************************************************************************
     * \brief 
    **************************************************************************************/ 
    static void commit_audio_buffer(void)
    {
        #ifdef USE_CIRCULAR_BUFFER
            port_audio_env.write_index += buffer_length_requested;
            
            if (port_audio_env.write_index == AUDIO_BUFFER_SIZE) { port_audio_env.write_index = 0; }
            port_audio_env.buffer_full = (port_audio_env.read_index == port_audio_env.write_index);            
        #else // USE_CIRCULAR_BUFFER
            if (audio_slot_to_be_committed != NULL)
            {
                audio_slot_to_be_committed->hasData = true;    
            }
        #endif
    }

    #ifdef CFG_AUDIO_EMULATE_PDM_MIC   

        #define EMULATE_NUM_OF_PACKETS 2

        const int32_t sine_a[72]=
        {          
            0,    93582720,   186453120,   277904640,   367240960,   453782528,   536870528,   615872512,   690187392,
            759249536,   822533376,   879557248,   929887104,   973140096,  1008986880,  1037154560,  1057428992,  1069655680,
            1073741696,  1069655936,  1057429504,  1037155328,  1008987776,   973141248,   929888640,   879558912,   822535168,  
            759251584,   690189568,   615874816,   536872960,   453785088,   367243648,   277907328,   186455936,    93585536,       
            2816,   -93579904,  -186450304,  -277901824,  -367238272,  -453779968,  -536867968,  -615870208,  -690185216, 
            -759247616,  -822531584,  -879555584,  -929885696,  -973138816, -1008985856, -1037153792, -1057428480, -1069655424, 
            -1073741696, -1069656192, -1057430016, -1037156096, -1008988800,  -973142528,  -929890048,  -879560576,  -822537088,  
            -759253632,  -690191744,  -615877248,  -536875392,  -453787648,  -367246336,  -277910144,  -186458752,   -93588352
        };
        
        // const int32_t sine_a[72]=
        // {
        //     1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000,
        //     -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000,
        //     1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000,
        //     -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000,
        //     1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000, 1000000000,
        //     -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000, -1000000000
        // };

        uint32_t emul_index;

        /****************************************************************************************
         * \brief Systick Handler. Interrupt handler for generating a waveform used for 
         * testing the audio module. The waveform data are used instead of the audio samples
         * acquired from the PDM microphone.
        *****************************************************************************************/
        static void SWTIM_Callback(void)
        {  
            audio_sample_t *data;
            int i;
            int num_of_packets;

            #if defined(USE_AUDIO_MARK) && DEVELOPMENT_DEBUG
                GPIO_ConfigurePin(AUDIO_MARK_PORT, AUDIO_MARK_PIN, OUTPUT, PID_GPIO, true);
            #endif            

            #ifdef CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE
                num_of_packets = (port_audio_env.sampling_rate == SAMPLING_RATE_16KHz ? EMULATE_NUM_OF_PACKETS : EMULATE_NUM_OF_PACKETS/2);
            #else
                num_of_packets = EMULATE_NUM_OF_PACKETS;
            #endif

            // Add 2 or 4 packets of AUDIO_NR_SAMP_PER_SLOT samples in the FIFO
            for (i = 0; i < num_of_packets; i++) {  
                data = get_next_audio_buffer(AUDIO_NR_SAMP_PER_SLOT);

                // int num_of_samples, j, k;            
                // #ifdef CFG_AUDIO_CONFIGURABLE_SAMPLING_RATE
                //     num_of_samples = (port_audio_env.sampling_rate == SAMPLING_RATE_16KHz ? AUDIO_NR_SAMP_PER_SLOT : AUDIO_NR_SAMP_PER_SLOT/2);
                // #else
                //     num_of_samples = AUDIO_NR_SAMP_PER_SLOT;
                // #endif            
                //     int step = 800 * AUDIO_NR_SAMP_PER_SLOT/num_of_samples;
                // for (k=0; k < (4-num_of_packets+2)/2; k++)
                // {
                //     for (j = 0; j < num_of_samples / 2; j++)
                //     {
                //         *data = (step * j) << AUDIO_SAMPLING_OFFSET;
                //         data++;
                //     }
                //     for (j = 0; j < num_of_samples / 2; j++)
                //     {
                //         *data = (16000 - step * j) << AUDIO_SAMPLING_OFFSET;
                //         data++;
                //     }
                // }

                #ifdef CFG_AUDIO_EMULATE_PDM_MIC_TRIANGULAR
                    uint8_t step =  port_audio_env.sampling_rate == SAMPLING_RATE_16KHz ? 1 : 2;
                    uint32_t val;

                    for (uint8_t j=0; j < AUDIO_NR_SAMP_PER_SLOT; j++)
                    {
                        if(emul_index < 8000)
                        {
                            val = emul_index;
                        }
                        else
                        {
                            val = 16000 - emul_index;
                        }
                            
                        data[j] = val << AUDIO_SAMPLING_OFFSET;
                        emul_index += step;

                        if (emul_index == 16000)
                        {
                            emul_index = 0;
                        }
                    }
                #else
                    uint8_t step = port_audio_env.sampling_rate == SAMPLING_RATE_16KHz ? 4 : 8;

                    for (uint8_t j=0; j < AUDIO_NR_SAMP_PER_SLOT; j++)
                    {
                        data[j] = sine_a[emul_index] >> (16+4-AUDIO_SAMPLING_OFFSET);
                        emul_index+=step;
                        if (emul_index == sizeof(sine_a)/sizeof(uint32_t))
                        {
                            emul_index = 0;
                        }
                    }
                #endif

                #if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
                    dbg_putc('a');
                #endif    
                    commit_audio_buffer();        
            }
            
            #if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
                dbg_putc('a');
            #endif
            #if defined(USE_AUDIO_MARK) && DEVELOPMENT_DEBUG
                GPIO_ConfigurePin(AUDIO_MARK_PORT, AUDIO_MARK_PIN, OUTPUT, PID_GPIO, false );
            #endif
                send_audio_notification();
        }


        /****************************************************************************************
         * \brief Local function for configuring the Timer
         * \param[in] ticks
        *****************************************************************************************/
        __INLINE void audio_swtim_configure(const int ticks)
        {
            //Enables TIMER0,TIMER2 clock
            set_tmr_enable(CLK_PER_REG_TMR_ENABLED);
            set_tmr_div(CLK_PER_REG_TMR_DIV_8);
            
            timer0_disable_irq();

            // initialize PWM with the desired settings
            timer0_init(TIM0_CLK_FAST, PWM_MODE_ONE, TIM0_CLK_NO_DIV);

            // set pwm Timer0 On, Timer0 'high' and Timer0 'low' reload values
            timer0_set(10, (ticks/2)-1, (ticks-ticks/2)-1);

            // register callback function for SWTIM_IRQn irq
            timer0_register_callback(SWTIM_Callback);
        }


        /****************************************************************************************
        * \brief Local function for starting the Timer
        *****************************************************************************************/
        __INLINE void audio_swtim_start(void)
        {
            // start pwm0
            timer0_start();
            while (!NVIC_GetPendingIRQ(SWTIM_IRQn));
            NVIC_ClearPendingIRQ(SWTIM_IRQn);
            // enable SWTIM_IRQn irq
            timer0_enable_irq();
        }

        /****************************************************************************************
         * \brief Local function for stopping the Timer
        *****************************************************************************************/
        __INLINE void audio_swtim_stop(void)
        {
            timer0_stop();
        }

    #endif // CFG_AUDIO_EMULATE_PDM_MIC


    /****************************************************************************************
     * \brief Local function for initializing the audio FIFO
    *****************************************************************************************/ 
    static void audio_init_fifo(void)
    {
        #ifdef USE_CIRCULAR_BUFFER    
            port_audio_env.write_index = 0;
            port_audio_env.read_index  = 0;
            port_audio_env.buffer_full = false;
        #else
            port_audio_env.audioSlotWrNr = 0;
            port_audio_env.audioSlotRdNr = 0;
            port_audio_env.audioSlotSize = 0;

            for (int i = 0; i < AUDIO_BUFFER_NR_SLOTS; i++)
            {
                port_audio_env.audioSlots[i].hasData = false;
            }
        #endif    
            port_audio_env.errors = 0;
    }


    void port_audio_next_package(void)
    {
        #ifdef USE_CIRCULAR_BUFFER 
            port_audio_env.read_index += AUDIO_NR_SAMP_PER_SLOT;
            if(port_audio_env.read_index == AUDIO_BUFFER_SIZE)
            {
                port_audio_env.read_index = 0; 
            }
            port_audio_env.buffer_full = false;
        #else    
            port_audio_env.audioSlots[port_audio_env.audioSlotRdNr].hasData = false;
            port_audio_env.audioSlotRdNr++;
            if (port_audio_env.audioSlotRdNr >= AUDIO_BUFFER_NR_SLOTS)
            {
                port_audio_env.audioSlotRdNr = 0;
            }
            port_audio_env.audioSlotSize--;
        #endif        
    }


    void *port_audio_get_data(void)
    {
        #ifdef USE_CIRCULAR_BUFFER    
            return &port_audio_env.buffer[port_audio_env.read_index];
        #else    
            return &port_audio_env.audioSlots[port_audio_env.audioSlotRdNr].samples[0];
        #endif    
    }


    bool port_audio_has_data(void)
    {
        #ifdef USE_CIRCULAR_BUFFER    
            return (port_audio_env.buffer_full == true || port_audio_env.read_index != port_audio_env.write_index);
        #else    
            return port_audio_env.audioSlots[port_audio_env.audioSlotRdNr].hasData == true;
        #endif    
    }


    uint16_t port_audio_buffer_get_used(void)
    {
        #ifdef USE_CIRCULAR_BUFFER 
            if (port_audio_env.buffer_full == true)
            {
                return AUDIO_BUFFER_SIZE;
            }
            
            if (port_audio_env.read_index <= port_audio_env.write_index)
            {
                return (port_audio_env.write_index - port_audio_env.read_index);
            }
            else
            {
                return (AUDIO_BUFFER_SIZE - port_audio_env.read_index + port_audio_env.write_index);
            }
        #else
            return port_audio_env.audioSlotSize * AUDIO_NR_SAMP_PER_SLOT;
        #endif    
    }


    uint8_t port_audio_get_errors(void)
    {
        uint8_t ret = port_audio_env.errors;
        port_audio_env.errors = 0;
        return ret;
    }


    uint32_t *audio_mic_callback(uint16_t length)
    {
        uint32_t *new_buffer;

        #if defined(CFG_AUDIO_UART_DEBUG) && DEVELOPMENT_DEBUG
            dbg_putc('a');
        #endif

        #ifdef USE_CIRCULAR_BUFFER
            while (((port_audio_env.write_index + buffer_length_requested) <= length) || port_audio_env.write_index > length)
            {
                commit_audio_buffer();  
            }
            new_buffer = (uint32_t *)port_audio_env.write_index;
        #else    
            if (length == AUDIO_NR_SAMP_PER_SLOT)
            {
                commit_audio_buffer();
                new_buffer = (uint32_t *)get_next_audio_buffer(AUDIO_NR_SAMP_PER_SLOT);
            }
        #endif    

        send_audio_notification();
        return new_buffer;
    }


    void port_audio_start_sampling(audio_sampling_rate_t sampling_rate)
    {
        arch_force_active_mode();
        if (GetBits16(CLK_RADIO_REG, BLE_ENABLE) == 0)
        {
            arch_ble_force_wakeup();
        }
        audio_init_fifo();        // Initialize FIFO
        port_audio_env.sampling_rate = sampling_rate;

        #ifdef CFG_AUDIO_EMULATE_PDM_MIC
            emul_index = 0;
            audio_swtim_configure(40000*EMULATE_NUM_OF_PACKETS/8);   // timer0 period is 8usec. Hit every EMULATE_NUM_OF_PACKETS packets of 40 samples
            audio_swtim_start();              //start the timer
        #else
            pdm_mic_setup_t mic_config = {
            #ifdef USE_CIRCULAR_BUFFER        
                .buffer          = (uint32_t *)port_audio_env.buffer,
                .buffer_length   = AUDIO_BUFFER_SIZE,
                .buffer_circular = true,
                .int_thresold    = AUDIO_NR_SAMP_PER_SLOT,
            #else
                .buffer          = (uint32_t *)get_next_audio_buffer(AUDIO_NR_SAMP_PER_SLOT),
                .buffer_length   = AUDIO_NR_SAMP_PER_SLOT,
                .buffer_circular = false,
                .int_thresold    = 0,
            #endif
                .clk_gpio.port  = (GPIO_PORT)app_audio_pins[AUDIO_CLK_PIN].port,
                .clk_gpio.pin   = (GPIO_PIN) app_audio_pins[AUDIO_CLK_PIN].pin,
                .data_gpio.port = (GPIO_PORT)app_audio_pins[AUDIO_DATA_PIN].port,
                .data_gpio.pin  = (GPIO_PIN) app_audio_pins[AUDIO_DATA_PIN].pin,
                .callback       = audio_mic_callback
            };

            #ifdef USE_CIRCULAR_BUFFER
                buffer_length_requested = mic_config.int_thresold; // Used in commit_audio_buffer()
            #endif
        
            switch (port_audio_env.sampling_rate) {
        case SAMPLING_RATE_16KHz:
            mic_config.sampling_rate = PDM_16000;
            break;

        case SAMPLING_RATE_8KHz:
            mic_config.sampling_rate = PDM_8000;
            break;

        default:
            ASSERT_ERROR(0);
            break;
        }
            
        pdm_mic_start(&mic_config);
    #endif //CFG_AUDIO_EMULATE_PDM_MIC
    }


    void port_audio_stop_sampling(void)
    {
        #ifdef CFG_AUDIO_EMULATE_PDM_MIC
            audio_swtim_stop();
        #else
            pdm_mic_stop();
            // Flush buffer to discard extra DMA packets
            audio_init_fifo();
        #endif
        arch_restore_sleep_mode();
    }


    void declare_audio_gpios(void)
    {
        PORT_RESERVE_GPIO(app_audio_pins[AUDIO_CLK_PIN]);
        PORT_RESERVE_GPIO(app_audio_pins[AUDIO_DATA_PIN]);
    }


    void init_audio_gpios(void)
    {
        PORT_SET_PIN_FUNCTION_DEFAULT(app_audio_pins[AUDIO_CLK_PIN]);
        PORT_SET_PIN_FUNCTION_DEFAULT(app_audio_pins[AUDIO_DATA_PIN]);
    }


#endif //HAS_AUDIO

/*****
 * \}
 * \}
 * \}
*****/
