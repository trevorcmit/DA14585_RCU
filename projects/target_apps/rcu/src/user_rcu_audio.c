/******************************************************************************************
* \file user_rcu_audio.c
* \brief RCU Audio function implementation.
*****************************************************************************************/

/******************************************************************************************
* \addtogroup USER
* \{
* \addtogroup USER_APP
* \{
* \addtogroup APP_RCU_AUDIO
* \{
*****************************************************************************************/

/*****************************************************************************************
* INCLUDE FILES
*****************************************************************************************/

#ifdef HAS_AUDIO
    #include "user_rcu.h"
    #include "user_rcu_audio.h"
    #include "user_modules.h"
    #include "user_rcu_audio.h"
    #include "port_platform.h"
    #include "l2cm.h"
    #include "gattc.h"
    #include "co_math.h"

    #ifdef AUDIO_USE_CUSTOM_PROFILE
        #include <user_custs1_def.h>
        #include "custs1_task.h"
    #endif

    #ifdef AUDIO_TEST_MODE
        #include "port_timer.h"
    #endif
    
    #ifdef HAS_PWR_MGR
        #include "user_pwr_mgr.h"
    #endif

/*
 ****************************************************************************************
 * DEFINES
******************************************************************************************/

    #if defined(AUDIO_USE_CUSTOM_PROFILE) && defined(AUDIO_USE_THREE_HID_REPORTS)
        #error "AUDIO_USE_CUSTOM_PROFILE and AUDIO_USE_THREE_HID_REPORTS cannot be defined at the same time"
    #endif

    #if (BLE_CUSTOM1_SERVER)
        #ifdef AUDIO_USE_CUSTOM_PROFILE
            #ifdef AUDIO_USE_THREE_HID_REPORTS
                #warning "AUDIO_USE_THREE_HID_REPORTS is not used when custom audio profile is used"
            #endif
        #else
            #warning "Custom audio profile is enabled but not used by application. Please define AUDIO_USE_CUSTOM_PROFILE to use it."
        #endif
    #else    
        #ifdef AUDIO_USE_CUSTOM_PROFILE
            #error "Custom audio profile must be enabled when AUDIO_USE_CUSTOM_PROFILE is defined"
        #endif
    #endif

    #ifndef HAS_BLE_STREAM
        #error "BLE Stream module is required"
    #endif

    #if defined(AUDIO_CONTROL_ESCAPE_VALUE) && defined(CFG_APP_STREAM_PACKET_BASED)
        #error "INBAND control information cannot be used with packet based stream"
    #endif
    
    #ifdef CFG_AUDIO_ADAPTIVE_RATE
typedef struct adpcm_mode_env_s {
    bool auto_mode;
    uint32_t prev_timestamp;
    app_audio_adpcm_mode_t mode;
    app_audio_adpcm_mode_t min_mode;
    uint8_t inc_step;
    uint32_t data_bandwidth;
} adpcm_mode_env_t;
    #endif

    #if defined(HAS_AUDIO_STATS) && DEVELOPMENT_DEBUG  
typedef struct audio_stats_s {
    uint32_t prev_timestamp;
    uint32_t latency;
    uint32_t latency_max;
    uint16_t audio_buffer_used;
    uint16_t audio_sbuffer_used;
    uint16_t stream_buffer_used;
    uint16_t audio_buffer_used_max;
    uint16_t audio_sbuffer_used_max;
    uint16_t stream_buffer_used_max;
} audio_stats_t;
    #endif

typedef enum audio_ctrl_cmd {
        AUDIO_STOP = 0,
        AUDIO_BUTTON_PRESSED,
        AUDIO_STARTED_FROM_HOST
} audio_ctrl_cmd_t;


/****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
*****************************************************************************************/
extern bool skip_slave_latency_flag;
extern struct gapc_param_updated_ind user_connection_params;

bool user_audio_samples_available    __PORT_RETAINED;
bool stop_when_buffer_empty          __PORT_RETAINED;
uint16_t user_max_att_packet_size    __PORT_RETAINED;
uint16_t user_fixed_att_packet_size  __PORT_RETAINED;
uint16_t user_audio_packet_size      __PORT_RETAINED;
uint16_t user_mtu_size               __PORT_RETAINED;

    #ifdef CFG_AUDIO_ADAPTIVE_RATE
adpcm_mode_env_t adpcm_env           __PORT_RETAINED;
bool user_audio_force_custom_bitrate __PORT_RETAINED;
    #endif

    #if defined(CFG_APP_STREAM_PACKET_BASED) && !defined(CFG_APP_STREAM_FIFO_PREDEFINED)
    uint16_t user_audio_enc_size     __PORT_RETAINED;
    #endif

    #ifdef AUDIO_USE_THREE_HID_REPORTS    
uint8_t user_audio_stream_report_nr __PORT_RETAINED;
uint16_t user_audio_data_hdl[3]     __PORT_RETAINED;
    #else
const uint8_t user_audio_stream_report_nr = 0;
uint16_t user_audio_data_hdl[1]     __PORT_RETAINED;
    #endif

uint16_t user_audio_ctrl_in_hdl     __PORT_RETAINED;

    #ifdef AUDIO_USE_CUSTOM_PROFILE
const uint16_t user_audio_report_length = DLG_AUDIO_CTRL_CHAR_LEN;
    #else
const uint16_t user_audio_report_length = HID_VENDOR_DEFINED_REPORT_SIZE;
    #endif
    
    #if defined(HAS_AUDIO_STATS) && DEVELOPMENT_DEBUG  
audio_stats_t audio_stats __PORT_RETAINED;
    #endif

uint8_t audio_control_packet_data[user_audio_report_length] __PORT_RETAINED;
uint16_t audio_control_packet_length                        __PORT_RETAINED;
bool audio_control_packet_pending                           __PORT_RETAINED;

    #ifdef AUDIO_TEST_MODE                    
bool user_audio_test_mode __PORT_RETAINED;
    #endif


/**************************************************************************************
 * FUNCTION DEFINITIONS
***************************************************************************************/
 
void user_audio_send_att_packet_size_report(void);
  
void user_audio_set_packet_size(void)
{
    #if APP_STREAM_USE_CIRCULAR_BUFFER
    extern uint8_t last_connection_idx;
    uint16_t buffer_size, min_mtu;
        
    if (user_fixed_att_packet_size)
    {
        // Host application has set the ATT packet size
        user_audio_packet_size = user_fixed_att_packet_size - 3; // The ATT header is 3 bytes
    } 
    else
    { 
        // aAtomatic packet size calculation
        user_mtu_size = gattc_get_mtu(last_connection_idx);
        uint16_t effective_mtu_size = user_max_att_packet_size ? co_min(user_mtu_size, user_max_att_packet_size) : user_mtu_size;
        
        #if (RWBLE_SW_VERSION_MAJOR >= 8) 
            buffer_size = l2cm_get_buffer_size(last_connection_idx); // 27 if packet length extension is not used
        #else
            buffer_size = 27;
        #endif
            
        const uint32_t BANDWIDTH_MULT = ((8 * MAX_NUM_OF_PACKETS_PER_CONNECTION * 1000000)) / (user_connection_params.con_interval*1250);
        uint32_t bandwidth;
        if(effective_mtu_size > (buffer_size - 4))
        {   // The L2CAP header is 4 bytes
            // The MTU will not fit in one packet
            uint8_t packets_per_mtu = (effective_mtu_size + 4 + buffer_size - 1) / buffer_size;
            bandwidth = (packets_per_mtu * buffer_size - 7) * BANDWIDTH_MULT / packets_per_mtu; // The L2CAP header is 4 bytes and the ATT header is 3 bytes
        }
        else
        {
            // The MTU will fit in one packet
            bandwidth = (effective_mtu_size - 3) * BANDWIDTH_MULT; // in Kbps
        }

        #ifdef CFG_AUDIO_ADAPTIVE_RATE    
        adpcm_env.data_bandwidth = bandwidth;
        
#ifdef CFG_AUDIO_ADAPTIVE_RATE        
        // Calculate the minimum ADPCM mode that can be used for the active connection interval
        if(bandwidth >= ADPM_MODE_TO_RATE(ADPCM_MODE_64KBPS_4_16KHZ)) {
            adpcm_env.min_mode = ADPCM_MODE_64KBPS_4_16KHZ;
        }
        else {
            adpcm_env.min_mode = (app_audio_adpcm_mode_t) (4 - bandwidth/16000);
        }
        dbg_printf(DBG_APP_LVL, "min ADPCM mode: %d\r\n", adpcm_env.min_mode);
#endif
        
        if(adpcm_env.min_mode == ADPCM_MODE_MIN) {
            adpcm_env.min_mode = ADPCM_MODE_24KBPS_3_8KHZ;
        }
        
        uint32_t BANDWIDTH_NEEDED = ADPM_MODE_TO_RATE_120(adpcm_env.min_mode);
        #else
            #define BANDWIDTH_NEEDED ADPM_MODE_TO_RATE_120(ADPCM_DEFAULT_MODE)
        #endif
        
        
        if(bandwidth > BANDWIDTH_NEEDED) {
        // BANDWIDTH_NEEDED must fit in data part of the packet that is MTU - 3bytes
        // bandwidth is used for MTU + 4bytes of L2CAP header
        // bandwidth = (min_mtu+4) * 1000 / CON_INTERVAL * MAX_NUM_OF_PACKETS_PER_CONNECTION * 8
        // bandwidth -> min_mtu+4 (4 is the L2CAP header)
        // BANDWIDTH_NEEDED -> min_mtu - 3 (3 is the GATT header)  
            min_mtu = BANDWIDTH_NEEDED/BANDWIDTH_MULT+3;
            min_mtu += (min_mtu > buffer_size-4) ? (buffer_size - 1) : 0; // Add (buffer_size - 1) so that min_mtu is recalculated below
            min_mtu = co_max(min_mtu, 50);
            min_mtu = co_min(min_mtu, effective_mtu_size);
        }
        else
        {
            min_mtu = effective_mtu_size;
        }

        if (min_mtu > buffer_size - 4)
        {
            min_mtu = (min_mtu - (buffer_size-4))/buffer_size*buffer_size + buffer_size - 4;
        }
        
        user_audio_packet_size = min_mtu - 3;
    }
        #ifndef CFG_APP_STREAM_PACKET_BASED 
	        #ifndef AUDIO_CONTROL_ESCAPE_VALUE
	// Ensuse that packets transferred contain whole ADPM samples even when 3bps are used in IMA-ADPCM encoder.
    user_audio_packet_size = user_audio_packet_size / 3 * 3; 
    	    #endif
        
    app_stream_set_packet_size(user_audio_packet_size);
        #endif
    
    dbg_printf(DBG_APP_LVL, "CI: %d (x1.25) msec, Packet size: %d, MTU size: %d, Max packet size: %d, Audio packet size: %d\r\n",
               (uint16_t)(user_connection_params.con_interval), buffer_size, user_mtu_size, user_max_att_packet_size, user_audio_packet_size);

    user_audio_send_att_packet_size_report();
    user_audio_set_config_data();
    #endif
}

void user_audio_get_handles(void)
{
    // Get audio handles used for sending notifications directly to L2CAP
    #ifdef AUDIO_USE_CUSTOM_PROFILE
        #if (RWBLE_SW_VERSION_MAJOR >= 8)      
    uint16_t custs1_get_att_handle(uint8_t att_idx);
        #else
            #define custs1_get_att_handle(handle) (custs1_env.shdl + handle)
        #endif

    user_audio_ctrl_in_hdl = custs1_get_att_handle(DLG_AUDIO_IDX_CTRL_VAL);
    user_audio_data_hdl[0] = custs1_get_att_handle(DLG_AUDIO_IDX_AUDIO_DATA_VAL);
    #else
    user_audio_ctrl_in_hdl = app_hogpd_report_handle(HID_STREAM_CTRL_IN_IDX);
    user_audio_data_hdl[0] = app_hogpd_report_handle(HID_AUDIO_DATA_1_IDX);
        #ifdef AUDIO_USE_THREE_HID_REPORTS
    user_audio_data_hdl[1] = app_hogpd_report_handle(HID_AUDIO_DATA_2_IDX);
    user_audio_data_hdl[2] = app_hogpd_report_handle(HID_AUDIO_DATA_3_IDX);
        #endif
    #endif
}

    #ifdef AUDIO_CONTROL_ESCAPE_VALUE    
        /****************************************************************************************
         * \brief Send audio control command in-band with audio data. This function cannot be used with packet based stream.
         * \param[in] cmd    The command to be sent in band with audio data
         * \param[in] param  The parameter of the command
        *****************************************************************************************/  
        static void user_audio_send_inband_command(app_audio_inband_cmd_t cmd, uint8_t param)
        {
            ASSERT_ERROR(param <= 0x0F);
            
            const uint16_t command_length = 2;
            
            uint8_t *buffer = app_stream_fifo_get_priority_write_dataptr(command_length);
            
            if (buffer == NULL)
            {
                ASSERT_WARNING(buffer != NULL); // FIFO is full. In-band command will be discarded.
                return;
            }
            
            buffer[0] = AUDIO_CONTROL_ESCAPE_VALUE;
            buffer[1] = cmd | param;
            app_stream_fifo_commit_write_pkt(command_length, user_audio_data_hdl[user_audio_stream_report_nr]);
        }
    #endif

/****************************************************************************************
 * \brief Set the ADPCM mode and start audio sampling and encoding. If in-band commands
 *        are supported the AUDIO_CONTROL_SET_ADPCM_MODE_CMD is also sent to the host
 * \param[in]  mode The ADPCM mode. If mode is ADPCM_MODE_MIN then the mode defined in adpcm_env.min_mode is used.
*****************************************************************************************/  
static void user_audio_set_adpm_mode_and_start(app_audio_adpcm_mode_t mode)
{
    #ifdef CFG_AUDIO_ADAPTIVE_RATE
        if (mode == ADPCM_MODE_MIN)
        {
            mode = adpcm_env.min_mode;
        }
        adpcm_env.mode = mode;
        
        #ifdef CFG_AUDIO_IMA_ADPCM
            app_audio_set_adpcm_mode(mode);
            dbg_printf(DBG_APP_LVL, "ADPCM mode: %d\r\n", mode);
        #endif
    #endif

    #if defined(CFG_AUDIO_IMA_ADPCM) && defined(AUDIO_CONTROL_ESCAPE_VALUE)
        user_audio_send_inband_command(AUDIO_CONTROL_SET_ADPCM_MODE_CMD, mode);
    #endif
        
    app_audio_start();
}


#if defined(AUDIO_CONTROL_ESCAPE_VALUE) && defined(CFG_AUDIO_ADAPTIVE_RATE)
/****************************************************************************************
 * \brief Change the ADPCM mode according to the available channel bandwidth. ADPCM
 *        rate is decreased when the stream FIFO is about to get full. ADPCM rate is 
 *        increased when channel bandwidth is enough for the new rate.
*****************************************************************************************/  
static void user_audio_adapt_rate(void)
{
    #ifdef CFG_AUDIO_ADAPTIVE_RATE
    if (adpcm_env.auto_mode == false)
    {
        return;
    }
    
    static const uint32_t CLOSED_LOOP_T = 10; // in msec*10
    static const uint8_t  DEC_THRES = 60;
    static const uint8_t  INC_THRES = 12;
    static const uint8_t  CLOSED_LOOP_INC_STEPS = 1000/10/CLOSED_LOOP_T; // Check if bitrate can be increased when 
                                                                         // FIFO level is below threshold for 1000 ms
    uint32_t timestamp = port_get_time();
        
    if (app_stream_is_enabled() && timestamp - adpcm_env.prev_timestamp > CLOSED_LOOP_T)
    {
        adpcm_env.prev_timestamp = timestamp;
        
        uint8_t usage = app_stream_get_fifo_usage();
        dbg_printf(DBG_APP_LVL, "t: %d, f: %d\r\n", timestamp*10, usage);
        
        if (usage < INC_THRES)
        {
            if (adpcm_env.inc_step >= CLOSED_LOOP_INC_STEPS)
            {
                adpcm_env.inc_step = 1;
                if (adpcm_env.mode <= adpcm_env.min_mode) { return; }

                #ifdef BLE_THROUGHPUT_METRICS        
                    extern uint32_t ble_throughput_rx_err;
                    extern uint32_t ble_throughput_rx_pkt;
                    
                    uint32_t bandwidth = adpcm_env.data_bandwidth * ble_throughput_rx_pkt/( ble_throughput_rx_pkt + ble_throughput_rx_err);
                    ble_throughput_rx_err = 0;
                    ble_throughput_rx_pkt = 0;
                    
                    dbg_printf(DBG_APP_LVL, "BW: %d, required BW: %d\r\n", bandwidth, ADPM_MODE_TO_RATE(adpcm_env.mode-1));
                    if (bandwidth < ADPM_MODE_TO_RATE(adpcm_env.mode-1) )
                    {
                        return;
                    }
                #else
                    #warning "BLE_THROUGHPUT_METRICS must be defined to properly calculate available bandwidth"
                #endif

                adpcm_env.mode--;
                user_audio_set_adpm_mode_and_start(adpcm_env.mode);
            }
            else
            {
                adpcm_env.inc_step++;
            }
        }
        else
        {
            adpcm_env.inc_step = 1;
            if (usage > (DEC_THRES + adpcm_env.mode*10))
            {
                if (adpcm_env.mode < ADPCM_MODE_24KBPS_3_8KHZ)
                {
                    adpcm_env.mode++;
                    user_audio_set_adpm_mode_and_start(adpcm_env.mode);
                }
            }
        }
    }
    #endif
}
#endif

/*****************************************************************************************
 * \brief CTRL_IN packet data format
 * byte 0: stream control. 1=enable, 0=disable
 * byte 1: notification type. 0 = Audio stream enable/disable command
 *                            1 = Audio stream configuration report
 *                            2 = Keyboard key report
 *                            3 = debug info report
 *                            4 = set decode mode (legacy, not implemented)
 *                            5 = Connection parameter report
 *                            6 = ATT packet size report
 * byte 2: report length
 * byte 3..19: report data
 *    For stream enable report: byte 3: 0 = Legacy Audio stream format. Data are transmitted using three reports with IDs 6,7, and 8
 *                                      report_id = the report ID of the HID report used for streaming data 
 *                              byte 4: bit0: 1 = in-band control information is used
 *                                      bit1: 1 = adaptive rate control is used
 *                                      bit2: 1 = non packet based audio data, 0 = packet based audio data
 *                                      bit3: 1 = support enhanced command set in byte 1 of CTRL_OUT packet
 *                              byte 5: ADPCM mode 
 *                                           bit4..5: 2 = automatic, 
 *                                                   1 = fixed, 
 *                                           bit0..3: current (or fixed) ADPCM mode
 *                                           0 is reserved for legacy mode (No ADPCM mode 
 *                                           information available)
 *                              byte 6:      1 = numberpad keyboard page
 *                                           2 = navigation keyboard page (when motion is active)
 *                                           0 = Reserved for DA14582 refdes
 *                              byte 7..8:   current ATT packet size (least significant byte first)
 *                              byte 9..10:  ATT MTU size (least significant byte first)
 *                              byte 11..12: connection interval (least significant byte first in steps of 1.25msec)
 *                              byte 13..14: slave latency (least significant byte first)
 *                              byte 15..16: supervision timeout (least significant byte first in steps of 10msec)
 *    For key report: byte 3..5: extended key report
 *                    byte 6..10: normal key report
 *                    byte 12: key page layout: 1: number page, 2: cursor page
 *    For debug info report: 
 *                    byte 3: 0
 *                    byte 4: 3
 *                    byte 5: Stream FIFO overflow
 *                    byte 6: Audio buffer overflow
 *                    byte 7: Stream FIFO size
 *                    byte 8: Stream FIFO write pointer
 *                    byte 9: Stream FIFO read pointer
 *    For connection parameter report:
 *                    byte3..4  = connection interval (least significant byte first in steps of 1.25msec)
 *                    byte5..6  = slave latency
 *                    byte7..8  = supervision timeout (least significant byte first in steps of 10msec)
 *    For ATT packet size report:
 *                    byte3..4: current ATT packet size (least significant byte first)
 *                    byte5..6: ATT MTU size (least significant byte first)
*****************************************************************************************/
 
typedef enum {
    CTRL_IN_ENABLE_REPORT          = 0,
    CTRL_IN_CONFIG                 = 1,
    CTRL_IN_KEY_REPORT             = 2,
    CTRL_IN_DEBUG_INFO_REPORT      = 3,
    CTRL_IN_CONN_PARAMS_REPORT     = 5,
    CTRL_IN_ATT_PACKET_SIZE_REPORT = 6,
} stream_report_type_t;

enum {
    CTRL_IN_ENABLE_REPORT_STREAM_EN_POS                    =  0,
    CTRL_IN_ENABLE_REPORT_TYPE_POS                         =  1,
    CTRL_IN_ENABLE_REPORT_LENGTH_POS                       =  2,
    CTRL_IN_ENABLE_REPORT_STREAM_FORMAT_POS                =  3,
    CTRL_IN_ENABLE_REPORT_STREAM_FEATURES_POS              =  4,
    CTRL_IN_ENABLE_REPORT_ADPCM_MODE_POS                   =  5,
    CTRL_IN_ENABLE_REPORT_KBD_PAGE_POS                     =  6,
    CTRL_IN_ENABLE_REPORT_ATT_PACKET_SIZE_LSB_POS          =  7,
    CTRL_IN_ENABLE_REPORT_ATT_PACKET_SIZE_MSB_POS          =  8,
    CTRL_IN_ENABLE_REPORT_ATT_MTU_SIZE_LSB_POS             =  9,
    CTRL_IN_ENABLE_REPORT_ATT_MTU_SIZE_MSB_POS             = 10,
    CTRL_IN_ENABLE_REPORT_CONN_PARAM_INT_LSB_POS           = 11,
    CTRL_IN_ENABLE_REPORT_CONN_PARAM_INT_MSB_POS           = 12,
    CTRL_IN_ENABLE_REPORT_CONN_PARAM_SL_LSB_POS            = 13,
    CTRL_IN_ENABLE_REPORT_CONN_PARAM_SL_MSB_POS            = 14,
    CTRL_IN_ENABLE_REPORT_CONN_PARAM_SUP_TIMEOUT_LSB_POS   = 15,
    CTRL_IN_ENABLE_REPORT_CONN_PARAM_SUP_TIMEOUT_MSB_POS   = 16,
    CTRL_IN_ENABLE_REPORT_SIZE
};

enum {
    CTRL_IN_ATT_PACKET_SIZE_REPORT_STREAM_EN_POS           = 0,
    CTRL_IN_ATT_PACKET_SIZE_REPORT_TYPE_POS                = 1,
    CTRL_IN_ATT_PACKET_SIZE_REPORT_LENGTH_POS              = 2,
    CTRL_IN_ATT_PACKET_SIZE_REPORT_ATT_PACKET_SIZE_LSB_POS = 3,
    CTRL_IN_ATT_PACKET_SIZE_REPORT_ATT_PACKET_SIZE_MSB_POS = 4,
    CTRL_IN_ATT_PACKET_SIZE_REPORT_ATT_MTU_SIZE_LSB_POS    = 5,
    CTRL_IN_ATT_PACKET_SIZE_REPORT_ATT_MTU_SIZE_MSB_POS    = 6,
    CTRL_IN_ATT_PACKET_SIZE_REPORT_SIZE
};

enum {
    CTRL_IN_KEY_REPORT_STREAM_EN_POS           = 0,
    CTRL_IN_KEY_REPORT_TYPE_POS                = 1,
    CTRL_IN_KEY_REPORT_LENGTH_POS              = 2,
    CTRL_IN_KEY_REPORT_DATA_POS                = 3,
    CTRL_IN_KEY_REPORT_KEYBOARD_PAGE_POS       = 12,
    CTRL_IN_KEY_REPORT_SIZE
};

enum {
    CTRL_IN_DEBUG_REPORT_STREAM_EN_POS           = 0,
    CTRL_IN_DEBUG_REPORT_TYPE_POS                = 1,
    CTRL_IN_DEBUG_REPORT_LENGTH_POS              = 2,
    CTRL_IN_DEBUG_REPORT_RESERVED1_POS           = 3,
    CTRL_IN_DEBUG_REPORT_RESERVED2_POS           = 4,
    CTRL_IN_DEBUG_REPORT_AUDIO_DATA_POS          = 5,
    CTRL_IN_DEBUG_REPORT_STREAM_DATA_POS         = 7,
};

enum {
    CTRL_IN_CONN_PARAMS_REPORT_STREAM_EN_POS               = 0,
    CTRL_IN_CONN_PARAMS_REPORT_TYPE_POS                    = 1,
    CTRL_IN_CONN_PARAMS_REPORT_LENGTH_POS                  = 2,
    CTRL_IN_CONN_PARAMS_REPORT_INT_LSB_POS                 = 3,
    CTRL_IN_CONN_PARAMS_REPORT_INT_MSB_POS                 = 4,
    CTRL_IN_CONN_PARAMS_REPORT_SL_LSB_POS                  = 5,
    CTRL_IN_CONN_PARAMS_REPORT_SL_MSB_POS                  = 6,
    CTRL_IN_CONN_PARAMS_REPORT_SUP_TIMEOUT_LSB_POS         = 7,
    CTRL_IN_CONN_PARAMS_REPORT_SUP_TIMEOUT_MSB_POS         = 8,
    CTRL_IN_CONN_PARAMS_REPORT_SIZE
};

    #define CTRL_IN_ENABLE_REPORT_ADPCM_AUTO_MODE_FLAG         0x20
    #define CTRL_IN_ENABLE_REPORT_ADPCM_FIXED_MODE_FLAG        0x10

    #define CTRL_IN_ENABLE_REPORT_INBAND_FLAG                  0x01
    #define CTRL_IN_ENABLE_REPORT_ADAPTIVE_RATE_FLAG           0x02
    #define CTRL_IN_ENABLE_REPORT_NON_PACKET_BASED_AUDIO_FLAG  0x04
    #define CTRL_IN_ENABLE_REPORT_ENHANCED_COMMAND_SET_FLAG    0x08


/****************************************************************************************
 * \brief Create the configuration packet that will be sent when:
 *        1. The host reads the STREAM_CTRL_IN HID report 
 *        2. The host reads the custom audio profile configuration characteristic
 *        3. The RCU sends an enable report to notification to STREAM_CTRL_IN HID report
 *           or to the custom audio profile control characteristic
 * \param[out] data The data of the configuration packet
 * \param[in]  audio_command The command of the notification
 * \param[in]  report_type The report type of the notification
 * \return     The size of the configuration packet
*****************************************************************************************/ 
static uint16_t user_audio_create_ctrl_in_packet(uint8_t *data, audio_ctrl_cmd_t audio_command, stream_report_type_t report_type)
{
    data[CTRL_IN_ENABLE_REPORT_STREAM_EN_POS] = audio_command;
    data[CTRL_IN_ENABLE_REPORT_TYPE_POS     ] = report_type;
    data[CTRL_IN_ENABLE_REPORT_LENGTH_POS   ] = CTRL_IN_ENABLE_REPORT_SIZE - CTRL_IN_ENABLE_REPORT_LENGTH_POS - 1; // length
    
    #if defined(AUDIO_USE_THREE_HID_REPORTS) || defined(AUDIO_USE_CUSTOM_PROFILE)
        data[CTRL_IN_ENABLE_REPORT_STREAM_FORMAT_POS] = 0;
    #else
        data[CTRL_IN_ENABLE_REPORT_STREAM_FORMAT_POS] = HID_AUDIO_DATA_1_REPORT_ID;
    #endif
    
    data[CTRL_IN_ENABLE_REPORT_STREAM_FEATURES_POS] = CTRL_IN_ENABLE_REPORT_ENHANCED_COMMAND_SET_FLAG;
    
    #if !defined(CFG_APP_STREAM_PACKET_BASED) && defined(AUDIO_CONTROL_ESCAPE_VALUE)
        data[CTRL_IN_ENABLE_REPORT_STREAM_FEATURES_POS] |= CTRL_IN_ENABLE_REPORT_INBAND_FLAG; // use inband control information
    #endif

    #ifndef CFG_APP_STREAM_PACKET_BASED
        data[CTRL_IN_ENABLE_REPORT_STREAM_FEATURES_POS] |= CTRL_IN_ENABLE_REPORT_NON_PACKET_BASED_AUDIO_FLAG;
    #endif
    
    #ifdef CFG_AUDIO_ADAPTIVE_RATE
        data[CTRL_IN_ENABLE_REPORT_STREAM_FEATURES_POS] |= CTRL_IN_ENABLE_REPORT_ADAPTIVE_RATE_FLAG; // use adaptive audio rate
        data[CTRL_IN_ENABLE_REPORT_ADPCM_MODE_POS] = CTRL_IN_ENABLE_REPORT_ADPCM_AUTO_MODE_FLAG | adpcm_env.mode;
    #else    
        data[CTRL_IN_ENABLE_REPORT_ADPCM_MODE_POS] = CTRL_IN_ENABLE_REPORT_ADPCM_FIXED_MODE_FLAG | ADPCM_DEFAULT_MODE;
    #endif

    #ifdef HAS_KBD    
        extern bool user_fn_locked;
        data[CTRL_IN_ENABLE_REPORT_KBD_PAGE_POS] = user_fn_locked ? 2 : 1;
    #else    
        data[CTRL_IN_ENABLE_REPORT_KBD_PAGE_POS] = 0;
    #endif
    
    #if APP_STREAM_USE_CIRCULAR_BUFFER
        port_write16(data + CTRL_IN_ENABLE_REPORT_ATT_PACKET_SIZE_LSB_POS, user_audio_packet_size+3);
    #else
        port_write16(data + CTRL_IN_ENABLE_REPORT_ATT_PACKET_SIZE_LSB_POS, APP_STREAM_PACKET_SIZE+3);
    #endif
    
    port_write16(data + CTRL_IN_ENABLE_REPORT_ATT_MTU_SIZE_LSB_POS          , user_mtu_size);
    port_write16(data + CTRL_IN_ENABLE_REPORT_CONN_PARAM_INT_LSB_POS        , user_connection_params.con_interval);
    port_write16(data + CTRL_IN_ENABLE_REPORT_CONN_PARAM_SL_LSB_POS         , user_connection_params.con_latency);
    port_write16(data + CTRL_IN_ENABLE_REPORT_CONN_PARAM_SUP_TIMEOUT_LSB_POS, user_connection_params.sup_to);
    
    return CTRL_IN_ENABLE_REPORT_SIZE;
}


/****************************************************************************************
 * \brief Create the configuration packet that will be sent when the host reads 
 *        the STREAM_CTRL_IN HID report or the custom audio profile configuration characteristic.
 * \param[out] data The data of the configuration packet
 * \return The size of the configuration packet
*****************************************************************************************/ 
static uint16_t user_audio_create_configuration_packet(uint8_t *data)
{
    return user_audio_create_ctrl_in_packet(data, AUDIO_STOP, CTRL_IN_CONFIG);
}


/****************************************************************************************
 * \brief Send an enable packet notification to the STREAM_CTRL_IN HID report or to the 
 *        custom audio profile control characteristic.
 * \param[in]  audio_command The command to be sent
*****************************************************************************************/  
static void user_audio_send_enable(audio_ctrl_cmd_t audio_command)
{    
#ifdef CFG_AUDIO_ADAPTIVE_RATE
    if(audio_command == AUDIO_BUTTON_PRESSED) {
        user_audio_force_custom_bitrate = false;
    }
#endif        
    
    if (audio_command == AUDIO_STOP || app_stream_is_enabled() == false) {
        // Send notification
        audio_control_packet_length = user_audio_create_ctrl_in_packet(audio_control_packet_data, audio_command, CTRL_IN_ENABLE_REPORT);
        ASSERT_ERROR(audio_control_packet_length <= user_audio_report_length);
        if(port_send_notification(user_audio_ctrl_in_hdl, audio_control_packet_data, audio_control_packet_length, false) == false) {
            // There was already a control packet pending which has been been discarded
            // If this assertion hits then using a FIFO for control packets should be consided
            ASSERT_WARNING(audio_control_packet_pending == false); 
            audio_control_packet_pending = true;
        }
        dbg_printf(DBG_APP_LVL, "Audio %s request sent\r\n", audio_command == AUDIO_STOP ? "stop" : "start");

    }
}

#if APP_STREAM_USE_CIRCULAR_BUFFER
/*****************************************************************************************
 * \brief Send a CTRL_IN_ATT_PACKET_SIZE_REPORT packet notification
******************************************************************************************/  
static void user_audio_send_att_packet_size_report(void)
{
    uint8_t pkt_data[user_audio_report_length];
    
    pkt_data[CTRL_IN_ATT_PACKET_SIZE_REPORT_STREAM_EN_POS] = 0;
    pkt_data[CTRL_IN_ATT_PACKET_SIZE_REPORT_TYPE_POS  ]    = CTRL_IN_ATT_PACKET_SIZE_REPORT;
    pkt_data[CTRL_IN_ATT_PACKET_SIZE_REPORT_LENGTH_POS]    = 4;
    port_write16(pkt_data + CTRL_IN_ATT_PACKET_SIZE_REPORT_ATT_PACKET_SIZE_LSB_POS, user_audio_packet_size+3);
    port_write16(pkt_data + CTRL_IN_ATT_PACKET_SIZE_REPORT_ATT_MTU_SIZE_LSB_POS   , user_mtu_size);
    
    port_send_notification(user_audio_ctrl_in_hdl, pkt_data, CTRL_IN_ATT_PACKET_SIZE_REPORT_SIZE, false);
}
#endif

void user_audio_send_conn_params_report(void)
{
    uint8_t pkt_data[user_audio_report_length];
    
    pkt_data[CTRL_IN_CONN_PARAMS_REPORT_STREAM_EN_POS] = 0;
    pkt_data[CTRL_IN_CONN_PARAMS_REPORT_TYPE_POS  ]    = CTRL_IN_CONN_PARAMS_REPORT;
    pkt_data[CTRL_IN_CONN_PARAMS_REPORT_LENGTH_POS]    = 6;
    port_write16(pkt_data + CTRL_IN_CONN_PARAMS_REPORT_INT_LSB_POS        , user_connection_params.con_interval);
    port_write16(pkt_data + CTRL_IN_CONN_PARAMS_REPORT_SL_LSB_POS         , user_connection_params.con_latency);
    port_write16(pkt_data + CTRL_IN_CONN_PARAMS_REPORT_SUP_TIMEOUT_LSB_POS, user_connection_params.sup_to);
    
    port_send_notification(user_audio_ctrl_in_hdl, pkt_data, CTRL_IN_CONN_PARAMS_REPORT_SIZE, false);
}

/*****************************************************************************************
 * \brief CTRL_OUT packet data format
 * byte 0: stream enable. 1=enable, 0=disable
 * byte 1: command: 0x00 = stream enable/disable
 *                  0x0M = change audio mode. M=2,3,4,5,6 
 *                         Mode = automatic if M = 6, otherwise mode is M -2
 *                  0x1M = Force audio mode. Same as 0x0M. All 0x0M commands following
 *                         a 0x1M are ignored
 *                  0x81 = Set ATT packet size.
 *                  0x82 = Set connection parameters
 *                  0x83 = Read configuration. A device configuration notification must 
 *                         be sent.
 *                  0x84 = issue system reset
 *                  0x85 = Emulate key presses
 * byte 2: command parameters length
 * byte 3..19: command parameters
 *      For set ATT packet size command:
 *           byte 3..4 = max ATT packet size (least significant byte first)
 *           byte 5..6 = Fixed ATT packet size (least significant byte first). 
 *                       If 0 then automatic
 *      For set connection parameters command:
 *           byte 3..4  = min connection interval (least significant byte first in 
 *                        steps of 1.25msec)
 *           byte 5..6  = max connection interval (least significant byte first in 
 *                        steps of 1.25msec)
 *           byte 7..8  = slave latency (least significant byte first)
 *           byte 9..10 = supervision timeout in steps of 10msec (least significant byte
 *                        first)
 *      For emulate key presses:
 *           byte 3..4   = Initial delay
 *           byte 5      = starting column
 *           byte 6      = starting row
 *           byte 7      = ending column
 *           byte 8      = ending row
 *           byte 9..10  = press duration
 *           byte 10..11 = repeat counter
 *           byte 12..13 = repeat interval
******************************************************************************************/
enum {
    CTRL_OUT_STREAM_ENABLE       = 0x00,
    CTRL_OUT_AUDIO_MODE          = 0x0F,
    CTRL_OUT_FORCE_AUDIO_MODE    = 0x1F,
    CTRL_OUT_SET_ATT_PACKET_SIZE = 0x81,
    CTRL_OUT_SET_CONN_PARAMS     = 0x82,
    CTRL_OUT_GET_CONFIGURATION   = 0x83,
#ifdef ENABLE_OTA_DEBUG     
    CTRL_OUT_SYSTEM_RESET        = 0x84,
    #ifdef HAS_KBD
    CTRL_OUT_EMULATE_KEY_PRESS   = 0x85,
    #endif
#endif    
};

enum {
    CTRL_OUT_STREAM_EN_POS      = 0,
    CTRL_OUT_COMMAND_POS        = 1,
    CTRL_OUT_COMMAND_LENGTH_POS = 2,
    CTRL_OUT_COMMAND_PARAM_POS  = 3,
};

enum {
    CTRL_OUT_ATT_PACKET_SIZE_MAX_LSB_POS = CTRL_OUT_COMMAND_PARAM_POS,
    CTRL_OUT_ATT_PACKET_SIZE_MAX_MSB_POS,
    CTRL_OUT_ATT_PACKET_SIZE_FIXED_LSB_POS,
    CTRL_OUT_ATT_PACKET_SIZE_FIXED_MSB_POS
};

enum {
    CTRL_OUT_CONN_PARAM_INT_MIN_LSB_POS = CTRL_OUT_COMMAND_PARAM_POS,
    CTRL_OUT_CONN_PARAM_INT_MIN_MSB_POS,
    CTRL_OUT_CONN_PARAM_INT_MAX_LSB_POS,
    CTRL_OUT_CONN_PARAM_INT_MAX_MSB_POS,
    CTRL_OUT_CONN_PARAM_LATENCY_LSB_POS,
    CTRL_OUT_CONN_PARAM_LATENCY_MSB_POS,
    CTRL_OUT_CONN_PARAM_SUP_TIMEOUT_LSB_POS,
    CTRL_OUT_CONN_PARAM_SUP_TIMEOUT_MSB_POS,
};

#define CTRL_OUT_FORCE_AUDIO_MODE_MASK 0x10
#define CTRL_OUT_AUDIO_MODE_MASK       0x0F

#if defined(HAS_KBD) && defined(ENABLE_OTA_DEBUG)
#include "port_timer.h"

enum {
    CTRL_OUT_DEBUG_KBD_EMUL_START_DELAY_LSB_POS = CTRL_OUT_COMMAND_PARAM_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_START_DELAY_MSB_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_START_COLUMN_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_START_ROW_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_END_COLUMN_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_END_ROW_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_PRESS_DURATION_LSB_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_PRESS_DURATION_MSB_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_REPEAT_COUNTER_LSB_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_REPEAT_COUNTER_MSB_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_REPEAT_INTERVAL_LSB_POS,
    CTRL_OUT_DEBUG_KBD_EMUL_REPEAT_INTERVAL_MSB_POS,
};

typedef struct kbd_emul_params_s {
    uint8_t start_column;
    uint8_t end_column;
    uint8_t start_row;
    uint8_t end_row;
    uint16_t press_duration;
    uint16_t repeat_counter;
    uint16_t repeat_interval;
    uint8_t current_column;
    uint8_t current_row;
    bool next_press;
    uint16_t count;
} kbd_emul_params_t;

kbd_emul_params_t user_kbd_emul_params __PORT_RETAINED;

void user_rcu_audio_kbd_emul_timer_handler(void)
{
    app_kbd_buffer_add_key(user_kbd_emul_params.current_row, user_kbd_emul_params.current_column, user_kbd_emul_params.next_press);
    user_kbd_emul_params.next_press = !user_kbd_emul_params.next_press;
    
    if(user_kbd_emul_params.next_press == true) {
        user_kbd_emul_params.count += 1;
                
        if(user_kbd_emul_params.count < user_kbd_emul_params.repeat_counter) {
            user_kbd_emul_params.current_column++;
            if(user_kbd_emul_params.current_column > user_kbd_emul_params.end_column) {
                user_kbd_emul_params.current_column = user_kbd_emul_params.start_column;
                user_kbd_emul_params.current_row++;
                if(user_kbd_emul_params.current_row > user_kbd_emul_params.end_row) {
                    user_kbd_emul_params.current_row = user_kbd_emul_params.start_row;
                }
            }
            port_timer_set(USER_AUDIO_KBD_EMUL_TIMER, TASK_APP, (user_kbd_emul_params.next_press == true) ? user_kbd_emul_params.repeat_interval : user_kbd_emul_params.press_duration);
        }
    }
    else {
        port_timer_set(USER_AUDIO_KBD_EMUL_TIMER, TASK_APP, user_kbd_emul_params.press_duration);
    }
}

#endif

/*****************************************************************************************
 * \brief Process the control packet written from the host to either HID CTRL_OUT report
 *        or to the custom audio profile control characteristic.
 *
 * \param[in]  data    The data of the control packet
 * \param[in]  length  The length of the control packet
******************************************************************************************/  
static void user_audio_process_ctrl_packet(const uint8_t *data, uint16_t length)
{
    #ifdef HAS_PWR_MGR
    user_pwr_mgr_reset_inactivity();
    #endif
    
    uint8_t command = data[CTRL_OUT_COMMAND_POS];
	bool enable_stream_reports = data[CTRL_OUT_STREAM_EN_POS] > 0;
    
    dbg_printf(DBG_APP_LVL, "Enable: %x, Command: %x\r\n", data[CTRL_OUT_STREAM_EN_POS], command);
    
    if(enable_stream_reports == false && (command == CTRL_OUT_STREAM_ENABLE || length == 1)) {
        // If length == 1 then the value of the one and only byte in the packet is zero
        // then this is an audio stop packet.
        dbg_puts(DBG_APP_LVL, "Audio stop request received\r\n");
        user_audio_stop(true);
        if(length == 1) {
            return;
        }
    }
    
    #ifdef CFG_AUDIO_ADAPTIVE_RATE    
    switch(command) {
        case CTRL_OUT_STREAM_ENABLE: {
            if(enable_stream_reports == true) {  
                if(user_audio_force_custom_bitrate == false) {  
                    // Configure bitrate to 64Kbps only if the bitrate has not been previously set
                    // Legacy support for host application that do not support the CTRL_OUT_AUDIO_MODE
                    // and CTRL_OUT_FORCE_AUDIO_MODE commands
                    dbg_puts(DBG_APP_LVL, "Audio start request received\r\n");
                    user_audio_start(ADPCM_MODE_64KBPS_4_16KHZ, false);
                }
            }
        } break;
        
        case CTRL_OUT_SET_ATT_PACKET_SIZE: {
            ASSERT_WARNING(data[CTRL_OUT_COMMAND_LENGTH_POS] == 4);
            dbg_puts(DBG_APP_LVL, "ATT packet size change request received\r\n");
                        
            user_max_att_packet_size = port_read16(data + CTRL_OUT_ATT_PACKET_SIZE_MAX_LSB_POS);
            user_fixed_att_packet_size = port_read16(data + CTRL_OUT_ATT_PACKET_SIZE_FIXED_LSB_POS);

            dbg_printf(DBG_APP_LVL, "Max ATT packet size: %d, Fixed ATT packet size: %d\r\n",
                       user_max_att_packet_size, user_fixed_att_packet_size);
            
            user_audio_set_packet_size();
        } break;
    
        case CTRL_OUT_SET_CONN_PARAMS: {
            ASSERT_WARNING(data[CTRL_OUT_COMMAND_LENGTH_POS] == 8);

            dbg_puts(DBG_APP_LVL, "Connection parameters update request received\r\n");
            
            // Allocate a message for GAP
            struct gapc_param_update_cmd* cmd = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CMD,
                                                             TASK_GAPC,
                                                             TASK_APP,
                                                             gapc_param_update_cmd);

            cmd->operation = GAPC_UPDATE_PARAMS;
    #if (RWBLE_SW_VERSION_MAJOR >= 8)           
            cmd->intv_min = port_read16(data + CTRL_OUT_CONN_PARAM_INT_MIN_LSB_POS);
            cmd->intv_max = port_read16(data + CTRL_OUT_CONN_PARAM_INT_MAX_LSB_POS);
            cmd->latency  = port_read16(data + CTRL_OUT_CONN_PARAM_LATENCY_LSB_POS);
            cmd->time_out = port_read16(data + CTRL_OUT_CONN_PARAM_SUP_TIMEOUT_LSB_POS);
            cmd->ce_len_min = 0xFFFF;
            cmd->ce_len_max = 0xFFFF;
            dbg_printf(DBG_APP_LVL, "Min CI: %d(x1.25) msec, Max CI: %d(x1.25) msec, SL:%d, Sup.TO:%d msec\r\n",
                       cmd->intv_min, cmd->intv_max, cmd->latency, cmd->time_out*10);
    #else
            cmd->params.intv_min = port_read16(data + CTRL_OUT_CONN_PARAM_INT_MIN_LSB_POS);
            cmd->params.intv_max = port_read16(data + CTRL_OUT_CONN_PARAM_INT_MAX_LSB_POS);
            cmd->params.latency  = port_read16(data + CTRL_OUT_CONN_PARAM_LATENCY_LSB_POS);
            cmd->params.time_out = port_read16(data + CTRL_OUT_CONN_PARAM_SUP_TIMEOUT_LSB_POS);
            dbg_printf(DBG_APP_LVL, "Min CI: %d(x1.25) msec, Max CI: %d(x1.25) msec, SL:%d, Sup.TO:%d msec\r\n",
                       cmd->params.intv_min, cmd->params.intv_max, cmd->params.latency, cmd->params.time_out*10);
    #endif                      

            // Send the message
            ke_msg_send(cmd);
        } break;
        case CTRL_OUT_GET_CONFIGURATION:
            // Send notification
            audio_control_packet_length = user_audio_create_ctrl_in_packet(audio_control_packet_data, AUDIO_STOP, CTRL_IN_CONFIG);
            ASSERT_ERROR(audio_control_packet_length <= user_audio_report_length);
            if(port_send_notification(user_audio_ctrl_in_hdl, audio_control_packet_data, audio_control_packet_length, false) == false) {
                // There was already a control packet pending which has been been discarded
                // If this assertion hits then using a FIFO for control packets should be consided
                ASSERT_WARNING(audio_control_packet_pending == false); 
                audio_control_packet_pending = true;
            }
            break;
#ifdef ENABLE_OTA_DEBUG
        case CTRL_OUT_SYSTEM_RESET:
            wdg_reload(1);
            wdg_resume();
            while(1);
    #ifdef HAS_KBD
        case CTRL_OUT_EMULATE_KEY_PRESS: {
            ASSERT_WARNING(data[CTRL_OUT_COMMAND_LENGTH_POS] == 4 || data[CTRL_OUT_COMMAND_LENGTH_POS] == 12);
            
            uint16_t start_delay = port_read16(data + CTRL_OUT_DEBUG_KBD_EMUL_START_DELAY_LSB_POS);
            
            user_kbd_emul_params.start_column = data[CTRL_OUT_DEBUG_KBD_EMUL_START_COLUMN_POS];
            user_kbd_emul_params.start_row    = data[CTRL_OUT_DEBUG_KBD_EMUL_START_ROW_POS];
            
            if(data[CTRL_OUT_COMMAND_LENGTH_POS] == 4) {
                user_kbd_emul_params.end_column   = user_kbd_emul_params.start_column;
                user_kbd_emul_params.end_row      = user_kbd_emul_params.start_row;
                user_kbd_emul_params.press_duration  = 50;
                user_kbd_emul_params.repeat_counter  = 1;
            }
            else {
                user_kbd_emul_params.end_column   = data[CTRL_OUT_DEBUG_KBD_EMUL_END_COLUMN_POS];
                user_kbd_emul_params.end_row      = data[CTRL_OUT_DEBUG_KBD_EMUL_END_ROW_POS];

                user_kbd_emul_params.press_duration  = port_read16(data + CTRL_OUT_DEBUG_KBD_EMUL_PRESS_DURATION_LSB_POS);
                ASSERT_WARNING(user_kbd_emul_params.press_duration > 10);
                
                user_kbd_emul_params.repeat_counter  = port_read16(data + CTRL_OUT_DEBUG_KBD_EMUL_REPEAT_COUNTER_LSB_POS);
                ASSERT_WARNING(user_kbd_emul_params.repeat_counter > 0);
        
                user_kbd_emul_params.repeat_interval = port_read16(data + CTRL_OUT_DEBUG_KBD_EMUL_REPEAT_INTERVAL_LSB_POS);
                ASSERT_WARNING(user_kbd_emul_params.repeat_counter == 1 || user_kbd_emul_params.repeat_interval > 10);
            }        
        
            user_kbd_emul_params.current_column = user_kbd_emul_params.start_column;
            user_kbd_emul_params.current_row = user_kbd_emul_params.start_row;
            user_kbd_emul_params.next_press = true;
            user_kbd_emul_params.count = 0;
        
            if(start_delay > 0) {
                port_timer_set(USER_AUDIO_KBD_EMUL_TIMER, TASK_APP, start_delay);
            }
            else {
                user_rcu_audio_kbd_emul_timer_handler();
            }
            break;            
        }
    #endif
        
#endif        
        default: {
            if((command & ~CTRL_OUT_FORCE_AUDIO_MODE) != 0) {
                break;
            }

            // CTRL_OUT_AUDIO_MODE and CTRL_OUT_FORCE_AUDIO_MODE commands
            
            bool force_mode = (command & CTRL_OUT_FORCE_AUDIO_MODE_MASK) != 0;
            
            if(force_mode || user_audio_force_custom_bitrate == false) {
                command &= CTRL_OUT_AUDIO_MODE_MASK;
                
                if (command == 6) { // rate = automatic
                    adpcm_env.mode = adpcm_env.min_mode;
                    adpcm_env.auto_mode = true;
                }
                else if (command > 1) {
                    adpcm_env.auto_mode = false;
                    adpcm_env.mode = (app_audio_adpcm_mode_t)(command - 2);
                }
                
                user_audio_set_config_data();
            }
            
            if (enable_stream_reports == true) {
                if(force_mode) {
                    user_audio_force_custom_bitrate = true;
                }
                dbg_puts(DBG_APP_LVL, "Audio start request received\r\n");
        #ifndef AUDIO_CONTROL_ESCAPE_VALUE        
                // if in-band audio control is used then there is no need to restart the audio.
                app_audio_stop();
                user_audio_start(adpcm_env.mode, false);
        #else
                if(app_stream_is_enabled()) {
                    user_audio_set_adpm_mode_and_start(adpcm_env.mode);
                }
                else {
                    user_audio_start(adpcm_env.mode, false);
                }
        #endif
            }
        } break;
    }
    #else
    if (enable_stream_reports == true) {
        dbg_puts(DBG_APP_LVL, "Audio start request received\r\n");
        #ifdef AUDIO_WAIT_FOR_HOST_START_CMD
        user_audio_start((app_audio_adpcm_mode_t)ADPCM_DEFAULT_MODE, false);
        #endif
    }
    #endif
}

void user_audio_start(app_audio_adpcm_mode_t mode, bool button_pressed)
{
    if(user_is_ble_connected()) {
        if(app_audio_is_active() == false) {
            user_audio_send_enable(button_pressed ? AUDIO_BUTTON_PRESSED : AUDIO_STARTED_FROM_HOST);
        }
        
        if(app_audio_is_active() == false || app_stream_get_fifo_usage() > 90) {
            // If the audio has not been started yet then initialize the stream FIFO
            // to purge previous encoded data if any.
            // New encoded audio data can be placed in the FIFO even before the stream 
            // is started in case the RCU is waiting for a start command from the host.
            app_stream_fifo_init();
            stop_when_buffer_empty = false;
    #ifdef AUDIO_CONTROL_ESCAPE_VALUE    
            user_audio_send_inband_command(AUDIO_CONTROL_RESET_CMD, 0);
    #endif
            
    #ifdef AUDIO_USE_THREE_HID_REPORTS
            user_audio_stream_report_nr = 0;
    #endif
        }
        
    #ifdef AUDIO_WAIT_FOR_HOST_START_CMD
        if(button_pressed == true) {
            skip_slave_latency_flag = true;
        }
        else {
        #if (RWBLE_SW_VERSION_MAJOR >= 8)
            skip_slave_latency_flag = false;
        #endif
            app_stream_start();
        }
    #else
        app_stream_start();
    #endif
        
        user_audio_set_adpm_mode_and_start(mode);
    }
}

void user_audio_stop(bool force_stop)
{
    #if (RWBLE_SW_VERSION_MAJOR >= 8)  
    skip_slave_latency_flag = false;
    #endif
    #ifdef CFG_AUDIO_ADAPTIVE_RATE
    user_audio_force_custom_bitrate = false;
    #endif
    app_audio_stop();
    if(force_stop) {
        if(app_stream_is_enabled()) {
            user_audio_send_enable(AUDIO_STOP);
            app_stream_stop();
        }
        stop_when_buffer_empty = false;
    }
    else {
        if(user_is_ble_connected() == true) {
            if(app_stream_is_enabled()) {
                stop_when_buffer_empty = true;
    #ifndef CFG_APP_STREAM_PACKET_BASED        
                app_stream_ignore_packet_size();
    #endif
            }
            else {
                app_stream_fifo_init(); // clear the stream FIFO
//                user_audio_send_enable(AUDIO_STOP);
            }
        }
    }
}

#ifdef AUDIO_TEST_MODE

void user_rcu_audio_start_audio_test(bool start)
{
    if(start == true) {
        user_audio_test_mode = !user_audio_test_mode;
    }
    else {
        user_audio_test_mode = false;
    }
    
    if(user_audio_test_mode == true) {
        dbg_puts(DBG_APP_LVL, "Audio test started\r\n");
        user_rcu_audio_test_mode_timer_handler();
    }
    else {
        port_timer_clear(USER_AUDIO_TEST_MODE_TIMER, TASK_APP);
        user_audio_stop(false);
        dbg_puts(DBG_APP_LVL, "Audio test ended\r\n");
        USER_LEDS_OFF(led_audio_test_mode_mic_on_param);
        USER_LEDS_OFF(led_audio_test_mode_mic_off_param);
    }
}

void user_rcu_audio_test_mode_timer_handler(void)
{
    if(user_audio_test_mode == true) {
        if(app_audio_is_active() == false) {
            USER_LEDS_OFF(led_audio_test_mode_mic_off_param);
            rwip_schedule();
            USER_LEDS_RAMP(led_audio_test_mode_mic_on_param);
#ifdef CFG_AUDIO_ADAPTIVE_RATE            
            user_audio_start(ADPCM_MODE_MIN, true);
#else            
            user_audio_start(ADPCM_DEFAULT_MODE, true);
#endif            
            port_timer_set(USER_AUDIO_TEST_MODE_TIMER, TASK_APP, 10000);
            dbg_puts(DBG_APP_LVL, "Audio input activated\r\n");
        }
        else {
            USER_LEDS_OFF(led_audio_test_mode_mic_on_param);
            rwip_schedule();
            USER_LEDS_RAMP(led_audio_test_mode_mic_off_param);
            user_audio_stop(false);
            port_timer_set(USER_AUDIO_TEST_MODE_TIMER, TASK_APP, 10000);
            dbg_puts(DBG_APP_LVL, "Audio input deactivated\r\n");
        }        
        user_action_triggered();                
    }
}
#endif

/*
 ****************************************************************************************
 * Audio module callbacks
******************************************************************************************/ 
void user_audio_on_disconnect(void)
{
    user_audio_stop(true);   
    audio_control_packet_pending = false;
#ifdef AUDIO_TEST_MODE    
    user_rcu_audio_start_audio_test(false);
#endif    
}

bool user_audio_on_ble_powered(void)
{
    if(audio_control_packet_pending == true &&
       port_send_notification(user_audio_ctrl_in_hdl, audio_control_packet_data, audio_control_packet_length, false) == true) {
         audio_control_packet_pending = false;
         return true;
    }
    return false;
}

uint8_t user_audio_on_system_powered(void)
{
    static uint16_t audio_errors = 0;
    static uint8_t packet_count = 0;
    uint8_t *buffer;
    uint8_t ret = APP_GOTO_SLEEP;
    int i;
    
    if(user_audio_samples_available == true) {

    #if defined(HAS_AUDIO_STATS) && DEVELOPMENT_DEBUG  
        audio_stats.audio_buffer_used = port_audio_buffer_get_used();
        audio_stats.audio_buffer_used_max = co_max(audio_stats.audio_buffer_used_max, audio_stats.audio_buffer_used);
          ASSERT_ERROR(audio_stats.audio_buffer_used_max < 160);
    #endif    

        // Keep the system powered so that all audio samples are encoded
        ret = APP_KEEP_POWERED;

    #ifdef CFG_APP_STREAM_PACKET_BASED
        // Set the encoder output size equal to the packet size.
        // Each audio packet must be encoded at once.
        #ifdef CFG_APP_STREAM_FIFO_PREDEFINED            
        const uint16_t enc_size = APP_STREAM_PACKET_SIZE;
        #else
        const uint16_t enc_size = user_audio_packet_size;
        #endif
    #else
        const uint16_t enc_size = 18;
    #endif
    
        for (i = 0; i < 4; i++) { 
            // encode up to 4 packets per function call
    #ifdef AUDIO_CONTROL_ESCAPE_VALUE
            buffer = app_stream_fifo_get_write_dataptr(enc_size+3); // +3 to allow byte stuffing at the last three bytes
    #else
            buffer = app_stream_fifo_get_write_dataptr(enc_size);
    #endif
            /** If the app_stream_fifo buffer is full a NULL pointer is returned.
             ** This means that not all packets can be sent in time,
             ** e.g. due to lack of sufficient bandwidth.
             ** Start skipping samples.
             ** Call app_audio_encode with NULL parameter to drop a packet.
             */
            
            // If buffer is NULL then the audio data will be dropped.
            uint16_t num_of_encoded_bytes = app_audio_encode(buffer, enc_size);
            if (num_of_encoded_bytes != 0) {
                app_stream_fifo_commit_write_pkt(num_of_encoded_bytes, user_audio_data_hdl[user_audio_stream_report_nr]);
    #ifdef AUDIO_USE_THREE_HID_REPORTS
                user_audio_stream_report_nr++;
                if (user_audio_stream_report_nr > 2) {
                    user_audio_stream_report_nr = 0;
                }
    #endif
            } else {
    #if defined(HAS_AUDIO_STATS) && DEVELOPMENT_DEBUG
                if(GetBits16(CLK_RADIO_REG, BLE_ENABLE) != 0 && audio_stats.prev_timestamp != 0 && app_audio_is_active()) {
                    audio_stats.latency = ea_time_get_halfslot_rounded() - audio_stats.prev_timestamp;
                    audio_stats.latency_max = co_max(audio_stats.latency_max, audio_stats.latency); 
                }
    #endif    
                user_audio_samples_available = false;
                // There is no need to keep the system powered if all audio data have been encoded
                ret = APP_GOTO_SLEEP;
                break;
            }
        }
        
        if(app_stream_is_enabled()) {
            // If buffer errors have been detected, send out CTRL_IN notification with error values
            packet_count++;
            audio_errors += app_audio_get_errors();
            if (packet_count > 100 && audio_errors > 0) {
        #ifdef CFG_APP_STREAM_PACKET_BASED            
                buffer = app_stream_fifo_get_write_dataptr(user_audio_report_length);
        #else        
                uint8_t buffer[user_audio_report_length];
        #endif
                if(buffer != NULL) {
                    memset(buffer, 0, user_audio_report_length);
                    buffer[CTRL_IN_DEBUG_REPORT_TYPE_POS] = CTRL_IN_DEBUG_INFO_REPORT; // TYPE of message = debug info message.
                    buffer[CTRL_IN_DEBUG_REPORT_RESERVED2_POS] = 3; // Reserved value must be 3
                    port_write16(buffer+CTRL_IN_DEBUG_REPORT_AUDIO_DATA_POS, audio_errors);
        #ifdef CFG_APP_STREAM_PACKET_BASED            
                    app_stream_fifo_commit_debug_pkt(user_audio_ctrl_in_hdl, 
                                                     CTRL_IN_DEBUG_REPORT_STREAM_DATA_POS, 
                                                     user_audio_report_length);
        #else        
                    port_send_notification(user_audio_ctrl_in_hdl, 
                                           buffer, 
                                           user_audio_report_length, 
                                           false);
        #endif
                    ret = APP_KEEP_POWERED; // to send the notification
                    audio_errors = 0;
                    packet_count = 0;
                }
            }
        }
    }
    else {
        if(user_is_ble_connected() == true && stop_when_buffer_empty == true && app_stream_is_fifo_empty()) {
            user_audio_send_enable(AUDIO_STOP);
            ret = APP_KEEP_POWERED; // to send the notification
            stop_when_buffer_empty = false;
            app_stream_stop();
        }
    }

#if defined(AUDIO_CONTROL_ESCAPE_VALUE) && defined(CFG_AUDIO_ADAPTIVE_RATE)          
    if(user_is_ble_connected() == true) {  
        user_audio_adapt_rate();
    }
#endif            
    
    return ret;
}

    #ifdef AUDIO_USE_CUSTOM_PROFILE

/*
 ****************************************************************************************
 * Custom Audio profile functions
******************************************************************************************/  

enum process_event_response user_audio_process_custom_profile_handler(ke_msg_id_t const msgid, void const *param)
{
    switch(msgid) {
    case CUSTS1_VAL_WRITE_IND:
        {
            struct custs1_val_write_ind const *msg_param = (struct custs1_val_write_ind const *)param;

            switch (msg_param->handle) {
            case DLG_AUDIO_IDX_CTRL_VAL:
                user_audio_process_ctrl_packet(msg_param->value, msg_param->length);
                break;
            
            default:
                break;
        }
        break;
    }
    default:
        return PR_EVENT_UNHANDLED;
    }
    return PR_EVENT_HANDLED;
}

    #endif

void user_audio_set_config_data(void)
{
    #ifdef AUDIO_USE_CUSTOM_PROFILE    
        #if (RWBLE_SW_VERSION_MAJOR >= 8)  
            #define  CUSTS1_TASK_ID prf_get_task_from_id(TASK_ID_CUSTS1)
        #else
            #define  CUSTS1_TASK_ID TASK_CUSTS1
        #endif
    
    struct custs1_val_set_req *req = KE_MSG_ALLOC_DYN(CUSTS1_VAL_SET_REQ,
                                                      CUSTS1_TASK_ID,
                                                      TASK_APP,
                                                      custs1_val_set_req,
                                                      DLG_AUDIO_CONFIG_CHAR_LEN);
    
    req->handle = DLG_AUDIO_IDX_CONFIG_VAL;
    req->length = user_audio_create_configuration_packet(req->value);

    ASSERT_ERROR(req->length <= DLG_AUDIO_CONFIG_CHAR_LEN);

    ke_msg_send(req);
    
    #endif
}

/*
 ****************************************************************************************
 * Callback functions
******************************************************************************************/
    #ifndef AUDIO_USE_CUSTOM_PROFILE
/*****************************************************************************************
 * \brief Called when STREAM_CTRL_OUT HID report is written
 * param[in] conidx
 * param[in] report_info
******************************************************************************************/
void user_audio_write_ctrl_out_callback(uint8_t conidx, struct hogpd_report_info *report_info)
{
        #if (RWBLE_SW_VERSION_MAJOR >= 8)     
    uint8_t *report_data = report_info->value;
    uint16_t length = report_info->length;
        #else
    uint8_t *report_data = report_info->report;
    uint16_t length = report_info->report_length;
        #endif
    
    user_audio_process_ctrl_packet(report_data, length);
}

/*****************************************************************************************
 * \brief Called when STREAM_CTRL_IN HID report is read
 *
 * param[in] conidx
 * param[out] report_data
 * param[out] report_length
******************************************************************************************/
void user_audio_read_ctrl_in_callback(uint8_t conidx, uint8_t *report_data, uint16_t *report_length)
{
    *report_length = user_audio_create_configuration_packet(report_data);
}

    #endif

void user_audio_callback(void)
{ 
    if(user_audio_samples_available == false) {
    #if defined(HAS_AUDIO_STATS) && DEVELOPMENT_DEBUG
        audio_stats.prev_timestamp = GetBits16(CLK_RADIO_REG, BLE_ENABLE) != 0 ? ea_time_get_halfslot_rounded() : 0;
    #endif    
        user_audio_samples_available = true;
    }  
}

#if defined(HAS_AUDIO) && (defined(HAS_KBD) || defined(HAS_TOUCHPAD_SLIDER))
void user_audio_hid_send_report_cb(uint8_t report_idx, uint8_t *data, uint16_t length)
{
    uint8_t pkt_data[user_audio_report_length];
    
    switch (report_idx) {
    case HID_REPORT_NORMAL_REPORT_IDX:
    case HID_REPORT_EXTENDED_REPORT_IDX:
        memset(pkt_data, 0, user_audio_report_length);
        pkt_data[CTRL_IN_KEY_REPORT_TYPE_POS] = CTRL_IN_KEY_REPORT;
        pkt_data[CTRL_IN_KEY_REPORT_LENGTH_POS] = length;
        memcpy(&(pkt_data[CTRL_IN_KEY_REPORT_DATA_POS]), data, length);
        
        #ifdef HAS_KBD
        extern bool user_fn_locked;
        pkt_data[CTRL_IN_KEY_REPORT_KEYBOARD_PAGE_POS] = user_fn_locked ? 2 : 1;
        #endif
        
        port_send_notification(user_audio_ctrl_in_hdl, pkt_data, CTRL_IN_KEY_REPORT_SIZE, false);
        break;
    default:
        break;
    }
}
    #endif

#endif

/**
 * \}
 * \}
 * \}
 */
