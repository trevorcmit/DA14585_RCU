/**
****************************************************************************************
* \file user_config.h
* \brief User configuration file.
****************************************************************************************
*/

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

/**
 ****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup APP_CONFIG
 * \{
 * \addtogroup USER_CFG
 *
 * \brief User configuration
 * \{
 ****************************************************************************************
 */
 
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app_user_config.h"
#include "arch_api.h"
#include "app_default_handlers.h"
#include "app_adv_data.h"
#include "port_ble_gap.h"

/*
 * VARIABLES
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Default sleep mode. Possible values are:
 *  - ARCH_SLEEP_OFF
 *  - ARCH_EXT_SLEEP_ON
 *  - ARCH_EXT_SLEEP_OTP_COPY_ON
 ****************************************************************************************
 */
const static sleep_state_t app_default_sleep_mode = ARCH_EXT_SLEEP_ON;

/**
 ****************************************************************************************
 * Advertising configuration
 ****************************************************************************************
 */
static const struct advertise_configuration user_adv_conf = {
    /**
     * Own BD address source of the device:
     * - GAPM_STATIC_ADDR: Public or Private Static Address according to device address configuration
     * - GAPM_GEN_RSLV_ADDR: Generated resolvable private random address
     * - GAPM_GEN_NON_RSLV_ADDR: Generated non-resolvable private random address
     */
    .addr_src = GAPM_STATIC_ADDR,
    /// Minimum interval for advertising
    .intv_min = MS_TO_BLESLOTS(31.25),                   

    /// Maximum interval for advertising
    .intv_max = MS_TO_BLESLOTS(62.5),                  

    /**
     *  Advertising channels map:
     * - ADV_CHNL_37_EN:   Advertising channel map for channel 37.
     * - ADV_CHNL_38_EN:   Advertising channel map for channel 38.
     * - ADV_CHNL_39_EN:   Advertising channel map for channel 39.
     * - ADV_ALL_CHNLS_EN: Advertising channel map for channel 37, 38 and 39.
     */
    .channel_map = ADV_ALL_CHNLS_EN,

    /*************************
     * Advertising information
     *************************
     */

    /// Host information advertising data (GAPM_ADV_NON_CONN and GAPM_ADV_UNDIRECT)
    /// Advertising mode :
    /// - GAP_NON_DISCOVERABLE: Non discoverable mode
    /// - GAP_GEN_DISCOVERABLE: General discoverable mode
    /// - GAP_LIM_DISCOVERABLE: Limited discoverable mode
    /// - GAP_BROADCASTER_MODE: Broadcaster mode
    .mode = GAP_GEN_DISCOVERABLE,

    /// Host information advertising data (GAPM_ADV_NON_CONN and GAPM_ADV_UNDIRECT)
    /// Advertising filter policy:
    /// - ADV_ALLOW_SCAN_ANY_CON_ANY: Allow both scan and connection requests from anyone
    /// - ADV_ALLOW_SCAN_WLST_CON_ANY: Allow both scan req from White List devices only and
    ///   connection req from anyone
    /// - ADV_ALLOW_SCAN_ANY_CON_WLST: Allow both scan req from anyone and connection req
    ///   from White List devices only
    /// - ADV_ALLOW_SCAN_WLST_CON_WLST: Allow scan and connection requests from White List
    ///   devices only
    .adv_filt_policy = ADV_ALLOW_SCAN_ANY_CON_ANY,

    /// Direct address information (GAPM_ADV_DIRECT/GAPM_ADV_DIRECT_LDC)
    /// (used only if reconnection address isn't set or privacy disabled)
    /// BD Address of device
    .peer_addr = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6},

    /// Direct address information (GAPM_ADV_DIRECT/GAPM_ADV_DIRECT_LDC)
    /// (used only if reconnection address isn't set or privacy disabled)
    /// Address type of the device 0=public/1=private random
    .peer_addr_type = 0,
};

/**
 ****************************************************************************************
 *
 * Advertising or scan response data for the following cases:
 *
 * - ADV_IND: Connectable undirected advertising event.
 *    - The maximum length of the user defined advertising data shall be 28 bytes.
 *    - The Flags data type are written by the related ROM function, hence the user shall
 *      not include them in the advertising data. The related ROM function adds 3 bytes in 
 *      the start of the advertising data that are to be transmitted over the air.
 *    - The maximum length of the user defined response data shall be 31 bytes.
 *
 * - ADV_NONCONN_IND: Non-connectable undirected advertising event.
 *    - The maximum length of the user defined advertising data shall be 31 bytes.
 *    - The Flags data type may be omitted, hence the user can use all the 31 bytes for 
 *      data.
 *    - The scan response data shall be empty.
 *
 * - ADV_SCAN_IND: Scannable undirected advertising event.
 *    - The maximum length of the user defined advertising data shall be 31 bytes.
 *    - The Flags data type may be omitted, hence the user can use all the 31 bytes for 
 *      data.
 *    - The maximum length of the user defined response data shal be 31 bytes.
 ****************************************************************************************
 */
/// Advertising service data
/// Advertising AD type flags, shall not be set in advertising data
#if defined(CFG_PRF_SUOTAR)
    #define USER_ADVERTISE_DATA \
            "\x03"\
            ADV_TYPE_APPEARANCE\
            ADV_APPEARANCE_GENERIC_REMOTE_CONTROL\
            "\x09"\
            ADV_TYPE_INCOMPLETE_LIST_16BIT_SERVICE_IDS\
            ADV_UUID_HUMAN_INTERFACE_DEVICE_SERVICE\
            ADV_UUID_BATTERY_SERVICE\
            ADV_UUID_DEVICE_INFORMATION_SERVICE\
            ADV_UUID_SUOTAR_SERVICE\
            "\x05"\
            ADV_TYPE_SERVICE_DATA_16BIT_UUID\
            ADV_UUID_DEVICE_INFORMATION_SERVICE\
            "\x00\x01"
#else
    #define USER_ADVERTISE_DATA \
            "\x03"\
            ADV_TYPE_APPEARANCE\
            ADV_APPEARANCE_GENERIC_REMOTE_CONTROL\
            "\x07"\
            ADV_TYPE_INCOMPLETE_LIST_16BIT_SERVICE_IDS\
            ADV_UUID_HUMAN_INTERFACE_DEVICE_SERVICE\
            ADV_UUID_BATTERY_SERVICE\
            ADV_UUID_DEVICE_INFORMATION_SERVICE\
            "\x05"\
            ADV_TYPE_SERVICE_DATA_16BIT_UUID\
            ADV_UUID_DEVICE_INFORMATION_SERVICE\
            "\x00\x01"
#endif

/// Advertising data length - maximum 28 bytes, 3 bytes are reserved to set
#define USER_ADVERTISE_DATA_LEN (sizeof(USER_ADVERTISE_DATA)-1)

/// Scan response data
#define USER_ADVERTISE_SCAN_RESPONSE_DATA ""

/// Scan response data length- maximum 31 bytes
#define USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN (sizeof(USER_ADVERTISE_SCAN_RESPONSE_DATA)-1)

/**
 ****************************************************************************************
 *
 * Device name.
 *
 * - If there is space left in the advertising or scan response data the device name is
 *   copied there. The device name can be anytime read by a connected peer, if the
 *   application supports it.
 * - The Bluetooth device name can be up to 248 bytes.
 *
 ****************************************************************************************
 */
#define USER_DEVICE_NAME    "DA14585 RCU"

/// Device name length
#define USER_DEVICE_NAME_LEN (sizeof(USER_DEVICE_NAME)-1)

/// Maximum MTU size
#define MAX_MTU_SIZE 133

/**
 ****************************************************************************************
 *
 * GAPM configuration
 *
 ****************************************************************************************
 */
static const struct gapm_configuration user_gapm_conf = {
    /// Device Role: Central, Peripheral, Observer, Broadcaster or All roles. (@see enum gap_role)
    .role = GAP_ROLE_PERIPHERAL,
    /// Maximal MTU
    .max_mtu = MAX_MTU_SIZE,
    /// Device Address Type
    /// - GAPM_CFG_ADDR_PUBLIC: Device Address is a Public Static address
    /// - GAPM_CFG_ADDR_PRIVATE: Device Address is a Private Static address
    /// - GAPM_CFG_ADDR_PRIVACY: Device Address generated using Privacy feature
    .addr_type = GAPM_CFG_ADDR_PUBLIC,

    /***********************
     * Privacy configuration
     ***********************
     */

    /// Duration before regenerate device address when privacy is enabled.
    .renew_dur = 0,
    /// Device IRK used for resolvable random BD address generation (LSB first)
    .irk = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // privacy 1.2 - Link Layer Privacy (4.2)
    .priv1_2 = 0,

    /****************************
     * ATT database configuration
     ****************************
     */

    /// Attribute database configuration (@see enum gapm_att_cfg_flag)
    ///    7     6    5     4     3    2    1    0
    /// +-----+-----+----+-----+-----+----+----+----+
    /// | DBG | RFU | SC | PCP | APP_PERM |NAME_PERM|
    /// +-----+-----+----+-----+-----+----+----+----+
    /// - Bit [0-1]: Device Name write permission requirements for peer device (@see gapm_write_att_perm)
    /// - Bit [2-3]: Device Appearance write permission requirements for peer device (@see gapm_write_att_perm)
    /// - Bit [4]  : Slave Preferred Connection Parameters present
    /// - Bit [5]  : Service change feature present in GATT attribute database.
    /// - Bit [6]  : Reserved
    /// - Bit [7]  : Enable Debug Mode
    .att_cfg = GAPM_MASK_ATT_SVC_CHG_EN,

    /// GAP service start handle
    .gap_start_hdl = 0,

    /// GATT service start handle
    .gatt_start_hdl = 0,

    /**************************************************
     * Data packet lenght extension configuration (4.2)
     **************************************************
     */

    /// Maximal MPS
    .max_mps = 0,

    /// Maximal Tx octets
    .max_txoctets = CFG_MAX_TX_PACKET_LENGTH,

    /// Maximal Tx time
    .max_txtime = (CFG_MAX_TX_PACKET_LENGTH+11+3)*8,
};

/**
 ****************************************************************************************
 *
 * Parameter update configuration
 *
 ****************************************************************************************
 */
/// Connection interval minimum measured in ble double slots (1.25ms)
/// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots

/// Connection interval maximum measured in ble double slots (1.25ms)
/// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
#define PARAM_UPDATE_INV_MIN (MS_TO_DOUBLESLOTS(11.25))
#define PARAM_UPDATE_INV_MAX (MS_TO_DOUBLESLOTS(15))

/// Latency measured in connection events
#define PARAM_UPDATE_LATENCY 31

/// Supervision timeout measured in timer units (10 ms)
/// use the macro MS_TO_TIMERUNITS to convert from milliseconds (ms) to timer units
#define PARAM_UPDATE_SUP_TIMEOUT (MS_TO_TIMERUNITS(2000))

static const struct connection_param_configuration user_connection_param_conf = {
    /// Connection interval minimum measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots

    /// Connection interval maximum measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
    .intv_min = PARAM_UPDATE_INV_MIN,
    .intv_max = PARAM_UPDATE_INV_MAX,

    /// Latency measured in connection events
    .latency = PARAM_UPDATE_LATENCY,

    /// Supervision timeout measured in timer units (10 ms)
    /// use the macro MS_TO_TIMERUNITS to convert from milliseconds (ms) to timer units
    .time_out = PARAM_UPDATE_SUP_TIMEOUT,
    /// Minimum Connection Event Duration measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
    .ce_len_min = MS_TO_DOUBLESLOTS(0),

    /// Maximum Connection Event Duration measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
    .ce_len_max = MS_TO_DOUBLESLOTS(0),
};

/**
 ****************************************************************************************
 *
 * Default handlers configuration
 *
 ****************************************************************************************
 */
static const struct default_handlers_configuration  user_default_hnd_conf = {
    /// Configure the advertise operation used by the default handlers
    /// Possible values:
    ///  - DEF_ADV_FOREVER
    ///  - DEF_ADV_WITH_TIMEOUT
    .adv_scenario = DEF_ADV_FOREVER,

    /// Configure the advertise period in case of DEF_ADV_WITH_TIMEOUT.
    /// It is measured in timer units (10ms). Use MS_TO_TIMERUNITS macro to convert
    /// from milliseconds (ms) to timer units.
    .advertise_period = MS_TO_TIMERUNITS(10000),

    /// Configure the security start operation of the default handlers
    /// if the security is enabled (CFG_APP_SECURITY)
    .security_request_scenario = DEF_SEC_REQ_NEVER
};

/**
 ****************************************************************************************
 *
 * Central configuration (not used by current example)
 *
 ****************************************************************************************
 */
static const struct central_configuration user_central_conf = {0};

/**
 ****************************************************************************************
 *
 * Security related configuration
 *
 * Default configuration for no MITM.
 * When MITM is used, the values will be overridden in user_on_pairing_request()
 *
 ****************************************************************************************
 */
static const struct security_configuration user_security_conf = {
    /**************************************************************************************
     * IO capabilities (@see gap_io_cap)
     *
     * - GAP_IO_CAP_DISPLAY_ONLY          Display Only
     * - GAP_IO_CAP_DISPLAY_YES_NO        Display Yes No
     * - GAP_IO_CAP_KB_ONLY               Keyboard Only
     * - GAP_IO_CAP_NO_INPUT_NO_OUTPUT    No Input No Output
     * - GAP_IO_CAP_KB_DISPLAY            Keyboard Display
     *
     **************************************************************************************
     */
    .iocap          = GAP_IO_CAP_NO_INPUT_NO_OUTPUT,

    /**************************************************************************************
     * OOB information (@see gap_oob)
     *
     * - GAP_OOB_AUTH_DATA_NOT_PRESENT    OOB Data not present
     * - GAP_OOB_AUTH_DATA_PRESENT        OOB data present
     *
     **************************************************************************************
     */
    .oob            = GAP_OOB_AUTH_DATA_NOT_PRESENT,

    /**************************************************************************************
     * Authentication (@see gap_auth)
     *
     * - GAP_AUTH_REQ_NO_MITM_NO_BOND     No MITM No Bonding
     * - GAP_AUTH_REQ_NO_MITM_BOND        No MITM Bonding
     * - GAP_AUTH_REQ_MITM_NO_BOND        MITM No Bonding
     * - GAP_AUTH_REQ_MITM_BOND           MITM and Bonding
     *
     **************************************************************************************
     */
    .auth           = GAP_AUTH_REQ_NO_MITM_BOND,

    /**************************************************************************************
     * Device security requirements (minimum security level). (@see gap_sec_req)
     *
     * - GAP_NO_SEC                       No security (no authentication and encryption)
     * - GAP_SEC1_NOAUTH_PAIR_ENC         Unauthenticated pairing with encryption
     * - GAP_SEC1_AUTH_PAIR_ENC           Authenticated pairing with encryption
     * - GAP_SEC2_NOAUTH_DATA_SGN         Unauthenticated pairing with data signing
     * - GAP_SEC2_AUTH_DATA_SGN           Authentication pairing with data signing
     * - GAP_SEC_UNDEFINED                Unrecognised security
     *
     **************************************************************************************
     */
    .sec_req        = GAP_SEC1_NOAUTH_PAIR_ENC,

     /// Encryption key size (7 to 16) - LTK Key Size
    .key_size       = KEY_LEN,

    /**************************************************************************************
     * Initiator key distribution (@see gap_kdist)
     *
     * - GAP_KDIST_NONE                   No Keys to distribute
     * - GAP_KDIST_ENCKEY                 LTK (Encryption key) in distribution
     * - GAP_KDIST_IDKEY                  IRK (ID key)in distribution
     * - GAP_KDIST_SIGNKEY                CSRK (Signature key) in distribution
     * - Any combination of the above
     *
     **************************************************************************************
     */
    .ikey_dist      = GAP_KDIST_NONE,

    /**************************************************************************************
     * Responder key distribution (@see gap_kdist)
     *
     * - GAP_KDIST_NONE                   No Keys to distribute
     * - GAP_KDIST_ENCKEY                 LTK (Encryption key) in distribution
     * - GAP_KDIST_IDKEY                  IRK (ID key)in distribution
     * - GAP_KDIST_SIGNKEY                CSRK (Signature key) in distribution
     * - Any combination of the above
     *
     **************************************************************************************
     */
    .rkey_dist      = GAP_KDIST_ENCKEY,
};

/**
 ******************************************************************************
 * \brief Set the pattery polling period in msec
 ******************************************************************************
 */
#define BATTERY_LEVEL_POLLING_PERIOD 10000

/**
 ******************************************************************************
 * \brief Define USE_ONE_BAS_INSTANCE to unse only one BASS instance
 ******************************************************************************
 */
#define USE_ONE_BAS_INSTANCE

/*
 ******************************************************************************
 *
 * Module configuration
 *
 ******************************************************************************
 */

/**
 ******************************************************************************
 * \brief Define HAS_KBD to use the keyboard matrix scanner
 ******************************************************************************
 */
#define HAS_KBD
    
/*
 ******************************************************************************
 * \brief Define HAS_GPIO_KEYS to use keys connected to GPIO pins
 ******************************************************************************
 */
#undef HAS_GPIO_KEYS
    
/**
 ******************************************************************************
 * \brief Define CFG_APP_AUDIO to use the audio features
 * Audio configuration is defined in app_audio_config.h
 ******************************************************************************
 */
#define HAS_AUDIO

#ifdef HAS_AUDIO
    /**
     **************************************************************************
     * \brief When AUDIO_USE_CUSTOM_PROFILE is defined the custom audio profile
     * is used for controlling audio and streaming audio data to the host. When 
     * not defined vendor defined HID reports are used for audio.
     **************************************************************************
     */
    #define AUDIO_USE_CUSTOM_PROFILE

    /**
     **************************************************************************
     * \brief When vendor defined HID reports are used for audio, if 
     * AUDIO_USE_THREE_HID_REPORTS is defined then three HID reports will be 
     * used for sending audio data to the host.
     **************************************************************************
     */
    #undef AUDIO_USE_THREE_HID_REPORTS

    /**
     **************************************************************************
     * \brief When AUDIO_WAIT_FOR_HOST_START_CMD is define audio acquisition 
     * starts when the corresponding command is received from the host. When 
     * not defined audio starts as soon as the corresponding button is pressed.
     **************************************************************************
     */
    #define AUDIO_WAIT_FOR_HOST_START_CMD

#endif

/**
 ******************************************************************************
 * \brief Define HAS_MOTION to use the motion sensor
 * Motion configuration is defined in app_motion_config.h
 ******************************************************************************
 */
#undef HAS_MOTION

/**
 ******************************************************************************
 * \brief Define HAS_IR to use the IR transmitter
 * IR transmitter output port and pin must be defined as well
 ******************************************************************************
 */
#undef HAS_IR

#ifdef HAS_IR
    static const uint16_t kbd_ir_keymap[][] =
    {
      /* 
          0        1        2        3
          ---------------------------------
          1        2        3        unused
          4        5        6        unused
          7        8        9        vol+
          unused   0        unused   vol-
          ----------------------------------*/
              
        { 0x0001,  0x0002,  0x0003,  0x0000},  // 0
        { 0x0004,  0x0005,  0x0006,  0x0000},  // 1
        { 0x0007,  0x0008,  0x0009,  0x0010},  // 2
        { 0x0000,  0x0000,  0x0000,  0x0011},  // 3
    };
#endif

/**
 ******************************************************************************
 * \brief Define HAS_TOUCHPAD_TRACKPAD to use the trackpad module
 ******************************************************************************
 */
#undef HAS_TOUCHPAD_TRACKPAD

/**
 ******************************************************************************
 * \brief Define HAS_TOUCHPAD_SLIDER to use the slider module
 ******************************************************************************
 */
#undef HAS_TOUCHPAD_SLIDER

/**
 ******************************************************************************
 * \brief Define HAS_LED_INDICATORS in order to use LED indicators
 ******************************************************************************
 */
#undef HAS_LED_INDICATORS

/**
 ******************************************************************************
 * \brief Define HAS_SOUND_INDICATION to use the buzzer as as sound indicator
 * Buzzer related port and pin must be defined as well
 ******************************************************************************
 */
#undef HAS_SOUND_INDICATION

/**
 ******************************************************************************
 * \brief Define HAS_MOUSE to use the mouse sensor
 * Mouse sensor related port and pins must be defined as well
 ******************************************************************************
 */
#undef HAS_MOUSE


#ifdef HAS_AUDIO
    /**
 ******************************************************************************
     * \brief Define HAS_BLE_STREAM to enable the BLE stream
 * Stream configuration is defined in app_stream_config.h
 ******************************************************************************
 */
    #define HAS_BLE_STREAM
#endif


#if defined(HAS_KBD) || defined(HAS_TOUCHPAD_SLIDER)
    /**
 ******************************************************************************
     * \brief Define HAS_HID_REPORT to enable the HID report FIFO. Normal and 
     *        extended keyboard reports can be added to the report FIFO
     ******************************************************************************
     */
    #define HAS_HID_REPORT
#endif

/**
 ******************************************************************************
 * \brief Use connection FSM for pairing/bonding to one or multiple hosts
 ******************************************************************************
 */
#define HAS_CONNECTION_FSM

/**
 ******************************************************************************
 * \brief Define HAS_ACTION_INACTIVITY_TIMEOUT to have systick hit when
 * ACTION_INACTIVITY_TIMEOUT have passed after the last user action or
 * wakeup interrupt
 ******************************************************************************
 */
#undef HAS_ACTION_INACTIVITY_TIMEOUT

#ifdef HAS_ACTION_INACTIVITY_TIMEOUT
    /**
     ******************************************************************************
     * \brief Set the timeout in usec when HAS_ACTION_INACTIVITY_TIMEOUT is used
     ******************************************************************************
     */
     #define ACTION_INACTIVITY_TIMEOUT_IN_US 100000 // in usec
#endif

/**
 ******************************************************************************
 * \brief Define HAS_PWR_MGR to enable inactivity timeout
 ******************************************************************************
 */
#define HAS_PWR_MGR

#if defined(HAS_KBD) || defined(HAS_TOUCHPAD_TRACKPAD) || defined(HAS_TOUCHPAD_SLIDER)\
    || defined(HAS_GPIO_KEYS)
    /**
 ******************************************************************************
     * \brief Define HAS_WKUP to enable wakeup controller sharing between various 
     *        modules
 ******************************************************************************
 */
    #define HAS_WKUP
#endif    

#if defined(HAS_LED_INDICATORS) || defined(HAS_MOTION) || \
    defined(HAS_CONNECTION_FSM) || defined(HAS_PWR_MGR) || \
    defined(HAS_KBD) || BLE_SPOTA_RECEIVER || BLE_SUOTA_RECEIVER
    /**
     ******************************************************************************
     * \brief Define HAS_TIMERS to enable timer handing module
     ******************************************************************************
     */
    #define HAS_TIMERS
#endif

#if defined(HAS_GPIO_KEYS) || defined(HAS_ACTION_INACTIVITY_TIMEOUT)
    /**
     ******************************************************************************
     * \brief Define HAS_SYSTICK to enable systick sharing between various modules
     ******************************************************************************
     */
    #define HAS_SYSTICK
#endif    

/**
 ******************************************************************************
 * \brief Define NORMALLY_CONNECTABLE to force the device to always advertise 
 * when not connected to a BLE host
 ******************************************************************************
 */
#undef NORMALLY_CONNECTABLE

/**
 ******************************************************************************
 * \brief Define HAS_POWERUP_BUTTON to allow a keyboard button to turn the 
 * system on or off
 ******************************************************************************
 */
#undef HAS_POWERUP_BUTTON
                          
#ifdef HAS_POWERUP_BUTTON
    /**
     **************************************************************************
     * Define HAS_LONG_PRESS_POWERUP_BUTTON to allow detection of the power
     * button being pressed for more than LONG_KEYPRESS_QUOTA_IN_MS
     **************************************************************************
     */
    #undef HAS_LONG_PRESS_POWERUP_BUTTON
    
    #ifdef HAS_LONG_PRESS_POWERUP_BUTTON
        #define LONG_KEYPRESS_QUOTA_IN_MS 3000
    #endif
#endif    

/**
 ******************************************************************************
 * \brief Define HAS_SPI_FLASH_STORAGE if SPI flash is used for storing 
 * parameters SPI Flash configuration is defined in app_flash_config.h
 ******************************************************************************
 */
#define HAS_SPI_FLASH_STORAGE

/**
 ******************************************************************************
 * \brief Define HAS_I2C_EEPROM_STORAGE if I2C EEPROM is used for storing 
 * parameters I2C pins must be defined as well.
 ******************************************************************************
 */
#undef HAS_I2C_EEPROM_STORAGE    

/*
 ******************************************************************************
 * SPI Flash configuration
 ******************************************************************************
 */
// 0x00000..0x02FFF reserved for secondary bootloader
// 0x03000..0x1AFFF reserved for 1st SUOTA image
// 0x1B000..0x32FFF reserved for 2nd SUOTA image
// SPI Flash page at 0x38000 is reserved for product header    
#define SPI_FLASH_BONDING_INFO_BASE_ADDR  0x39000   // a whole sector must be assigned for data storage

#define NV_DEBUG_BASE_ADDR                0x3A000   // a whole sector must be assigned for debug info

/*
 ******************************************************************************
 * I2C EEPROM configuration
 ******************************************************************************
 */
#define I2C_SLAVE_ADDRESS       0x50            // Set slave device address
#define I2C_ADDRESS_MODE        I2C_7BIT_ADDR   // 7-bit addressing
#define I2C_EEPROM_SIZE         8192            // EEPROM size in bytes
#define I2C_EEPROM_PAGE         32              // EEPROM's page size in bytes
#define I2C_SPEED_MODE          I2C_FAST        // fast mode (400 kbits/s)
#define I2C_ADRESS_BYTES_CNT    I2C_2BYTES_ADDR
#define I2C_EEPROM_BONDING_INFO_BASE_ADDR    0

/**
 ******************************************************************************
 * \brief Define HAS_DEEPSLEEP to allow device to go into deep sleep when idle
 ******************************************************************************
 */
#undef HAS_DEEPSLEEP

/**
 ******************************************************************************
 * \brief Set MAX_NUM_OF_PACKETS_PER_CONNECTION to the maximum number of 
 * packets that can be sent in one connection interval. This number is used 
 * for bandwidth calculation.
 ******************************************************************************
 */
#define MAX_NUM_OF_PACKETS_PER_CONNECTION 4

/*
 ******************************************************************************
 * Debugging options
 ******************************************************************************
 */
 
/**
 ******************************************************************************
 * \brief Define debug level
 * Choose between DBG_NONE, DBG_ALL or any combination of  DBG_APP_LVL, 
 * DBG_CONN_LVL, DBG_ADV_LVL, DBG_KBD_LVL, DBG_HID_LVL, DBG_PWR_MGR_LVL,
 * DBG_MOT_LVL
 ******************************************************************************
 */
#define DEBUG_LEVEL (DBG_APP_LVL | DBG_CONN_LVL | DBG_ADV_LVL | DBG_HID_LVL | DBG_MOT_LVL)

/**
 ******************************************************************************
 * \brief Define MEASURE_CPU_LOAD to calculate CPU load while BLE is 
 * active. CPU load percentage is stored in global variable cpu_load
 ******************************************************************************
 */
#undef MEASURE_CPU_LOAD

/**
 ******************************************************************************
 * \brief Define INDICATE_IDLE_STATE to set a GPIO high while CPU is in 
 *        WFI state
 ******************************************************************************
 */
#undef INDICATE_IDLE_STATE

#ifdef INDICATE_IDLE_STATE      
    #define IDLE_INDICATION_PORT GPIO_PORT_0
    #define IDLE_INDICATION_PIN  GPIO_PIN_7
#endif

/**
 ******************************************************************************
 * \brief Define HAS_AUDIO_STATS to log audio and stream buffer statistics
 ******************************************************************************
 */
#ifdef HAS_AUDIO
    #define HAS_AUDIO_STATS
#endif    

/**
 ******************************************************************************
 * \brief Define DISABLE_UART_RX to disable UART Rx pin
 ******************************************************************************
 */
#define DISABLE_UART_RX

/**
 ******************************************************************************
 * \brief Define DEBUG_EMULATE_PACKET_LOSS to emulate BLE packet loss by 
 * turning the RF radio off. Functions user_packet_loss_inc() and 
 * user_packet_loss_dec() can be used to control the packet loss according 
 * to the pattern defined in global variable radio_mute_pattern.
 ******************************************************************************
 */
#ifdef HAS_KBD
    #undef DEBUG_EMULATE_PACKET_LOSS
#endif

/**
 ******************************************************************************
 * \brief Define MOUSE_GENERATE_TEST_PATTERN to emulate the circular movement 
 * of a mouse cursor.
 ******************************************************************************
 */
#undef MOUSE_GENERATE_TEST_PATTERN

/**
 ******************************************************************************
 * \brief Define BLE_THROUGHPUT_METRICS to measure the available BLE bandwidth
 ******************************************************************************
 */
#define BLE_THROUGHPUT_METRICS

/**
 * \}
 * \}
 * \}
 */

#endif // _USER_CONFIG_H_
