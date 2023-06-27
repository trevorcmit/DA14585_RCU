/*****************************************************************************************
 *
 * @file user_config.h
 *
 * @brief User configuration file.
 *
******************************************************************************************/

#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

/*
 * INCLUDE FILES
******************************************************************************************/

#include "app_user_config.h"
#include "arch_api.h"
#include "app_default_handlers.h"
#include "app_adv_data.h"
#include "co_bt.h"

/*
 * VARIABLES
******************************************************************************************/

/******************************************
 * Default sleep mode. Possible values are:
 *
 * - ARCH_SLEEP_OFF
 * - ARCH_EXT_SLEEP_ON
 * - ARCH_EXT_SLEEP_OTP_COPY_ON
 *
 ******************************************
 */
const static sleep_state_t app_default_sleep_mode = ARCH_EXT_SLEEP_OTP_COPY_ON;

/*
 ****************************************************************************************
 *
 * Advertising configuration
 *
******************************************************************************************/
static const struct advertise_configuration user_adv_conf = {
    /**
     * Own BD address source of the device:
     * - GAPM_STATIC_ADDR: Public or Private Static Address according to device address configuration
     * - GAPM_GEN_RSLV_ADDR: Generated resolvable private random address
     * - GAPM_GEN_NON_RSLV_ADDR: Generated non-resolvable private random address
     */
    .addr_src = GAPM_STATIC_ADDR,

    /// Minimum interval for advertising
    .intv_min = MS_TO_BLESLOTS(687.5),                    // 687.5ms

    /// Maximum interval for advertising
    .intv_max = MS_TO_BLESLOTS(687.5),                    // 687.5ms

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

/*
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
 *    - The maximum length of the user defined response data shall be 31 bytes.
******************************************************************************************/
/// Advertising data
#define USER_ADVERTISE_DATA         ("\x03"\
                                    ADV_TYPE_COMPLETE_LIST_16BIT_SERVICE_IDS\
                                    ADV_UUID_DEVICE_INFORMATION_SERVICE\
                                    "\x11"\
                                    ADV_TYPE_COMPLETE_LIST_128BIT_SERVICE_IDS\
                                    "\x2F\x2A\x93\xA6\xBD\xD8\x41\x52\xAC\x0B\x10\x99\x2E\xC6\xFE\xED")

/// Advertising data length - maximum 28 bytes, 3 bytes are reserved to set
#define USER_ADVERTISE_DATA_LEN               (sizeof(USER_ADVERTISE_DATA)-1)

/// Scan response data
#define USER_ADVERTISE_SCAN_RESPONSE_DATA ""

/// Scan response data length- maximum 31 bytes
#define USER_ADVERTISE_SCAN_RESPONSE_DATA_LEN (sizeof(USER_ADVERTISE_SCAN_RESPONSE_DATA)-1)

/*
 ****************************************************************************************
 *
 * Device name.
 *
 * - If there is space left in the advertising or scan response data the device name is
 *   copied there. The device name can be anytime read by a connected peer, if the
 *   application supports it.
 * - The Bluetooth device name can be up to 248 bytes.
 *
******************************************************************************************/
/// Device name
#define USER_DEVICE_NAME        "DLG-SLEEP"

/// Device name length
#define USER_DEVICE_NAME_LEN    (sizeof(USER_DEVICE_NAME)-1)

/*
 ****************************************************************************************
 *
 * GAPM configuration
 *
******************************************************************************************/
static const struct gapm_configuration user_gapm_conf = {
    /// Device Role: Central, Peripheral, Observer, Broadcaster or All roles. (@see enum gap_role)
    .role = GAP_ROLE_PERIPHERAL,

    /// Maximal MTU
    .max_mtu = 23,

    /// Device Address Type
    /// - GAPM_CFG_ADDR_PUBLIC: Device Address is a Public Static address
    /// - GAPM_CFG_ADDR_PRIVATE: Device Address is a Private Static address
    /// - GAPM_CFG_ADDR_PRIVACY: Device Address generated using Privacy feature
    .addr_type = GAPM_CFG_ADDR_PUBLIC,

    /***********************
     * Privacy configuration
     ***********************
     */

    /// Private static address
    // NOTE: The address shall comply with the following requirements:
    // - the two most significant bits of the address shall be equal to 1,
    // - all the remaining bits of the address shall NOT be equal to 1,
    // - all the remaining bits of the address shall NOT be equal to 0.
    // In case the {0x00, 0x00, 0x00, 0x00, 0x00, 0x00} null address is used, a
    // random static address will be automatically generated.
    .addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    /// Duration before regenerate device address when privacy is enabled.
    .renew_dur = 0,

    /// Device IRK used for resolvable random BD address generation (LSB first)
    .irk = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f},

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
     * Data packet length extension configuration (4.2)
     **************************************************
     */

    /// Maximal MPS
    .max_mps = 0,

    /// Maximal Tx octets (connInitialMaxTxOctets value, as defined in 4.2 Specification)
    .max_txoctets = 251,

    /// Maximal Tx time (connInitialMaxTxTime value, as defined in 4.2 Specification)
    .max_txtime = 2120,
};

/*
 ****************************************************************************************
 *
 * Parameter update configuration
 *
******************************************************************************************/
static const struct connection_param_configuration user_connection_param_conf = {
    /// Connection interval minimum measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
    .intv_min = MS_TO_DOUBLESLOTS(10),

    /// Connection interval maximum measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
    .intv_max = MS_TO_DOUBLESLOTS(20),

    /// Latency measured in connection events
    .latency = 0,

    /// Supervision timeout measured in timer units (10 ms)
    /// use the macro MS_TO_TIMERUNITS to convert from milliseconds (ms) to timer units
    .time_out = MS_TO_TIMERUNITS(1250),

    /// Minimum Connection Event Duration measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
    .ce_len_min = MS_TO_DOUBLESLOTS(0),

    /// Maximum Connection Event Duration measured in ble double slots (1.25ms)
    /// use the macro MS_TO_DOUBLESLOTS to convert from milliseconds (ms) to double slots
    .ce_len_max = MS_TO_DOUBLESLOTS(0),
};

/*
 ****************************************************************************************
 *
 * Default handlers configuration (applies only for @app_default_handlers.c)
 *
******************************************************************************************/
static const struct default_handlers_configuration  user_default_hnd_conf = {
    //Configure the advertise operation used by the default handlers
    //Possible values:
    //  - DEF_ADV_FOREVER
    //  - DEF_ADV_WITH_TIMEOUT
    .adv_scenario = DEF_ADV_FOREVER,

    //Configure the advertise period in case of DEF_ADV_WITH_TIMEOUT.
    //It is measured in timer units (3 min). Use MS_TO_TIMERUNITS macro to convert
    //from milliseconds (ms) to timer units.
    .advertise_period = MS_TO_TIMERUNITS(180000),

    //Configure the security start operation of the default handlers
    //if the security is enabled (CFG_APP_SECURITY)
    .security_request_scenario = DEF_SEC_REQ_NEVER
};

/*
 ****************************************************************************************
 *
 * Central configuration (not used by current example)
 *
******************************************************************************************/
static const struct central_configuration user_central_conf = {
    /// GAPM requested operation:
    /// - GAPM_CONNECTION_DIRECT: Direct connection operation
    /// - GAPM_CONNECTION_AUTO: Automatic connection operation
    /// - GAPM_CONNECTION_SELECTIVE: Selective connection operation
    /// - GAPM_CONNECTION_NAME_REQUEST: Name Request operation (requires to start a direct
    ///   connection)
    .code = GAPM_CONNECTION_DIRECT,

    /// Own BD address source of the device:
    ///  - GAPM_STATIC_ADDR: Public or Private Static Address according to device address configuration
    ///  - GAPM_GEN_RSLV_ADDR: Generated resolvable private random address
    ///  - GAPM_GEN_NON_RSLV_ADDR: Generated non-resolvable private random address
    .addr_src = GAPM_STATIC_ADDR,

    /// Scan interval
    .scan_interval = 0x180,

    /// Scan window size
    .scan_window = 0x160,

     /// Minimum of connection interval
    .con_intv_min = 100,

    /// Maximum of connection interval
    .con_intv_max = 100,

    /// Connection latency
    .con_latency = 0,

    /// Link supervision timeout
    .superv_to = 0x1F4,

     /// Minimum CE length
    .ce_len_min = 0,

    /// Maximum CE length
    .ce_len_max = 0x5,

    /**************************************************************************************
     * Peer device information (maximum number of peers = 8)
     **************************************************************************************
     */

    /// BD Address of device
    .peer_addr_0 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_0_type = 0,

    /// BD Address of device
    .peer_addr_1 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_1_type = 0,

    /// BD Address of device
    .peer_addr_2 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_2_type = 0,

    /// BD Address of device
    .peer_addr_3 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_3_type = 0,

    /// BD Address of device
    .peer_addr_4 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_4_type = 0,

    /// BD Address of device
    .peer_addr_5 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_5_type = 0,

    /// BD Address of device
    .peer_addr_6 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_6_type = 0,

    /// BD Address of device
    .peer_addr_7 = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0},

    /// Address type of the device 0=public/1=private random
    .peer_addr_7_type = 0,
};

/*
 ****************************************************************************************
 *
 * Security related configuration
 *
******************************************************************************************/
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
     * - GAP_SEC_UNDEFINED                Unrecognized security
     *
     **************************************************************************************
     */
    .sec_req        = GAP_NO_SEC,

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
    .ikey_dist      = GAP_KDIST_SIGNKEY,

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

#endif // _USER_CONFIG_H_
