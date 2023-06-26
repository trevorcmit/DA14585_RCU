/*****************************************************************************************
 *
 * @file dialog_commands.c
 *
 * @brief dialog vendor specific command handlers project source code.
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com> and contributors.
 *
******************************************************************************************/
 
 /*
 * INCLUDE FILES
******************************************************************************************/

#include "dialog_commands.h"
#include "hci_int.h"
#include "hci.h"
#include "co_hci.h"
#include "rwip.h"
#include "sdk_version.h"
#include "user_config_sw_ver.h"
#include "dialog_prod.h"
#include "rdtest_api.h"
#include "user_periph_setup.h" 
#include "spi.h"
#include "otp_hdr.h"
#include "system_library.h"
#include "em_map.h"
#include "h4tl.h"
#include "uart.h"

/*
 * DEFINES
******************************************************************************************/
 
// Value to generate 1s timeout when checking for a rising edge on the input pulse
#define PULSE_TIMEOUT    0x8CB6

// Maximum number of words that can be read or written by a command at once
#define MAX_READ_WRITE_OTP_WORDS 60

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/

volatile uint8_t test_data_pattern;
volatile uint8_t test_freq;

/*
 * LOCAL FUNCTIONS DECLARATIONS
******************************************************************************************/

extern void uart_finish_transfers_func(void);
static uint8_t hci_otp_rd_data_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_wr_otp_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len);
static uint8_t hci_pack_bytes(uint8_t** pp_in, uint8_t** pp_out, uint8_t* p_in_end, uint8_t* p_out_end, uint8_t len);

// HCI dialog command descriptors (OGF Vendor Specific)
const struct hci_cmd_desc_tag hci_cmd_desc_tab_dialog_vs[] =
{
    CMD(CUSTOM_ACTION            , DBG, 0, PK_GEN_GEN, "B"                    , "H"                            ),
    CMD(SLEEP_TEST               , DBG, 0, PK_GEN_GEN, "BBB"                  , NULL                           ),
    CMD(XTAL_TRIM                , DBG, 0, PK_GEN_GEN, "3B"                   , "H"                            ),
    CMD(OTP_RW                   , DBG, 0, PK_GEN_GEN, "B6B"                  , "B6B"                          ),
    CMD(OTP_READ                 , DBG, 0, PK_GEN_SPE, "HB"                   , &hci_otp_rd_data_cmd_cmp_evt_pk),
    CMD(OTP_WRITE                , DBG, 0, PK_SPE_GEN, &hci_wr_otp_cmd_upk    , "BB"                           ),
    CMD(REGISTER_RW              , DBG, 0, PK_GEN_GEN, "BLL"                  , "BBL"                          ),
    CMD(AUDIO_TEST               , DBG, 0, PK_GEN_GEN, NULL                   , NULL                           ),
    CMD(FIRMWARE_VERSION_GET     , DBG, 0, PK_GEN_GEN, NULL                   , "66B"                          ),
    CMD(CHANGE_UART_PINS_ACTION  , DBG, 0, PK_GEN_GEN, "4B"                   , "H"                            ),
    CMD(RDTESTER                 , DBG, 0, PK_GEN_GEN, "BH"                   , NULL                           ),
    CMD(TX_TEST                  , DBG, 0, PK_GEN_GEN, "BBBBB"                , NULL                           ),
    CMD(START_PROD_RX_TEST       , DBG, 0, PK_GEN_GEN, "B"                    , NULL                           ),
    CMD(END_PROD_RX_TEST         , DBG, 0, PK_GEN_GEN, NULL                   , "HHHH"                         ),
    CMD(UNMODULATED_ON           , DBG, 0, PK_GEN_GEN, "BB"                   , NULL                           ),
    CMD(TX_START_CONTINUE_TEST   , DBG, 0, PK_GEN_GEN, "BB"                   , NULL                           ),
    CMD(TX_END_CONTINUE_TEST     , DBG, 0, PK_GEN_GEN, NULL                   , NULL                           ),
    CMD(SENSOR_TEST              , DBG, 0, PK_GEN_GEN, "17B"                  , "H"                            ),
    CMD(GPIO_SET                 , DBG, 0, PK_GEN_GEN, "4B"                   , "B"                            ),
    CMD(GPIO_READ                , DBG, 0, PK_GEN_GEN, "B"                    , "B"                            ),
    CMD(UART_LOOP                , DBG, 0, PK_GEN_GEN, "100B"                 , "100B"                         ),
    CMD(UART_BAUD                , DBG, 0, PK_GEN_GEN, "B"                    , "B"                            ),
};

const uint8_t dialog_commands_num = ARRAY_LEN (hci_cmd_desc_tab_dialog_vs);

/*****************************************************************************************
 * @brief Handles the reception of the custom_action dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_custom_action_handler(ke_msg_id_t const msgid, 
                                                 struct hci_custom_action_dialog_cmd const *param,
                                                 ke_task_id_t const dest_id, 
                                                 ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_custom_action_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_CUSTOM_ACTION_CMD_OPCODE, hci_custom_action_dialog_cmd_cmp_evt);

    //adding return values 
    event->returnData = param->inputData & 0x00FF;
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the audio_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_audio_test_handler(ke_msg_id_t const msgid, 
                                              void const *param,
                                              ke_task_id_t const dest_id, 
                                              ke_task_id_t const src_id)
{
    // structure type for the complete command event
    void *event = ke_msg_alloc(HCI_CMD_CMP_EVENT , src_id, HCI_AUDIO_TEST_CMD_OPCODE, 0);


    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the firmware_version dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_firmware_version_get_handler(ke_msg_id_t const msgid, 
                                                        void const *param,
                                                        ke_task_id_t const dest_id, 
                                                        ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_firmware_version_get_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_FIRMWARE_VERSION_GET_CMD_OPCODE, hci_firmware_version_get_dialog_cmd_cmp_evt);

    //adding return values 
    event->ble_ver_len = sizeof(SDK_VERSION);
    strcpy(event->ble_ver, SDK_VERSION);
    event->app_ver_len = sizeof(DA14585_REFDES_SW_VERSION);
    strcpy(event->app_ver, DA14585_REFDES_SW_VERSION);
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the change_uart_pins_action dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_change_uart_pins_action_handler(ke_msg_id_t const msgid, 
                                                           struct hci_change_uart_pins_action_dialog_cmd const *param,
                                                           ke_task_id_t const dest_id, 
                                                           ke_task_id_t const src_id)
{
    // Structure type for the complete command event
    struct hci_change_uart_pins_action_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_CHANGE_UART_PINS_ACTION_CMD_OPCODE, hci_change_uart_pins_action_dialog_cmd_cmp_evt);
    uint8_t data[4];
    data[0] = param->tx_port;
    data[1] = param->tx_pin;
    data[2] = param->rx_port;
    data[3] = param->rx_pin;
    // Check if new pins are valid
    event->status = check_uart_pins_cmd(data);
    // Send completion event
    hci_send_2_host(event);
    
    // Change to new pins if they are valid
    if (!event->status)
    {
        change_uart_pins_cmd(data);
    }

    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the register_rw dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_register_rw_handler(ke_msg_id_t const msgid, 
                                               struct hci_register_rw_dialog_cmd const *param,
                                               ke_task_id_t const dest_id, 
                                               ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_register_rw_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_REGISTER_RW_CMD_OPCODE, hci_register_rw_dialog_cmd_cmp_evt);

    volatile void * reg_addr = 0;

    event->operation = param->operation;
    event->reserved = 0;

    reg_addr = (volatile void *)(param->addr);
    switch(param->operation)
    {
    case CMD__REGISTER_RW_OP_READ_REG32:
        event->data = GetWord32(reg_addr);
        break;        
    case CMD__REGISTER_RW_OP_WRITE_REG32:
        SetWord32(reg_addr, param->data );
        break;
    case CMD__REGISTER_RW_OP_READ_REG16:
        event->data = GetWord16(reg_addr);
        break;
    case CMD__REGISTER_RW_OP_WRITE_REG16:
        SetWord16(reg_addr, param->data );
        break;
    default:
        break;    
    }
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the tx_start_continue_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_tx_start_continue_test_handler(ke_msg_id_t const msgid, 
                                                          struct hci_tx_start_continue_test_dialog_cmd const *param,
                                                          ke_task_id_t const dest_id, 
                                                          ke_task_id_t const src_id)
{
    // structure type for the complete command event
    void *event = ke_msg_alloc(HCI_CMD_CMP_EVENT , src_id, HCI_TX_START_CONTINUE_TEST_CMD_OPCODE, 0);
    
    test_freq = param->frequency;
    test_data_pattern = param->payload_type;
    set_state_start_continue_tx();
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the otp_rw dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_otp_rw_handler(ke_msg_id_t const msgid, 
                                          struct hci_otp_rw_dialog_cmd const *param,
                                          ke_task_id_t const dest_id, 
                                          ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_otp_rw_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_OTP_RW_CMD_OPCODE, hci_otp_rw_dialog_cmd_cmp_evt);
    uint32_t word[2];
    
    memset(word, 0, sizeof(word));
    event->operation = param->operation;
    
    switch(param->operation)
    {
    case CMD__OTP_OP_RD_XTRIM:
        otp_read(OTP_HDR_TRIM_XTAL16M_ADDR, event->data , 2);
        break;        
    case CMD__OTP_OP_WR_XTRIM:
        word[0] = param->data[0] | param->data[1] << 8 ;
        otp_write_words(OTP_HDR_TRIM_XTAL16M_OFFSET, word , 1); 
        break;
    case CMD__OTP_OP_RD_BDADDR:
        otp_read(OTP_HDR_BDADDR_ADDR, event->data , 6);
        break;
    case CMD__OTP_OP_WR_BDADDR:
        word[0] = ((param->data[3] << 24) | (param->data[2] << 16) | (param->data[1] << 8) | (param->data[0]));
        word[1] = ( param->data[5] << 8 | param->data[4] );
        otp_write_words(OTP_HDR_BDADDR_OFFSET, word , 1);
        break;
    case CMD__OTP_OP_RE_XTRIM:
        //DO NOTHING IN 585 
        break;
    case CMD__OTP_OP_WE_XTRIM:
        //DO NOTHING IN 585 
        break;
    default:
        break;
    }
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the xtal_trim dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_xtal_trim_handler(ke_msg_id_t const msgid, 
                                             struct hci_xtal_trim_dialog_cmd const *param,
                                             ke_task_id_t const dest_id, 
                                             ke_task_id_t const src_id)
{
    // Structure type for the complete command event
    struct hci_xtal_trim_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_XTAL_TRIM_CMD_OPCODE, hci_xtal_trim_dialog_cmd_cmp_evt);
    uint16_t reg_value;
    uint16_t delta = param->trim_MSB << 8|param->trim_LSB;
    uint8_t xtal_cal_status = 0;
    int res = 0;
    int auto_trim(uint8_t port_number);
    int value;
    uint16_t gpio_prev_mode;
    uint32_t port_reg;
    
    switch(param->operation)
    {
    case CMD__XTRIM_OP_RD:
        event->trim_value = GetWord16(CLK_FREQ_TRIM_REG);
        break;        
    case CMD__XTRIM_OP_WR:
        SetWord16(CLK_FREQ_TRIM_REG, delta);
        event->trim_value = 0x00;
        break;
    case CMD__XTRIM_OP_EN:
        SetWord32(TEST_CTRL_REG, 1);
        SetWord32(P05_MODE_REG, OUTPUT | PID_GPIO);
        event->trim_value = 0x00;
        break;
    case CMD__XTRIM_OP_INC:
        reg_value = GetWord16(CLK_FREQ_TRIM_REG);   
        if (reg_value <= 0xFFFF - delta)
        {
            SetWord16(CLK_FREQ_TRIM_REG, reg_value + delta);
        }
        event->trim_value = 0x00;
        break;
    case CMD__XTRIM_OP_DEC:
        reg_value = GetWord16(CLK_FREQ_TRIM_REG);
        if (reg_value >= delta)
        {
            SetWord16(CLK_FREQ_TRIM_REG, reg_value - delta);
        }
        event->trim_value = 0x00;
        break;
    case CMD__XTRIM_OP_DIS:
        SetWord32(TEST_CTRL_REG, 0);
        SetWord32(P05_MODE_REG, INPUT_PULLDOWN | PID_GPIO);
        event->trim_value = 0x00;
        break;
    case CMD__XTRIM_OP_CALTEST:
    case CMD__XTRIM_OP_CAL:
        // Get the register of the port
        port_reg = find_mode_register(param->trim_LSB);
        if (port_reg)
        {
            // Hold previous mode for the pin that will be used for the pulse.
            gpio_prev_mode = GetWord16(port_reg);
            // Clear any previous role assigned to the pin and configured as GPIO with pull up.
            SetWord16(port_reg, INPUT_PULLUP | PID_GPIO);
            value = auto_trim(param->trim_LSB);
            // Pulse found in the pulse pin assigned GPIO was out of acceptable range
            if (value == -1)
            {
                xtal_cal_status = 0x01;
            }
            // No pulse found, or pulse > 740 ms
            else if (value == -2)
            {
                xtal_cal_status = 0x02;
            }
            else 
            {
                if (param->operation == CMD__XTRIM_OP_CAL)
                {
                    uint32_t word[2];
                    memset(word, 0, sizeof(word));
                    
                    // Write xtal trimming value in otp
                    word[0] = value & 0xFFFF;
                    res = otp_write_words(OTP_HDR_TRIM_XTAL16M_OFFSET, word, 1);
                    if (res != 0)
                    {
                        xtal_cal_status = 0x03; 
                    }
                }
            }
        }
        else
        {
            xtal_cal_status = 0x04;
        }
        event->trim_value = xtal_cal_status;
        break;
    default:
        break;
    }
    
    hci_send_2_host(event);
    
    // Check if command is CMD__XTRIM_OP_CALTEST or CMD__XTRIM_OP_CAL to restore UART_RX pin
    if ((param->operation == CMD__XTRIM_OP_CALTEST) || (param->operation == CMD__XTRIM_OP_CAL))
    {
        // If pulse was generated on the UART Rx Pin, wait until the end of the pulse, to avoid UART collision
        if (((gpio_prev_mode & PID) == PID_UART1_RX) || ((gpio_prev_mode & PID) == PID_UART2_RX))
        {
            // Timeout in 1sec
            uint16_t timeout = PULSE_TIMEOUT;
            // Wait until the pin gets high or timeout is reached
            while(!GPIO_GetPinStatus((GPIO_PORT)(param->trim_LSB/10), (GPIO_PIN)(param->trim_LSB%10)) && (timeout-- != 0) );
            // Return pin to its previous mode
            SetWord16(port_reg, gpio_prev_mode);
        }
    }
    
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the tx_start_continue_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_tx_end_continue_test_handler(ke_msg_id_t const msgid, 
                                                        void const *param,
                                                        ke_task_id_t const dest_id, 
                                                        ke_task_id_t const src_id)
{
    // structure type for the complete command event
    void *event = ke_msg_alloc(HCI_CMD_CMP_EVENT , src_id, HCI_TX_END_CONTINUE_TEST_CMD_OPCODE, 0);
    
    if (test_state == STATE_START_CONTINUE_TX)
    {
        set_state_stop();
        SetWord32(BLE_RFTESTCNTL_REG,0);
        hci_send_2_host(event);
    }
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the start_prod_rx_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_start_prod_rx_test_handler(ke_msg_id_t const msgid, 
                                                      struct hci_start_prod_rx_dialog_cmd const *param,
                                                      ke_task_id_t const dest_id, 
                                                      ke_task_id_t const src_id)
{
    // structure type for the complete command event
    void *event = ke_msg_alloc(HCI_CMD_CMP_EVENT , src_id, HCI_START_PROD_RX_TEST_CMD_OPCODE, 0);
    
    test_freq = param->frequency;
    set_state_start_rx();
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the end_prod_rx_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_end_prod_rx_test_handler(ke_msg_id_t const msgid, 
                                                    void const *param,
                                                    ke_task_id_t const dest_id, 
                                                    ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_end_prod_rx_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_END_PROD_RX_TEST_CMD_OPCODE, hci_end_prod_rx_dialog_cmd_cmp_evt);
    
    if (test_state == STATE_START_RX)
    {
        event->rp = metrics.rx_pkt - metrics.rx_err;
        event->rp_SYNC_error = metrics.rx_err_sync;
        event->rp_CRC_error = metrics.rx_err_crc;
        event->RSSI = metrics.rx_rssi;
        set_state_stop();
    
        hci_send_2_host(event);
    }
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the unmodulated_on dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_unmodulated_on_handler(ke_msg_id_t const msgid, 
                                                  struct hci_unmodulated_on_dialog_cmd const *param,
                                                  ke_task_id_t const dest_id, 
                                                  ke_task_id_t const src_id)
{
    // structure type for the complete command event
    void *event = ke_msg_alloc(HCI_CMD_CMP_EVENT , src_id, HCI_UNMODULATED_ON_CMD_OPCODE, 0);
    uint16_t cn;
    bool cmd_flag =false;
    
    switch(param->operation)
    {
    case UNMODULATED_CMD_MODE_OFF: //UNMODULATED OFF
        SetWord16(RF_BMCW_REG, 0);              //DISABLE OVERRULE CN_SEL IS 0
        SetWord16(RF_OVERRULE_REG, 0);          // SO DS IS NOT CORRECT, DISABLE DONE BY WRITING '0' TO ENABLE ISO '1' TO DISABLE 
        SetWord16(RF_SYNTH_CTRL3_REG, 0x0003);  //ENABLE MODULATION
        NVIC_EnableIRQ(BLE_RF_DIAG_IRQn);
        NVIC_ClearPendingIRQ(BLE_RF_DIAG_IRQn); //clear eventual pending bit, but not necessary becasuse this is already cleared automatically in HW
        cmd_flag=true;
        test_state = STATE_IDLE;
        break;
    case UNMODULATED_CMD_MODE_TX: //UNMODULATED TX
        NVIC_DisableIRQ(BLE_RF_DIAG_IRQn);
        
        cn = param->frequency;

#if (LUT_PATCH_ENABLED)
        const volatile struct LUT_CFG_struct *pLUT_CFG; // = (const volatile struct LUT_CFG_struct *)(jump_table_struct[lut_cfg_pos]);
        pLUT_CFG = &LUT_CFG;
        if(!pLUT_CFG->HW_LUT_MODE) //so SW_LUT_MODE
        { 
            set_rf_cal_cap(cn); 
            SetBits16(RF_BMCW_REG, CN_SEL,1);       // ENABLE OVERRULE CN_SEL IS 1
            SetBits16(RF_BMCW_REG, CN_WR, cn);
        }
        else //so HW_LUT_MODE
        {
            //HW
            uint8_t * BLE_freq_tbl;
            BLE_freq_tbl = (uint8_t *)(_ble_base + EM_FT_OFFSET); // = EM base address + Frequency Table offset
            SetBits16(RF_BMCW_REG, CN_SEL,1);                                 // ENABLE OVERRULE CN_SEL IS 1
            SetBits16(RF_BMCW_REG, CN_WR, BLE_freq_tbl[cn]);
        }
#else //NO LUT 
        SetBits16(RF_BMCW_REG, CN_SEL,1);                                     // ENABLE OVERRULE CN_SEL IS 1
        SetBits16(RF_BMCW_REG, CN_WR, cn);
#endif

        SetWord16(RF_SYNTH_CTRL3_REG, 0x4000);      //DISABLE MODULATION
        SetWord16(RF_OVERRULE_REG, RX_DIS_WR);      //DISABLE EVENTUAL RX
        SetWord16(RF_OVERRULE_REG, TX_EN_WR);       //ENABLE EVENTUAL TX
        cmd_flag=true;
        test_state = STATE_UNMODULATED_ON;
        break;
    case UNMODULATED_CMD_MODE_RX: //UNMODULATED RX
        NVIC_DisableIRQ(BLE_RF_DIAG_IRQn); 
        
        cn = param->frequency;

#if (LUT_PATCH_ENABLED)
        pLUT_CFG= &LUT_CFG;;
        if(!pLUT_CFG->HW_LUT_MODE) //so SW_LUT_MODE
        { 
            set_rf_cal_cap(cn); 
            SetBits16(RF_BMCW_REG, CN_SEL,1);                                 // ENABLE OVERRULE CN_SEL IS 1
            SetBits16(RF_BMCW_REG, CN_WR, cn);
        }
        else //so HW_LUT_MODE
        {
        //HW
            uint8_t * BLE_freq_tbl;	 
            BLE_freq_tbl = (uint8_t *)(_ble_base + EM_FT_OFFSET); // = EM base address + Frequency Table offset
            SetBits16(RF_BMCW_REG, CN_SEL,1);                                 // ENABLE OVERRULE CN_SEL IS 1
            SetBits16(RF_BMCW_REG, CN_WR, BLE_freq_tbl[cn]);
        }
#else //NO LUT 
        SetBits16(RF_BMCW_REG, CN_SEL,1);                                     // ENABLE OVERRULE CN_SEL IS 1
        SetBits16(RF_BMCW_REG, CN_WR, cn);
#endif
        
        SetWord16(RF_SYNTH_CTRL3_REG, 0x4000);      //DISABLE MODULATION
        SetWord16(RF_OVERRULE_REG, TX_DIS_WR);      //DISABLE EVENTUAL TX
        SetWord16(RF_OVERRULE_REG, RX_EN_WR);       //ENABLE EVENTUAL RX
        cmd_flag=true;
        test_state = STATE_UNMODULATED_ON;
        break;
    default:
        break;
    }
        
    if(cmd_flag)
    {
        hci_send_2_host(event);

    }

    return (KE_MSG_CONSUMED);
}


/*****************************************************************************************
 * @brief Handles the reception of the tx_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_tx_test_handler(ke_msg_id_t const msgid, 
                                           struct hci_tx_test_dialog_cmd const *param,
                                           ke_task_id_t const dest_id, 
                                           ke_task_id_t const src_id)
{
    // structure type for the status event 
    struct hci_cmd_stat_event *event = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT , src_id, HCI_TX_TEST_CMD_OPCODE, hci_cmd_stat_event);
    
    event->status = 0x00;
    hci_send_2_host(event);
    
    test_freq = param->frequency;
    test_data_len = param->data_length;
    test_data_pattern = param->payload_type; 
    text_tx_nr_of_packets = (param->NpacketsMSB<<8) | param->NpacketsLSB;
    set_state_start_tx();
    
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the sleep_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_sleep_test_handler(ke_msg_id_t const msgid, 
                                              struct hci_sleep_test_dialog_cmd const *param,
                                              ke_task_id_t const dest_id, 
                                              ke_task_id_t const src_id)
{
    // structure type for the status event 
    struct hci_cmd_stat_event *event = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT , src_id, HCI_SLEEP_TEST_CMD_OPCODE, hci_cmd_stat_event);
    
    event->status = 0x00;
    hci_send_2_host(event);

    // Finish current UART transfer
    uart_finish_transfers_func();
    
    switch(param->sleep_mode)
    {
    case CMD__SLEEP_MODE_ACTIVE:
        arch_disable_sleep();
        break;
    case CMD__SLEEP_MODE_EXTENDED:
        if (param->mins_to_sleep == 0 && param->seconds_to_sleep == 0)
        {
            rwip_env.ext_wakeup_enable = 2;
        }
        else
        {
             rwip_env.ext_wakeup_enable = 1; 
             rom_cfg_table[max_sleep_duration_periodic_wakeup_pos] = param->mins_to_sleep * 1600 * 60 + param->seconds_to_sleep * 1600;
        }
        arch_set_extended_sleep(false); // no OTP copy
        break;
    case CMD__SLEEP_MODE_DEEP:
        arch_set_deep_sleep(false);
        break;
    default:
        break;
    }

    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the rdtest dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_rdtester_handler(ke_msg_id_t const msgid, 
                                            struct hci_rdtest_dialog_cmd const *param,
                                            ke_task_id_t const dest_id, 
                                            ke_task_id_t const src_id)
{
    // structure type for the complete command event
    void *event = ke_msg_alloc(HCI_CMD_CMP_EVENT , src_id, HCI_RDTESTER_CMD_OPCODE, 0);
    
    switch(param->operation)
    {
    case RDTESTER_INIT:
        rdtest_initialize(param->data & 0x00FF);
        break;        
    case RDTESTER_UART_CONNECT:
        rdtest_uart_connect(param->data);
        break;
    case RDTESTER_UART_LOOPBACK:
        rdtest_make_loopback(param->data & 0x00FF);
        break;
    case RDTESTER_VBAT_CNTRL:
        rdtest_vbatcontrol(param->data);
        rdtest_uart_connect(param->data);
        break;        
    case RDTESTER_VPP_CNTRL:
        rdtest_vppcontrol(param->data & 0x00FF);
        break;
    case RDTESTER_RST_PULSE:
        rdtest_resetpulse(param->data);
        break;
    case RDTESTER_UART_PULSE:
        rdtest_pulse_to_uart(param->data);
        if (param->data)
        {
            rdtest_generate_pulse();
        }
        else
        {
            rdtest_select_pulsewidth(0);
        }
        break;        
    case RDTESTER_XTAL_PULSE:
        rdtest_generate_pulse();
        break;
    case RDTESTER_PULSE_WIDTH:
        rdtest_select_pulsewidth(param->data & 0x00FF);
        break;
    default:
        break;
    }

    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the otp_read dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_otp_read_handler(ke_msg_id_t const msgid, 
                                            struct hci_otp_read_dialog_cmd const *param,
                                            ke_task_id_t const dest_id, 
                                            ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_otp_read_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_OTP_READ_CMD_OPCODE, hci_otp_read_dialog_cmd_cmp_evt);
    uint8_t length = 0;
    uint16_t init_addr = 0;
    uint8_t val[4];
    uint32_t i = 0;
    
    init_addr = param->address;
    length = param->num_of_words;
    
    /* Check that data length is null or bigger than 60*/
    if ((length == 0) || (length > MAX_READ_WRITE_OTP_WORDS))
    {
        event->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
        for (i = 0; i < length; i++)
        {
           otp_read(MEMORY_OTP_BASE + init_addr + (i*4) , val , 4);
           event->wordbyte[i*4]     = val[0];
           event->wordbyte[(i*4)+1] = val[1];
           event->wordbyte[(i*4)+2] = val[2];
           event->wordbyte[(i*4)+3] = val[3];
        }
         event->status = CO_ERROR_NO_ERROR;
    }
    
    event->num_of_words = length;
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the otp_read dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/

static int dialog_commands_otp_write_handler(ke_msg_id_t const msgid, 
                                             struct hci_otp_write_dialog_cmd const *param,
                                             ke_task_id_t const dest_id, 
                                             ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_otp_write_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_OTP_WRITE_CMD_OPCODE, hci_otp_write_dialog_cmd_cmp_evt);
    uint32_t length = 0;
    uint32_t init_addr = 0;
    uint8_t *data_buf;
    uint32_t i = 0;
    uint32_t word[2];
    memset(word, 0, sizeof(word));

    length = param->num_of_words;
    data_buf = (uint8_t*)&param->wordbyte[0];
    init_addr = param->start_addr;
    
    /* Check that data length is null or bigger than 60*/
    if ((length == 0) || (length > MAX_READ_WRITE_OTP_WORDS))
    {
        event->status = CO_ERROR_INVALID_HCI_PARAM;
    }
    else
    {
         for (i = 0; i < length ; i += 2)
         {
             word[0] =  data_buf[(i*4)+3] << 24 | data_buf[(i*4)+2] << 16 | data_buf[(i*4)+1] << 8 | data_buf[(i*4)];
             word[1] =  data_buf[(i*4)+7] << 24 | data_buf[(i*4)+6] << 16 | data_buf[(i*4)+5] << 8 | data_buf[(i*4)+4];
             otp_write_words( init_addr, word , 1);
             init_addr += 8;
         }
         event->status = CO_ERROR_NO_ERROR;
    }
    
    event->num_of_words = param->num_of_words;
    event->status = CO_ERROR_NO_ERROR;
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the sensor_test dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/

static int dialog_commands_sensor_test_handler(ke_msg_id_t const msgid, 
                                               struct hci_sensor_test_dialog_cmd const *param,
                                               ke_task_id_t const dest_id, 
                                               ke_task_id_t const src_id)
{
    // structure type for the complete command event
    struct hci_sensor_test_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT , src_id, HCI_SENSOR_TEST_CMD_OPCODE, hci_sensor_test_dialog_cmd_cmp_evt);
    
    switch(param->interface)
    {
    case 0:
        // Enable the SPI clock.
        SetBits16(CLK_PER_REG, SPI_ENABLE, 1);
    
        // Initialize the SPI.
        SetBits16(SPI_CTRL_REG, SPI_ON,     0);                                     // close SPI block, if opened
        SetBits16(SPI_CTRL_REG, SPI_WORD,   SPI_MODE_8BIT);                         // set SPI bitmode
        SetBits16(SPI_CTRL_REG, SPI_SMN,    SPI_ROLE_MASTER);                       // select role (master/slave)
        SetBits16(SPI_CTRL_REG, SPI_POL,    SPI_CLK_IDLE_POL_HIGH);                 // select SPI clock idle polarity
        SetBits16(SPI_CTRL_REG, SPI_PHA,    SPI_PHA_MODE_1);                        // select SPI sampling edge selection (pha_mode - refer to datasheet p.53-54)
        SetBits16(SPI_CTRL_REG, SPI_MINT,   SPI_MINT_DISABLE);                      // enable/disable SPI interrupt to the NVIC
        SetBits16(SPI_CTRL_REG, SPI_CLK,    SPI_XTAL_DIV_2);                        // SPI block clock divider
        SetBits16(SPI_CTRL_REG, SPI_ON,     1);                                     // enable SPI block

        // CLK
        GPIO_ConfigurePin((GPIO_PORT) param->spi_clk_or_i2c_port, (GPIO_PIN) param->spi_clk_or_i2c_pin, OUTPUT, PID_SPI_CLK, true);
        // MISO
        GPIO_ConfigurePin((GPIO_PORT) param->spi_di_or_i2c_sda_port, (GPIO_PIN) param->spi_di_or_i2c_sda_pin, INPUT_PULLUP, PID_SPI_DI, true);
        // MOSI
        GPIO_ConfigurePin((GPIO_PORT) param->spi_do_port, (GPIO_PIN) param->spi_do_pin, OUTPUT, PID_SPI_DO, true);
        // CS
        GPIO_ConfigurePin((GPIO_PORT) param->spi_cs_port, (GPIO_PIN) param->spi_cs_pin, OUTPUT, PID_GPIO, true);

        // INT/DRDY
        if (param->int_gpio_check)
        {
            GPIO_ConfigurePin((GPIO_PORT) param->int_port, (GPIO_PIN) param->int_pin, INPUT_PULLDOWN, PID_GPIO, false);
        }
        // Write
        if (param->rw == 1) 
        {
            sensor_spi_write_byte((uint16_t) param->register_address, param->register_data_write, (GPIO_PORT) param->spi_cs_port, (GPIO_PIN) param->spi_cs_pin);
        }
        event->data = sensor_spi_read_byte((uint16_t) param->register_address, (GPIO_PORT) param->spi_cs_port, (GPIO_PIN) param->spi_cs_pin);

        if (param->int_gpio_check) 
        {
            for (volatile int i=0; i<500; i++){}
            event->data = GPIO_GetPinStatus((GPIO_PORT) param->int_port, (GPIO_PIN) param->int_pin);
        }

        // CLK
        GPIO_SetPinFunction((GPIO_PORT) param->spi_clk_or_i2c_port, (GPIO_PIN) param->spi_clk_or_i2c_pin, INPUT_PULLDOWN, PID_GPIO);
        // MISO
        GPIO_SetPinFunction((GPIO_PORT) param->spi_di_or_i2c_sda_port, (GPIO_PIN) param->spi_di_or_i2c_sda_pin, INPUT_PULLDOWN, PID_GPIO);
        // MOSI
        GPIO_SetPinFunction((GPIO_PORT) param->spi_do_port, (GPIO_PIN) param->spi_do_pin, INPUT_PULLDOWN, PID_GPIO);
        // CS
        GPIO_SetPinFunction((GPIO_PORT) param->spi_cs_port, (GPIO_PIN) param->spi_cs_pin, INPUT_PULLUP, PID_GPIO);

        // SPI release
        SetBits16(SPI_CTRL_REG, SPI_ON, 0);                                             // Switch off SPI block.
        SetBits16(CLK_PER_REG, SPI_ENABLE, 0);                                          // SPI clock disable.

        break;
    case 1:
        SetBits16(CLK_PER_REG, I2C_ENABLE, 1);                                          // Enable the I2C clock.
        SetWord16(I2C_ENABLE_REG, 0x0);                                                 // Disable the I2C controller.
        SetWord16(I2C_CON_REG, I2C_MASTER_MODE | I2C_SLAVE_DISABLE |I2C_RESTART_EN);    // Slave is disabled.
        SetBits16(I2C_CON_REG, I2C_SPEED, 2);                                           // Set speed to fast mode at 400kbits/s.
        SetBits16(I2C_CON_REG, I2C_10BITADDR_MASTER, 0);                                // Set addressing mode to 7-bit addressing.
        SetWord16(I2C_TAR_REG,param->i2c_slave_address);                                // Set the slave device address.
        SetWord16(I2C_ENABLE_REG, 0x1);                                                 // Enable the I2C controller
        while(GetWord16(I2C_STATUS_REG) & 0x20);                                        // Wait for I2C master FSM to be IDLE
        
        // I2C_SCL GPIO initialize.
        GPIO_SetPinFunction((GPIO_PORT) param->spi_clk_or_i2c_port, (GPIO_PIN) param->spi_clk_or_i2c_pin, INPUT, PID_I2C_SCL);
        // I2C_SDA GPIO initialize.
        GPIO_SetPinFunction((GPIO_PORT) param->spi_di_or_i2c_sda_port, (GPIO_PIN) param->spi_di_or_i2c_sda_pin, INPUT, PID_I2C_SDA);
        // INT/DRDY
        if (param->int_gpio_check)
        {
            GPIO_ConfigurePin((GPIO_PORT) param->int_port, (GPIO_PIN) param->int_pin, INPUT_PULLDOWN, PID_GPIO, false);
        }
        // Write
        if (param->rw == 1) {
            sensor_i2c_write_byte((uint32_t) param->register_address, param->register_data_write);
        }
        event->data  = sensor_i2c_read_byte((uint32_t) param->register_address);

        if (param->int_gpio_check) 
        {
            for (volatile int i=0; i<500; i++){}
            event->data = GPIO_GetPinStatus((GPIO_PORT) param->int_port, (GPIO_PIN) param->int_pin);
        }

        SetWord16(I2C_ENABLE_REG, 0x0);                                                 // Disable the I2C controller.
        SetBits16(CLK_PER_REG, I2C_ENABLE, 0);                                          // Disable clock for I2C.
        break;
    default:
        break;
    }

    set_pad_functions();                                                                // Restore UART pins in case they are used in a sensor.
    // Send the answer.
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the gpio_set dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_gpio_set_handler(ke_msg_id_t const msgid, 
                                            struct hci_gpio_set_dialog_cmd const *param,
                                            ke_task_id_t const dest_id, 
                                            ke_task_id_t const src_id)
{
    // structure type for the status event 
    struct hci_gpio_set_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT ,src_id, HCI_GPIO_SET_CMD_OPCODE, hci_gpio_set_dialog_cmd_cmp_evt);
    uint8_t err = 0x00;
    GPIO_PORT port;
    GPIO_PIN pin;
    GPIO_PUPD mode;
    uint8_t volt_rail;
    uint8_t state;
    
    /* Transform the input. */
    port = (GPIO_PORT) (param->gpio_pad / 10);
    pin = (GPIO_PIN) (param->gpio_pad % 10);
    volt_rail = param->gpio_lvl;
    state = param->val;
    
     /* Validate the GPIO port. */
     if (!(port <= GPIO_PORT_3))
     {
        err = 1;
     }

     /* Validate the GPIO pin. */
     if (!(pin <= GPIO_PIN_7) 
                 ||(port == GPIO_PORT_2 && pin >= GPIO_PIN_4) 
                 || (port == GPIO_PORT_1 && pin == GPIO_PIN_4) || (port == GPIO_PORT_1 && pin == GPIO_PIN_5)
                 || (port == GPIO_PORT_0 && pin == GPIO_PIN_4) || (port == GPIO_PORT_0 && pin == GPIO_PIN_5))
     {
         err = 1;
     }
    


     /* Transform and validate the GPIO mode. */
     switch (param->mode) 
     {
         case 0: mode = INPUT; break;
         case 1: mode = INPUT_PULLUP; break;
         case 2: mode = INPUT_PULLDOWN; break;
         case 3: mode = OUTPUT; break;
         default: err = 1; break;
     }
     
     /* Validate the GPIO voltage rail. 0 = 3.3V, 1 = 1.8V. */
      if (volt_rail > 1) err = 1;

      /* 0=Reset, 1=Set. Only valid in output mode. */
      if ((state > 1) && (param->mode >= 0x300)) err = 1;

      /* If error exists in the received message, reply with error. */
      if (err) 
      {
          event->error = 0x01;
      }
      else
      {
          GPIO_ConfigurePin( port, pin, mode, PID_GPIO, state==1 ? true : false);
          GPIO_ConfigurePinPower(port, pin, volt_rail ? GPIO_POWER_RAIL_1V : GPIO_POWER_RAIL_3V);
          event->error = 0x00;
          
      }

    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the gpio_read dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_gpio_read_handler(ke_msg_id_t const msgid, 
                                            struct hci_gpio_read_dialog_cmd const *param,
                                            ke_task_id_t const dest_id, 
                                            ke_task_id_t const src_id)
{
    // structure type for the status event 
    struct hci_gpio_read_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT ,src_id, HCI_GPIO_READ_CMD_OPCODE, hci_gpio_read_dialog_cmd_cmp_evt);
    uint8_t err = 0x00;
    GPIO_PORT port;
    GPIO_PIN pin;

    /* Transform the input. */
    port = (GPIO_PORT) (param->gpio_pad / 10);
    pin = (GPIO_PIN) (param->gpio_pad % 10);
    
    /* Validate the GPIO port. */
    if (!(port <= GPIO_PORT_3))
                err = 1;
    
         /* Validate the GPIO pin. */
     if (!(pin <= GPIO_PIN_7) 
                 ||(port == GPIO_PORT_2 && pin >= GPIO_PIN_4) 
                 || (port == GPIO_PORT_1 && pin == GPIO_PIN_4) || (port == GPIO_PORT_1 && pin == GPIO_PIN_5)
                 || (port == GPIO_PORT_0 && pin == GPIO_PIN_4) || (port == GPIO_PORT_0 && pin == GPIO_PIN_5))
     {
         err = 1;
     }
    
    /* If error exists in the received message, reply with error. */
    if (err)
    {
        event->data = 0xFF;
    }
    else 
    {
        event->data = GPIO_GetPinStatus( port, pin);
    }
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the uart_loop dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_uart_loop_handler(ke_msg_id_t const msgid, 
                                            struct hci_uart_loop_dialog_cmd const *param,
                                            ke_task_id_t const dest_id, 
                                            ke_task_id_t const src_id)
{
    // structure type for the status event 
    struct hci_uart_loop_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT ,src_id, HCI_UART_LOOP_CMD_OPCODE, hci_uart_loop_dialog_cmd_cmp_evt);
    
    memcpy(event->data, param->data, sizeof(event->data));
    
    hci_send_2_host(event);
    return (KE_MSG_CONSUMED);
}

/*****************************************************************************************
 * @brief Handles the reception of the uart_baud dialog hci command.
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
******************************************************************************************/
static int dialog_commands_uart_baud_handler(ke_msg_id_t const msgid,
                                            struct hci_uart_baud_dialog_cmd const *param,
                                            ke_task_id_t const dest_id,
                                            ke_task_id_t const src_id)
{
    // Structure type for the status event.
    struct hci_uart_baud_dialog_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, src_id, HCI_UART_BAUD_CMD_OPCODE, hci_uart_baud_dialog_cmd_cmp_evt);

    if ((param->data == 4) ||
        (param->data == 3) ||
        (param->data == 2) ||
        (param->data == 1) ||
        (param->data == 0))
        event->error = 0;
    else event->error = 1;

    hci_send_2_host(event);
    uart_finish_transfers_func();

    for (volatile int i=0; i<200; i++) {}; // Wait some time for UART reset.

    switch (param->data)
    {
        case 4:
            baud_rate_sel = UART_BAUDRATE_1M;
            baud_rate_frac_sel = UART_FRAC_BAUDRATE_1M;
            break;
        case 3:
            baud_rate_sel = UART_BAUDRATE_115K2;
            baud_rate_frac_sel = UART_FRAC_BAUDRATE_115K2;
            break;
        case 2:
            baud_rate_sel = UART_BAUDRATE_57K6;
            baud_rate_frac_sel = UART_FRAC_BAUDRATE_57K6;
            break;
        case 1:
            baud_rate_sel = UART_BAUDRATE_19K2;
            baud_rate_frac_sel = UART_FRAC_BAUDRATE_19K2;
            break;
        case 0:
            baud_rate_sel = UART_BAUDRATE_9K6;
            baud_rate_frac_sel = UART_FRAC_BAUDRATE_9K6;
            break;
        default:
            baud_rate_sel = UART_BAUDRATE_115K2;
            baud_rate_frac_sel = UART_FRAC_BAUDRATE_115K2;
            break;
    }
    uart_init(baud_rate_sel, baud_rate_frac_sel, UART_CHARFORMAT_8);

    h4tl_init(rwip_eif_get(RWIP_EIF_HCIC));

    return (KE_MSG_CONSUMED);
}

/// Special unpacking function for HCI Debug Write OTP Command
static uint8_t hci_wr_otp_cmd_upk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_otp_write_dialog_cmd* cmd = (struct hci_otp_write_dialog_cmd*) out;
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;

    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Start address
            p_out = (uint8_t*) &cmd->start_addr;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 2);
            if(status != HCI_PACK_OK)
                break;

            // Data length
            data_len = *p_in;
            p_out = &cmd->num_of_words;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Data
            data_len *= 4;
            p_out = &cmd->wordbyte[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        // If no input data, size max is returned
        *out_len = sizeof(struct hci_dbg_wr_mem_cmd);
    }

    return status;
}

/// Special packing function for Command Complete Event of HCI  Read otp Command
static uint8_t hci_otp_rd_data_cmd_cmp_evt_pk(uint8_t *out, uint8_t *in, uint16_t* out_len, uint16_t in_len)
{
    struct hci_otp_read_dialog_cmd_cmp_evt* evt = (struct hci_otp_read_dialog_cmd_cmp_evt*)(in);
    uint8_t* p_in = in;
    uint8_t* p_out = out;
    uint8_t* p_in_end = in + in_len;
    uint8_t* p_out_end = out + *out_len;
    uint8_t status = HCI_PACK_OK;
    
    // Check if there is input data to parse
    if(in != NULL)
    {
        do
        {
            uint8_t data_len;

            // Status
            p_in = &evt->status;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;

            // Number of Words
            p_in = &evt->num_of_words;
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, 1);
            if(status != HCI_PACK_OK)
                break;
            
            // Words
            data_len = evt->num_of_words * 4;
            p_in = &evt->wordbyte[0];
            status = hci_pack_bytes(&p_in, &p_out, p_in_end, p_out_end, data_len);
            if(status != HCI_PACK_OK)
                break;

        } while(0);

        *out_len =  (uint16_t)(p_out - out);
    }
    else
    {
        *out_len = 0;
    }

    return status;
}

/**
****************************************************************************************
* @brief Apply a basic pack operation
*
* @param[inout] pp_in      Current input buffer position
* @param[inout] pp_out     Current output buffer position
* @param[in]    p_in_end   Input buffer end
* @param[in]    p_out_end  Output buffer end
* @param[in]    len        Number of bytes to copy
*
* @return status
*****************************************************************************************
*/
static uint8_t hci_pack_bytes(uint8_t** pp_in, uint8_t** pp_out, uint8_t* p_in_end, uint8_t* p_out_end, uint8_t len)
{
    uint8_t status = HCI_PACK_OK;
    // Check if enough space in input buffer to read
    if((*pp_in + len) > p_in_end)
    {
        status = HCI_PACK_IN_BUF_OVFLW;
    }
    else
    {
        if(p_out_end != NULL)
        {
            // Check if enough space in out buffer to write
            if((*pp_out + len) > p_out_end)
            {
                status = HCI_PACK_OUT_BUF_OVFLW;
            }

            // Copy bytes
            memcpy(*pp_out, *pp_in, len);
        }
        *pp_in = (*pp_in + len);
        *pp_out = *pp_out + len;
    }

    return status;
}
// The message handlers for dialog HCI command complete events
const struct ke_msg_handler dialog_commands_handler_tab[] =
{
        {HCI_CUSTOM_ACTION_CMD_OPCODE           ,  (ke_msg_func_t)dialog_commands_custom_action_handler          },
        {HCI_SLEEP_TEST_CMD_OPCODE              ,  (ke_msg_func_t)dialog_commands_sleep_test_handler             },
        {HCI_XTAL_TRIM_CMD_OPCODE               ,  (ke_msg_func_t)dialog_commands_xtal_trim_handler              },
        {HCI_OTP_RW_CMD_OPCODE                  ,  (ke_msg_func_t)dialog_commands_otp_rw_handler                 },
        {HCI_OTP_READ_CMD_OPCODE                ,  (ke_msg_func_t)dialog_commands_otp_read_handler               },
        {HCI_OTP_WRITE_CMD_OPCODE               ,  (ke_msg_func_t)dialog_commands_otp_write_handler              },
        {HCI_REGISTER_RW_CMD_OPCODE             ,  (ke_msg_func_t)dialog_commands_register_rw_handler            },
        {HCI_AUDIO_TEST_CMD_OPCODE              ,  (ke_msg_func_t)dialog_commands_audio_test_handler             },
        {HCI_FIRMWARE_VERSION_GET_CMD_OPCODE    ,  (ke_msg_func_t)dialog_commands_firmware_version_get_handler   },
        {HCI_CHANGE_UART_PINS_ACTION_CMD_OPCODE ,  (ke_msg_func_t)dialog_commands_change_uart_pins_action_handler},
        {HCI_RDTESTER_CMD_OPCODE                ,  (ke_msg_func_t)dialog_commands_rdtester_handler               },
        {HCI_TX_TEST_CMD_OPCODE                 ,  (ke_msg_func_t)dialog_commands_tx_test_handler                },
        {HCI_START_PROD_RX_TEST_CMD_OPCODE      ,  (ke_msg_func_t)dialog_commands_start_prod_rx_test_handler     },
        {HCI_END_PROD_RX_TEST_CMD_OPCODE        ,  (ke_msg_func_t)dialog_commands_end_prod_rx_test_handler       },
        {HCI_UNMODULATED_ON_CMD_OPCODE          ,  (ke_msg_func_t)dialog_commands_unmodulated_on_handler         },
        {HCI_TX_START_CONTINUE_TEST_CMD_OPCODE  ,  (ke_msg_func_t)dialog_commands_tx_start_continue_test_handler },
        {HCI_TX_END_CONTINUE_TEST_CMD_OPCODE    ,  (ke_msg_func_t)dialog_commands_tx_end_continue_test_handler   },
        {HCI_SENSOR_TEST_CMD_OPCODE             ,  (ke_msg_func_t)dialog_commands_sensor_test_handler            },
        {HCI_GPIO_SET_CMD_OPCODE                ,  (ke_msg_func_t)dialog_commands_gpio_set_handler               },
        {HCI_GPIO_READ_CMD_OPCODE               ,  (ke_msg_func_t)dialog_commands_gpio_read_handler              },
        {HCI_UART_LOOP_CMD_OPCODE               ,  (ke_msg_func_t)dialog_commands_uart_loop_handler              },
        {HCI_UART_BAUD_CMD_OPCODE               ,  (ke_msg_func_t)dialog_commands_uart_baud_handler              },
};

const uint8_t dialog_commands_handler_num = ARRAY_LEN (dialog_commands_handler_tab);
