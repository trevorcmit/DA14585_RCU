/*****************************************************************************************
 *
 * @file Dialog_prod.c
 *
 * @brief Dialog production source code.
  *
******************************************************************************************/

 /*
 * INCLUDE FILES
******************************************************************************************/
#include <stdint.h>
#include "app.h"
#include "rwip_config.h"
#include "hw_otpc.h"
#include "dialog_prod.h"
#include "user_periph_setup.h"
#include "gpio.h"
#include "llm.h"
#include "lld.h"
#include "lld_evt.h"
#include "spi.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
******************************************************************************************/
volatile uint8_t test_state;
volatile uint16_t test_rx_irq_cnt;
volatile uint8_t test_data_len;
volatile uint16_t text_tx_nr_of_packets;
volatile uint16_t test_tx_packet_nr;

// I2C macros
#define SEND_I2C_COMMAND(X)             SetWord16(I2C_DATA_CMD_REG, (X))
#define WAIT_WHILE_I2C_FIFO_IS_FULL()   while(!(GetWord16(I2C_STATUS_REG) & TFNF))
#define WAIT_UNTIL_I2C_FIFO_IS_EMPTY()  while(!(GetWord16(I2C_STATUS_REG) & TFE))
#define WAIT_UNTIL_NO_MASTER_ACTIVITY() while(GetWord16(I2C_STATUS_REG) & MST_ACTIVITY)
#define WAIT_FOR_RECEIVED_BYTE()        while(!GetWord16(I2C_RXFLR_REG))

void app_init(void)
{
    arch_disable_sleep();
    
    test_state = STATE_IDLE;
    test_data_pattern = 2;
    test_freq = 0;
    test_data_len = 37;
    text_tx_nr_of_packets = 50;
    test_tx_packet_nr = 0;
    metrics.rx_err = 0;
    metrics.rx_err_sync = 0;
    metrics.rx_err_crc = 0;
    metrics.rx_pkt = 0;
    metrics.rx_rssi = 0;
    SetBits32(BLE_CNTL2_REG, BLE_RSSI_SEL, 1); //SELECT CURRENT AVERAGE RSSI.    
    
    init_TXEN_RXEN_irqs();

#if HAS_AUDIO
    audio_init();
#endif
}

int otp_write_words(uint32_t otp_pos, uint32_t *val_addr, uint8_t words_count)
{
    int result;
    uint32_t otp_addr;

    for(uint16_t i = 0; i < words_count; i += 2)
    {
        otp_addr = otp_pos;
        //Initialize OTP Controller
        hw_otpc_init();
        //write the OTP
        result = hw_otpc_fifo_prog((const uint32_t *) (val_addr), otp_addr >> 3, HW_OTPC_WORD_LOW, 2, false) ? 0 : 1;
       
        hw_otpc_disable();
        otp_pos += 8;
    
        if (result != 0)
            break;
    }

    //Close the OTP Controller
    hw_otpc_close();

    return 0;
}

void otp_read(uint32_t otp_pos, uint8_t *val, uint8_t len)
{

    uint8_t *otp_val = (uint8_t *)otp_pos;   //where in OTP header to read
    
    //Initialize OTP Controller
    hw_otpc_init();
    // set OTP in read mode
    hw_otpc_manual_read_on(false);

    memcpy(val, otp_val, len);

    hw_otpc_disable();
}

void set_state_start_continue_tx(void)
{
    if(test_state == STATE_IDLE)
    {
        struct hci_le_tx_test_cmd  tx_con_test;
        tx_con_test.tx_freq = test_freq;
        tx_con_test.test_data_len = 37; //select a valid value.
        tx_con_test.pk_payload_type = test_data_pattern;

        llm_test_mode_start_tx( (struct hci_le_tx_test_cmd const *)&tx_con_test);

        SetBits32(BLE_RFTESTCNTL_REG,INFINITETX,1);
        SetBits32(BLE_RFTESTCNTL_REG,TXLENGTHSRC,0);
        SetBits32(BLE_RFTESTCNTL_REG,TXPLDSRC,0);
        SetBits32(BLE_RFTESTCNTL_REG,TXLENGTH,0);
    
        test_state = STATE_START_CONTINUE_TX;
    }
}

void set_state_stop(void)
{
    if( (test_state==STATE_START_TX) || (test_state==STATE_START_RX) || test_state==STATE_START_CONTINUE_TX )// in case of default
                                                                                                             //direct TX or RX stack handles the end of these tasks
    {

         lld_test_stop(llm_le_env.elt);
         // Set the state to stopping
         ke_state_set(TASK_LLM, LLM_IDLE);
    }
    test_state = STATE_IDLE;
}


void set_state_start_tx(void)
{
    if(test_state==STATE_IDLE)
    {
        test_tx_packet_nr=0;
        struct hci_le_tx_test_cmd  tx_test;
        tx_test.tx_freq = test_freq;
        tx_test.test_data_len = test_data_len; //37;
        tx_test.pk_payload_type = test_data_pattern;

        llm_test_mode_start_tx( (struct hci_le_tx_test_cmd const *)&tx_test);
        while(llm_le_env.elt == NULL) 
            if(llm_test_mode_start_tx( (struct hci_le_tx_test_cmd const *)&tx_test) != CO_ERROR_NO_ERROR)
                while(1);

        test_state=STATE_START_TX;
    }
}

void set_state_start_rx(void)
{
    if(test_state==STATE_IDLE)
    {
        metrics.rx_err = 0;
        metrics.rx_err_sync = 0;
        metrics.rx_err_crc = 0;
        metrics.rx_pkt = 0;
        metrics.rx_rssi = 0;
        struct hci_le_rx_test_cmd  rx_test;
        rx_test.rx_freq = test_freq;
        
        llm_test_mode_start_rx( (struct hci_le_rx_test_cmd const *)&rx_test);
        test_state=STATE_START_RX;
    }
}

uint8_t check_uart_pins_cmd(uint8_t *ptr_data)
{
    uint8_t result = 0;

    if ((ptr_data[0] == GPIO_PORT_0) || (ptr_data[0] == GPIO_PORT_3)) {
        if (ptr_data[1] > GPIO_PIN_7)
            result = 1;
    }

    if ((ptr_data[2] == GPIO_PORT_0) || (ptr_data[2] == GPIO_PORT_3)) {
        if (ptr_data[3] > GPIO_PIN_7)
            result = 1;
    }

    if (ptr_data[0] == GPIO_PORT_1) {
        if (ptr_data[1] > GPIO_PIN_5)
            result = 1;
    }

    if (ptr_data[2] == GPIO_PORT_1) {
        if (ptr_data[3] > GPIO_PIN_5)
            result = 1;
    }

    if (ptr_data[0] == GPIO_PORT_2) {
        if (ptr_data[1] > GPIO_PIN_9)
            result = 1;
    }

    if (ptr_data[2] == GPIO_PORT_2) {
        if (ptr_data[3] > GPIO_PIN_9)
            result = 1;
    }
    
    return result;
}

void change_uart_pins_cmd(uint8_t *ptr_data)
{
    for (volatile int i=0; i<2000; i++){}
    // Return previous UART GPIOs to default settings
    GPIO_ConfigurePin((GPIO_PORT) uart_sel_pins.uart_port_tx, (GPIO_PIN) uart_sel_pins.uart_tx_pin, INPUT, PID_GPIO, false);
    GPIO_ConfigurePin((GPIO_PORT) uart_sel_pins.uart_port_rx, (GPIO_PIN) uart_sel_pins.uart_rx_pin, INPUT, PID_GPIO, false);
    #ifndef GPIO_DRV_PIN_ALLOC_MON_DISABLED
        RESERVE_GPIO(UART1_TX, (GPIO_PORT) ptr_data[0], (GPIO_PIN) ptr_data[1], PID_UART1_TX);
        RESERVE_GPIO(UART1_TX, (GPIO_PORT) ptr_data[2], (GPIO_PIN) ptr_data[3], PID_UART1_RX);
    #endif // GPIO_DRV_PIN_ALLOC_MON_DISABLED
    update_uart_pads ((GPIO_PORT) ptr_data[0],(GPIO_PIN) ptr_data[1],(GPIO_PORT) ptr_data[2], (GPIO_PIN) ptr_data[3]);
    set_pad_uart();
}

uint8_t sensor_i2c_read_byte(uint8_t address)
{
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();                 // Wait in case Tx FIFO is full.
    SEND_I2C_COMMAND(address);                      // Write the address of the register.
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();                 // Wait in case Tx FIFO is full.
    SEND_I2C_COMMAND(0x0100);                       // Set R/W bit to 1 (read access)
    WAIT_FOR_RECEIVED_BYTE();                       // Wait for the received byte.
    return (0xFF & GetWord16(I2C_DATA_CMD_REG));    // Get the received byte.
}

void sensor_i2c_write_byte(uint32_t address, uint8_t wr_data)
{
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();
    SEND_I2C_COMMAND(address);          // Write the address of the register
    SEND_I2C_COMMAND(wr_data & 0xFF);   // Write the data of the register
    WAIT_UNTIL_I2C_FIFO_IS_EMPTY();     // Wait if I2C Tx FIFO is full

    WAIT_UNTIL_NO_MASTER_ACTIVITY();    // Wait until no master activity 
}

uint32_t sensor_spi_access(uint16_t dataToSend)
{
    uint32_t dataRead = 0;

    SetWord16(SPI_RX_TX_REG0, dataToSend);                      // write (low part of) dataToSend

    do
    {
    }while (GetBits16(SPI_CTRL_REG, SPI_INT_BIT) == 0);         // polling to wait for spi transmission

    SetWord16(SPI_CLEAR_INT_REG, 0x01);                         // clear pending flag

    // Read from Registers
    dataRead += GetWord16(SPI_RX_TX_REG0);                      // read (low part of) data from spi slave

    return dataRead;                                            // return data read from spi slave
}

void sensor_spi_write_byte(uint8_t address, uint8_t wr_data, GPIO_PORT cs_port, GPIO_PIN cs_pin)
{
    // CS low
    GPIO_SetInactive(cs_port, cs_pin);
    // Address
    sensor_spi_access((uint16_t) address);
    // Data
    sensor_spi_access((uint16_t) wr_data);
    // CS high
    GPIO_SetActive(cs_port, cs_pin);
}

uint8_t sensor_spi_read_byte(uint8_t address, GPIO_PORT cs_port, GPIO_PIN cs_pin)
{
    uint8_t result = 0;

    // CS low
    GPIO_SetInactive(cs_port, cs_pin);
    // Address
    sensor_spi_access((uint16_t) (address | 0x80));
    // Read data
    result = (uint8_t) sensor_spi_access(0x0000);
    // CS high
    GPIO_SetActive(cs_port, cs_pin);
    
    return result;
}

uint32_t find_mode_register(uint8_t port_number)
{
    uint32_t port_reg = 0;

    // Get the register of the port
    switch (port_number)
    {
        case 00:    port_reg = P00_MODE_REG; break;
        case 01:    port_reg = P01_MODE_REG; break;
        case 02:    port_reg = P02_MODE_REG; break;
        case 03:    port_reg = P03_MODE_REG; break;
        case 04:    port_reg = P04_MODE_REG; break;
        case 05:    port_reg = P05_MODE_REG; break;
        case 06:    port_reg = P06_MODE_REG; break;
        case 07:    port_reg = P07_MODE_REG; break;
        case 10:    port_reg = P10_MODE_REG; break;
        case 11:    port_reg = P11_MODE_REG; break;
        case 12:    port_reg = P12_MODE_REG; break;
        case 13:    port_reg = P13_MODE_REG; break;
        case 14:    port_reg = P14_MODE_REG; break;
        case 15:    port_reg = P15_MODE_REG; break;
        case 20:    port_reg = P20_MODE_REG; break;
        case 21:    port_reg = P21_MODE_REG; break;
        case 22:    port_reg = P22_MODE_REG; break;
        case 23:    port_reg = P23_MODE_REG; break;
        case 24:    port_reg = P24_MODE_REG; break;
        case 25:    port_reg = P25_MODE_REG; break;
        case 26:    port_reg = P26_MODE_REG; break;
        case 27:    port_reg = P27_MODE_REG; break;
        case 28:    port_reg = P28_MODE_REG; break;
        case 29:    port_reg = P29_MODE_REG; break;
        case 30:    port_reg = P30_MODE_REG; break;
        case 31:    port_reg = P31_MODE_REG; break;
        case 32:    port_reg = P32_MODE_REG; break;
        case 33:    port_reg = P33_MODE_REG; break;
        case 34:    port_reg = P34_MODE_REG; break;
        case 35:    port_reg = P35_MODE_REG; break;
        case 36:    port_reg = P36_MODE_REG; break;
        case 37:    port_reg = P37_MODE_REG; break;
    }
    
    return(port_reg);
}

