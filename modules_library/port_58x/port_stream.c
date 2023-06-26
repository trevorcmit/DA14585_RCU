 /*****************************************************************************************
 *
 * \file port_stream.c
 *
 * \brief BLE stream module platform adaptation source file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup STREAM
 * \{
 * \addtogroup PORT_STREAM
 * \{
 ****************************************************************************************	 
 */ 

/*
 * INCLUDE FILES
******************************************************************************************/

#include "rwip_config.h"              

#include <port_platform.h>
#include "app_stream.h"
#include <app_stream_config.h>
#include "l2cc_task.h"
#include "l2cm.h"
#include "port_stream.h"
#if (BLE_HID_DEVICE)
    #include "hogpd.h"
#endif

#include "prf_utils.h"


//#define STREAM_DATA_TO_UART

#ifdef HAS_BLE_STREAM

uint8_t stream_cpt_event = 0;
bool transmitting_data = false;

#ifndef CFG_APP_STREAM_PACKET_BASED
    extern uint8_t stream_packet_size;
    extern bool stream_ignore_packet_size;
#endif

static bool port_stream_send_pkt_to_l2cc(app_stream_pkt_t *strpkt)
{
    uint16_t length;
#if APP_STREAM_USE_PREDEFINED_BUFFERS
    length = APP_STREAM_PACKET_SIZE;
#else
    length = strpkt->len;
#endif

    struct l2cc_pdu_send_req *pkt = port_create_l2cc_pdu(length, strpkt->used_hndl);

    if (!pkt) {
        return false;
    }
    // copy the content to value 
    app_stream_copy_pkt_data(pkt->pdu.data.hdl_val_ntf.value, strpkt);
    ke_msg_send(pkt);
    return true;
}

#define MAX_BUFS_PCON_INT (7)
#define MIN_AVAILABLE (2)
#define MAX_PACKETS_COMP_EVT (2)


/*****************************************************************************************
 * \brief
 *
 * \param[in] min_avail
 *
 * \return
******************************************************************************************/
static inline int stream_queue_data_until(uint8_t min_avail)
{
        int max_count=0;
        int available;
        app_stream_pkt_t *pkt;
        int retval=0;
        uint16_t buffer_size;
        
#ifdef CFG_APP_STREAM_PACKET_BASED  
        pkt = app_stream_get_data(0);
#else    
        pkt = app_stream_get_data(stream_packet_size);
#endif    
        
#ifdef CFG_APP_STREAM_PACKET_BASED
        if (pkt == NULL) {
#else            
        // Don't send packets smaller than stream_packet_size
        if (pkt == NULL || (stream_ignore_packet_size == false && pkt->len < stream_packet_size)) {
#endif          
                return retval; //nothing to send quick check in order not to spend time
        }
        
#if (RWBLE_SW_VERSION_MAJOR >= 8) 
        buffer_size = l2cm_get_buffer_size(0); // 27 if packet length extension is not used
#else
        buffer_size = 27;    
#endif        

        int16_t packet_length;
#if APP_STREAM_USE_PREDEFINED_BUFFERS  
        packet_length = APP_STREAM_PACKET_SIZE;
#else        
        packet_length = pkt->len;
#endif
        
        uint8_t number_of_packets = 1;
        int16_t remaining_len = packet_length - (buffer_size-7);
        
        if(remaining_len > 0) {                   
            number_of_packets += (uint8_t)(remaining_len/buffer_size+((remaining_len%buffer_size)>0 ? 1 : 0));  
        }   
        
        ASSERT_ERROR(number_of_packets < MAX_TX_BUFS);
        
        available=l2cm_get_nb_buffer_available();
        if (available <= number_of_packets) {
                return retval;       //never place the device into busy state
        }
	  
        max_count=(available-min_avail)/number_of_packets;  //maximum number of packets to add 
        // Don't send too many big packets because the message heap will get full
        max_count = co_min(max_count, 300 / packet_length + 1); 
    
#ifdef CFG_APP_STREAM_PACKET_BASED
        while (pkt != NULL && (max_count > 0)) {
#else            
        // Don't send packets smaller than stream_packet_size
        while (pkt != NULL && (stream_ignore_packet_size == true || pkt->len == stream_packet_size) && (max_count > 0)) {
#endif            
                if(port_stream_send_pkt_to_l2cc(pkt) == true) {
#if APP_STREAM_USE_CIRCULAR_BUFFER  
                        if (pkt->p_callback) {
                                pkt->p_callback(pkt->datapt,pkt->used_hndl);
                        }
#endif

#ifdef STREAM_DEBUG
                        GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_3, OUTPUT, PID_GPIO, true); 
                        for(volatile int i=0;i<100;i++);
                        GPIO_ConfigurePin(GPIO_PORT_1, GPIO_PIN_3, OUTPUT, PID_GPIO, false); 
                        dbg_putc('-');
#endif            
                        app_stream_free_packet();
                        retval++;
                }
                max_count--;
    #ifdef CFG_APP_STREAM_PACKET_BASED  
                pkt = app_stream_get_data(0);
    #else    
                pkt = app_stream_get_data(stream_packet_size);
    #endif 
        }
        return (retval);  
}

bool asynch_flag = false;
int str_count=MAX_TX_BUFS;
int min_val=1;


/*****************************************************************************************
 * \brief       
 *
 * \return
******************************************************************************************/
static int stream_queue_more_data_during_tx(void)
{
    int retval=0;
    int until=min_val;
    int available, already_in;
    available=l2cm_get_nb_buffer_available();
    already_in=MAX_TX_BUFS-available;
  
    if (asynch_flag) { //previous connection event ended with asynch, current frees the buffers
        until=available-(str_count-1);
    }
    if (until < 1) {
        until = 1;
    }

    if (already_in > str_count) {   //send more than we want to, there will not be enough time 
        asynch_flag = true;     //to add more on the complete event we will add and flag 
    } else {                        //to complete event NOT to try to add more to get synched again
        asynch_flag = false;
    }

    retval=stream_queue_data_until (until);
    return (retval);
}


/*****************************************************************************************
 * \brief       
 *
 * \return
******************************************************************************************/
static int stream_queue_more_data_end_of_event(void)
{
    int retval=0;
	  
    int available, already_in, until;
    available=l2cm_get_nb_buffer_available();
    already_in=MAX_TX_BUFS-available;

    if (already_in >= str_count) {
        return retval; //no need to send anything
    }
  
    if ((available < (MAX_TX_BUFS-1)) && asynch_flag) {
        return retval;  //try to get synched again
    }
   
    until=MAX_TX_BUFS-str_count;
    if (!until && (str_count != MAX_TX_BUFS)) {
        until=1;
    }
    
    retval=stream_queue_data_until(until);
    return retval;
}

#ifdef STREAM_DATA_TO_UART


/*****************************************************************************************
 * \brief       
 *
 * \param   strpkt
******************************************************************************************/
static void stream_send_pkt_to_uart(app_stream_pkt_t *strpkt)
{
    uint8_t length;
    uint8_t *data;
 
#if APP_STREAM_USE_CIRCULAR_BUFFER  
    length = strpkt->len;
#else
    length = APP_STREAM_PACKET_SIZE;
#endif
    data = (uint8_t *)strpkt->datapt;
    
    for( ; length > 0; length--) {
        dbg_putc(*data++);
    }
}


/*****************************************************************************************
 * \brief
 *
 * \return
******************************************************************************************/
static enum stream_status stream_data_to_uart(void)
{
    int max_count=5;
    app_stream_pkt_t *pkt;
    
    bool ret = app_stream_get_data(&pkt);
   
    if (ret == false) {
        return STREAM_NO_MORE_DATA;
    }

    while (ret == true && (max_count > 0)) {
            stream_send_pkt_to_uart(pkt);
            app_stream_next_packet();
            max_count--;
            ret = app_stream_get_data(&pkt);
    }
    return (ret == true) ? STREAM_MORE_DATA : STREAM_NO_MORE_DATA;  
}
#endif

enum stream_status port_stream_queue_data(void)
{
#ifndef STREAM_DATA_TO_UART    
    extern bool stream_first_packets;
    int retval=0;
      
    if (stream_cpt_event == 1) {         
        retval=stream_queue_more_data_end_of_event();
        stream_first_packets = (retval == 0);
        transmitting_data = false;
        stream_cpt_event = 2;
    } else if ((transmitting_data == true && stream_cpt_event == 2) || stream_first_packets == true) {
        retval=stream_queue_more_data_during_tx();
        transmitting_data = false;
//        stream_first_packets = (retval == 0);
        if (stream_cpt_event == 2) {
            //if transmission stopped from remote the cpt_event may be already there
            stream_cpt_event=0;
        }
    }
    return (retval !=0 ? STREAM_MORE_DATA : STREAM_NO_MORE_DATA);
#else  //STREAM_DATA_TO_UART
    return stream_data_to_uart();
#endif    
}

#endif // HAS_BLE_STREAM

/**
 * \}
 * \}
 * \}
 */
