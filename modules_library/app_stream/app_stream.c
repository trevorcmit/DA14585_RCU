/**
 ****************************************************************************************
 *
 * \file app_stream.c
 *
 * \brief BLE stream module source file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */
 
/**
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup APP_STREAM
 * \brief AudioStreamer Application
 *
 * \{
 ****************************************************************************************
 */

#include "app_stream.h"
#include <app_stream_config.h>
#include "port_stream.h"

#ifdef HAS_BLE_STREAM

#include <string.h>
#include "co_math.h"

#ifndef STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES
        #define STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES 0
#endif

typedef  struct app_stream_env_s
{  
        bool stream_enabled;
        uint16_t fifo_size;
} app_stream_env_t;

bool stream_first_packets       __PORT_RETAINED;

app_stream_env_t app_stream_env __PORT_RETAINED;
#ifndef CFG_APP_STREAM_PACKET_BASED
        uint8_t stream_packet_size      __PORT_RETAINED;
        bool stream_ignore_packet_size  __PORT_RETAINED;
#endif
 
#if !defined(CFG_APP_STREAM_PACKET_BASED) && defined(CFG_APP_STREAM_FIFO_PREDEFINED)
        #error "CFG_APP_STREAM_FIFO_PREDEFINED can only be used when CFG_APP_STREAM_PACKET_BASED is defined"
#endif
                
#if APP_STREAM_USE_PREDEFINED_BUFFERS
    #ifdef APP_STREAM_MULTIPLE_ARRAYS
        #if (APP_STREAM_MAX_PACKET_FIFO_LEN!=(APP_STREAM_MAX_PACKET_FIFO_LEN0+APP_STREAM_MAX_PACKET_FIFO_LEN1+APP_STREAM_MAX_PACKET_FIFO_LEN2))
            #error "FIX THE FIFO SIZES."
        #endif
        
        uint8_t pkts0 [APP_STREAM_MAX_PACKET_FIFO_LEN0][APP_STREAM_PACKET_SIZE] __PKTS0_SECTION;
        uint8_t pkts1 [APP_STREAM_MAX_PACKET_FIFO_LEN1][APP_STREAM_PACKET_SIZE] __PKTS1_SECTION;
        uint8_t pkts2 [APP_STREAM_MAX_PACKET_FIFO_LEN2][APP_STREAM_PACKET_SIZE] __PKTS2_SECTION;
    #else
    /* Pre-allocated Packets */
        uint8_t pkts[APP_STREAM_MAX_PACKET_FIFO_LEN][APP_STREAM_PACKET_SIZE];
    #endif
#endif

#if APP_STREAM_USE_CIRCULAR_BUFFER    
        uint8_t  stream_buffer[APP_STREAM_FIFO_SIZE];
#endif

typedef struct app_stream_fifo_s {
#ifdef CFG_APP_STREAM_PACKET_BASED
        app_stream_pkt_t pkt_fifo[APP_STREAM_MAX_PACKET_FIFO_LEN];
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
        uint8_t  fifo_read;
        uint8_t  fifo_write;
    #else
        uint8_t *buffer;
        uint16_t fifo_read;
        uint16_t fifo_write;
        uint8_t *requested_buffer_start;
        bool buffer_full;    
    #endif
#else    
        uint8_t *buffer;
        app_stream_pkt_t pkt;
        uint16_t fifo_read;
        uint16_t fifo_write;
        uint16_t requested_buffer_start;
        uint16_t fifo_end;
        bool buffer_full;
#endif    

} app_stream_fifo_t;

/**
 * Global Variable that holds the Steam Application FIFO data
 */
app_stream_fifo_t app_stream_fifo __PORT_RETAINED;

void app_stream_fifo_init(void)
{
        app_stream_fifo.fifo_read  = 0;
        app_stream_fifo.fifo_write = 0;
        app_stream_env.fifo_size   = 0;
    
#ifdef CFG_APP_STREAM_PACKET_BASED    
        for (int i=0; i<APP_STREAM_MAX_PACKET_FIFO_LEN; i++) {
            app_stream_fifo.pkt_fifo[i].used_hndl = 0;
        }
    
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
//        #ifndef APP_STREAM_MULTIPLE_ARRAYS
//        memset(pkts,0,sizeof(pkts));
//        #else
//        memset(pkts0, 0, sizeof(pkts0));
//        memset(pkts1, 0, sizeof(pkts1));
//        memset(pkts2, 0, sizeof(pkts2));
//        #endif
    #else
        app_stream_fifo.buffer_full = false;
    #endif
#else
        app_stream_fifo.buffer_full = false;
        app_stream_fifo.fifo_end = APP_STREAM_FIFO_SIZE;
#endif
}

/**
 ****************************************************************************************
 * \brief
 ****************************************************************************************
 */
static void app_stream_fifo_buffer_init(void)
{
#ifdef CFG_APP_STREAM_PACKET_BASED    
        memset(app_stream_fifo.pkt_fifo, 0, sizeof(app_stream_pkt_t) * APP_STREAM_MAX_PACKET_FIFO_LEN);
    
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
        #ifndef APP_STREAM_MULTIPLE_ARRAYS
        for (int i=0; i<APP_STREAM_MAX_PACKET_FIFO_LEN; i++) {
                app_stream_fifo.pkt_fifo[i].datapt = pkts[i];
//                app_stream_fifo.pkt_fifo[i].used_hndl = 0;
        }
        #else

        for (int i = 0; i < APP_STREAM_MAX_PACKET_FIFO_LEN0; i++) {
                app_stream_fifo.pkt_fifo[i].datapt = pkts0[i];
        }
        for (int i = 0; i < APP_STREAM_MAX_PACKET_FIFO_LEN1; i++) {
                app_stream_fifo.pkt_fifo[APP_STREAM_MAX_PACKET_FIFO_LEN0 + i].datapt = pkts1[i];
        }
        for (int i = 0; i < APP_STREAM_MAX_PACKET_FIFO_LEN2; i++) {
                app_stream_fifo.pkt_fifo[(APP_STREAM_MAX_PACKET_FIFO_LEN0 + APP_STREAM_MAX_PACKET_FIFO_LEN1) + i].datapt = pkts2[i];
        }
        #endif
    #else
        app_stream_fifo.buffer = stream_buffer;
        app_stream_fifo.pkt_fifo[0].datapt = app_stream_fifo.buffer;
    #endif    
#else
        app_stream_fifo.buffer = stream_buffer;
#endif
}

void app_stream_init(void)
{
        app_stream_env.stream_enabled = false;
        app_stream_fifo_buffer_init();
        app_stream_fifo_init(); 
#ifndef CFG_APP_STREAM_PACKET_BASED    
        if(stream_packet_size == 0) {
                stream_packet_size = 20;
        }
#endif        
}

void app_stream_start(void)
{
        if (app_stream_env.stream_enabled == false) {
                app_stream_env.stream_enabled = true;
                stream_first_packets = true;
#ifndef CFG_APP_STREAM_PACKET_BASED           
                stream_ignore_packet_size = false;
#endif            
        }
}

void app_stream_stop(void)
{
        app_stream_env.stream_enabled = false;
        app_stream_fifo_init();             //drop all packages when you stop.
}

bool app_stream_is_enabled(void)
{
        return app_stream_env.stream_enabled;
}

static uint8_t *stream_fifo_get_write_dataptr(uint16_t size, bool high_priority)
{
        bool fifo_full = false;
        do {
#ifdef CFG_APP_STREAM_PACKET_BASED
                ASSERT_WARNING(high_priority == false); // priority not implemented for packet based FIFO
                if(app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].used_hndl != 0) {
                        fifo_full = true;
                        break;
                }
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
                ASSERT_ERROR(size == APP_STREAM_PACKET_SIZE);
    #else
                if(app_stream_fifo.buffer_full == true) {
                        fifo_full = true;
                        break;
                }
        
                uint8_t *read_ptr = app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_read].datapt;
                uint8_t *write_ptr = app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].datapt;
                
                if(write_ptr >= read_ptr && (write_ptr + size) > (app_stream_fifo.buffer + APP_STREAM_FIFO_SIZE)) {
            // If there is not enough space at the end of the FIFO then start from the beginning
                        if(read_ptr == app_stream_fifo.buffer) {
                                // There is no space in the FIFO
                                fifo_full = true;
                                break;
                        }
                        else {
                                write_ptr = app_stream_fifo.buffer;
                        }   
                }
                if((write_ptr < read_ptr) && ((read_ptr - write_ptr) < size)) {
                        fifo_full = true;
                }
                else {
                        app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].len = size;   
                        app_stream_fifo.requested_buffer_start = write_ptr;
                }
    #endif    
#else      
                if(high_priority == false) {
                    // Always leave 10 bytes available for high priority data such as inband commands
                    size += STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES;
                }
                else {
                    // high_priority is used but STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES is 0 or not defined
                    ASSERT_WARNING(STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES != 0); 
                }
                if(app_stream_fifo.buffer_full == true) {
                        fifo_full = true;
                        break;
                }
         
                uint16_t read_ptr = app_stream_fifo.fifo_read;
                uint16_t write_ptr = app_stream_fifo.fifo_write;
            
                if(write_ptr >= read_ptr && (write_ptr + size) > APP_STREAM_FIFO_SIZE) {
                    // If there is not enough space at the end of the FIFO then start from the beginning
                        if(read_ptr == 0) {
                                // There is no space in the FIFO
                                fifo_full = true;
                                break;
                        }
                        else {
                                write_ptr = 0;
                        }
                }
                if(( write_ptr < read_ptr) && ((read_ptr - write_ptr) < size)) {
                        fifo_full = true;
                }
                else {
                        app_stream_fifo.requested_buffer_start = write_ptr;
                }
#endif       
        } while(0);
        
        if(fifo_full == true) {
                return NULL;
#ifdef STREAM_UART_DEBUG
                putchar('U');
#endif
        }

#ifdef CFG_APP_STREAM_PACKET_BASED
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
        return app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].datapt;
    #else
        return app_stream_fifo.requested_buffer_start;
    #endif
#else        
        return &app_stream_fifo.buffer[app_stream_fifo.requested_buffer_start];
#endif    
}

uint8_t *app_stream_fifo_get_write_dataptr(uint16_t size)
{
        return stream_fifo_get_write_dataptr(size, false);
}

uint8_t *app_stream_fifo_get_priority_write_dataptr(uint16_t size)
{
        return stream_fifo_get_write_dataptr(size, true);
}

void app_stream_fifo_commit_write_pkt(uint16_t num_of_bytes, uint16_t handle)
{
//#ifdef APP_STREAM_OVERWRITE_PACKETS
//    #ifdef CFG_APP_STREAM_PACKET_BASED
//        if (app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].used_hndl != 0) {
//                /* Packet already in use so fifo overflows... do not commit packet, just skip..*/
//            return;
//        }
//    #else
//        ASSERT_ERROR(0);
//    #endif
//#endif
   
#ifdef CFG_APP_STREAM_PACKET_BASED
        /* 
        ** Put the HANDLE number in the used_hndl field. If used_hndl != 0, then packet is used.
        ** Otherwise, the used_hndl field is the packet number, which will be used later to 
        ** obtain the correct Handle number.
        */        
        app_stream_env.fifo_size++;
        app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].used_hndl = handle;
    #ifndef CFG_APP_STREAM_FIFO_PREDEFINED
        uint16_t length = app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].len;
        app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].datapt = app_stream_fifo.requested_buffer_start;
    #endif  
        
        app_stream_fifo.fifo_write++;
        if (app_stream_fifo.fifo_write >= APP_STREAM_MAX_PACKET_FIFO_LEN) {
                app_stream_fifo.fifo_write = 0;
        }
        
    #ifndef CFG_APP_STREAM_FIFO_PREDEFINED
        uint8_t *new_buffer = app_stream_fifo.requested_buffer_start + length;
        if(new_buffer == app_stream_fifo.buffer + APP_STREAM_FIFO_SIZE) {
                new_buffer = app_stream_fifo.buffer;
        }
        app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].datapt = new_buffer; 
        app_stream_fifo.buffer_full = (app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_write].datapt == app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_read].datapt);    
    #endif
#else
        if(app_stream_fifo.requested_buffer_start == 0 && app_stream_fifo.fifo_write != 0) {
                // If the requested buffer size could not fit at the end of the FIFO buffer then set fifo_end to the 
                // valid data length in the FIFO
                app_stream_fifo.fifo_end = app_stream_fifo.fifo_write;    
                app_stream_fifo.fifo_write = app_stream_fifo.requested_buffer_start;
        }

        app_stream_fifo.fifo_write += num_of_bytes;
        app_stream_env.fifo_size += num_of_bytes; 
        
        if (app_stream_fifo.fifo_write == APP_STREAM_FIFO_SIZE) {
                app_stream_fifo.fifo_write = 0;
                app_stream_fifo.fifo_end = APP_STREAM_FIFO_SIZE;
        }
        app_stream_fifo.buffer_full = (app_stream_fifo.fifo_write == app_stream_fifo.fifo_read);    
        app_stream_fifo.pkt.used_hndl = handle;
#endif  
}

void app_stream_fifo_commit_debug_pkt(uint16_t handle, uint8_t offset, uint16_t size)
{
#ifdef CFG_APP_STREAM_PACKET_BASED    
    #ifdef ADD_PACKET_DEBUG
        /* Add some debugging to this packet  */
        uint8_t *pkt = app_stream_fifo_get_next_dataptr(size);
        pkt[offset] = (uint8_t)app_stream_env.fifo_size;
        pkt[offset+1] = (uint8_t)app_stream_fifo.fifo_write;
        pkt[offset+2] = (uint8_t)app_stream_fifo.fifo_read;
        int i;
        pkt += offset+3;
        uint8_t vuse = app_stream_fifo.pkt_fifo[0].used_hndl;
        int idx = 0;
        for (i=0;i<APP_STREAM_MAX_PACKET_FIFO_LEN;i++) {
                if ( ((app_stream_fifo.pkt_fifo[i].used_hndl != 0) && (vuse == 0))
                        || ((app_stream_fifo.pkt_fifo[i].used_hndl == 0) && (vuse != 0)))
                {
                        vuse = app_stream_fifo.pkt_fifo[i].used_hndl;
                        *pkt++ = (uint8_t)i;
                        *pkt++ = vuse;
                        idx++;
                        if (idx > 5) break; // cannot store more ..
                }
        }
    #endif
    
        app_stream_fifo_commit_write_pkt(size, handle);
#else
        ASSERT_ERROR(0);
#endif        
}

uint8_t app_stream_get_fifo_usage(void)
{
#ifdef CFG_APP_STREAM_PACKET_BASED    
        return app_stream_env.fifo_size*100/APP_STREAM_MAX_PACKET_FIFO_LEN;
#else
        return app_stream_env.fifo_size*100/APP_STREAM_FIFO_SIZE;
#endif    
}

bool app_stream_is_fifo_empty(void)
{
#ifdef CFG_APP_STREAM_PACKET_BASED
        return app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_read].used_hndl == 0;
#else
        return app_stream_fifo.buffer_full == false && app_stream_fifo.fifo_read == app_stream_fifo.fifo_write;
#endif    
}

app_stream_pkt_t *app_stream_get_data(uint16_t max_length) 
{
        if(app_stream_is_fifo_empty()) {
		        return NULL;
	    }
        
#ifdef CFG_APP_STREAM_PACKET_BASED
        return &app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_read];
#else
        app_stream_fifo.pkt.wrap_around = false;
        if(app_stream_fifo.fifo_read < app_stream_fifo.fifo_write) {
                app_stream_fifo.pkt.len = co_min(app_stream_fifo.fifo_write - app_stream_fifo.fifo_read, max_length);
        }
        else {
                uint16_t bytes_remaining_to_end_of_buffer = app_stream_fifo.fifo_end - app_stream_fifo.fifo_read;
                if(bytes_remaining_to_end_of_buffer >= max_length) {
                        app_stream_fifo.pkt.len = max_length;
                }
                else {
                        app_stream_fifo.pkt.len = co_min(max_length, app_stream_fifo.fifo_write + bytes_remaining_to_end_of_buffer);
                        app_stream_fifo.pkt.wrap_around = true;
                }
        }

        ASSERT_ERROR(app_stream_fifo.pkt.len <= app_stream_env.fifo_size);

        app_stream_fifo.pkt.datapt = &app_stream_fifo.buffer[app_stream_fifo.fifo_read];
                
        return &app_stream_fifo.pkt;
#endif    
}

void app_stream_copy_pkt_data(uint8_t *dest, app_stream_pkt_t *src_pkt)
{ 
#ifdef CFG_APP_STREAM_PACKET_BASED
        uint16_t length;
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
        length = APP_STREAM_PACKET_SIZE;
    #else
        length = src_pkt->len;
    #endif
        memcpy(dest, src_pkt->datapt, length);
#else
        if(src_pkt->wrap_around == true) {
                uint16_t bytes_remaining_to_end_of_buffer = app_stream_fifo.fifo_end - (src_pkt->datapt - app_stream_fifo.buffer);
                // copy bytes until the end of the buffer
                memcpy(dest, src_pkt->datapt, bytes_remaining_to_end_of_buffer);
                // copy remaining bytes for the beginning of the buffer
                memcpy(dest+bytes_remaining_to_end_of_buffer, app_stream_fifo.buffer, src_pkt->len - bytes_remaining_to_end_of_buffer);
            
	    }
        else {
                memcpy(dest, src_pkt->datapt, src_pkt->len);
        }

#endif    
}

void app_stream_free_packet(void)
{
#ifdef CFG_APP_STREAM_PACKET_BASED
        // Set used_hndl to 0, to denote that packet is available
        app_stream_fifo.pkt_fifo[app_stream_fifo.fifo_read].used_hndl = 0;
        app_stream_fifo.fifo_read++;
        if (app_stream_fifo.fifo_read >= APP_STREAM_MAX_PACKET_FIFO_LEN) {
                app_stream_fifo.fifo_read = 0;
        }
        app_stream_env.fifo_size--;
#else    
        app_stream_fifo.fifo_read += app_stream_fifo.pkt.len;
        if(app_stream_fifo.pkt.wrap_around == true) {
                app_stream_fifo.fifo_read += (APP_STREAM_FIFO_SIZE - app_stream_fifo.fifo_end);            
        }
        
        if(app_stream_fifo.fifo_read >= APP_STREAM_FIFO_SIZE) {
                app_stream_fifo.fifo_read -= APP_STREAM_FIFO_SIZE;
        }
        
        app_stream_env.fifo_size -= app_stream_fifo.pkt.len;
#endif        
#if APP_STREAM_USE_CIRCULAR_BUFFER
        app_stream_fifo.buffer_full = false;
#endif        
}

bool app_stream_queue_data(void)
{       
        if(app_stream_env.stream_enabled == true) {
                return APP_STREAM_QUEUE_DATA_CALLBACK() != STREAM_NO_MORE_DATA;
        }
        else {
                return false;
        }
}

#ifndef CFG_APP_STREAM_PACKET_BASED
void app_stream_set_packet_size(uint8_t packet_size)
{
        stream_packet_size = packet_size;
}

void app_stream_ignore_packet_size(void)
{
        stream_ignore_packet_size = true;
}
#endif

#endif // HAS_BLE_STREAM

/**
 * \}
 * \}
 * \}
 */
