/**
 ****************************************************************************************
 *
 * \file app_stream.h
 *
 * \brief This module provides an API for streaming data to a BLE host
 *
 * Define symbol HAS_BLE_STREAM to include this module in the application.
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

#ifndef APP_STREAM_H_
#define APP_STREAM_H_

#include <port_platform.h>

#ifdef HAS_BLE_STREAM

/*
 * APP_STREAM Env DataStructure
 ****************************************************************************************
 *
 * NOTE that number and size of payload packets is similar to number and size of 
 * notifications in StreamData profile. 
 * STREAMDATAD_PACKET_SIZE = 20, and STREAMDATAD_MAX=10
 */
 
#define APP_STREAM_USE_CIRCULAR_BUFFER    (!defined(CFG_APP_STREAM_PACKET_BASED) || !defined(CFG_APP_STREAM_FIFO_PREDEFINED))
#define APP_STREAM_USE_PREDEFINED_BUFFERS (defined(CFG_APP_STREAM_PACKET_BASED) && defined(CFG_APP_STREAM_FIFO_PREDEFINED))

#define MEMORY_OPTIMIZATION2

enum stream_status {
        STREAM_NO_MORE_DATA,
        STREAM_HID_FAIL,
        STREAM_MORE_DATA
};

typedef struct app_stream_pkt_s {
    uint8_t *datapt;
#if APP_STREAM_USE_CIRCULAR_BUFFER
    void   (*p_callback) (void* , int);
    uint16_t  len;
#endif
#ifndef CFG_APP_STREAM_PACKET_BASED
    bool wrap_around;
#endif    
    uint16_t  used_hndl;   // if 0, stream packet not used, else it contains the handle
} app_stream_pkt_t;

/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * \brief Initialize AudioStreamer Application 
 ****************************************************************************************
 */
void app_stream_init(void);

/**
 ****************************************************************************************
 * \brief Stream Fifo initialization. This function initializes the FIFO. 
 ****************************************************************************************
 */
void app_stream_fifo_init(void);

/**
 ****************************************************************************************
 * \brief Called after STREAM-ON was received.
 ****************************************************************************************
 */
void app_stream_start(void);

/**
 ****************************************************************************************
 * \brief Stop the stream. Called when STREAMOFF is received, or button released.
 ****************************************************************************************
 */
void app_stream_stop(void);

/**
 ****************************************************************************************
 * \brief Get the value of streamdata Enable value
 *        Called when STREAMOFF is received, or button released.
 *
 * \return      true if steam is enabled
 ****************************************************************************************
 */
bool app_stream_is_enabled(void);
                        
/**
 ****************************************************************************************
 * \brief Commit a packet in the FIFO
 *
 * Commit packet in stream_fifo. Call this function after you have updated the data
 * in the packet dataptr (get pointer with app_stream_fifo_get_next_pkt). This
 * function will then set the used flag to 1, and increment the Fifo Write index.
 *
 * \param[in]  num_of_bytes Number of bytes to commit in the FIFO
 * \param[in]  handle Tha handle of the characteristic
 ****************************************************************************************
 */
void app_stream_fifo_commit_write_pkt(uint16_t num_of_bytes, uint16_t handle);

/**
 ****************************************************************************************
 * \brief Get datapointer of the next packet available for writing in Stream FIFO. 
 *        Always leave at least STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES empty fro high
 *        priority data.
 *
 * \param  size of allocated buffer
 *
 * \return The datapointer
 ****************************************************************************************
 */
uint8_t *app_stream_fifo_get_write_dataptr(uint16_t size);

/**
 ****************************************************************************************
 * \brief Get datapointer of the next packet available for writing in Stream FIFO.
 *        STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES are always available in the FIFO for 
 *        this call. 
 *
 * \param  size of allocated buffer
 *
 * \return The datapointer
 ****************************************************************************************
 */
uint8_t *app_stream_fifo_get_priority_write_dataptr(uint16_t size);

/**
 ****************************************************************************************
 * \brief 
 *
 * \param  handle 
 * \param  offset 
 * \param  size 
 ****************************************************************************************
 */
void app_stream_fifo_commit_debug_pkt(uint16_t handle, uint8_t offset, uint16_t size);

/**
 ****************************************************************************************
 * \brief Get the FIFO utilization
 *
 * \return The percetage of the FIFO utilization
 ****************************************************************************************
 */
uint8_t app_stream_get_fifo_usage(void);

/**
 ****************************************************************************************
 * \brief Check if stream FIFO is empty
 *
 * \return True if FIFO is empty
 ****************************************************************************************
 */
bool app_stream_is_fifo_empty(void);

/**
 ****************************************************************************************
 * \brief Get the next available packet from the FIFO
 *
 * \param  length maximum number of data packet bytes
 *
 * \return Pointer to the pointer to the data packet. NULL is stream fifo is empty
 ****************************************************************************************
 */
app_stream_pkt_t *app_stream_get_data(uint16_t length);

/**
 ****************************************************************************************
 * \brief Copy data from stream FIFO to destination buffer. When a circular buffer is 
 *        used this function handles the case when data whaps around the end of the 
 *        buffer.
 *
 * \param  dest Pointer to the distination buffer
 * \param  src_pkt Pointer to the source packet
 ****************************************************************************************
 */
void app_stream_copy_pkt_data(uint8_t *dest, app_stream_pkt_t *src_pkt);

/**
 ****************************************************************************************
 * \brief Proceed to the next data packet
 ****************************************************************************************
 */
void app_stream_free_packet(void);

/**
 ****************************************************************************************
 * \brief Send data from the FIFOs to L2CC
 *
 * \return  true if the system sleep must be blocked
 ****************************************************************************************
 */
bool app_stream_queue_data(void);

#if APP_STREAM_USE_CIRCULAR_BUFFER

/**
 ****************************************************************************************
 * \brief Set the BLE data packet size when FIFO is not packet based. The streamer is 
 *        waiting for enough data to be available in the FIFO before sending a BLE
 *        packet.
 *
 * \param  packet_size The size of the packet
 ****************************************************************************************
 */
void app_stream_set_packet_size(uint8_t packet_size);

/**
 ****************************************************************************************
 * \brief Ignore the packet size set by app_stream_set_packet_size(). This function is 
 *        used to force the transmission of the remaining bytes in the FIFO when the
 *        stream is about to be stopped.
 ****************************************************************************************
 */
void app_stream_ignore_packet_size(void);
#endif

#endif //HAS_BLE_STREAM

#endif // APP_STREAM_H_

/**
 * \}
 * \}
 * \}
 */
