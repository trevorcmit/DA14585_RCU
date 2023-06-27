/*****************************************************************************************
 *
 * \file app_stream_config.h
 *
 * \brief BLE Stream module configuration file.
 * 
******************************************************************************************/

#ifndef _APP_STREAM_CONFIG_H_
#define _APP_STREAM_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup STREAM_CFG
 *
 * \brief BLE Stream module configuration
 * \{
******************************************************************************************/

/**
 ******************************************************************************
 * \brief Define CFG_APP_STREAM_PACKET_BASED to use a packet-based FIFO. In 
 * this case the FIFO consists of an array of packets. Otherwise a non-packet
 * based FIFO is used. In this case the FIFO is a circular buffer.
 ******************************************************************************
 */
#undef CFG_APP_STREAM_PACKET_BASED

/**
 ******************************************************************************
 * \brief Define CFG_APP_STREAM_FIFO_PREDEFINED to use pre-allocated buffers
 * for stream FIFO. The number of buffers is defined in 
 * APP_STREAM_MAX_PACKET_FIFO_LEN. The size of each buffer is defined 
 * APP_STREAM_PACKET_SIZE.
 ******************************************************************************
 */
#undef CFG_APP_STREAM_FIFO_PREDEFINED

#ifdef CFG_APP_STREAM_PACKET_BASED
   /**
    ******************************************************************************
    * \brief The maximum number of packets in the packet based FIFO
    ******************************************************************************
    */
    #define APP_STREAM_MAX_PACKET_FIFO_LEN        45

   /**
    ******************************************************************************
    * \brief If APP_STREAM_MULTIPLE_ARRAYS is defined then stream FIFO is split
    * in multiple arrays to enable more flexible placement in the RAM.
    ******************************************************************************
    */
    #define APP_STREAM_MULTIPLE_ARRAYS
    
    #ifdef APP_STREAM_MULTIPLE_ARRAYS
       /**
        **************************************************************************
        * \brief The maximum number of packets of each FIFO array
        **************************************************************************
        */
        #define APP_STREAM_MAX_PACKET_FIFO_LEN0   15
        #define APP_STREAM_MAX_PACKET_FIFO_LEN1   15
        #define APP_STREAM_MAX_PACKET_FIFO_LEN2   15
    #endif
    #ifdef CFG_APP_STREAM_FIFO_PREDEFINED
       /**
        **************************************************************************
        * \brief The size of each packet when FIFO is using pre-allocated buffers
        **************************************************************************
        */
        #define APP_STREAM_PACKET_SIZE 20
    #else
       /**
        **************************************************************************
        * \brief The size of the buffer used for packets when FIFO is not using
        * re-allocated buffers.
        **************************************************************************
        */
        #define APP_STREAM_FIFO_SIZE   900
    #endif
#else
   /**
    ******************************************************************************
    * \brief The size of the buffer of the non-packet based FIFO
    ******************************************************************************
    */
    #define APP_STREAM_FIFO_SIZE 1024
    
   /**
    ******************************************************************************
    * \brief Define the number of bytes left available in the steam FIFO by 
    * app_stream_fifo_get_write_dataptr(). These bytes can be used by 
    * app_stream_fifo_get_priority_write_dataptr() to ensure that high priority 
    * data are added in the stram FIFO.
    ******************************************************************************
    */
    #define STREAM_FIFO_NUM_OF_HIGH_PRIORITY_BYTES 10    
#endif

#define APP_STREAM_QUEUE_DATA_CALLBACK port_stream_queue_data

/**
 * \}
 * \}
 * \}
 */

#endif // _APP_STREAM_CONFIG_H_
