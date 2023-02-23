/**
 ****************************************************************************************
 *
 * \file port_stream.h
 *
 * \brief AudioStreamer Application entry point
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
 * \addtogroup STREAM
 * \{
 * \addtogroup PORT_STREAM
 * \{
 ****************************************************************************************	 
 */ 
 
#ifndef PORT_STREAM_H_
#define PORT_STREAM_H_

#define __PKTS0_SECTION __attribute__((section("pkts0_area"),zero_init))
#define __PKTS1_SECTION __attribute__((section("pkts1_area"),zero_init))
#define __PKTS2_SECTION __attribute__((section("pkts2_area"),zero_init))


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "arch.h"
#include "rwip_config.h"

#ifdef HAS_BLE_STREAM

/**
 ****************************************************************************************
 * \brief Send data from the FIFOs to L2CC
 *
 * \return
 ****************************************************************************************
 */
enum stream_status port_stream_queue_data(void);

#endif //HAS_BLE_STREAM


#endif // PORT_STREAM_H_

/**
 * \}
 * \}
 * \}
 */
