/******************************************************************************************
 * \file port_audio.h
 * \brief Platform specific AudioStreamer Application entry point
******************************************************************************************/

/******************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup AUDIO
 * \{
 * \addtogroup PORT_AUDIO
 * \brief Audio HAL implementation
 * \{
*****************************************************************************************/
 
#ifndef PORT_AUDIO_H_
#define PORT_AUDIO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "app_audio.h"

/*****************************************************************************************
 * \brief
*****************************************************************************************/
void declare_audio_gpios(void);


/*****************************************************************************************
 * \brief
*****************************************************************************************/
void init_audio_gpios(void);


/*****************************************************************************************
 * \brief Proceed to the next packet in the FIFO
*****************************************************************************************/
void port_audio_next_package(void);


/*****************************************************************************************
 * \brief Get the data packet from the FIFO head
 *
 * \return Pointer the the  data packet in the FIFO head
*****************************************************************************************/
void *port_audio_get_data(void);


/*****************************************************************************************
 * \brief Check if there is a data packet in the FIFO
 *
 * \return true if there is a data packet available, false if not
*****************************************************************************************/
bool port_audio_has_data(void);

/*****************************************************************************************
 * \brief Get the number of used samples positions in the audio buffer
 *
 * \return number of used positions
*****************************************************************************************/
uint16_t port_audio_buffer_get_used(void);

/*****************************************************************************************
 * \brief Get the number of errors caused by FIFO overrun
 *
 * \return The number of errors
*****************************************************************************************/
uint8_t port_audio_get_errors(void);

/*****************************************************************************************
 * \brief Start audio sampling and buffering
 *
 * \param[in] sampling_rate The sampling rate, either SAMPLING_RATE_16KHz or
 *                          SAMPLING_RATE_8KHz
*****************************************************************************************/
void port_audio_start_sampling(audio_sampling_rate_t sampling_rate);

/*****************************************************************************************
 * \brief Stop audio sampling and buffering
*****************************************************************************************/
void port_audio_stop_sampling(void);

#endif /* PORT_AUDIO_H_ */

/**
 * \}
 * \}
 * \}
 */
