/*****************************************************************************************
 *
 * \file port_systick.h
 *
 * \brief Systick controller functions
 * 
******************************************************************************************/

/*****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup SYSTICK
 * \{
 * \addtogroup PORT_SYSTICK
 * \{
 ****************************************************************************************	 
 */ 
 
#ifndef PORT_SYSTICK_H_
    #define PORT_SYSTICK_H_

typedef void (*systick_callback_t)(void);

typedef struct systick_config_s {
    systick_callback_t callback;
} systick_config_t;

    #include <port_systick_config.h>

/*****************************************************************************************
 * \brief Initializes the SysTick platform       
******************************************************************************************/
void port_systick_init(void);

/*****************************************************************************************
 * \brief Starts the SysTick for a specific channel
 *
 * \param[in]  channel   The channel for which the SysTick should start
 * \param[in]  period    The period in usec of this SysTick channel
******************************************************************************************/
void port_systick_start(enum port_systick_channel channel, uint32_t period);

/*****************************************************************************************
 * \brief Stops and restarts the SysTick for a specific channel
 *
 * \param[in]  channel   The channel for which the SysTick should restart
 * \param[in]  period    The period in usec of this SysTick channel
******************************************************************************************/
void port_systick_restart(enum port_systick_channel channel, uint32_t period);

/*****************************************************************************************
 * \brief Stops the SysTick for a specific channel
 *
 * \param[in]  channel   The channel for which the SysTick should stop
******************************************************************************************/
void port_systick_stop(enum port_systick_channel channel);

#endif // PORT_SYSTICK_H_

/**
 * \}
 * \}
 * \}
 */
