/**
 ****************************************************************************************
 *
 * \file port_motion.h
 *
 * \brief motion interface header file.
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
 * \addtogroup MOTION
 * \{
 * \addtogroup PORT_MOTION
 * \{
 ****************************************************************************************	 
 */

#ifndef _PORT_MOTION_H
#   define _PORT_MOTION_H

#   include <stdint.h>
#   include "gpio.h"

#if defined(MOTION_IF) && (MOTION_IF == I2C)
    #ifndef HAS_I2C
        #define HAS_I2C
    #endif
#endif

#if defined(MOTION_IF) && (MOTION_IF == SPI)
    #ifndef HAS_SPI
        #define HAS_SPI
    #endif
    
/**
 ****************************************************************************************
 * \brief
 ****************************************************************************************
 */
void port_motion_declare_spi_cs_gpio(void);

/**
 ****************************************************************************************
 * \brief
 ****************************************************************************************
 */
void port_motion_declare_spi_gpios(void);

/**
 ****************************************************************************************
 * \brief Activate SPI motion GPIOs
 ****************************************************************************************
 */
void activate_spi_motion_gpios(void);

/**
 ****************************************************************************************
 * \brief Deactivate SPI motion GPIOs
 ****************************************************************************************
 */
void deactivate_spi_motion_gpios(void);

/**
 ****************************************************************************************
 * \brief
 ****************************************************************************************
 */
void deactivate_spi_motion_cs(void);

#endif // MOTION_IF == SPI


/**
 ****************************************************************************************
 * \brief Reads temperature, acceleration and rotation data
 *
 * \param[out] temp Pointer to the temperature variable
 * \param[out] acc  Pointer to the first element of the acceleration array
 * \param[out] rot  Pointer to the first element of the rotation array
 *
 * \return SUCCESS is defined as 0 and FAILURE as -1
 ****************************************************************************************
 */
int8_t port_motion_read(int16_t* temp, int16_t* acc, int16_t* rot);

/**
 ****************************************************************************************
 * \brief
 ****************************************************************************************
 */
void port_motion_issue_bist(void);

/**
 ****************************************************************************************
 * \brief
 *
 * \return true if BIST was successful
 ****************************************************************************************
 */
bool port_motion_test_bist(void);

/**
 ****************************************************************************************
 * \brief
 ****************************************************************************************
 */
void port_motion_config(void);

/**
 ****************************************************************************************
 * \brief
 *
 * \return SUCCESS is defined as 0 and FAILURE as a negative value
 ****************************************************************************************
 */
int8_t port_motion_sleep(void);

/**
 ****************************************************************************************
 * \brief
 ****************************************************************************************
 */
void port_motion_wakeup(void);

#endif // _PORT_MOTION_H

/**
 * \}
 * \}
 * \}
 */
