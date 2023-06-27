/*****************************************************************************************
 *
 * \file app_motion_config.h
 *
 * \brief  Motion module configuration header file
 * 
******************************************************************************************/ 
 
#ifndef _APP_MOTION_CONFIG_H_
#define _APP_MOTION_CONFIG_H_

/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup MOTION_CFG
 *
 * \brief Motion module configuration
 * \{
******************************************************************************************/

#include <user_periph_setup.h>
#include "port_platform.h"
#include "app_motion_defs.h"

/*****************************************************************************************
 * Select the gyro/accelerometer sensor
******************************************************************************************/
#define BMI160
//#define BMI055

#ifdef BMI160
    #include <app_bmi160_config.h> // MOTION_IF is defined in this file
#endif

#ifdef BMI055
    #define I2C       0
    #define MOTION_IF I2C
#endif 

/*****************************************************************************************
 * \brief Set motion sensor rotation. Sensor can be rotated 0, 90, 180, or 270 degrees 
 *        clockwise as seen from the top
******************************************************************************************/
#define MOTION_ROTATION 90 

/*****************************************************************************************
 * \brief Set motion sensor PCB side placement. MOTION_PCB_BOTTOM is defined when sensor 
 *        is mounted on the bottom of the board
******************************************************************************************/
#undef MOTION_PCB_BOTTOM

/*****************************************************************************************
 * \brief Set the time that the sensor will remain active after app_motion_stop() has 
 *        been called
******************************************************************************************/
//#define MOTION_DEACTIVATION_TIMEOUT_IN_MS 100 // in msec

#if MOTION_IF == SPI
    /**
     ************************************************************************************
     * pin configuration                                                                 
     ************************************************************************************
     */
    enum motion_cs_pin_ids {
        MOTION_SPI_CS_PIN,
    };

    static const pin_type_t app_motion_cs_pin[] = {
        [MOTION_SPI_CS_PIN]  = {.port = GPIO_PORT_2, .pin = GPIO_PIN_9, .high = 1, .mode_function = INPUT_PULLUP | PID_GPIO },
    };

    enum motion_spi_pins_ids {
        MOTION_SPI_CLK_PIN,
        MOTION_SPI_DO_PIN,
        MOTION_SPI_DI_PIN
    };

    static const pin_type_t app_motion_spi_pins[] = {
        [MOTION_SPI_CLK_PIN] = {.port = SPI_CLK_PORT, .pin = SPI_CLK_PIN, .high = 0, .mode_function = INPUT_PULLUP | PID_GPIO },
        [MOTION_SPI_DO_PIN]  = {.port = SPI_DO_PORT,  .pin = SPI_DO_PIN,  .high = 0, .mode_function = INPUT_PULLUP | PID_GPIO },
        [MOTION_SPI_DI_PIN]  = {.port = SPI_DI_PORT,  .pin = SPI_DI_PIN,  .high = 0, .mode_function = INPUT_PULLUP | PID_GPIO },
    };

#endif

#include "port_motion.h"

static const motion_util_funcs_t app_motion_funcs = {
    .wakeup     = port_motion_wakeup,    
    .sleep      = port_motion_sleep,     
    .config     = port_motion_config,    
    .read       = port_motion_read,      
    .issue_bist = port_motion_issue_bist,
    .test_bist  = port_motion_test_bist, 
};

/**
 * \}
 * \}
 * \}
 */

#endif //_APP_MOTION_CONFIG_H_
