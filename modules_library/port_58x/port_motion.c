/*****************************************************************************************
 *
 * \file port_motion.c
 *
 * \brief motion module platform adaptation source file
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
 * \addtogroup MOTION
 * \{
 * \addtogroup PORT_MOTION
 * \{
 ****************************************************************************************	 
 */

#include <app_motion_config.h>
#include "port_motion.h"

#if defined(HAS_MOTION) && defined(BMI160)
#   include "bmi160.h"
#   include "bmi160_support.h"
#   if MOTION_IF == I2C
#       include "i2c_core.h"
#   elif MOTION_IF == SPI
#       include "spi.h"
#   endif
#   include "port_platform.h"
#   include <user_periph_setup.h>

/*****************************************************************************************
 * \brief Initialize I2C controller as a master for BMI160 handling.
******************************************************************************************/
#   if MOTION_IF == I2C
#       define port_bmi160_init() \
            i2c_init(BMI160_I2C_ADDR1, I2C_SPEED_MODE, I2C_ADDRESS_MODE)
#   elif MOTION_IF == SPI
static SPI_Pad_t spi_MOTION_CS_Pad;

__INLINE void port_bmi160_init(void)
{
    //Initialize the SPI interface and power up the SPI flash if needed
    //in case it is shared with another device on different pins.
    // Enable SPI
    spi_MOTION_CS_Pad.pin = (GPIO_PIN)app_motion_cs_pin[MOTION_SPI_CS_PIN].pin; 
    spi_MOTION_CS_Pad.port = (GPIO_PORT)app_motion_cs_pin[MOTION_SPI_CS_PIN].port; 
    
    spi_init(&spi_MOTION_CS_Pad, SPI_MODE_8BIT, SPI_ROLE_MASTER, SPI_CLK_IDLE_POL_LOW,
             SPI_PHA_MODE_0, SPI_MINT_DISABLE, SPI_XTAL_DIV_2);
    
    activate_spi_motion_gpios();        
}
#   endif //MOTION_IF == SPI

void port_motion_wakeup(void)
{
    port_bmi160_init();

    bmi160_initialize_sensor();
    bmi160_set_accel_range(8); // Set accelerometer range to +/- 8g
}

/*****************************************************************************************
 * \brief
 *
 * \param[inout] data
******************************************************************************************/
static void port_bmi160_remap_axes(int16_t* data)
{
    if (!data) {
        return;
    }
    int16_t x = data[0],
            y = data[1];
#   ifndef MOTION_PCB_BOTTOM
#       if MOTION_ROTATION == 90
    data[0] = y;
    data[1] = -x;
#       elif MOTION_ROTATION == 180
    data[0] = -x;
    data[1] = -y;
#       elif MOTION_ROTATION == 270
    data[0] = -y;
    data[1] = x;
#       endif //MOTION_ROTATION == 270
#   else //MOTION_PCB_BOTTOM
#       if MOTION_ROTATION == 0
    data[0] = -x;
    data[1] = y;
#       elif MOTION_ROTATION == 90
    data[0] = -y;
    data[1] = -x;
#       elif MOTION_ROTATION == 180
    data[0] = x;
    data[1] = -y;
#       elif MOTION_ROTATION == 270
    data[0] = y;
    data[1] = x;
#       endif //MOTION_ROTATION == 270
    data[2] *= -1;
#   endif //MOTION_PCB_BOTTOM
}

int8_t port_motion_read(int16_t* temp, int16_t* acc, int16_t* rot)
{
    port_bmi160_init();
    
#   ifdef INCLUDE_BMI160ACC
    if (bmi160_read_accel_xyz((struct bmi160_accel_t*) acc)) {
        return -1;
    }
    port_bmi160_remap_axes(acc);
#   endif //INCLUDE_BMI160ACC
    
#   ifdef INCLUDE_BMI160TEM
    if (bmi160_get_temp(temp)) {
        return -1;
    }
#   endif //INCLUDE_BMI160TEM

#   ifdef INCLUDE_BMI160GYR    
    if (bmi160_read_gyro_xyz((struct bmi160_gyro_t*) rot)) {
        return -1;
    }
    port_bmi160_remap_axes(rot);
#   endif //INCLUDE_BMI160GYR
    
    return 0;
}

void port_motion_issue_bist(void)
{
    port_bmi160_init();
    
    bmi160_set_gyro_selftest_start(1);
}

bool port_motion_test_bist(void)
{
    port_bmi160_init();
    
    uint8_t gyr_self_test_ok;
    return !bmi160_get_gyro_selftest(&gyr_self_test_ok) && gyr_self_test_ok;    
}

void port_motion_config(void)
{
    port_bmi160_init();
}

int8_t port_motion_sleep(void)
{
    port_bmi160_init();
	BMI160_RETURN_FUNCTION_TYPE com_rslt = SUCCESS;
    
    com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
    port_delay_usec(BMI160_MODE_SWITCHING_DELAY*1000);

    com_rslt += bmi160_set_command_register(GYRO_MODE_SUSPEND);
    port_delay_usec(BMI160_MODE_SWITCHING_DELAY*1000);

    com_rslt += bmi160_set_command_register(MAG_MODE_SUSPEND);
    port_delay_usec(BMI160_MODE_SWITCHING_DELAY*1000);

    SetBits16(I2C_ENABLE_REG, CTRL_ENABLE, 0); // Disable the I2C controller	
    SetBits16(CLK_PER_REG,    I2C_ENABLE,  0); // Disable clock for I2C
    
#if MOTION_IF == SPI
    deactivate_spi_motion_gpios();
#endif

    return com_rslt;
}

#if MOTION_IF == SPI
void port_motion_declare_spi_cs_gpio(void)
{
    PORT_RESERVE_GPIO(app_motion_cs_pin[MOTION_SPI_CS_PIN]); 
}

void port_motion_declare_spi_gpios(void)
{
    PORT_RESERVE_GPIO(app_motion_cs_pin[MOTION_SPI_CS_PIN]); 
    PORT_RESERVE_GPIO(app_motion_spi_pins[MOTION_SPI_CLK_PIN]); 
    PORT_RESERVE_GPIO(app_motion_spi_pins[MOTION_SPI_DO_PIN]); 
    PORT_RESERVE_GPIO(app_motion_spi_pins[MOTION_SPI_DI_PIN]); 
}

void activate_spi_motion_gpios(void)
{
    PORT_CONFIGURE_GPIO(app_motion_cs_pin[MOTION_SPI_CS_PIN],  OUTPUT, PID_SPI_EN,  true );
    PORT_CONFIGURE_GPIO(app_motion_spi_pins[MOTION_SPI_CLK_PIN], OUTPUT, PID_SPI_CLK, false);
    PORT_CONFIGURE_GPIO(app_motion_spi_pins[MOTION_SPI_DO_PIN],  OUTPUT, PID_SPI_DO,  false);
    PORT_CONFIGURE_GPIO(app_motion_spi_pins[MOTION_SPI_DI_PIN],  INPUT,  PID_SPI_DI,  false);
}

void deactivate_spi_motion_gpios(void)
{
    PORT_SET_PIN_FUNCTION_DEFAULT(app_motion_cs_pin[MOTION_SPI_CS_PIN]);
    PORT_SET_PIN_FUNCTION_DEFAULT(app_motion_spi_pins[MOTION_SPI_CLK_PIN]); 
    PORT_SET_PIN_FUNCTION_DEFAULT(app_motion_spi_pins[MOTION_SPI_DO_PIN]); 
    PORT_SET_PIN_FUNCTION_DEFAULT(app_motion_spi_pins[MOTION_SPI_DI_PIN]); 
}

void deactivate_spi_motion_cs(void)
{
    PORT_SET_PIN_FUNCTION_DEFAULT(app_motion_cs_pin[MOTION_SPI_CS_PIN]);
}
#endif // MOTION_IF == SPI

#endif // defined(HAS_MOTION) && defined(BMI160)

/**
 * \}
 * \}
 * \}
 */
