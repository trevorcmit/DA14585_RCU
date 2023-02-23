/*
 ****************************************************************************
 * Copyright (C) 2016 Bosch Sensortec GmbH
 *
 * bmi160_support.c
 * Date: 2016/06/22
 * Revision: 1.1.4 $
 *
 * Usage: Sensor Driver support file for BMI160 sensor
 *
 ****************************************************************************
 * License:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holder nor the names of the
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 **************************************************************************/

/*! \file bmi160_support.c
    \brief BMI160 Sensor Driver Support Header File */

/**
 ****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup BMI160_DRV
 *
 * \{
 ****************************************************************************************
 */

#include "bmi160_support.h"
#include "bmi160.h"
#if MOTION_IF == I2C
    #include "i2c_core.h"
#elif MOTION_IF == SPI
    #include "spi.h"
#endif
#include "port_platform.h"

/* Mapping the structure*/
struct bmi160_t s_bmi160;

/* Read the sensor data of accel, gyro and mag*/
struct bmi160_gyro_t gyroxyz;
struct bmi160_accel_t accelxyz;
struct bmi160_mag_xyz_s32_t magxyz;

static inline BMI160_RETURN_FUNCTION_TYPE bmi160_config_running_mode(
    u8 v_running_mode_u8) __attribute__((always_inline));
    
BMI160_RETURN_FUNCTION_TYPE bmi160_initialize_sensor(void)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE;
 
#ifdef INCLUDE_BMI160API
    /*	Based on the user need configure I2C or SPI interface.
     *	It is sample code to explain how to use the bmi160 API*/
    #if MOTION_IF == I2C
	com_rslt = i2c_routine();
    #elif MOTION_IF == SPI
    com_rslt = spi_routine();
    #endif
#endif //INCLUDE_BMI160API
    
    /*
     *  This function used to assign the value/reference of
     *	the following parameters
     *	I2C address
     *	Bus Write
     *	Bus read
     *	company_id
     */
	com_rslt += bmi160_init(&s_bmi160);

    /**** standard 9Dof with FIFO output****/
	com_rslt += bmi160_config_running_mode(APPLICATION_REMOTE_CONTROL);
	
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_config_running_mode(
    u8 v_running_mode_u8)
{
#if defined(INCLUDE_BMI160GYR) && defined(INCLUDE_BMI160PMU)
	struct gyro_sleep_setting gyr_setting;
#endif //defined(INCLUDE_BMI160GYR) && defined(INCLUDE_BMI160PMU)

#ifdef FIFO_ENABLE
    #ifdef INCLUDE_BMI160MAG
	struct bmi160_fifo_data_header_t header_data;
    #endif //INCLUDE_BMI160MAG
#endif //FIFO_ENABLE
    
#ifdef INCLUDE_BMI160MAG
	/* Variable used for get the status of mag interface*/
	u8 v_mag_interface_u8 = BMI160_INIT_VALUE;
	u8 v_bmm_chip_id_u8 = BMI160_INIT_VALUE;
#endif //INCLUDE_BMI160MAG
    
	BMI160_RETURN_FUNCTION_TYPE com_rslt = ERROR;

#if defined(INCLUDE_BMI160GYR) && defined(INCLUDE_BMI160PMU)
    /* Configure the gyro sleep setting based on your need*/
	if (v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE) {
		gyr_setting. sleep_trigger = BMI160_SLEEP_TRIGGER;
		gyr_setting. wakeup_trigger = BMI160_WAKEUP_TRIGGER;
		gyr_setting. sleep_state = BMI160_SLEEP_STATE;
		gyr_setting. wakeup_int = BMI160_WAKEUP_INTR;
	}
#endif //defined(INCLUDE_BMI160GYR) && defined(INCLUDE_BMI160PMU)

#ifdef INCLUDE_BMI160MAG
	/* The below code used for enable and
	disable the secondary mag interface*/
	com_rslt = bmi160_get_if_mode(&v_mag_interface_u8);
    
	if((v_running_mode_u8 == STANDARD_UI_IMU_FIFO ||
        v_running_mode_u8 == STANDARD_UI_IMU ||
        v_running_mode_u8 == STANDARD_UI_ADVANCEPOWERSAVE ||
        v_running_mode_u8 == APPLICATION_NAVIGATION ||
        v_running_mode_u8 == ACCEL_PEDOMETER ||
        v_running_mode_u8 == APPLICATION_REMOTE_CONTROL ||
        v_running_mode_u8 == APPLICATION_INDOOR_NAVIGATION) &&
       v_mag_interface_u8 == BMI160_MAG_INTERFACE_ON_PRIMARY_ON) {
		com_rslt += bmi160_set_bmm150_mag_and_secondary_if_power_mode(
                        MAG_SUSPEND_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_if_mode(BMI160_MAG_INTERFACE_OFF_PRIMARY_ON);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
       
	if((v_running_mode_u8 == STANDARD_UI_9DOF_FIFO ||
		v_running_mode_u8 == APPLICATION_HEAD_TRACKING ||
		v_running_mode_u8 == APPLICATION_NAVIGATION) &&
       v_mag_interface_u8 == BMI160_MAG_INTERFACE_OFF_PRIMARY_ON) {
        /* Init the magnetometer */
        com_rslt += bmi160_bmm150_mag_interface_init(&v_bmm_chip_id_u8);
        /* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
#endif //INCLUDE_BMI160MAG
    
	switch (v_running_mode_u8) {        
#ifdef FIFO_ENABLE
        
	case STANDARD_UI_9DOF_FIFO:
    #ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
    #endif //INCLUDE_BMI160ACC
    
    #ifdef INCLUDE_BMI160GYR
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
    #endif //INCLUDE_BMI160GYR
    
    #ifdef INCLUDE_BMI160ACC
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160ACC
		
    #ifdef INCLUDE_BMI160GYR
        /* Set the gyro bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		/* bmi160_delay_ms in ms*/        
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160GYR
		
    #ifdef INCLUDE_BMI160ACC
        /* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ, 
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160ACC
		
        /***** read FIFO data based on interrupt*****/
		com_rslt += bmi160_interrupt_configuration();
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
    #ifdef INCLUDE_BMI160MAG
        /* Enable the FIFO mag*/
		com_rslt += bmi160_set_fifo_mag_enable(FIFO_MAG_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160MAG
    
    #ifdef INCLUDE_BMI160ACC
        /* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160ACC
		
    #ifdef INCLUDE_BMI160GYR
        /* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160GYR
        
    #ifdef INCLUDE_BMI160TIM
		/* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160TIM

    #ifdef INCLUDE_BMI160INT
		/* Enable FIFO water mark interrupts in INT_EN[1] */
		com_rslt += bmi160_set_intr_enable_1(
                        BMI160_FIFO_WM_ENABLE,
                        BMI160_ENABLE);
                        
		/* Enable the FIFO water mark interrupt1*/
		com_rslt += bmi160_set_intr_fifo_wm(
                        BMI160_INIT_VALUE,
                        FIFO_WM_INTERRUPT_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_wm(
                        BMI160_ENABLE,
                        FIFO_WM_INTERRUPT_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160INT

		/* set the fifo water mark*/
		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);

    #ifdef INCLUDE_BMI160MAG
		/* read the FIFO data*/
		com_rslt +=  bmi160_read_fifo_header_data(
                        BMI160_SEC_IF_BMM150,
                        &header_data);
    #endif //INCLUDE_BMI160MAG
        break;
        
	case STANDARD_UI_IMU_FIFO:
    #ifdef INCLUDE_BMI160ACC
        com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
    #endif //INCLUDE_BMI160ACC
    
    #ifdef INCLUDE_BMI160GYR
        /*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
    #endif //INCLUDE_BMI160GYR
    
    #ifdef INCLUDE_BMI160ACC
        /* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		/* bmi160_delay_ms in ms*/        
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160ACC
    
    #ifdef INCLUDE_BMI160GYR
        /* Set the gyro bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160GYR
    
    #ifdef INCLUDE_BMI160ACC
        /* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160ACC
    
        /***** read FIFO data based on interrupt*****/
		com_rslt += bmi160_interrupt_configuration();
		
        /* Enable the FIFO header*/
		com_rslt += bmi160_set_fifo_header_enable(FIFO_HEADER_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
    #ifdef INCLUDE_BMI160ACC
        /* Enable the FIFO accel*/
		com_rslt += bmi160_set_fifo_accel_enable(FIFO_ACCEL_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160ACC
		
    #ifdef INCLUDE_BMI160GYR
        /* Enable the FIFO gyro*/
		com_rslt += bmi160_set_fifo_gyro_enable(FIFO_GYRO_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160GYR
		
    #ifdef INCLUDE_BMI160TIM
        /* Enable the FIFO time*/
		com_rslt += bmi160_set_fifo_time_enable(FIFO_TIME_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160TIM

    #ifdef INCLUDE_BMI160INT
		/* Enable FIFO water mark interrupts in INT_EN[1] */
		com_rslt += bmi160_set_intr_enable_1(
                        BMI160_FIFO_WM_ENABLE,
                        BMI160_ENABLE);
		
        /* Enable the FIFO water mark interrupt1*/
		com_rslt += bmi160_set_intr_fifo_wm(
                        BMI160_INIT_VALUE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable the FIFO water mark interrupt2*/
		com_rslt += bmi160_set_intr_fifo_wm(
                        BMI160_ENABLE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160INT
    
		/* set the fifo water mark as 10*/
		com_rslt += bmi160_set_fifo_wm(BMI160_ENABLE_FIFO_WM);

    #ifdef INCLUDE_BMI160MAG
		/* read the FIFO data*/
		com_rslt += bmi160_read_fifo_header_data(
                        BMI160_SEC_IF_BMM150,
                        &header_data);
    #endif //INCLUDE_BMI160MAG
        break;

#endif //FIFO_ENABLE

	case STANDARD_UI_IMU:
#ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /* Set the gyro bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
#endif //INCLUDE_BMI160ACC
	
        break;
	
    case STANDARD_UI_ADVANCEPOWERSAVE:
#ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
		
        /* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /* Set the gyro bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 100Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_100HZ);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* set accel data rate as 100Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
    
#ifdef INCLUDE_BMI160INT
		/* Enable any motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_0(
                        BMI160_ANY_MOTION_X_ENABLE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable any motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_0(
                        BMI160_ANY_MOTION_Y_ENABLE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable any motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_0(
                        BMI160_ANY_MOTION_Z_ENABLE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable no motion interrupt - x axis*/
		com_rslt += bmi160_set_intr_enable_2(
                        BMI160_NOMOTION_X_ENABLE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable no motion interrupt - y axis*/
		com_rslt += bmi160_set_intr_enable_2(
                        BMI160_NOMOTION_Y_ENABLE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* Enable no motion interrupt - z axis*/
		com_rslt += bmi160_set_intr_enable_2(
                        BMI160_NOMOTION_Z_ENABLE,
                        BMI160_ENABLE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160INT

#ifdef INCLUDE_BMI160GYR
    #ifdef INCLUDE_BMI160PMU
        /* set the gyro sleep trigger*/
		com_rslt += bmi160_set_gyro_sleep_trigger(gyr_setting.sleep_trigger);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the gyro wakeup trigger*/
		com_rslt += bmi160_set_gyro_wakeup_trigger(gyr_setting.wakeup_trigger);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the gyro sleep state*/
		com_rslt += bmi160_set_gyro_sleep_state(gyr_setting.sleep_state);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the gyro wakeup interrupt*/
		com_rslt += bmi160_set_gyro_wakeup_intr(gyr_setting.wakeup_int);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    #endif //INCLUDE_BMI160PMU
    
        /* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
		/* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
#endif //INCLUDE_BMI160ACC
	
        break;
	
    case ACCEL_PEDOMETER:
#ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_LOWPOWER);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160ACC
    
#ifdef INCLUDE_BMI160ACC
		/*Set the gyro mode as SUSPEND write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_SUSPEND);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160GYR
    
#ifdef INCLUDE_BMI160ACC
		/* Set the accel bandwidth as OSR4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set accel data rate as 25Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* 10 not available*/
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
#endif //INCLUDE_BMI160ACC
	
        break;
        
	case APPLICATION_HEAD_TRACKING:
#ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160ACC
    
#ifdef INCLUDE_BMI160GYR
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160GYR
    
#ifdef INCLUDE_BMI160ACC
		/* Set the accel bandwidth as Normal */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_NORMAL_AVG4);
		/* bmi160_delay_ms in ms*/
    s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
    
#ifdef INCLUDE_BMI160GYR
        /* Set the gyro bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 1600Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_1600HZ);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
		/* set accel data rate as 1600Hz*/
        com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160GYR
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160MAG
		/* read mag data */
		com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);
#endif //INCLUDE_BMI160MAG
	
        break;
	
    case APPLICATION_NAVIGATION:
#ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160ACC
    
#ifdef INCLUDE_BMI160GYR
		/*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
		/* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
    
#ifdef INCLUDE_BMI160GYR
		/* Set the gryo bandwidth as Normal */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
		/* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160GYR
		/* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
		/* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160MAG
		/* read mag data*/
		com_rslt += bmi160_bmm150_mag_compensate_xyz(&magxyz);
#endif //INCLUDE_BMI160MAG
	
        break;
        
	case APPLICATION_REMOTE_CONTROL:
#ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
        /* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
		s_bmi160.delay_msec(BMI160_MODE_SWITCHING_DELAY);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /* Set the gyro bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /* read gyro data */
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* read accel data*/
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
#endif //INCLUDE_BMI160ACC
	
        break;
	
    case APPLICATION_INDOOR_NAVIGATION:
#ifdef INCLUDE_BMI160ACC
		/*Set the accel mode as Normal write in the register 0x7E*/
		com_rslt = bmi160_set_command_register(ACCEL_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /*Set the gyro mode as Normal write in the register 0x7E*/
		com_rslt += bmi160_set_command_register(GYRO_MODE_NORMAL);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* Set the accel bandwidth as OSRS4 */
		com_rslt += bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /* Set the gyro bandwidth as OSR4 */
		com_rslt += bmi160_set_gyro_bw(BMI160_GYRO_OSR4_MODE);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set gyro data rate as 200Hz*/
		com_rslt += bmi160_set_gyro_output_data_rate(
                        BMI160_GYRO_OUTPUT_DATA_RATE_400HZ);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* set accel data rate as 200Hz*/
		com_rslt += bmi160_set_accel_output_data_rate(
                        BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ,
                        BMI160_ACCEL_OSR4_AVG1);
		/* bmi160_delay_ms in ms*/
        s_bmi160.delay_msec(BMI160_GEN_READ_WRITE_DELAY);
#endif //INCLUDE_BMI160ACC
		
#ifdef INCLUDE_BMI160GYR
        /* read gyro data*/
		com_rslt += bmi160_read_gyro_xyz(&gyroxyz);
#endif //INCLUDE_BMI160GYR
		
#ifdef INCLUDE_BMI160ACC
        /* read accel data */
		com_rslt += bmi160_read_accel_xyz(&accelxyz);
#endif //INCLUDE_BMI160ACC
		
        break;
	}

	return com_rslt;
}

#ifdef FIFO_ENABLE
BMI160_RETURN_FUNCTION_TYPE bmi160_interrupt_configuration(void)
{
	/* Configure the in/out control of interrupt1*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_set_output_enable(BMI160_INIT_VALUE, BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

	/* Configure the in/out control of interrupt2*/
	com_rslt += bmi160_set_output_enable(BMI160_ENABLE, BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* Configure the interrupt1 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_INIT_VALUE, BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* Configure the interrupt2 active high
	0x00 -	Active low
	0x01 -	Active high*/
	com_rslt += bmi160_set_intr_level(BMI160_ENABLE, BMI160_ENABLE);
	s_bmi160.delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    return com_rslt;
}
#endif //FIFO_ENABLE

#ifdef INCLUDE_BMI160API
    #define MASK_DATA2	0x80

    #if MOTION_IF == I2C
s8 i2c_routine(void)
{
    /*--------------------------------------------------------------------------*
     *  By using bmi160 the following structure parameter can be accessed
     *	Bus write function pointer: BMI160_WR_FUNC_PTR
     *	Bus read function pointer: BMI160_RD_FUNC_PTR
     *	bmi160_delay_ms function pointer: bmi160_delay_ms_msec
     *	I2C address: dev_addr
     *--------------------------------------------------------------------------*/
	s_bmi160.bus_write = bmi160_i2c_bus_write;
	s_bmi160.bus_read = bmi160_i2c_bus_read;
	s_bmi160.delay_msec = bmi160_delay_ms;
	s_bmi160.dev_addr = BMI160_I2C_ADDR2;

	return BMI160_INIT_VALUE;
}
    #elif MOTION_IF == SPI
s8 spi_routine(void)
{
    /*--------------------------------------------------------------------------*
     *  By using bmi160 the following structure parameter can be accessed
     *	Bus write function pointer: BMI160_WR_FUNC_PTR
     *	Bus read function pointer: BMI160_RD_FUNC_PTR
     *	bmi160_delay_ms function pointer: bmi160_delay_ms_msec
     *--------------------------------------------------------------------------*/
	s_bmi160.bus_write = bmi160_spi_bus_write;
	s_bmi160.bus_read = bmi160_spi_bus_read;
	s_bmi160.delay_msec = bmi160_delay_ms;

	return BMI160_INIT_VALUE;
}
    #endif //MOTION_IF

/*-------------------------------------------------------------------*
*
*	This is a sample code for read and write the data by using I2C/SPI
*	Use either I2C or SPI based on your need
*	Configure the below code to your SPI or I2C driver
*
*-----------------------------------------------------------------------*/
    #if MOTION_IF == I2C
s8 bmi160_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return i2c_read_data(reg_data, reg_addr, cnt) < cnt ? -1
                                                        : 0;
}

s8 bmi160_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return i2c_write_data(reg_data, reg_addr, cnt) < cnt ? -1
                                                         : 0;
}
    #elif MOTION_IF == SPI
        #define MAX_READY_WAIT_COUNT    2000000
        #define ERR_OK                  0
        #define ERR_TIMEOUT             -1

/**
 ****************************************************************************************
 * @brief Wait till flash is ready for next action
 * @return  Success : ERR_OK
 *          Failure : ERR_TIMEOUT
 ****************************************************************************************
 */
static int8_t spi_wait_till_ready(void)
{
    for (uint32_t readCount = 0; readCount < MAX_READY_WAIT_COUNT; readCount++) {
        if (!GetBits16(SPI_CTRL_REG1, SPI_BUSY)){
            return ERR_OK;
        }
    }
    return ERR_TIMEOUT;
}

/**
 ****************************************************************************************
 * @brief Read data from a given starting address
 *
 * @param[in] *rd_data_ptr:  Points to the position the read data will be stored
 * @param[in] address:       Starting address of data to be read
 * @param[in] size:          Size of the data to be read
 *
 * @return  Number of read bytes or error code
 ****************************************************************************************
 */
static int32_t spi_read_data(uint8_t *rd_data_ptr, uint32_t address, uint32_t size)
{
    if (spi_wait_till_ready() != ERR_OK) {
        return ERR_TIMEOUT;                             // an error has occured
    }
    
    spi_set_bitmode(SPI_MODE_8BIT);
    spi_cs_low();                                       // pull CS low
    spi_access(MASK_DATA2 | address);                   // Command for sequencial reading from memory
    for (uint32_t i=0; i<size; i++) {
        *rd_data_ptr++ = (uint8_t)spi_access(0x0000);   // bare SPI transaction
    }
    spi_cs_high();                                      // push CS high
    
    return size;
}

s8 bmi160_spi_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return spi_read_data(reg_data, reg_addr, cnt) < cnt ? -1
                                                        : 0;
}

/**
 ****************************************************************************************
 * @brief Write data to any starting address
 *
 * @param[in] *wr_data_ptr:  Pointer to the data to be written
 * @param[in] address:       Starting address of data to be written
 * @param[in] size:          Size of the data to be written
 *
 * @return  Number of bytes actually written
 ****************************************************************************************
 */
static int32_t spi_write_data(uint8_t *wr_data_ptr, uint32_t address, uint32_t size)
{
    spi_set_bitmode(SPI_MODE_8BIT);
    if (spi_wait_till_ready() != ERR_OK) {
        return ERR_TIMEOUT;                 // an error has occured
    }    
    spi_cs_low();                           // pull CS low
    spi_access(address);                    // Command for sequencial writing to memory
    for (uint32_t i=0; i<size; i++) {
        spi_access(*wr_data_ptr++);         // Write data bytes
    }
    spi_cs_high();                          // push CS high
    if (spi_wait_till_ready() != ERR_OK) {
        return 0;                           // an error has occured
    }    
    return size;
}

s8 bmi160_spi_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return spi_write_data(reg_data, reg_addr, cnt) < cnt ? -1
                                                         : 0;
}
    #endif //MOTION_IF
#endif //INCLUDE_BMI160API

void bmi160_delay_ms(u32 msec)
{
    port_delay_usec(1000*msec);
}

/**
 * \}
 * \}
 * \}
 */
