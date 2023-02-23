/**
 ****************************************************************************************
 *
 * \file app_bmi160_config.h
 *
 * \brief  BMI160 sensor driver configuration
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 ****************************************************************************************
 */ 
 
#ifndef _APP_BMI160_CONFIG_H_
#   define _APP_BMI160_CONFIG_H_

#   define I2C 0
#   define SPI 1

#ifndef MOTION_IF
#   define MOTION_IF SPI
#endif

/* enable the macro for FIFO functionalities when FIFO is used */
#   define FIFO_ENABLE

#   define INCLUDE_BMI160API
//#   define INCLUDE_BMI160ERR //errors
//#   define INCLUDE_BMI160FOC //fast offset compensation
//#   define INCLUDE_BMI160GET //read status and configuration
#   define INCLUDE_BMI160INT //interrupts
#   define INCLUDE_BMI160PMU //power management unit 

//#   define INCLUDE_BMI160PED //pedometer
//#   define INCLUDE_BMI160TIM //sensor time
#   define INCLUDE_BMI160TEM //temperature

#   define INCLUDE_BMI160ACC //accelerometer
#   define INCLUDE_BMI160GYR //gyroscope
//#   define INCLUDE_BMI160MAG //magnetometer

/* enable the macro of the secondary interface which is used */
//#   define YAS537 /* used to select the secondary interface as YAS537 */
//#   define YAS532  /* used to select the secondary interface as YAS532 */
//#   define AKM09911 /* used to select the secondary interface as AKM09911 */
//#   define AKM09912 /* used to select the secondary interface as AKM09912 */

#endif // _APP_BMI160_CONFIG_H_
