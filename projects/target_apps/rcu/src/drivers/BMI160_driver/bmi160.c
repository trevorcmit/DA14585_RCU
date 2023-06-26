/*
 ****************************************************************************
 * Copyright (C) 2016 Bosch Sensortec GmbH
 *
 * bmi160.c
 * Date: 2016/06/27
 * Revision: 2.2.1 $
 *
 * Usage: Sensor Driver for BMI160 sensor
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

/*! \file bmi160.c
    \brief Sensor driver for BMI160 */

/*****************************************************************************************
 * \addtogroup USER
 * \{
 * \addtogroup USER_DRIVERS
 * \{
 * \addtogroup BMI160_DRV
 *
 * \{
******************************************************************************************/
 
#include "bmi160.h"

struct bmi160_t *p_bmi160;

#ifdef INCLUDE_BMI160MAG
/* Used for reading the Mag trim values for compensation*/
struct trim_data_t mag_trim;
/* the following variable is used for avoiding the selecting of auto mode
   when it is running in the manual mode of BMM150 Mag interface*/
u8 bmm150_manual_auto_condition_u8_g;
#endif //INCLUDE_BMI160MAG

#ifdef INCLUDE_BMI160PMU
/* Power mode monitoring variable used to introduce delays after primary
   interface write in low power and suspend modes of sensor */
u8 bmi160_power_mode_status_u8_g;
#endif //INCLUDE_BMI160PMU


#ifdef INCLUDE_BMI160MAG
    #ifdef YAS532
    /* YAMAHA-YAS532*/

/* value of coefficient*/
static const int yas532_version_ac_coef[] = {
    YAS532_VERSION_AC_COEF_X, YAS532_VERSION_AC_COEF_Y1, YAS532_VERSION_AC_COEF_Y2
};

/* used for reading the yas532 calibration data*/
struct yas532_t yas532_data;

struct yas532_vector fifo_xyz_data;
    #endif //YAS532

    #ifdef YAS537
/* used for reading the yas537 calibration data*/
struct yas537_t yas537_data;

struct yas_vector fifo_vector_xyz;
    #endif //YAS537
    
    #if defined AKM09911 || defined AKM09912
/* used to read the AKM compensating data */
struct bst_akm_sensitivity_data_t akm_asa_data;
    #endif //defined AKM09911 || defined AKM09912

struct bmi160_mag_xyz_s32_t processed_data;
#endif //INCLUDE_BMI160MAG

BMI160_RETURN_FUNCTION_TYPE bmi160_init(
    struct bmi160_t *bmi160)
{
    u8 v_pmu_data_u8 = BMI160_INIT_VALUE;
	
    /* assign bmi160 pointer */
	p_bmi160 = bmi160;
	
    /* store the chip id which is read from the sensor */
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_CHIP_ID__REG,
            &p_bmi160->chip_id,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
	/* To avoid gyro wakeup it is required to write 0x00 to 0x6C*/
	com_rslt += bmi160_write_reg(BMI160_USER_PMU_TRIGGER_ADDR,
                                 &v_pmu_data_u8,
                                 BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_write_reg(
    u8 v_addr_u8, u8 *v_data_u8, u8 v_len_u8)
{
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write data from register*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_WRITE_FUNC(
            p_bmi160->dev_addr, v_addr_u8, v_data_u8, v_len_u8);

#ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
#endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_reg(
    u8 v_addr_u8, u8 *v_data_u8, u8 v_len_u8)
{
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* Read data from register*/
    return p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
                                          v_addr_u8, 
                                          v_data_u8, 
                                          v_len_u8);
}

#ifdef INCLUDE_BMI160ERR
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fatal_err(
    u8 *v_fatal_err_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read the fatal error status*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FATAL_ERR__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fatal_err_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FATAL_ERR);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_err_code(
    u8 *v_err_code_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_ERR_CODE__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_err_code_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ERR_CODE);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_fail_err(
    u8 *v_i2c_err_code_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_I2C_FAIL_ERR__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_i2c_err_code_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_I2C_FAIL_ERR);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_drop_cmd_err(
    u8 *v_drop_cmd_err_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DROP_CMD_ERR__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_drop_cmd_err_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_DROP_CMD_ERR);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_data_rdy_err(
    u8 *v_mag_data_rdy_err_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_DATA_RDY_ERR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_data_rdy_err_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_MAG_DATA_RDY_ERR);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_error_status(
    u8 *v_fatal_err_u8, u8 *v_err_code_u8, u8 *v_i2c_fail_err_u8,
    u8 *v_drop_cmd_err_u8, u8 *v_mag_data_rdy_err_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read the error codes*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_ERR_STAT__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* fatal error*/
    *v_fatal_err_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FATAL_ERR);   
    
    /* user error*/
    *v_err_code_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ERR_CODE);    
    
    /* i2c fail error*/
    *v_i2c_fail_err_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_I2C_FAIL_ERR);    
    
    /* drop command error*/
    *v_drop_cmd_err_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_DROP_CMD_ERR);
    
    /* Mag data ready error*/
    *v_mag_data_rdy_err_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_MAG_DATA_RDY_ERR);
	
    return com_rslt;
}
#endif //INCLUDE_BMI160ERR

#ifdef INCLUDE_BMI160PMU
    #ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_power_mode_stat(
    u8 *v_mag_power_mode_stat_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_POWER_MODE_STAT__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_power_mode_stat_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_MAG_POWER_MODE_STAT);

	return com_rslt;
}
    #endif //INCLUDE_BMI160MAG

    #ifdef INCLUDE_BMI160GYR
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_power_mode_stat(
    u8 *v_gyro_power_mode_stat_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_GYRO_POWER_MODE_STAT__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_power_mode_stat_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_POWER_MODE_STAT);
    
	return com_rslt;
}
    #endif //INCLUDE_BMI160GYR

    #ifdef INCLUDE_BMI160ACC
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_power_mode_stat(
    u8 *v_accel_power_mode_stat_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_ACCEL_POWER_MODE_STAT__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_power_mode_stat_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ACCEL_POWER_MODE_STAT);
    
	return com_rslt;
}
    #endif //INCLUDE_BMI160ACC
#endif //INCLUDE_BMI160PMU

#ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_interface_normal(void)
{
	/* aim to check the result of switching Mag normal */
	u8 v_try_times_u8 = BMI160_MAG_NORMAL_SWITCH_TIMES;
    
	u8 v_mag_pmu_status_u8 = BMI160_INIT_VALUE;

	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_set_command_register(MAG_MODE_NORMAL);    
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
	while (v_try_times_u8 != 0) {
		com_rslt = bmi160_get_mag_power_mode_stat(&v_mag_pmu_status_u8);
		if (v_mag_pmu_status_u8 == MAG_INTERFACE_PMU_ENABLE) {
			break;
        }
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		v_try_times_u8--;
	}
	if (v_mag_pmu_status_u8 == MAG_INTERFACE_PMU_ENABLE) {
		com_rslt += SUCCESS;
    } else {
		com_rslt += E_BMI160_COMM_RES;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_x(
    s16 *v_mag_x_s16, u8 v_sensor_select_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	/* Array contains the Mag X LSB and MSB data
		v_data_u8[0] - LSB
		v_data_u8[1] - MSB*/
	u8 v_data_u8[BMI160_MAG_X_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_sensor_select_u8) {
    case BST_BMM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_MAG_X_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_X_DATA_LENGTH);
    
        /* X axis*/
        v_data_u8[BMI160_MAG_X_LSB_BYTE] = BMI160_GET_BITSLICE(
                                            v_data_u8[BMI160_MAG_X_LSB_BYTE],
                                            BMI160_USER_DATA_MAG_X_LSB);
        *v_mag_x_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_MAG_X_MSB_BYTE])) <<
                               BMI160_SHIFT_BIT_POSITION_BY_05_BITS) |
                              v_data_u8[BMI160_MAG_X_LSB_BYTE]);
        break;    
    case BST_AKM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_0_MAG_X_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_X_DATA_LENGTH);
        *v_mag_x_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_MAG_X_MSB_BYTE])) <<
                               BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                              v_data_u8[BMI160_MAG_X_LSB_BYTE]);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_y(
    s16 *v_mag_y_s16, u8 v_sensor_select_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_OUT_OF_RANGE;
	
    /* Array contains the Mag Y LSB and MSB data
		v_data_u8[0] - LSB
		v_data_u8[1] - MSB*/
	u8 v_data_u8[BMI160_MAG_Y_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_sensor_select_u8) {
    case BST_BMM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_MAG_Y_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_Y_DATA_LENGTH);
        
        /*Y-axis LSB value shifting*/
        v_data_u8[BMI160_MAG_Y_LSB_BYTE] = BMI160_GET_BITSLICE(
                                            v_data_u8[BMI160_MAG_Y_LSB_BYTE],
                                            BMI160_USER_DATA_MAG_Y_LSB);
        *v_mag_y_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_MAG_Y_MSB_BYTE])) <<
                               BMI160_SHIFT_BIT_POSITION_BY_05_BITS) |
                              v_data_u8[BMI160_MAG_Y_LSB_BYTE]);
        break;
    case BST_AKM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_2_MAG_Y_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_Y_DATA_LENGTH);
        *v_mag_y_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_MAG_Y_MSB_BYTE])) <<
                               BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                              v_data_u8[BMI160_MAG_Y_LSB_BYTE]);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_z(
    s16 *v_mag_z_s16, u8 v_sensor_select_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* Array contains the Mag Z LSB and MSB data
		v_data_u8[0] - LSB
		v_data_u8[1] - MSB*/
	u8 v_data_u8[BMI160_MAG_Z_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_sensor_select_u8) {
    case BST_BMM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_MAG_Z_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_Z_DATA_LENGTH);
    
        /*Z-axis LSB value shifting*/
        v_data_u8[BMI160_MAG_Z_LSB_BYTE] = BMI160_GET_BITSLICE(
                                            v_data_u8[BMI160_MAG_Z_LSB_BYTE],
                                            BMI160_USER_DATA_MAG_Z_LSB);    
        *v_mag_z_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_MAG_Z_MSB_BYTE])) <<
                               BMI160_SHIFT_BIT_POSITION_BY_07_BITS) |
                              v_data_u8[BMI160_MAG_Z_LSB_BYTE]);
        break;
    case BST_AKM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_4_MAG_Z_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_Z_DATA_LENGTH);
        *v_mag_z_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_MAG_Z_MSB_BYTE])) <<
                               BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                              v_data_u8[BMI160_MAG_Z_LSB_BYTE]);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_r(s16 *v_mag_r_s16)
{
	/* Array contains the Mag R LSB and MSB data
		v_data_u8[0] - LSB
		v_data_u8[1] - MSB*/
	u8 v_data_u8[BMI160_MAG_R_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_6_RHALL_LSB__REG,
            v_data_u8, 
            BMI160_MAG_R_DATA_LENGTH);
    
    /*R-axis LSB value shifting*/
    v_data_u8[BMI160_MAG_R_LSB_BYTE] = BMI160_GET_BITSLICE(
                                        v_data_u8[BMI160_MAG_R_LSB_BYTE],
                                        BMI160_USER_DATA_MAG_R_LSB);
    *v_mag_r_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_MAG_R_MSB_BYTE])) <<
                           BMI160_SHIFT_BIT_POSITION_BY_06_BITS) |
                          v_data_u8[BMI160_MAG_R_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_xyz(
    struct bmi160_mag_t *mag, u8 v_sensor_select_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* Array contains the Mag XYZ LSB and MSB data
		v_data_u8[0] - X-LSB
		v_data_u8[1] - X-MSB
		v_data_u8[0] - Y-LSB
		v_data_u8[1] - Y-MSB
		v_data_u8[0] - Z-LSB
		v_data_u8[1] - Z-MSB
		*/
	u8 v_data_u8[BMI160_MAG_XYZ_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_sensor_select_u8) {
    case BST_BMM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_MAG_X_LSB__REG,
            v_data_u8, 
            BMI160_MAG_XYZ_DATA_LENGTH);
        
        /*X-axis LSB value shifting*/
        v_data_u8[BMI160_DATA_FRAME_MAG_X_LSB_BYTE] =
            BMI160_GET_BITSLICE(v_data_u8[BMI160_DATA_FRAME_MAG_X_LSB_BYTE],
                                BMI160_USER_DATA_MAG_X_LSB);
        
        /* Data X */
        mag->x = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_X_MSB_BYTE])) <<
                         BMI160_SHIFT_BIT_POSITION_BY_05_BITS) |
                        v_data_u8[BMI160_DATA_FRAME_MAG_X_LSB_BYTE]);
        
        /*Y-axis LSB value shifting*/
        v_data_u8[BMI160_DATA_FRAME_MAG_Y_LSB_BYTE] =
            BMI160_GET_BITSLICE(v_data_u8[BMI160_DATA_FRAME_MAG_Y_LSB_BYTE],
                                BMI160_USER_DATA_MAG_Y_LSB);

        /* Data Y */
        mag->y = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_Y_MSB_BYTE])) <<
                         BMI160_SHIFT_BIT_POSITION_BY_05_BITS) |
                        v_data_u8[BMI160_DATA_FRAME_MAG_Y_LSB_BYTE]);

        /*Z-axis LSB value shifting*/
        v_data_u8[BMI160_DATA_FRAME_MAG_Z_LSB_BYTE] =
            BMI160_GET_BITSLICE(v_data_u8[BMI160_DATA_FRAME_MAG_Z_LSB_BYTE],
                                BMI160_USER_DATA_MAG_Z_LSB);
        /* Data Z */
        mag->z = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_Z_MSB_BYTE])) <<
                         BMI160_SHIFT_BIT_POSITION_BY_07_BITS) |
                        v_data_u8[BMI160_DATA_FRAME_MAG_Z_LSB_BYTE]);
        break;
    case BST_AKM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_0_MAG_X_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_XYZ_DATA_LENGTH);
    
        /* Data X */
        mag->x = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_X_MSB_BYTE])) <<
                         BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                        v_data_u8[BMI160_DATA_FRAME_MAG_X_LSB_BYTE]);
        
        /* Data Y */
        mag->y  = ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_Y_MSB_BYTE])) <<
                    BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                   v_data_u8[BMI160_DATA_FRAME_MAG_Y_LSB_BYTE]);
        
        /* Data Z */
        mag->z = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_Z_MSB_BYTE])) <<
                         BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                        v_data_u8[BMI160_DATA_FRAME_MAG_Z_LSB_BYTE]);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_mag_xyzr(
    struct bmi160_mag_xyzr_t *mag)
{
	u8 v_data_u8[BMI160_MAG_XYZR_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
    
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_MAG_X_LSB__REG,
            v_data_u8, 
            BMI160_MAG_XYZR_DATA_LENGTH);

    /* Data X */
    /*X-axis LSB value shifting*/
    v_data_u8[BMI160_DATA_FRAME_MAG_X_LSB_BYTE] =
        BMI160_GET_BITSLICE(v_data_u8[BMI160_DATA_FRAME_MAG_X_LSB_BYTE],
                            BMI160_USER_DATA_MAG_X_LSB);
    mag->x = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_X_MSB_BYTE])) <<
                     BMI160_SHIFT_BIT_POSITION_BY_05_BITS) |
                    v_data_u8[BMI160_DATA_FRAME_MAG_X_LSB_BYTE]);
    
    /* Data Y */
    /*Y-axis LSB value shifting*/
    v_data_u8[BMI160_DATA_FRAME_MAG_Y_LSB_BYTE] =
        BMI160_GET_BITSLICE(v_data_u8[BMI160_DATA_FRAME_MAG_Y_LSB_BYTE],
                            BMI160_USER_DATA_MAG_Y_LSB);
    mag->y = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_Y_MSB_BYTE])) <<
                     BMI160_SHIFT_BIT_POSITION_BY_05_BITS) |
                    v_data_u8[BMI160_DATA_FRAME_MAG_Y_LSB_BYTE]);

    /* Data Z */
    /*Z-axis LSB value shifting*/
    v_data_u8[BMI160_DATA_FRAME_MAG_Z_LSB_BYTE] =
        BMI160_GET_BITSLICE(v_data_u8[BMI160_DATA_FRAME_MAG_Z_LSB_BYTE],
                            BMI160_USER_DATA_MAG_Z_LSB);
    mag->z = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_Z_MSB_BYTE])) <<
                     BMI160_SHIFT_BIT_POSITION_BY_07_BITS) |
                    v_data_u8[BMI160_DATA_FRAME_MAG_Z_LSB_BYTE]);

    /* RHall */
    /*R-axis LSB value shifting*/
    v_data_u8[BMI160_DATA_FRAME_MAG_R_LSB_BYTE] =
        BMI160_GET_BITSLICE(v_data_u8[BMI160_DATA_FRAME_MAG_R_LSB_BYTE],
                            BMI160_USER_DATA_MAG_R_LSB);
    mag->r = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_MAG_R_MSB_BYTE])) <<
                     BMI160_SHIFT_BIT_POSITION_BY_06_BITS) |
                    v_data_u8[BMI160_DATA_FRAME_MAG_R_LSB_BYTE]);

    return com_rslt;
}
#endif //INCLUDE_BMI160MAG

#ifdef INCLUDE_BMI160GYR
BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_x(
    s16 *v_gyro_x_s16)
{
    /* Array contains the gyro X LSB and MSB data
		v_data_u8[0] - LSB
		v_data_u8[MSB_ONE] - MSB*/
	u8 v_data_u8[BMI160_GYRO_X_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_8_GYRO_X_LSB__REG,
            v_data_u8, 
            BMI160_GYRO_DATA_LENGTH);
    *v_gyro_x_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_GYRO_X_MSB_BYTE])) <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_data_u8[BMI160_GYRO_X_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_y(
    s16 *v_gyro_y_s16)
{
    /* Array contains the gyro Y LSB and MSB data
		v_data_u8[LSB_ZERO] - LSB
		v_data_u8[MSB_ONE] - MSB*/
	u8 v_data_u8[BMI160_GYRO_Y_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read gyro y data*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_10_GYRO_Y_LSB__REG,
            v_data_u8,
            BMI160_GYRO_DATA_LENGTH);
    *v_gyro_y_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_GYRO_Y_MSB_BYTE])) <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_data_u8[BMI160_GYRO_Y_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_z(
    s16 *v_gyro_z_s16)
{
	/* Array contains the gyro Z LSB and MSB data
		v_data_u8[LSB_ZERO] - LSB
		v_data_u8[MSB_ONE] - MSB*/
	u8 v_data_u8[BMI160_GYRO_Z_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read gyro z data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_12_GYRO_Z_LSB__REG,
            v_data_u8, 
            BMI160_GYRO_DATA_LENGTH);
    *v_gyro_z_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_GYRO_Z_MSB_BYTE])) <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_data_u8[BMI160_GYRO_Z_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_gyro_xyz(
    struct bmi160_gyro_t *gyro)
{
    /* Array contains the Mag XYZ LSB and MSB data
		v_data_u8[0] - X-LSB
		v_data_u8[1] - X-MSB
		v_data_u8[0] - Y-LSB
		v_data_u8[1] - Y-MSB
		v_data_u8[0] - Z-LSB
		v_data_u8[1] - Z-MSB
		*/
	u8 v_data_u8[BMI160_GYRO_XYZ_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the gyro xyz data*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_8_GYRO_X_LSB__REG,
            v_data_u8, 
            BMI160_GYRO_XYZ_DATA_LENGTH);

    /* Data X */
    gyro->x = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_GYRO_X_MSB_BYTE])) <<
                      BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                     v_data_u8[BMI160_DATA_FRAME_GYRO_X_LSB_BYTE]);
    
    /* Data Y */
    gyro->y = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_GYRO_Y_MSB_BYTE])) <<
                      BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                     v_data_u8[BMI160_DATA_FRAME_GYRO_Y_LSB_BYTE]);

    /* Data Z */
    gyro->z = (s16) ((((s32)((s8)v_data_u8[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE])) <<
                      BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                     v_data_u8[BMI160_DATA_FRAME_GYRO_Z_LSB_BYTE]);

    return com_rslt;
}
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_x(
    s16 *v_accel_x_s16)
{
    /* Array contains the Accel X LSB and MSB data
		v_data_u8[0] - LSB
		v_data_u8[1] - MSB*/
	u8 v_data_u8[BMI160_ACCEL_X_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_14_ACCEL_X_LSB__REG,
            v_data_u8, 
            BMI160_ACCEL_DATA_LENGTH);
    *v_accel_x_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_ACCEL_X_MSB_BYTE])) <<
                             BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                            v_data_u8[BMI160_ACCEL_X_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_y(
    s16 *v_accel_y_s16)
{
	/* Array contains the Accel Y LSB and MSB data
		v_data_u8[0] - LSB
		v_data_u8[1] - MSB*/

	u8 v_data_u8[BMI160_ACCEL_Y_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_16_ACCEL_Y_LSB__REG,
            v_data_u8, 
            BMI160_ACCEL_DATA_LENGTH);
    *v_accel_y_s16 = (s16) ((((s32)((s8)v_data_u8[BMI160_ACCEL_Y_MSB_BYTE])) <<
                             BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                            v_data_u8[BMI160_ACCEL_Y_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_z(
    s16 *v_accel_z_s16)
{
	/* Array contains the Accel Z LSB and MSB data
		a_data_u8r[LSB_ZERO] - LSB
		a_data_u8r[MSB_ONE] - MSB*/

	u8 a_data_u8r[BMI160_ACCEL_Z_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_18_ACCEL_Z_LSB__REG,
            a_data_u8r,
            BMI160_ACCEL_DATA_LENGTH);
    *v_accel_z_s16 = (s16) ((((s32)((s8)a_data_u8r[BMI160_ACCEL_Z_MSB_BYTE])) <<
                             BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                            a_data_u8r[BMI160_ACCEL_Z_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_xyz(
    struct bmi160_accel_t *accel)
{
    /* Array contains the Accel XYZ LSB and MSB data
        a_data_u8r[0] - X-LSB
        a_data_u8r[1] - X-MSB
        a_data_u8r[0] - Y-LSB
        a_data_u8r[1] - Y-MSB
        a_data_u8r[0] - Z-LSB
        a_data_u8r[1] - Z-MSB
	*/
	u8 a_data_u8r[BMI160_ACCEL_XYZ_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_DATA_14_ACCEL_X_LSB__REG,
            a_data_u8r,
            BMI160_ACCEL_XYZ_DATA_LENGTH);

    /* Data X */
    accel->x = (s16) ((((s32)((s8)a_data_u8r[BMI160_DATA_FRAME_ACCEL_X_MSB_BYTE])) <<
                       BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                      a_data_u8r[BMI160_DATA_FRAME_ACCEL_X_LSB_BYTE]);
    
    /* Data Y */
    accel->y = (s16) ((((s32)((s8)a_data_u8r[BMI160_DATA_FRAME_ACCEL_Y_MSB_BYTE])) <<
                       BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                      a_data_u8r[BMI160_DATA_FRAME_ACCEL_Y_LSB_BYTE]);

    /* Data Z */
    accel->z = (s16) ((((s32)((s8)a_data_u8r[BMI160_DATA_FRAME_ACCEL_Z_MSB_BYTE])) <<
                       BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                      a_data_u8r[BMI160_DATA_FRAME_ACCEL_Z_LSB_BYTE]);

    return com_rslt;
}
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160TIM
BMI160_RETURN_FUNCTION_TYPE bmi160_get_sensor_time(
    u32 *v_sensor_time_u32)
{
    /* Array contains the sensor time it is 32 bit data
        a_data_u8r[0] - sensor time
        a_data_u8r[1] - sensor time
        a_data_u8r[0] - sensor time
	*/
	u8 a_data_u8r[BMI160_SENSOR_TIME_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_SENSORTIME_0_SENSOR_TIME_LSB__REG,
            a_data_u8r, 
            BMI160_SENSOR_TIME_LENGTH);
    *v_sensor_time_u32 = (u32) ((((u32)a_data_u8r[BMI160_SENSOR_TIME_MSB_BYTE]) <<
                                 BMI160_SHIFT_BIT_POSITION_BY_16_BITS) |
                                (((u32)a_data_u8r[BMI160_SENSOR_TIME_XLSB_BYTE]) <<
                                 BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                a_data_u8r[BMI160_SENSOR_TIME_LSB_BYTE]);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_accel_gyro_sensor_time(
	u8 accel_gyro_sensortime_select,
    struct bmi160_sensortime_accel_gyro_data *accel_gyro_sensor_time)
{
    u8 a_data_u8r[BMI160_GYRO_ACCEL_SENSORTIME_DATA_SIZE];
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    switch (accel_gyro_sensortime_select) {
    case BMI160_ACCEL_SENSORTIME_DATA:
        com_rslt = p_bmi160->BMI160_BURST_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_14_ACCEL_X_LSB__REG, 
                    a_data_u8r,
                    BMI160_ACCEL_SENSORTIME_DATA_SIZE);
        
        /* Accel Data X */
        accel_gyro_sensor_time->accel.x =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_ACCEL_X_MSB_BYTE])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_ACCEL_X_LSB_BYTE]);
    
        /* Accel Data Y */
        accel_gyro_sensor_time->accel.y =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_ACCEL_Y_MSB_BYTE])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_ACCEL_Y_LSB_BYTE]);
    
        /* Accel Data Z */
        accel_gyro_sensor_time->accel.z =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_ACCEL_Z_MSB_BYTE])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_ACCEL_Z_LSB_BYTE]);
                  
        /* Sensor time data */
        accel_gyro_sensor_time->v_sensor_time_u32 =
            (u32)((((u32)a_data_u8r[BMI160_DATA_FRAME_ACCEL_Z_MSB_BYTE+3]) <<
                   BMI160_SHIFT_BIT_POSITION_BY_16_BITS) |
                  (((u32)a_data_u8r[BMI160_DATA_FRAME_ACCEL_Z_MSB_BYTE+2]) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_ACCEL_Z_MSB_BYTE+1]);
        break;
    case BMI160_GYRO_ACCEL_SENSORTIME_DATA:
        com_rslt = p_bmi160->BMI160_BURST_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_8_GYRO_X_LSB__REG,
                    a_data_u8r,
                    BMI160_GYRO_ACCEL_SENSORTIME_DATA_SIZE);
        
        /* Gyro Data X */
        accel_gyro_sensor_time->gyro.x =
            (s16)((((s32)((s8) a_data_u8r[BMI160_DATA_FRAME_GYRO_X_MSB_BYTE])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_GYRO_X_LSB_BYTE]);
        
        /* Gyro Data Y */
        accel_gyro_sensor_time->gyro.y =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_GYRO_Y_MSB_BYTE])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_GYRO_Y_LSB_BYTE]);
        
        /* Gyro Data Z */
        accel_gyro_sensor_time->gyro.z =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_LSB_BYTE]);
        
        /* Accel Data X */
        accel_gyro_sensor_time->accel.x =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+2])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+1]);
        
        /* Accel Data Y */
        accel_gyro_sensor_time->accel.y =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+4])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+3]);
        
        /* Accel Data Z */
        accel_gyro_sensor_time->accel.z =
            (s16)((((s32) ((s8)a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+6])) <<
                   BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                  a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+5]);
        
        /* Sensor time data */
        accel_gyro_sensor_time->v_sensor_time_u32 =
            (u32) ((((u32)a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+9]) <<
                    BMI160_SHIFT_BIT_POSITION_BY_16_BITS) |
                   (((u32)a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+8]) <<
                    BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                   a_data_u8r[BMI160_DATA_FRAME_GYRO_Z_MSB_BYTE+7]);
        break;
    }
    
	return com_rslt;
}
#endif //INCLUDE_BMI160TIM

#ifdef INCLUDE_BMI160GYR
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_selftest(
    u8 *v_gyro_selftest_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STAT_GYRO_SELFTEST_OK__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_selftest_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_STAT_GYRO_SELFTEST_OK);

	return com_rslt;
}
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160GET
    #ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_manual_operation_stat(
    u8 *v_mag_manual_stat_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read manual operation*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STAT_MAG_MANUAL_OPERATION__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_mag_manual_stat_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_STAT_MAG_MANUAL_OPERATION);

	return com_rslt;
}
    #endif //INCLUDE_BMI160MAG

    #ifdef INCLUDE_BMI160FOC
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_rdy(
    u8 *v_foc_rdy_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read the FOC status*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STAT_FOC_RDY__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_foc_rdy_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_STAT_FOC_RDY);

    return com_rslt;
}
    #endif //INCLUDE_BMI160FOC

    #ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_get_data_rdy_mag(
    u8 *v_data_rdy_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STAT_DATA_RDY_MAG__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_data_rdy_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_STAT_DATA_RDY_MAG);

	return com_rslt;
}
    #endif //INCLUDE_BMI160MAG

    #ifdef INCLUDE_BMI160GYR
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_data_rdy(
    u8 *v_data_rdy_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STAT_DATA_RDY_GYRO__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_data_rdy_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_STAT_DATA_RDY_GYRO);
    
	return com_rslt;
}
    #endif //INCLUDE_BMI160GYR

    #ifdef INCLUDE_BMI160ACC
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_data_rdy(
    u8 *v_data_rdy_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /*reads the status of Accel data ready*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STAT_DATA_RDY_ACCEL__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_data_rdy_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_STAT_DATA_RDY_ACCEL);
    
	return com_rslt;
}
    #endif //INCLUDE_BMI160ACC
#endif //INCLUDE_BMI160GET

#ifdef INCLUDE_BMI160INT
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_step_intr(
    u8 *v_step_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_STEP_INTR__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_step_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_STEP_INTR);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_significant_intr(
    u8 *v_significant_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_significant_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_SIGNIFICANT_INTR);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_any_motion_intr(
    u8 *v_any_motion_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_ANY_MOTION__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_any_motion_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_ANY_MOTION);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_pmu_trigger_intr(
    u8 *v_pmu_trigger_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_PMU_TRIGGER__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);    
    *v_pmu_trigger_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_PMU_TRIGGER);
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_double_tap_intr(
    u8 *v_double_tap_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_double_tap_intr_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_DOUBLE_TAP_INTR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_single_tap_intr(
    u8 *v_single_tap_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_single_tap_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_SINGLE_TAP_INTR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_orient_intr(
    u8 *v_orient_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_ORIENT__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_ORIENT);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat0_flat_intr(
    u8 *v_flat_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_0_FLAT__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_flat_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_0_FLAT);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_high_g_intr(
    u8 *v_high_g_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_1_HIGH_G_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_1_HIGH_G_INTR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_low_g_intr(
    u8 *v_low_g_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_1_LOW_G_INTR__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_low_g_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_1_LOW_G_INTR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_data_rdy_intr(
    u8 *v_data_rdy_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_1_DATA_RDY_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_data_rdy_intr_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_1_DATA_RDY_INTR);

	return com_rslt;
}

    #ifdef FIFO_ENABLE
BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_fifo_full_intr(
    u8 *v_fifo_full_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_full_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_1_FIFO_FULL_INTR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_fifo_wm_intr(
    u8 *v_fifo_wm_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_1_FIFO_WM_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_wm_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_1_FIFO_WM_INTR);

	return com_rslt;
}
    #endif //FIFO_ENABLE

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat1_nomotion_intr(
    u8 *v_nomotion_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read the no motion interrupt*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_1_NOMOTION_INTR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_nomotion_intr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_1_NOMOTION_INTR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_x(
    u8 *v_anymotion_first_x_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read the any motion first x interrupt*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_anymotion_first_x_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_X);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_y(
    u8 *v_any_motion_first_y_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read the any motion first y interrupt*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_any_motion_first_y_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Y);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_first_z(
    u8 *v_any_motion_first_z_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read the any motion first z interrupt*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_any_motion_first_z_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_ANY_MOTION_FIRST_Z);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_any_motion_sign(
    u8 *v_anymotion_sign_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read any motion sign interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_anymotion_sign_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_ANY_MOTION_SIGN);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_first_x(
    u8 *v_tap_first_x_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read tap first x interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_TAP_FIRST_X__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_first_x_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_TAP_FIRST_X);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_first_y(
    u8 *v_tap_first_y_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read tap first y interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_TAP_FIRST_Y__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_first_y_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_TAP_FIRST_Y);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_first_z(
    u8 *v_tap_first_z_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read tap first z interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_TAP_FIRST_Z__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_first_z_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_TAP_FIRST_Z);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat2_tap_sign(
    u8 *v_tap_sign_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read tap_sign interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_2_TAP_SIGN__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_sign_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_2_TAP_SIGN);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_x(
    u8 *v_high_g_first_x_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read highg_x interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_first_x_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_X);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_y(
    u8 *v_high_g_first_y_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read highg_y interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_first_y_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Y);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_first_z(
    u8 *v_high_g_first_z_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read highg_z interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_first_z_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_3_HIGH_G_FIRST_Z);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_high_g_sign(
    u8 *v_high_g_sign_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read highg_sign interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_3_HIGH_G_SIGN__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_sign_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_3_HIGH_G_SIGN);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_orient_xy(
    u8 *v_orient_xy_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read orient plane xy interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_3_ORIENT_XY__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_xy_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_3_ORIENT_XY);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_orient_z(
    u8 *v_orient_z_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read orient z plane interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_3_ORIENT_Z__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_z_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_3_ORIENT_Z);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_stat3_flat(
    u8 *v_flat_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read flat interrupt status */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_STAT_3_FLAT__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_flat_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_STAT_3_FLAT);

	return com_rslt;
}
#endif //INCLUDE_BMI160INT

#ifdef INCLUDE_BMI160TEM
BMI160_RETURN_FUNCTION_TYPE bmi160_get_temp(
    s16 *v_temp_s16)
{
	/* Array contains the temperature LSB and MSB data
        v_data_u8[0] - LSB
        v_data_u8[1] - MSB*/
	u8 v_data_u8[BMI160_TEMP_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read temperature data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_TEMP_LSB_VALUE__REG, 
            v_data_u8,
            BMI160_TEMP_DATA_LENGTH);
    *v_temp_s16 = (s16) (((s32) ((s8) v_data_u8[BMI160_TEMP_MSB_BYTE] <<
                                      BMI160_SHIFT_BIT_POSITION_BY_08_BITS)) |
                         v_data_u8[BMI160_TEMP_LSB_BYTE]);

	return com_rslt;
}
#endif //INCLUDE_BMI160TEM

#ifdef FIFO_ENABLE
BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_length(
    u32 *v_fifo_length_u32)
{
	/* Array contains the FIFO length data
        v_data_u8[0] - FIFO length
        v_data_u8[1] - FIFO length*/
	u8 a_data_u8r[BMI160_FIFO_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read FIFO length*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_BYTE_COUNTER_LSB__REG, 
            a_data_u8r,
            BMI160_FIFO_DATA_LENGTH);

    a_data_u8r[BMI160_FIFO_LENGTH_MSB_BYTE] =
        BMI160_GET_BITSLICE(a_data_u8r[BMI160_FIFO_LENGTH_MSB_BYTE],
                            BMI160_USER_FIFO_BYTE_COUNTER_MSB);

    *v_fifo_length_u32 = (u32) (((u32) ((u8) a_data_u8r[BMI160_FIFO_LENGTH_MSB_BYTE] <<
                                             BMI160_SHIFT_BIT_POSITION_BY_08_BITS)) |
                                a_data_u8r[BMI160_FIFO_LENGTH_LSB_BYTE]);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_fifo_data(
    u8 *v_fifodata_u8, u16 v_fifo_length_u16)
{
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if(v_fifo_length_u16 > FIFO_FRAME) {
        return E_BMI160_OUT_OF_RANGE;
    }
  
    /* read FIFO data*/
    return p_bmi160->BMI160_BURST_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_DATA__REG,
            v_fifodata_u8, 
            v_fifo_length_u16);
}
#endif //FIFO_ENABLE

#ifdef INCLUDE_BMI160ACC
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_output_data_rate(
    u8 *v_output_data_rate_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the Accel output data rate*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_output_data_rate_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_output_data_rate(
    u8 v_output_data_rate_u8, u8 v_accel_bw_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_odr_u8 = BMI160_INIT_VALUE;
	u8 v_assign_bw = BMI160_ASSIGN_DATA;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_accel_bw_u8 >= BMI160_ACCEL_RES_AVG2 &&
        v_accel_bw_u8 <= BMI160_ACCEL_RES_AVG128) {
        /* enable the under sampling*/
        com_rslt = bmi160_set_accel_under_sampling_parameter(BMI160_US_ENABLE);
    } else if ((v_accel_bw_u8 > BMI160_ACCEL_OSR4_AVG1 &&
                v_accel_bw_u8 <= BMI160_ACCEL_CIC_AVG8) ||
               v_accel_bw_u8 == BMI160_ACCEL_OSR4_AVG1) {
        /* disable the under sampling*/
        com_rslt = bmi160_set_accel_under_sampling_parameter(BMI160_US_DISABLE);
    }
               
    /* assign the output data rate*/
    switch (v_accel_bw_u8) {
    case BMI160_ACCEL_RES_AVG2:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_RES_AVG4:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_RES_AVG8:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_100HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_RES_AVG16:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_50HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_RES_AVG32:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_25HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_RES_AVG64:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_RES_AVG128:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_0_78HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_6_25HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_OSR4_AVG1:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_OSR2_AVG2:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_NORMAL_AVG4:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    case BMI160_ACCEL_CIC_AVG8:
        if (v_output_data_rate_u8 >= BMI160_ACCEL_OUTPUT_DATA_RATE_12_5HZ &&
            v_output_data_rate_u8 <= BMI160_ACCEL_OUTPUT_DATA_RATE_1600HZ) {
            v_odr_u8 = v_output_data_rate_u8;
            v_assign_bw = SUCCESS;
        } else {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    if (v_assign_bw == SUCCESS) {
        /* write Accel output data rate */
        com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE,
                            v_odr_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_ACCEL_CONFIG_OUTPUT_DATA_RATE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }
	return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_bw(
    u8 *v_bw_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the Accel bandwidth */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_bw_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ACCEL_CONFIG_ACCEL_BW);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_bw(
    u8 v_bw_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* select Accel bandwidth*/
    if (v_bw_u8 <= BMI160_MAX_ACCEL_BW) {
        /* write Accel bandwidth*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG, 
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_ACCEL_CONFIG_ACCEL_BW,
                            v_bw_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_ACCEL_CONFIG_ACCEL_BW__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_under_sampling_parameter(
    u8 *v_accel_under_sampling_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Accel under sampling parameter */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_under_sampling_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_under_sampling_parameter(
    u8 v_accel_under_sampling_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
	if (v_accel_under_sampling_u8 <= BMI160_MAX_UNDER_SAMPLING) {
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			/* write the Accel under sampling parameter */
			v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING,
                            v_accel_under_sampling_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_ACCEL_CONFIG_ACCEL_UNDER_SAMPLING__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
			/*Accel and Gyro power mode check*/
			if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
				/*interface idle time delay */
				p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
		}
	} else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
	}

    return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_range(
    u8 *v_range_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read the Accel range*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_ACCEL_RANGE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_range_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ACCEL_RANGE);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_range(
    u8 v_range_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_range_u8 == BMI160_ACCEL_RANGE0 ||
        v_range_u8 == BMI160_ACCEL_RANGE1 ||
        v_range_u8 == BMI160_ACCEL_RANGE3 ||
        v_range_u8 == BMI160_ACCEL_RANGE4) {
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_ACCEL_RANGE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8, 
                            BMI160_USER_ACCEL_RANGE,
                            v_range_u8);
            /* write the Accel range*/
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_ACCEL_RANGE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160GYR
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_output_data_rate(
    u8 *v_output_data_rate_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the gyro output data rate*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_output_data_rate_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_output_data_rate(
    u8 v_output_data_rate_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* select the gyro output data rate*/
    if (v_output_data_rate_u8 < BMI160_OUTPUT_DATA_RATE6 &&
        v_output_data_rate_u8 != BMI160_INIT_VALUE &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE1 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE2 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE3 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE4 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE5 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE6 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE7) {
        /* write the gyro output data rate */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE,
                            v_output_data_rate_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_CONFIG_OUTPUT_DATA_RATE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_bw(
    u8 *v_bw_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read gyro bandwidth*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_GYRO_CONFIG_BW__REG,
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_bw_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_CONFIG_BW);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_bw(
    u8 v_bw_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_bw_u8 <= BMI160_MAX_GYRO_BW) {
        /* write the gyro bandwidth*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_CONFIG_BW__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_CONFIG_BW, 
                            v_bw_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_CONFIG_BW__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_range(
    u8 *v_range_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read the gyro range */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_GYRO_RANGE__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_range_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_RANGE);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_range(
    u8 v_range_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_range_u8 <= BMI160_MAX_GYRO_RANGE) {
        /* write the gyro range value */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_RANGE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_RANGE,
                            v_range_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_RANGE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_output_data_rate(
    u8 *v_output_data_rate_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the Mag data output rate*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_output_data_rate_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_output_data_rate(
    u8 v_output_data_rate_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* select the Mag data output rate*/
    if (v_output_data_rate_u8 <= BMI160_MAX_ACCEL_OUTPUT_DATA_RATE &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE0 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE6 &&
        v_output_data_rate_u8 != BMI160_OUTPUT_DATA_RATE7) {
        /* write the Mag data output rate*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE,
                            v_output_data_rate_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_MAG_CONFIG_OUTPUT_DATA_RATE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160MAG

#ifdef FIFO_ENABLE
    #ifdef INCLUDE_BMI160GYR
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_down_gyro(
    u8 *v_fifo_down_gyro_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the gyro FIFO down*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_DOWN_GYRO__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_down_gyro_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_DOWN_GYRO);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_down_gyro(
    u8 v_fifo_down_gyro_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write the gyro FIFO down*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_DOWN_GYRO__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_FIFO_DOWN_GYRO,
                        v_fifo_down_gyro_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FIFO_DOWN_GYRO__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_fifo_filter_data(
    u8 *v_gyro_fifo_filter_data_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the gyro FIFO filter data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_FILTER_GYRO__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_fifo_filter_data_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_FILTER_GYRO);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_fifo_filter_data(
    u8 v_gyro_fifo_filter_data_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_gyro_fifo_filter_data_u8 <= BMI160_MAX_VALUE_FIFO_FILTER) {
        /* write the gyro FIFO filter data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FIFO_FILTER_GYRO__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_FILTER_GYRO,
                            v_gyro_fifo_filter_data_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_FILTER_GYRO__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
    #endif //INCLUDE_BMI160GYR

    #ifdef INCLUDE_BMI160ACC
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_down_accel(
    u8 *v_fifo_down_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Accel FIFO down data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_DOWN_ACCEL__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_down_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_DOWN_ACCEL);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_down_accel(
    u8 v_fifo_down_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write the Accel FIFO down data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_DOWN_ACCEL__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_FIFO_DOWN_ACCEL, 
                        v_fifo_down_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FIFO_DOWN_ACCEL__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_fifo_filter_data(
    u8 *accel_fifo_filter_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Accel FIFO filter data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_FILTER_ACCEL__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *accel_fifo_filter_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_FILTER_ACCEL);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_fifo_filter_data(
    u8 v_accel_fifo_filter_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_accel_fifo_filter_u8 <= BMI160_MAX_VALUE_FIFO_FILTER) {
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FIFO_FILTER_ACCEL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            /* write Accel FIFO filter data */
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_FILTER_ACCEL,
                            v_accel_fifo_filter_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_FILTER_ACCEL__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
    #endif //INCLUDE_BMI160ACC

BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_wm(
    u8 *v_fifo_wm_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the FIFO water mark level*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_WM__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_wm_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_WM);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_wm(
    u8 v_fifo_wm_u8)
{
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write the FIFO water mark level*/
    return p_bmi160->BMI160_BUS_WRITE_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_WM__REG,
            &v_fifo_wm_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
}

    #ifdef INCLUDE_BMI160TIM
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_time_enable(
    u8 *v_fifo_time_enable_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the FIFO sensor time*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_TIME_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_time_enable_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_TIME_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_time_enable(
    u8 v_fifo_time_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_fifo_time_enable_u8 <= BMI160_MAX_VALUE_FIFO_TIME) {
        /* write the FIFO sensor time*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FIFO_TIME_ENABLE__REG, 
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_TIME_ENABLE,
                            v_fifo_time_enable_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_TIME_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
    #endif //INCLUDE_BMI160TIM

    #ifdef INCLUDE_BMI160INT
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_tag_intr2_enable(
    u8 *v_fifo_tag_intr2_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the FIFO tag interrupt2*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_tag_intr2_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_TAG_INTR2_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_tag_intr2_enable(
    u8 v_fifo_tag_intr2_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_fifo_tag_intr2_u8 <= BMI160_MAX_VALUE_FIFO_INTR) {
        /* write the FIFO tag interrupt2*/
        com_rslt = bmi160_set_input_enable(1, v_fifo_tag_intr2_u8);
        com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG, 
                        &v_data_u8,
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_TAG_INTR2_ENABLE,
                            v_fifo_tag_intr2_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_TAG_INTR2_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

        #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
        #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_tag_intr1_enable(
    u8 *v_fifo_tag_intr1_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read FIFO tag interrupt*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_tag_intr1_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_TAG_INTR1_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_tag_intr1_enable(
    u8 v_fifo_tag_intr1_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_fifo_tag_intr1_u8 <= BMI160_MAX_VALUE_FIFO_INTR) {
        /* write the FIFO tag interrupt*/
        com_rslt = bmi160_set_input_enable(
                    BMI160_INIT_VALUE, v_fifo_tag_intr1_u8);
        com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG, 
                        &v_data_u8,
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_TAG_INTR1_ENABLE,
                            v_fifo_tag_intr1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_TAG_INTR1_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
        #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }
    
	return com_rslt;
}
    #endif //INCLUDE_BMI160INT

BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_header_enable(
    u8 *v_fifo_header_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read FIFO header */
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_HEADER_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_header_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_HEADER_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_header_enable(
    u8 v_fifo_header_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_fifo_header_u8 <= BMI160_MAX_VALUE_FIFO_HEADER) {
        /* write the FIFO header */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FIFO_HEADER_ENABLE__REG, 
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_HEADER_ENABLE,
                            v_fifo_header_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_HEADER_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_mag_enable(
    u8 *v_fifo_mag_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the FIFO Mag enable*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_MAG_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_mag_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_MAG_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_mag_enable(
    u8 v_fifo_mag_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_fifo_mag_u8 <= BMI160_MAX_VALUE_FIFO_MAG) {
        /* write the FIFO Mag enable*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FIFO_MAG_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_MAG_ENABLE,
                            v_fifo_mag_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_MAG_ENABLE__REG,
                            &v_data_u8,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
    #endif //INCLUDE_BMI160MAG

    #ifdef INCLUDE_BMI160ACC
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_accel_enable(
    u8 *v_fifo_accel_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Accel FIFO enable*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_ACCEL_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_accel_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_ACCEL_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_accel_enable(
    u8 v_fifo_accel_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_fifo_accel_u8 <= BMI160_MAX_VALUE_FIFO_ACCEL) {
        /* write the FIFO Mag enables*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FIFO_ACCEL_ENABLE__REG, 
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_ACCEL_ENABLE, 
                            v_fifo_accel_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_ACCEL_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
    #endif //INCLUDE_BMI160ACC

    #ifdef INCLUDE_BMI160GYR
BMI160_RETURN_FUNCTION_TYPE bmi160_get_fifo_gyro_enable(
    u8 *v_fifo_gyro_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read FIFO gyro enable */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_FIFO_GYRO_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_fifo_gyro_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FIFO_GYRO_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_fifo_gyro_enable(
    u8 v_fifo_gyro_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_fifo_gyro_u8 <= BMI160_MAX_VALUE_FIFO_GYRO) {
        /* write FIFO gyro enable*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FIFO_GYRO_ENABLE__REG, 
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FIFO_GYRO_ENABLE, 
                            v_fifo_gyro_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FIFO_GYRO_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
    #endif //INCLUDE_BMI160GYR
#endif //FIFO_ENABLE

#if MOTION_IF == I2C
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_device_addr(
    u8 *v_i2c_device_addr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Mag I2C device address*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_I2C_DEVICE_ADDR__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_i2c_device_addr_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_I2C_DEVICE_ADDR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_device_addr(
    u8 v_i2c_device_addr_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write the Mag I2C device address*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_I2C_DEVICE_ADDR__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_I2C_DEVICE_ADDR,
                        v_i2c_device_addr_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_I2C_DEVICE_ADDR__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

	return com_rslt;
}
#endif //MOTION_IF == I2C

#ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_burst(
    u8 *v_mag_burst_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Mag burst mode length*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_BURST__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_burst_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_MAG_BURST);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_burst(
    u8 v_mag_burst_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
    
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write Mag burst mode length*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_BURST__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_MAG_BURST, 
                        v_mag_burst_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_MAG_BURST__REG, 
                        &v_data_u8,
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);

        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_offset(
    u8 *v_mag_offset_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_OFFSET__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_offset_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_MAG_OFFSET);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_offset(
    u8 v_mag_offset_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_OFFSET__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_MAG_OFFSET, 
                        v_mag_offset_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_MAG_OFFSET__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);

        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_manual_enable(
    u8 *v_mag_manual_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Mag manual */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_MANUAL_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_manual_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_MAG_MANUAL_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_manual_enable(
    u8 v_mag_manual_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }

    /* write the Mag manual*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_MAG_MANUAL_ENABLE__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    if (com_rslt == SUCCESS) {
        /* set the bit of Mag manual enable*/
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_MAG_MANUAL_ENABLE, 
                        v_mag_manual_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_MAG_MANUAL_ENABLE__REG, 
                        &v_data_u8,
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);

        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    }
    if (com_rslt == SUCCESS) {
        p_bmi160->mag_manual_enable = v_mag_manual_u8;
    } else {
        p_bmi160->mag_manual_enable = E_BMI160_COMM_RES;
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_read_addr(
    u8 *v_mag_read_addr_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the written address*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_READ_ADDR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_read_addr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_READ_ADDR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_read_addr(
    u8 v_mag_read_addr_u8)
{
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* write the Mag read address*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_WRITE_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_READ_ADDR__REG, 
            &v_mag_read_addr_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_write_addr(
    u8 *v_mag_write_addr_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the address of last written */
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_WRITE_ADDR__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_write_addr_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_WRITE_ADDR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_write_addr(
    u8 v_mag_write_addr_u8)
{
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write the data of Mag address to write data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_WRITE_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_WRITE_ADDR__REG, 
            &v_mag_write_addr_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_mag_write_data(
    u8 *v_mag_write_data_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_WRITE_DATA__REG, 
            &v_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_mag_write_data_u8 =
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_WRITE_DATA);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_mag_write_data(
    u8 v_mag_write_data_u8)
{
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt =
        p_bmi160->BMI160_BUS_WRITE_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_WRITE_DATA__REG, 
            &v_mag_write_data_u8,
            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160MAG

#ifdef INCLUDE_BMI160INT
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_0(
    u8 v_enable_u8, u8 *v_intr_enable_zero_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
    
	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* select interrupt to read*/
    switch (v_enable_u8) {
    case BMI160_ANY_MOTION_X_ENABLE:
        /* read the any motion interrupt x data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_zero_u8 =
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE);
        break;
    case BMI160_ANY_MOTION_Y_ENABLE:
        /* read the any motion interrupt y data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_zero_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE);
        break;
    case BMI160_ANY_MOTION_Z_ENABLE:
        /* read the any motion interrupt z data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_zero_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE);
        break;
    case BMI160_DOUBLE_TAP_ENABLE:
        /* read the double tap interrupt data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_zero_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE);
        break;
    case BMI160_SINGLE_TAP_ENABLE:
        /* read the single tap interrupt data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_zero_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE);
        break;
    case BMI160_ORIENT_ENABLE:
        /* read the orient interrupt data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_zero_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE);
        break;
    case BMI160_FLAT_ENABLE:
        /* read the flat interrupt data */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_zero_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_0(
    u8 v_enable_u8, u8 v_intr_enable_zero_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_enable_u8) {
    case BMI160_ANY_MOTION_X_ENABLE:
        /* write any motion x*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE,
                            v_intr_enable_zero_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_0_ANY_MOTION_X_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_ANY_MOTION_Y_ENABLE:
        /* write any motion y*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE,
                            v_intr_enable_zero_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Y_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_ANY_MOTION_Z_ENABLE:
        /* write any motion z*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE,
                            v_intr_enable_zero_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_0_ANY_MOTION_Z_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_DOUBLE_TAP_ENABLE:
        /* write double tap*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE,
                            v_intr_enable_zero_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_0_DOUBLE_TAP_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_SINGLE_TAP_ENABLE:
        /* write single tap */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE,
                            v_intr_enable_zero_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_0_SINGLE_TAP_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_ORIENT_ENABLE:
        /* write orient interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE,
                            v_intr_enable_zero_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_0_ORIENT_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_FLAT_ENABLE:
        /* write flat interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE,
                            v_intr_enable_zero_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_0_FLAT_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_1(
    u8 v_enable_u8, u8 *v_intr_enable_1_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_enable_u8) {
    case BMI160_HIGH_G_X_ENABLE:
        /* read high_g_x interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_1_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE);
        break;
    case BMI160_HIGH_G_Y_ENABLE:
        /* read high_g_y interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_1_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE);
        break;
    case BMI160_HIGH_G_Z_ENABLE:
        /* read high_g_z interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_1_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE);
        break;
    case BMI160_LOW_G_ENABLE:
        /* read low_g interrupt */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_1_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE);
        break;
    case BMI160_DATA_RDY_ENABLE:
        /* read data ready interrupt */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_1_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE);
        break;
    #ifdef FIFO_ENABLE
    case BMI160_FIFO_FULL_ENABLE:
        /* read FIFO full interrupt */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_1_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE);
        break;
    case BMI160_FIFO_WM_ENABLE:
        /* read FIFO water mark interrupt */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_1_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE);
        break;
    #endif //FIFO_ENABLE
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_1(
    u8 v_enable_u8, u8 v_intr_enable_1_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_enable_u8) {
    case BMI160_HIGH_G_X_ENABLE:
        /* write high_g_x interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE,
                            v_intr_enable_1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_1_HIGH_G_X_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_HIGH_G_Y_ENABLE:
        /* write high_g_y interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE,
                            v_intr_enable_1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_1_HIGH_G_Y_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_HIGH_G_Z_ENABLE:
        /* write high_g_z interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE,
                            v_intr_enable_1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_1_HIGH_G_Z_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_LOW_G_ENABLE:
        /* write low_g interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE,
                            v_intr_enable_1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_1_LOW_G_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_DATA_RDY_ENABLE:
        /* write data ready interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE,
                            v_intr_enable_1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_1_DATA_RDY_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    #ifdef FIFO_ENABLE
    case BMI160_FIFO_FULL_ENABLE:
        /* write FIFO full interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE,
                            v_intr_enable_1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_1_FIFO_FULL_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_FIFO_WM_ENABLE:
        /* write FIFO water mark interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE,
                            v_intr_enable_1_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_1_FIFO_WM_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    #endif //FIFO_ENABLE
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_enable_2(
    u8 v_enable_u8, u8 *v_intr_enable_2_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_enable_u8) {
    case BMI160_NOMOTION_X_ENABLE:
        /* read no motion x */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_2_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE);
        break;
    case BMI160_NOMOTION_Y_ENABLE:
        /* read no motion y */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_2_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE);
        break;
    case BMI160_NOMOTION_Z_ENABLE:
        /* read no motion z */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_enable_2_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_enable_2(
    u8 v_enable_u8, u8 v_intr_enable_2_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_enable_u8) {
    case BMI160_NOMOTION_X_ENABLE:
        /* write no motion x */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE,
                            v_intr_enable_2_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_2_NOMOTION_X_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_NOMOTION_Y_ENABLE:
        /* write no motion y */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE,
                            v_intr_enable_2_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_2_NOMOTION_Y_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_NOMOTION_Z_ENABLE:
        /* write no motion z */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE,
                            v_intr_enable_2_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ENABLE_2_NOMOTION_Z_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_detector_enable(
    u8 *v_step_intr_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read the step detector interrupt*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_step_intr_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_detector_enable(
    u8 v_step_intr_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        p_bmi160->BMI160_BUS_READ_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
            &v_data_u8, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE,
                        v_step_intr_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_INTR_ENABLE_2_STEP_DETECTOR_ENABLE__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_edge_ctrl(
    u8 v_channel_u8, u8 *v_intr_edge_ctrl_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    case BMI160_INTR1_EDGE_CTRL:
        /* read the edge trigger interrupt1*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_EDGE_CTRL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_edge_ctrl_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR1_EDGE_CTRL);
        break;
    case BMI160_INTR2_EDGE_CTRL:
        /* read the edge trigger interrupt2*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_EDGE_CTRL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_edge_ctrl_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR2_EDGE_CTRL);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_edge_ctrl(
    u8 v_channel_u8, u8 v_intr_edge_ctrl_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    case BMI160_INTR1_EDGE_CTRL:
        /* write the edge trigger interrupt1*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_EDGE_CTRL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR1_EDGE_CTRL,
                            v_intr_edge_ctrl_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR1_EDGE_CTRL__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_EDGE_CTRL:
        /* write the edge trigger interrupt2*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_EDGE_CTRL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR2_EDGE_CTRL,
                            v_intr_edge_ctrl_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR2_EDGE_CTRL__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_level(
    u8 v_channel_u8, u8 *v_intr_level_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    case BMI160_INTR1_LEVEL:
        /* read the interrupt1 level*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_LEVEL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_level_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR1_LEVEL);
        break;
    case BMI160_INTR2_LEVEL:
        /* read the interrupt2 level*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_LEVEL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_level_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR2_LEVEL);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_level(
    u8 v_channel_u8, u8 v_intr_level_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    case BMI160_INTR1_LEVEL:
        /* write the interrupt1 level*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_LEVEL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR1_LEVEL, 
                            v_intr_level_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR1_LEVEL__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_LEVEL:
        /* write the interrupt2 level*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_LEVEL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR2_LEVEL, 
                            v_intr_level_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR2_LEVEL__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_output_type(
    u8 v_channel_u8, u8 *v_intr_output_type_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    case BMI160_INTR1_OUTPUT_TYPE:
        /* read the output type of interrupt1*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_OUTPUT_TYPE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_output_type_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR1_OUTPUT_TYPE);
        break;
    case BMI160_INTR2_OUTPUT_TYPE:
        /* read the output type of interrupt2*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_OUTPUT_TYPE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_output_type_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR2_OUTPUT_TYPE);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_output_type(
    u8 v_channel_u8, u8 v_intr_output_type_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    case BMI160_INTR1_OUTPUT_TYPE:
        /* write the output type of interrupt1*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_OUTPUT_TYPE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR1_OUTPUT_TYPE,
                            v_intr_output_type_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR1_OUTPUT_TYPE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_OUTPUT_TYPE:
        /* write the output type of interrupt2*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_OUTPUT_TYPE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR2_OUTPUT_TYPE,
                            v_intr_output_type_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR2_OUTPUT_TYPE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_output_enable(
    u8 v_channel_u8, u8 *v_output_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    case BMI160_INTR1_OUTPUT_ENABLE:
        /* read the output enable of interrupt1*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_output_enable_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR1_OUTPUT_ENABLE);
        break;
    case BMI160_INTR2_OUTPUT_ENABLE:
        /* read the output enable of interrupt2*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_OUTPUT_EN__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_output_enable_u8 =
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR2_OUTPUT_EN);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_output_enable(
    u8 v_channel_u8, u8 v_output_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    case BMI160_INTR1_OUTPUT_ENABLE:
        /* write the output enable of interrupt1*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR1_OUTPUT_ENABLE,
                            v_output_enable_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR1_OUTPUT_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    break;
    case BMI160_INTR2_OUTPUT_ENABLE:
        /* write the output enable of interrupt2*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_OUTPUT_EN__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR2_OUTPUT_EN,
                            v_output_enable_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR2_OUTPUT_EN__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
    break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_latch_intr(
    u8 *v_latch_intr_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the latch duration value */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr, 
                BMI160_USER_INTR_LATCH__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_latch_intr_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_LATCH);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_latch_intr(
    u8 v_latch_intr_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
    
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_latch_intr_u8 <= BMI160_MAX_LATCH_INTR) {
        /* write the latch duration value */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_LATCH__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_LATCH, 
                            v_latch_intr_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->
                            dev_addr, 
                            BMI160_USER_INTR_LATCH__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_input_enable(
    u8 v_channel_u8, u8 *v_input_en_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read input enable of interrupt1 and interrupt2*/
    case BMI160_INTR1_INPUT_ENABLE:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_INPUT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_input_en_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR1_INPUT_ENABLE);
        break;
    case BMI160_INTR2_INPUT_ENABLE:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_INPUT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_input_en_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR2_INPUT_ENABLE);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_input_enable(
    u8 v_channel_u8, u8 v_input_en_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /* write input enable of interrup1 and interrupt2*/
    case BMI160_INTR1_INPUT_ENABLE:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR1_INPUT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR1_INPUT_ENABLE, 
                            v_input_en_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR1_INPUT_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_INPUT_ENABLE:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR2_INPUT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR2_INPUT_ENABLE, 
                            v_input_en_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR2_INPUT_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g(
    u8 v_channel_u8, u8 *v_intr_low_g_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    switch (v_channel_u8) {
    /* read the low_g interrupt */
    case BMI160_INTR1_MAP_LOW_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_low_g_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_0_INTR1_LOW_G);
        break;
    case BMI160_INTR2_MAP_LOW_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_low_g_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_2_INTR2_LOW_G);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160PED
BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g(
    u8 v_channel_u8, u8 v_intr_low_g_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    u8 v_step_cnt_stat_u8 = BMI160_INIT_VALUE;
    u8 v_step_det_stat_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    /* check the step detector interrupt enable status*/
    com_rslt = bmi160_get_step_detector_enable(&v_step_det_stat_u8);
    
    /* disable the step detector interrupt */
    if (v_step_det_stat_u8 != BMI160_INIT_VALUE) {
        com_rslt += bmi160_set_step_detector_enable(BMI160_INIT_VALUE);
    }
    
    /* check the step counter interrupt enable status*/
    com_rslt += bmi160_get_step_counter_enable(&v_step_cnt_stat_u8);
    
    /* disable the step counter interrupt */
    if (v_step_cnt_stat_u8 != BMI160_INIT_VALUE) {
        com_rslt += bmi160_set_step_counter_enable(BMI160_INIT_VALUE);
    }
    
    switch (v_channel_u8) {
    /* write the low_g interrupt*/
    case BMI160_INTR1_MAP_LOW_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_LOW_G, 
                            v_intr_low_g_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_LOW_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_LOW_G, 
                            v_intr_low_g_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
        #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
        #endif //INCLUDE_BMI160PMU

    return com_rslt;
}
    #endif //INCLUDE_BMI160PED

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g(
    u8 v_channel_u8, u8 *v_intr_high_g_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}

    /* read the high_g interrupt*/
    switch (v_channel_u8) {
    case BMI160_INTR1_MAP_HIGH_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_high_g_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_0_INTR1_HIGH_G);
        break;
    case BMI160_INTR2_MAP_HIGH_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_high_g_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_2_INTR2_HIGH_G);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g(
    u8 v_channel_u8, u8 v_intr_high_g_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /* write the high_g interrupt*/
    case BMI160_INTR1_MAP_HIGH_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_HIGH_G, 
                            v_intr_high_g_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_0_INTR1_HIGH_G__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_HIGH_G:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_HIGH_G, 
                            v_intr_high_g_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_2_INTR2_HIGH_G__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion(
    u8 v_channel_u8, u8 *v_intr_any_motion_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read the any motion interrupt */
    case BMI160_INTR1_MAP_ANY_MOTION:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_any_motion_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION);
        break;
    case BMI160_INTR2_MAP_ANY_MOTION:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_any_motion_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion(
    u8 v_channel_u8, u8 v_intr_any_motion_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    u8 sig_mot_stat = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    /* read the status of significant motion interrupt */
    com_rslt = bmi160_get_intr_significant_motion_select(&sig_mot_stat);
    
    /* disable the significant motion interrupt */
    if (sig_mot_stat != BMI160_INIT_VALUE)
        com_rslt += bmi160_set_intr_significant_motion_select(
                        BMI160_INIT_VALUE);
    
    switch (v_channel_u8) {
    /* write the any motion interrupt */
    case BMI160_INTR1_MAP_ANY_MOTION:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION,
                            v_intr_any_motion_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_ANY_MOTION:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION,
                            v_intr_any_motion_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_nomotion(
    u8 v_channel_u8, u8 *v_intr_nomotion_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read the no motion interrupt*/
    case BMI160_INTR1_MAP_NOMO:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_nomotion_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_0_INTR1_NOMOTION);
        break;
    case BMI160_INTR2_MAP_NOMO:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_nomotion_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_2_INTR2_NOMOTION);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_nomotion(
    u8 v_channel_u8, u8 v_intr_nomotion_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /* write the no motion interrupt*/
    case BMI160_INTR1_MAP_NOMO:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_NOMOTION,
                            v_intr_nomotion_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_0_INTR1_NOMOTION__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_NOMO:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_NOMOTION,
                            v_intr_nomotion_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_2_INTR2_NOMOTION__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_double_tap(
    u8 v_channel_u8, u8 *v_intr_double_tap_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    case BMI160_INTR1_MAP_DOUBLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_double_tap_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP);
        break;
    case BMI160_INTR2_MAP_DOUBLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_double_tap_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_double_tap(
    u8 v_channel_u8, u8 v_intr_double_tap_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    /* check the p_bmi160 structure for NULL pointer assignment*/
    
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /* set the double tap interrupt */
    case BMI160_INTR1_MAP_DOUBLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP,
                            v_intr_double_tap_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_0_INTR1_DOUBLE_TAP__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_DOUBLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP,
                            v_intr_double_tap_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_2_INTR2_DOUBLE_TAP__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_single_tap(
    u8 v_channel_u8, u8 *v_intr_single_tap_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* reads the single tap interrupt*/
    case BMI160_INTR1_MAP_SINGLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_single_tap_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP);
        break;
    case BMI160_INTR2_MAP_SINGLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_single_tap_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_single_tap(
    u8 v_channel_u8, u8 v_intr_single_tap_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /* write the single tap interrupt */
    case BMI160_INTR1_MAP_SINGLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP,
                            v_intr_single_tap_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_0_INTR1_SINGLE_TAP__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_SINGLE_TAP:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP,
                            v_intr_single_tap_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_2_INTR2_SINGLE_TAP__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient(
    u8 v_channel_u8, u8 *v_intr_orient_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read the orientation interrupt*/
    case BMI160_INTR1_MAP_ORIENT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(  
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_orient_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_0_INTR1_ORIENT);
        break;
    case BMI160_INTR2_MAP_ORIENT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG,
                    &v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_orient_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_2_INTR2_ORIENT);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient(
    u8 v_channel_u8, u8 v_intr_orient_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /* write the orientation interrupt*/
    case BMI160_INTR1_MAP_ORIENT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_ORIENT, 
                            v_intr_orient_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_0_INTR1_ORIENT__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_ORIENT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_ORIENT, 
                            v_intr_orient_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->
                            dev_addr, 
                            BMI160_USER_INTR_MAP_2_INTR2_ORIENT__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat(
    u8 v_channel_u8, u8 *v_intr_flat_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read the flat interrupt*/
    case BMI160_INTR1_MAP_FLAT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_flat_u8 =
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_0_INTR1_FLAT);
        break;
    case BMI160_INTR2_MAP_FLAT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_flat_u8 =
            BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_INTR_MAP_2_INTR2_FLAT);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat(
    u8 v_channel_u8, u8 v_intr_flat_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /* write the flat interrupt */
    case BMI160_INTR1_MAP_FLAT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_0_INTR1_FLAT,
                            v_intr_flat_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_MAP_0_INTR1_FLAT__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_FLAT:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_2_INTR2_FLAT,
                            v_intr_flat_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_MAP_2_INTR2_FLAT__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check*/
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

    #ifdef INCLUDE_BMI160PMU
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_pmu_trig(
    u8 v_channel_u8, u8 *v_intr_pmu_trig_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read the pmu trigger interrupt*/
    case BMI160_INTR1_MAP_PMUTRIG:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_pmu_trig_u8 =
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG);
        break;
    case BMI160_INTR2_MAP_PMUTRIG:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_pmu_trig_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_pmu_trig(
    u8 v_channel_u8, u8 v_intr_pmu_trig_u8)
{
    /* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
	}
    
	switch (v_channel_u8) {
	/* write the pmu trigger interrupt */
	case BMI160_INTR1_MAP_PMUTRIG:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG,
                            v_intr_pmu_trig_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_1_INTR1_PMU_TRIG__REG,
                            &v_data_u8,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
        break;
	case BMI160_INTR2_MAP_PMUTRIG:
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG,
                            v_intr_pmu_trig_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_1_INTR2_PMU_TRIG__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
		}
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
    
	/*Accel and Gyro power mode check */
	if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
		/*interface idle time delay */
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }

    return com_rslt;
}
    #endif //INCLUDE_BMI160PMU

    #ifdef FIFO_ENABLE
BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_fifo_full(
    u8 v_channel_u8, u8 *v_intr_fifo_full_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	/* check the p_bmi160 structure for NULL pointer assignment*/
	
    if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read the FIFO full interrupt */
    case BMI160_INTR1_MAP_FIFO_FULL:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_fifo_full_u8 =
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL);
        break;
    case BMI160_INTR2_MAP_FIFO_FULL:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_fifo_full_u8 =
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_fifo_full(
    u8 v_channel_u8, u8 v_intr_fifo_full_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* write the FIFO full interrupt */
    case BMI160_INTR1_MAP_FIFO_FULL:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL,
                            v_intr_fifo_full_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_MAP_1_INTR1_FIFO_FULL__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_FIFO_FULL:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL,
                            v_intr_fifo_full_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_MAP_1_INTR2_FIFO_FULL__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_fifo_wm(
    u8 v_channel_u8, u8 *v_intr_fifo_wm_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* read the FIFO water mark interrupt */
    case BMI160_INTR1_MAP_FIFO_WM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_fifo_wm_u8 =
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM);
        break;
    case BMI160_INTR2_MAP_FIFO_WM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_fifo_wm_u8 =
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_fifo_wm(
    u8 v_channel_u8, u8 v_intr_fifo_wm_u8)
{
    /* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /* write the FIFO water mark interrupt */
    case BMI160_INTR1_MAP_FIFO_WM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM,
                            v_intr_fifo_wm_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_MAP_1_INTR1_FIFO_WM__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_FIFO_WM:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM,
                            v_intr_fifo_wm_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_MAP_1_INTR2_FIFO_WM__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}
    #endif //FIFO_ENABLE

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_data_rdy(
    u8 v_channel_u8, u8 *v_intr_data_rdy_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    switch (v_channel_u8) {
    /*Read Data Ready interrupt*/
    case BMI160_INTR1_MAP_DATA_RDY:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_data_rdy_u8 = 
            BMI160_GET_BITSLICE(v_data_u8, 
                                BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY);
        break;
    case BMI160_INTR2_MAP_DATA_RDY:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        *v_intr_data_rdy_u8 = 
            BMI160_GET_BITSLICE(v_data_u8,
                                BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY);
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_data_rdy(
    u8 v_channel_u8, u8 v_intr_data_rdy_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    switch (v_channel_u8) {
    /*Write Data Ready interrupt*/
    case BMI160_INTR1_MAP_DATA_RDY:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY,
                            v_intr_data_rdy_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_1_INTR1_DATA_RDY__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    case BMI160_INTR2_MAP_DATA_RDY:
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr, 
                    BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY,
                            v_intr_data_rdy_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr, 
                            BMI160_USER_INTR_MAP_1_INTR2_DATA_RDY__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
        break;
    default:
        com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
    }
    
    #ifdef INCLUDE_BMI160PMU
    /*Accel and Gyro power mode check */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_source(
    u8 *v_tap_source_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the tap source interrupt */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_source_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_source(
    u8 v_tap_source_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_tap_source_u8 <= BMI160_MAX_VALUE_SOURCE_INTR) {
        /* write the tap source interrupt */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE,
                            v_tap_source_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_DATA_0_INTR_TAP_SOURCE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Check for the power mode of Accel and
            gyro not in normal mode */
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
	}
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_high_source(
    u8 *v_low_high_source_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the high_low_g source interrupt */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_low_high_source_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_high_source(
    u8 v_low_high_source_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    if (v_low_high_source_u8 <= BMI160_MAX_VALUE_SOURCE_INTR) {
        /* write the high_low_g source interrupt */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE,
                            v_low_high_source_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_DATA_0_INTR_LOW_HIGH_SOURCE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Check for the power mode of Accel and gyro not in normal mode */
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_motion_source(
    u8 *v_motion_source_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the any/no motion interrupt  */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_motion_source_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_motion_source(
    u8 v_motion_source_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_motion_source_u8 <= BMI160_MAX_VALUE_SOURCE_INTR) {
        /* write the any/no motion interrupt  */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE,
                            v_motion_source_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_DATA_1_INTR_MOTION_SOURCE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Check for the power mode of Accel and gyro not in normal mode */
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_durn(
    u8 *v_low_g_durn_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the low_g interrupt */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_low_g_durn_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_durn(
    u8 v_low_g_durn_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write the low_g interrupt */
    com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_0_INTR_LOW_DURN__REG,
                &v_low_g_durn_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    #ifdef INCLUDE_BMI160PMU
    /*Check for the power mode of Accel and gyro not in normal mode */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_thres(
    u8 *v_low_g_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read low_g threshold */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_low_g_thres_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_thres(
    u8 v_low_g_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    /* write low_g threshold */
    com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_1_INTR_LOW_THRES__REG,
                &v_low_g_thres_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
    /*Check for the power mode of Accel and gyro not in normal mode */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_hyst(
    u8 *v_low_hyst_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read low_g hysteresis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_low_hyst_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_hyst(
    u8 v_low_hyst_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write low_g hysteresis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST,
                        v_low_hyst_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_HYST__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
        /*Check for the power mode of Accel and gyro not in normal mode */
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_low_g_mode(
    u8 *v_low_g_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /*read Low-g interrupt mode*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_low_g_mode_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_low_g_mode(
    u8 v_low_g_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_low_g_mode_u8 <= BMI160_MAX_VALUE_LOW_G_MODE) {
        /*write Low-g interrupt mode*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE,
                            v_low_g_mode_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_LOWHIGH_2_INTR_LOW_G_MODE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Check for the power mode of Accel and gyro not in normal mode */
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_hyst(
    u8 *v_high_g_hyst_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read high_g hysteresis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_hyst_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_hyst(
    u8 v_high_g_hyst_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
	}
    
    /* write high_g hysteresis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST,
                        v_high_g_hyst_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_INTR_LOWHIGH_2_INTR_HIGH_G_HYST__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
        /*Check for the power mode of Accel and gyro not in normal mode */
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_durn(
    u8 *v_high_g_durn_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read high_g duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_durn_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_durn(
    u8 v_high_g_durn_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write high_g duration*/
    com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_3_INTR_HIGH_G_DURN__REG,
                &v_high_g_durn_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    #ifdef INCLUDE_BMI160PMU
    /*Check for the power mode of Accel and gyro not in normal mode */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_high_g_thres(
    u8 *v_high_g_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_high_g_thres_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_high_g_thres(
    u8 v_high_g_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_LOWHIGH_4_INTR_HIGH_THRES__REG,
                &v_high_g_thres_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    #ifdef INCLUDE_BMI160PMU
    /* Check for the power mode of Accel and gyro not in normal mode */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion_durn(
    u8 *v_any_motion_durn_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read any motion duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_any_motion_durn_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion_durn(
    u8 v_any_motion_durn_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write any motion duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN,
                        v_any_motion_durn_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_INTR_MOTION_0_INTR_ANY_MOTION_DURN__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
        /*Check for the power mode of Accel and gyro not in normal mode */
        if (bmi160_power_mode_status_u8_g !=BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
	}

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_durn(
    u8 *v_slow_no_motion_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }

    /* read slow no motion duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_slow_no_motion_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN);

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_durn(
    u8 v_slow_no_motion_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    /* write slow no motion duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN,
                        v_slow_no_motion_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_INTR_MOTION_0_INTR_SLOW_NO_MOTION_DURN__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
        /*Check for the power mode of Accel and gyro not in normal mode */
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_any_motion_thres(
    u8 *v_any_motion_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read any motion threshold*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_any_motion_thres_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_any_motion_thres(
    u8 v_any_motion_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write any motion threshold*/
    com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_1_INTR_ANY_MOTION_THRES__REG,
                &v_any_motion_thres_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    #ifdef INCLUDE_BMI160PMU
    /* Check for the power mode of Accel and gyro not in normal mode */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_thres(
    u8 *v_slow_no_motion_thres_u8)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }

    /* read slow no motion threshold*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_slow_no_motion_thres_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES);

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_thres(
    u8 v_slow_no_motion_thres_u8)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    /* write slow no motion threshold*/
    com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_2_INTR_SLOW_NO_MOTION_THRES__REG,
                &v_slow_no_motion_thres_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    #ifdef INCLUDE_BMI160PMU
    /*Check for the power mode of Accel and gyro not in normal mode */
    if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
        /*interface idle time delay */
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    #endif //INCLUDE_BMI160PMU

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_slow_no_motion_select(
    u8 *v_intr_slow_no_motion_select_u8)
{
    BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
	}
    
    /* read slow no motion select*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_intr_slow_no_motion_select_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT);

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_slow_no_motion_select(
    u8 v_intr_slow_no_motion_select_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    if (v_intr_slow_no_motion_select_u8 <= BMI160_MAX_VALUE_NO_MOTION) {
        /* write slow no motion select*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT,
                            v_intr_slow_no_motion_select_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_MOTION_3_INTR_SLOW_NO_MOTION_SELECT__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Check for the power mode of Accel and gyro not in normal mode */
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_select(
    u8 *v_intr_significant_motion_select_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the significant or any motion interrupt*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_intr_significant_motion_select_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_select(
    u8 v_intr_significant_motion_select_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    if (v_intr_significant_motion_select_u8 <= BMI160_MAX_VALUE_SIGNIFICANT_MOTION) {
        /* write the significant or any motion interrupt*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT,
                            v_intr_significant_motion_select_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_SIGNIFICATION_MOTION_SELECT__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_unmap_significant_motion_intr(
    u8 v_significant_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_any_motion_intr1_stat_u8 = V_ANY_MOTION_INTR_STAT;
    u8 v_any_motion_intr2_stat_u8 = V_ANY_MOTION_INTR_STAT;
    u8 v_any_motion_axis_stat_u8 = V_ANY_MOTION_AXIS_STAT;
    u8 v_data_u8 = BMI160_INIT_VALUE;

	switch (v_significant_u8) {
	case BMI160_MAP_INTR1:
		/* interrupt */
		com_rslt = bmi160_read_reg(
                    BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
                    &v_data_u8, 
                    BMI160_ASSIGN_DATA);
		v_data_u8 &= ~(v_any_motion_intr1_stat_u8);
		
        /* map the signification interrupt to any-motion interrupt1*/
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
                        &v_data_u8, 
                        BMI160_ASSIGN_DATA);
		p_bmi160->delay_msec(BMI160_ASSIGN_DATA);
		
        /* axis*/
		com_rslt = bmi160_read_reg(
                    BMI160_USER_INTR_ENABLE_0_ADDR,
                    &v_data_u8, 
                    BMI160_ASSIGN_DATA);
		v_data_u8 &= ~(v_any_motion_axis_stat_u8);
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_ENABLE_0_ADDR,
                        &v_data_u8, 
                        BMI160_ASSIGN_DATA);
		p_bmi160->delay_msec(BMI160_ASSIGN_DATA);
        break;
	case BMI160_MAP_INTR2:
		/* map the signification interrupt to any-motion interrupt2*/
		com_rslt = bmi160_read_reg(
                    BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
                    &v_data_u8, 
                    BMI160_ASSIGN_DATA);
		v_data_u8 &= ~(v_any_motion_intr2_stat_u8);
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
                        &v_data_u8, 
                        BMI160_ASSIGN_DATA);
		p_bmi160->delay_msec(BMI160_ASSIGN_DATA);
		
        /* axis*/
		com_rslt = bmi160_read_reg(
                    BMI160_USER_INTR_ENABLE_0_ADDR,
                    &v_data_u8, 
                    BMI160_ASSIGN_DATA);
		v_data_u8 &= ~(v_any_motion_axis_stat_u8);
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_ENABLE_0_ADDR,
                        &v_data_u8, 
                        BMI160_ASSIGN_DATA);
		p_bmi160->delay_msec(BMI160_ASSIGN_DATA);
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
		
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_skip(
    u8 *v_int_sig_mot_skip_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read significant skip time*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_int_sig_mot_skip_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_skip(
    u8 v_int_sig_mot_skip_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_int_sig_mot_skip_u8 <= BMI160_MAX_UNDER_SIG_MOTION) {
        /* write significant skip time*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP,
                            v_int_sig_mot_skip_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_SIGNIFICANT_MOTION_SKIP__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_significant_motion_proof(
    u8 *v_significant_motion_proof_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read significant proof time */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_significant_motion_proof_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_significant_motion_proof(
    u8 v_significant_motion_proof_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_significant_motion_proof_u8 <= BMI160_MAX_UNDER_SIG_MOTION) {
        /* write significant proof time */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF,
                            v_significant_motion_proof_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_SIGNIFICANT_MOTION_PROOF__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_durn(
    u8 *v_tap_durn_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read tap duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_durn_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_DURN);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_durn(
    u8 v_tap_durn_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_tap_durn_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_tap_durn_u8 <= BMI160_MAX_TAP_TURN) {
        switch (v_tap_durn_u8) {
        case BMI160_TAP_DURN_50MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_50MS;
            break;
        case BMI160_TAP_DURN_100MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_100MS;
            break;
        case BMI160_TAP_DURN_150MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_150MS;
            break;
        case BMI160_TAP_DURN_200MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_200MS;
            break;
        case BMI160_TAP_DURN_250MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_250MS;
            break;
        case BMI160_TAP_DURN_375MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_375MS;
            break;
        case BMI160_TAP_DURN_500MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_500MS;
            break;
        case BMI160_TAP_DURN_700MS:
            v_data_tap_durn_u8 = BMI160_TAP_DURN_700MS;
            break;
        default:
            break;
        }
        
        /* write tap duration*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_DURN,
                            v_data_tap_durn_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_DURN__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_shock(
    u8 *v_tap_shock_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read tap shock duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_shock_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_shock(
    u8 v_tap_shock_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_tap_shock_u8 <= BMI160_MAX_VALUE_TAP_SHOCK) {
        /* write tap shock duration*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK,
                            v_tap_shock_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_SHOCK__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_quiet(
    u8 *v_tap_quiet_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read tap quiet duration*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_quiet_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_quiet(
    u8 v_tap_quiet_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_tap_quiet_u8 <= BMI160_MAX_VALUE_TAP_QUIET) {
        /* write tap quiet duration*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET,
                            v_tap_quiet_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_TAP_0_INTR_TAP_QUIET__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_tap_thres(
    u8 *v_tap_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read tap threshold*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_tap_thres_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_TAP_1_INTR_TAP_THRES);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_tap_thres(
    u8 v_tap_thres_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write tap threshold*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_INTR_TAP_1_INTR_TAP_THRES,
                        v_tap_thres_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_INTR_TAP_1_INTR_TAP_THRES__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_mode(
    u8 *v_orient_mode_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read orientation threshold*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_mode_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_mode(
    u8 v_orient_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_orient_mode_u8 <= BMI160_MAX_ORIENT_MODE) {
        /* write orientation threshold*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE,
                            v_orient_mode_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_MODE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_blocking(
    u8 *v_orient_blocking_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read orient blocking mode*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_blocking_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_blocking(
    u8 v_orient_blocking_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
	}
    
	if (v_orient_blocking_u8 <= BMI160_MAX_ORIENT_BLOCKING) {
		/* write orient blocking mode*/
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING,
                            v_orient_blocking_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_BLOCKING__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
			/*Accel and Gyro power mode check*/
			if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
				/*interface idle time delay */
				p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
		}
	} else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
	}

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_hyst(
    u8 *v_orient_hyst_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read orient hysteresis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_hyst_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_hyst(
    u8 v_orient_hyst_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* write orient hysteresis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST,
                        v_orient_hyst_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_INTR_ORIENT_0_INTR_ORIENT_HYST__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_theta(
    u8 *v_orient_theta_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Orient blocking angle*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_theta_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_theta(
    u8 v_orient_theta_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    if (v_orient_theta_u8 <= BMI160_MAX_ORIENT_THETA) {
        /* write Orient blocking angle*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA,
                            v_orient_theta_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_THETA__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_ud_enable(
    u8 *v_orient_ud_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read orient up/down enable*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_ud_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_ud_enable(
    u8 v_orient_ud_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
	}
    
	if (v_orient_ud_u8 <= BMI160_MAX_VALUE_ORIENT_UD) {
		/* write orient up/down enable */
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE,
                            v_orient_ud_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_UD_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
			/*Accel and Gyro power mode check*/
			if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
				/*interface idle time delay */
				p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
		}
	} else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
	}

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_orient_axes_enable(
    u8 *v_orient_axes_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read orientation axes changes  */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_orient_axes_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_orient_axes_enable(
    u8 v_orient_axes_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    if (v_orient_axes_u8 <= BMI160_MAX_VALUE_ORIENT_AXES) {
        /*write orientation axes changes  */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX,
                            v_orient_axes_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_ORIENT_1_INTR_ORIENT_AXES_EX__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_theta(
    u8 *v_flat_theta_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Flat angle*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_flat_theta_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_theta(
    u8 v_flat_theta_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_flat_theta_u8 <= BMI160_MAX_FLAT_THETA) {
        /* write Flat angle */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA,
                            v_flat_theta_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_FLAT_0_INTR_FLAT_THETA__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_hold(
    u8 *v_flat_hold_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read flat hold time*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_flat_hold_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_hold(
    u8 v_flat_hold_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_flat_hold_u8 <= BMI160_MAX_FLAT_HOLD) {
        /* write flat hold time*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD,
                            v_flat_hold_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_FLAT_1_INTR_FLAT_HOLD__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_intr_flat_hyst(
    u8 *v_flat_hyst_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
    
	u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the flat hysteresis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG,
                &v_data_u8,
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_flat_hyst_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_intr_flat_hyst(
    u8 v_flat_hyst_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    if (v_flat_hyst_u8 <= BMI160_MAX_FLAT_HYST) {
        /* read the flat hysteresis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST,
                            v_flat_hyst_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_INTR_FLAT_1_INTR_FLAT_HYST__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160INT

#ifdef INCLUDE_BMI160FOC
    #ifdef INCLUDE_BMI160ACC
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_z(
    u8 *v_foc_accel_z_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Accel offset compensation for z axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_FOC_ACCEL_Z__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_foc_accel_z_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FOC_ACCEL_Z);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_z(
    u8 v_foc_accel_z_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write the Accel offset compensation for z axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_FOC_ACCEL_Z__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_FOC_ACCEL_Z,
                        v_foc_accel_z_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FOC_ACCEL_Z__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_y(
    u8 *v_foc_accel_y_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Accel offset compensation for y axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_FOC_ACCEL_Y__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_foc_accel_y_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_FOC_ACCEL_Y);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_y(
    u8 v_foc_accel_y_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_foc_accel_y_u8 <= BMI160_MAX_ACCEL_FOC) {
        /* write the Accel offset compensation for y axis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FOC_ACCEL_Y__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FOC_ACCEL_Y,
                            v_foc_accel_y_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FOC_ACCEL_Y__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_accel_x(
    u8 *v_foc_accel_x_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the Accel offset compensation for x axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_FOC_ACCEL_X__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_foc_accel_x_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FOC_ACCEL_X);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_accel_x(
    u8 v_foc_accel_x_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_foc_accel_x_u8 <= BMI160_MAX_ACCEL_FOC) {
        /* write the Accel offset compensation for x axis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FOC_ACCEL_X__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FOC_ACCEL_X,
                            v_foc_accel_x_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FOC_ACCEL_X__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_foc_trigger(
    u8 v_axis_u8, u8 v_foc_accel_u8, s8 *v_accel_offset_s8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    s8 v_status_s8 = SUCCESS;
    u8 v_timeout_u8 = BMI160_INIT_VALUE;
    s8 v_foc_accel_offset_x_s8  = BMI160_INIT_VALUE;
    s8 v_foc_accel_offset_y_s8 =  BMI160_INIT_VALUE;
    s8 v_foc_accel_offset_z_s8 =  BMI160_INIT_VALUE;
    u8 focstatus = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    v_status_s8 = bmi160_set_accel_offset_enable(ACCEL_OFFSET_ENABLE);
    if (v_status_s8 == SUCCESS) {
        switch (v_axis_u8) {
        case FOC_X_AXIS:
            com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FOC_ACCEL_X__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
            if (com_rslt == SUCCESS) {
                v_data_u8 = BMI160_SET_BITSLICE(
                                v_data_u8,
                                BMI160_USER_FOC_ACCEL_X,
                                v_foc_accel_u8);
                com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                                p_bmi160->dev_addr,
                                BMI160_USER_FOC_ACCEL_X__REG,
                                &v_data_u8, 
                                BMI160_GEN_READ_WRITE_DATA_LENGTH);
            }

            /* trigger the FOC */
            com_rslt += bmi160_set_command_register(START_FOC_ACCEL_GYRO);

            com_rslt += bmi160_get_foc_rdy(&focstatus);
            if (com_rslt != SUCCESS || focstatus != BMI160_FOC_STAT_HIGH) {
                while (com_rslt != SUCCESS ||
                       (focstatus != BMI160_FOC_STAT_HIGH &&
                        v_timeout_u8 < BMI160_MAXIMUM_TIMEOUT)) {
                    p_bmi160->delay_msec(BMI160_DELAY_SETTLING_TIME);
                    com_rslt = bmi160_get_foc_rdy(&focstatus);
                    v_timeout_u8++;
                }
            }
            if (com_rslt == SUCCESS && focstatus == BMI160_FOC_STAT_HIGH) {
                com_rslt += bmi160_get_accel_offset_compensation_xaxis(
                                &v_foc_accel_offset_x_s8);
                *v_accel_offset_s8 = v_foc_accel_offset_x_s8;
            }
            break;
        case FOC_Y_AXIS:
            com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FOC_ACCEL_Y__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
            if (com_rslt == SUCCESS) {
                v_data_u8 = BMI160_SET_BITSLICE(
                                v_data_u8,
                                BMI160_USER_FOC_ACCEL_Y,
                                v_foc_accel_u8);
                com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                                p_bmi160->dev_addr,
                                BMI160_USER_FOC_ACCEL_Y__REG,
                                &v_data_u8, 
                                BMI160_GEN_READ_WRITE_DATA_LENGTH);
            }

            /* trigger the FOC */
            com_rslt += bmi160_set_command_register(START_FOC_ACCEL_GYRO);

            com_rslt += bmi160_get_foc_rdy(&focstatus);
            if (com_rslt != SUCCESS || focstatus != BMI160_FOC_STAT_HIGH) {
                while (com_rslt != SUCCESS ||
                       (focstatus != BMI160_FOC_STAT_HIGH &&
                        v_timeout_u8 < BMI160_MAXIMUM_TIMEOUT)) {
                    p_bmi160->delay_msec(BMI160_DELAY_SETTLING_TIME);
                    com_rslt = bmi160_get_foc_rdy(&focstatus);
                    v_timeout_u8++;
                }
            }
            if (com_rslt == SUCCESS && focstatus == BMI160_FOC_STAT_HIGH) {
                com_rslt += bmi160_get_accel_offset_compensation_yaxis(
                                &v_foc_accel_offset_y_s8);
                *v_accel_offset_s8 = v_foc_accel_offset_y_s8;
            }
            break;
        case FOC_Z_AXIS:
            com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FOC_ACCEL_Z__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
            if (com_rslt == SUCCESS) {
                v_data_u8 = BMI160_SET_BITSLICE(
                                v_data_u8,
                                BMI160_USER_FOC_ACCEL_Z,
                                v_foc_accel_u8);
                com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                                p_bmi160->dev_addr,
                                BMI160_USER_FOC_ACCEL_Z__REG,
                                &v_data_u8, 
                                BMI160_GEN_READ_WRITE_DATA_LENGTH);
            }

            /* trigger the FOC */
            com_rslt += bmi160_set_command_register(START_FOC_ACCEL_GYRO);

            com_rslt += bmi160_get_foc_rdy(&focstatus);
            if (com_rslt != SUCCESS || focstatus != BMI160_FOC_STAT_HIGH) {
                while (com_rslt != SUCCESS ||
                       (focstatus != BMI160_FOC_STAT_HIGH &&
                        v_timeout_u8 < BMI160_MAXIMUM_TIMEOUT)) {
                    p_bmi160->delay_msec(BMI160_DELAY_SETTLING_TIME);
                    com_rslt = bmi160_get_foc_rdy(&focstatus);
                    v_timeout_u8++;
                }
            }
            if (com_rslt == SUCCESS && focstatus == BMI160_FOC_STAT_HIGH) {
                com_rslt += bmi160_get_accel_offset_compensation_zaxis(
                                &v_foc_accel_offset_z_s8);
                *v_accel_offset_s8 = v_foc_accel_offset_z_s8;
            }
            break;
        default:
            break;
        }
    } else {
        com_rslt =  ERROR;
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_accel_foc_trigger_xyz(
    u8 v_foc_accel_x_u8, u8 v_foc_accel_y_u8, u8 v_foc_accel_z_u8,
    s8 *v_accel_off_x_s8, s8 *v_accel_off_y_s8, s8 *v_accel_off_z_s8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 focx = BMI160_INIT_VALUE;
    u8 focy = BMI160_INIT_VALUE;
    u8 focz = BMI160_INIT_VALUE;
    s8 v_foc_accel_offset_x_s8 = BMI160_INIT_VALUE;
    s8 v_foc_accel_offset_y_s8 = BMI160_INIT_VALUE;
    s8 v_foc_accel_offset_z_s8 = BMI160_INIT_VALUE;
    u8 v_status_s8 = SUCCESS;
    u8 v_timeout_u8 = BMI160_INIT_VALUE;
    u8 focstatus = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
	}
    
    v_status_s8 = bmi160_set_accel_offset_enable(ACCEL_OFFSET_ENABLE);
    if (v_status_s8 == SUCCESS) {
        /* foc x axis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FOC_ACCEL_X__REG,
                    &focx, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            focx = BMI160_SET_BITSLICE(
                    focx,
                    BMI160_USER_FOC_ACCEL_X,
                    v_foc_accel_x_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FOC_ACCEL_X__REG,
                            &focx, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }

        /* foc y axis*/
        com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FOC_ACCEL_Y__REG,
                        &focy, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            focy = BMI160_SET_BITSLICE(
                    focy,
                    BMI160_USER_FOC_ACCEL_Y,
                    v_foc_accel_y_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FOC_ACCEL_Y__REG,
                            &focy, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }

        /* foc z axis*/
        com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_FOC_ACCEL_Z__REG,
                        &focz, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            focz = BMI160_SET_BITSLICE(
                    focz,
                    BMI160_USER_FOC_ACCEL_Z,
                    v_foc_accel_z_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FOC_ACCEL_Z__REG,
                            &focz, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }

        /* trigger the FOC */
        com_rslt += bmi160_set_command_register(START_FOC_ACCEL_GYRO);

        com_rslt += bmi160_get_foc_rdy(&focstatus);
        if (com_rslt != SUCCESS || focstatus != BMI160_FOC_STAT_HIGH) {
            while (com_rslt != SUCCESS ||
                   (focstatus != BMI160_FOC_STAT_HIGH &&
                    v_timeout_u8 < BMI160_MAXIMUM_TIMEOUT)) {
                p_bmi160->delay_msec(BMI160_DELAY_SETTLING_TIME);
                com_rslt = bmi160_get_foc_rdy(&focstatus);
                v_timeout_u8++;
            }
        }
        if (com_rslt == SUCCESS &&
            focstatus == BMI160_GEN_READ_WRITE_DATA_LENGTH) {
            com_rslt += bmi160_get_accel_offset_compensation_xaxis(
                            &v_foc_accel_offset_x_s8);
            *v_accel_off_x_s8 = v_foc_accel_offset_x_s8;
            com_rslt += bmi160_get_accel_offset_compensation_yaxis(
                            &v_foc_accel_offset_y_s8);
            *v_accel_off_y_s8 = v_foc_accel_offset_y_s8;
            com_rslt += bmi160_get_accel_offset_compensation_zaxis(
                            &v_foc_accel_offset_z_s8);
            *v_accel_off_z_s8 = v_foc_accel_offset_z_s8;
        }
    } else {
        com_rslt =  ERROR;
    }

    return com_rslt;
}
    #endif //INCLUDE_BMI160ACC

    #ifdef INCLUDE_BMI160GYR
BMI160_RETURN_FUNCTION_TYPE bmi160_get_foc_gyro_enable(
    u8 *v_foc_gyro_u8)
{
	/* used to return the status of bus communication */
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the gyro fast offset enable*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_FOC_GYRO_ENABLE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_foc_gyro_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_FOC_GYRO_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_foc_gyro_enable(
    u8 v_foc_gyro_u8, s16 *v_gyro_off_x_s16, s16 *v_gyro_off_y_s16,
    s16 *v_gyro_off_z_s16)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;
    u8 v_status_s8 = SUCCESS;
    u8 v_timeout_u8 = BMI160_INIT_VALUE;
    s16 offsetx = BMI160_INIT_VALUE;
    s16 offsety = BMI160_INIT_VALUE;
    s16 offsetz = BMI160_INIT_VALUE;
    u8 focstatus = BMI160_INIT_VALUE;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    v_status_s8 = bmi160_set_gyro_offset_enable(GYRO_OFFSET_ENABLE);
    if (v_status_s8 == SUCCESS) {
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_FOC_GYRO_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_FOC_GYRO_ENABLE,
                            v_foc_gyro_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_FOC_GYRO_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
        }

        /* trigger the FOC */
        com_rslt += bmi160_set_command_register(START_FOC_ACCEL_GYRO);

        com_rslt += bmi160_get_foc_rdy(&focstatus);
        if (com_rslt != SUCCESS || focstatus != BMI160_FOC_STAT_HIGH) {
            while (com_rslt != SUCCESS ||
                   (focstatus != BMI160_FOC_STAT_HIGH &&
                    v_timeout_u8 < BMI160_MAXIMUM_TIMEOUT)) {
                p_bmi160->delay_msec(BMI160_DELAY_SETTLING_TIME);
                com_rslt = bmi160_get_foc_rdy(&focstatus);
                v_timeout_u8++;
            }
        }
        if (com_rslt == SUCCESS && focstatus == BMI160_FOC_STAT_HIGH) {
            com_rslt += bmi160_get_gyro_offset_compensation_xaxis(&offsetx);
            *v_gyro_off_x_s16 = offsetx;

            com_rslt += bmi160_get_gyro_offset_compensation_yaxis(&offsety);
            *v_gyro_off_y_s16 = offsety;

            com_rslt +=bmi160_get_gyro_offset_compensation_zaxis(&offsetz);
            *v_gyro_off_z_s16 = offsetz;
        }
    } else {
        com_rslt = ERROR;
    }

    return com_rslt;
}
    #endif //INCLUDE_BMI160GYR
#endif //INCLUDE_BMI160FOC

#if MOTION_IF == SPI
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spi3(
    u8 *v_spi3_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read SPI mode*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_IF_CONFIG_SPI3__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_spi3_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_IF_CONFIG_SPI3);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_spi3(
    u8 v_spi3_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_spi3_u8 <= BMI160_MAX_VALUE_SPI3) {
        /* write SPI mode*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_IF_CONFIG_SPI3__REG,
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_IF_CONFIG_SPI3,
                            v_spi3_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_IF_CONFIG_SPI3__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#elif MOTION_IF == I2C
BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_wdt_select(
    u8 *v_i2c_wdt_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read I2C watch dog timer */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_i2c_wdt_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_IF_CONFIG_I2C_WDT_SELECT);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_wdt_select(
    u8 v_i2c_wdt_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_i2c_wdt_u8 <= BMI160_MAX_VALUE_I2C_WDT) {
        /* write I2C watch dog timer */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_IF_CONFIG_I2C_WDT_SELECT,
                            v_i2c_wdt_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_IF_CONFIG_I2C_WDT_SELECT__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_i2c_wdt_enable(
    u8 *v_i2c_wdt_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read i2c watch dog enable */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_i2c_wdt_u8 = 
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_i2c_wdt_enable(
    u8 v_i2c_wdt_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_i2c_wdt_u8 <= BMI160_MAX_VALUE_I2C_WDT) {
        /* write i2c watch dog enable */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE,
                            v_i2c_wdt_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_IF_CONFIG_I2C_WDT_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

#ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
#endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_if_mode(
    u8 *v_if_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read if mode*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_IF_CONFIG_IF_MODE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_if_mode_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_IF_CONFIG_IF_MODE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_if_mode(
    u8 v_if_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_if_mode_u8 <= BMI160_MAX_IF_MODE) {
        /* write if mode*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_IF_CONFIG_IF_MODE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_IF_CONFIG_IF_MODE,
                            v_if_mode_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_IF_CONFIG_IF_MODE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#endif //MOTION_IF == I2C

#ifdef INCLUDE_BMI160GYR
    #ifdef INCLUDE_BMI160PMU
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_sleep_trigger(
    u8 *v_gyro_sleep_trigger_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read gyro sleep trigger */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_GYRO_SLEEP_TRIGGER__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_sleep_trigger_u8 =
        BMI160_GET_BITSLICE(v_data_u8,
                            BMI160_USER_GYRO_SLEEP_TRIGGER);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_sleep_trigger(
    u8 v_gyro_sleep_trigger_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_gyro_sleep_trigger_u8 <= BMI160_MAX_GYRO_SLEEP_TRIGGER) {
        /* write gyro sleep trigger */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_SLEEP_TRIGGER__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_SLEEP_TRIGGER,
                            v_gyro_sleep_trigger_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_SLEEP_TRIGGER__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_wakeup_trigger(
    u8 *v_gyro_wakeup_trigger_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read gyro wakeup trigger */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_GYRO_WAKEUP_TRIGGER__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_wakeup_trigger_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_WAKEUP_TRIGGER);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_wakeup_trigger(
    u8 v_gyro_wakeup_trigger_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_gyro_wakeup_trigger_u8 <= BMI160_MAX_GYRO_WAKEUP_TRIGGER) {
        /* write gyro wakeup trigger */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_WAKEUP_TRIGGER__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_WAKEUP_TRIGGER,
                            v_gyro_wakeup_trigger_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_WAKEUP_TRIGGER__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_sleep_state(
    u8 *v_gyro_sleep_state_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;

	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read gyro sleep state*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_GYRO_SLEEP_STATE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_sleep_state_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_SLEEP_STATE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_sleep_state(
    u8 v_gyro_sleep_state_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_gyro_sleep_state_u8 <= BMI160_MAX_VALUE_SLEEP_STATE) {
        /* write gyro sleep state*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_SLEEP_STATE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_SLEEP_STATE,
                            v_gyro_sleep_state_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_SLEEP_STATE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_wakeup_intr(
    u8 *v_gyro_wakeup_intr_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read gyro wakeup interrupt */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_GYRO_WAKEUP_INTR__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_wakeup_intr_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_WAKEUP_INTR);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_wakeup_intr(
    u8 v_gyro_wakeup_intr_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_gyro_wakeup_intr_u8 <= BMI160_MAX_VALUE_WAKEUP_INTR) {
        /* write gyro wakeup interrupt */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_WAKEUP_INTR__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_WAKEUP_INTR,
                            v_gyro_wakeup_intr_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_WAKEUP_INTR__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
    #endif //INCLUDE_BMI160PMU
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_axis(
    u8 *v_accel_selftest_axis_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Accel self test axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_ACCEL_SELFTEST_AXIS__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_selftest_axis_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ACCEL_SELFTEST_AXIS);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_axis(
    u8 v_accel_selftest_axis_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_accel_selftest_axis_u8 <= BMI160_MAX_ACCEL_SELFTEST_AXIS) {
        /* write Accel self test axis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_ACCEL_SELFTEST_AXIS__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_ACCEL_SELFTEST_AXIS,
                            v_accel_selftest_axis_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_ACCEL_SELFTEST_AXIS__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
	}

	return com_rslt;
}

#ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_sign(
    u8 *v_accel_selftest_sign_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Accel self test axis sign*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_ACCEL_SELFTEST_SIGN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_selftest_sign_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_ACCEL_SELFTEST_SIGN);

	return com_rslt;
}
#endif

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_sign(
    u8 v_accel_selftest_sign_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_accel_selftest_sign_u8 <= BMI160_MAX_VALUE_SELFTEST_SIGN) {
        /* write Accel self test axis sign*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_ACCEL_SELFTEST_SIGN__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_ACCEL_SELFTEST_SIGN,
                            v_accel_selftest_sign_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_ACCEL_SELFTEST_SIGN__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_selftest_amp(
    u8 *v_accel_selftest_amp_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read  self test amplitude*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_SELFTEST_AMP__REG,
    &v_data_u8, BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_selftest_amp_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_SELFTEST_AMP);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_selftest_amp(
    u8 v_accel_selftest_amp_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_accel_selftest_amp_u8 <= BMI160_MAX_VALUE_SELFTEST_AMP) {
        /* write  self test amplitude*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_SELFTEST_AMP__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_SELFTEST_AMP,
                            v_accel_selftest_amp_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_SELFTEST_AMP__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

#ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
#endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160GYR
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_selftest_start(
    u8 *v_gyro_selftest_start_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	/* check the p_bmi160 structure for NULL pointer assignment*/
	
    if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read gyro self test start */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_GYRO_SELFTEST_START__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_selftest_start_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_GYRO_SELFTEST_START);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_selftest_start(
    u8 v_gyro_selftest_start_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    if (v_gyro_selftest_start_u8 <= BMI160_MAX_VALUE_SELFTEST_START) {
        /* write gyro self test start */
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_GYRO_SELFTEST_START__REG,
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_GYRO_SELFTEST_START,
                            v_gyro_selftest_start_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_GYRO_SELFTEST_START__REG,
                            &v_data_u8,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160GYR

#if MOTION_IF == SPI
BMI160_RETURN_FUNCTION_TYPE bmi160_get_spi_enable(
    u8 *v_spi_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read interface section*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_NV_CONFIG_SPI_ENABLE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_spi_enable_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_NV_CONFIG_SPI_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_spi_enable(
    u8 v_spi_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
        /* write interface section*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_NV_CONFIG_SPI_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_NV_CONFIG_SPI_ENABLE,
                            v_spi_enable_u8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_NV_CONFIG_SPI_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }

        return com_rslt;
}
#endif //MOTION_IF == SPI

#ifdef INCLUDE_BMI160ACC
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_compensation_xaxis(
    s8 *v_accel_off_x_s8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Accel manual offset compensation of x axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_off_x_s8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_OFFSET_0_ACCEL_OFF_X);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_xaxis(
    s8 v_accel_off_x_s8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    u8 v_data_u8 = BMI160_INIT_VALUE;
    u8 v_status_s8 = SUCCESS;
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }

    /* enable Accel offset */
    v_status_s8 = bmi160_set_accel_offset_enable(ACCEL_OFFSET_ENABLE);
    if (v_status_s8 == SUCCESS) {
        /* write Accel manual offset compensation of x axis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_OFFSET_0_ACCEL_OFF_X,
                            v_accel_off_x_s8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_0_ACCEL_OFF_X__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt =  ERROR;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_compensation_yaxis(
    s8 *v_accel_off_y_s8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;

	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read Accel manual offset compensation of y axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_off_y_s8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_OFFSET_1_ACCEL_OFF_Y);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_yaxis(
    s8 v_accel_off_y_s8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data_u8 = BMI160_INIT_VALUE;
    u8 v_status_s8 = SUCCESS;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    /* enable Accel offset */
    v_status_s8 = bmi160_set_accel_offset_enable(ACCEL_OFFSET_ENABLE);
    if (v_status_s8 == SUCCESS) {
        /* write Accel manual offset compensation of y axis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_OFFSET_1_ACCEL_OFF_Y,
                            v_accel_off_y_s8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_1_ACCEL_OFF_Y__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = ERROR;
    }

	return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_compensation_zaxis(
    s8 *v_accel_off_z_s8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;

	u8 v_data_u8 = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read Accel manual offset compensation of z axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_off_z_s8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_OFFSET_2_ACCEL_OFF_Z);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_compensation_zaxis(
    s8 v_accel_off_z_s8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_status_s8 = SUCCESS;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* enable Accel offset */
    v_status_s8 = bmi160_set_accel_offset_enable(ACCEL_OFFSET_ENABLE);
    if (v_status_s8 == SUCCESS) {
        /* write Accel manual offset
        compensation of z axis*/
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG,
                    &v_data_u8,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_OFFSET_2_ACCEL_OFF_Z,
                            v_accel_off_z_s8);
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_2_ACCEL_OFF_Z__REG,
                            &v_data_u8,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Check for the power mode of Accel and gyro not in normal mode */
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        com_rslt = ERROR;
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160GYR
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_compensation_xaxis(
    s16 *v_gyro_off_x_s16)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data1_u8r = BMI160_INIT_VALUE;
	u8 v_data2_u8r = BMI160_INIT_VALUE;
	s16 v_data3_u8r, v_data4_u8r = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read gyro offset x*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_3_GYRO_OFF_X__REG,
                &v_data1_u8r, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    v_data1_u8r = 
        BMI160_GET_BITSLICE(v_data1_u8r, BMI160_USER_OFFSET_3_GYRO_OFF_X);
    com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_6_GYRO_OFF_X__REG,
                    &v_data2_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    v_data2_u8r = 
        BMI160_GET_BITSLICE(v_data2_u8r, BMI160_USER_OFFSET_6_GYRO_OFF_X);
    v_data3_u8r = v_data2_u8r << BMI160_SHIFT_BIT_POSITION_BY_14_BITS;
    v_data4_u8r =  v_data1_u8r << BMI160_SHIFT_BIT_POSITION_BY_06_BITS;
    v_data3_u8r = v_data3_u8r | v_data4_u8r;
    *v_gyro_off_x_s16 = v_data3_u8r >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS;

return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_xaxis(
    s16 v_gyro_off_x_s16)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data1_u8r, v_data2_u8r = BMI160_INIT_VALUE;
    u16 v_data3_u8r = BMI160_INIT_VALUE;
    u8 v_status_s8 = SUCCESS;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    /* write gyro offset x*/
    v_status_s8 = bmi160_set_gyro_offset_enable(GYRO_OFFSET_ENABLE);
    if (v_status_s8 == SUCCESS) {
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_3_GYRO_OFF_X__REG,
                    &v_data2_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data1_u8r = (s8) (v_gyro_off_x_s16 &
                                BMI160_GYRO_MANUAL_OFFSET_0_7);
            v_data2_u8r = BMI160_SET_BITSLICE(
                            v_data2_u8r,
                            BMI160_USER_OFFSET_3_GYRO_OFF_X,
                            v_data1_u8r);
            /* write 0x74 bit 0 to 7*/
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_3_GYRO_OFF_X__REG,
                            &v_data2_u8r,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }

        com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_OFFSET_6_GYRO_OFF_X__REG,
                        &v_data2_u8r, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data3_u8r = (u16) (v_gyro_off_x_s16 &
                                 BMI160_GYRO_MANUAL_OFFSET_8_9);
            v_data1_u8r = (u8) (v_data3_u8r >>
                                BMI160_SHIFT_BIT_POSITION_BY_08_BITS);
            v_data2_u8r = BMI160_SET_BITSLICE(
                            v_data2_u8r,
                            BMI160_USER_OFFSET_6_GYRO_OFF_X,
                            v_data1_u8r);
            
            /* write 0x77 bit 0 and 1*/
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_6_GYRO_OFF_X__REG,
                            &v_data2_u8r,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        return ERROR;
    }

    return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_compensation_yaxis(
    s16 *v_gyro_off_y_s16)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data1_u8r = BMI160_INIT_VALUE;
	u8 v_data2_u8r = BMI160_INIT_VALUE;
	s16 v_data3_u8r, v_data4_u8r = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read gyro offset y*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG,
                &v_data1_u8r, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    v_data1_u8r = 
        BMI160_GET_BITSLICE(v_data1_u8r, BMI160_USER_OFFSET_4_GYRO_OFF_Y);
    com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG,
                    &v_data2_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    v_data2_u8r = 
        BMI160_GET_BITSLICE(v_data2_u8r, BMI160_USER_OFFSET_6_GYRO_OFF_Y);
    v_data3_u8r = v_data2_u8r << BMI160_SHIFT_BIT_POSITION_BY_14_BITS;
    v_data4_u8r =  v_data1_u8r << BMI160_SHIFT_BIT_POSITION_BY_06_BITS;
    v_data3_u8r = v_data3_u8r | v_data4_u8r;
    *v_gyro_off_y_s16 = v_data3_u8r >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS;

    return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_yaxis(
    s16 v_gyro_off_y_s16)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data1_u8r, v_data2_u8r = BMI160_INIT_VALUE;
    u16 v_data3_u8r = BMI160_INIT_VALUE;
    u8 v_status_s8 = SUCCESS;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }
    
    /* enable gyro offset bit */
    v_status_s8 = bmi160_set_gyro_offset_enable(GYRO_OFFSET_ENABLE);
    /* write gyro offset y*/
    if (v_status_s8 == SUCCESS) {
        com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG,
                    &v_data2_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data1_u8r = (s8) (v_gyro_off_y_s16 &
                                BMI160_GYRO_MANUAL_OFFSET_0_7);
            v_data2_u8r = BMI160_SET_BITSLICE(
                            v_data2_u8r,
                            BMI160_USER_OFFSET_4_GYRO_OFF_Y,
                            v_data1_u8r);
            
            /* write 0x75 bit 0 to 7*/
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_4_GYRO_OFF_Y__REG,
                            &v_data2_u8r,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }

        com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG,
                        &v_data2_u8r, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
        if (com_rslt == SUCCESS) {
            v_data3_u8r = (u16) (v_gyro_off_y_s16 &
                                 BMI160_GYRO_MANUAL_OFFSET_8_9);
            v_data1_u8r = (u8) (v_data3_u8r >>
                                BMI160_SHIFT_BIT_POSITION_BY_08_BITS);
            v_data2_u8r = BMI160_SET_BITSLICE(
                            v_data2_u8r,
                            BMI160_USER_OFFSET_6_GYRO_OFF_Y,
                            v_data1_u8r);
            
            /* write 0x77 bit 2 and 3*/
            com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_6_GYRO_OFF_Y__REG,
                            &v_data2_u8r,
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
            /*Accel and Gyro power mode check*/
            if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
                /*interface idle time delay */
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
    #endif //INCLUDE_BMI160PMU
        }
    } else {
        return ERROR;
    }

    return com_rslt;
}

    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_compensation_zaxis(
    s16 *v_gyro_off_z_s16)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;

	u8 v_data1_u8r = BMI160_INIT_VALUE;
	u8 v_data2_u8r = BMI160_INIT_VALUE;
	s16 v_data3_u8r, v_data4_u8r = BMI160_INIT_VALUE;

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read gyro manual offset z axis*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG,
                &v_data1_u8r,
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    v_data1_u8r = BMI160_GET_BITSLICE(
                    v_data1_u8r,
                    BMI160_USER_OFFSET_5_GYRO_OFF_Z);
    com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG,
                    &v_data2_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    v_data2_u8r = 
        BMI160_GET_BITSLICE(v_data2_u8r,
                            BMI160_USER_OFFSET_6_GYRO_OFF_Z);
    v_data3_u8r = v_data2_u8r << BMI160_SHIFT_BIT_POSITION_BY_14_BITS;
    v_data4_u8r =  v_data1_u8r << BMI160_SHIFT_BIT_POSITION_BY_06_BITS;
    v_data3_u8r = v_data3_u8r | v_data4_u8r;
    *v_gyro_off_z_s16 = v_data3_u8r >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS;

    return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_compensation_zaxis(
    s16 v_gyro_off_z_s16)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

    u8 v_data1_u8r, v_data2_u8r = BMI160_INIT_VALUE;
    u16 v_data3_u8r = BMI160_INIT_VALUE;
    u8 v_status_s8 = SUCCESS;

    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
	} else {
		/* enable gyro offset*/
		v_status_s8 = bmi160_set_gyro_offset_enable(GYRO_OFFSET_ENABLE);
		/* write gyro manual offset z axis*/
		if (v_status_s8 == SUCCESS) {
			com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG,
                        &v_data2_u8r, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data1_u8r = (u8) (v_gyro_off_z_s16 &
                                    BMI160_GYRO_MANUAL_OFFSET_0_7);
				v_data2_u8r = BMI160_SET_BITSLICE(
                                v_data2_u8r,
                                BMI160_USER_OFFSET_5_GYRO_OFF_Z,
                                v_data1_u8r);
				
                /* write 0x76 bit 0 to 7*/
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                                p_bmi160->dev_addr,
                                BMI160_USER_OFFSET_5_GYRO_OFF_Z__REG,
                                &v_data2_u8r,
                                BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
				/*Accel and Gyro power mode check*/
				if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
					/*interface idle time delay */
					p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
                }
    #endif //INCLUDE_BMI160PMU
			}

			com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG,
                            &v_data2_u8r, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);
			if (com_rslt == SUCCESS) {
				v_data3_u8r = (u16) (v_gyro_off_z_s16 &
                                     BMI160_GYRO_MANUAL_OFFSET_8_9);
				v_data1_u8r = (u8) (v_data3_u8r >>
                                    BMI160_SHIFT_BIT_POSITION_BY_08_BITS);
				v_data2_u8r = BMI160_SET_BITSLICE(
                                v_data2_u8r,
                                BMI160_USER_OFFSET_6_GYRO_OFF_Z,
                                v_data1_u8r);
				
                /* write 0x77 bit 4 and 5*/
				com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                                p_bmi160->dev_addr,
                                BMI160_USER_OFFSET_6_GYRO_OFF_Z__REG,
                                &v_data2_u8r,
                                BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
				/*Accel and Gyro power mode check*/
				if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
					/*interface idle time delay */
					p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
                }
    #endif //INCLUDE_BMI160PMU
			}
		} else {
            return ERROR;
		}
	}
    return com_rslt;
}
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160ACC
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_accel_offset_enable(
    u8 *v_accel_off_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read Accel offset enable */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_accel_off_enable_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_accel_offset_enable(
    u8 v_accel_off_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write Accel offset enable */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE,
                        v_accel_off_enable_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_OFFSET_6_ACCEL_OFF_ENABLE__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160ACC

#ifdef INCLUDE_BMI160GYR
    #ifdef INCLUDE_BMI160GET
BMI160_RETURN_FUNCTION_TYPE bmi160_get_gyro_offset_enable(
    u8 *v_gyro_off_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read gyro offset*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_gyro_off_enable_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, BMI160_USER_OFFSET_6_GYRO_OFF_EN);

	return com_rslt;
}
    #endif //INCLUDE_BMI160GET

BMI160_RETURN_FUNCTION_TYPE bmi160_set_gyro_offset_enable(
    u8 v_gyro_off_enable_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write gyro offset*/
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    if (com_rslt == SUCCESS) {
        v_data_u8 = BMI160_SET_BITSLICE(
                        v_data_u8,
                        BMI160_USER_OFFSET_6_GYRO_OFF_EN,
                        v_gyro_off_enable_u8);
        com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                        p_bmi160->dev_addr,
                        BMI160_USER_OFFSET_6_GYRO_OFF_EN__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);

    #ifdef INCLUDE_BMI160PMU
        /*Accel and Gyro power mode check*/
        if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
            /*interface idle time delay */
            p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
    #endif //INCLUDE_BMI160PMU
    }

	return com_rslt;
}
#endif //INCLUDE_BMI160GYR

#ifdef INCLUDE_BMI160PED
BMI160_RETURN_FUNCTION_TYPE bmi160_read_step_count(
    s16 *v_step_cnt_s16)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* array having the step counter LSB and MSB data
        v_data_u8[0] - LSB
        v_data_u8[1] - MSB*/
	u8 a_data_u8r[BMI160_STEP_COUNT_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }

    /* read step counter */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_STEP_COUNT_LSB__REG,
                a_data_u8r, 
                BMI160_STEP_COUNTER_LENGTH);

    *v_step_cnt_s16 =
        (s16) ((((s32) ((s8) a_data_u8r[BMI160_STEP_COUNT_MSB_BYTE])) <<
                BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               a_data_u8r[BMI160_STEP_COUNT_LSB_BYTE]);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_config(
    u16 *v_step_config_u16)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data1_u8r = BMI160_INIT_VALUE;
	u8 v_data2_u8r = BMI160_INIT_VALUE;
	u16 v_data3_u8r = BMI160_INIT_VALUE;
	
    /* Read the 0 to 7 bit*/
	com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_STEP_CONFIG_ZERO__REG,
                &v_data1_u8r, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* Read the 8 to 10 bit*/
	com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_STEP_CONFIG_ONE_CNF1__REG,
                    &v_data2_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    v_data2_u8r = 
        BMI160_GET_BITSLICE(v_data2_u8r, BMI160_USER_STEP_CONFIG_ONE_CNF1);
	
    v_data3_u8r = (u16) ((((u32) ((u8) v_data2_u8r)) <<
                          BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                         v_data1_u8r);
	
    /* Read the 11 to 14 bit*/
	com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_STEP_CONFIG_ONE_CNF2__REG,
                    &v_data1_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	v_data1_u8r = 
        BMI160_GET_BITSLICE(v_data1_u8r, BMI160_USER_STEP_CONFIG_ONE_CNF2);
	*v_step_config_u16 = (u16) ((((u32) ((u8)v_data1_u8r)) <<
                                 BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                v_data3_u8r);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_config(
    u16 v_step_config_u16)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_data1_u8r = BMI160_INIT_VALUE;
	u8 v_data2_u8r = BMI160_INIT_VALUE;
	u16 v_data3_u16 = BMI160_INIT_VALUE;

	/* write the 0 to 7 bit*/
	v_data1_u8r = (u8) (v_step_config_u16 & BMI160_STEP_CONFIG_0_7);
	p_bmi160->BMI160_BUS_WRITE_FUNC(
        p_bmi160->dev_addr,
        BMI160_USER_STEP_CONFIG_ZERO__REG,
        &v_data1_u8r, 
        BMI160_GEN_READ_WRITE_DATA_LENGTH);

	/*Accel and Gyro power mode check*/
	if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
		/*interface idle time delay */
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }

	/* write the 8 to 10 bit*/
	com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_STEP_CONFIG_ONE_CNF1__REG,
                &v_data2_u8r, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
	if (com_rslt == SUCCESS) {
		v_data3_u16 = (u16) (v_step_config_u16 & BMI160_STEP_CONFIG_8_10);
		v_data1_u8r = (u8)(v_data3_u16 >> BMI160_SHIFT_BIT_POSITION_BY_08_BITS);
		v_data2_u8r = BMI160_SET_BITSLICE(
                        v_data2_u8r,
                        BMI160_USER_STEP_CONFIG_ONE_CNF1, 
                        v_data1_u8r);
		p_bmi160->BMI160_BUS_WRITE_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STEP_CONFIG_ONE_CNF1__REG,
            &v_data2_u8r, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);

		/*Accel and Gyro power mode check*/
		if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
			/*interface idle time delay */
			p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
	}
    
	/* write the 11 to 14 bit*/
	com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_STEP_CONFIG_ONE_CNF2__REG,
                    &v_data2_u8r, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	if (com_rslt == SUCCESS) {
		v_data3_u16 = (u16) (v_step_config_u16 & BMI160_STEP_CONFIG_11_14);
		v_data1_u8r = (u8) (v_data3_u16 >> 
                            BMI160_SHIFT_BIT_POSITION_BY_12_BITS);
		v_data2_u8r = BMI160_SET_BITSLICE(
                        v_data2_u8r,
                        BMI160_USER_STEP_CONFIG_ONE_CNF2, 
                        v_data1_u8r);
		p_bmi160->BMI160_BUS_WRITE_FUNC(
            p_bmi160->dev_addr,
            BMI160_USER_STEP_CONFIG_ONE_CNF2__REG,
            &v_data2_u8r, 
            BMI160_GEN_READ_WRITE_DATA_LENGTH);

		/*Accel and Gyro power mode check*/
		if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
			/*interface idle time delay */
			p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        }
	}

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_get_step_counter_enable(
    u8 *v_step_counter_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* read the step counter */
    com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                p_bmi160->dev_addr,
                BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
                &v_data_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    *v_step_counter_u8 = 
        BMI160_GET_BITSLICE(v_data_u8, 
                            BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE);

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_counter_enable(
    u8 v_step_counter_u8)
{
    /* variable used to return the status of communication result*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    u8 v_data_u8 = BMI160_INIT_VALUE;
    
    /* check the p_bmi160 structure for NULL pointer assignment*/
    if (p_bmi160 == BMI160_NULL) {
        return E_BMI160_NULL_PTR;
    }

	if (v_step_counter_u8 <= BMI160_MAX_GYRO_STEP_COUNTER) {
		/* write the step counter */
		com_rslt = p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		if (com_rslt == SUCCESS) {
			v_data_u8 = BMI160_SET_BITSLICE(
                            v_data_u8,
                            BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE,
                            v_step_counter_u8);
			com_rslt += p_bmi160->BMI160_BUS_WRITE_FUNC(
                            p_bmi160->dev_addr,
                            BMI160_USER_STEP_CONFIG_1_STEP_COUNT_ENABLE__REG,
                            &v_data_u8, 
                            BMI160_GEN_READ_WRITE_DATA_LENGTH);

			/*Accel and Gyro power mode check*/
			if (bmi160_power_mode_status_u8_g != BMI160_NORMAL_MODE) {
				/*interface idle time delay */
				p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            }
		}
	} else {
        com_rslt = E_BMI160_OUT_OF_RANGE;
	}

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_step_mode(
    u8 v_step_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

	switch (v_step_mode_u8) {
	case BMI160_STEP_NORMAL_MODE:
		com_rslt = bmi160_set_step_config(STEP_CONFIG_NORMAL);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_STEP_SENSITIVE_MODE:
		com_rslt = bmi160_set_step_config(STEP_CONFIG_SENSITIVE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_STEP_ROBUST_MODE:
		com_rslt = bmi160_set_step_config(STEP_CONFIG_ROBUST);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}

	return com_rslt;
}
#endif //INCLUDE_BMI160PED

#ifdef INCLUDE_BMI160INT
BMI160_RETURN_FUNCTION_TYPE bmi160_map_significant_motion_intr(
    u8 v_significant_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_sig_motion_u8 = BMI160_INIT_VALUE;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_any_motion_intr1_stat_u8 = BMI160_ENABLE_ANY_MOTION_INTR1;
	u8 v_any_motion_intr2_stat_u8 = BMI160_ENABLE_ANY_MOTION_INTR2;
	u8 v_any_motion_axis_stat_u8 = BMI160_ENABLE_ANY_MOTION_AXIS;
	
    /* enable the significant motion interrupt */
	com_rslt = bmi160_get_intr_significant_motion_select(&v_sig_motion_u8);
	if (v_sig_motion_u8 != BMI160_SIG_MOTION_STAT_HIGH)
		com_rslt += bmi160_set_intr_significant_motion_select(
                        BMI160_SIG_MOTION_INTR_ENABLE);
	
    switch (v_significant_u8) {
	case BMI160_MAP_INTR1:
		/* interrupt */
		com_rslt += bmi160_read_reg(
                        BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_intr1_stat_u8;
		
        /* map the signification interrupt to any-motion interrupt1*/
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_MAP_0_INTR1_ANY_MOTION__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* axis*/
		com_rslt = bmi160_read_reg(
                    BMI160_USER_INTR_ENABLE_0_ADDR,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_axis_stat_u8;
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_ENABLE_0_ADDR,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

	case BMI160_MAP_INTR2:
		/* map the signification interrupt to any-motion interrupt2*/
		com_rslt += bmi160_read_reg(
                        BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_intr2_stat_u8;
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_MAP_2_INTR2_ANY_MOTION__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* axis*/
		com_rslt = bmi160_read_reg(
                    BMI160_USER_INTR_ENABLE_0_ADDR,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_any_motion_axis_stat_u8;
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_ENABLE_0_ADDR,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;

	}
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_map_step_detector_intr(
    u8 v_step_detector_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_step_det_u8 = BMI160_INIT_VALUE;
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_low_g_intr_u81_stat_u8 = BMI160_LOW_G_INTR_STAT;
	u8 v_low_g_intr_u82_stat_u8 = BMI160_LOW_G_INTR_STAT;
	
    /* read the v_status_s8 of step detector interrupt*/
	com_rslt = bmi160_get_step_detector_enable(&v_step_det_u8);	
    if (v_step_det_u8 != BMI160_STEP_DET_STAT_HIGH)
		com_rslt += bmi160_set_step_detector_enable(
                        BMI160_STEP_DETECT_INTR_ENABLE);
	switch (v_step_detector_u8) {
	case BMI160_MAP_INTR1:
		com_rslt += bmi160_read_reg(
                        BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_low_g_intr_u81_stat_u8;
		
        /* map the step detector interrupt to Low-g interrupt 1*/
		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_MAP_0_INTR1_LOW_G__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_MAP_INTR2:
		/* map the step detector interrupt to Low-g interrupt 2*/
		com_rslt += bmi160_read_reg(
                        BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_data_u8 |= v_low_g_intr_u82_stat_u8;

		com_rslt += bmi160_write_reg(
                        BMI160_USER_INTR_MAP_2_INTR2_LOW_G__REG,
                        &v_data_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
	return com_rslt;
}
#endif //INCLUDE_BMI160INT

#ifdef INCLUDE_BMI160PED
BMI160_RETURN_FUNCTION_TYPE bmi160_clear_step_counter(void)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* clear the step counter*/
	com_rslt = bmi160_set_command_register(RESET_STEP_COUNTER);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

	return com_rslt;
}
#endif //INCLUDE_BMI160PED

BMI160_RETURN_FUNCTION_TYPE bmi160_set_command_register(
    u8 v_command_reg_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt  = E_BMI160_COMM_RES;
	
    /* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
	}
    
    /* write command register */
    com_rslt = p_bmi160->BMI160_BUS_WRITE_FUNC(
                p_bmi160->dev_addr,
                BMI160_CMD_COMMANDS__REG,
                &v_command_reg_u8, 
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /*interface idle time delay */
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
#ifdef INCLUDE_BMI160PMU
    /*power mode status of Accel and gyro is stored in the
      global variable bmi160_power_mode_status_u8_g */
    com_rslt += bmi160_read_reg(
                    BMI160_USER_PMU_STAT_ADDR,
                    &bmi160_power_mode_status_u8_g,
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    bmi160_power_mode_status_u8_g &= BMI160_ACCEL_GYRO_PMU_MASK;
#endif //INCLUDE_BMI160PMU

	return com_rslt;
}

#if defined(FIFO_ENABLE) && defined(INCLUDE_BMI160MAG)
    /* FIFO data read for 1024 bytes of data */
    static u8 v_fifo_data_u8[FIFO_FRAME];

    struct bmi160_mag_fifo_data_t mag_data;

BMI160_RETURN_FUNCTION_TYPE bmi160_second_if_mag_compensate_xyz(
    struct bmi160_mag_fifo_data_t mag_fifo_data, u8 v_mag_second_if_u8)
{
    s8 com_rslt = BMI160_INIT_VALUE;
    s16 v_mag_x_s16 = BMI160_INIT_VALUE;
    s16 v_mag_y_s16 = BMI160_INIT_VALUE;
    s16 v_mag_z_s16 = BMI160_INIT_VALUE;
    u16 v_mag_r_u16 = BMI160_INIT_VALUE;
    
    #ifdef YAS537
	u8 v_outflow_u8 = BMI160_INIT_VALUE;
	u8 v_busy_u8 = BMI160_INIT_VALUE;
	u8 v_coil_stat_u8 = BMI160_INIT_VALUE;
	u16 v_temperature_u16 = BMI160_INIT_VALUE;
	s32 a_h_s32[BMI160_YAS_H_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	s32 a_s_s32[BMI160_YAS_S_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	u16 xy1y2[3] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
    #endif //YAS537
    
    #ifdef YAS532
	u16 v_xy1y2_u16[3] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	u8 v_busy_yas532_u8 = BMI160_INIT_VALUE;
	u16 v_temp_yas532_u16 = BMI160_INIT_VALUE;
	u8 v_overflow_yas532_u8 = BMI160_INIT_VALUE;
    #endif //YAS532
    
    switch (v_mag_second_if_u8) {
        case BMI160_SEC_IF_BMM150: {
            /* x data*/
            v_mag_x_s16 = (s16) ((mag_fifo_data.mag_x_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_x_lsb);
            v_mag_x_s16 = (s16) (v_mag_x_s16 >> 
                                 BMI160_SHIFT_BIT_POSITION_BY_03_BITS);
            
            /* y data*/
            v_mag_y_s16 = (s16) ((mag_fifo_data.mag_y_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_y_lsb);
            v_mag_y_s16 = (s16) (v_mag_y_s16 >> 
                                 BMI160_SHIFT_BIT_POSITION_BY_03_BITS);
            
            /* z data*/
            v_mag_z_s16 = (s16) ((mag_fifo_data.mag_z_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_z_lsb);
            v_mag_z_s16 = (s16) (v_mag_z_s16 >> 
                                 BMI160_SHIFT_BIT_POSITION_BY_01_BIT);
            
            /* r data*/
            v_mag_r_u16 = (u16) ((mag_fifo_data.mag_r_y2_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_r_y2_lsb);
            v_mag_r_u16 = (u16) (v_mag_r_u16 >> 
                                 BMI160_SHIFT_BIT_POSITION_BY_02_BITS);
            
            /* Compensated Mag x data */
            processed_data.x = bmi160_bmm150_mag_compensate_X(
                                v_mag_x_s16, v_mag_r_u16);
            
            /* Compensated Mag y data */
            processed_data.y = bmi160_bmm150_mag_compensate_Y(
                                v_mag_y_s16, v_mag_r_u16);
            
            /* Compensated Mag z data */
            processed_data.z = bmi160_bmm150_mag_compensate_Z(
                                v_mag_z_s16, v_mag_r_u16);
        } break;
    
    #ifdef AKM09911
        case BMI160_SEC_IF_AKM09911: {
            /* x data*/
            v_mag_x_s16 = (s16) ((mag_fifo_data.mag_x_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_x_lsb);
            
            /* y data*/
            v_mag_y_s16 = (s16) ((mag_fifo_data.mag_y_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_y_lsb);
            
            /* z data*/
            v_mag_z_s16 = (s16) ((mag_fifo_data.mag_z_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_z_lsb);
            
            /* Compensated for X data */
            processed_data.x = bmi160_bst_akm09911_compensate_X(v_mag_x_s16);
            
            /* Compensated for Y data */
            processed_data.y = bmi160_bst_akm09911_compensate_Y(v_mag_y_s16);
            
            /* Compensated for Z data */
            processed_data.z = bmi160_bst_akm09911_compensate_Z(v_mag_z_s16);
        } break;
    #endif //AKM09911
        
    #ifdef AKM09912
        case BMI160_SEC_IF_AKM09912: {
            /* x data*/
            v_mag_x_s16 = (s16) ((mag_fifo_data.mag_x_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_x_lsb);
            
            /* y data*/
            v_mag_y_s16 = (s16) ((mag_fifo_data.mag_y_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_y_lsb);
            
            /* z data*/
            v_mag_z_s16 = (s16) ((mag_fifo_data.mag_z_msb <<
                                  BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                 mag_fifo_data.mag_z_lsb);
            
            /* Compensated for X data */
            processed_data.x = bmi160_bst_akm09912_compensate_X(v_mag_x_s16);
        
            /* Compensated for Y data */
            processed_data.y = bmi160_bst_akm09912_compensate_Y(v_mag_y_s16);
        
            /* Compensated for Z data */
            processed_data.z = bmi160_bst_akm09912_compensate_Z(v_mag_z_s16);
        } break;
    #endif //AKM09912
        
    #ifdef YAS532
        case BMI160_SEC_IF_YAS532: {
            u8 i = BMI160_INIT_VALUE;
            
            /* read the xyy1 data*/
            v_busy_yas532_u8 = (mag_fifo_data.mag_x_lsb >> 
                                BMI160_SHIFT_BIT_POSITION_BY_07_BITS) & 0x01;
            v_temp_yas532_u16 = (u16) ((((s32) mag_fifo_data.mag_x_lsb <<
                                         BMI160_SHIFT_BIT_POSITION_BY_03_BITS) &
                                        0x3F8) |
                                       ((mag_fifo_data.mag_x_msb >>
                                         BMI160_SHIFT_BIT_POSITION_BY_05_BITS) & 
                                        0x07));

            v_xy1y2_u16[0] = (u16) ((((s32) mag_fifo_data.mag_y_lsb <<
                                      BMI160_SHIFT_BIT_POSITION_BY_06_BITS) & 
                                     0x1FC0) |
                                    ((mag_fifo_data.mag_y_msb >>
                                      BMI160_SHIFT_BIT_POSITION_BY_02_BITS) &
                                     0x3F));
            v_xy1y2_u16[1] = (u16) ((((s32) mag_fifo_data.mag_z_lsb <<
                                      BMI160_SHIFT_BIT_POSITION_BY_06_BITS) &
                                     0x1FC0) |
                                    ((mag_fifo_data.mag_z_msb >>
                                      BMI160_SHIFT_BIT_POSITION_BY_02_BITS) &
                                     0x3F));
            v_xy1y2_u16[2] = (u16) ((((s32) mag_fifo_data.mag_r_y2_lsb <<
                                      BMI160_SHIFT_BIT_POSITION_BY_06_BITS) &
                                     0x1FC0) |
                                    ((mag_fifo_data.mag_r_y2_msb >>
                                      BMI160_SHIFT_BIT_POSITION_BY_02_BITS) &
                                     0x3F));
            v_overflow_yas532_u8 = 0;
            for (i = 0; i < 3; i++) {
                if (v_xy1y2_u16[i] == YAS532_DATA_OVERFLOW) {
                    v_overflow_yas532_u8 |= 1 << (i * 2);
                }
                if (v_xy1y2_u16[i] == YAS532_DATA_UNDERFLOW) {
                    v_overflow_yas532_u8 |= 1 << (i * 2 + 1);
                }
            }
            /* assign the data*/
            com_rslt = bmi160_bst_yas532_fifo_xyz_data(
                        v_xy1y2_u16, 
                        1, 
                        v_overflow_yas532_u8,
                        v_temp_yas532_u16, 
                        v_busy_yas532_u8);
            processed_data.x = fifo_xyz_data.yas532_vector_xyz[0];
            processed_data.y = fifo_xyz_data.yas532_vector_xyz[1];
            processed_data.z = fifo_xyz_data.yas532_vector_xyz[2];
        } break;
    #endif //YAS532
        
    #ifdef YAS537
        case BMI160_SEC_IF_YAS537: {
            u8 i = BMI160_INIT_VALUE;
            
            /* read the busy flag*/
            v_busy_u8 = mag_fifo_data.mag_y_lsb >>
                        BMI160_SHIFT_BIT_POSITION_BY_07_BITS;
            
            /* read the coil status*/
            v_coil_stat_u8 = (mag_fifo_data.mag_y_lsb >>
                              BMI160_SHIFT_BIT_POSITION_BY_06_BITS) & 0X01;
            
            /* read temperature data*/
            v_temperature_u16 = (u16) ((mag_fifo_data.mag_x_lsb <<
                                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                                       mag_fifo_data.mag_x_msb);
            
            /* read x data*/
            xy1y2[0] = (u16) (((mag_fifo_data.mag_y_lsb & 0x3F) <<
                               BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                              mag_fifo_data.mag_y_msb);
            
            /* read y1 data*/
            xy1y2[1] = (u16) ((mag_fifo_data.mag_z_lsb <<
                               BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                              mag_fifo_data.mag_z_msb);
            
            /* read y2 data*/
            xy1y2[2] = (u16) ((mag_fifo_data.mag_r_y2_lsb <<
                               BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                              mag_fifo_data.mag_r_y2_msb);
            
            for (i = 0; i < 3; i++) {
                yas537_data.last_raw[i] = xy1y2[i];
            }
            yas537_data.last_raw[i] = v_temperature_u16;
            
            if (yas537_data.calib_yas537.ver == 1) {
                for (i = 0; i < 3; i++) {
                    a_s_s32[i] = xy1y2[i] - 8192;
                }
                
                /* read hx*/
                a_h_s32[0] = yas537_data.calib_yas537.k * (
                                128 * a_s_s32[0] +
                                yas537_data.calib_yas537.a2 * a_s_s32[1] +
                                yas537_data.calib_yas537.a3 * a_s_s32[2]) / 8192;
                
                /* read hy1*/
                a_h_s32[1] = yas537_data.calib_yas537.k * (
                                yas537_data.calib_yas537.a4 * a_s_s32[0] +
                                yas537_data.calib_yas537.a5 * a_s_s32[1] +
                                yas537_data.calib_yas537.a6 * a_s_s32[2]) / 8192;
                
                /* read hy2*/
                a_h_s32[2] = yas537_data.calib_yas537.k * (
                                yas537_data.calib_yas537.a7 * a_s_s32[0] +
                                yas537_data.calib_yas537.a8 * a_s_s32[1] +
                                yas537_data.calib_yas537.a9 * a_s_s32[2]) / 8192;

                for (i = 0; i < 3; i++) {
                    if (a_h_s32[i] < -8192) {
                        a_h_s32[i] = -8192;
                    }

                    if (8192 < a_h_s32[i]) {
                        a_h_s32[i] = 8192;
                    }

                    xy1y2[i] = a_h_s32[i] + 8192;
                }
            }
            
            v_outflow_u8 = 0;
            for (i = 0; i < 3; i++) {
                if (YAS537_DATA_OVERFLOW <= xy1y2[i]) {
                    v_outflow_u8 |= 1 << (i * 2);
                }
                if (xy1y2[i] == YAS537_DATA_UNDERFLOW) {
                    v_outflow_u8 |= 1 << (i * 2 + 1);
                    
                }
            }
            com_rslt = bmi160_bst_yamaha_yas537_fifo_xyz_data(
                        xy1y2, v_outflow_u8, v_coil_stat_u8, v_busy_u8);
            processed_data.x = fifo_vector_xyz.yas537_vector_xyz[0];
            processed_data.y = fifo_vector_xyz.yas537_vector_xyz[1];
            processed_data.z = fifo_vector_xyz.yas537_vector_xyz[2];
        } break;
    #endif //YAS537
    
        default: {
            com_rslt = E_BMI160_OUT_OF_RANGE;
        } break;
    }
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_header_data(
    u8 v_mag_if_u8, struct bmi160_fifo_data_header_t *header_data)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* read the whole FIFO data*/
	com_rslt = bmi160_read_fifo_header_data_user_defined_length(
                FIFO_FRAME, v_mag_if_u8, header_data);
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_header_data_user_defined_length(
    u16 v_fifo_user_length_u16, u8 v_mag_if_mag_u8,
    struct bmi160_fifo_data_header_t *fifo_header_data)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    u8 v_accel_index_u8 = BMI160_INIT_VALUE;
	u8 v_gyro_index_u8 = BMI160_INIT_VALUE;
	u8 v_mag_index_u8 = BMI160_INIT_VALUE;
	s8 v_last_return_stat_s8 = BMI160_INIT_VALUE;
	u16 v_fifo_index_u16 = BMI160_INIT_VALUE;
	u8 v_frame_head_u8 = BMI160_INIT_VALUE;
	u8 v_frame_index_u8 = BMI160_INIT_VALUE;
	u16 v_fifo_length_u16 = BMI160_INIT_VALUE;

	fifo_header_data->accel_frame_count = BMI160_INIT_VALUE;
	fifo_header_data->mag_frame_count = BMI160_INIT_VALUE;
	fifo_header_data->gyro_frame_count = BMI160_INIT_VALUE;
	
    /* read FIFO data*/
	com_rslt = bmi160_fifo_data(
                &v_fifo_data_u8[BMI160_INIT_VALUE], v_fifo_user_length_u16);
	v_fifo_length_u16 = v_fifo_user_length_u16;
	for (v_fifo_index_u16 = BMI160_INIT_VALUE;
         v_fifo_index_u16 < v_fifo_length_u16;) {
		fifo_header_data->fifo_header[v_frame_index_u8] =   
             v_fifo_data_u8[v_fifo_index_u16];
		v_frame_head_u8 = fifo_header_data->fifo_header[v_frame_index_u8] &
                          BMI160_FIFO_TAG_INTR_MASK;
		v_frame_index_u8++;
		switch (v_frame_head_u8) {
		/* Header frame of Accel */
            case FIFO_HEAD_A: {
            	/*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_A_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_A_OVER_LEN;
                    break;
                }
                    
                /* Accel raw x data */
                fifo_header_data->accel_fifo[v_accel_index_u8].x =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_X_MSB_DATA] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_FIFO_X_LSB_DATA]);
                
                /* Accel raw y data */
                fifo_header_data->accel_fifo[v_accel_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_Y_MSB_DATA] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_FIFO_Y_LSB_DATA]);
                
                /* Accel raw z data */
                fifo_header_data->accel_fifo[v_accel_index_u8].z =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_Z_MSB_DATA] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_FIFO_Z_LSB_DATA]);
                
                /* check for Accel frame count*/
                fifo_header_data->accel_frame_count += BMI160_FRAME_COUNT;
                v_fifo_index_u16 = v_fifo_index_u16 + BMI160_FIFO_A_LENGTH;
                v_accel_index_u8++;
            } break;

            /* Header frame of gyro */
            case FIFO_HEAD_G: {
            	/*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_G_LENGTH > 
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_G_OVER_LEN;
                    break;
                }
                
                /* Gyro raw x data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].x  =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_X_MSB_DATA] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_FIFO_X_LSB_DATA]);
                
                /* Gyro raw y data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_Y_MSB_DATA] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_FIFO_Y_LSB_DATA]);
                
                /* Gyro raw z data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].z  =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_Z_MSB_DATA] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_FIFO_Z_LSB_DATA]);
                
                /* check for gyro frame count*/
                fifo_header_data->gyro_frame_count += BMI160_FRAME_COUNT;
                
                /*fifo G data frame index + 6*/
                v_fifo_index_u16 = v_fifo_index_u16 + BMI160_FIFO_G_LENGTH;
                v_gyro_index_u8++;
            } break;

            /* Header frame of Mag */
            case FIFO_HEAD_M: {
            	/*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_M_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_M_OVER_LEN;
                    break;
                }
                
                /* Mag x data*/
                mag_data.mag_x_lsb = v_fifo_data_u8[v_fifo_index_u16 +
                                                    BMI160_FIFO_X_LSB_DATA];
                mag_data.mag_x_msb = v_fifo_data_u8[v_fifo_index_u16 +
                                                    BMI160_FIFO_X_MSB_DATA];
                
                /* Mag y data*/
                mag_data.mag_y_lsb = v_fifo_data_u8[v_fifo_index_u16 +
                                                    BMI160_FIFO_Y_LSB_DATA];
                mag_data.mag_y_msb = v_fifo_data_u8[v_fifo_index_u16 +
                                                    BMI160_FIFO_Y_MSB_DATA];
                
                mag_data.mag_z_lsb = v_fifo_data_u8[v_fifo_index_u16 +
                                                    BMI160_FIFO_Z_LSB_DATA];
                mag_data.mag_z_msb = v_fifo_data_u8[v_fifo_index_u16 +
                                                    BMI160_FIFO_Z_MSB_DATA];
                
                /* Mag r data*/
                mag_data.mag_r_y2_lsb = v_fifo_data_u8[v_fifo_index_u16 +
                                                       BMI160_FIFO_R_LSB_DATA];
                mag_data.mag_r_y2_msb = v_fifo_data_u8[v_fifo_index_u16 +
                                                       BMI160_FIFO_R_MSB_DATA];

                com_rslt = bmi160_second_if_mag_compensate_xyz(
                            mag_data, v_mag_if_mag_u8);
                 
                 /* compensated Mag x */
                fifo_header_data->mag_fifo[v_gyro_index_u8].x =
                    processed_data.x;
                
                /* compensated Mag y */
                fifo_header_data->mag_fifo[v_gyro_index_u8].y =
                    processed_data.y;
                
                /* compensated Mag z */
                fifo_header_data->mag_fifo[v_gyro_index_u8].z =
                    processed_data.z;

                /* check for Mag frame count*/
                fifo_header_data->mag_frame_count += BMI160_FRAME_COUNT;

                v_mag_index_u8++;
                
                /*fifo M data frame index + 8*/
                v_fifo_index_u16 += BMI160_FIFO_M_LENGTH;
            } break;

            /* Header frame of gyro and Accel */
            case FIFO_HEAD_G_A: {
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;
                if (v_fifo_index_u16 + BMI160_FIFO_AG_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_G_A_OVER_LEN;
                    break;
                }
                
                /* Raw gyro x */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].x =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_GA_FIFO_G_X_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_GA_FIFO_G_X_LSB]);
                
                /* Raw gyro y */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_GA_FIFO_G_Y_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_GA_FIFO_G_Y_LSB]);
                
                /* Raw gyro z */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].z =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_GA_FIFO_G_Z_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_GA_FIFO_G_Z_LSB]);
                
                /* check for gyro frame count*/
                fifo_header_data->gyro_frame_count += BMI160_FRAME_COUNT;
                
                /* Raw Accel x */
                fifo_header_data->accel_fifo[v_accel_index_u8].x =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_GA_FIFO_A_X_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_GA_FIFO_A_X_LSB]);
                
                /* Raw Accel y */
                fifo_header_data->accel_fifo[v_accel_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_GA_FIFO_A_Y_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_GA_FIFO_A_Y_LSB]);
                
                /* Raw Accel z */
                fifo_header_data->accel_fifo[v_accel_index_u8].z =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_GA_FIFO_A_Z_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 |
                                          BMI160_GA_FIFO_A_Z_LSB]);
                
                /* check for Accel frame count*/
                fifo_header_data->accel_frame_count += BMI160_FRAME_COUNT;
                
                /* Index added to 12 for gyro and Accel*/
                v_fifo_index_u16 += BMI160_FIFO_AG_LENGTH;
                
                v_gyro_index_u8++;
                v_accel_index_u8++;
            } break;
            
            /* Header frame of mag, gyro and Accel */
            case FIFO_HEAD_M_G_A: {
                /*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_AMG_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_M_G_A_OVER_LEN;
                    break;
                }
                
                /* Mag x data*/
                mag_data.mag_x_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_LSB_DATA];
                mag_data.mag_x_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_MSB_DATA];
                
                /* Mag y data*/
                mag_data.mag_y_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_LSB_DATA];
                mag_data.mag_y_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_MSB_DATA];
                
                mag_data.mag_z_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_LSB_DATA];
                mag_data.mag_z_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_MSB_DATA];
                
                /* Mag r data*/
                mag_data.mag_r_y2_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_LSB_DATA];
                mag_data.mag_r_y2_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_MSB_DATA];
                
                /* Processing the compensation data*/
                com_rslt = bmi160_second_if_mag_compensate_xyz(
                            mag_data, v_mag_if_mag_u8);
                 
                /* compensated Mag x */
                fifo_header_data->mag_fifo[v_mag_index_u8].x = processed_data.x;
                
                /* compensated Mag y */
                fifo_header_data->mag_fifo[v_mag_index_u8].y = processed_data.y;
                
                /* compensated Mag z */
                fifo_header_data->mag_fifo[v_mag_index_u8].z = processed_data.z;
                
                /* check for Mag frame count*/
                fifo_header_data->mag_frame_count += BMI160_FRAME_COUNT;
                
                /* Gyro raw x data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].x =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MGA_FIFO_G_X_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MGA_FIFO_G_X_LSB]);
                
                /* Gyro raw y data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MGA_FIFO_G_Y_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MGA_FIFO_G_Y_LSB]);
                
                /* Gyro raw z data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].z =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 + 
                                           BMI160_MGA_FIFO_G_Z_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 + 
                                          BMI160_MGA_FIFO_G_Z_LSB]);
                
                /* check for gyro frame count*/
                fifo_header_data->gyro_frame_count += BMI160_FRAME_COUNT;
                
                /* Accel raw x data */
                fifo_header_data->accel_fifo[v_accel_index_u8].x =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 + 
                                           BMI160_MGA_FIFO_A_X_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MGA_FIFO_A_X_LSB]);
                
                /* Accel raw y data */
                fifo_header_data->accel_fifo[v_accel_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 + 
                                           BMI160_MGA_FIFO_A_Y_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MGA_FIFO_A_Y_LSB]);
                
                /* Accel raw z data */
                fifo_header_data->accel_fifo[v_accel_index_u8].z =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 + 
                                           BMI160_MGA_FIFO_A_Z_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MGA_FIFO_A_Z_LSB]);
                
                /* check for Accel frame count*/
                fifo_header_data->accel_frame_count += BMI160_FRAME_COUNT;
                
                /* Index added to 20 for mag, gyro and Accel*/
                v_fifo_index_u16 += BMI160_FIFO_AMG_LENGTH;
                
                v_accel_index_u8++;
                v_mag_index_u8++;
                v_gyro_index_u8++;
            } break;

            /* Header frame of Mag and Accel */
            case FIFO_HEAD_M_A: {
                /*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_GEN_READ_WRITE_DATA_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_MA_OR_MG_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_M_A_OVER_LEN;
                    break;
                }
                    
                /* Mag x data*/
                mag_data.mag_x_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_LSB_DATA];
                mag_data.mag_x_msb = 
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_MSB_DATA];
                
                /* Mag y data*/
                mag_data.mag_y_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_LSB_DATA];
                mag_data.mag_y_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_MSB_DATA];
                
                mag_data.mag_z_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_LSB_DATA];
                mag_data.mag_z_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_MSB_DATA];
                
                /* Mag r data*/
                mag_data.mag_r_y2_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_LSB_DATA];
                mag_data.mag_r_y2_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_MSB_DATA];
                
                com_rslt = bmi160_second_if_mag_compensate_xyz(
                            mag_data, v_mag_if_mag_u8);
                 
                /* compensated Mag x */
                fifo_header_data->mag_fifo[v_mag_index_u8].x = processed_data.x;
                
                /* compensated Mag y */
                fifo_header_data->mag_fifo[v_mag_index_u8].y = processed_data.y;
                
                /* compensated Mag z */
                fifo_header_data->mag_fifo[v_mag_index_u8].z = processed_data.z;
                
                /* check for Mag frame count*/
                fifo_header_data->mag_frame_count += BMI160_FRAME_COUNT;
                
                /* Accel raw x data */
                fifo_header_data->accel_fifo[v_accel_index_u8].x =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MA_FIFO_A_X_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MA_FIFO_A_X_LSB]);
                
                /* Accel raw y data */
                fifo_header_data->accel_fifo[v_accel_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MA_FIFO_A_Y_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MA_FIFO_A_Y_LSB]);
                
                /* Accel raw z data */
                fifo_header_data->accel_fifo[v_accel_index_u8].z =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MA_FIFO_A_Z_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MA_FIFO_A_Z_LSB]);
                
                /* check for Accel frame count*/
                fifo_header_data->accel_frame_count += BMI160_FRAME_COUNT;
                
                /*fifo AM data frame index + 14(8+6)*/
                v_fifo_index_u16 += BMI160_FIFO_MA_OR_MG_LENGTH;
                
                v_accel_index_u8++;
                v_mag_index_u8++;
            } break;

            /* Header frame of Mag and gyro */
            case FIFO_HEAD_M_G: {
                /*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_GEN_READ_WRITE_DATA_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_MA_OR_MG_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_M_G_OVER_LEN;
                    break;
                }
                
                /* Mag x data*/
                mag_data.mag_x_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_LSB_DATA];
                mag_data.mag_x_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_MSB_DATA];
                
                /* Mag y data*/
                mag_data.mag_y_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_LSB_DATA];
                mag_data.mag_y_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_MSB_DATA];
                
                mag_data.mag_z_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_LSB_DATA];
                mag_data.mag_z_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_MSB_DATA];
                
                /* Mag r data*/
                mag_data.mag_r_y2_lsb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_LSB_DATA];
                mag_data.mag_r_y2_msb =
                    v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_MSB_DATA];
                
                com_rslt = bmi160_second_if_mag_compensate_xyz(
                            mag_data, v_mag_if_mag_u8);
                 
                 /* compensated Mag x */
                fifo_header_data->mag_fifo[v_mag_index_u8].x = processed_data.x;
                
                /* compensated Mag y */
                fifo_header_data->mag_fifo[v_mag_index_u8].y = processed_data.y;
                
                /* compensated Mag z */
                fifo_header_data->mag_fifo[v_mag_index_u8].z = processed_data.z;
                
                /* check for Mag frame count*/
                fifo_header_data->mag_frame_count += BMI160_FRAME_COUNT;
                
                /* Gyro raw x data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].x =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MG_FIFO_G_X_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MG_FIFO_G_X_LSB]);
                
                /* Gyro raw y data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].y =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MG_FIFO_G_Y_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MG_FIFO_G_Y_LSB]);
                
                /* Gyro raw z data */
                fifo_header_data->gyro_fifo[v_gyro_index_u8].z =
                    (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_MG_FIFO_G_Z_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_MG_FIFO_G_Z_LSB]);
                
                /* check for gyro frame count*/
                fifo_header_data->gyro_frame_count += BMI160_FRAME_COUNT;
                
                /*fifo GM data frame index + 14(8+6)*/
                v_fifo_index_u16 += BMI160_FIFO_MA_OR_MG_LENGTH;
                
                v_mag_index_u8++;
                v_gyro_index_u8++;
            } break;

            /* Header frame of sensor time */
            case FIFO_HEAD_SENSOR_TIME: {
                v_fifo_index_u16 = v_fifo_index_u16 +
                BMI160_GEN_READ_WRITE_DATA_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_SENSOR_TIME_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_SENSORTIME_RETURN;
                    break;
                }
                    
                /* Sensor time */
                fifo_header_data->fifo_time =
                    (u32) ((v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_SENSOR_TIME_MSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_16_BITS) |
                           (v_fifo_data_u8[v_fifo_index_u16 +
                                           BMI160_FIFO_SENSOR_TIME_XLSB] <<
                            BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                           v_fifo_data_u8[v_fifo_index_u16 +
                                          BMI160_FIFO_SENSOR_TIME_LSB]);

                v_fifo_index_u16 += BMI160_FIFO_SENSOR_TIME_LENGTH;
            } break;

            /* Header frame of skip frame */
            case FIFO_HEAD_SKIP_FRAME: {
                /*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;
                
                if (v_fifo_index_u16 + BMI160_FIFO_INDEX_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_SKIP_OVER_LEN;
                    break;
                }
                fifo_header_data->skip_frame = v_fifo_data_u8[v_fifo_index_u16];
                v_fifo_index_u16 = v_fifo_index_u16 + BMI160_FIFO_INDEX_LENGTH;
            } break;

            case FIFO_HEAD_INPUT_CONFIG: {
                /*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;
                if (v_fifo_index_u16 + BMI160_FIFO_INDEX_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_INPUT_CONFIG_OVER_LEN;
                    break;
                }
                fifo_header_data->fifo_input_config_info = 
                    v_fifo_data_u8[v_fifo_index_u16];
                v_fifo_index_u16 = v_fifo_index_u16 + BMI160_FIFO_INDEX_LENGTH;
            } break;

            /* Header frame of over read FIFO data */
            case FIFO_HEAD_OVER_READ_LSB: {
                /*fifo data frame index + 1*/
                v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;

                if (v_fifo_index_u16 + BMI160_FIFO_INDEX_LENGTH >
                    v_fifo_length_u16) {
                    v_last_return_stat_s8 = FIFO_OVER_READ_RETURN;
                    break;
                }
                if (v_fifo_data_u8[v_fifo_index_u16] ==
                    FIFO_HEAD_OVER_READ_MSB) {
                    /*fifo over read frame index + 1*/
                    v_fifo_index_u16 += BMI160_FIFO_INDEX_LENGTH;
                } else {
                    v_last_return_stat_s8 = FIFO_OVER_READ_RETURN;
                }
            } break;

            default: {
                v_last_return_stat_s8 = BMI160_FIFO_INDEX_LENGTH;
            } break;
		}
	if (v_last_return_stat_s8 != 0)
		break;
	}

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_headerless_mode(
    u8 v_mag_if_u8, struct bmi160_fifo_data_header_less_t *headerless_data)
{
    /* read the whole FIFO data*/
	return bmi160_read_fifo_headerless_mode_user_defined_length(
            FIFO_FRAME, headerless_data, v_mag_if_u8);
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_fifo_headerless_mode_user_defined_length(
    u16 v_fifo_user_length_u16,
    struct bmi160_fifo_data_header_less_t *fifo_data, u8 v_mag_if_mag_u8)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
    u32 v_fifo_index_u16 = BMI160_INIT_VALUE;
    u32 v_fifo_length_u16 = BMI160_INIT_VALUE;
    u8 v_accel_index_u8 = BMI160_INIT_VALUE;
    u8 v_gyro_index_u8 = BMI160_INIT_VALUE;
    u8 v_mag_index_u8 = BMI160_INIT_VALUE;
    
    BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
    
    fifo_data->accel_frame_count = BMI160_INIT_VALUE;
    fifo_data->mag_frame_count = BMI160_INIT_VALUE;
    fifo_data->gyro_frame_count = BMI160_INIT_VALUE;
    
    /* disable the header data */
    com_rslt = bmi160_set_fifo_header_enable(BMI160_INIT_VALUE);
    
    /* read mag, Accel and gyro enable status*/
    com_rslt += bmi160_read_reg(    
                    BMI160_USER_FIFO_CONFIG_1_ADDR,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    v_data_u8 = v_data_u8 & BMI160_FIFO_M_G_A_ENABLE;
    
    /* read the FIFO data of 1024 bytes*/
    com_rslt += bmi160_fifo_data(
                    &v_fifo_data_u8[BMI160_INIT_VALUE], v_fifo_user_length_u16);
    v_fifo_length_u16 = v_fifo_user_length_u16;
    
    /* loop for executing the different conditions */
    for (v_fifo_index_u16 = BMI160_INIT_VALUE;
         v_fifo_index_u16 < v_fifo_length_u16;) {
        /* condition for mag, gyro and Accel enable*/
        if (v_data_u8 == BMI160_FIFO_M_G_A_ENABLE) {
            /* Raw Mag x*/
            mag_data.mag_x_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_LSB_DATA];
            mag_data.mag_x_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_MSB_DATA];
            
            /* Mag y data*/
            mag_data.mag_y_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_LSB_DATA];
            mag_data.mag_y_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_MSB_DATA];
            
            /* Mag z data*/
            mag_data.mag_z_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_LSB_DATA];
            mag_data.mag_z_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_MSB_DATA];
            
            /* Mag r data*/
            mag_data.mag_r_y2_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_LSB_DATA];
            mag_data.mag_r_y2_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_MSB_DATA];
            
            com_rslt = 
                bmi160_second_if_mag_compensate_xyz(mag_data, v_mag_if_mag_u8);
            
            /* compensated Mag x */
            fifo_data->mag_fifo[v_mag_index_u8].x = processed_data.x;
            
            /* compensated Mag y */
            fifo_data->mag_fifo[v_mag_index_u8].y = processed_data.y;
            
            /* compensated Mag z */
            fifo_data->mag_fifo[v_mag_index_u8].z = processed_data.z;
            
            /* check for Mag frame count*/
            fifo_data->mag_frame_count += BMI160_FRAME_COUNT;
            
            /* Gyro raw x v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].x  =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MGA_FIFO_G_X_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MGA_FIFO_G_X_LSB]);
            
            /* Gyro raw y v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MGA_FIFO_G_Y_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MGA_FIFO_G_Y_LSB]);
            
            /* Gyro raw z v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].z =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MGA_FIFO_G_Z_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MGA_FIFO_G_Z_LSB]);
            
            /* check for gyro frame count*/
            fifo_data->gyro_frame_count += BMI160_FRAME_COUNT;
            
            /* Accel raw x v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].x =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MGA_FIFO_A_X_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MGA_FIFO_A_X_LSB]);
            
            /* Accel raw y v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MGA_FIFO_A_Y_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MGA_FIFO_A_Y_LSB]);
            
            /* Accel raw z v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].z =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MGA_FIFO_A_Z_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MGA_FIFO_A_Z_LSB]);
            
            /* check for Accel frame count*/
            fifo_data->accel_frame_count += BMI160_FRAME_COUNT;
            
            v_accel_index_u8++;
            v_mag_index_u8++;
            v_gyro_index_u8++;
           
            v_fifo_index_u16 += BMI160_FIFO_AMG_LENGTH;
        } else if (v_data_u8 == BMI160_FIFO_M_G_ENABLE) {
        /* condition for Mag and gyro enable*/
            
            /* Raw Mag x*/
            mag_data.mag_x_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_LSB_DATA];
            mag_data.mag_x_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_MSB_DATA];
            
            /* Mag y data*/
            mag_data.mag_y_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_LSB_DATA];
            mag_data.mag_y_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_MSB_DATA];
            
            /* Mag z data*/
            mag_data.mag_z_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_LSB_DATA];
            mag_data.mag_z_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_MSB_DATA];
            
            /* Mag r data*/
            mag_data.mag_r_y2_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_LSB_DATA];
            mag_data.mag_r_y2_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_MSB_DATA];
             
            com_rslt =
                bmi160_second_if_mag_compensate_xyz(mag_data, v_mag_if_mag_u8);
             
             /* compensated Mag x */
            fifo_data->mag_fifo[v_mag_index_u8].x = processed_data.x;
            
            /* compensated Mag y */
            fifo_data->mag_fifo[v_mag_index_u8].y = processed_data.y;
            
            /* compensated Mag z */
            fifo_data->mag_fifo[v_mag_index_u8].z = processed_data.z;
            
            /* check for Mag frame count*/
            fifo_data->mag_frame_count += BMI160_FRAME_COUNT;
            
            /* Gyro raw x v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].x  =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MG_FIFO_G_X_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MG_FIFO_G_X_LSB]);
            
            /* Gyro raw y v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MG_FIFO_G_Y_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MG_FIFO_G_Y_LSB]);
            
            /* Gyro raw z v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].z  =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MG_FIFO_G_Z_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MG_FIFO_G_Z_LSB]);
            
            /* check for gyro frame count*/
            fifo_data->gyro_frame_count += BMI160_FRAME_COUNT;
            
            v_gyro_index_u8++;
            v_mag_index_u8++;
            v_fifo_index_u16 += BMI160_FIFO_MA_OR_MG_LENGTH;
        } else if (v_data_u8 == BMI160_FIFO_M_A_ENABLE) {
        /* condition for Mag and Accel enable*/
            
            /* Raw Mag x*/
            mag_data.mag_x_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_LSB_DATA];
            mag_data.mag_x_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_MSB_DATA];
            
            /* Mag y data*/
            mag_data.mag_y_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_LSB_DATA];
            mag_data.mag_y_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_MSB_DATA];
            
            /* Mag z data*/
            mag_data.mag_z_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_LSB_DATA];
            mag_data.mag_z_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_MSB_DATA];
                
            /* Mag r data*/
            mag_data.mag_r_y2_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_LSB_DATA];
            mag_data.mag_r_y2_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_MSB_DATA];
            
            com_rslt = 
                bmi160_second_if_mag_compensate_xyz(mag_data, v_mag_if_mag_u8);
             
            /* compensated Mag x */
            fifo_data->mag_fifo[v_mag_index_u8].x = processed_data.x;
            /* compensated Mag y */
            
            fifo_data->mag_fifo[v_mag_index_u8].y = processed_data.y;
            
            /* compensated Mag z */
            fifo_data->mag_fifo[v_mag_index_u8].z = processed_data.z;
            
            /* check for Mag frame count*/
            fifo_data->mag_frame_count += BMI160_FRAME_COUNT;
            
            /* Accel raw x v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].x =
            (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                   BMI160_MA_FIFO_A_X_MSB] <<
                    BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                   v_fifo_data_u8[v_fifo_index_u16 +
                                  BMI160_MA_FIFO_A_X_LSB]);
            
            /* Accel raw y v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MA_FIFO_A_Y_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MA_FIFO_A_Y_LSB]);
            
            /* Accel raw z v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].z =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_MA_FIFO_A_Z_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_MA_FIFO_A_Z_LSB]);
            
            /* check for Accel frame count*/
            fifo_data->accel_frame_count += BMI160_FRAME_COUNT;
            
            v_accel_index_u8++;
            v_mag_index_u8++;
            
            v_fifo_index_u16 += BMI160_FIFO_MA_OR_MG_LENGTH;
        } else if (v_data_u8 == BMI160_FIFO_G_A_ENABLE) {           
        /* condition for gyro and Accel enable*/
            
            /* Gyro raw x v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].x  =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_GA_FIFO_G_X_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_GA_FIFO_G_X_LSB]);
            
            /* Gyro raw y v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_GA_FIFO_G_Y_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_GA_FIFO_G_Y_LSB]);
            
            /* Gyro raw z v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].z  =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_GA_FIFO_G_Z_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_GA_FIFO_G_Z_LSB]);
            
            /* check for gyro frame count*/
            fifo_data->gyro_frame_count += BMI160_FRAME_COUNT;
            
            /* Accel raw x v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].x =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_GA_FIFO_A_X_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_GA_FIFO_A_X_LSB]);
            
            /* Accel raw y v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_GA_FIFO_A_Y_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_GA_FIFO_A_Y_LSB]);
            
            /* Accel raw z v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].z =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_GA_FIFO_A_Z_MSB] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_GA_FIFO_A_Z_LSB]);
            
            /* check for Accel frame count*/
            fifo_data->accel_frame_count += BMI160_FRAME_COUNT;
            
            v_accel_index_u8++;
            v_gyro_index_u8++;
            
            v_fifo_index_u16 += BMI160_FIFO_AG_LENGTH;
        } else if (v_data_u8 == BMI160_FIFO_GYRO_ENABLE) {
        /* condition  for gyro enable*/
            
            /* Gyro raw x v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].x  =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_FIFO_X_MSB_DATA] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 |
                                      BMI160_FIFO_X_LSB_DATA]);
            
            /* Gyro raw y v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_FIFO_Y_MSB_DATA] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_FIFO_Y_LSB_DATA]);
            
            /* Gyro raw z v_data_u8 */
            fifo_data->gyro_fifo[v_gyro_index_u8].z  =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_FIFO_Z_MSB_DATA] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 +
                                      BMI160_FIFO_Z_LSB_DATA]);
            
            /* check for gyro frame count*/
            fifo_data->gyro_frame_count += BMI160_FRAME_COUNT;
            
            v_fifo_index_u16 += BMI160_FIFO_G_LENGTH;
            
            v_gyro_index_u8++;
        } else if (v_data_u8 == BMI160_FIFO_A_ENABLE) {
        /* condition  for Accel enable*/
            
            /* Accel raw x v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].x =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_FIFO_X_MSB_DATA] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 + 
                                      BMI160_FIFO_X_LSB_DATA]);
            
            /* Accel raw y v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].y =
                (s16) ((v_fifo_data_u8[v_fifo_index_u16 +
                                       BMI160_FIFO_Y_MSB_DATA] <<
                        BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                       v_fifo_data_u8[v_fifo_index_u16 + 
                                      BMI160_FIFO_Y_LSB_DATA]);
            
            /* Accel raw z v_data_u8 */
            fifo_data->accel_fifo[v_accel_index_u8].z =
            (s16) ((v_fifo_data_u8[v_fifo_index_u16 |
                                   BMI160_FIFO_Z_MSB_DATA] <<
                    BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
                   v_fifo_data_u8[v_fifo_index_u16 + 
                                  BMI160_FIFO_Z_LSB_DATA]);
            
            /* check for Accel frame count*/
            fifo_data->accel_frame_count += BMI160_FRAME_COUNT;
            
            v_fifo_index_u16 += BMI160_FIFO_A_LENGTH;
            v_accel_index_u8++;
        } else if (v_data_u8 == BMI160_FIFO_M_ENABLE) {
        /* condition  for Mag enable*/
            
            /* Raw Mag x*/
            mag_data.mag_x_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_LSB_DATA];
            mag_data.mag_x_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_X_MSB_DATA];
            
            /* Mag y data*/
            mag_data.mag_y_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_LSB_DATA];
            mag_data.mag_y_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Y_MSB_DATA];
            
            /* Mag z data*/
            mag_data.mag_z_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_LSB_DATA];
            mag_data.mag_z_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_Z_MSB_DATA];
            
            /* Mag r data*/
            mag_data.mag_r_y2_lsb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_LSB_DATA];
            mag_data.mag_r_y2_msb =
                v_fifo_data_u8[v_fifo_index_u16 + BMI160_FIFO_R_MSB_DATA];
            
            com_rslt = 
                bmi160_second_if_mag_compensate_xyz(mag_data, v_mag_if_mag_u8);
             
            /* compensated Mag x */
            fifo_data->mag_fifo[v_mag_index_u8].x = processed_data.x;
            
            /* compensated Mag y */
            fifo_data->mag_fifo[v_mag_index_u8].y = processed_data.y;
            
            /* compensated Mag z */
            fifo_data->mag_fifo[v_mag_index_u8].z = processed_data.z;
            
            /* check for Mag frame count*/
            fifo_data->mag_frame_count += BMI160_FRAME_COUNT;
            
            v_fifo_index_u16 += BMI160_FIFO_M_LENGTH;
            v_mag_index_u8++;
        }
        
        /* condition  for FIFO over read enable*/
        if (v_fifo_data_u8[v_fifo_index_u16] == FIFO_CONFIG_CHECK1 &&
            v_fifo_data_u8[v_fifo_index_u16 +
                           BMI160_FIFO_INDEX_LENGTH] == FIFO_CONFIG_CHECK2) {
            break;
        }
    }
    return com_rslt;
}
#endif //defined(FIFO_ENABLE) && defined(INCLUDE_BMI160MAG)

#ifdef INCLUDE_BMI160MAG
BMI160_RETURN_FUNCTION_TYPE bmi160_bmm150_mag_compensate_xyz(
    struct bmi160_mag_xyz_s32_t *mag_comp_xyz)
{
	struct bmi160_mag_xyzr_t mag_xyzr;

	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = bmi160_read_mag_xyzr(&mag_xyzr);

	if (com_rslt != 0) {
		return com_rslt;
    }
    
	/* Compensation for X axis */
	mag_comp_xyz->x = bmi160_bmm150_mag_compensate_X(mag_xyzr.x, mag_xyzr.r);

	/* Compensation for Y axis */
	mag_comp_xyz->y = bmi160_bmm150_mag_compensate_Y(mag_xyzr.y, mag_xyzr.r);

	/* Compensation for Z axis */
	mag_comp_xyz->z = bmi160_bmm150_mag_compensate_Z(mag_xyzr.z, mag_xyzr.r);

	return com_rslt;
}

s32 bmi160_bmm150_mag_compensate_X(
    s16 v_mag_data_x_s16, u16 v_data_r_u16)
{
    s32 inter_retval = BMI160_INIT_VALUE;
    
    /* no overflow */
    if (v_mag_data_x_s16 != BMI160_MAG_FLIP_OVERFLOW_ADCVAL) {
        if (v_data_r_u16 != 0 || mag_trim.dig_xyz1 != 0) {
            inter_retval =
                (s32) (((u16) ((((s32) mag_trim.dig_xyz1) <<
                                BMI160_SHIFT_BIT_POSITION_BY_14_BITS) /
                               (v_data_r_u16 != 0 ? v_data_r_u16
                                                  : mag_trim.dig_xyz1))) -
                       (u16) 0x4000);
        } else {
            inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
            return inter_retval;
        }
        inter_retval = 
            ((s32) ((((s32) v_mag_data_x_s16) *
                     ((((((((s32) mag_trim.dig_xy2) *
                           ((((s32) inter_retval) * ((s32) inter_retval)) >>
                            BMI160_SHIFT_BIT_POSITION_BY_07_BITS)) +
                          (((s32) inter_retval) *
                           ((s32) (((s16) mag_trim.dig_xy1) <<
                                   BMI160_SHIFT_BIT_POSITION_BY_07_BITS)))) >>
                         BMI160_SHIFT_BIT_POSITION_BY_09_BITS) + ((s32) 0x100000)) *
                       ((s32) (((s16) mag_trim.dig_x2) + ((s16) 0xA0)))) >>
                      BMI160_SHIFT_BIT_POSITION_BY_12_BITS)) >>
                    BMI160_SHIFT_BIT_POSITION_BY_13_BITS)) +
            (((s16) mag_trim.dig_x1) << BMI160_SHIFT_BIT_POSITION_BY_03_BITS);
        
        /* check the overflow output */
        if (inter_retval == (s32)BMI160_MAG_OVERFLOW_OUTPUT) {
            inter_retval = BMI160_MAG_OVERFLOW_OUTPUT_S32;
        }
    } else {
        /* overflow */
        inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
    }
    return inter_retval;
}

s32 bmi160_bmm150_mag_compensate_Y(
    s16 v_mag_data_y_s16, u16 v_data_r_u16)
{
    s32 inter_retval = BMI160_INIT_VALUE;
    
    /* no overflow */
    if (v_mag_data_y_s16 != BMI160_MAG_FLIP_OVERFLOW_ADCVAL) {
        if (v_data_r_u16 != 0 || mag_trim.dig_xyz1 != 0) {
            inter_retval = 
                (s32) (((u16) ((((s32) mag_trim.dig_xyz1) <<
                                BMI160_SHIFT_BIT_POSITION_BY_14_BITS) /
                               (v_data_r_u16 != 0 ? v_data_r_u16
                                                  : mag_trim.dig_xyz1))) -
                       (u16) 0x4000);
            } else {
                inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
                return inter_retval;
            }
        inter_retval = 
            ((s32) ((((s32) v_mag_data_y_s16) *
                     ((((((((s32) mag_trim.dig_xy2) *
                           ((((s32) inter_retval) * ((s32) inter_retval)) >>
                            BMI160_SHIFT_BIT_POSITION_BY_07_BITS)) +
                          (((s32) inter_retval) *
                           ((s32) (((s16) mag_trim.dig_xy1) <<
                                   BMI160_SHIFT_BIT_POSITION_BY_07_BITS)))) >>
                         BMI160_SHIFT_BIT_POSITION_BY_09_BITS) +
                        ((s32) 0x100000)) *
                       ((s32) (((s16) mag_trim.dig_y2) + ((s16) 0xA0)))) >>
                      BMI160_SHIFT_BIT_POSITION_BY_12_BITS)) >>
                    BMI160_SHIFT_BIT_POSITION_BY_13_BITS)) +
            (((s16) mag_trim.dig_y1) << BMI160_SHIFT_BIT_POSITION_BY_03_BITS);
        
            /* check the overflow output */
        if (inter_retval == (s32)BMI160_MAG_OVERFLOW_OUTPUT) {
            inter_retval = BMI160_MAG_OVERFLOW_OUTPUT_S32;
        }
    } else {
        /* overflow */
        inter_retval = BMI160_MAG_OVERFLOW_OUTPUT;
    }
    return inter_retval;
}

s32 bmi160_bmm150_mag_compensate_Z(
    s16 v_mag_data_z_s16, u16 v_data_r_u16)
{
	s32 retval = BMI160_INIT_VALUE;

	if (v_mag_data_z_s16 != BMI160_MAG_HALL_OVERFLOW_ADCVAL) {
		if (v_data_r_u16 != 0 && mag_trim.dig_z2 != 0 && mag_trim.dig_z1 != 0) {
			retval = ((((s32) (v_mag_data_z_s16 - mag_trim.dig_z4)) <<
                       BMI160_SHIFT_BIT_POSITION_BY_15_BITS) -
                      ((((s32) mag_trim.dig_z3) *
                        ((s32) (((s16)v_data_r_u16) -
                                ((s16) mag_trim.dig_xyz1)))) >>
                       BMI160_SHIFT_BIT_POSITION_BY_02_BITS)) /
                     (mag_trim.dig_z2 +
                      ((s16) (((((s32) mag_trim.dig_z1) *
                                ((((s16)v_data_r_u16) <<
                                  BMI160_SHIFT_BIT_POSITION_BY_01_BIT))) +
                               (1 << BMI160_SHIFT_BIT_POSITION_BY_15_BITS)) >>
                              BMI160_SHIFT_BIT_POSITION_BY_16_BITS)));
		}
	} else {
		retval = BMI160_MAG_OVERFLOW_OUTPUT;
	}
    return retval;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bmm150_mag_interface_init(
    u8 *v_chip_id_u8)
{
	
    u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_accel_power_mode_status = BMI160_INIT_VALUE;

	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt =
        bmi160_get_accel_power_mode_stat(&v_accel_power_mode_status);
	
    /* Accel operation mode to normal*/
	if (v_accel_power_mode_status != BMI160_ACCEL_NORMAL_MODE) {
		com_rslt += bmi160_set_command_register(ACCEL_MODE_NORMAL);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	
    /* write the Mag power mode as NORMAL*/
	com_rslt += bmi160_set_mag_interface_normal();
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* Write the BMM150 i2c address*/
	com_rslt += bmi160_set_i2c_device_addr(BMI160_AUX_BMM150_I2C_ADDRESS);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* enable the Mag interface to manual mode*/
	com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	bmi160_get_mag_manual_enable(&v_data_u8);
	
    /*Enable the MAG interface */
	com_rslt += bmi160_set_if_mode(BMI160_ENABLE_MAG_IF_MODE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	bmi160_get_if_mode(&v_data_u8);
	
    /* Mag normal mode*/
	com_rslt += bmi160_bmm150_mag_wakeup();
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* Read the BMM150 device id is 0x32*/
	com_rslt += bmi160_set_mag_read_addr(BMI160_BMM150_CHIP_ID);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	*v_chip_id_u8 = v_data_u8;
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* write the power mode register*/
	com_rslt += bmi160_set_mag_write_data(BMI160_BMM_POWER_MODE_REG);
	
    /*write 0x4C register to write set power mode to normal*/
	com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_MODE_REG);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* read the Mag trim values*/
	com_rslt += bmi160_read_bmm150_mag_trim();
	
    /* To avoid the auto mode enable when manual mode operation running*/
	bmm150_manual_auto_condition_u8_g = BMI160_MANUAL_ENABLE;
	
    /* write the XY and Z repetitions*/
	com_rslt += bmi160_set_bmm150_mag_presetmode(BMI160_MAG_PRESETMODE_REGULAR);
	
    /* To avoid the auto mode enable when manual mode operation running*/
	bmm150_manual_auto_condition_u8_g = BMI160_MANUAL_DISABLE;
	
    /* Set the power mode of Mag as force mode*/
	com_rslt += bmi160_set_mag_write_data(BMI160_BMM150_FORCE_MODE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* write into power mode register*/
	com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_MODE_REG);
	
    /* write the Mag v_data_bw_u8 as 25Hz*/
	com_rslt += 
        bmi160_set_mag_output_data_rate(BMI160_MAG_OUTPUT_DATA_RATE_25HZ);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

	/* When Mag interface is in auto mode - The Mag read address
        starts at the register 0x42*/
	com_rslt += bmi160_set_mag_read_addr(BMI160_BMM150_DATA_REG);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* enable Mag interface to auto mode*/
	com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

    switch (v_accel_power_mode_status) {
    case BMI160_ACCEL_SUSPEND:
        com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

    case BMI160_ACCEL_LOW_POWER:
        com_rslt += bmi160_set_command_register(ACCEL_LOWPOWER);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

    default:
        break;
	}
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bmm150_mag_wakeup(void)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = BMI160_INIT_VALUE;
	
    u8 v_try_times_u8 = BMI160_BMM150_MAX_RETRY_WAKEUP;
	u8 v_power_control_bit_u8 = BMI160_INIT_VALUE;
	u8 i = BMI160_INIT_VALUE;

	for (i = BMI160_INIT_VALUE; i < v_try_times_u8; i++) {
		com_rslt = bmi160_set_mag_write_data(BMI160_BMM150_POWER_ON);
		p_bmi160->delay_msec(BMI160_BMM150_WAKEUP_DELAY1);
        
		/*write 0x4B register to enable power control bit*/
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_CONTROL_REG);
		p_bmi160->delay_msec(BMI160_BMM150_WAKEUP_DELAY2);
		com_rslt += bmi160_set_mag_read_addr(BMI160_BMM150_POWER_CONTROL_REG);
		
        /* 0x04 is secondary read Mag x LSB register */
		p_bmi160->delay_msec(BMI160_BMM150_WAKEUP_DELAY3);
		com_rslt += bmi160_read_reg(
                        BMI160_USER_DATA_0_ADDR,
                        &v_power_control_bit_u8, 
                        BMI160_GEN_READ_WRITE_DATA_LENGTH);
		v_power_control_bit_u8 = 
            BMI160_BMM150_SET_POWER_CONTROL & v_power_control_bit_u8;
		if (v_power_control_bit_u8 == BMI160_BMM150_POWER_ON) {
			break;
        }
	}
	com_rslt = (i >= v_try_times_u8) ? BMI160_BMM150_POWER_ON_FAIL
                                     : BMI160_BMM150_POWER_ON_SUCCESS;
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_bmm150_mag_and_secondary_if_power_mode(
    u8 v_mag_sec_if_pow_mode_u8)
{
    u8 v_accel_power_mode_status = BMI160_INIT_VALUE;
	
    /* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_get_accel_power_mode_stat(&v_accel_power_mode_status);
	
    /* set the Accel power mode to NORMAL*/
	if (v_accel_power_mode_status != BMI160_ACCEL_NORMAL_MODE) {
		com_rslt += bmi160_set_command_register(ACCEL_MODE_NORMAL);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}

	switch (v_mag_sec_if_pow_mode_u8) {
	case BMI160_MAG_FORCE_MODE:
		/* set the secondary Mag power mode as NORMAL*/
		com_rslt += bmi160_set_mag_interface_normal();
    
		/* set the Mag power mode as FORCE mode*/
		com_rslt += bmi160_bmm150_mag_set_power_mode(FORCE_MODE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_MAG_SUSPEND_MODE:
		/* set the Mag power mode as SUSPEND mode*/
		com_rslt += bmi160_bmm150_mag_set_power_mode(SUSPEND_MODE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the secondary Mag power mode as SUSPEND*/
		com_rslt += bmi160_set_command_register(MAG_MODE_SUSPEND);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
	if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
		/* set Mag interface auto mode*/
		com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}

    switch (v_accel_power_mode_status) {
    case BMI160_ACCEL_SUSPEND:
        com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

    case BMI160_ACCEL_LOW_POWER:
        com_rslt += bmi160_set_command_register(ACCEL_LOWPOWER);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

    default:
        break;
	}
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bmm150_mag_set_power_mode(
    u8 v_mag_pow_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* set Mag interface manual mode*/
	if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
		com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		if (com_rslt != SUCCESS) {
			return com_rslt;
        }
	} else {
		com_rslt = SUCCESS;
	}
    
	switch (v_mag_pow_mode_u8) {
	case FORCE_MODE:
		/* Set the power control bit enabled */
		com_rslt = bmi160_bmm150_mag_wakeup();
    
		/* write the Mag power mode as FORCE mode*/
		com_rslt += bmi160_set_mag_write_data(BMI160_BMM150_FORCE_MODE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(
		BMI160_BMM150_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
		
        /* To avoid the auto mode enable when manual mode operation running*/
		bmm150_manual_auto_condition_u8_g = BMI160_MANUAL_ENABLE;
		
        /* set the preset mode */
		com_rslt += 
            bmi160_set_bmm150_mag_presetmode(BMI160_MAG_PRESETMODE_REGULAR);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* To avoid the auto mode enable when manual mode operation running*/
		bmm150_manual_auto_condition_u8_g = BMI160_MANUAL_DISABLE;
		
        /* set the Mag read address to data registers*/
		com_rslt += bmi160_set_mag_read_addr(BMI160_BMM150_DATA_REG);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case SUSPEND_MODE:
		/* Set the power mode of Mag as suspend mode*/
		com_rslt = bmi160_set_mag_write_data(BMI160_BMM150_POWER_OFF);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_CONTROL_REG);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
    
	/* set Mag interface auto mode*/
	if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
		com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_bmm150_mag_presetmode(
    u8 v_mode_u8)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* set Mag interface manual mode*/
	if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
        com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
    }
	
    switch (v_mode_u8) {
	case BMI160_MAG_PRESETMODE_LOWPOWER:
		/* write the XY and Z repetitions*/
		com_rslt = bmi160_set_mag_write_data(BMI160_MAG_LOWPOWER_REPXY);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_XY_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* write the Z repetitions*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_LOWPOWER_REPZ);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_Z_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the Mag v_data_u8 rate as 10 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_LOWPOWER_DR);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_MAG_PRESETMODE_REGULAR:
		/* write the XY and Z repetitions*/
		com_rslt = bmi160_set_mag_write_data(BMI160_MAG_REGULAR_REPXY);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_XY_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* write the Z repetitions*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_REGULAR_REPZ);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_Z_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the Mag v_data_u8 rate as 10 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_REGULAR_DR);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_MAG_PRESETMODE_HIGHACCURACY:
		/* write the XY and Z repetitions*/
		com_rslt = bmi160_set_mag_write_data(BMI160_MAG_HIGHACCURACY_REPXY);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_XY_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* write the Z repetitions*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_HIGHACCURACY_REPZ);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_Z_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the Mag v_data_u8 rate as 20 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_HIGHACCURACY_DR);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_MAG_PRESETMODE_ENHANCED:
		/* write the XY and Z repetitions*/
		com_rslt = bmi160_set_mag_write_data(BMI160_MAG_ENHANCED_REPXY);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_XY_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* write the Z repetitions*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_ENHANCED_REPZ);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_Z_REP);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set the Mag v_data_u8 rate as 10 to the register 0x4C*/
		com_rslt += bmi160_set_mag_write_data(BMI160_MAG_ENHANCED_DR);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
	
    if (bmm150_manual_auto_condition_u8_g == BMI160_MANUAL_DISABLE) {
        com_rslt += bmi160_set_mag_write_data(BMI160_BMM150_FORCE_MODE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_mag_write_addr(BMI160_BMM150_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_mag_read_addr(BMI160_BMM150_DATA_REG);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        /* set Mag interface auto mode*/
		if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
			com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
		}
    }
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_bmm150_mag_trim(void)
{
	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* Array holding the bmm150 trim data */
	u8 v_data_u8[BMI160_MAG_TRIM_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* read dig_x1 value */
	com_rslt = bmi160_set_mag_read_addr(BMI160_MAG_DIG_X1);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_X1],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_x1 = v_data_u8[BMI160_BMM150_DIG_X1];
	
    /* read dig_y1 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Y1);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_Y1],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_y1 = v_data_u8[BMI160_BMM150_DIG_Y1];

	/* read dig_x2 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_X2);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_X2],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_x2 = v_data_u8[BMI160_BMM150_DIG_X2];
	
    /* read dig_y2 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Y2);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_Y3],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_y2 = v_data_u8[BMI160_BMM150_DIG_Y3];

	/* read dig_xy1 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XY1);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_XY1],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_xy1 = v_data_u8[BMI160_BMM150_DIG_XY1];
	
    /* read dig_xy2 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XY2);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is v_mag_x_s16 ls register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_XY2],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_xy2 = v_data_u8[BMI160_BMM150_DIG_XY2];

	/* read dig_z1 LSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z1_LSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_Z1_LSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* read dig_z1 MSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z1_MSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is v_mag_x_s16 MSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_Z1_MSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_z1 =
        (u16) ((((u32) ((u8) v_data_u8[BMI160_BMM150_DIG_Z1_MSB])) <<
                BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               v_data_u8[BMI160_BMM150_DIG_Z1_LSB]);

	/* read dig_z2 LSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z2_LSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_Z2_LSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* read dig_z2 MSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z2_MSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is v_mag_x_s16 MSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_Z2_MSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_z2 =
        (s16) ((((s32) ((s8) v_data_u8[BMI160_BMM150_DIG_Z2_MSB])) <<
                BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               v_data_u8[BMI160_BMM150_DIG_Z2_LSB]);

	/* read dig_z3 LSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z3_LSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_DIG_Z3_LSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* read dig_z3 MSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z3_MSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is v_mag_x_s16 MSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_DIG_Z3_MSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_z3 =
        (s16) ((((s32) ((s8) v_data_u8[BMI160_BMM150_DIG_DIG_Z3_MSB])) <<
			    BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
			   v_data_u8[BMI160_BMM150_DIG_DIG_Z3_LSB]);
	
    /* read dig_z4 LSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z4_LSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_DIG_Z4_LSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* read dig_z4 MSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_Z4_MSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is v_mag_x_s16 MSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_DIG_Z4_MSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_z4 =
        (s16) ((((s32) ((s8) v_data_u8[BMI160_BMM150_DIG_DIG_Z4_MSB])) <<
                BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               v_data_u8[BMI160_BMM150_DIG_DIG_Z4_LSB]);

	/* read dig_xyz1 LSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XYZ1_LSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_DIG_XYZ1_LSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* read dig_xyz1 MSB value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_MAG_DIG_XYZ1_MSB);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is v_mag_x_s16 MSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[BMI160_BMM150_DIG_DIG_XYZ1_MSB],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	mag_trim.dig_xyz1 =
        (u16) ((((u32) ((u8) v_data_u8[BMI160_BMM150_DIG_DIG_XYZ1_MSB])) <<
                BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               v_data_u8[BMI160_BMM150_DIG_DIG_XYZ1_LSB]);

	return com_rslt;
}

    #ifdef AKM09912
s32 bmi160_bst_akm09912_compensate_X(s16 v_bst_akm_x_s16)
{
    /* Convert raw data into compensated data*/
	return v_bst_akm_x_s16 *
           (akm_asa_data.asax + AKM09912_SENSITIVITY) /
           AKM09912_SENSITIVITY_DIV;
}

s32 bmi160_bst_akm09912_compensate_Y(
    s16 v_bst_akm_y_s16)
{
    /* Convert raw data into compensated data*/
	return v_bst_akm_y_s16 *
           (akm_asa_data.asax + AKM09912_SENSITIVITY) /
           AKM09912_SENSITIVITY_DIV;
}

s32 bmi160_bst_akm09912_compensate_Z(
    s16 v_bst_akm_z_s16)
{
	/* Convert raw data into compensated data*/
	return v_bst_akm_z_s16 *
           (akm_asa_data.asax + AKM09912_SENSITIVITY) /
           AKM09912_SENSITIVITY_DIV;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm09912_compensate_xyz(
    struct bmi160_bst_akm_xyz_t *bst_akm_xyz)
{
	struct bmi160_mag_t mag_xyz;

	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_read_mag_xyz(&mag_xyz, BST_AKM);

	/* Compensation for X axis */
	bst_akm_xyz->x = bmi160_bst_akm09912_compensate_X(mag_xyz.x);

	/* Compensation for Y axis */
	bst_akm_xyz->y = bmi160_bst_akm09912_compensate_Y(mag_xyz.y);

	/* Compensation for Z axis */
	bst_akm_xyz->z = bmi160_bst_akm09912_compensate_Z(mag_xyz.z);

	return com_rslt;
}
    #endif //AKM09912

    #ifdef AKM09911
s32 bmi160_bst_akm09911_compensate_X(
    s16 v_bst_akm_x_s16)
{
	/* Convert raw v_data_u8 into compensated v_data_u8*/
	return v_bst_akm_x_s16 *
           (akm_asa_data.asax / AKM09911_SENSITIVITY_DIV +
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
}

s32 bmi160_bst_akm09911_compensate_Y(
    s16 v_bst_akm_y_s16)
{
	/* Convert raw v_data_u8 into compensated v_data_u8*/
	return v_bst_akm_y_s16 *
           (akm_asa_data.asay / AKM09911_SENSITIVITY_DIV +
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
}

s32 bmi160_bst_akm09911_compensate_Z(
    s16 v_bst_akm_z_s16)
{
	/* Convert raw v_data_u8 into compensated v_data_u8*/
	return v_bst_akm_z_s16 *
           (akm_asa_data.asaz / AKM09911_SENSITIVITY_DIV +
            BMI160_GEN_READ_WRITE_DATA_LENGTH);
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm09911_compensate_xyz(
    struct bmi160_bst_akm_xyz_t *bst_akm_xyz)
{
	struct bmi160_mag_t mag_xyz;

	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_read_mag_xyz(&mag_xyz, BST_AKM);

	/* Compensation for X axis */
	bst_akm_xyz->x = bmi160_bst_akm09911_compensate_X(mag_xyz.x);

	/* Compensation for Y axis */
	bst_akm_xyz->y = bmi160_bst_akm09911_compensate_Y(mag_xyz.y);

	/* Compensation for Z axis */
	bst_akm_xyz->z = bmi160_bst_akm09911_compensate_Z(mag_xyz.z);

	return com_rslt;
}
    #endif //AKM09911

    #if defined AKM09911 || defined AKM09912
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_mag_interface_init(
    u8 v_akm_i2c_address_u8)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 v_akm_chip_id_u8 = BMI160_INIT_VALUE;
	u8 v_accel_power_mode_status = BMI160_INIT_VALUE;

	/* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_get_accel_power_mode_stat(&v_accel_power_mode_status);
	
    /* set Accel operation mode to normal*/
	if (v_accel_power_mode_status != BMI160_ACCEL_NORMAL_MODE) {
		com_rslt += bmi160_set_command_register(ACCEL_MODE_NORMAL);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	
    com_rslt += bmi160_set_command_register(MAG_MODE_NORMAL);
	p_bmi160->delay_msec(BMI160_AKM_INIT_DELAY);
	bmi160_get_mag_power_mode_stat(&v_data_u8);
	
    /* Write the AKM09911 0r AKM09912 i2c address*/
	com_rslt += bmi160_set_i2c_device_addr(v_akm_i2c_address_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* enable the Mag interface to manual mode*/
	com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /*Enable the MAG interface */
	com_rslt += bmi160_set_if_mode(BMI160_ENABLE_MAG_IF_MODE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    bmi160_get_if_mode(&v_data_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

	/* Set the AKM Fuse ROM mode */
	com_rslt += bmi160_set_mag_write_data(AKM_FUSE_ROM_MODE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* AKM mode address is 0x31*/
	com_rslt += bmi160_set_mag_write_addr(AKM_POWER_MODE_REG);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* Read the Fuse ROM v_data_u8 from registers
        0x60,0x61 and 0x62*/
	/* ASAX v_data_u8 */
	com_rslt += bmi160_read_bst_akm_sensitivity_data();
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* read the device id of the AKM sensor
        if device id is 0x05 - AKM09911
        if device id is 0x04 - AKM09912*/
	com_rslt += bmi160_set_mag_read_addr(AKM_CHIP_ID_REG);

    /* 0x04 is mag_x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_akm_chip_id_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* Set power down mode*/
	com_rslt += bmi160_set_mag_write_data(AKM_POWER_DOWN_MODE_DATA);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* AKM mode address is 0x31*/
	com_rslt += bmi160_set_mag_write_addr(AKM_POWER_MODE_REG);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* Set AKM Force mode*/
	com_rslt += bmi160_set_mag_write_data(AKM_SINGLE_MEASUREMENT_MODE);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* AKM mode address is 0x31*/
	com_rslt += bmi160_set_mag_write_addr(AKM_POWER_MODE_REG);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* Set the AKM read xyz v_data_u8 address*/
	com_rslt += bmi160_set_mag_read_addr(AKM_DATA_REGISTER);
	
    /* write the Mag v_data_bw_u8 as 25Hz*/
	com_rslt += 
        bmi160_set_mag_output_data_rate(BMI160_MAG_OUTPUT_DATA_RATE_25HZ);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* Enable Mag interface to auto mode*/
	com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    switch (v_accel_power_mode_status) {
    case BMI160_ACCEL_SUSPEND:
        com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

    case BMI160_ACCEL_LOW_POWER:
        com_rslt += bmi160_set_command_register(ACCEL_LOWPOWER);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;

    default:
        break;
	}
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_read_bst_akm_sensitivity_data(void)
{
	/* This variable is used to provide the communication results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* Array holding the sensitivity ax,ay and az data*/
	u8 v_data_u8[BMI160_AKM_SENSITIVITY_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* read asax value */
	com_rslt = bmi160_set_mag_read_addr(BMI160_BST_AKM_ASAX);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[AKM_ASAX],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	akm_asa_data.asax = v_data_u8[AKM_ASAX];
	
    /* read asay value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_BST_AKM_ASAY);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[AKM_ASAY],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	akm_asa_data.asay = v_data_u8[AKM_ASAY];
	
    /* read asaz value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_BST_AKM_ASAZ);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[AKM_ASAZ],
                BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	akm_asa_data.asaz = v_data_u8[AKM_ASAZ];

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_akm_set_powermode(
    u8 v_akm_pow_mode_u8)
{
	/* variable is used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* set Mag interface manual mode*/
	if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
		com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
    
	switch (v_akm_pow_mode_u8) {
	case AKM_POWER_DOWN_MODE:
		/* Set the power mode of AKM as power down mode*/
		com_rslt += bmi160_set_mag_write_data(AKM_POWER_DOWN_MODE_DATA);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_mag_write_addr(AKM_POWER_MODE_REG);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        break;
	case AKM_SINGLE_MEAS_MODE:
		/* Set the power mode of AKM as single measurement mode*/
		com_rslt += bmi160_set_mag_write_data(AKM_SINGLE_MEASUREMENT_MODE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_mag_write_addr(AKM_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_mag_read_addr(AKM_DATA_REGISTER);
        break;
	case FUSE_ROM_MODE:
		/* Set the power mode of AKM as Fuse ROM mode*/
		com_rslt += bmi160_set_mag_write_data(AKM_FUSE_ROM_MODE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
		com_rslt += bmi160_set_mag_write_addr(AKM_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
		/* Sensitivity v_data_u8 */
		com_rslt += bmi160_read_bst_akm_sensitivity_data();
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
		
        /* power down mode*/
		com_rslt += bmi160_set_mag_write_data(AKM_POWER_DOWN_MODE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
		com_rslt += bmi160_set_mag_write_addr(AKM_POWER_MODE_REG);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
    
	/* set Mag interface auto mode*/
	if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
		com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_set_bst_akm_and_secondary_if_powermode(
    u8 v_mag_sec_if_pow_mode_u8)
{
	u8 v_accel_power_mode_status = BMI160_INIT_VALUE;
	
    /* variable used to return the status of communication result*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_get_accel_power_mode_stat(&v_accel_power_mode_status);

	/* Accel operation mode to normal*/
	if (v_accel_power_mode_status != BMI160_ACCEL_NORMAL_MODE) {
		com_rslt += bmi160_set_command_register(ACCEL_MODE_NORMAL);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	
    /* set Mag interface manual mode*/
	if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
		com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	
    switch (v_mag_sec_if_pow_mode_u8) {
	case BMI160_MAG_FORCE_MODE:
		/* set the secondary Mag power mode as NORMAL*/
		com_rslt += bmi160_set_mag_interface_normal();
    
		/* set the akm power mode as single measurement mode*/
		com_rslt += bmi160_bst_akm_set_powermode(AKM_SINGLE_MEAS_MODE);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
		com_rslt += bmi160_set_mag_read_addr(AKM_DATA_REGISTER);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
	case BMI160_MAG_SUSPEND_MODE:
		/* set the akm power mode as power down mode*/
		com_rslt += bmi160_bst_akm_set_powermode(AKM_POWER_DOWN_MODE);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
		
        /* set the secondary Mag power mode as SUSPEND*/
		com_rslt += bmi160_set_command_register(MAG_MODE_SUSPEND);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        break;
	default:
		com_rslt = E_BMI160_OUT_OF_RANGE;
        break;
	}
	
    /* set Mag interface auto mode*/
	if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
		com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
    }
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    switch (v_accel_power_mode_status) {

	case BMI160_ACCEL_SUSPEND:
		com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		break;

	case BMI160_ACCEL_LOW_POWER:
		com_rslt += bmi160_set_command_register(ACCEL_LOWPOWER);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		break;

	default:
		break;
	}
	return com_rslt;
}
    #endif //defined AKM09911 || defined AKM09912

    #ifdef YAS532
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yamaha_yas532_mag_interface_init(void)
{
	u8 v_data_u8 = BMI160_INIT_VALUE;
	u8 i = BMI160_INIT_VALUE;
	u8 v_accel_power_mode_status = BMI160_INIT_VALUE;

	/* This variable used to provide the communication results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_get_accel_power_mode_stat(&v_accel_power_mode_status);
	
    /* Accel operation mode to normal*/
	if (v_accel_power_mode_status != BMI160_ACCEL_NORMAL_MODE) {
		com_rslt += bmi160_set_command_register(ACCEL_MODE_NORMAL);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	}
	
    /* write Mag power mode as NORMAL*/
	com_rslt += bmi160_set_mag_interface_normal();
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* Write the YAS532 i2c address*/
	com_rslt += bmi160_set_i2c_device_addr(BMI160_AUX_YAS532_I2C_ADDRESS);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* enable the Mag interface to manual mode*/
	com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /*Enable the MAG interface */
	com_rslt += bmi160_set_if_mode(BMI160_ENABLE_MAG_IF_MODE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    bmi160_get_if_mode(&v_data_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* Read the YAS532 device id is 0x02*/
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS_DEVICE_ID_REG);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

    /* Read the YAS532 calibration data*/
	com_rslt += bmi160_bst_yamaha_yas532_calib_values();
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* Assign the data acquisition mode*/
	yas532_data.measure_state = YAS532_MAG_STATE_INIT_COIL;
	
    /* Set the default offset as invalid offset*/
	set_vector(yas532_data.v_hard_offset_s8, INVALID_OFFSET);
	
    /* set the transform to zero */
	yas532_data.transform = BMI160_NULL;
	
    /* Assign overflow as zero*/
	yas532_data.overflow = 0;
        #if 1 < YAS532_MAG_TEMPERATURE_LOG
    yas532_data.temp_data.num =
    yas532_data.temp_data.idx = 0;
        #endif //1 < YAS532_MAG_TEMPERATURE_LOG
	
    /* Assign the coefficient value*/
	for (i = 0; i < 3; i++) {
		yas532_data.coef[i] = yas532_version_ac_coef[i];
		yas532_data.last_raw[i] = 0;
	}
	yas532_data.last_raw[3] = 0;
	
    /* Set the initial values of yas532*/
	com_rslt += bmi160_bst_yas532_set_initial_values();
	
    /* write the Mag v_data_bw_u8 as 25Hz*/
	com_rslt += 
        bmi160_set_mag_output_data_rate(BMI160_MAG_OUTPUT_DATA_RATE_25HZ);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    /* Enable Mag interface to auto mode*/
	com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
	
    bmi160_get_mag_manual_enable(&v_data_u8);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    switch (v_accel_power_mode_status) {
    case BMI160_ACCEL_SUSPEND:
        com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
    case BMI160_ACCEL_LOW_POWER:
        com_rslt += bmi160_set_command_register(ACCEL_LOWPOWER);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
    default:
        break;
	}

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_set_initial_values(void)
{
	/* write testr1 as 0x00*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_set_mag_write_data(BMI160_YAS532_WRITE_TESTR1);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

	com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_TESTR1);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

	/* write testr2 as 0x00*/
	com_rslt += bmi160_set_mag_write_data(BMI160_YAS532_WRITE_TESTR2);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_TESTR2);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    /* write Rcoil as 0x00*/
	com_rslt += bmi160_set_mag_write_data(BMI160_YAS532_WRITE_RCOIL);
	p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
	
    com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_RCOIL);
	p_bmi160->delay_msec(BMI160_YAS532_SET_INITIAL_VALUE_DELAY);
	
    /* check the valid offset*/
	if (is_valid_offset(yas532_data.v_hard_offset_s8)) {
		com_rslt += bmi160_bst_yas532_set_offset(
		yas532_data.v_hard_offset_s8);
		yas532_data.measure_state = YAS532_MAG_STATE_NORMAL;
	} else {
		/* set the default offset as invalid offset*/
		set_vector(yas532_data.v_hard_offset_s8, INVALID_OFFSET);
        
		/*Set the default measure state for offset correction*/
		yas532_data.measure_state = YAS532_MAG_STATE_MEASURE_OFFSET;
	}
    
	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_magnetic_measure_set_offset(void)
{
	/* This variable used to provide the communication results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* to set  the offset register*/
	s8 v_hard_offset_s8[BMI160_HARD_OFFSET_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* offset correction factors*/
	static const u8 v_correct_u8[BMI160_YAS_CORRECT_DATA_SIZE] = {
        16, 8, 4, 2, 1
    };
	
    /* used to store the temperature */
	u16 v_temp_u16 = BMI160_INIT_VALUE;
	
    /* used to read for the xy1y2 value */
	u16 v_xy1y2_u16[BMI160_YAS_XY1Y2_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* local flag for assigning the values*/
	s32 v_flag_s32[BMI160_YAS_FLAG_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    u8 i, j, v_busy_u8, v_overflow_u8 = BMI160_INIT_VALUE;

	for (i = 0; i < 5; i++) {
		/* set the offset values*/
		com_rslt = bmi160_bst_yas532_set_offset(v_hard_offset_s8);
        
		/* read the sensor data*/
		com_rslt += bmi160_bst_yas532_normal_measurement_data(
                        BMI160_YAS532_ACQ_START, 
                        &v_busy_u8, 
                        &v_temp_u16,
                        v_xy1y2_u16, 
                        &v_overflow_u8);
		
        /* check the sensor busy status*/
		if (v_busy_u8) {
			return E_BMI160_BUSY;
        }
        
		/* calculate the magnetic correction with
            offset and assign the values
            to the offset register */
		for (j = 0; j < 3; j++) {
			if (YAS532_DATA_CENTER == v_xy1y2_u16[j]) {
				v_flag_s32[j] = 0;
			}
            if (YAS532_DATA_CENTER < v_xy1y2_u16[j]) {
				v_flag_s32[j] = 1;
			}
            if (v_xy1y2_u16[j] < YAS532_DATA_CENTER) {
				v_flag_s32[j] = -1;
            }
		}        
		for (j = 0; j < 3; j++) {
			if (v_flag_s32[j]) {
				v_hard_offset_s8[j] = (s8)(v_hard_offset_s8[j] +
                                           v_flag_s32[j] * v_correct_u8[i]);
            }
		}
	}
    
	/* set the offset */
	com_rslt += bmi160_bst_yas532_set_offset(v_hard_offset_s8);
	
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yamaha_yas532_calib_values(void)
{
	/* Array holding the YAS532 calibration values */
	u8 v_data_u8[BMI160_YAS532_CALIB_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* Read the DX value */
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_set_mag_read_addr(BMI160_YAS532_CALIB_CX);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

	/* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[0], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	yas532_data.calib_yas532.cx = (s32)((v_data_u8[0] * 10) - 1280);
	
    /* Read the DY1 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB_CY1);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[1], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	yas532_data.calib_yas532.cy1 = (s32)((v_data_u8[1] * 10) - 1280);
	
    /* Read the DY2 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB_CY2);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[2], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	yas532_data.calib_yas532.cy2 = (s32)((v_data_u8[2] * 10) - 1280);
	
    /* Read the D2 and D3 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB1);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[3], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	yas532_data.calib_yas532.a2 =
        (s32) (((v_data_u8[3] >> BMI160_SHIFT_BIT_POSITION_BY_02_BITS) &
                0x03F) - 32);
	
    /* Read the D3 and D4 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB2);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[4], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* calculate a3*/
	yas532_data.calib_yas532.a3 = 
        (s32) ((((v_data_u8[3] << BMI160_SHIFT_BIT_POSITION_BY_02_BITS) &
                 0x0C) |
                ((v_data_u8[4] >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS) &
                 0x03)) - 8);
	
    /* calculate a4*/
	yas532_data.calib_yas532.a4 = (s32) ((v_data_u8[4] & 0x3F) - 32);
	p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* Read the D5 and D6 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB3);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[5], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* calculate a5*/
	yas532_data.calib_yas532.a5 =
        (s32) (((v_data_u8[5] >> BMI160_SHIFT_BIT_POSITION_BY_02_BITS) &
                0x3F) + 38);
	
    /* Read the D6 and D7 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB4);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[6], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* calculate a6*/
	yas532_data.calib_yas532.a6 =
        (s32) ((((v_data_u8[5] << BMI160_SHIFT_BIT_POSITION_BY_04_BITS) &
                 0x30) |
                ((v_data_u8[6] >> BMI160_SHIFT_BIT_POSITION_BY_04_BITS) &
                 0x0F)) - 32);
	 
     /* Read the D7 and D8 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB5);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[7], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* calculate a7*/
	yas532_data.calib_yas532.a7 =
        (s32) ((((v_data_u8[6] << BMI160_SHIFT_BIT_POSITION_BY_03_BITS) &
                 0x78) |
                ((v_data_u8[7] >> BMI160_SHIFT_BIT_POSITION_BY_05_BITS) &
                 0x07)) - 64);
	
    /* Read the D8 and D9 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB6);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[8], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* calculate a8*/
	yas532_data.calib_yas532.a8 = 
        (s32) ((((v_data_u8[7] << BMI160_GEN_READ_WRITE_DATA_LENGTH)
                 & 0x3E) |
                ((v_data_u8[8] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS)
                 & 0x01)) - 32);

	/* Read the D8 and D9 value */
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB7);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[9], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* calculate a9*/
	yas532_data.calib_yas532.a9 =
        (s32) (((v_data_u8[8] << BMI160_GEN_READ_WRITE_DATA_LENGTH) & 0xFE) |
               ((v_data_u8[9] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS) & 0x01));
	
    /* calculate k*/
	yas532_data.calib_yas532.k =
        (s32) ((v_data_u8[9] >> BMI160_SHIFT_BIT_POSITION_BY_02_BITS) & 0x1F);
	
    /* Read the  value from register 0x9A*/
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB8);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[10],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* Read the  value from register 0x9B*/
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB9);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[11],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* Read the  value from register 0x9C*/
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB10);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[12],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* Read the  value from register 0x9D*/
	com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_CALIB11);
	
    /* 0x04 is secondary read Mag x LSB register */
	com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8[13],
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
	
    /* Calculate the fxy1y2 and rxy1y1*/
	yas532_data.calib_yas532.fxy1y2[0] =
        (u8) (((v_data_u8[10] & 0x01) << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) |
              ((v_data_u8[11] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS) & 0x01));
	yas532_data.calib_yas532.rxy1y2[0] =
        ((s8) (((v_data_u8[10] >> BMI160_SHIFT_BIT_POSITION_BY_01_BIT)
                & 0x3F) << BMI160_SHIFT_BIT_POSITION_BY_02_BITS)) >>
        BMI160_SHIFT_BIT_POSITION_BY_02_BITS;
	yas532_data.calib_yas532.fxy1y2[1] =
        (u8) (((v_data_u8[11] & 0x01) << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) |
              ((v_data_u8[12] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS) & 0x01));
	yas532_data.calib_yas532.rxy1y2[1] =
        ((s8) (((v_data_u8[11] >> BMI160_SHIFT_BIT_POSITION_BY_01_BIT) &
                0x3F) << BMI160_SHIFT_BIT_POSITION_BY_02_BITS)) >>
        BMI160_SHIFT_BIT_POSITION_BY_02_BITS;
	yas532_data.calib_yas532.fxy1y2[2] =
        (u8) (((v_data_u8[12] & 0x01) << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) |
              ((v_data_u8[13] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS) & 0x01));
	yas532_data.calib_yas532.rxy1y2[2] =
        ((s8) (((v_data_u8[12] >> BMI160_SHIFT_BIT_POSITION_BY_01_BIT) &
                0x3F) << BMI160_SHIFT_BIT_POSITION_BY_02_BITS)) >>
        BMI160_SHIFT_BIT_POSITION_BY_02_BITS;

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_xy1y2_to_linear(
    u16 *v_xy1y2_u16, s32 *xy1y2_linear)
{
	static const u16 v_calib_data[] = {
        3721, 3971, 4221, 4471
    };

	for (u8 i = 0; i < 3; i++) {
		xy1y2_linear[i] = v_xy1y2_u16[i] -
                          v_calib_data[yas532_data.calib_yas532.fxy1y2[i]] +
                          (yas532_data.v_hard_offset_s8[i] -
                           yas532_data.calib_yas532.rxy1y2[i]) *
                          yas532_data.coef[i];
    }
	return SUCCESS;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_normal_measurement_data(
    u8 v_acquisition_command_u8, u8 *v_busy_u8, u16 *v_temp_u16,
    u16 *v_xy1y2_u16, u8 *v_overflow_u8)
{
	/* Array holding the YAS532 xyy1 data*/
	u8 v_data_u8[BMI160_YAS_XY1Y2T_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };

	/* check the p_bmi160 structure for NULL pointer assignment*/
	if (p_bmi160 == BMI160_NULL) {
		return E_BMI160_NULL_PTR;
    }
    
    /* read the sensor data */
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_bst_yas532_acquisition_command_register(
            v_acquisition_command_u8);
    com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_MAG_X_LSB__REG,
                    v_data_u8, 
                    BMI160_MAG_YAS_DATA_LENGTH);
    
    /* read the xyy1 data*/
    *v_busy_u8 = (v_data_u8[0] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS) & 0x01;
    *v_temp_u16 =
        (u16) ((((s32) v_data_u8[0] << BMI160_SHIFT_BIT_POSITION_BY_03_BITS) &
                0x3F8) |
               ((v_data_u8[1] >> BMI160_SHIFT_BIT_POSITION_BY_05_BITS) & 0x07));
    v_xy1y2_u16[0] =
        (u16) ((((s32) v_data_u8[2] << BMI160_SHIFT_BIT_POSITION_BY_06_BITS) &
                0x1FC0) |
               ((v_data_u8[3] >> BMI160_SHIFT_BIT_POSITION_BY_02_BITS) & 0x3F));
    v_xy1y2_u16[1] =
        (u16) ((((s32) v_data_u8[4] << BMI160_SHIFT_BIT_POSITION_BY_06_BITS) &
                0x1FC0) |
               ((v_data_u8[5] >> BMI160_SHIFT_BIT_POSITION_BY_02_BITS) & 0x3F));
    v_xy1y2_u16[2] =
        (u16) ((((s32) v_data_u8[6] << BMI160_SHIFT_BIT_POSITION_BY_06_BITS) &
                0x1FC0) |
               ((v_data_u8[7] >> BMI160_SHIFT_BIT_POSITION_BY_02_BITS) & 0x3F));
    
    *v_overflow_u8 = 0;
    for (u8 i = 0; i < 3; i++) {
        if (v_xy1y2_u16[i] == YAS532_DATA_OVERFLOW) {
            *v_overflow_u8 |= (1 << (i * 2));
        }
        if (v_xy1y2_u16[i] == YAS532_DATA_UNDERFLOW) {
            *v_overflow_u8 |= (1 << (i * 2 + 1));
        }
    }

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_measurement_xyz_data(
    struct yas532_vector *xyz_data, u8 *v_overflow_s8, u8 v_temp_correction_u8,
    u8 v_acquisition_command_u8)
{
	/* This variable is used to provide the communication results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;
	
    /* Array holding the linear calculation output*/
	s32 v_xy1y2_linear_s32[BMI160_YAS_XY1Y2_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* Array holding the temperature data */
	s32 v_xyz_tmp_s32[BMI160_YAS_TEMP_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    s32 tmp = BMI160_INIT_VALUE;
	s32 sx, sy1, sy2, sy, sz = BMI160_INIT_VALUE;
	u8 i, v_busy_u8 = BMI160_INIT_VALUE;
	u16 v_temp_u16 = BMI160_INIT_VALUE;
	
    /* Array holding the xyy1 sensor raw data*/
	u16 v_xy1y2_u16[BMI160_YAS_XY1Y2_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
    
        #if 1 < YAS532_MAG_TEMPERATURE_LOG
	s32 sum = BMI160_INIT_VALUE;
        #endif //1 < YAS532_MAG_TEMPERATURE_LOG
	
    *v_overflow_s8 = BMI160_INIT_VALUE;
    
	switch (yas532_data.measure_state) {	
    case YAS532_MAG_STATE_INIT_COIL:
		if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
			com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
        }
		
        /* write Rcoil*/
		com_rslt += bmi160_set_mag_write_data(BMI160_YAS_DISABLE_RCOIL);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_RCOIL);
		p_bmi160->delay_msec(BMI160_YAS532_MEASUREMENT_DELAY);
		
        if (!yas532_data.overflow &&
            is_valid_offset(yas532_data.v_hard_offset_s8)) {
			yas532_data.measure_state = 0;
        }
        break;	
    case YAS532_MAG_STATE_MEASURE_OFFSET:
		com_rslt = bmi160_bst_yas532_magnetic_measure_set_offset();
		yas532_data.measure_state = 0;
        break;    
	default:
        break;
	}
	
    /* Read sensor data*/
	com_rslt += bmi160_bst_yas532_normal_measurement_data(
                    v_acquisition_command_u8, 
                    &v_busy_u8, 
                    &v_temp_u16,
                    v_xy1y2_u16, 
                    v_overflow_s8);
	
    /* Calculate the linear data*/
	com_rslt += 
        bmi160_bst_yas532_xy1y2_to_linear(v_xy1y2_u16, v_xy1y2_linear_s32);
	
    /* Calculate temperature correction */
        #if 1 < YAS532_MAG_TEMPERATURE_LOG
    yas532_data.temp_data.log[yas532_data.temp_data.idx++] = v_temp_u16;
	if (YAS532_MAG_TEMPERATURE_LOG <= yas532_data.temp_data.idx) {
		yas532_data.temp_data.idx = 0;
    }
    yas532_data.temp_data.num++;
	if (YAS532_MAG_TEMPERATURE_LOG <= yas532_data.temp_data.num) {
		yas532_data.temp_data.num = YAS532_MAG_TEMPERATURE_LOG;
    }
	for (i = 0; i < yas532_data.temp_data.num; i++) {
		sum += yas532_data.temp_data.log[i];
    }
    tmp = sum * 10 / yas532_data.temp_data.num -
          YAS532_TEMP20DEGREE_TYPICAL * 10;
        #else //1 >= YAS532_MAG_TEMPERATURE_LOG
    tmp = (v_temp_u16 - YAS532_TEMP20DEGREE_TYPICAL) * 10;
        #endif //1 >= YAS532_MAG_TEMPERATURE_LOG
	
    sx  = v_xy1y2_linear_s32[0];
	sy1 = v_xy1y2_linear_s32[1];
	sy2 = v_xy1y2_linear_s32[2];
	
    /* Temperature correction */
	if (v_temp_correction_u8) {
		sx  -= yas532_data.calib_yas532.cx  * tmp / 1000;
		sy1 -= yas532_data.calib_yas532.cy1 * tmp / 1000;
		sy2 -= yas532_data.calib_yas532.cy2 * tmp / 1000;
	}
	sy = sy1 - sy2;
	sz = -sy1 - sy2;
    
        #if 1 < YAS532_MAG_TEMPERATURE_LOG
	xyz_data->yas532_vector_xyz[0] = yas532_data.calib_yas532.k * (
                                        100                         * sx + 
                                        yas532_data.calib_yas532.a2 * sy +
                                        yas532_data.calib_yas532.a3 * sz
                                     ) / 10;
	xyz_data->yas532_vector_xyz[1] = yas532_data.calib_yas532.k * (
                                        yas532_data.calib_yas532.a4 * sx +
                                        yas532_data.calib_yas532.a5 * sy +
                                        yas532_data.calib_yas532.a6 * sz
                                     ) / 10;
	xyz_data->yas532_vector_xyz[2] = yas532_data.calib_yas532.k * (
                                        yas532_data.calib_yas532.a7 * sx +
                                        yas532_data.calib_yas532.a8 * sy +
                                        yas532_data.calib_yas532.a9 * sz
                                     ) / 10;
	if (yas532_data.transform != BMI160_NULL) {
		for (u8 i = 0; i < 3; i++) {
            v_xyz_tmp_s32[i] =
                yas532_data.transform[i*3  ] * xyz_data->yas532_vector_xyz[0] +
                yas532_data.transform[i*3+1] * xyz_data->yas532_vector_xyz[1] +
                yas532_data.transform[i*3+2] * xyz_data->yas532_vector_xyz[2];
		}
		set_vector(xyz_data->yas532_vector_xyz, v_xyz_tmp_s32);
	}
	for (i = 0; i < 3; i++) {
		xyz_data->yas532_vector_xyz[i] -= xyz_data->yas532_vector_xyz[i] % 10;
		if (*v_overflow_s8 & (1 << (i * 2))) {
			xyz_data->yas532_vector_xyz[i] += 1; /* set overflow */
        }
		if (*v_overflow_s8 & (1 << (i * 2 + 1))) {
			xyz_data->yas532_vector_xyz[i] += 2; /* set underflow */
        }
	}
        #else //1 >= YAS532_MAG_TEMPERATURE_LOG
	xyz_data->yas532_vector_xyz[0] = sx;
	xyz_data->yas532_vector_xyz[1] = sy;
	xyz_data->yas532_vector_xyz[2] = sz;
        #endif //1 >= YAS532_MAG_TEMPERATURE_LOG
    
    if (v_busy_u8) {
		return com_rslt;
    }
    
	if (0 < *v_overflow_s8) {
		if (!yas532_data.overflow) {
			yas532_data.overflow = 1;
        }
		yas532_data.measure_state = YAS532_MAG_STATE_INIT_COIL;
	} else {
		yas532_data.overflow = 0;
    }
	
    for (i = 0; i < 3; i++) {
		yas532_data.last_raw[i] = v_xy1y2_u16[i];
    }
    yas532_data.last_raw[i] = v_temp_u16;
	
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_fifo_xyz_data(
    u16 *v_xy1y2_u16, u8 v_temp_correction_u8, s8 v_overflow_s8, u16 v_temp_u16,
    u8 v_busy_u8)
{
    /* Array holding the linear calculation output*/
	s32 v_xy1y2_linear_s32[BMI160_YAS_XY1Y2_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* Array holding the temperature data */
	s32 v_xyz_tmp_s32[BMI160_YAS_TEMP_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    s32 tmp = BMI160_INIT_VALUE;
	s32 sx, sy1, sy2, sy, sz = BMI160_INIT_VALUE;
	u8 i = BMI160_INIT_VALUE;
    
        #if 1 < YAS532_MAG_TEMPERATURE_LOG
	s32 sum = BMI160_INIT_VALUE;
        #endif //1 < YAS532_MAG_TEMPERATURE_LOG
	
    v_overflow_s8 = BMI160_INIT_VALUE;
	
    /* Calculate the linear data*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_bst_yas532_xy1y2_to_linear(v_xy1y2_u16, v_xy1y2_linear_s32);
	
    /* Calculate temperature correction */
        #if 1 < YAS532_MAG_TEMPERATURE_LOG
    yas532_data.temp_data.log[yas532_data.temp_data.idx++] = v_temp_u16;
	if (YAS532_MAG_TEMPERATURE_LOG <= yas532_data.temp_data.idx) {
		yas532_data.temp_data.idx = 0;
    }
    yas532_data.temp_data.num++;
	if (YAS532_MAG_TEMPERATURE_LOG <= yas532_data.temp_data.num) {
		yas532_data.temp_data.num = YAS532_MAG_TEMPERATURE_LOG;
    }
	for (i = 0; i < yas532_data.temp_data.num; i++) {
		sum += yas532_data.temp_data.log[i];
    }
    tmp = sum * 10 / yas532_data.temp_data.num -
          YAS532_TEMP20DEGREE_TYPICAL * 10;
        #else //1 >= YAS532_MAG_TEMPERATURE_LOG
    tmp = (v_temp_u16 - YAS532_TEMP20DEGREE_TYPICAL) * 10;
        #endif //1 >= YAS532_MAG_TEMPERATURE_LOG
	
    sx  = v_xy1y2_linear_s32[0];
	sy1 = v_xy1y2_linear_s32[1];
	sy2 = v_xy1y2_linear_s32[2];
	
    /* Temperature correction */
	if (v_temp_correction_u8) {
		sx  -= yas532_data.calib_yas532.cx  * tmp / 1000;
		sy1 -= yas532_data.calib_yas532.cy1 * tmp / 1000;
		sy2 -= yas532_data.calib_yas532.cy2 * tmp / 1000;
	}
	sy = sy1 - sy2;
	sz = -sy1 - sy2;
    
        #if 1 < YAS532_MAG_TEMPERATURE_LOG
	fifo_xyz_data.yas532_vector_xyz[0] = yas532_data.calib_yas532.k * (
                                            100                         * sx +
                                            yas532_data.calib_yas532.a2 * sy +
                                            yas532_data.calib_yas532.a3 * sz
                                         ) / 10;
	fifo_xyz_data.yas532_vector_xyz[1] = yas532_data.calib_yas532.k * (
                                            yas532_data.calib_yas532.a4 * sx +
                                            yas532_data.calib_yas532.a5 * sy +
                                            yas532_data.calib_yas532.a6 * sz
                                         ) / 10;
	fifo_xyz_data.yas532_vector_xyz[2] = yas532_data.calib_yas532.k * (
                                            yas532_data.calib_yas532.a7 * sx +
                                            yas532_data.calib_yas532.a8 * sy +
                                            yas532_data.calib_yas532.a9 * sz
                                         ) / 10;
	if (yas532_data.transform != BMI160_NULL) {
		for (i = 0; i < 3; i++) {
            v_xyz_tmp_s32[i] = yas532_data.transform[i*3    ] * 
                               fifo_xyz_data.yas532_vector_xyz[0]
                             + yas532_data.transform[i*3 + 1] * 
                               fifo_xyz_data.yas532_vector_xyz[1]
                             + yas532_data.transform[i*3 + 2] * 
                               fifo_xyz_data.yas532_vector_xyz[2];
		}
		set_vector(fifo_xyz_data.yas532_vector_xyz, v_xyz_tmp_s32);
	}
	for (i = 0; i < 3; i++) {
		fifo_xyz_data.yas532_vector_xyz[i] -= fifo_xyz_data.yas532_vector_xyz[i]
                                            % 10;
		if (v_overflow_s8 & (1 << (i * 2))) {
			fifo_xyz_data.yas532_vector_xyz[i] += 1; /* set overflow */
        }
		if (v_overflow_s8 & (1 << (i * 2 + 1))) {
			fifo_xyz_data.yas532_vector_xyz[i] += 2;
        }
	}
        #else //1 >= YAS532_MAG_TEMPERATURE_LOG
	fifo_xyz_data.yas532_vector_xyz[0] = sx;
	fifo_xyz_data.yas532_vector_xyz[1] = sy;
	fifo_xyz_data.yas532_vector_xyz[2] = sz;
        #endif //1 >= YAS532_MAG_TEMPERATURE_LOG
    
    if (v_busy_u8) {
		return com_rslt;
    }
    
	if (0 < v_overflow_s8) {
		if (!yas532_data.overflow) {
			yas532_data.overflow = 1;
        }
		yas532_data.measure_state = YAS532_MAG_STATE_INIT_COIL;
	} else {
		yas532_data.overflow = 0;
    }
	for (i = 0; i < 3; i++) {
		yas532_data.last_raw[i] = v_xy1y2_u16[i];
    }
    yas532_data.last_raw[i] = v_temp_u16;
	
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_acquisition_command_register(
    u8 v_command_reg_data_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

	if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
        com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
    }

    com_rslt = bmi160_set_mag_write_data(v_command_reg_data_u8);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* YAMAHA YAS532-0x82*/
    com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_COMMAND_REGISTER);
    p_bmi160->delay_msec(BMI160_YAS_ACQ_COMMAND_DELAY);
    
    com_rslt += bmi160_set_mag_read_addr(BMI160_YAS532_DATA_REGISTER);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

	if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
		com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
    }
	
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas532_set_offset(
    const s8 *p_offset_s8)
{
	/* This variable is used to provide the communication results*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

	if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
		com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
    }
    p_bmi160->delay_msec(BMI160_YAS532_OFFSET_DELAY);

    /* Write offset X data*/
    com_rslt = bmi160_set_mag_write_data(p_offset_s8[0]);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* YAS532 offset x write*/
    com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_OFFSET_X);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

    /* Write offset Y data*/
    com_rslt = bmi160_set_mag_write_data(p_offset_s8[1]);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* YAS532 offset y write*/
    com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_OFFSET_Y);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

    /* Write offset Z data*/
    com_rslt = bmi160_set_mag_write_data(p_offset_s8[2]);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* YAS532 offset z write*/
    com_rslt += bmi160_set_mag_write_addr(BMI160_YAS532_OFFSET_Z);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    set_vector(yas532_data.v_hard_offset_s8, p_offset_s8);

	if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
		com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
    }
    
	return com_rslt;
}
    #endif //YAS532
    
    #ifdef YAS537
BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yamaha_yas537_mag_interface_init(void)
{
    u8 v_data_u8 = BMI160_INIT_VALUE;
    u8 i = BMI160_INIT_VALUE;
    u8 v_accel_power_mode_status = BMI160_INIT_VALUE;

    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_get_accel_power_mode_stat(&v_accel_power_mode_status);
    
    /* Accel operation mode to normal*/
    if (v_accel_power_mode_status != BMI160_ACCEL_NORMAL_MODE) {
        com_rslt += bmi160_set_command_register(ACCEL_MODE_NORMAL);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    }
    
    /* write Mag power mode as NORMAL*/
    com_rslt += bmi160_set_mag_interface_normal();
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* Write the YAS532 i2c address*/
    com_rslt += bmi160_set_i2c_device_addr(BMI160_YAS537_I2C_ADDRESS);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* enable the Mag interface to manual mode*/
    com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    bmi160_get_mag_manual_enable(&v_data_u8);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /*Enable the MAG interface */
    com_rslt += bmi160_set_if_mode(BMI160_ENABLE_MAG_IF_MODE);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    bmi160_get_if_mode(&v_data_u8);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    v_data_u8 = BMI160_MANUAL_DISABLE;
    
    /* Read the YAS537 device id 0x07*/
    com_rslt += bmi160_set_mag_read_addr(BMI160_YAS_DEVICE_ID_REG);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &v_data_u8, 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    yas537_data.dev_id = v_data_u8;
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* Read the YAS537 calibration data*/
    com_rslt += bmi160_bst_yamaha_yas537_calib_values(
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
    /* set the mode to NORMAL*/
    yas537_data.measure_state = YAS537_MAG_STATE_NORMAL;
    
    /* set the transform to zero */
    yas537_data.transform = BMI160_NULL;
    yas537_data.average = 32;
    for (i = 0; i < 3; i++) {
        yas537_data.hard_offset[i] = -128;
        yas537_data.last_after_rcoil[i] = 0;
    }
    for (i = 0; i < 4; i++) {
        yas537_data.last_raw[i] = 0;
    }
    
    /* write the Mag bandwidth as 25Hz*/
    com_rslt += 
        bmi160_set_mag_output_data_rate(BMI160_MAG_OUTPUT_DATA_RATE_25HZ);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* Enable Mag interface to auto mode*/
    com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    bmi160_get_mag_manual_enable(&v_data_u8);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

    switch (v_accel_power_mode_status) {
    case BMI160_ACCEL_SUSPEND:
        com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
    case BMI160_ACCEL_LOW_POWER:
        com_rslt += bmi160_set_command_register(ACCEL_LOWPOWER);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        break;
    default:
        break;
    }
    
    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yamaha_yas537_calib_values(
    u8 v_rcoil_u8)
{
    /* Array holding the YAS532 calibration values */
    u8 a_data_u8[BMI160_YAS537_CALIB_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE,
    };
    
    static const u8 v_avrr_u8[] = {
        0x50, 0x60, 0x70
    };    
    u8 v_cal_valid_u8 = BMI160_INIT_VALUE, i;
    
    /* write soft reset as 0x02*/
    BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_set_mag_write_data(YAS537_SRSTR_DATA);
    p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
    com_rslt += bmi160_set_mag_write_addr(YAS537_REG_SRSTR);
    p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
    /* Read the DX value */
    com_rslt = bmi160_set_mag_read_addr(YAS537_REG_CALR_C0);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[0], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the DY1 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C1);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[1], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the DY2 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C2);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[2], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D2 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C3);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[3], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D3 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C4);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[4], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D4 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C5);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[5], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D5 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C6);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[6], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D6 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C7);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[7], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D7 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C8);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[8], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D8 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_C9);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[9], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the D9 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_CA);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[10], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the RX value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_CB);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[11], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the RY1 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_CC);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[12], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the RY2 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_CD);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[13], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the RY2 value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_CE);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[14], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the CHF value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_CF);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[15], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* Read the VER value */
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_CALR_DO);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* 0x04 is secondary read Mag x LSB register */
    com_rslt += bmi160_read_reg(
                    BMI160_MAG_DATA_READ_REG,
                    &a_data_u8[16], 
                    BMI160_GEN_READ_WRITE_DATA_LENGTH);
    
    /* get the calib ver*/
    yas537_data.calib_yas537.ver =
        a_data_u8[16] >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS;
    for (i = 0; i < 17; i++) {
        if (((i < 16 && a_data_u8[i]) != 0)) {
            v_cal_valid_u8 = 1;
        }
        if ((i < 16 && (a_data_u8[i] & 0x3F)) != 0) {
            v_cal_valid_u8 = 1;
        }
    }
    if (!v_cal_valid_u8) {
        return ERROR;
    }
    
    if (yas537_data.calib_yas537.ver == 0) {
        for (i = 0; i < 17; i++) {
            if (i < 12) {
                /* write offset*/
                com_rslt += bmi160_set_mag_write_data(a_data_u8[i]);
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
                com_rslt += bmi160_set_mag_write_addr(YAS537_REG_MTCR + i);
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
            } else if (i < 15) {
                /* write offset correction*/
                com_rslt += bmi160_set_mag_write_data(a_data_u8[i]);
                p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
                com_rslt += bmi160_set_mag_write_addr(YAS537_REG_OXR + i - 12);
                p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
                yas537_data.hard_offset[i - 12] = a_data_u8[i];
            } else {
                /* write offset correction*/
                com_rslt += bmi160_set_mag_write_data(a_data_u8[i]);
                p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
                com_rslt += bmi160_set_mag_write_addr(YAS537_REG_OXR + i - 11);
                p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
            }
        }
    } else if (yas537_data.calib_yas537.ver == 1) {
        for (i = 0; i < 3; i++) {
            /* write offset*/
            com_rslt += bmi160_set_mag_write_data(a_data_u8[i]);
            p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
            
            com_rslt += bmi160_set_mag_write_addr(YAS537_REG_MTCR + i);
            p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
            
            if (com_rslt == SUCCESS) {
                /* write offset*/
                com_rslt += bmi160_set_mag_write_data(a_data_u8[i + 12]);
                p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
                
                com_rslt += bmi160_set_mag_write_addr(YAS537_REG_OXR + i);
                p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
                
                yas537_data.hard_offset[i] = a_data_u8[i + 12];
            } else {
                com_rslt = ERROR;
            }
        }
        
        /* write offset*/
        com_rslt += bmi160_set_mag_write_data((a_data_u8[i] & 0xE0) | 0x10);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        com_rslt += bmi160_set_mag_write_addr(YAS537_REG_MTCR + i);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        /* write offset*/
        com_rslt +=
            bmi160_set_mag_write_data((a_data_u8[15] >>
                                       BMI160_SHIFT_BIT_POSITION_BY_03_BITS) &
                                      0x1E);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        com_rslt += bmi160_set_mag_write_addr(YAS537_REG_HCKR);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        /* write offset*/
        com_rslt += bmi160_set_mag_write_data((a_data_u8[15] << 1) & 0x1E);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        com_rslt += bmi160_set_mag_write_addr(YAS537_REG_LCKR);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        /* write offset*/
        com_rslt += bmi160_set_mag_write_data(a_data_u8[16] & 0x3F);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        com_rslt += bmi160_set_mag_write_addr(YAS537_REG_OCR);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);

        /* Assign the calibration values*/
        /* a2 */
        yas537_data.calib_yas537.a2 = (
            ((a_data_u8[3] << BMI160_SHIFT_BIT_POSITION_BY_02_BITS) & 0x7C) |
            (a_data_u8[4] >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS)
        ) - 64;
        
        /* a3 */
        yas537_data.calib_yas537.a3 = (
            ((a_data_u8[4] << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) & 0x7E) |
            (a_data_u8[5] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS)
        ) - 64;
        
        /* a4 */
        yas537_data.calib_yas537.a4 = (
            ((a_data_u8[5] << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) & 0xFE) |
            (a_data_u8[6] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS)
        ) - 128;
        
        /* a5 */
        yas537_data.calib_yas537.a5 = (
            ((a_data_u8[6] << BMI160_SHIFT_BIT_POSITION_BY_02_BITS) & 0x1FC) |
            (a_data_u8[7] >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS)
        ) - 112;
        
        /* a6 */
        yas537_data.calib_yas537.a6 = (
            ((a_data_u8[7] << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) & 0x7E) |
            (a_data_u8[8] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS)
        ) - 64;
        
        /* a7 */
        yas537_data.calib_yas537.a7 = (
            ((a_data_u8[8] << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) & 0xFE) |
            (a_data_u8[9] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS)
        ) - 128;
        
        /* a8 */
        yas537_data.calib_yas537.a8 = (a_data_u8[9] & 0x7F) - 64;
        
        /* a9 */
        yas537_data.calib_yas537.a9 = (
            ((a_data_u8[10] << BMI160_SHIFT_BIT_POSITION_BY_01_BIT) & 0x1FE) |
            (a_data_u8[11] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS)
        ) - 112;
        
        /* k */
        yas537_data.calib_yas537.k = a_data_u8[11] & 0x7F;
	} else {
		return ERROR;
	}
    
    /* write A/D converter*/
    com_rslt += bmi160_set_mag_write_data(YAS537_WRITE_A_D_CONVERTER);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    com_rslt += bmi160_set_mag_write_addr(YAS537_REG_ADCCALR);
    p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
    /* write A/D converter second register*/
    com_rslt += bmi160_set_mag_write_data(YAS537_WRITE_A_D_CONVERTER2);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    com_rslt += bmi160_set_mag_write_addr(YAS537_REG_ADCCALR_ONE);
    p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
    /* write temperature calibration register*/
    com_rslt += bmi160_set_mag_write_data(YAS537_WRITE_TEMP_CALIB);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    com_rslt += bmi160_set_mag_write_addr(YAS537_REG_TRMR);
    p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
    /* write average filter register*/
    com_rslt += bmi160_set_mag_write_data(v_avrr_u8[yas537_data.average]);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    com_rslt += bmi160_set_mag_write_addr(YAS537_REG_AVRR);
    p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    
    if (v_rcoil_u8) {
        /* write average; filter register*/
        com_rslt += bmi160_set_mag_write_data(YAS537_WRITE_FILTER);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        
        com_rslt += bmi160_set_mag_write_addr(YAS537_REG_CONFR);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
    }

    return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yas537_acquisition_command_register(
    u8 v_command_reg_data_u8)
{
	BMI160_RETURN_FUNCTION_TYPE com_rslt = E_BMI160_COMM_RES;

	if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
        com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
    }
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

    com_rslt = bmi160_set_mag_write_data(v_command_reg_data_u8);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
    
    /* YAMAHA YAS532-0x82*/
    com_rslt += bmi160_set_mag_write_addr(BMI160_REG_YAS537_CMDR);
    
    /* set the mode to RECORD*/
    yas537_data.measure_state = YAS537_MAG_STATE_RECORD_DATA;
    p_bmi160->delay_msec(BMI160_YAS_ACQ_COMMAND_DELAY);
    
    com_rslt += bmi160_set_mag_read_addr(YAS537_REG_TEMPERATURE_0);
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

	if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
		com_rslt += bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
    }
    p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);

	return com_rslt;
}

/*!
 *	@brief This function is used for processing the
 *	YAMAHA YAS537 xy1y2 raw data
 *
 *	@param xy1y2: The value of raw xy1y2 data
 *	@param xyz: The value of  xyz data
 *
 *	@return None
 */
static void xy1y2_to_xyz(u16 *xy1y2, s32 *xyz)
{
	xyz[0] = (xy1y2[0] - 8192) * 300;
	xyz[1] = (xy1y2[1] - xy1y2[2]) * 1732 / 10;
	xyz[2] = (-xy1y2[2] - xy1y2[2] + 16384) * 300;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yamaha_yas537_read_xy1y2_data(
    u8 *v_coil_stat_u8, u8 *v_busy_u8, u16 *v_temperature_u16, u16 *xy1y2,
    u8 *v_outflow_u8)
{
    /* Array holding the YAS532 calibration values */
	u8 a_data_u8[BMI160_YAS_XY1Y2T_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE,
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE,
    };
	
    u8 i = BMI160_INIT_VALUE;
	s32 a_h_s32[BMI160_YAS_H_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	s32 a_s_s32[BMI160_YAS_S_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	
    /* set command register*/
	BMI160_RETURN_FUNCTION_TYPE com_rslt = 
        bmi160_bst_yas537_acquisition_command_register(
            YAS537_SET_COMMAND_REGISTER);

	/* read the yas537 sensor data of xy1y2*/
	com_rslt += p_bmi160->BMI160_BUS_READ_FUNC(
                    p_bmi160->dev_addr,
                    BMI160_USER_DATA_MAG_X_LSB__REG,
                    a_data_u8, 
                    BMI160_MAG_YAS_DATA_LENGTH);
	
    /* read the busy flag*/
	*v_busy_u8 = a_data_u8[2] >> BMI160_SHIFT_BIT_POSITION_BY_07_BITS;
	
    /* read the coil status*/
	*v_coil_stat_u8 = (a_data_u8[2] >> BMI160_SHIFT_BIT_POSITION_BY_06_BITS) &
                      0X01;
	
    /* read temperature data*/
	*v_temperature_u16 =
        (u16) ((a_data_u8[0] << BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               a_data_u8[1]);
	
    /* read x data*/
	xy1y2[0] =
        (u16) (((a_data_u8[2] & 0x3F) << BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               a_data_u8[3]);
	
    /* read y1 data*/
	xy1y2[1] =
        (u16) ((a_data_u8[4]          << BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               a_data_u8[5]);
	
    /* read y2 data*/
	xy1y2[2] =
        (u16) ((a_data_u8[6]          << BMI160_SHIFT_BIT_POSITION_BY_08_BITS) |
               a_data_u8[7]);
	
    for (i = 0; i < 3; i++) {
		yas537_data.last_raw[i] = xy1y2[i];
    }
	yas537_data.last_raw[i] = *v_temperature_u16;
	
    if (yas537_data.calib_yas537.ver == 1) {
		for (i = 0; i < 3; i++) {
			a_s_s32[i] = xy1y2[i] - 8192;
        }
        
		/* read hx*/
		a_h_s32[0] = yas537_data.calib_yas537.k * (
            128                         * a_s_s32[0] +
            yas537_data.calib_yas537.a2 * a_s_s32[1] +
            yas537_data.calib_yas537.a3 * a_s_s32[2]
		) / 8192;
		
        /* read hy1*/
		a_h_s32[1] = yas537_data.calib_yas537.k * (
            yas537_data.calib_yas537.a4 * a_s_s32[0] +
            yas537_data.calib_yas537.a5 * a_s_s32[1] +
            yas537_data.calib_yas537.a6 * a_s_s32[2]
		) / 8192;
		
        /* read hy2*/
		a_h_s32[2] = yas537_data.calib_yas537.k * (
            yas537_data.calib_yas537.a7 * a_s_s32[0] +
            yas537_data.calib_yas537.a8 * a_s_s32[1] +
            yas537_data.calib_yas537.a9 * a_s_s32[2]
        ) / 8192;

		for (i = 0; i < 3; i++) {
			if (a_h_s32[i] < -8192) {
				a_h_s32[i] = -8192;
            }
			if (8192 < a_h_s32[i]) {
				a_h_s32[i] = 8192;
            }
			xy1y2[i] = a_h_s32[i] + 8192;
		}
	}
    
	*v_outflow_u8 = 0;
	for (i = 0; i < 3; i++) {
		if (YAS537_DATA_OVERFLOW <= xy1y2[i]) {
			*v_outflow_u8 |= (1 << (i * 2));
        }
		if (xy1y2[i] == YAS537_DATA_UNDERFLOW) {
			*v_outflow_u8 |= (1 << (i * 2 + 1));
        }
	}

	return com_rslt;
}

/*!
 *	@brief This function is used for detecting whether the mag
 *  data obtained is valid or not
 *
 *	@param v_cur_u16: The value of current Mag data
 *  @param v_last_u16: The value of last Mag data
 *
 *	@return results of magnetic field data's validity
 *	@retval 0 -> VALID DATA
 *	@retval 1 -> INVALID DATA
 */
static BMI160_RETURN_FUNCTION_TYPE invalid_magnetic_field(
    u16 *v_cur_u16, u16 *v_last_u16)
{
	s16 invalid_thresh[] = {
        1500, 1500, 1500
    };

	for (u8 i = 0; i < 3; i++) {
		if (invalid_thresh[i] < ABS(v_cur_u16[i] - v_last_u16[i])) {
			return 1;
        }
    }
	return 0;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yamaha_yas537_measure_xyz_data(
    u8 *v_outflow_u8, struct yas_vector *vector_xyz)
{
	s32 a_xyz_tmp_s32[BMI160_YAS_TEMP_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	u8 v_busy_u8 = BMI160_INIT_VALUE;
	u8 v_rcoil_u8 = BMI160_INIT_VALUE;
	u16 v_temperature_u16 = BMI160_INIT_VALUE;
	u16 a_xy1y2_u16[BMI160_YAS_XY1Y2_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
	*v_outflow_u8 = 0;
	
    /* read the yas537 xy1y2 data*/
	s8 com_rslt = bmi160_bst_yamaha_yas537_read_xy1y2_data(
                    &v_rcoil_u8, &v_busy_u8,
                    &v_temperature_u16, 
                    a_xy1y2_u16, v_outflow_u8);
	
    /* linear calculation*/
	xy1y2_to_xyz(a_xy1y2_u16, vector_xyz->yas537_vector_xyz);
	if (yas537_data.transform != BMI160_NULL) {
		for (u8 i = 0; i < 3; i++) {
			a_xyz_tmp_s32[i] = 
                yas537_data.transform[i*3    ]
                * vector_xyz->yas537_vector_xyz[0] +
                yas537_data.transform[i*3 + 1]
                * vector_xyz->yas537_vector_xyz[1] +
                yas537_data.transform[i*3 + 2]
                * vector_xyz->yas537_vector_xyz[2];
		}
		yas537_set_vector(vector_xyz->yas537_vector_xyz, a_xyz_tmp_s32);
	}
	for (u8 i = 0; i < 3; i++) {
		vector_xyz->yas537_vector_xyz[i] -= vector_xyz->yas537_vector_xyz[i] %
                                            10;
		if (*v_outflow_u8 & (1 << (i * 2))) {
			vector_xyz->yas537_vector_xyz[i] += 1; /* set overflow */
        }
		if (*v_outflow_u8 & (1 << (i * 2 + 1))) {
			/* set underflow */
			vector_xyz->yas537_vector_xyz[i] += 2;
        }
	}
	if (v_busy_u8) {
		return ERROR;
    }
    
	switch (yas537_data.measure_state) {
	case YAS537_MAG_STATE_INIT_COIL:
		if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
			com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
        }
		
        com_rslt += bmi160_set_mag_write_data(YAS537_WRITE_CONFR);
		p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
		
        com_rslt += bmi160_set_mag_write_addr(YAS537_REG_CONFR);
		p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
		
        yas537_data.measure_state = YAS537_MAG_STATE_RECORD_DATA;
		if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
			com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
        }
        break;
	case YAS537_MAG_STATE_RECORD_DATA:
		if (v_rcoil_u8) {
			break;
        }
		yas537_set_vector(yas537_data.last_after_rcoil, a_xy1y2_u16);
		yas537_data.measure_state = YAS537_MAG_STATE_NORMAL;
        break;
	case YAS537_MAG_STATE_NORMAL:
		if (BMI160_INIT_VALUE < v_outflow_u8 ||
            invalid_magnetic_field(a_xy1y2_u16, yas537_data.last_after_rcoil)) {
			yas537_data.measure_state = YAS537_MAG_STATE_INIT_COIL;
			for (u8 i = 0; i < 3; i++) {
				if (!*v_outflow_u8) {
					vector_xyz->yas537_vector_xyz[i] += 3;
                }
			}
		}
        break;
	}

	return com_rslt;
}

BMI160_RETURN_FUNCTION_TYPE bmi160_bst_yamaha_yas537_fifo_xyz_data(
    u16 *a_xy1y2_u16, u8 v_over_flow_u8, u8 v_rcoil_u8, u8 v_busy_u8)
{
    s32 a_xyz_tmp_s32[BMI160_YAS_TEMP_DATA_SIZE] = {
        BMI160_INIT_VALUE, BMI160_INIT_VALUE, BMI160_INIT_VALUE
    };
    s8 com_rslt = BMI160_INIT_VALUE;

    /* linear calculation*/
    xy1y2_to_xyz(a_xy1y2_u16, fifo_vector_xyz.yas537_vector_xyz);
    if (yas537_data.transform != BMI160_NULL) {
        for (u8 i = 0; i < 3; i++) {
            a_xyz_tmp_s32[i] =
                yas537_data.transform[i*3    ]
                * fifo_vector_xyz.yas537_vector_xyz[0] +
                yas537_data.transform[i*3 + 1]
                * fifo_vector_xyz.yas537_vector_xyz[1] +
                yas537_data.transform[i*3 + 2]
                * fifo_vector_xyz.yas537_vector_xyz[2];
        }
        yas537_set_vector(
        fifo_vector_xyz.yas537_vector_xyz, a_xyz_tmp_s32);
    }
    for (u8 i = 0; i < 3; i++) {
        fifo_vector_xyz.yas537_vector_xyz[i] -= fifo_vector_xyz.yas537_vector_xyz[i] % 10;
        if (v_over_flow_u8 & (1 << (i * 2))) {
            fifo_vector_xyz.yas537_vector_xyz[i] += 1; /* set overflow */
        }
        if (v_over_flow_u8 & (1 << (i * 2 + 1))) {
            /* set underflow */
            fifo_vector_xyz.yas537_vector_xyz[i] += 2;
        }
    }
    if (v_busy_u8) {
        return ERROR;
    }
    
    switch (yas537_data.measure_state) {
    case YAS537_MAG_STATE_INIT_COIL:
        if (p_bmi160->mag_manual_enable != BMI160_MANUAL_ENABLE) {
            com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_ENABLE);
        }
        com_rslt += bmi160_set_mag_write_data(YAS537_WRITE_CONFR);
        p_bmi160->delay_msec(BMI160_GEN_READ_WRITE_DELAY);
        
        com_rslt += bmi160_set_mag_write_addr(YAS537_REG_CONFR);
        p_bmi160->delay_msec(BMI160_SEC_INTERFACE_GEN_READ_WRITE_DELAY);
        
        yas537_data.measure_state = YAS537_MAG_STATE_RECORD_DATA;
        if (p_bmi160->mag_manual_enable == BMI160_MANUAL_ENABLE) {
            com_rslt = bmi160_set_mag_manual_enable(BMI160_MANUAL_DISABLE);
        }
        break;
    case YAS537_MAG_STATE_RECORD_DATA:
        if (v_rcoil_u8) {
            break;
        }
        yas537_set_vector(yas537_data.last_after_rcoil, a_xy1y2_u16);
        yas537_data.measure_state = YAS537_MAG_STATE_NORMAL;
        break;
    case YAS537_MAG_STATE_NORMAL:
        if (BMI160_INIT_VALUE < v_over_flow_u8 ||
            invalid_magnetic_field(a_xy1y2_u16, yas537_data.last_after_rcoil)) {
            yas537_data.measure_state = YAS537_MAG_STATE_INIT_COIL;
            for (u8 i = 0; i < 3; i++) {
                if (!v_over_flow_u8) {
                    fifo_vector_xyz.yas537_vector_xyz[i] += 3;
                }
            }
        }
        break;
    }

    return com_rslt;
}
    #endif //YAS537

#endif //INCLUDE_BMI160MAG

/**
 * \}
 * \}
 * \}
 */
