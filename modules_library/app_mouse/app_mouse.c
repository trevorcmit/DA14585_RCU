/**
 ****************************************************************************************
 *
 * \file app_mouse.c
 *
 * \brief Mouse module source file
 *
 * Copyright (C) 2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information  
 * of Dialog Semiconductor. All Rights Reserved.
 *
 * <bluetooth.support@diasemi.com>
 *
 *****************************************************************************************
 */
 
/**
 ****************************************************************************************
 * \addtogroup APP_UTILS
 * \{
 * \addtogroup MOUSE
 * \{
 * \addtogroup APP_MOUSE
 * \{
 ****************************************************************************************
 */ 

#ifdef HAS_MOUSE

#include "app_mouse.h"

burst_data_t burst_read;
burst_data_t burst_read_accumulated __PORT_RETAINED;

/**
 ****************************************************************************************
 * \brief Adds two int16_t integers, only if the result fits in uint16_t
 *
 * \param[in]   a       operand a
 * \param[in]   b       operand b
 * \param[out]  sum     result
 *
 * \return  false, if the result overflows
 *          true, if the addition is successful
 ****************************************************************************************
 */
static bool safe_addition(int16_t a, int16_t b, int16_t *sum)
{
   int16_t temp_sum = a + b;
   if ( ((a>0) && (b>0) && (temp_sum<=0)) || ((a<0) && (b<0) && (temp_sum>=0)) ) {
        return false;
   }
   else {
        *sum = temp_sum;     
        return true;
   }
}

void app_mouse_poll_sensor(void)
{
    int16_t temp_x;
    int16_t temp_y;  
                              
    // read motion sensor
    mouse_sensor_read_motion_burst(&burst_read);  // NOTE: Can check for X,Y registers overflow (MOT. register - OVF flag)
        
    if (burst_read.overflow_x || burst_read.overflow_y) {
        //mouse sensor overflow detected
//            ASSERT_WARNING(0);          
    }        

    if ( (safe_addition (burst_read_accumulated.deltaX, burst_read.deltaX, &temp_x)) &&
         (safe_addition (burst_read_accumulated.deltaY, burst_read.deltaY, &temp_y)) ) {
        burst_read_accumulated.deltaX = temp_x;             
        burst_read_accumulated.deltaY = temp_y;                                     
    }
    else {
        //Motion data is ignored due to burst_read_accumulated overflow
//        ASSERT_WARNING(0);   
    }        

}

bool app_mouse_sensor_data_accumulated_over_quota(void)
{
    return ( (burst_read_accumulated.deltaX > MOUSE_MOTION_DATA_QUOTA) || (burst_read_accumulated.deltaX < -MOUSE_MOTION_DATA_QUOTA) ||
             (burst_read_accumulated.deltaY > MOUSE_MOTION_DATA_QUOTA) || (burst_read_accumulated.deltaY < -MOUSE_MOTION_DATA_QUOTA) );
}

#ifdef MOUSE_GENERATE_TEST_PATTERN
uint16_t special_test_circle_sample_index;

#define SAMPLE_DIVIDER 6
const uint8_t X[] = {4/SAMPLE_DIVIDER,11/SAMPLE_DIVIDER,19/SAMPLE_DIVIDER,26/SAMPLE_DIVIDER,33/SAMPLE_DIVIDER,40/SAMPLE_DIVIDER,47/SAMPLE_DIVIDER,53/SAMPLE_DIVIDER,59/SAMPLE_DIVIDER};
const uint8_t Y[] = {87/SAMPLE_DIVIDER,86/SAMPLE_DIVIDER,85/SAMPLE_DIVIDER,83/SAMPLE_DIVIDER,81/SAMPLE_DIVIDER,77/SAMPLE_DIVIDER,74/SAMPLE_DIVIDER,69/SAMPLE_DIVIDER,64/SAMPLE_DIVIDER};

void app_mouse_prepare_next_test_data_sample(void)
{
    static uint16_t k,l,m;
           
    int16_t delta_x;
    int16_t delta_y;
    
    k = special_test_circle_sample_index % 18;
    l = special_test_circle_sample_index / 18;
    m = k%9;
    
    if (k<9) {
        delta_x =  -X[m];
        delta_y =  Y[m];          
    }
    else {
        delta_x =  -Y[8-m];
        delta_y =  X[8-m];           
    }
           
    switch (l) {
    case 0:
        burst_read_accumulated.deltaX = delta_x;
        burst_read_accumulated.deltaY = delta_y;    
    break;
    
    case 1:
        burst_read_accumulated.deltaX = -delta_y;
        burst_read_accumulated.deltaY = delta_x;                
    break;

    case 2:
        burst_read_accumulated.deltaX = -delta_x;
        burst_read_accumulated.deltaY = -delta_y;                
    break;            

    case 3:
        burst_read_accumulated.deltaX = delta_y;
        burst_read_accumulated.deltaY = -delta_x;                  
    break;
    }

    special_test_circle_sample_index = ++special_test_circle_sample_index % 72;
}
#endif

bool app_mouse_has_valid_data(void)
{
    return (burst_read_accumulated.deltaX != 0 || burst_read_accumulated.deltaY != 0);
}

void app_mouse_get_data(burst_data_t *data)
{
    *data = burst_read_accumulated;
    burst_read_accumulated.deltaX = 0;
    burst_read_accumulated.deltaY = 0;
}

void app_mouse_reset_data(void)
{
        burst_read_accumulated.deltaX = 0;
        burst_read_accumulated.deltaY = 0;
}

bool app_mouse_is_active(void)
{
    return port_gpio_get_pin_status(MOUSE_MOTION_PORT, MOUSE_MOTION_PIN) == (MOUSE_MOTION_POLARITY == GPIO_ACTIVE_HIGH);
}

#endif // HAS_MOUSE

/**
 * \}
 * \}
 * \}
 */
