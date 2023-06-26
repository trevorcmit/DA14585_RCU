/*****************************************************************************************
 * \file app_kbd_scan_matrix.h
 * \brief HID Keyboard scan matrix.
*****************************************************************************************/
#ifndef _APP_KBD_SCAN_MATRIX_H_
#define _APP_KBD_SCAN_MATRIX_H_


/*****************************************************************************************
 * \addtogroup CONFIGURATION
 * \{
 * \addtogroup MODULE_CONFIG
 * \{
 * \addtogroup KEYBOARD_SCAN_MATRIX_CFG
 * \brief Keyboard module scan matrix configuration
 * \{
*****************************************************************************************/


/*****************************************************************************************
 * \brief Number of input pins used for key matrix columns
*****************************************************************************************/
#define KBD_NR_COLUMN_INPUTS     3


/*****************************************************************************************
 * \brief Number of output pins used for key matrix rows
*****************************************************************************************/
#define KBD_NR_ROW_OUTPUTS       4


/*****************************************************************************************
 * \brief Define the pins used for columns. Maximum number of columns supported is 16.
 *        If a column is not used set the corresponding port and pin to PORT_UNUSED
 *        and PIN_UNUSED respectively.
******************************************************************************************/
#define COLUMN_INPUT_0_PORT		 2	
#define COLUMN_INPUT_0_PIN		 8
                                 
#define COLUMN_INPUT_1_PORT		 0	
#define COLUMN_INPUT_1_PIN		 1
                                 
#define COLUMN_INPUT_2_PORT		 0	
#define COLUMN_INPUT_2_PIN		 2

/*****************************************************************************************
 * \brief Define the pins used for rows. Maximum number of rows supported is 16.
 *        If a row is not used set the corresponding port and pin to PORT_UNUSED
 *        and PIN_UNUSED respectively.
******************************************************************************************/
#define ROW_OUTPUT_0_PORT	     2	
#define ROW_OUTPUT_0_PIN	     1
                                 
#define ROW_OUTPUT_1_PORT	     2	
#define ROW_OUTPUT_1_PIN	     2
                                 
#define ROW_OUTPUT_2_PORT	     2	
#define ROW_OUTPUT_2_PIN	     3
                                
#define ROW_OUTPUT_3_PORT	     2	
#define ROW_OUTPUT_3_PIN	     4

#ifdef HAS_POWERUP_BUTTON
    #define POWER_BUTTON_COLUMN  3
    #define POWER_BUTTON_ROW     1
#endif

/*******
 * \}
 * \}
 * \}
*******/

#endif // _APP_KBD_SCAN_MATRIX_H_
