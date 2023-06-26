/*****************************************************************************************
 *
 * @file app_console.h
 *
 * @brief Application console related functions header file.
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
 * \addtogroup UTILS
 * \{
 * \addtogroup APP_CONSOLE
 * \{
 ****************************************************************************************	 
 */

#ifndef _APP_CONSOLE_H_
#define _APP_CONSOLE_H_

// printf() functionality
#if (HAS_PRINTF)

        #include <stdarg.h>

typedef struct __print_msg {
	char *pBuf;
	struct __print_msg *pNext;
} printf_msg;

typedef enum {
   ST_INIT,
   ST_NORMAL,
   ST_PERCENT,
   ST_NUM,
   ST_QUAL,
   ST_TYPE
} printf_state_t;

/*****************************************************************************************
 * \brief
 *          
 * \param[in] val
 * \param[in] sign
 * \param[in] width
 * \param[in] fill
 * \param[in] base
 *****************************************************************************************
 */
void printint(unsigned long val, int sign, int width, char fill, int base);

/*****************************************************************************************
 * \brief
 *          
 * \param[in] s
 * \param[in] width
 *****************************************************************************************
 */
void printstr(const char *s, int width);

/*****************************************************************************************
 * \brief
 *          
 * \param[in] s
 *****************************************************************************************
 */
void arch_puts(const char *s);

/*****************************************************************************************
 * \brief
 *          
 * \param[in] fmt
 * \param[in] args
 *
 * \return
 *****************************************************************************************
 */
int arch_vprintf(const char *fmt, va_list args);

/*****************************************************************************************
 * \brief
 *          
 * \param[in] fmt
 *
 * \return
 *****************************************************************************************
 */
int arch_printf(const char *fmt, ...);

#ifndef putchar
    #define putchar(c)                              __putchar(c)
#endif

/*****************************************************************************************
 * \brief
 *****************************************************************************************
 */
void arch_printf_process(void);

/*****************************************************************************************
 * \brief
 *          
 * \param[in] val
 * \param[in] delim
 *
 * \return
 *****************************************************************************************
 */
void print_hex (unsigned char val, unsigned char delim);

#else // HAS_PRINTF

	#define arch_puts(s) {}
	#define arch_vprintf(fmt, args) {}
	#define arch_printf(fmt, args...) {}
	#define arch_printf_process() {}
	#define print_hex(val, delim) {}
        
#endif // HAS_PRINTF

/*****************************************************************************************
 * \brief
 *          
 * \param[in] val
 * \param[in] delim
 *
 * \return
 *****************************************************************************************
 */
void print_hex_wr (unsigned char val, unsigned char delim);
    
#endif // _APP_CONSOLE_H_

/**
 * \}
 * \}
 * \}
 */
