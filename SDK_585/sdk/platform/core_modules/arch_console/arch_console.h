/*****************************************************************************************
 *
 * @file arch_console.h
 *
 * @brief Serial logging interface header file.
  *
******************************************************************************************/

#ifndef _ARCH_CONSOLE_H_
#define _ARCH_CONSOLE_H_


#if defined (CFG_PRINTF)

#include <stdarg.h>

void arch_puts(const char *s);

int arch_vprintf(const char *fmt, va_list args);

int arch_printf(const char *fmt, ...);

void arch_printf_process(void);
bool arch_printf_transfer_pending(void);

#else // CFG_PRINTF

#define arch_puts(s) {}
#define arch_vprintf(fmt, args) {}
#define arch_printf(fmt, args...) {}
#define arch_printf_process() {}    
#define arch_printf_transfer_pending() false
    
#endif // CFG_PRINTF

    
#endif // _ARCH_CONSOLE_H_
