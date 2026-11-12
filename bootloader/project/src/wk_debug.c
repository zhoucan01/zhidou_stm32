/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_debug.c
  * @brief    work bench config program
  **************************************************************************
  * Copyright (c) 2025, Artery Technology, All rights reserved.
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "wk_debug.h"

/* add user code begin 0 */
#include "wk_system.h"
#include <stdarg.h>
#include <stdio.h>

/* add user code end 0 */

/**
  * @brief  init debug function.
  * @param  none
  * @retval none
  */
void wk_debug_config(void)
{
  /* jtag-dp disabled and sw-dp enabled */
  gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
}

/* add user code begin 1 */

// volatile uint32_t timebase_ticks;
// __WEAK uint32_t wk_timebase_get(void)
// {
//   return timebase_ticks;
// }

// static void log_format_integer(char *out, unsigned int capacity,
//                                const char *format, va_list args)
// {
//   unsigned int used = 0U;
//   while(*format != '\0' && used + 1U < capacity)
//   {
//     unsigned int width = 0U;
//     unsigned int count;
//     unsigned int base;
//     unsigned int longValue = 0U;
//     unsigned int negative = 0U;
//     unsigned long number;
//     long signedNumber;
//     char pad = ' ';
//     char spec;
//     char digits[24];
//     const char *str;
//     if(*format != '%') { out[used++] = *format++; continue; }
//     format++;
//     if(*format == '%') { out[used++] = *format++; continue; }
//     if(*format == '0') { pad = '0'; format++; }
//     while(*format >= '0' && *format <= '9')
//     {
//       width = width * 10U + (unsigned int)(*format++ - '0');
//       if(width > capacity) width = capacity;
//     }
//     if(*format == 'l') { longValue = 1U; format++; }
//     spec = *format;
//     if(spec == '\0') break;
//     format++;
//     if(spec == 's')
//     {
//       str = va_arg(args, const char *);
//       if(str == NULL) str = "(null)";
//       while(*str != '\0' && used + 1U < capacity) out[used++] = *str++;
//       continue;
//     }
//     if(spec == 'c') { out[used++] = (char)va_arg(args, int); continue; }
//     if(spec == 'd' || spec == 'i')
//     {
//       signedNumber = longValue ? va_arg(args, long) : (long)va_arg(args, int);
//       negative = (signedNumber < 0L) ? 1U : 0U;
//       number = negative ? 0UL - (unsigned long)signedNumber : (unsigned long)signedNumber;
//     }
//     else if(spec == 'u' || spec == 'x' || spec == 'X')
//     {
//       number = longValue ? va_arg(args, unsigned long) : (unsigned long)va_arg(args, unsigned int);
//     }
//     else { out[used++] = '?'; break; }
//     base = (spec == 'x' || spec == 'X') ? 16U : 10U;
//     count = 0U;
//     do
//     {
//       unsigned int digit = (unsigned int)(number % base);
//       digits[count++] = (char)(digit < 10U ? '0' + digit :
//                                   ((spec == 'X') ? 'A' : 'a') + digit - 10U);
//       number /= base;
//     } while(number != 0UL);
//     if(negative != 0U && pad == '0' && used + 1U < capacity) out[used++] = '-';
//     while(width > count + negative && used + 1U < capacity)
//     {
//       out[used++] = pad;
//       width--;
//     }
//     if(negative != 0U && pad != '0' && used + 1U < capacity) out[used++] = '-';
//     while(count != 0U && used + 1U < capacity) out[used++] = digits[--count];
//   }
//   if(capacity != 0U) out[used] = '\0';
// }


// void wk_debug_putc(uint8_t ch)
// {
//  /* 超时退出：避免 TDBE 永不置位导致 printf 永久卡死。
//   * 115200 波特率下每字节约 87μs，超时计数 500000 次循环足够覆盖
//   * 最坏情况（被高优先级任务抢占后恢复）。 */
//  uint32_t timeout = 500000U;
//  while(usart_flag_get(UART5, USART_TDBE_FLAG) == RESET)
//  {
//    if((timeout--) == 0U)
//    {
//      return;  /* TDBE 超时，丢弃该字节，避免永久卡死 */
//    }
//  }
//  usart_data_transmit(UART5, ch);
// }


// void log_write(char level, const char *function, uint32_t line,
//               const char *format, ...)
// {
//  char message[192];
//  char prefix[96];
//  uint32_t timestamp = wk_timebase_get() % 100000000U;
//  uint16_t i;
//  va_list args;

//  (void)snprintf(prefix, sizeof(prefix), "[%08lu|%c]%s.%lu :: ",
//                 (unsigned long)timestamp, level, function,
//                 (unsigned long)line);
//  va_start(args, format);
//  log_format_integer(message, sizeof(message), format, args);
//  va_end(args);

//  for(i = 0U; prefix[i] != '\0'; i++) wk_debug_putc((uint8_t)prefix[i]);
//  for(i = 0U; message[i] != '\0'; i++) wk_debug_putc((uint8_t)message[i]);
//  wk_debug_putc('\r');
//  wk_debug_putc('\n');
// }

/* add user code end 1 */
