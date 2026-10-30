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
#include "wk_usart.h"
#include "at32f415_usart.h"
#include "wk_system.h"
#include "log.h"
#include <stdarg.h>
#include <stdio.h>
/* 毫秒级时基（实现在 wk_system.c，声明在 wk_debug.h 的用户代码区）。 */
extern uint32_t wk_timebase_get(void);
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

/* ---------- UART5 调试输出 ----------
 * UART5 (PC12 TX / PD2 RX, 115200 8N1) 由 wk_uart5_init() 配置。
 * 原上位机 hostif.c / legacy modbus.c 符号已被链接器移除，UART5 现专用于
 * 调试 printf 输出。以下 fputc 将 printf 重定向到 UART5。
 * 注意：AC5 (ARMCC V5.06) 的标准库通过 fputc 输出字符，
 * #pragma import(__use_no_semihosting) 关闭半主机模式，需提供桩函数。 */
#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;
FILE __stderr;

/* AC5 关闭半主机模式后必须提供的桩函数。 */
void _sys_exit(int x)
{
  (void)x;
  while(1)
  {
  }
}

void _ttywrch(int ch)
{
  (void)ch;
}

void wk_debug_putc(uint8_t ch)
{
  /* 超时退出：避免 TDBE 永不置位导致 printf 永久卡死。
   * 115200 波特率下每字节约 87μs，超时计数 500000 次循环足够覆盖
   * 最坏情况（被高优先级任务抢占后恢复）。 */
  uint32_t timeout = 500000U;
  while(usart_flag_get(UART5, USART_TDBE_FLAG) == RESET)
  {
    if((timeout--) == 0U)
    {
      return;  /* TDBE 超时，丢弃该字节，避免永久卡死 */
    }
  }
  usart_data_transmit(UART5, ch);
}

/* printf 重定向：AC5 标准库通过 fputc 输出每个字符。 */
int fputc(int ch, FILE *f)
{
  (void)f;
  wk_debug_putc((uint8_t)ch);
  return ch;
}

void log_write(char level, const char *function, uint32_t line,
               const char *format, ...)
{
  char message[192];
  char prefix[96];
  uint32_t timestamp = wk_timebase_get() % 100000000U;
  uint16_t i;
  va_list args;

  (void)snprintf(prefix, sizeof(prefix), "[%08lu|%c]%s.%lu :: ",
                 (unsigned long)timestamp, level, function,
                 (unsigned long)line);
  va_start(args, format);
  (void)vsnprintf(message, sizeof(message), format, args);
  va_end(args);

  for(i = 0U; prefix[i] != '\0'; i++) wk_debug_putc((uint8_t)prefix[i]);
  for(i = 0U; message[i] != '\0'; i++) wk_debug_putc((uint8_t)message[i]);
  wk_debug_putc('\r');
  wk_debug_putc('\n');
}

/* 通过 UART5 打印三个版本号（ASCII 文本行），每次复位/上电调用一次。 */
void wk_debug_print_version(void)
{
  /* V2.1.0-rc1 标记用于确认烧录的是最新固件（每次修改源码重新编译会更新 __DATE__/__TIME__）。 */
  log_d("NULLVE_V2 V2.1.0-rc1 Boot OK FW=%u.%u.%u build=%u HW=%u.%u.%u board=%u PROTO=%u.%u.%u build_date=%s build_time=%s",
        FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH, FW_VERSION_BUILD,
        HW_VERSION_MAJOR, HW_VERSION_MINOR, HW_VERSION_PATCH, HW_VERSION_BOARD,
        PROTO_VERSION_MAJOR, PROTO_VERSION_MINOR, PROTO_VERSION_PATCH,
        __DATE__, __TIME__);
}

/* 通过 UART5 打印心跳信息（每秒一次）。
 * 格式：HEARTBEAT:[时间戳]\r\n
 * 时间戳为 wk_timebase_get() 的原始毫秒计数值（TMR9 1ms 时基）。
 * 用于上位机/串口助手确认固件存活。 */
void wk_debug_print_heartbeat(void)
{
  uint32_t timestamp = wk_timebase_get();
  printf("HEARTBEAT:[%lu]\r\n", (unsigned long)timestamp);
}

/* add user code end 1 */
