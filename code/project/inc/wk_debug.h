/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_debug.h
  * @brief    header file of work bench config
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

/* define to prevent recursive inclusion -----------------------------------*/
#ifndef __WK_DEBUG_H
#define __WK_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------------*/
#include "at32f415_wk_config.h"

/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* 三个版本号（上电/复位时通过 UART5 调试口和 USART3 RS485 口打印）
 *   FW_VERSION  = 固件版本  {主, 次, 修订, 构建}
 *   HW_VERSION  = 硬件版本  {主, 次, 修订, 板号}
 *   PROTO_VERSION = 协议版本 {主, 次, 修订, 保留}
 * 每个版本号占 4 字节，与 legacy modbus.c 版本帧格式兼容。
 * 固件版本同时映射到 Modbus 输入寄存器 0x000B~0x000C（FC04），
 * 主站可随时读取。 */
#define FW_VERSION_MAJOR     0U
#define FW_VERSION_MINOR     0U
#define FW_VERSION_PATCH     0U
#define FW_VERSION_BUILD     1U

#define HW_VERSION_MAJOR     1U
#define HW_VERSION_MINOR     0U
#define HW_VERSION_PATCH     0U
#define HW_VERSION_BOARD     0U

#define PROTO_VERSION_MAJOR  1U
#define PROTO_VERSION_MINOR  0U
#define PROTO_VERSION_PATCH  0U
#define PROTO_VERSION_RSVD   0U

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* exported functions ------------------------------------------------------- */

  /* init debug function. */
  void wk_debug_config(void);

/* add user code begin exported functions */

  /* 通过 UART5 阻塞发送一个字节（供 printf 重定向使用）。 */
  void wk_debug_putc(uint8_t ch);

  /* 通过 UART5 打印三个版本号（ASCII 文本），每次复位/上电调用一次。 */
  void wk_debug_print_version(void);

  /* 通过 UART5 打印心跳信息，格式：HEARTBEAT:[时间戳]\r\n
   * 时间戳为 wk_timebase_get() 的原始毫秒计数值（TMR9 1ms 时基）。
   * 由低优先级任务每秒调用一次，用于确认固件存活。 */
  void wk_debug_print_heartbeat(void);

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
