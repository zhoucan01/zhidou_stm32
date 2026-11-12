/* add user code begin Header */
/**
  **************************************************************************
  * @file     wk_usart.c
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
#include "wk_usart.h"

/* add user code begin 0 */
#include "../app/modbus_port.h"
#include "wk_system.h"
#include "wk_dma.h"
#include <string.h>
#include "boot_log.h"

/* add user code end 0 */

/**
  * @brief  init usart3 function
  * @param  none
  * @retval none
  */
void wk_usart3_init(void)
{
  /* add user code begin usart3_init 0 */

  /* add user code end usart3_init 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin usart3_init 1 */

  /* add user code end usart3_init 1 */

  /* configure the TX pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_10;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the RX pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_11;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure param */
  usart_init(USART3, 1000000, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART3, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_parity_selection_config(USART3, USART_PARITY_NONE);

  usart_dma_receiver_enable(USART3, TRUE);

  usart_hardware_flow_control_set(USART3, USART_HARDWARE_FLOW_NONE);

  /* enable idle interrupt */
  usart_interrupt_enable(USART3, USART_IDLE_INT, TRUE);

  /* add user code begin usart3_init 2 */
  /* PB2 RE/DE: low receives, high transmits, matching modbusTask.c. */
  gpio_bits_reset(GPIOB, GPIO_PINS_2);
  /* 保留生成配置的 1 Mbit/s、8N1、DMA RX 和 IDLE；禁止中断抢读 DMA 的 DT。 */
  usart_interrupt_enable(USART3, USART_RDBF_INT, FALSE);
  usart_interrupt_enable(USART3, USART_ERR_INT, TRUE);

  /* add user code end usart3_init 2 */

  usart_enable(USART3, TRUE);

  /* add user code begin usart3_init 3 */

  /* add user code end usart3_init 3 */
}

/**
  * @brief  init uart5 function
  * @param  none
  * @retval none
  */
void wk_uart5_init(void)
{
  /* add user code begin uart5_init 0 */

  /* add user code end uart5_init 0 */

  gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);

  /* add user code begin uart5_init 1 */

  /* add user code end uart5_init 1 */

  /* configure the TX pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_12;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOC, &gpio_init_struct);

  /* configure the RX pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_2;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOD, &gpio_init_struct);

  /* configure param */
  usart_init(UART5, 1000000, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(UART5, TRUE);
  usart_receiver_enable(UART5, TRUE);
  usart_parity_selection_config(UART5, USART_PARITY_NONE);

  usart_hardware_flow_control_set(UART5, USART_HARDWARE_FLOW_NONE);

  /* add user code begin uart5_init 2 */

  /* add user code end uart5_init 2 */

  usart_enable(UART5, TRUE);

  /* add user code begin uart5_init 3 */

  /* add user code end uart5_init 3 */
}

/* add user code begin 1 */

#define MB_TICK_MASK 0x00FFFFFFu
#define MB_RX_ERRORS (USART_ROERR_FLAG | USART_NERR_FLAG | USART_FERR_FLAG | USART_PERR_FLAG)

typedef struct
{
  uint16_t size;
  uint8_t data[MODBUS_PORT_FRAME_SIZE];
  union
  {
    modbus_10_data_t control;
    modbus_aa_data_t packet;
  } decoded;
} mb_rx_slot_t;

/* DMA/USART 中断共同生产完整帧，普通上下文消费；相同抢占优先级防止生产者嵌套。 */
static mb_rx_slot_t mb_slots[MODBUS_PORT_QUEUE_DEPTH];
static volatile uint8_t mb_head, mb_tail, mb_discard, mb_ready, mb_transmitting;
/* 上电绑定此缓冲区，运行时与第二块缓冲区交替接收。 */
uint8_t modbus_port_dma_buffer[MODBUS_PORT_DMA_SIZE];
static volatile uint32_t mb_last_tick;
static uint32_t mb_timeout_dma_count;
static modbus_t mb_context;
/* 跨 DMA 满传输/IDLE 事件保留的组帧缓存及双缓冲状态。 */
static uint16_t mb_assembly_size;
static uint32_t mb_oversize_remaining;
static uint8_t mb_assembly[MODBUS_PORT_FRAME_SIZE];
static uint8_t mb_dma_second[MODBUS_PORT_DMA_SIZE];
static uint8_t *mb_dma_target = modbus_port_dma_buffer;
static volatile uint16_t mb_registers[4];
/* 接收错误独立保存应答码及包号，交给 modbusdeal 发送 0x03/0x07。 */
static uint16_t mb_error_packets[MODBUS_PORT_QUEUE_DEPTH];
static uint8_t mb_error_codes[MODBUS_PORT_QUEUE_DEPTH];
static volatile uint32_t mb_bad_crc_values; /* 上16位收到值，下16位计算值；ISR只保存。 */
static volatile uint8_t mb_error_head, mb_error_tail;
static volatile uint8_t mb_reset_requested;
static volatile uint8_t mb_version_pending;
static volatile uint16_t mb_version_offset, mb_version_count;
#ifndef MODBUS_VERSION_BASE
#define MODBUS_VERSION_BASE ((const uint8_t *)0x08003800u)
#endif
static volatile uint8_t mb_rx_paused;
modbus_port_stats_t modbus_port_stats;
extern volatile uint32_t ticks_count_us;

/**
 * @brief 计算自由递减 SysTick 从指定时刻起经过的计数值。
 * @param tick 先前读取的 24 位计数快照。
 * @return 经过的计数值；按现有时基乘 ticks_count_us 可换算目标微秒数。
 * @note 仅适用于间隔小于一个计数周期的短时等待，可处理一次回绕。
 */
static uint32_t mb_elapsed(uint32_t tick)
{
  /* Existing wk_timebase is a free-running down-counter, no SysTick IRQ. */
  return (tick - SysTick->VAL) & MB_TICK_MASK;
}

/**
 * @brief 初始化 USART3 协议上下文、接收队列和诊断计数。
 * @return 初始化完成后返回 0；协议固定，无运行时配置和存储回调。
 * @note 必须在串口接收中断启用前调用，不负责初始化硬件或擦写 Flash。
 */
int modbus_port_init(void)
{
  mb_ready = 0;
  mb_context.key = 0;
  mb_context.status = MODBUS_STATUS_IDLE;
  mb_head = mb_tail = mb_discard = mb_transmitting = 0;
  mb_error_head = mb_error_tail = 0;
  mb_reset_requested = 0;
  mb_version_pending = 0;
  mb_rx_paused = 0;
  mb_last_tick = 0;
  mb_timeout_dma_count = MODBUS_PORT_DMA_SIZE;
  mb_assembly_size = 0;
  mb_oversize_remaining = 0;
  mb_dma_target = modbus_port_dma_buffer;
  memset((void *)mb_registers, 0, sizeof(mb_registers));
  memset(&modbus_port_stats, 0, sizeof(modbus_port_stats));
  __DMB();
  mb_ready = 1;
  return 0;
}

/**
 * @brief 重装非循环 DMA 接收缓冲区，并清除上一段的 DMA 状态。
 * @note 调用前必须已关闭 DMA 通道，且不能与接收中断并发执行。
 */
static void mb_dma_restart(void)
{
  dma_flag_clear(DMA1_GL1_FLAG);
  wk_dma_channel_config(DMA1_CHANNEL1, (uint32_t)(uintptr_t)&USART3->dt,
                        (uint32_t)(uintptr_t)mb_dma_target,
                        MODBUS_PORT_DMA_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);
}


/**
 * @brief 将 CRC16 正确的完整帧解码并写入寄存器/接收队列。
 * @param length 当前完整帧的字节数，数据位于 mb_assembly。
 * @note E6 为 AT32 本地 OTA 状态，主机不得覆盖；0x10 写请求只更新 E7～E9。
 *       查询请求只入队；0xAA 保存字段及固件，不执行操作码对应业务。
 */
static void mb_store_frame(uint16_t length)
{
  mb_rx_slot_t *slot = &mb_slots[mb_head];
  uint8_t next = (uint8_t)((mb_head + 1u) % MODBUS_PORT_QUEUE_DEPTH);
  int result;
  if(mb_assembly[1] == 0x11u)
  {
    if(!mb_version_pending)
    {
      mb_version_offset = (uint16_t)((uint16_t)mb_assembly[2] << 8 | mb_assembly[3]);
      mb_version_count = (uint16_t)((uint16_t)mb_assembly[4] << 8 | mb_assembly[5]);
      __DMB();
      mb_version_pending = 1;
    }
    ++modbus_port_stats.valid_frames;
    return;
  }
  /* 完整帧只复制一次；AA 直接在队列内解码，不再先复制固件后覆盖整帧。 */
  memcpy(slot->data, mb_assembly, length);
  if(mb_assembly[1] == MODBUS_FUNCTION_CONTROL)
  {
    result = modbus_read_10(&mb_context, mb_assembly, length, &slot->decoded.control);
    if(result == 1 && !slot->decoded.control.is_query)
    {
      mb_registers[1] = (uint16_t)(slot->decoded.control.key >> 16);
      mb_registers[2] = (uint16_t)slot->decoded.control.key;
      mb_registers[3] = slot->decoded.control.operation;
      if(slot->decoded.control.operation == 2)
        mb_reset_requested = 1;
      ++modbus_port_stats.register_writes;
    }
  }
  else
    result = modbus_read_aa(&mb_context, slot->data, length, &slot->decoded.packet,
                            slot->data + 16, MODBUS_PORT_FRAME_SIZE - 22u);
  if(result != 1)
  {
    ++modbus_port_stats.frame_error;
    return;
  }
  ++modbus_port_stats.valid_frames;
  if(next == mb_tail)
  {
    ++modbus_port_stats.queue_overflow;
    return;
  }
  slot->size = length;
  __DMB();
  mb_head = next;
}

/** @brief 按当前协议字节序读取 2 字节字段，调用方保证地址有效。 */
static uint16_t mb_field16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}

/** @brief 登记一条接收错误；中断直接调用，普通上下文必须先屏蔽中断。 */
static void mb_queue_rx_error(uint8_t code, uint16_t packet)
{
  uint8_t next = (uint8_t)((mb_error_head + 1u) % MODBUS_PORT_QUEUE_DEPTH);
  if(next == mb_error_tail)
  {
    ++modbus_port_stats.error_reply_overflow;
    return;
  }
  mb_error_packets[mb_error_head] = packet;
  mb_error_codes[mb_error_head] = code;
  __DMB();
  mb_error_head = next;
}

/**
 * @brief 将一个接收字节追加到组帧缓存，完整后计算 CRC16 并写入缓存。
 * @param byte 本次 DMA 数据块中的一个字节。
 * @note 半帧跨 DMA/IDLE 保留；错误头部、长度或 CRC 按字节重新寻找帧头。
 *       CRC32 只保存，不计算。该函数仅由同优先级接收中断串行调用。
 */
static void mb_assemble_byte(uint8_t byte)
{
  uint32_t expected;
  uint16_t count, received, calculated;
  if(mb_oversize_remaining != 0)
  {
    --mb_oversize_remaining;
    return;
  }
  mb_assembly[mb_assembly_size++] = byte;
  while(mb_assembly_size != 0)
  {
    if(mb_assembly[0] != MODBUS_ADDRESS)
      goto skip;
    if(mb_assembly_size < 2)
      return;
    if(mb_assembly[1] == 0x11u)
    {
      if(mb_assembly_size < 6u) return;
      count = mb_field16(mb_assembly + 4);
      if(count == 0u || count > 32u || mb_field16(mb_assembly + 2) > 1024u - count)
        goto skip;
      expected = 8u;
    }
    else if(mb_assembly[1] == MODBUS_FUNCTION_CONTROL)
    {
      if(mb_assembly_size < 6)
        return;
      if(mb_field16(mb_assembly + 2) != MODBUS_REGISTER_START)
        goto skip;
      count = mb_field16(mb_assembly + 4);
      if(count != 3 && count != 4)
        goto skip;
      expected = count == 3 ? 14u : 20u;
    }
    else if(mb_assembly[1] == MODBUS_FUNCTION_DATA)
    {
      if(mb_assembly_size < 4)
        return;
      expected = (uint32_t)mb_field16(mb_assembly + 2);
      if(expected < 23)
        goto skip;
      if(mb_assembly_size < 12)
        return;
      if((uint32_t)mb_field16(mb_assembly + 10) + 22u != expected)
        goto skip;
      if(expected > MODBUS_PORT_FRAME_SIZE || expected - 22 > MODBUS_MAX_PAYLOAD)
      {
        if(mb_assembly_size < 16)
          return;
        mb_queue_rx_error(MODBUS_ACK_TOO_LONG, mb_field16(mb_assembly + 14));
        mb_oversize_remaining = expected - mb_assembly_size;
        mb_assembly_size = 0;
        return;
      }
    }
    else
      goto skip;
    if(mb_assembly_size < expected)
      return;
    received = (uint16_t)((uint16_t)mb_assembly[expected-1] << 8 | mb_assembly[expected-2]);
    calculated = modbus_crc16(mb_assembly, expected - 2);
    if(calculated != received)
    {
      mb_bad_crc_values = ((uint32_t)received << 16) | calculated;
      ++modbus_port_stats.crc16_error;
      if(mb_assembly[1] == MODBUS_FUNCTION_DATA)
      {
        mb_queue_rx_error(MODBUS_ACK_FRAME_CRC, mb_field16(mb_assembly + 14));
      }
      goto skip;
    }
    mb_store_frame((uint16_t)expected);
    mb_assembly_size = (uint16_t)(mb_assembly_size - expected);
    memmove(mb_assembly, mb_assembly + expected, mb_assembly_size);
    continue;
skip:
    ++modbus_port_stats.frame_error;
    --mb_assembly_size;
    memmove(mb_assembly, mb_assembly + 1, mb_assembly_size);
  }
}

/**
 * @brief DMA 满传输/错误与 USART3 IDLE/错误共用的中断接收入口。
 * @note 先停 DMA、读取接收计数及必要的 STS/DT，再交换双缓冲并重启 DMA，
 *       随后在当前中断内组帧、校验 CRC16、更新寄存器和完整帧队列。
 *       两个中断必须保持相同抢占优先级；不发送、不擦写 Flash、不重启。
 */
void modbus_port_irq(void)
{
  uint32_t status = USART3->sts;
  uint32_t remaining;
  uint16_t length, i;
  uint8_t *completed;
  int dma_error = dma_flag_get(DMA1_DTERR1_FLAG) != RESET;
  int full = dma_flag_get(DMA1_FDT1_FLAG) != RESET;
  if(mb_rx_paused)
    return;
  if((status & (USART_IDLEF_FLAG | MB_RX_ERRORS)) == 0 && !full && !dma_error)
    return;
  dma_channel_enable(DMA1_CHANNEL1, FALSE);
  dma_error |= dma_flag_get(DMA1_DTERR1_FLAG) != RESET;
  remaining = DMA1_CHANNEL1->dtcnt;
  completed = mb_dma_target;
  length = remaining <= MODBUS_PORT_DMA_SIZE ?
    (uint16_t)(MODBUS_PORT_DMA_SIZE - remaining) : 0;
  __DMB();
  /* 保留进入中断时的 IDLE/错误，并捕获停止 DMA 前后新到达的状态。 */
  status = (status & (USART_IDLEF_FLAG | MB_RX_ERRORS)) | USART3->sts;
  if((status & (USART_IDLEF_FLAG | MB_RX_ERRORS)) != 0)
  {
    uint16_t pending;
    status = (status & (USART_IDLEF_FLAG | MB_RX_ERRORS)) | USART3->sts;
    pending = (uint16_t)USART3->dt;
    if((status & USART_RDBF_FLAG) != 0 && (status & MB_RX_ERRORS) == 0)
    {
      if(length < MODBUS_PORT_DMA_SIZE)
        completed[length++] = (uint8_t)pending;
      else
        dma_error = 1;
    }
  }
  if(mb_transmitting)
  {
    dma_flag_clear(DMA1_GL1_FLAG);
    return;
  }
  mb_last_tick = SysTick->VAL;
  if(dma_error || remaining > MODBUS_PORT_DMA_SIZE || (status & MB_RX_ERRORS) != 0)
  {
    ++modbus_port_stats.receive_error;
    mb_discard = 1;
    mb_assembly_size = 0;
    mb_oversize_remaining = 0;
  }
  mb_dma_target = completed == modbus_port_dma_buffer ? mb_dma_second : modbus_port_dma_buffer;
  mb_dma_restart();
  mb_timeout_dma_count = MODBUS_PORT_DMA_SIZE;
  if(mb_ready && !mb_discard)
    for(i = 0; i < length; ++i)
      mb_assemble_byte(completed[i]);
  if((status & USART_IDLEF_FLAG) != 0)
    mb_discard = 0;
}

/**
 * @brief 检查半帧 3.5 ms 静默超时，不把一次 IDLE 直接当作接收失败。
 * @note 活动 DMA 进度改变会刷新计时；只有 DMA 已静默、无待处理串口事件时
 *       才清理残帧，防止仍在接收时误报。完整包号可用才登记 0x03。
 *       复用现有 SysTick，由 modbusdeal 周期调度，不能保证精确时刻中断触发。
 */
void modbus_port_check_timeout(void)
{
  uint32_t mask, remaining;
  if(!mb_ready || mb_rx_paused || ticks_count_us == 0)
    return;
  mask = __get_PRIMASK();
  __disable_irq();
  remaining = DMA1_CHANNEL1->dtcnt;
  if(mb_transmitting || (USART3->sts & (USART_RDBF_FLAG | USART_IDLEF_FLAG | MB_RX_ERRORS)) != 0)
  {
    __set_PRIMASK(mask);
    return;
  }
  if(remaining != mb_timeout_dma_count)
  {
    mb_timeout_dma_count = remaining;
    mb_last_tick = SysTick->VAL;
  }
  if(remaining == MODBUS_PORT_DMA_SIZE && (mb_assembly_size != 0 || mb_oversize_remaining != 0) &&
     mb_elapsed(mb_last_tick) >= ticks_count_us * MODBUS_RX_TIMEOUT_US)
  {
    if(mb_assembly_size >= 16 && mb_assembly[0] == MODBUS_ADDRESS &&
       mb_assembly[1] == MODBUS_FUNCTION_DATA)
      mb_queue_rx_error(MODBUS_ACK_RX_TIMEOUT, mb_field16(mb_assembly + 14));
    mb_assembly_size = 0;
    mb_oversize_remaining = 0;
    ++modbus_port_stats.receive_timeout;
  }
  __set_PRIMASK(mask);
}

/**
 * @brief 带超时地等待 USART3 指定状态标志置位。
 * @param flag 等待的标志，例如 TDBE 或 TDC。
 * @return 1 表示标志已置位，0 表示等待超过 10 ms。
 * @note 依赖已初始化的 SysTick 时基，仅在主循环发送流程调用。
 */
static int mb_wait_flag(uint32_t flag)
{
  uint32_t start = SysTick->VAL;
  while(usart_flag_get(USART3, flag) == RESET)
    if(mb_elapsed(start) >= ticks_count_us * 10000u)
      return 0;
  return 1;
}

/**
 * @brief 通过 USART3 发送完整 RS485 应答，使用 PB2 控制收发方向。
 * @param data 应答数据地址，由内部调用方保证有效。
 * @param size 应答字节数。
 * @return 1 表示发送完成且已恢复接收方向；0 表示总线等待或发送超时。
 * @note 等待接收静默至少 2 ms 后拉高 PB2；逐字节发送后等待 TDC，
 *       再保持 30 us（当前 1 Mbit/s、8N1 下的 3 字符时间）才拉低 PB2。
 */
static int mb_send(const uint8_t *data, uint16_t size)
{
  uint16_t i;
  uint32_t mask, start = SysTick->VAL;
  uint32_t previous_count = DMA1_CHANNEL1->dtcnt;
  /* Same bus-release rule as reference: 2 ms since the last RX activity.
   * Every byte restarts the wait; abandon a bus busy for 10 ms. */
  for(;;)
  {
    if(mb_elapsed(start) >= ticks_count_us * 10000u)
      return 0;
    mask = __get_PRIMASK();
    __disable_irq();
    /* DMA 仍有进度就重新开始静默计时，不能只依赖上一次 IDLE 的时间。 */
    if(previous_count != DMA1_CHANNEL1->dtcnt)
    {
      previous_count = DMA1_CHANNEL1->dtcnt;
      mb_last_tick = SysTick->VAL;
    }
    if(DMA1_CHANNEL1->dtcnt == MODBUS_PORT_DMA_SIZE &&
       (USART3->sts & (USART_RDBF_FLAG | USART_IDLEF_FLAG | MB_RX_ERRORS)) == 0 &&
       mb_elapsed(mb_last_tick) >= ticks_count_us * 2000u)
      break;
    __set_PRIMASK(mask);
  }
  mb_transmitting = 1;
  /* 发送期间暂停 RX DMA，防止回环数据进入下一段。 */
  dma_channel_enable(DMA1_CHANNEL1, FALSE);
  gpio_bits_set(GPIOB, GPIO_PINS_2);
  __set_PRIMASK(mask);
  for(i = 0; i < size; ++i)
  {
    if(!mb_wait_flag(USART_TDBE_FLAG))
      goto failed;
    usart_data_transmit(USART3, data[i]);
  }
  /* TDBE does not mean the last stop bit was sent. Wait TDC, then hold
   * PB2 for another three characters: 30 us at generated 1 Mbit/s, 8N1. */
  if(!mb_wait_flag(USART_TDC_FLAG))
    goto failed;
  wk_delay_us(30);
  mask = __get_PRIMASK();
  __disable_irq();
  usart_flag_clear(USART3, USART_IDLEF_FLAG);
  mb_discard = 0;
  mb_dma_restart();
  gpio_bits_reset(GPIOB, GPIO_PINS_2);
  mb_transmitting = 0;
  __set_PRIMASK(mask);
  return 1;
failed:
  mask = __get_PRIMASK();
  __disable_irq();
  usart_flag_clear(USART3, USART_IDLEF_FLAG);
  mb_discard = 0;
  mb_dma_restart();
  gpio_bits_reset(GPIOB, GPIO_PINS_2);
  mb_transmitting = 0;
  __set_PRIMASK(mask);
  return 0;
}

/* Keep protocol scanning in modbus.c; this adapter owns only the UART/GPIO
 * send operation. Success means TDC and RE/DE release have completed. */
/**
 * @brief 将协议层同步发送回调适配到 USART3，并更新发送诊断计数。
 * @param user 回调透传参数，本实现不使用。
 * @param data 协议层生成的应答数据。
 * @param size 应答字节数，来自协议层固定容量的应答缓冲区。
 * @return 1 表示物理发送和方向释放完成；0 表示发送失败。
 */
static int mb_send_response(void *user, const uint8_t *data, size_t size)
{
  (void)user;
  if(!mb_send(data, (uint16_t)size))
  {
    ++modbus_port_stats.transmit_error;
    return 0;
  }
  boot_log_hex("TX", data, size);
#if BOOT_LOG_ENABLED
  if(data[1] == 0x11u)
    boot_notice("TX version chunk", mb_field16(data+3), BOOT_LOG_NONE);
  else if(size == 7u)
  {
    static const char *const replies[] = {"", "WRITE OK", "ERROR: payload CRC32", "ERROR: RX timeout", "ERROR: protocol version", "ERROR: session key", "ERROR: packet sequence", "ERROR: frame CRC16", "ERROR: length", "ERROR: Flash write"};
    boot_notice("TX ACK packet", mb_field16(data+3), BOOT_LOG_NONE);
    boot_notice(data[2] <= 9u ? replies[data[2]] : replies[0], BOOT_LOG_NONE, BOOT_LOG_NONE);
  }
  else if(size == 14u)
    boot_notice("TX OTA status", mb_field16(data+6), BOOT_LOG_NONE);
  else
    boot_notice("TX control ACK", BOOT_LOG_NONE, BOOT_LOG_NONE);
#endif
  ++modbus_port_stats.responses;
  return 1;
}

/**
 * @brief 底层发送指定 0xAA 应答，业务应先通过 modbus_set_ack() 登记。
 * @param ack 应答码；packet 为对应包号，包号大端、CRC16 小端。
 * @return 1 已发送，-1 未初始化、参数错误或发送失败；禁止中断内调用。
 */
int modbus_port_reply_aa(modbus_ack_t ack, uint16_t packet)
{
  uint8_t response[7];
  int size;
  if(!mb_ready)
    return -1;
  size = modbus_build_aa_reply(&mb_context, ack, packet, response, sizeof(response));
  if(size < 0)
    return -1;
  return mb_send_response(NULL, response, (size_t)size) ? 1 : -1;
}

/** @brief 仅由 modbusdeal 调用，发送已登记的查询/控制应答。 */
int modbus_port_reply_10(uint8_t is_query)
{
  uint8_t response[14];
  int size;
  if(!mb_ready)
    return -1;
  size = modbus_build_10_reply(&mb_context, is_query, response, sizeof(response));
  return size > 0 && mb_send_response(NULL, response, (size_t)size) ? 1 : -1;
}

int modbus_port_reply_version(void)
{
  uint8_t response[40];
  uint16_t offset, count, crc;
  if(!mb_ready || !mb_version_pending) return 0;
  offset = mb_version_offset; count = mb_version_count;
  response[0] = MODBUS_ADDRESS; response[1] = 0x11u;
  response[2] = 0u; /* Bootloader, never evidence of APP startup. */
  response[3] = (uint8_t)(offset >> 8); response[4] = (uint8_t)offset;
  response[5] = (uint8_t)count;
  memcpy(response + 6, MODBUS_VERSION_BASE + offset, count);
  crc = modbus_crc16(response, count + 6u);
  response[count + 6u] = (uint8_t)crc; response[count + 7u] = (uint8_t)(crc >> 8);
  if(!mb_send_response(NULL, response, count + 8u)) return -1;
  mb_version_pending = 0;
  return 1;
}

/** @brief 接受开始 OTA 后由 AT32 保存会话密钥，查询应答使用该值。 */
void modbus_port_set_session_key(uint32_t key)
{
  mb_context.key = key;
}

/** @brief 仅对本包固件内容核对硬件 CRC32，协议字段不参与计算。 */
int modbus_port_check_aa_crc32(const modbus_aa_data_t *packet, const uint8_t *firmware, size_t size)
{
  return mb_ready ? modbus_check_aa_crc32(packet, firmware, size) : -1;
}

/** @brief 分区擦除前停止接收，禁止暂停期间的 DMA/USART 事件更新组帧缓存。 */
void modbus_port_pause_rx(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  mb_rx_paused = 1;
  usart_interrupt_enable(USART3, USART_IDLE_INT, FALSE);
  usart_interrupt_enable(USART3, USART_ERR_INT, FALSE);
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT | DMA_DTERR_INT, FALSE);
  dma_channel_enable(DMA1_CHANNEL1, FALSE);
  usart_receiver_enable(USART3, FALSE);
  usart_flag_clear(USART3, USART_IDLEF_FLAG);
  dma_flag_clear(DMA1_GL1_FLAG);
  mb_assembly_size = 0;
  mb_oversize_remaining = 0;
  /* 新会话/跳转之前清掉旧的完整帧和错误队列，避免擦除后再消费旧包。 */
  mb_head = mb_tail = 0;
  mb_error_head = mb_error_tail = 0;
  mb_version_pending = 0;
  __set_PRIMASK(mask);
}

/** @brief 擦除完成后重装 DMA，清理残留标志再恢复接收。 */
void modbus_port_resume_rx(void)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  usart_flag_clear(USART3, USART_IDLEF_FLAG);
  mb_discard = 0;
  mb_assembly_size = 0;
  mb_oversize_remaining = 0;
  mb_dma_restart();
  mb_timeout_dma_count = MODBUS_PORT_DMA_SIZE;
  mb_last_tick = SysTick->VAL;
  mb_rx_paused = 0;
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT | DMA_DTERR_INT, TRUE);
  usart_receiver_enable(USART3, TRUE);
  usart_interrupt_enable(USART3, USART_IDLE_INT, TRUE);
  usart_interrupt_enable(USART3, USART_ERR_INT, TRUE);
  __set_PRIMASK(mask);
}

/** @brief 发送一条待处理接收错误应答（超时/CRC），成功才释放，失败留待重试。 */
int modbus_port_reply_errors(void)
{
  uint16_t packet;
  if(!mb_ready || mb_error_tail == mb_error_head)
    return 0;
  __DMB();
  packet = mb_error_packets[mb_error_tail];
  if(modbus_port_reply_aa((modbus_ack_t)mb_error_codes[mb_error_tail], packet) != 1)
    return -1;
  __DMB();
  mb_error_tail = (uint8_t)((mb_error_tail + 1u) % MODBUS_PORT_QUEUE_DEPTH);
  return 1;
}

/** @brief 原子取走有效控制帧登记的重启标记，不受正常数据队列满或取数顺序影响。 */
int modbus_port_take_reset_request(void)
{
  int requested;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  requested = mb_reset_requested;
  mb_reset_requested = 0;
  __set_PRIMASK(mask);
  return requested;
}


/**
 * @brief 读取 E6～E9 一致快照：E6 来自本地 OTA 状态，E7～E9 来自接收缓存。
 * @param start 起始寄存器地址；count 为寄存器数量；out 为调用方输出数组。
 * @return 1 表示成功，-1 表示参数或寄存器范围无效；不消费接收帧。
 */
int modbus_port_read_registers(uint16_t start, uint16_t count, uint16_t *out)
{
  uint32_t mask;
  uint16_t i, offset;
  if(out == NULL || count == 0 || start < MODBUS_REGISTER_START)
    return -1;
  offset = (uint16_t)(start - MODBUS_REGISTER_START);
  if(offset >= 4 || count > 4u - offset)
    return -1;
  mask = __get_PRIMASK();
  __disable_irq();
  for(i = 0; i < count; ++i)
    out[i] = offset + i == 0 ? mb_context.status : mb_registers[offset + i];
  __set_PRIMASK(mask);
  return 1;
}

/**
 * @brief 由 AT32 业务逻辑设置本地 OTA 状态，主机请求不能直接写入该值。
 * @param status 空闲为 1，下载中为 2，其余值表示禁止 OTA。
 * @note 与查询应答和 230 寄存器共用 mb_context.status，避免多份状态不一致。
 *       只改 RAM 状态，不应答、不启动升级、不擦写 Flash、不复位。
 */
void modbus_port_set_ota_status(uint16_t status)
{
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if(mb_ready)
    mb_context.status = status;
  __set_PRIMASK(mask);
}

/** @brief 读取 AT32 当前 OTA 状态；初始化前返回禁止 OTA。 */
uint16_t modbus_port_get_ota_status(void)
{
  uint16_t status;
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  status = mb_ready ? mb_context.status : MODBUS_STATUS_FORBIDDEN;
  __set_PRIMASK(mask);
  return status;
}

/**
 * @brief 复制中断已组帧、校验和解码的 0x10 字段，不再解析原始字节。
 * @param out 调用方结构体。
 * @return 1 成功，0 暂无匹配帧，-1 参数无效；队首另一类型不会被跳过。
 */
int modbus_port_get_10(modbus_10_data_t *out)
{
  if(out == NULL)
    return -1;
  if(!mb_ready || mb_tail == mb_head)
    return 0;
  __DMB();
  if(mb_slots[mb_tail].data[1] != MODBUS_FUNCTION_CONTROL)
    return 0;
  boot_log_hex("RX10 OK", mb_slots[mb_tail].data, mb_slots[mb_tail].size);
  *out = mb_slots[mb_tail].decoded.control;
#if BOOT_LOG_ENABLED
  {
    static const char *const commands[] = {"RX: no operation", "RX: start OTA", "RX: reboot", "RX: rollback"};
    boot_notice(out->is_query ? "RX: query OTA status" : out->operation<=3u ? commands[out->operation] : "RX: unknown command", BOOT_LOG_NONE, BOOT_LOG_NONE);
  }
#endif
  __DMB();
  mb_tail = (uint8_t)((mb_tail + 1u) % MODBUS_PORT_QUEUE_DEPTH);
  return 1;
}

/**
 * @brief 复制中断已处理的 0xAA 字段及固件内容，不重复计算 CRC。
 * @param out 输出字段；firmware 为固件输出缓冲区；capacity 为可写字节数。
 * @return 1 成功，0 暂无匹配帧，-1 参数或容量无效，保留该帧供重试。
 */
int modbus_port_get_aa(modbus_aa_data_t *out, uint8_t *firmware, size_t capacity)
{
  mb_rx_slot_t *slot;
  if(out == NULL || firmware == NULL)
    return -1;
  if(!mb_ready || mb_tail == mb_head)
    return 0;
  __DMB();
  slot = &mb_slots[mb_tail];
  if(slot->data[1] != MODBUS_FUNCTION_DATA)
    return 0;
  if(capacity < slot->decoded.packet.firmware_size)
    return -1;
  memcpy(firmware, slot->data + 16, slot->decoded.packet.firmware_size);
  boot_log_hex("AA head", slot->data, 16);
  boot_log_hex("AA tail", slot->data + slot->size - 6u, 6);
  boot_log_hex("DATA prefix", slot->data + 16,
      slot->decoded.packet.firmware_size < BOOT_LOG_PAYLOAD_BYTES ? slot->decoded.packet.firmware_size : BOOT_LOG_PAYLOAD_BYTES);
  boot_log("CRC16 OK", slot->decoded.packet.received_crc16, slot->decoded.packet.received_crc16);
  *out = slot->decoded.packet;
  boot_notice("RX packet/last", out->packet, out->total_packets);
  __DMB();
  mb_tail = (uint8_t)((mb_tail + 1u) % MODBUS_PORT_QUEUE_DEPTH);
  return 1;
}


#if BOOT_LOG_ENABLED
static uint8_t log_ready;

/* UART5轮询输出，单字符等待最多2ms；不关闭全局中断、不使用USART3。 */
static void log_char(char value)
{
  uint32_t start = SysTick->VAL;
  if(!log_ready) return;
  while(usart_flag_get(UART5, USART_TDBE_FLAG) == RESET)
    if(mb_elapsed(start) >= ticks_count_us * 2000u) return;
  usart_data_transmit(UART5, (uint8_t)value);
}
static void log_text(const char *text)
{
  while(*text) log_char(*text++);
}
static void log_decimal(uint32_t value)
{
  char digits[10]; unsigned n=0;
  do { digits[n++]=(char)('0'+value%10u); value/=10u; } while(value);
  while(n) log_char(digits[--n]);
}
/* 文字与十进制包号，NONE表示省略该数值。 */
void boot_notice(const char *text, uint32_t first, uint32_t second)
{
  log_text(text);
  if(first != BOOT_LOG_NONE) { log_char(' '); log_decimal(first); }
  if(second != BOOT_LOG_NONE) { log_char('/'); log_decimal(second); }
  log_text("\r\n");
}
void boot_trace_at(const char *function, uint32_t line, const char *text,
                   uint32_t first, uint32_t second)
{
  log_char('['); log_text(function); log_char(':'); log_decimal(line);
  log_text("] ");
  boot_notice(text, first, second);
}
#if BOOT_LOG_VERBOSE
static void log_hex(uint32_t value, unsigned digits)
{
  static const char hex[] = "0123456789ABCDEF";
  while(digits) { --digits; log_char(hex[(value >> (digits * 4u)) & 15u]); }
}
void boot_log(const char *tag, uint32_t first, uint32_t second)
{
  log_text(tag); log_char(' '); log_hex(first, 8); log_char(' '); log_hex(second, 8); log_text("\r\n");
}
void boot_log_hex(const char *tag, const uint8_t *data, size_t size)
{
  log_text(tag); log_char(' ');
  while(size--) { log_hex(*data++, 2); log_char(' '); }
  log_text("\r\n");
}
#endif
void boot_log_start(void)
{
  log_ready=1;
}
void boot_log_flush(void)
{
  uint32_t start = SysTick->VAL;
  while(log_ready && usart_flag_get(UART5, USART_TDC_FLAG) == RESET)
    if(mb_elapsed(start) >= ticks_count_us * 2000u) return;
}
/* ISR诊断只读快照，打印时保持中断打开；多次异常合并为计数和最后CRC值。 */
void boot_log_poll(void)
{
  static uint32_t last_crc, last_drop, last_error, last_timeout;
  uint32_t mask = __get_PRIMASK(), crc, values;
  __disable_irq(); crc = modbus_port_stats.crc16_error; values = mb_bad_crc_values; __set_PRIMASK(mask);
  (void)values; /* 简明模式不展开CRC数值，详细模式仍输出。 */
  if(crc != last_crc) { boot_notice("CRC16 error count", crc, BOOT_LOG_NONE); boot_log("CRC16 BAD", values >> 16, values & 65535u); last_crc=crc; }
  if(modbus_port_stats.queue_overflow != last_drop) { last_drop=modbus_port_stats.queue_overflow; boot_log("RX full", last_drop, 0); }
#if BOOT_LOG_VERBOSE
  if(modbus_port_stats.receive_error != last_error || modbus_port_stats.receive_timeout != last_timeout)
  {
    last_error=modbus_port_stats.receive_error;last_timeout=modbus_port_stats.receive_timeout;
    boot_notice("RX errors/timeouts", last_error, last_timeout);
    boot_log("RX err/tmo", last_error, last_timeout);
  }
#else
  (void)last_error; (void)last_timeout;
#endif
}
#endif

/* add user code end 1 */
