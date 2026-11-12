/* add user code begin Header */
/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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
#include "at32f415_wk_config.h"
#include "wk_crc.h"
#include "wk_debug.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "../app/modbus_port.h"
#include "upgrade.h"
#include "boot_log.h"
#include "wk_debug.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */


/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */

/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */
modbus_10_data_t handshake; /* 与升级模块共享，保持静态零初始化。 */
bootloader_state_t at32bootloader_state = bootloaderWating;
volatile int debugLastArea;
volatile int debugEtase;
volatile const uint32_t bootloaderVersion = 1;
extern volatile uint32_t ticks_count_us; /* 与串口超时共用自由运行 SysTick 时基。 */
/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/**
  * @brief main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  /* add user code begin 1 */
  /* Software queue must be ready BEFORE wk_usart3_init enables RX IRQs.
   * NULL enables query/reset. Bind upgrade callbacks and real application
   * capacity here when the Flash layout is available. */
  (void)modbus_port_init();

  /* add user code end 1 */

  /* system clock config. */
  wk_system_clock_config();

  /* config periph clock. */
  wk_periph_clock_config();

  /* init debug function. */
  wk_debug_config();

  /* nvic config. */
  wk_nvic_config();

  /* timebase config. */
  wk_timebase_init();

  /* init gpio function. */
  wk_gpio_config();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART3->dt, 
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init usart3 function. */
  wk_usart3_init();

  /* init uart5 function. */
  wk_uart5_init();

  /* init crc function. */
  wk_crc_init();

  /* add user code begin 2 */

  uint32_t lastState=0xFFFFFFFFu;
  uint32_t lastStatus=0xFFFFFFFFu;
  boot_log_start();
  boot_trace("Bootloader version", bootloaderVersion, BOOT_LOG_NONE);
  boot_notice("BuildTime " __DATE__ " " __TIME__, BOOT_LOG_NONE, BOOT_LOG_NONE);
  uint32_t bootTick=0;
  uint32_t damagedTick=SysTick->VAL;


  /* add user code end 2 */

  while(1)
  {
    /* add user code begin 3 */
    if(lastState != (uint32_t)at32bootloader_state || lastStatus != modbus_port_get_ota_status())
    {
        lastState = at32bootloader_state; lastStatus = modbus_port_get_ota_status();
        boot_log("STATE boot/OTA", lastState, lastStatus);
    }
    boot_log_poll();
    switch(at32bootloader_state)
    {
        case bootloaderWating:
            appControl();
            if(at32bootloader_state == bootloaderWating && bootTick > 2000)
                at32bootloader_state = bootloaderIntoApp;
            break;
        case bootloaderHandShaking:
            appControl();
            break;
        case bootloaderUpgrading:
            appUpgrade();
            break;
        case bootloaderIntoApp:
            if(!modbus_reply_pending())
                intoApp(); /* 每次启动先校验 A，失败才从 B 恢复。 */
            break;
        default:
            break;
    }
    /* 与查询应答相同的14字节0x10帧，状态0x55；通过公共发送层上报。 */
    if(modbus_port_get_ota_status() == MODBUS_STATUS_DAMAGED)
    {
        if(((damagedTick - SysTick->VAL) & 0x00FFFFFFu) >= 100000u * ticks_count_us)
        {
            if(!modbus_reply_pending())
            {
                (void)modbus_set_10_reply(1);
                damagedTick = SysTick->VAL;
            }
        }
    }
    else
        damagedTick = SysTick->VAL;

        if(bootTick < 2001u) ++bootTick;
    modbusdeal(); /* 统一处理重启请求、CRC16 错误应答等公共操作。 */
		wk_delay_ms(1);
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
