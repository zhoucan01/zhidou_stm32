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
#include "wk_adc.h"
#include "wk_debug.h"
#include "wk_exint.h"
#include "wk_tmr.h"
#include "wk_usart.h"
#include "wk_dma.h"
#include "wk_gpio.h"
#include "wk_system.h"
#include "freertos_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "log.h"
#include "enc.h"
#include "moter28.h"
#include "moter42.h"
#include "moter57.h"
#include "AT8236.h"
#include "totolCTL.h"
#include "ws2812.h"
#include "current.h"
#include "tmux1308.h"
#include "getBusVol.h"
#include "wk_debug.h"
#include "../../task/modbusTask.h"
#include "../../app/usrDebug.h"
/* add user code end private includes */

/* private typedef -----------------------------------------------------------*/
/* add user code begin private typedef */

/* add user code end private typedef */

/* private define ------------------------------------------------------------*/
/* add user code begin private define */
bool moter28Fault;
/* add user code end private define */

/* private macro -------------------------------------------------------------*/
/* add user code begin private macro */

/* add user code end private macro */

/* private variables ---------------------------------------------------------*/
/* add user code begin private variables */

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

  /* init adc1 function. */
  wk_adc1_init();

  /* init dma1 channel1 */
  wk_dma1_channel1_init();
  /* config dma channel transfer parameter */
  /* user need to modify define values DMAx_CHANNELy_XXX_BASE_ADDR and DMAx_CHANNELy_BUFFER_SIZE in at32xxx_wk_config.h */
  wk_dma_channel_config(DMA1_CHANNEL1, 
                        (uint32_t)&USART3->dt, 
                        DMA1_CHANNEL1_MEMORY_BASE_ADDR, 
                        DMA1_CHANNEL1_BUFFER_SIZE);
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* init dma1 channel2 */
  /* UART5 现专用于 TX 调试输出，不再使用 DMA RX。wk_config.h 中
   * DMA1_CHANNEL2 的地址/长度占位为 0，若使能该通道会导致 DMA 写入
   * 地址 0 的非法操作，可能干扰 UART5 外设并使 TX 也无法工作。
   * 此处仅初始化通道模式但不使能，保留以备将来需要时重新配置。 */
  wk_dma1_channel2_init();

  /* init usart3 function. */
  wk_usart3_init();

  /* init uart5 function. */
  wk_uart5_init();

  /* init exint function. */
  wk_exint_config();

  /* init tmr1 function. */
  wk_tmr1_init();

  /* init tmr2 function. */
  wk_tmr2_init();

  /* init tmr3 function. */
  wk_tmr3_init();

  /* init tmr4 function. */
  wk_tmr4_init();

  /* init tmr5 function. */
  wk_tmr5_init();

  /* init tmr10 function. */
  wk_tmr10_init();

  /* add user code begin 2 */
	
	/* moter42 双电机：正速度 = 电机1正转 + 电机2反转（自动反向）。 */
  /* PC10（moterRST）上电置高：用 moterEN() 保证内部电平状态与实际引脚一致 */
  moterEN();
	StepperPulse42_Init();
	encInit();
  StepperPulse57_8412_Init();
//  StepperPulse42_Run(500,15);
//	StepperPulse57_8412_RunAtSpeed(300,10,200,15);
  /* WS2812 / 电流 / TMUX1308 / 母线电压 初始化 */
  ws2812Init();
  ws2812SetRgb(255, 255, 255);
  currentInit();
  tmux1308Init();
  /* 此时抬升电机尚未动作（绕组零电流），标定四路电流的零基线。 */
  currentCalibrateRiseZero();
  getBusVoltage();
  /* 上位机接口初始化（UART5 DMA 接收 + 命令解析 + 波形上报） */
  /* 屏幕通信协议初始化（USART3 重配 115200 + DMA 重定向 + 上电版本帧） */
  /* Start USART3 / PB2 RS485 Modbus RTU before the scheduler accepts traffic.
     Calling it again from the Modbus task is intentionally harmless. */
  modbusinit();

  /* UART5 调试口 DMA 接收初始化：重配 DMA1_CHANNEL2 为本地缓冲区。
   * 必须在 wk_uart5_init() 之后调用；与 modbusinit() 对 DMA1_CH1 的处理一致。 */
  usrDebugInit();
	
  /* UART5 调试口：上电/复位后打印三个版本号（固件/硬件/协议）。 */
  wk_debug_print_version();

  /* DEBUG: 打印 USART3 关键寄存器，确认 IDLE 中断使能 / RX 使能 / DMA 使能 */
  {
    uint32_t c1 = USART3->ctrl1;
    uint32_t c3 = USART3->ctrl3;
    uint32_t st = USART3->sts;
    log_d("USART3 C1:%04lX C3:%04lX ST:%04lX",
          (unsigned long)c1, (unsigned long)c3, (unsigned long)st);
  }
	valveOff();
	pressMoterUp(1000);
	{
		/* 抬升上电自检：下行 8 秒消隙 → 上行到高位（碰高位下位机自动停机并清零
		 * 编码器，作为后面下行计程的零点）→ 再从高位直接下行到最低位停住。
		 * 这样上电后抬升一定停在最低位，压制流程可以直接从最低位起算。 */
		uint32_t riseWait;

		riseMoterDown();
		wk_delay_ms(8000);
		riseMoterUp();
		/* 等高位到位：碰高位后下位机自动停机；限位异常时超时放行，交给下行堵转保护。 */
		for(riseWait = 0U; riseWait < 60000U; riseWait += 50U)
		{
			if(riseMoterHighPositionGet() != 0U)
			{
				break;
			}
			wk_delay_ms(50);
		}
		wk_delay_ms(300);
		/* 高位 → 最低位：走满全行程后下位机自动停机并锁存低位标志（离散输入 0x0001）。 */
		riseMoterDown();
		for(riseWait = 0U; riseWait < 60000U; riseWait += 50U)
		{
			if(riseMoterLowPositionGet() != 0U)
			{
				break;
			}
			wk_delay_ms(50);
		}
		riseMoterStop();
	}
	

  /* add user code end 2 */

  /* init freertos function. */
  wk_freertos_init();

  while(1)
  {
    /* add user code begin 3 */
    /* 主循环不再延时，全速轮询上位机命令与波形上报 */
    /* add user code end 3 */
  }
}

  /* add user code begin 4 */

  /* add user code end 4 */
