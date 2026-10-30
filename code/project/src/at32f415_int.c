/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f415_int.c
  * @brief    main interrupt service routines.
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

/* includes ------------------------------------------------------------------*/
#include "at32f415_int.h"
#include "wk_system.h"
#include "freertos_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "wk_debug.h"
#include "moter28.h"
#include "moter42.h"
#include "moter57.h"
#include "enc.h"
#include "../../task/modbusTask.h"
#include "FT.h"
#include "../../app/usrDebug.h"
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


/* add user code end private variables */

/* private function prototypes --------------------------------------------*/
/* add user code begin function prototypes */

/* add user code end function prototypes */

/* private user code ---------------------------------------------------------*/
/* add user code begin 0 */

/* add user code end 0 */

/* external variables ---------------------------------------------------------*/
/* add user code begin external variables */

/* add user code end external variables */

/**
  * @brief  this function handles nmi exception.
  * @param  none
  * @retval none
  */
void NMI_Handler(void)
{
  /* add user code begin NonMaskableInt_IRQ 0 */

  /* add user code end NonMaskableInt_IRQ 0 */

  /* add user code begin NonMaskableInt_IRQ 1 */

  /* add user code end NonMaskableInt_IRQ 1 */
}

/**
  * @brief  this function handles hard fault exception.
  * @param  none
  * @retval none
  */
void HardFault_Handler(void)
{
  /* add user code begin HardFault_IRQ 0 */

  /* add user code end HardFault_IRQ 0 */
  /* go to infinite loop when hard fault exception occurs */
  while (1)
  {
    /* add user code begin W1_HardFault_IRQ 0 */

    /* add user code end W1_HardFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles memory manage exception.
  * @param  none
  * @retval none
  */
void MemManage_Handler(void)
{
  /* add user code begin MemoryManagement_IRQ 0 */

  /* add user code end MemoryManagement_IRQ 0 */
  /* go to infinite loop when memory manage exception occurs */
  while (1)
  {
    /* add user code begin W1_MemoryManagement_IRQ 0 */

    /* add user code end W1_MemoryManagement_IRQ 0 */
  }
}

/**
  * @brief  this function handles bus fault exception.
  * @param  none
  * @retval none
  */
void BusFault_Handler(void)
{
  /* add user code begin BusFault_IRQ 0 */

  /* add user code end BusFault_IRQ 0 */
  /* go to infinite loop when bus fault exception occurs */
  while (1)
  {
    /* add user code begin W1_BusFault_IRQ 0 */

    /* add user code end W1_BusFault_IRQ 0 */
  }
}

/**
  * @brief  this function handles usage fault exception.
  * @param  none
  * @retval none
  */
void UsageFault_Handler(void)
{
  /* add user code begin UsageFault_IRQ 0 */

  /* add user code end UsageFault_IRQ 0 */
  /* go to infinite loop when usage fault exception occurs */
  while (1)
  {
    /* add user code begin W1_UsageFault_IRQ 0 */

    /* add user code end W1_UsageFault_IRQ 0 */
  }
}


/**
  * @brief  this function handles debug monitor exception.
  * @param  none
  * @retval none
  */
void DebugMon_Handler(void)
{
  /* add user code begin DebugMonitor_IRQ 0 */

  /* add user code end DebugMonitor_IRQ 0 */
  /* add user code begin DebugMonitor_IRQ 1 */

  /* add user code end DebugMonitor_IRQ 1 */
}

extern void xPortSysTickHandler(void);

/**
  * @brief  this function handles systick handler.
  * @param  none
  * @retval none
  */
void SysTick_Handler(void)
{
  /* add user code begin SysTick_IRQ 0 */

  /* add user code end SysTick_IRQ 0 */

#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */

  /* add user code begin SysTick_IRQ 1 */

  /* add user code end SysTick_IRQ 1 */
}

/**
  * @brief  this function handles ADC1 handler.
  * @param  none
  * @retval none
  */
void ADC1_IRQHandler(void)
{
  /* add user code begin ADC1_IRQ 0 */

  /* add user code end ADC1_IRQ 0 */

  if(adc_interrupt_flag_get(ADC1, ADC_PCCE_FLAG) != RESET)
  {
    /* add user code begin ADC1_ADC_PCCE_FLAG */
    /* clear flag */
    adc_flag_clear(ADC1, ADC_PCCE_FLAG);
    updateMoter57CurAdc();
		
    /* add user code end ADC1_ADC_PCCE_FLAG */ 
  }

  /* add user code begin ADC1_IRQ 1 */

  /* add user code end ADC1_IRQ 1 */
}

/**
  * @brief  this function handles EXINT Line [9:5] handler.
  * @param  none
  * @retval none
  */
void EXINT9_5_IRQHandler(void)
{
  /* add user code begin EXINT9_5_IRQ 0 */

  /* add user code end EXINT9_5_IRQ 0 */

  if(exint_interrupt_flag_get(EXINT_LINE_5) != RESET)
  {
    /* add user code begin EXINT_LINE_5 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_5);
    waterFTtick();
    /* add user code end EXINT_LINE_5 */ 
  }

  if(exint_interrupt_flag_get(EXINT_LINE_8) != RESET)
  {
    /* add user code begin EXINT_LINE_8 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_8);
    encRiseMoterLEncExintIrq();
    /* add user code end EXINT_LINE_8 */ 
  }

  if(exint_interrupt_flag_get(EXINT_LINE_9) != RESET)
  {
    /* add user code begin EXINT_LINE_9 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_9);
    encPressMoterREncExintIrq();
    /* add user code end EXINT_LINE_9 */ 
  }

  /* add user code begin EXINT9_5_IRQ 1 */

  /* add user code end EXINT9_5_IRQ 1 */
}

/**
  * @brief  this function handles TMR1 Brake and TMR9 handler.
  * @param  none
  * @retval none
  */
void TMR1_BRK_TMR9_IRQHandler(void)
{
  /* add user code begin TMR1_BRK_TMR9_IRQ 0 */

  /* add user code end TMR1_BRK_TMR9_IRQ 0 */

  wk_timebase_handler();
  /* add user code begin TMR1_BRK_TMR9_IRQ 1 */

  /* add user code end TMR1_BRK_TMR9_IRQ 1 */
}

/**
  * @brief  this function handles TMR1 Overflow and TMR10 handler.
  * @param  none
  * @retval none
  */
void TMR1_OVF_TMR10_IRQHandler(void)
{
  /* add user code begin TMR1_OVF_TMR10_IRQ 0 */

  /* add user code end TMR1_OVF_TMR10_IRQ 0 */

 /* overflow interrupt management */
  if(tmr_interrupt_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
  {
    /* add user code begin TMR1_TMR_OVF_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
    /* add user code end TMR1_TMR_OVF_FLAG */
  }

  /* overflow interrupt management */
  if(tmr_interrupt_flag_get(TMR10, TMR_OVF_FLAG) != RESET)
  {
    /* add user code begin TMR10_TMR_OVF_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR10, TMR_OVF_FLAG);
		moter28RunStep();
    /* add user code end TMR10_TMR_OVF_FLAG */
  }

  /* add user code begin TMR1_OVF_TMR10_IRQ 1 */

  /* add user code end TMR1_OVF_TMR10_IRQ 1 */
}

/**
  * @brief  this function handles TMR1 Channel handler.
  * @param  none
  * @retval none
  */
void TMR1_CH_IRQHandler(void)
{
  /* add user code begin TMR1_CH_IRQ 0 */

  /* add user code end TMR1_CH_IRQ 0 */

  /* add user code begin TMR1_CH_IRQ 1 */

  /* add user code end TMR1_CH_IRQ 1 */
}

/**
  * @brief  this function handles TMR3 handler.
  * @param  none
  * @retval none
  */
void TMR3_GLOBAL_IRQHandler(void)
{
  /* add user code begin TMR3_GLOBAL_IRQ 0 */

  if(tmr_interrupt_flag_get(TMR3, TMR_C3_FLAG) != RESET)
  {
    tmr_flag_clear(TMR3, TMR_C3_FLAG);
    encPressMoterREncTmr3Ch3Irq();
  }

  /* add user code end TMR3_GLOBAL_IRQ 0 */

  /* overflow interrupt management */
  if(tmr_interrupt_flag_get(TMR3, TMR_OVF_FLAG) != RESET)
  {
    /* add user code begin TMR3_TMR_OVF_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR3, TMR_OVF_FLAG);
    /* add user code end TMR3_TMR_OVF_FLAG */
  }

  /* add user code begin TMR3_GLOBAL_IRQ 1 */

  /* add user code end TMR3_GLOBAL_IRQ 1 */
}

/**
  * @brief  this function handles TMR4 handler.
  * @param  none
  * @retval none
  */
void TMR4_GLOBAL_IRQHandler(void)
{
  /* add user code begin TMR4_GLOBAL_IRQ 0 */

  /* add user code end TMR4_GLOBAL_IRQ 0 */

  /* overflow interrupt management */
  if(tmr_interrupt_flag_get(TMR4, TMR_OVF_FLAG) != RESET)
  {
    /* add user code begin TMR4_TMR_OVF_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR4, TMR_OVF_FLAG);
    /* moter42 双电机步进驱动：单一 Tick 同时驱动电机1(TMR4 CH1/CH2)
     * 与电机2(TMR5 CH1/CH2)，电机2相位自动反向。
     * 注意：moter57 与 moter42 共用 TMR4 定时器，不能同时使用。 */
    StepperPulse42_Tick();
    StepperPulse57_8412_Tick();

    /* add user code end TMR4_TMR_OVF_FLAG */
  }

  /* add user code begin TMR4_GLOBAL_IRQ 1 */

  /* add user code end TMR4_GLOBAL_IRQ 1 */
}

/**
  * @brief  this function handles USART3 handler.
  * @param  none
  * @retval none
  */
void USART3_IRQHandler(void)
{
  /* add user code begin USART3_IRQ 0 */
  /* add user code end USART3_IRQ 0 */

  if(usart_interrupt_flag_get(USART3, USART_IDLEF_FLAG) != RESET)
  {
    /* add user code begin USART3_USART_IDLEF_FLAG */
    /* IDLE flag is cleared INSIDE modbusUsart3IdleIsr(), AFTER DMA is
     * stopped. Clearing it here (which reads STS then DT) while DMA is
     * still running would steal a byte from the receive register and
     * corrupt the DMA buffer. V1 (NUVELL_V1) clears IDLE after stopping
     * DMA; we follow the same pattern. */
    modbusUsart3IdleIsr();
    /* add user code end USART3_USART_IDLEF_FLAG */ 
  }

  /* add user code begin USART3_IRQ 1 */

  /* add user code end USART3_IRQ 1 */
}

/**
  * @brief  this function handles EXINT Line [15:10] handler.
  * @param  none
  * @retval none
  */
void EXINT15_10_IRQHandler(void)
{
  /* add user code begin EXINT15_10_IRQ 0 */

  /* add user code end EXINT15_10_IRQ 0 */

  if(exint_interrupt_flag_get(EXINT_LINE_10) != RESET)
  {
    /* add user code begin EXINT_LINE_10 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_10);
    encRiseMoterREncExintIrq();
    /* add user code end EXINT_LINE_10 */ 
  }

  if(exint_interrupt_flag_get(EXINT_LINE_12) != RESET)
  {
    /* add user code begin EXINT_LINE_12 */
    /* clear flag */
    exint_flag_clear(EXINT_LINE_12);
    brineFTtick();
    /* add user code end EXINT_LINE_12 */ 
  }

  /* add user code begin EXINT15_10_IRQ 1 */

  /* add user code end EXINT15_10_IRQ 1 */
}

/**
  * @brief  this function handles TMR5 handler.
  * @param  none
  * @retval none
  */
void TMR5_GLOBAL_IRQHandler(void)
{
  /* add user code begin TMR5_GLOBAL_IRQ 0 */

  /* add user code end TMR5_GLOBAL_IRQ 0 */

  /* overflow interrupt management */
  if(tmr_interrupt_flag_get(TMR5, TMR_OVF_FLAG) != RESET)
  {
    /* add user code begin TMR5_TMR_OVF_FLAG */
    /* clear flag */
    tmr_flag_clear(TMR5, TMR_OVF_FLAG);
    /* moter42 电机2 的 PWM 由 TMR4 中断中的 StepperPulse42_Tick() 统一驱动；
     * wk_tmr5_init 已关闭本 OVF 中断（TMR5 为 TMR4 从机，无需独立时基），此分支仅为兜底。 */
    /* add user code end TMR5_TMR_OVF_FLAG */
  }

  /* add user code begin TMR5_GLOBAL_IRQ 1 */

  /* add user code end TMR5_GLOBAL_IRQ 1 */
}

/**
  * @brief  this function handles UART5 handler.
  * @param  none
  * @retval none
  */
void UART5_IRQHandler(void)
{
  /* add user code begin UART5_IRQ 0 */

  /* add user code end UART5_IRQ 0 */

  if(usart_interrupt_flag_get(UART5, USART_IDLEF_FLAG) != RESET)
  {
    /* add user code begin UART5_USART_IDLEF_FLAG */
    /* IDLE flag is cleared INSIDE usrDebugUart5IdleIsr(), AFTER DMA is
     * stopped. Clearing it here (which reads STS then DT) while DMA is
     * still running would steal a byte from the receive register and
     * corrupt the DMA buffer. 与 modbusUsart3IdleIsr() 的处理顺序一致。 */
    usrDebugUart5IdleIsr();
    /* add user code end UART5_USART_IDLEF_FLAG */ 
  }

  /* add user code begin UART5_IRQ 1 */

  /* add user code end UART5_IRQ 1 */
}

/* add user code begin 1 */

/* add user code end 1 */
