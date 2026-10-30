/* add user code begin Header */
/**
  **************************************************************************
  * @file     at32f415_wk_config.h
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
#ifndef __AT32F415_WK_CONFIG_H
#define __AT32F415_WK_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* includes -----------------------------------------------------------------------*/
#include "stdio.h"
#include "at32f415.h"

/* private includes -------------------------------------------------------------*/
/* add user code begin private includes */

/* add user code end private includes */

/* exported types -------------------------------------------------------------*/
/* add user code begin exported types */

/* add user code end exported types */

/* exported constants --------------------------------------------------------*/
/* add user code begin exported constants */

/* add user code end exported constants */

/* exported macro ------------------------------------------------------------*/
/* add user code begin exported macro */

/* add user code end exported macro */

/* add user code begin dma define */
/* user can only modify the dma define value */
#define DMA1_CHANNEL1_BUFFER_SIZE   0
#define DMA1_CHANNEL1_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL1_PERIPHERAL_BASE_ADDR  0

#define DMA1_CHANNEL2_BUFFER_SIZE   0
#define DMA1_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL2_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL3_BUFFER_SIZE   0
//#define DMA1_CHANNEL3_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL3_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL4_BUFFER_SIZE   0
//#define DMA1_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL4_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL5_BUFFER_SIZE   0
//#define DMA1_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL6_BUFFER_SIZE   0
//#define DMA1_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA1_CHANNEL7_BUFFER_SIZE   0
//#define DMA1_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA1_CHANNEL7_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL1_BUFFER_SIZE   0
//#define DMA2_CHANNEL1_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL1_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL2_BUFFER_SIZE   0
//#define DMA2_CHANNEL2_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL2_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL3_BUFFER_SIZE   0
//#define DMA2_CHANNEL3_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL3_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL4_BUFFER_SIZE   0
//#define DMA2_CHANNEL4_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL4_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL5_BUFFER_SIZE   0
//#define DMA2_CHANNEL5_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL5_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL6_BUFFER_SIZE   0
//#define DMA2_CHANNEL6_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL6_PERIPHERAL_BASE_ADDR   0

//#define DMA2_CHANNEL7_BUFFER_SIZE   0
//#define DMA2_CHANNEL7_MEMORY_BASE_ADDR   0
//#define DMA2_CHANNEL7_PERIPHERAL_BASE_ADDR   0
/* add user code end dma define */

/* Private defines -------------------------------------------------------------*/
#define vavleRST_PIN    GPIO_PINS_13
#define vavleRST_GPIO_PORT    GPIOC
#define moterStop_PIN    GPIO_PINS_14
#define moterStop_GPIO_PORT    GPIOC
#define LED1_PIN    GPIO_PINS_15
#define LED1_GPIO_PORT    GPIOC
#define moterAndLED_PIN    GPIO_PINS_0
#define moterAndLED_GPIO_PORT    GPIOC
#define pressMoterCurrA_PIN    GPIO_PINS_1
#define pressMoterCurrA_GPIO_PORT    GPIOC
#define pressMoterCurrB_PIN    GPIO_PINS_2
#define pressMoterCurrB_GPIO_PORT    GPIOC
#define tmux1308ADCData1_PIN    GPIO_PINS_3
#define tmux1308ADCData1_GPIO_PORT    GPIOC
#define hosePumpPWMA_PIN    GPIO_PINS_0
#define hosePumpPWMA_GPIO_PORT    GPIOA
#define hosePumpPWMB_PIN    GPIO_PINS_1
#define hosePumpPWMB_GPIO_PORT    GPIOA
#define waterPumpPWM_PIN    GPIO_PINS_2
#define waterPumpPWM_GPIO_PORT    GPIOA
#define brinePumpPWM_PIN    GPIO_PINS_3
#define brinePumpPWM_GPIO_PORT    GPIOA
#define riseMoterRPWMA_PIN    GPIO_PINS_4
#define riseMoterRPWMA_GPIO_PORT    GPIOF
#define riseMoterRPWMB_PIN    GPIO_PINS_5
#define riseMoterRPWMB_GPIO_PORT    GPIOF
#define busCurr_PIN    GPIO_PINS_4
#define busCurr_GPIO_PORT    GPIOA
#define busVol_PIN    GPIO_PINS_5
#define busVol_GPIO_PORT    GPIOA
#define valveGPIOA_PIN    GPIO_PINS_6
#define valveGPIOA_GPIO_PORT    GPIOA
#define valveGPIOB_PIN    GPIO_PINS_7
#define valveGPIOB_GPIO_PORT    GPIOA
#define tmux1308ADCData2_PIN    GPIO_PINS_4
#define tmux1308ADCData2_GPIO_PORT    GPIOC
#define waterTankFT_PIN    GPIO_PINS_5
#define waterTankFT_GPIO_PORT    GPIOC
#define strainGage_PIN    GPIO_PINS_0
#define strainGage_GPIO_PORT    GPIOB
#define USART3SEL_PIN    GPIO_PINS_2
#define USART3SEL_GPIO_PORT    GPIOB
#define pressHighPos_PIN    GPIO_PINS_12
#define pressHighPos_GPIO_PORT    GPIOB
#define tmux1308IOData0_PIN    GPIO_PINS_13
#define tmux1308IOData0_GPIO_PORT    GPIOB
#define valveGPIOD_PIN    GPIO_PINS_14
#define valveGPIOD_GPIO_PORT    GPIOB
#define valveGPIOC_PIN    GPIO_PINS_15
#define valveGPIOC_GPIO_PORT    GPIOB
#define pressMoterEncB_PIN    GPIO_PINS_8
#define pressMoterEncB_GPIO_PORT    GPIOC
#define pressMoterEncA_PIN    GPIO_PINS_9
#define pressMoterEncA_GPIO_PORT    GPIOC
#define riseMoterLEncB_PIN    GPIO_PINS_8
#define riseMoterLEncB_GPIO_PORT    GPIOA
#define riseMoterLEncA_PIN    GPIO_PINS_9
#define riseMoterLEncA_GPIO_PORT    GPIOA
#define riseMoterREncB_PIN    GPIO_PINS_10
#define riseMoterREncB_GPIO_PORT    GPIOA
#define riseMoterREncA_PIN    GPIO_PINS_11
#define riseMoterREncA_GPIO_PORT    GPIOA
#define brineFT_PIN    GPIO_PINS_12
#define brineFT_GPIO_PORT    GPIOA
#define riseMoterRPosL_PIN    GPIO_PINS_6
#define riseMoterRPosL_GPIO_PORT    GPIOF
#define valveOFFPos_PIN    GPIO_PINS_7
#define valveOFFPos_GPIO_PORT    GPIOF
#define riseMoterLPosL_PIN    GPIO_PINS_15
#define riseMoterLPosL_GPIO_PORT    GPIOA
#define moterRST_PIN    GPIO_PINS_10
#define moterRST_GPIO_PORT    GPIOC
#define tmux1308A0_PIN    GPIO_PINS_11
#define tmux1308A0_GPIO_PORT    GPIOC
#define tmux1308A1_PIN    GPIO_PINS_3
#define tmux1308A1_GPIO_PORT    GPIOB
#define tmux1308A2_PIN    GPIO_PINS_4
#define tmux1308A2_GPIO_PORT    GPIOB
#define riseMoterLPWMB_PIN    GPIO_PINS_6
#define riseMoterLPWMB_GPIO_PORT    GPIOB
#define riseMoterLPWMA_PIN    GPIO_PINS_7
#define riseMoterLPWMA_GPIO_PORT    GPIOB
#define pressMoterPWMB_PIN    GPIO_PINS_8
#define pressMoterPWMB_GPIO_PORT    GPIOB
#define pressMoterPWMA_PIN    GPIO_PINS_9
#define pressMoterPWMA_GPIO_PORT    GPIOB

/* exported functions ------------------------------------------------------- */
  /* system clock config. */
  void wk_system_clock_config(void);

  /* config periph clock. */
  void wk_periph_clock_config(void);

  /* nvic config. */
  void wk_nvic_config(void);

/* add user code begin exported functions */

/* add user code end exported functions */

#ifdef __cplusplus
}
#endif

#endif
