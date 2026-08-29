/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define clearWaterFT_Pin GPIO_PIN_13
#define clearWaterFT_GPIO_Port GPIOC
#define clearWaterFT_EXTI_IRQn EXTI15_10_IRQn
#define boxAvail_Pin GPIO_PIN_14
#define boxAvail_GPIO_Port GPIOC
#define power_Pin GPIO_PIN_15
#define power_GPIO_Port GPIOC
#define power_EXTI_IRQn EXTI15_10_IRQn
#define Servo_Pin GPIO_PIN_2
#define Servo_GPIO_Port GPIOA
#define busCurr_Pin GPIO_PIN_4
#define busCurr_GPIO_Port GPIOA
#define tmux1308D2_Pin GPIO_PIN_5
#define tmux1308D2_GPIO_Port GPIOA
#define stepMoterA_Pin GPIO_PIN_6
#define stepMoterA_GPIO_Port GPIOA
#define stepMoterB_Pin GPIO_PIN_7
#define stepMoterB_GPIO_Port GPIOA
#define waterPumpPwm_Pin GPIO_PIN_0
#define waterPumpPwm_GPIO_Port GPIOB
#define waterPumpIo_Pin GPIO_PIN_1
#define waterPumpIo_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define tmux1308A2_Pin GPIO_PIN_12
#define tmux1308A2_GPIO_Port GPIOB
#define tmux1308A1_Pin GPIO_PIN_13
#define tmux1308A1_GPIO_Port GPIOB
#define tmux1308A0_Pin GPIO_PIN_14
#define tmux1308A0_GPIO_Port GPIOB
#define tmux1308D1_Pin GPIO_PIN_15
#define tmux1308D1_GPIO_Port GPIOB
#define tmux1308D0_Pin GPIO_PIN_12
#define tmux1308D0_GPIO_Port GPIOA
#define stepMoter1Rst_Pin GPIO_PIN_15
#define stepMoter1Rst_GPIO_Port GPIOA
#define beep_Pin GPIO_PIN_4
#define beep_GPIO_Port GPIOB
#define drinePumpA_Pin GPIO_PIN_6
#define drinePumpA_GPIO_Port GPIOB
#define drinePumpB_Pin GPIO_PIN_7
#define drinePumpB_GPIO_Port GPIOB
#define hosePumpPwm_Pin GPIO_PIN_8
#define hosePumpPwm_GPIO_Port GPIOB
#define hosePumpIo_Pin GPIO_PIN_9
#define hosePumpIo_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void highVolBroad(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
