/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AT8236.h"
#include "ws2812.h"
#include "tmux1308.h"
#include "Current.h"
#include "BusVol.h"
#include "Tempurature.h"
#include "stepMoter.h"
#include "StepperPulse57_8412.h"
#include "uln2803.h"
#include "memoryPool.h"
#include "PulseTurns.h"
#include "taskScheme.h"
#include "crc16.h"
#include "string.h"
#include "stdio.h"
#include "servo.h"
#include "led.h"
#include "moveFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

volatile	uint32_t timeTick=0;
volatile  uint32_t task1msTick=0;
volatile  uint32_t task10msTick=0;
volatile  uint32_t task30msTick=0;
volatile  uint32_t task50msTick=0;
volatile  uint32_t task200msTick=0;
volatile  uint32_t task1000msTick=0;
volatile 	float current;
volatile 	float busvol;
volatile 	float temper; 
volatile 	float testTurns;

extern uint8_t highBroadOrder[2];
extern uint8_t lowBroadOrder[6];
extern uint8_t recDateBuffer[10];

static void parseParam(uint8_t *data, CommandParam_t *param);

extern uint8_t recDate[10];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//uint8_t  testDate[]={0x34,0x45,0x56,0x23,0x34,0x45,0x56,0x23,0x34,0x45,0x56,0x23};


static void task1ms(void);
static void task10ms(void);
static void task30ms(void);
static void task50ms(void);
static void task200ms(void);
static void task1000ms(void);
static void highVolBroadDateReTrans(void);
int ishighVolData=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
   
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
	
	if (HAL_SYSTICK_Config(SystemCoreClock / 1000) != HAL_OK)
	{
			Error_Handler();
	}
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
	pulseTurnsInit();
	
	WS2812_Init();
	WS2812_SetColor(0,255,255,255);
	WS2812_SetColor(1,255,255,255);
	WS2812_SetColor(2,255,255,255);
	WS2812_SetColor(3,255,255,255);
	WS2812_Show();	
	
	HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,0);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	HAL_GPIO_WritePin(stepMoter1Rst_GPIO_Port, stepMoter1Rst_Pin, 1);

	HAL_UART_Receive_IT(&huart1, recDate, sizeof(recDate));
	HAL_UART_Receive_IT(&huart3, highBroadOrder, sizeof(highBroadOrder));
	
	HAL_TIM_Base_Start_IT(&htim4); 
	stepMoterStart(); /* 内部已调 StepperPulse57_Init() */
	moter28Start();
	brinePumpOff();
	hosePumpOff();
	waterPumpOff();
	riseMoterDown(); 
	valveOff();
	stepMoterStop();
	ledOn();
	HAL_Delay(10000);
	versionSync();
	//stepMoterUp(150); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	StepperPulse57_8412_RunAtSpeed(3000, 80, 20000, 15);
  servoUp(1000);
	
  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1ms 高优先级任务：独立判断，确保主机指令延迟 */
    if(timeTick - task1msTick >= 1)
    {
        task1msTick = timeTick;  // 注意：这里记录的是执行时刻，不是"下一次到期时"
        // 执行 1ms 主机指令（这里务必保证代码执行时
        //�?关阀�?关检�?
				task1ms();
    }

    /*  1ms 任务：每次主循环只执行其中最近到期的 */
    uint32_t minDiff = 0xFFFFFFFF;
    uint8_t selectedTask = 0;
    
    // 找出有到期任务中
    if(timeTick - task10msTick >= 10)   { minDiff = timeTick - task10msTick; selectedTask = 1; }
    if(timeTick - task30msTick >= 30)   { if((timeTick - task30msTick) < minDiff) { minDiff = timeTick - task30msTick; selectedTask = 2; }}
    if(timeTick - task50msTick >= 50)   { if((timeTick - task50msTick) < minDiff) { minDiff = timeTick - task50msTick; selectedTask = 3; }}
    if(timeTick - task200msTick >= 200) { if((timeTick - task200msTick) < minDiff) { minDiff = timeTick - task200msTick; selectedTask = 4; }}
		if(timeTick - task1000msTick>= 1000){ if((timeTick - task1000msTick)< minDiff) { minDiff = timeTick - task1000msTick; selectedTask = 5; }}

    switch(selectedTask)
    {
        case 1: task10msTick = timeTick; 		
								task10ms();
								break;
        case 2: task30msTick = timeTick; /* 20ms 任务 */   
								task30ms();
								break;
        case 3: task50msTick = timeTick; /* 50ms 任务 */ 
								task50ms();								
								break;
        case 4: task200msTick = timeTick; /* 200ms 任务 */ 
								task200ms();					
								break;
        case 5: task1000msTick = timeTick; /* 1000ms 任务 */ 
								task1000ms();					
								break;
        default: break; // 没有任务到期
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void ledTest(void)
{
    static uint8_t i = 0;

    uint8_t color[4][3] =
    {
       {255,0,0},{0,255,0},{0,0,255},{255,255,255}
    };

    for(uint8_t j = 0; j < 5; j++)
    {
      WS2812_SetColor(j,color[i][0],color[i][1],color[i][2]);
    }
    WS2812_Show();
    i++;
    if(i >= 4)
    {
       i = 0;
    }
}
int servoCnt=0;
int moter28Dir=0;
int pumpCnt;
int aliveTick=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance==TIM2)
    {
        moter28step(moter28Dir);
    }
    if(htim->Instance==TIM3)
    {
        StepperPulse57_8412_Tick();
    }
    if(htim->Instance==TIM4)
    {
			aliveTick++;
			if(aliveTick<100)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
			}
			else if((aliveTick>=100)&&aliveTick<=200)
			{
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);				
			}
			else if(aliveTick>200)
			{
				aliveTick=0;
			}
				if(hosePumpFlag==1)
				{
					pumpCnt++;
					if(pumpCnt>3000)
					{
						pumpCnt=0;
						TIM4->CCR3--;
						if(TIM4->CCR3<hosePumpOnSpeed)
						{
							hosePumpFlag=0;
						}
					}
				}
#if ServoRise==1
        servoCnt++;

        /* 浮点渐变：每次回调推进servoStepSize */
        if(servoCurCnt < servoDesCnt) {
            servoCurCnt += servoStepSize;
            if(servoCurCnt > servoDesCnt) servoCurCnt = servoDesCnt;  /* 防止过冲 */
        } else if(servoCurCnt > servoDesCnt) {
            servoCurCnt -= servoStepSize;
            if(servoCurCnt < servoDesCnt) servoCurCnt = servoDesCnt;  /* 防止过冲 */
        }

        /* PWM输出取整数部�? */
        if(servoCnt<200&&servoCnt>(int)servoCurCnt)
        {
            HAL_GPIO_WritePin(Servo_GPIO_Port,Servo_Pin,GPIO_PIN_RESET);
        }
        if(servoCnt>199)
        {
            HAL_GPIO_WritePin(Servo_GPIO_Port,Servo_Pin,GPIO_PIN_SET);
            servoCnt=0;
        }
#else
			riseTick();
#endif
    }
}

void highVolBroadDateReTrans(void)
{
	static int reTransNum=3;
	if(ishighVolData==1)
	{
		HAL_UART_Transmit(&huart3,&highBroadOrder[0],sizeof(char),0x0f);
		reTransNum--;
	}
	if(reTransNum<0)
	{
		reTransNum=3;
		ishighVolData=0;
	}
}

void highVolBroad(void)
{
	highBroadOrder[0]=lowBroadOrder[1];
	highBroadOrder[1]=lowBroadOrder[2];
	HAL_UART_Transmit(&huart3,&highBroadOrder[0],sizeof(char),0x0f);
	ishighVolData=1;
}
static void parseParam(uint8_t *data, CommandParam_t *param)
{
    for(int i = 0; i < 8; i++)
    {
        param->para[i] = data[i + 1];
    }
}

void task1ms(void)
{
	
	//�?关阀是否到位
} 
void task10ms(void)
{
		CommandParam_t param;
		parseParam(lowBroadOrder, &param);
		commandExecute(lowBroadOrder[0], &param);
		memset(lowBroadOrder,0x00,sizeof(lowBroadOrder));
		StepperPulse57_8412_RampUpdate();
}
void task30ms(void)
{
		highVolBroadDateReTrans();
		getTmux1308Data(); 
		dataPoolUpdate();	
	
		valveDect();
		dataSync();
		riseRunDec();
}
void task50ms(void)
{   
		pulseTurnsGet();
}

void task200ms(void)
{
	//ledTest();
	temper=filter(calTemp());

}

int i=0;
void task1000ms(void)
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
