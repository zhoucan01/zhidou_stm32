/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : freertos_app.c
  * Description        : Code for freertos applications
  */
/* add user code end Header */

/* Includes ------------------------------------------------------------------*/
#include "freertos_app.h"

/* private includes ----------------------------------------------------------*/
/* add user code begin private includes */
#include "../../task/modbusTask.h"
#include "../../task/actionRun.h"
#include "../../app/usrDebug.h"
#include "../inc/log.h"
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

/* task handler */
TaskHandle_t modbusTask_handle;
TaskHandle_t getState_handle;
TaskHandle_t actionRunTask_handle;
TaskHandle_t usrDubug_handle;

/* queue handler */
QueueHandle_t modbusCMD_handle;
QueueHandle_t modbusData_handle;
QueueHandle_t usrDebugData_handle;

/* Idle task control block and stack */
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

static StaticTask_t idle_task_tcb;
static StaticTask_t timer_task_tcb;

/* External Idle and Timer task static memory allocation functions */
extern void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );
extern void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize );

/*
  vApplicationGetIdleTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
  equals to 1 and is required for static memory allocation support.
*/
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer, StackType_t ** ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &idle_task_tcb;
  *ppxIdleTaskStackBuffer = &idle_task_stack[0];
  *pulIdleTaskStackSize = (uint32_t)configMINIMAL_STACK_SIZE;
}
/*
  vApplicationGetTimerTaskMemory gets called when configSUPPORT_STATIC_ALLOCATION
  equals to 1 and is required for static memory allocation support.
*/
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer, StackType_t ** ppxTimerTaskStackBuffer, uint32_t * pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &timer_task_tcb;
  *ppxTimerTaskStackBuffer = &timer_task_stack[0];
  *pulTimerTaskStackSize = (uint32_t)configTIMER_TASK_STACK_DEPTH;
}

void vApplicationMallocFailedHook(void)
{
  /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */

/* add user code begin vApplicationMallocFailedHook */

/* add user code end vApplicationMallocFailedHook */
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */

/* add user code begin vApplicationStackOverflowHook */
  (void)xTask;
  (void)pcTaskName;
  /* 栈溢出时不做任何事，避免在 ISR 上下文调用 printf 导致死锁。 */
/* add user code end vApplicationStackOverflowHook */
}

/* add user code begin 1 */

/* add user code end 1 */

/**
  * @brief  initializes all task.
  * @param  none
  * @retval none
  */
void freertos_task_create(void)
{
  /* create modbusTask task */
  xTaskCreate(my_task01_func,
              "modbusTask",
              1024,
              NULL,
              10,
              &modbusTask_handle);

  /* create getState task */
  xTaskCreate(my_task02_func,
              "getState",
              256,
              NULL,
              8,
              &getState_handle);

  /* create actionRunTask task */
  xTaskCreate(my_task03_func,
              "actionRunTask",
              256,
              NULL,
              6,
              &actionRunTask_handle);

  /* create usrDubug task. 栈 512 字=2048 字节：usrDebugAtPoll 内局部变量
   * 已全部改为 static，栈消耗主要来自 printf 内部缓冲。
   * 优先级提高到 9（高于 getState 的 8），避免被频繁抢占导致 printf 卡死。 */
  xTaskCreate(my_task04_func,
              "usrDubug",
              512,
              NULL,
              9,
              &usrDubug_handle);
}

/**
  * @brief  initializes all queue.
  * @param  none
  * @retval none
  */
void freertos_queue_create(void)
{
  /* Create the modbusCMD, storing the returned handle in the xQueue variable. */
  modbusCMD_handle = xQueueCreate(8, sizeof(uint16_t));

  /* Create the modbusData, storing the returned handle in the xQueue variable. */
  modbusData_handle = xQueueCreate(1000, sizeof(uint16_t));

  /* Create the usrDebugData, storing the returned handle in the xQueue variable. */
  usrDebugData_handle = xQueueCreate(30, sizeof(uint16_t));
}

/**
  * @brief  freertos init and begin run.
  * @param  none
  * @retval none
  */
void wk_freertos_init(void)
{
  /* add user code begin freertos_init 0 */

  /* add user code end freertos_init 0 */

  /* enter critical */
  taskENTER_CRITICAL();

  freertos_queue_create();
  freertos_task_create();
	
  /* add user code begin freertos_init 1 */

  /* add user code end freertos_init 1 */

  /* exit critical */
  taskEXIT_CRITICAL();

  /* start scheduler */
  vTaskStartScheduler();
}

/**
  * @brief modbusTask function.
  * @param  none
  * @retval none
  */
__WEAK void my_task01_func(void *pvParameters)
{
  /* add user code begin my_task01_func 0 */

  /* add user code end my_task01_func 0 */

  /* add user code begin my_task01_func 2 */

  /* add user code end my_task01_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin my_task01_func 1 */

     /* 屏幕通信协议轮询：拆帧入队 + 命令执行 + 30ms 周期上报 */
     /* USART3 / RS485 Modbus RTU: process complete DMA/IDLE frames.
      * The legacy UART5 screen protocol is no longer serviced here. */
     Modbus_Poll();
     vTaskDelay(1);

  /* add user code end my_task01_func 1 */
  }
}

/**
  * @brief getState function.
  * @param  none
  * @retval none
  */
__WEAK void my_task02_func(void *pvParameters)
{
  /* add user code begin my_task02_func 0 */

  /* add user code end my_task02_func 0 */

  /* add user code begin my_task02_func 2 */

  /* add user code end my_task02_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin my_task02_func 1 */

     vTaskDelay(1);

  /* add user code end my_task02_func 1 */
  }
}

/**
  * @brief actionRunTask function.
  * @param  none
  * @retval none
  */
__WEAK void my_task03_func(void *pvParameters)
{
  /* add user code begin my_task03_func 0 */

  /* add user code end my_task03_func 0 */

  /* add user code begin my_task03_func 2 */

  /* add user code end my_task03_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin my_task03_func 1 */

     vTaskDelay(1);

  /* add user code end my_task03_func 1 */
  }
}

/**
  * @brief usrDubug function.
  * @param  none
  * @retval none
  */
__WEAK void my_task04_func(void *pvParameters)
{
  /* add user code begin my_task04_func 0 */
  log_d("task04 started");
  /* add user code end my_task04_func 0 */

  /* add user code begin my_task04_func 2 */

  /* add user code end my_task04_func 2 */

  /* Infinite loop */
  while(1)
  {
  /* add user code begin my_task04_func 1 */
    /* 诊断：第一次进入无条件打印，之后每 10 秒打印一次。
       所有变量 static，避免栈消耗。 */
    {
      static uint8_t diagFirst = 1U;
      static uint32_t diagLast = 0U;
      static uint32_t now;
      now = (uint32_t)xTaskGetTickCount();
      if(diagFirst != 0U || (now - diagLast) >= 10000U)
      {
        diagFirst = 0U;
        diagLast = now;
        log_d("task04 alive t=%lu", (unsigned long)now);
      }
    }

    /* UART5 AT 命令由专用 usrDubug 任务消费，避免占用 getState 的
       20 ms 传感器刷新周期。 */
    usrDebugAtPoll();
    vTaskDelay(pdMS_TO_TICKS(10U));
  /* add user code end my_task04_func 1 */
  }
}

/* add user code begin 2 */

/* add user code end 2 */

