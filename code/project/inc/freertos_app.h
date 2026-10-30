/* add user code begin Header */
/**
  ******************************************************************************
  * File Name          : freertos_app.h
  * Description        : Code for freertos applications
  */
/* add user code end Header */
  
#ifndef FREERTOS_APP_H
#define FREERTOS_APP_H

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#include "event_groups.h"
#include "wk_system.h"

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

/* task handler */
extern TaskHandle_t modbusTask_handle;
extern TaskHandle_t getState_handle;
extern TaskHandle_t actionRunTask_handle;
extern TaskHandle_t usrDubug_handle;
/* declaration for task function */
void my_task01_func(void *pvParameters);
void my_task02_func(void *pvParameters);
void my_task03_func(void *pvParameters);
void my_task04_func(void *pvParameters);

/* queue handler */
extern QueueHandle_t modbusCMD_handle;
extern QueueHandle_t modbusData_handle;
extern QueueHandle_t usrDebugData_handle;

/* add user code begin 0 */

/* add user code end 0 */

void freertos_task_create(void);
void freertos_queue_create(void);
void wk_freertos_init(void);

/* add user code begin 1 */

/* add user code end 1 */

#endif /* FREERTOS_APP_H */
