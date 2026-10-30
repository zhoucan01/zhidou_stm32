#include "actionRun.h"

/* 延后动作执行器：在单独任务中串行执行非 Modbus 的执行器调用。 */
#include "freertos_app.h"
#include "moter28.h"
#include "moter42.h"
#include "moter57.h"

/** 将执行器命令非阻塞地投递到动作任务队列。 */
void actionRunSubmit(const action_command_t *command)
{
  if(command != NULL) (void)xQueueSend(modbusCMD_handle, command, 0U);
}

/** 解析一条队列动作命令，并调用对应的电机操作。 */
static void actionRunExecute(const action_command_t *c)
{
  switch(c->id)
  {
    case ACTION_57_RUN: StepperPulse57_8412_RunAtSpeed((signed int)c->value0, (unsigned char)c->value1, c->value2, c->value3); break;
    case ACTION_57_STOP: StepperPulse57_8412_Stop(); break;
    case ACTION_42_RUN: StepperPulse42_Run((signed int)c->value0, (unsigned char)c->value1); break;
    case ACTION_42_STOP: StepperPulse42_Stop(); break;
    case ACTION_28_FORWARD: moter28Forward(); break;
    case ACTION_28_BACKWARD: moter28Backward(); break;
    case ACTION_28_STOP: moter28Stop(); break;
    default: break;
  }
}

/**
  * @brief actionRunTask function.
  * @param  none
  * @retval none
  */
/** FreeRTOS 动作任务：阻塞等待命令，收到后立即执行。 */
void my_task03_func(void *pvParameters)
{
  /* add user code begin my_task03_func 0 */

  /* add user code end my_task03_func 0 */

  /* add user code begin my_task03_func 2 */

  /* add user code end my_task03_func 2 */

  action_command_t command;
  while(1)
  {
    if(xQueueReceive(modbusCMD_handle, &command, portMAX_DELAY) == pdPASS)
      actionRunExecute(&command);
  }
}
