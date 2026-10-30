#include "getState.h"

/* 周期状态刷新任务：固定每 20 ms 更新全部传感器缓存。 */
#include "freertos_app.h"
#include "AT8236.h"
#include "current.h"
#include "getBusVol.h"
#include "tmux1308.h"
#include "wk_debug.h"

/**
  * @brief getState function.
  * @param  none
  * @retval none
  */
/** FreeRTOS 状态任务；Modbus 仅读取其缓存，不在通信路径中扫描 ADC。 */
void my_task02_func(void *pvParameters)
{
  TickType_t lastWakeTime;
  uint8_t pumpTick;
  uint16_t heartbeatTick;
  /* add user code begin my_task02_func 0 */

  /* add user code end my_task02_func 0 */

  /* add user code begin my_task02_func 2 */

  /* add user code end my_task02_func 2 */

  (void)pvParameters;
  lastWakeTime = xTaskGetTickCount();
  heartbeatTick = 0U;

  /* Refresh the cache consumed by Modbus input registers every 20 ms.
   * The old UART5 poll also advanced pump ramps every 1 ms, so perform the
   * equivalent 20 ramp steps here while UART5 remains disabled.
   * 每 20 ms 刷新一次传感器缓存；每 250 个周期（5 秒）通过 UART5 打印一次心跳。 */
  while(1)
  {
    getTMUX1308Data();
    currentUpdateAll();
    getBusVoltage();
    for(pumpTick = 0U; pumpTick < 20U; pumpTick++)
    {
      pumpDecSpdTick();
    }
    /* 心跳打印：每 250 个周期（250×20ms = 5s）通过 UART5 输出一次存活信号。 */
    heartbeatTick++;
    if(heartbeatTick >= 250U)
    {
      heartbeatTick = 0U;
      wk_debug_print_heartbeat();
    }
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(20U));
  }
}
