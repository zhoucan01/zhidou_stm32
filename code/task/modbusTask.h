#ifndef TASK_MODBUSTASK_H
#define TASK_MODBUSTASK_H
/* Modbus RTU slave initialization, ISR handoff, and task-facing API. */

#include "at32f415_wk_config.h"
#include <stdbool.h>

#define MODBUS_SLAVE_ADDRESS       2U
#define MODBUS_RX_BUFFER_SIZE    256U

/* 寄存器布局严格遵循 站号2_AT32F415CBT7_Modbus协议.md（所有地址为零基 PDU 地址）：
 * 线圈 0x0000~0x0009：清水泵/点卤泵/蠕动泵/阀门/LED/预留×3/压制命令状态/抬升命令状态。
 * 离散输入 0x0000~0x000F：抬升高位/抬升低位/压制高位/压制低位/压制头在位/水箱在位/
 *   水箱高液位/水箱低液位/废水箱在位/阀门开到位/阀门关到位/点卤盒在位/溢出报警/
 *   上盖在位/滤网在位/成型盒在位。
 * 保持寄存器 0x0000~0x0003：清水泵速度(0~1000)/蠕动泵速度(0~255)/压制电机int16/
 *   抬升电机int16（写 -1000 触发"下降一段"、写 -601 触发"上行一段"（段长都写在下位机））；
 *   0x0004~0x0066 为参数/预留区（RAM，上电清零）。
 * 输入寄存器 0x0000~0x0027（共 40）：温度/电压/电流float32/抬升设定值/圈数float32/
 *   设备状态字/故障码/固件版本/保留/离散位图/线圈位图/泵速镜像/队列深度/诊断计数uint32/
 *   TMUX D0/D1位图/TMUX ADC/板载电流ADC。
 * 输入寄存器 0x0028~0x004F（V2 追加）：多路电流float32/编码器位置int32/NTC温度/
 *   流量计脉冲/57绕组零点/ADC采集状态。
 * FC04 波形窗口 8000~16399：57 绕组 A/B 的 40 kHz ADC 波形（只读，不占用快照）。 */

/* Bind the Modbus RTU receive buffer to the already initialized USART3 DMA
 * channel, and keep RS485 RE/DE low. GPIO, USART, DMA mode and IRQ setup
 * remain owned by the configuration-tool BSP. Call once from main(). */
void modbusinit(void);

/* Send one complete RTU response. Before driving DE high the function waits
 * for the Modbus t3.5 turn-around delay and confirms the bus is idle (no
 * ongoing RX activity). RE/DE is released only after TDC and an additional
 * three character times, as required by the external transceiver. */
bool modbusRs485Send(const uint8_t *data, uint16_t length);

/* Call from the USART3 DMA + IDLE receive handler with newly received bytes.
 * IDLE segments are accumulated; only a valid-length RTU request is copied to
 * an internal slot and its uint16_t index is sent to modbusData_handle. */
void Modbus_RxIdleCallback(const uint8_t *data, uint16_t length);

/* Call from USART3_IRQHandler after clearing USART_IDLEF_FLAG. It consumes
 * newly received bytes from DMA1 channel1, which remains in circular mode. */
void modbusUsart3IdleIsr(void);

/* Call periodically from the Modbus task; validates and executes RTU frames. */
void Modbus_Poll(void);

/* 通过 RS485 口打印三个版本号（ASCII 文本 + 二进制版本帧），
 * 每次复位/上电调用一次，用于确认板子功能正常。 */
void modbusPrintVersion(void);

uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length);

extern volatile uint32_t modbusRxQueueOverflow;
extern volatile uint32_t modbusCrcErrorCount;
extern volatile uint32_t modbusFrameErrorCount;
/* CRC 校验通过的有效请求帧总数、实际发送的应答帧总数。
 * 供 UART5 调试口 NB+stat 指令周期性打印。 */
extern volatile uint32_t modbusRxTotalCount;
extern volatile uint32_t modbusReplyTotalCount;

#endif
