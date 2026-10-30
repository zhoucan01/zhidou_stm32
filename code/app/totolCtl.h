#ifndef TOTOLCTL_H
#define TOTOLCTL_H
/* Shared motor-reset/enable control API. */

#include "stdbool.h"

bool getTotolRstLevel(void);

/* 电机复位脚 PC10（moterRST）控制：moterEN=高电平，moterDIS=低电平。
 * 两者都会维护内部状态并同步泵的公共输入电平。 */
void moterEN(void);
void moterDIS(void);

/* 按参数设置 PC10 电平（true=高，false=低），返回设置后的电平。
 * 电平未变化时不会重复操作引脚，供上位机命令使用。 */
bool moterRstSet(bool high);

#endif
