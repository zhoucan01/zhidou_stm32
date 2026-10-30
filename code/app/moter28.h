#ifndef MOTER28_H
#define MOTER28_H
/* 28-step motor and valve control public API. */

#include "stdbool.h"

extern bool valveOnPos;//阀门开位
extern bool valveOffPos;//阀门关位

#include "stdbool.h"

void moter28Stop(void);//28步进停止
bool moter28RunStep(void);//28步进运行
void moter28Forward(void);//28步进正转
void moter28Backward(void);//28步进反转
bool getMoter28Fault(void);//获取28步进是否故障
void valveOn(void);//开阀门
void valveOff(void);//关阀门



#endif // MOTER28_H

