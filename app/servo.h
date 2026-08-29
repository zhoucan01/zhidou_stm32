#ifndef SERVO__H
#define SERVO__H

extern int servoDesCnt;      // 目标值（整数）
extern float servoCurCnt;    // 当前值（浮点）
extern float servoStepSize;  // 每次回调步长（由时长反推）

/* duration: 完成移动的秒数，如1.0=1秒 */
void servoUp(float duration);
void servoDown(float duration);


#endif
