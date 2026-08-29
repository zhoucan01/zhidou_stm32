int servoDesCnt=13;            // 目标值（整数）
float servoCurCnt=13.0f;       // 当前值（浮点）
float servoStepSize=0.0f;      // 每次回调(0.1ms)步长，由时长反推
//15最高
//24最低
#include "servo.h"

/* TIM4回调频率10kHz(每0.1ms)，1秒=10000次回调 */
#define SERVO_TICKS_PER_SEC 10000.0f
int servoUpPos=6;
int servoLowPos=24;
void servoUp(float duration)
{
	servoDesCnt=servoUpPos;
	float diff=servoDesCnt-servoCurCnt;
	if(diff!=0.0f)
		servoStepSize=(diff<0?-diff:diff)/(duration*SERVO_TICKS_PER_SEC);
}

void servoDown(float duration)
{
	servoDesCnt=servoLowPos;
	float diff=servoDesCnt-servoCurCnt;
	if(diff!=0.0f)
		servoStepSize=(diff<0?-diff:diff)/(duration*SERVO_TICKS_PER_SEC);
}
