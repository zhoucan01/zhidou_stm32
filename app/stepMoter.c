#include "stepMoter.h"
#include "StepperPulse57_8412.h"
#include "main.h"

void stepMoterStart(void)
{
	StepperPulse57_8412_Init();
}

void stepMoterStop(void)
{
	StepperPulse57_8412_Stop();
	HAL_GPIO_WritePin(stepMoter1Rst_GPIO_Port, stepMoter1Rst_Pin, GPIO_PIN_RESET);
}


int speed_up,speed_down;
int step_count_low,step_motor_high;
int step_motor_up;
void stepMoterUp(float speed)
{
	speed_up++;
	step_motor_up++;
	HAL_GPIO_WritePin(stepMoter1Rst_GPIO_Port, stepMoter1Rst_Pin, GPIO_PIN_SET);
	/* speed 参数暂未使用，默认 55RPM，可按需修改 */
	
	if(speed>=0&&speed<100)//低速大电流
	{
		step_count_low++;
		 StepperPulse57_8412_RunAtSpeed(speed, 50, 2000, 0);
	}
	else if(speed>=100&&speed<=200)//高速小电流
	{
		step_motor_high++;
		 StepperPulse57_8412_RunAtSpeed(speed*2*10, 50, 2000, 0);
	}	
	else 
	{
		StepperPulse57_8412_RunAtSpeed(300, 50, 2000, 0);
	}
	
//		StepperPulse57_8412_RunAtSpeed(speed, 20, 200, 0);

}
volatile int tempStepMoterSpd;
void stepMoterDown(float speed)
{
	speed_down=speed;
	tempStepMoterSpd = speed;
	HAL_GPIO_WritePin(stepMoter1Rst_GPIO_Port, stepMoter1Rst_Pin, GPIO_PIN_SET);
	if(speed>=0&&speed<100)//低速大电流
	{
		step_count_low++;
		 StepperPulse57_8412_RunAtSpeed(-speed, 50, 2000, 0);
	}
	else if(speed>=100&&speed<=200)//高速小电流
	{
		step_motor_high++;
		 StepperPulse57_8412_RunAtSpeed(-speed*2*10, 50, 2000, 0);
	}	
	else 
	{
		StepperPulse57_8412_RunAtSpeed(-2000, 50, 2000, 0);
	}
}
 



