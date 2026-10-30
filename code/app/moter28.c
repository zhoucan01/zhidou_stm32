#include "moter28.h"

/* 28 步进电机/阀门状态机：负责步进时序和故障保护。 */
#include "at32f415.h"
#include "at32f415_wk_config.h"
#include "tmux1308.h"

bool valveOnPos  = false;//阀门开位
bool valveOffPos = false;//阀门关位

static confirm_state moter28State[8][4] = {		//步进表
    {TRUE, FALSE, FALSE, FALSE},  //A+ B- C- D-
    {TRUE, TRUE, FALSE, FALSE},   //A+ B+ C- D-
    {FALSE, TRUE, FALSE, FALSE},  //A- B+ C- D-
    {FALSE, TRUE, TRUE, FALSE},   //A- B+ C+ D-
    {FALSE, FALSE, TRUE, FALSE},  //A- B- C+ D-
    {FALSE, FALSE, TRUE, TRUE},   //A- B- C+ D+
    {FALSE, FALSE, FALSE, TRUE},  //A- B- C- D+
    {TRUE, FALSE, FALSE, TRUE}    //A+ B- C- D+
};

 volatile uint8_t moter28Step = 0;  //当前步进位置，0～7
 volatile bool    moter28Dir= 0;      //方向 false正  true反
 volatile bool    moter28Run = false;  //运行状态，false停止，true运行

/* TMR10: 150 MHz / 12 = 12.5 MHz. Ramp over 2448 steps (<6 seconds). */
#define MOTER28_START_TICKS 50000U
#define MOTER28_FAST_TICKS  11250U
#define MOTER28_ACCEL_STEPS 2448U
static volatile uint32_t moter28AccelStep = 0;

static void moter28Start(bool direction)
{
    uint32_t interruptMask = __get_PRIMASK();
    __disable_irq();
    /* Repeated commands in the same direction keep the current speed. */
    if (!moter28Run || moter28Dir != direction)
    {
        moter28Run = false;
        tmr_counter_enable(TMR10, FALSE);
        moter28AccelStep = 0;
        tmr_period_value_set(TMR10, MOTER28_START_TICKS - 1U);
        tmr_counter_value_set(TMR10, 0);
        tmr_flag_clear(TMR10, TMR_OVF_FLAG);
        moter28Dir = direction;
        moter28Run = true;
        tmr_counter_enable(TMR10, TRUE);
    }
    __set_PRIMASK(interruptMask);
}
//28步进停止
void moter28Stop(void)
{
    moter28Run = false;
    gpio_bits_reset(valveGPIOA_GPIO_PORT, valveGPIOA_PIN);
    gpio_bits_reset(valveGPIOB_GPIO_PORT, valveGPIOB_PIN);
    gpio_bits_reset(valveGPIOC_GPIO_PORT, valveGPIOC_PIN);
    gpio_bits_reset(valveGPIOD_GPIO_PORT, valveGPIOD_PIN);
}

static int32_t totalTurns = 55000;//总的步数，采用这种驱动方式速度会慢一点，但是平滑，力大，一圈4096步，暂定20圈
 int32_t currTurns=0;//当前圈数
static bool valveOnFlag=false;//阀开检测
static bool valveOffFlag=false;//阀关检测
bool moter28RunStep(void)
{
    if (!moter28Run) 
    {
        return false;
    }	

    if (moter28Dir == 0) 
    {
        moter28Step++;
        if (moter28Step > 7) 
        {
            moter28Step = 0;
        }
        currTurns--;
    } 
    else 
    {
        if (moter28Step == 0) 
        {
            moter28Step = 7;
        } 
        else 
        {
            moter28Step--;
        }
        currTurns++;
    }

    if(currTurns>totalTurns)
    {
        valveOnPos = true;//阀门开位
    }
    else
    {
       valveOnPos = false;//阀门开位 
    }

	if(gpio_input_data_bit_read(valveOFFPos_GPIO_PORT,valveOFFPos_PIN)==RESET)
	{
        valveOffPos = true;//阀门关位  
    }
    else
    {
        valveOffPos = false;//阀门关位 
    }

    gpio_bits_write(valveGPIOA_GPIO_PORT, valveGPIOA_PIN, moter28State[moter28Step][0]);
    gpio_bits_write(valveGPIOB_GPIO_PORT, valveGPIOB_PIN, moter28State[moter28Step][1]);
    gpio_bits_write(valveGPIOD_GPIO_PORT, valveGPIOD_PIN, moter28State[moter28Step][2]);
    gpio_bits_write(valveGPIOC_GPIO_PORT, valveGPIOC_PIN, moter28State[moter28Step][3]);
	
	if(valveOnFlag)
	{
		if(currTurns>totalTurns)
		{
            valveOnPos = true;//阀门开位
			moter28Stop();
			valveOnFlag = false;
		}
        else
        {
            valveOnPos = false;//阀门开位
        }
	}
	if(valveOffFlag)
    {
		if(gpio_input_data_bit_read(valveOFFPos_GPIO_PORT,valveOFFPos_PIN)==RESET)
		{
            valveOffPos = true;//阀门关位
			moter28Stop();
			currTurns = 0;
			valveOffFlag = false;
		}
        else
        {
          valveOffPos = false;//阀门关位  
        }
	}
    if (moter28Run && moter28AccelStep < MOTER28_ACCEL_STEPS)
    {
        uint32_t ticks;
        moter28AccelStep++;
        ticks = MOTER28_START_TICKS -
                ((MOTER28_START_TICKS - MOTER28_FAST_TICKS) * moter28AccelStep /
                 MOTER28_ACCEL_STEPS);
        tmr_period_value_set(TMR10, ticks - 1U);
    }
    return true;
}

//28步进正转
void moter28Forward(void)
{
    moter28Start(false);
}
//28步进反转
void moter28Backward(void)
{
    moter28Start(true);
}

//开阀门
void valveOn(void)
{
	moter28Backward();
	valveOnFlag=true;

}
//关阀门
void valveOff(void)
{
	moter28Forward();
	valveOffFlag=true;
}
//false表示故障，true表示正常
bool getMoter28Fault(void)
{
    return (tmux1308ReadData0(valveFault) == 0) ? true : false;
}

//清除28步进故障 true成功 false失败
bool clearFault(void)
{
    /* No hardware fault-reset operation is implemented for this driver.
       Report failure rather than falling through without a return value. */
    return false;
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
