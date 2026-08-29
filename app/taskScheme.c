#include "taskScheme.h"

#include "AT8236.h"
#include "ws2812.h"
#include "uln2803.h"
#include "tmux1308.h"
#include "stepMoter.h"
#include "led.h"

#define WRAP_0(F) \
static void wrapper_##F(CommandParam_t *param){ \
    (void)param; \
    F(); \
}

#define WRAP_1(F) \
static void wrapper_##F(CommandParam_t *param){ \
    F(param->para[0]); \
}

#define WRAP_2(F) \
static void wrapper_##F(CommandParam_t *param){ \
    F(param->para[0],param->para[1]); \
}

#define WRAP_3(F) \
static void wrapper_##F(CommandParam_t *param){ \
    F(param->para[0],param->para[1],param->para[2]); \
}

#define WRAP_4(F) \
static void wrapper_##F(CommandParam_t *param){ \
    F(param->para[0],param->para[1],param->para[2],param->para[3]); \
}

#define WRAP_5(F) \
static void wrapper_##F(CommandParam_t *param){ \
    F(param->para[0],param->para[1],param->para[2],param->para[3],param->para[4]); \
}

typedef enum{
    CMDNONE=0x00,

    CMDWATERPUMPON,			//清水泵开
    CMDWATERPUMPOFF,		//清水泵关
    CMDBRINEPUMPON,			//点卤泵开
    CMDBRINEPUMPOFF,		//点卤泵关
    CMDHOSEPUMPON,			//蠕动泵开
    CMDHOSEPUMPOFF,			//蠕动泵关
		CMDPUSHMOTORDOWN,		//压制电机下压
		CMDPUSHMOTORUP,			//压制电机上升
		CMDPUSHMOTORSTOP,		//压制电机停止
		CMDRISEMOTORDOWN,		//抬升电机下压
		CMDRISEMOTORUP,			//抬升电机上升
		CMDRISEMOTORSTOP,		//抬升电机停止	
		CMDLIGHTSHOW,				//WS2812发光
		CMDHIGHVOLBROAD,		//高压板控制
		CMDVAVLEON,					//开关阀开
		CMDVAVLEOFF,				//开关阀关
		CMDLEDON,						//开灯
		CMDLEDOFF,					//关灯
    CMD_MAX
}CommandCmd_t;


WRAP_0(waterPumpOff)
WRAP_0(hosePumpOff)
WRAP_0(brinePumpOff)
WRAP_0(brinePumpOn)
WRAP_0(stepMoterStop)
WRAP_0(valveOn)
WRAP_0(valveOff)
WRAP_0(riseMoterUp)
WRAP_0(riseMoterDown)
WRAP_0(riseMoterStop)
WRAP_0(ledOn)
WRAP_0(ledOff)
WRAP_0(highVolBroad)


WRAP_1(hosePumpOn)
WRAP_1(stepMoterUp)
WRAP_1(stepMoterDown)
WRAP_1(waterPumpOn)

//WRAP_2()

WRAP_4(WS2812_SetColor)

static CommandAction_t actionTable[CMD_MAX]={
    [CMDWATERPUMPON]=wrapper_waterPumpOn,
    [CMDWATERPUMPOFF]=wrapper_waterPumpOff,

    [CMDBRINEPUMPON]=wrapper_brinePumpOn,
    [CMDBRINEPUMPOFF]=wrapper_brinePumpOff,

    [CMDHOSEPUMPON]=wrapper_hosePumpOn,
    [CMDHOSEPUMPOFF]=wrapper_hosePumpOff,

    [CMDPUSHMOTORDOWN]=wrapper_stepMoterDown,
    [CMDPUSHMOTORUP]=wrapper_stepMoterUp,
    [CMDPUSHMOTORSTOP]=wrapper_stepMoterStop,

    [CMDRISEMOTORDOWN]=wrapper_riseMoterDown,
    [CMDRISEMOTORUP]=wrapper_riseMoterUp,
    [CMDRISEMOTORSTOP]=wrapper_riseMoterStop,

    [CMDLEDON]=wrapper_ledOn,
    [CMDLEDOFF]=wrapper_ledOff,
	
    [CMDLIGHTSHOW]=wrapper_WS2812_SetColor,

    [CMDHIGHVOLBROAD]=wrapper_highVolBroad,

    [CMDVAVLEON]=wrapper_valveOn,
    [CMDVAVLEOFF]=wrapper_valveOff
};

void commandExecute(uint8_t command,CommandParam_t *param)
{
    if(param==NULL){
        return;
    }

    if(command==CMDNONE||command>=CMD_MAX){
        return;
    }

    if(actionTable[command]==NULL){
        return;
    }

    actionTable[command](param);
}
