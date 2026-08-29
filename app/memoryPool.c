#include "memoryPool.h"
#include "tmux1308.h"
#include "Tempurature.h"
#include "string.h"
#include "BusVol.h"
#include "stdbool.h"
#include "Current.h"
#include "PulseTurns.h"
#include "gpio.h"
#include "crc.h"
#include "crc16.h"
#include "usart.h"
#include "moveFilter.h"

uint8_t highBroadOrder[2];
uint8_t lowBroadOrder[6];
uint8_t recDate[10];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;

const char codeVersion[4]={0,0,0,1};//版本号

//用于存储数据，分4类，一类是控制相关，一类是在位相关，一类是参数相关,一类是电机状态
//	type1: beginning:0x63; ending:0x6c;
//	pressdate：beginning: 0x70 抬升高位，抬升低位，压制高位(pressSwitch1)，
//	压制低位(pressSwitch2),压制头在位(S4)，圈数float
//	type2 ：bejinning:0x61;ending:0x6c;
//	station:bejinning: 0x73 水箱在位，高位，低位;黄水盒在位;开关阀开位、关位;点卤盒在位;
//溢出;盖子在位;滤网在位;成型盒在位
//	type3 ：bejinning:0x76 ;ending:0x6c;
//  value bejing:0x64 温度，电压，电流
// 	type4 ：bejinning:0x6D;ending:0x72;
//	station:bejinning: 0x6f 开关阀,压制步进,清水泵,卤水泵,蠕动泵,抬升步进,破壁电机
//  type5 :0X76开头 0x6E结束 只用于版本号
static uint8_t pressDate[12]  ={0x63,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x6c};
static uint8_t stationDate[14]={0x61,0x73,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x6c};
static uint8_t valueDate[15]  ={0x76,0x64,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x6c};
static uint8_t moterState[10]	={0x6D,0x6f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x72};
static uint8_t versionData[8]	={0x76,codeVersion[0],codeVersion[1],codeVersion[2],codeVersion[3],0x00,0x00,0x6E};
bool lidAvail=false;
bool filterAvail=false;
bool overFlow=false;

void dataPoolUpdate(void)
{
	
	pressDate[2]=getTmux1308Io0Data(riseHighPos);
	pressDate[3]=getTmux1308Io0Data(riseLowPos);
	pressDate[4]=getTmux1308Io1Data(pressSwitch1);
	pressDate[5]=getTmux1308Io1Data(pressSwitch2);
	pressDate[6]=getTmux1308Io1Data(s4);
	float turns = pulseTurnsGet();  // 先存储到变量
	memcpy(&pressDate[7], &turns, sizeof(float));
	
	if((getTmux1308AdcData(lid1)>4000)&&(getTmux1308AdcData(lid2)<100))
	{
		lidAvail=false;
	}
	stationDate[2]=((getTmux1308AdcData(waterBoxAvail)>2800)?1:0);
	stationDate[3]=getTmux1308Io1Data(waterBoxHighPos);
	stationDate[4]=getTmux1308Io1Data(waterBoxLowPos);
	stationDate[5]=getTmux1308Io1Data(wasteBoxAvail);
	stationDate[6]=getTmux1308Io0Data(valveOpenPos);
	stationDate[7]=getTmux1308Io0Data(valveClosePos);
	stationDate[8]=getTmux1308Io0Data(brineAvail);
	stationDate[9]=overFlow;
	stationDate[10]=lidAvail;
	stationDate[11]=filterAvail;
	stationDate[12]=HAL_GPIO_ReadPin(boxAvail_GPIO_Port,boxAvail_Pin);
	
	//float temp = getTemp();  // 先存储到变量
	float temp =filter(calTemp());
	
	memcpy(&valueDate[2], &temp, sizeof(float));
	
	float vol = getBusVoltage();  // 先存储到变量
	memcpy(&valueDate[6], &vol, sizeof(float));	
	
	float cur = getCurrentAdc();  // 先存储到变量
	memcpy(&valueDate[10], &cur, sizeof(float));	
}	


uint8_t *getPressDate(void)
{
	return pressDate;
}

uint8_t *getStationDate(void)
{
	return stationDate;
}

uint8_t *getValueDate(void)
{
	return valueDate;
}

uint8_t *getmoterStateDate(void)
{
	return moterState;
}

void sendPressDate(void)
{
//	type1: beginning:0x63; ending:0x6c;
//	pressdate：beginning: 0x70 抬升高位，抬升低位，
//  压制高位(pressSwitch1)，压制低位(pressSwitch2),压制头在位(S4)，圈数float
    uint8_t txData[14];
    uint16_t crc;
	
    memcpy(txData,pressDate,11);

    crc=CRC16_Calc(txData,11);

    txData[11]=(uint8_t)(crc&0xff);
    txData[12]=(uint8_t)((crc>>8)&0xff);
    txData[13]=pressDate[11];

    HAL_UART_Transmit(&huart1,txData,sizeof(txData),0xff);
}
void sendStationDate(void)
{
//	type2 ：bejinning:0x61;ending:0x6c;
//	station:bejinning: 0x73 水箱在位，高位，低位;
//黄水盒在位;开关阀开位、关位;点卤盒在位;溢出;盖子在位;滤网在位;成型盒在位
    uint8_t txData[16];
    uint16_t crc;

    memcpy(txData,stationDate,13);

    crc=CRC16_Calc(txData,13);

    txData[13]=(uint8_t)(crc&0xff);
    txData[14]=(uint8_t)((crc>>8)&0xff);
    txData[15]=stationDate[13];

    HAL_UART_Transmit(&huart1,txData,sizeof(txData),0xff);
}

uint8_t recDateBuffer[10];
uint8_t rightReturn[2]={0xff,0x00};
uint8_t repeatTrans[2]={0x00,0xff};

uint16_t usartcrc;
uint8_t tempCrcHigh;//crc高位
uint8_t tempCrcLow;//crc低位

bool isTrans=false;
uint8_t recHighData;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
    // 判断是哪个串口，以防多个串口共用同一个回调
		
    if (huart->Instance == USART1) 
		{//屏幕发送的数据
      //把数据复制给数据缓冲区
			memcpy(recDateBuffer,recDate,10);
			//验证帧头和中间数据
			usartcrc=CRC16_Calc(recDateBuffer,7);
			//赋值
			tempCrcLow=(uint8_t)(usartcrc&0xff);
			tempCrcHigh=(uint8_t)((usartcrc>>8)&0xff);	
			//crc值对比过了的话，传入指令表
			if((tempCrcLow==recDateBuffer[7])&&(tempCrcHigh==recDateBuffer[8]))
			{
				//复制指令数据
				memcpy(lowBroadOrder,&recDateBuffer[1],6);
				//返回正确数据指令
				//HAL_UART_Transmit(&huart1,rightReturn,sizeof(rightReturn),0x0f);
			}
			else
			{
				//错误的话返回重发指令
				//HAL_UART_Transmit(&huart1,repeatTrans,sizeof(repeatTrans),0x0f);
			}
			//开启下次接收
			HAL_UART_Receive_IT(&huart1, recDate, sizeof(recDate));
    }
		if (huart->Instance == USART3)//高压板数据返回
		{
			//数据回传 70
			
//			HAL_UART_Transmit(&huart3,highBroadOrder,sizeof(highBroadOrder),0x0f);
//			
			HAL_UART_Receive_IT(&huart3, &recHighData, sizeof(recHighData));
		}
}


void sendValueDate(void)
{
	//	type3 ：bejinning:0x76 0x64;ending:0x6c;
    uint8_t txData[17];
    uint16_t crc;

    memcpy(txData,valueDate,14);

    crc=CRC16_Calc(txData,14);

    txData[14]=(uint8_t)(crc&0xff);
    txData[15]=(uint8_t)((crc>>8)&0xff);
    txData[16]=valueDate[14];

    HAL_UART_Transmit(&huart1,txData,sizeof(txData),0xff);
}

void dataSync(void)
{
	static uint8_t Synctick = 0;

	switch(Synctick)
	{
		case 0:
			sendPressDate();
			break;
		case 1:
			sendStationDate();
			break;
		case 2:
			sendValueDate();
			break;
		default:
			Synctick = 0;
			return;
	}

	Synctick++;
	if(Synctick >= 3)
	{
		Synctick = 0;
	}
}

void versionSync(void)
{
	 uint16_t crc;
   crc=CRC16_Calc(versionData,5);
	 versionData[5]=(uint8_t)(crc&0xff);
	 versionData[6]=(uint8_t)((crc>>8)&0xff);
	 HAL_UART_Transmit(&huart1,versionData,sizeof(versionData),0xff);
}
