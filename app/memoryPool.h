#ifndef MEMORYPOOL__H
#define MEMORYPOOL__H

#include "stdint.h"

extern uint8_t recDate[10];
extern uint8_t lowBroadOrder[6];
extern uint8_t highBroadOrder[2];
extern uint8_t recDateBuffer[10];

typedef enum{
	switchValve=2,
	preeMoter,
	clearWaterPump,
	drinePump,
	hosePump,
	riseMoter,
	breakMoter
}moterDate;

void dataPoolUpdate(void);
uint8_t *getPressDate(void);
uint8_t *getStationDate(void);
uint8_t *getValueDate(void);
uint8_t *getmoterStateDate(void);
void sendPressDate(void);
void sendStationDate(void);
void sendValueDate(void);
void dataSync(void);
void versionSync(void);
#endif
