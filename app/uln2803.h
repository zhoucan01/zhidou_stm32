#ifndef ULN2803_H
#define ULN2803_H

#include "main.h"

void moter28step(uint8_t dir);
void moter28Start(void);


void valveDect(void);

void valveOn(void);
void valveOff(void);
void riseMoterUp(void);
void riseMoterDown(void);
void riseMoterStop(void);
void riseRunDec(void);
void riseTick(void);

#endif