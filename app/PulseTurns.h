#ifndef PULSETURNS__H
#define PULSETURNS__H

#include "stdint.h"

extern volatile int64_t pulseCount;

void pulseTurnsInit(void);

float pulseTurnsGet(void);

#endif
