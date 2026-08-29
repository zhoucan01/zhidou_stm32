#ifndef STEP_MOTER_H
#define STEP_MOTER_H

#include "stdint.h"

void stepMoterStop(void);
void stepMoterUp(float speed);
void stepMoterDown(float speed);
void stepMoterStart(void);

#endif