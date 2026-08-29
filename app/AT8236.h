#ifndef AT8236__H
#define AT8236__H

#include "main.h"
#include "stdint.h"

extern int hosePumpOnSpeed;
extern int hosePumpFlag;

void brinePumpOn(void);
void brinePumpOff(void);
void hosePumpOn(uint8_t ppPumpSpeed);
void hosePumpOff(void);
void waterPumpOn(int ISPumpSpeed);
void waterPumpOff(void);

#endif

