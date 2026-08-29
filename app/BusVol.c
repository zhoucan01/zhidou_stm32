#include "BusVol.h"
#include "stdint.h"
#include "tmux1308.h"
 
	
float getBusVoltage(void)
{
	return (getTmux1308AdcData(busVol)/4095.0f*3.3*11);
}

