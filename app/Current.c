#include "Current.h"

#include "stdint.h"
#include "tmux1308.h"


float getCurrent(void)
{
	 return getCurrentAdc()/4095.0f*3.3;
}
