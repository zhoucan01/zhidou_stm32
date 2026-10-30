#ifndef FT__H
#define FT__H
/* Water and brine flow-meter pulse counter APIs. */

#include <stdint.h>

void waterFTtick(void);
void brineFTtick(void);
uint32_t waterFTPulseGet(void);
uint32_t brineFTPulseGet(void);
void clearWaterFT(void);

#endif
