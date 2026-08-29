#ifndef CRC16_H
#define CRC16_H


#include <stdint.h>
#include "main.h"


uint16_t CRC16_Calc(uint8_t *pData, uint32_t len);


#endif