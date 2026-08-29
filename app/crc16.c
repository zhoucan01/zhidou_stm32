#include "crc16.h"

uint16_t CRC16_Calc(uint8_t *data, uint32_t len)
{
    uint32_t word = 0;
    uint32_t i;
    
    CRC->CR = CRC_CR_RESET;
    
    while(len >= 4)
    {
        word = (uint32_t)data[0] | ((uint32_t)data[1] << 8) | 
               ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
        CRC->DR = word;
        data += 4;
        len -= 4;
    }
    
    if(len > 0)
    {
        word = 0;
        for(i = 0; i < len; i++)
            word |= (uint32_t)data[i] << (i * 8);
        CRC->DR = word;
    }
    
    return (uint16_t)(CRC->DR);
}
