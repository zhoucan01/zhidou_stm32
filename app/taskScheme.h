#ifndef TASKSCHEME__H
#define TASKSCHEME__H
#include "stdint.h"

typedef struct{
    int32_t para[8];
}CommandParam_t;

typedef void (*CommandAction_t)(CommandParam_t *param);

void commandExecute(uint8_t command,CommandParam_t *param);

#endif
