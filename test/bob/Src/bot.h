#ifndef __bots /* __serial */
#define __bots

#include <math.h>

#include "serial.h"
#include "stm32l4xx_hal.h"
#include "peripheralHandles.h"
#include "main.h"



void initBot(void);
void initMPU(void);
void processData(void);

#endif
