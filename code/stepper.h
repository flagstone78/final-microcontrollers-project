#ifndef __stepper /* __stepper */
#define __stepper
#include "stm32l476xx.h" 

void stepper_Init(void);
void setDirection(int dir);
void step(void);

#endif /* __stepper */
