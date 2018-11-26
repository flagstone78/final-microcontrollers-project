#ifndef __systick /* __systick */
#define __systick

#include "stm32l476xx.h"
#include "stepper.h"
volatile int count = 0;

void SysTick_Handler(){
	//if(count%10000 == 0){SysTick->LOAD = SysTick->LOAD - 2;}
	step();
}

void Systick_Initialization(){
	SysTick->CTRL = 0; //disable systick
	
	SysTick->LOAD = (SysTick->CALIB & 0x001B7740)/100; //(SysTick->CALIB & 0x001B7740)/200; //40*(SysTick->CALIB & 0x001B7740);  //systick calibration tenMS val is 10 ms.
	
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1); //set systick priority to least important //check if nescessary
	
	SysTick->VAL = 0; //reset systick counter
	
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; //select processor clock
	
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //enable the systick interrupt
	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable the systick counter
}
#endif /*__systick*/
