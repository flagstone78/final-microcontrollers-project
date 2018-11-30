#ifndef __systick /* __systick */
#define __systick

#include "stm32l476xx.h"
#include "stepper.h"
#include <stdlib.h>
volatile int oldx = 0;
volatile unsigned int speed = 0;

const unsigned int clk_freq = 4000000;
const unsigned int minSystick = 300;
const int adjustSpeed = 1;


void SysTick_Handler(){
	//if(count%10000 == 0){SysTick->LOAD = SysTick->LOAD - 2;}
	//if( oldx > 0){ }
	step();
}

void Systick_Initialization(){
	SysTick->CTRL = 0; //disable systick
	
	SysTick->LOAD = 100000;//(SysTick->CALIB & 0x001B7740)/100; //(SysTick->CALIB & 0x001B7740)/200; //40*(SysTick->CALIB & 0x001B7740);  //systick calibration tenMS val is 10 ms.
	
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1); //set systick priority to least important //check if nescessary
	
	SysTick->VAL = 0; //reset systick counter
	
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; //select processor clock
	
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; //enable the systick interrupt
	
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable the systick counter
}

void setSpeed(unsigned int hertz){
	if(hertz > 0){
		unsigned int ticks = clk_freq/hertz;
		if(ticks < minSystick){ticks = minSystick;}
		SysTick->LOAD = ticks;
	}
}

void Systick_Freq_Update(int16_t gyro_x){
	//SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; //disable the systick counter
	//unsigned int oldLoad = SysTick->LOAD;
	
	//if(gyro_x > 0  && gyro_x > oldx){ //as it moves farther away from zero continue to increase the speed
	//	speed += adjustSpeed; //if it keeps falling over increase this value
	//} else if(gyro_x > 0  && gyro_x < oldx){ //as it begins to correct gradually slow down the speed
	//	speed -= adjustSpeed; //may need to shrink this value if it keeps moving to fast and over corrects
	//}
	setSpeed(gyro_x/10);
	//oldx = gyro_x;
	
	//SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //enable the systick counter
	
	//oldx = gyro_x;
}

#endif /*__systick*/
