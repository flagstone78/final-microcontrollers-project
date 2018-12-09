#include "stepper.h"

void stepper_Init(void){
	//stepper 1
	//dir PE15 PE13
	//step PE12 PE14
	RCC-> AHB2ENR |= RCC_AHB2ENR_GPIOEEN; // enable port b clock
	//set gpio e 15, 13, 12 and 14 to output
	GPIOE->MODER &= ~(GPIO_MODER_MODE15 | GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE12);
	GPIOE->MODER |= (GPIO_MODER_MODE15_0 | GPIO_MODER_MODE13_0 | GPIO_MODER_MODE14_0 | GPIO_MODER_MODE12_0);
	
	//Set PD7 otypr to push/pull (0x0)
  GPIOE->OTYPER &= ~(GPIO_OTYPER_OT_15 | GPIO_OTYPER_OT_13 | GPIO_OTYPER_OT_12 | GPIO_OTYPER_OT_14);
}

int deadzone = 0;
void setDirection(int dir){
	if(dir > deadzone){
		GPIOE->ODR |= (1U << 15 | 1U << 13); //set dir pin PE15 abd 13	
	} else if(dir < -deadzone) {
		GPIOE->ODR &= ~(1U << 15 | 1U << 13); //clear dir pin E15 and 13
	}
}

void step(void){
	GPIOE->ODR ^= ((1U << 12) | (1U << 14)); //toggle step pin PE12 and 14
}
