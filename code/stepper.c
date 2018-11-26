#include "stepper.h"

void stepper_Init(void){
	//stepper 1
	//dir PB6
	//step PB7
	RCC-> AHB2ENR |= RCC_AHB2ENR_GPIOBEN; // enable port b clock
	//set gpio b 6 and 7 to output
	GPIOB->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE6);
	GPIOB->MODER |= (GPIO_MODER_MODE7_0 | GPIO_MODER_MODE6_0);
	
	//Set PD7 otypr to push/pull (0x0)
  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6);
}

int deadzone = 20;
void setDirection(int dir){
	if(dir > deadzone){
		GPIOB->ODR |= (1U << 6); //set dir pin PB6
	} else if(dir < -deadzone) {
		GPIOB->ODR &= ~(1U << 6); //clear dir pin PB6
	}
}

void step(void){
	GPIOB->ODR ^= (1U << 7); //toggle step pin PB7
}
