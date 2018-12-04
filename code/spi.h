#ifndef __spi /* __spi */
#define __spi

#include "stm32l476xx.h" 

#define L3GD20_CS_LOW 	GPIOD->ODR &= ~(1U << 7)
#define L3GD20_CS_HIGH 	GPIOD->ODR |= (1U << 7) 

void delay(uint32_t);
void SPI_Init(void);
uint8_t sendRecieve8(uint8_t data);
	
#endif /* __spi */
