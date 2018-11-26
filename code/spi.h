#ifndef __spi /* __spi */
#define __spi

#include "stm32l476xx.h" 

#define L3GD20_CS_LOW 	GPIOD->ODR &= ~(1U << 7)
#define L3GD20_CS_HIGH 	GPIOD->ODR |= (1U << 7) 

void delay(uint32_t);
void SPI_Init(void);
void SPI2_ReadWrite(uint8_t*, uint8_t*, int );
int spi_read4Wire(int);

	
#endif /* __spi */
