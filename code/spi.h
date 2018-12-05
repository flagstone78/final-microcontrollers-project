#ifndef __spi /* __spi */
#define __spi

#include "stm32l476xx.h" 

void delay(uint32_t);
void SPI_Init(void);
uint8_t sendRecieve8(uint8_t data);
//void send8(uint8_t data);
uint8_t recieve8(void);
uint8_t spi_read3Wire(void);
	
#endif /* __spi */
