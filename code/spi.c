#include "spi.h"


void delay(uint32_t cycles){
	while(cycles > 0){cycles--;}
}

void SPI_PinSetup(){
	//spi setup	--------------------------------
	RCC-> AHB2ENR |= RCC_AHB2ENR_GPIODEN ;//Enable GPIO D clock
		
	//Set PD1, PD3, PD4 moder to alternate function (0x10)
	GPIOD->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4); //clear
	GPIOD->MODER |= (GPIO_MODER_MODE1_1 | GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1); //set to AF mode
	
	//Set PD1, PD3, PD4 alternate function to AF5 for SPI	
	GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL1 | GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4);
	GPIOD->AFR[0] |= (GPIO_AFRL_AFSEL1_0 | GPIO_AFRL_AFSEL1_2 | 
										GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_2 | 
										GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2);
		
	//gyro chip select
	GPIOD->MODER &= ~GPIO_MODER_MODE7;
	GPIOD->MODER |= GPIO_MODER_MODE7_0;	//Set PD7 moder to output mode (0x01)
	
	//Set PD7 otypr to push/pull (0x0)
  GPIOD->OTYPER &= ~GPIO_OTYPER_OT_7;
	L3GD20_CS_HIGH;
	
	//accelerometer chip select
		
	//RCC-> AHB2ENR |= RCC_AHB2ENR_GPIOEEN; //Enable GPIO E clock
	//GPIOE->MODER &= ~GPIO_MODER_MODE0;
	//GPIOE->MODER |= GPIO_MODER_MODE0_0;	//Set PE0 moder to output mode (0x01)
	
	//Set PE0 otypr to push/pull (0x0)
  //GPIOD->OTYPER &= ~GPIO_OTYPER_OT_0;
}

void SPI_Init(){ //pg 1451
	SPI_PinSetup();
	// Turn on SPI2 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
	// reset the SPI2 interface
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST;
	delay(10);
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;
	
	SPI2->CR2 = (0x07 << 8); // 8 bits per transfer, 1 byte threshold	
	// setup the SPI operating modes etc
	/*
		Bidirectional mode (BIT15 = 1 for bidirectional mode)
		Software slave management (chip select controlled by softare)
		Internal slave select 
		8 databits (Bit 7 = 0)
		Enable SPI
		Freq = fpclk/256
		Master
		CPOL (CK = 1 when idle)
		CPHA (Data valid on second clock edge)
	*/
  SPI2->CR1 |= SPI_CR1_CPHA; //Data valid on second clock edge
	SPI2->CR1 |= SPI_CR1_CPOL; //CK = 1 when idle
	SPI2->CR1 |= SPI_CR1_MSTR; //master
	SPI2->CR1 &= ~SPI_CR1_BR; //clear baud rate to /2
	//SPI2->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2); //set baud rate
	
	SPI2->CR1 |= SPI_CR1_SSI; //internal slave select
	SPI2->CR1 |= SPI_CR1_SSM; //software slave managment
	
	SPI2->CR1 |= SPI_CR1_SPE; //enable
}

uint8_t sendRecieve8(uint8_t data){
	*((uint8_t*)&SPI2->DR) = (uint8_t) data; //write only 8 bit to DR; otherwise data packing is used
	int timeout = 100000;
	while ((timeout--) && ((SPI2->SR & SPI_SR_BSY)!=0) ); // wait for tx complete
	
	//while((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE);//wait for recieve data
	
	return *((uint8_t*)&SPI2->DR); //read an 8 bit value
}
