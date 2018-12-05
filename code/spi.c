#include "spi.h"


void delay(uint32_t cycles){
	while(cycles > 0){cycles--;}
}

void BitBangSPI_PinSetup(int isSPI){
	//all gpio to output
	if(isSPI > 0){
		//Set PD1, PD3, PD4 moder to alternate function (0x10)
		GPIOD->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4); //clear
		GPIOD->MODER |= (GPIO_MODER_MODE1_1 | GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1); //set to AF mode
		
		//Set PD1, PD3, PD4 alternate function to AF5 for SPI	
		GPIOD->AFR[0] &= ~(GPIO_AFRL_AFSEL1 | GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4);
		GPIOD->AFR[0] |= (GPIO_AFRL_AFSEL1_0 | GPIO_AFRL_AFSEL1_2 | 
											GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_2 | 
											GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2);
	} else {
		GPIOD->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE4);
		//output mode for clk (pd1) and input for MOSI (pd4)
		GPIOD->MODER |=  (GPIO_MODER_MODE1_0 );	
		GPIOD->ODR |= GPIO_ODR_OD1; //set clk high
	}
}

/*
//0 recieve - data pin set to input (high impedance)
//1 transmit - data pin set to output
void RxTxPinSetup(int mode){
	
}

void BitBangSPITransmit(uint8_t data){
	RxTxPinSetup(1); //set to transmit
	for(int i = 0; i<8; i++){
		//set clock low
		//set data
		//set clock high
	}
}

uint8_t BitBangSPIRecieve(void){

}*/

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
	
	//accelerometer chip select
		
	RCC-> AHB2ENR |= RCC_AHB2ENR_GPIOEEN; //Enable GPIO E clock
	GPIOE->MODER &= ~GPIO_MODER_MODE0;
	GPIOE->MODER |= GPIO_MODER_MODE0_0;	//Set PE0 moder to output mode (0x01)
	
	//Set PE0 otypr to push/pull (0x0)
  GPIOE->OTYPER &= ~GPIO_OTYPER_OT_0;
	
	//SPI1
	//spi setup	--------------------------------
	RCC-> AHB2ENR |= RCC_AHB2ENR_GPIODEN ;//Enable GPIO D clock
		
	//Set PE13, PE14, PE15 moder to alternate function (0x10)
	GPIOE->MODER &= ~(GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15); //clear
	GPIOE->MODER |= (GPIO_MODER_MODE13_1 | GPIO_MODER_MODE14_1 | GPIO_MODER_MODE15_1); //set to AF mode
	
	//Set PE13, PE14, PE15 alternate function to AF5 for SPI	
	GPIOE->AFR[0] &= ~(GPIO_AFRH_AFSEL13 | GPIO_AFRH_AFSEL14 | GPIO_AFRH_AFSEL15);
	GPIOE->AFR[0] |= (GPIO_AFRH_AFSEL13_0 | GPIO_AFRH_AFSEL13_2 | 
										GPIO_AFRH_AFSEL14_0 | GPIO_AFRH_AFSEL14_2 | 
										GPIO_AFRH_AFSEL15_0 | GPIO_AFRH_AFSEL15_2);
										
	//spi1 chip select
	GPIOE->MODER &= ~GPIO_MODER_MODE12;
	GPIOE->MODER |= GPIO_MODER_MODE12_0;	//Set PD7 moder to output mode (0x01)
	GPIOE->ODR |= GPIO_ODR_OD12;
	
	//Set PD7 otypr to push/pull (0x0)
  GPIOE->OTYPER &= ~GPIO_OTYPER_OT_12;
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
	SPI2->CR2 |= SPI_CR2_FRXTH; //set FIFO threshhold to 8 bit
	
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
	//SPI2->CR1 |= SPI_CR1_BIDIMODE; //3 wire bidirectional interface
  SPI2->CR1 |= SPI_CR1_CPHA; //Data valid on second clock edge
	SPI2->CR1 |= SPI_CR1_CPOL; //CK = 1 when idle
	SPI2->CR1 |= SPI_CR1_MSTR; //master
	SPI2->CR1 &= ~SPI_CR1_BR; //clear baud rate to /2
	//SPI2->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2); //set baud rate
	
	SPI2->CR1 |= SPI_CR1_SSI; //internal slave select
	SPI2->CR1 |= SPI_CR1_SSM; //software slave managment
	
	SPI2->CR1 |= SPI_CR1_SPE; //enable
	
	//SPI1 ------------------------------
	// Turn on SPI1 clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// reset the SPI1 interface
	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	delay(10);
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
	
	SPI1->CR2 = (0x07 << 8); // 8 bits per transfer, 1 byte threshold
	//SPI1->CR2 |= SPI_CR2_FRXTH; //set FIFO threshhold to 8 bit
	
	//SPI1->CR1 |= SPI_CR1_BIDIMODE; //3 wire bidirectional interface
  SPI1->CR1 |= SPI_CR1_CPHA; //Data valid on second clock edge
	SPI1->CR1 |= SPI_CR1_CPOL; //CK = 1 when idle
	SPI1->CR1 |= SPI_CR1_MSTR; //master
	SPI1->CR1 &= ~SPI_CR1_BR; //clear baud rate to /2
	//SPI2->CR1 |= (SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2); //set baud rate
	
	SPI1->CR1 |= SPI_CR1_SSI; //internal slave select
	SPI1->CR1 |= SPI_CR1_SSM; //software slave managment
	
	SPI1->CR1 |= SPI_CR1_SPE; //enable
}

//full duplex
uint8_t sendRecieve8(uint8_t data){
	*((uint8_t*)&SPI2->DR) = (uint8_t) data; //write only 8 bit to DR; otherwise data packing is used
	int timeout = 100000;
	while ((timeout--) && ((SPI2->SR & SPI_SR_BSY)!=0) ); // wait for tx complete
	
	return *((uint8_t*)&SPI2->DR); //read an 8 bit value
}

//half duplex
uint8_t recieve8(void){
	SPI2->CR2 |= SPI_CR1_BIDIMODE;//set bidirectional mode
	SPI2->CR2 &= ~SPI_CR1_BIDIOE;//set spi line to rx mode to start clk

	int timeout = 100000;
	while ((timeout--) && ((SPI2->SR & SPI_SR_RXNE) != SPI_SR_RXNE)); // wait for read to complete
	
	SPI2->CR2 |= SPI_CR1_BIDIOE;//set spi line to tx mode to stop clk
	SPI2->CR2 &= ~SPI_CR1_BIDIMODE;//set full duplex mode
	return *((uint8_t*)&SPI2->DR); //read an 8 bit value
}

uint8_t spi_read3Wire(){	
	uint8_t out = 0;
	BitBangSPI_PinSetup(0); //set bitbang mode
	for(int i = 0; i<8; i++){
		GPIOD->ODR &= ~GPIO_ODR_OD1; //low clock
		out = out << 1;
		if( (GPIOD->IDR & GPIO_IDR_ID4) > 0) {out += 1;}
		GPIOD->ODR |= GPIO_ODR_OD1; //high clock
	}
	BitBangSPI_PinSetup(1); //set spi mode
	
	return out;
}
