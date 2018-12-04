#include "gyro.h"


/*void GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadADDR, uint8_t size){
	uint8_t rxBuffer[32];
	
	//select read and multiple-byte mode
	uint8_t AddrByte = (ReadADDR | (1U << 7) ); //read mode
	
	if(size > 1) {AddrByte |= (1U << 6);} //multiple byte mode
	
	//set chip select low at the beginning of the transmission
	L3GD20_CS_LOW; // 0 = SPI
	delay(10);
	
	//send the address of the indexed register
	SPI2_ReadWrite(&AddrByte, rxBuffer, 1);
	
	uint8_t txBuffer[32] ={0xff}; //fill with 0xff filler
	txBuffer[0] = AddrByte;
	//receive the data that will be read from the device (MSB first)
	SPI2_ReadWrite(txBuffer, pBuffer, size);

	//set chip select
	delay(10);
	L3GD20_CS_HIGH;
}

void WriteGyroRegister(int SubAddress, int Value)
{
	GPIOD->ODR &= ~GPIO_ODR_OD7; // Drive CS low
	short RValue;
	int timeout = 100000;
	
	SPI2->DR = (short)((Value << 8)+SubAddress); //put write bit on address
	RValue = SPI2->SR; //???
	
	while ( (timeout--) && ((SPI2->SR & SPI_SR_BSY)!=0) ); // wait for tx complete or time out
	RValue = SPI2->DR; // dummy read of data register
	GPIOD->ODR |= GPIO_ODR_OD7; // Drive CS high
}*/

void GYRO_IO_Read(uint8_t readADDR, unsigned int size, uint8_t *rxBuffer){
	//select read and multiple-byte mode
	readADDR |= (1U << 7); //read mode
	
	if(size > 1) {readADDR |= (1U << 6);} //multiple byte mode
	
	//set chip select low at the beginning of the transmission
	L3GD20_CS_LOW; // 0 = SPI
	delay(10);
	
	sendRecieve8(readADDR); //send address, ingore input
	for(int i = 0; i < size; i++){
		rxBuffer[i] = sendRecieve8(0xff); //send a bunch of dummy 0xff and read the response
	}

	//release chip select
	L3GD20_CS_HIGH;
	delay(10);
}

void GYRO_IO_Write(uint8_t writeAddress, uint8_t size, uint8_t *txBuffer){
	writeAddress &=  ~(1U << 7); //write mode
	if(size > 1) {writeAddress |= (1U << 6);} //multiple byte mode
	//set chip select low at the beginning of the transmission
	L3GD20_CS_LOW; // 0 = SPI
	delay(10);
	
	sendRecieve8(writeAddress); //send address, ingore input
	for(int i = 0; i < size; i++){
		sendRecieve8(txBuffer[i]); //send data and ignore the response
	}

	//release chip select
	L3GD20_CS_HIGH;
	delay(10);
}
