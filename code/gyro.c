#include "gyro.h"

void GYRO_Init(){
	L3GD20_CS_HIGH; //set chip select high
	
	//3-wire mode is entered by setting the bit SIM to ‘1’ in CTRL_REG4.
	
	uint8_t val = 0;
	val &= ~(1U << 0);  //4 wire spi mode
	val &= ~(1U << 1); //must be 0
	val &= ~(1U << 2); //must be 0
	val &= ~(1U << 3); //blank
	
	//full scale select 250 dps
	//for 250dps, 1 unit equals 3.814 millidegrees per second
	val &= ~(1U << 4);
	val &= ~(1U << 5);
	
	val &= ~(1U << 6); //LSB at lower address
	val &= ~(1U << 7); //continuous update
	GYRO_IO_Write(CTRL_REG4, 1, &val);
	
	uint8_t ctrl_reg_val = 0xff; // power up gyro at full speed, all axes.
	GYRO_IO_Write(CTRL_REG1, 1, &ctrl_reg_val);
}

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
	GPIOE->ODR |= GPIO_ODR_OD12;
	delay(10);
}

void printAllGyro(){
	uint8_t data = 0;
	GYRO_IO_Read(WHO_AM_I, 1, &data);
	
	uint8_t alldata[25];
	for(int i = 0; i<25; i++){
		GYRO_IO_Read(CTRL_REG1+i, 1, &alldata[i]);
	}
	SerialHex(&data, 1);
	SerialHex(alldata, 25);
	newline();
}
