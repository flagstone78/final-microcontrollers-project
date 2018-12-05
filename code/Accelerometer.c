#include "Accelerometer.h"

void ACCEL_Init(){
	LSM303CTR_CS_HIGH; //set chip select high
	
	uint8_t val = 0;
	//acceleromter setup
	//CTRL_REG4_A SIM bit set to 1 for 3 wire mode
	val |= (1U << 0); //enable 3 wire mode
	val &= ~(1U << 1); //disable I2C
	val |= (1U << 2); //enable auto increment when reading 
	val &= ~(1U << 3); //auto set bandwidth
	
	// set full scale to +-2g
	val &= ~(1U << 4); 
	val &= ~(1U << 5);
	
	//band width selection
	val &= ~(1U << 6); 
	val &= ~(1U << 7); //enable high resolution (might not need)
	ACCEL_IO_Write(CTRL_REG4_A, 1, &val);
	
	//CTRL_REG1_A settings
	val = 0;
	val |= (1U << 0); //enable X axis 
	val |= (1U << 1); //enable Y axis 
	val |= (1U << 2); //enable Z axis 
	val &= ~(1U << 3); //continuous update
	
	// set odr to 800Hz
	val |= (1U << 4); 
	val |= (1U << 5); 
	val |= (1U << 6); 
	
	val &= ~(1U << 7); //enable high resolution (might not need)

	ACCEL_IO_Write(CTRL_REG1_A, 1, &val);
}

void ACCEL_IO_Read(uint8_t readADDR, unsigned int size, uint8_t *rxBuffer){
	//select read and multiple-byte mode
	readADDR |= (1U << 7); //read mode
		
	//set chip select low at the beginning of the transmission
	LSM303CTR_CS_LOW; // 0 = SPI
	delay(10);
	
	sendRecieve8(readADDR); //send address, ingore input
	for(int i = 0; i < size; i++){
		//rxBuffer[i] = sendRecieve8(0xff); //send a bunch of dummy 0xff and read the response
		rxBuffer[i] = spi_read3Wire(); //half duplex recieve
	}

	//release chip select
	LSM303CTR_CS_HIGH;
	delay(10);
}

void ACCEL_IO_Write(uint8_t writeAddress, uint8_t size, uint8_t *txBuffer){
	writeAddress &=  ~(1U << 7); //write mode
	if(size > 1) {writeAddress |= (1U << 6);} //multiple byte mode
	//set chip select low at the beginning of the transmission
	LSM303CTR_CS_LOW; // 0 = SPI
	delay(10);
	
	sendRecieve8(writeAddress); //send address, ingore input
	for(int i = 0; i < size; i++){
		sendRecieve8(txBuffer[i]); //send data and ignore the response
	}

	//release chip select
	LSM303CTR_CS_HIGH;
	delay(10);
}

void printAllAccel(){
	uint8_t data = 0;
	ACCEL_IO_Read(WHO_AM_I, 1, &data);
	
	uint8_t alldata[34];
	for(int i = 0; i<34; i++){
		ACCEL_IO_Read(ACT_THS_A+i, 1, &alldata[i]);
	}
	SerialHex(&data, 1);
	SerialHex(alldata, 34);
	newline();
}
