#include "stm32l476xx.h"

//*************************************  32L476GDISCOVERY ***************************************************************************
// STM32L4:  STM32L476VGT6 MCU = ARM Cortex-M4 + FPU + DSP, 
//           LQFP100, 1 MB of Flash, 128 KB of SRAM
//           Instruction cache = 32 lines of 4x64 bits (1KB)
//           Data cache = 8 lines of 4x64 bits (256 B)
//
// Joystick (MT-008A): 
//   Right = PA2        Up   = PA3         Center = PA0
//   Left  = PA1        Down = PA5
//
// User LEDs: 
//   LD4 Red   = PB2    LD5 Green = PE8
//   
// CS43L22 Audio DAC Stereo (I2C address 0x94):  
//   SAI1_MCK = PE2     SAI1_SD  = PE6    I2C1_SDA = PB7    Audio_RST = PE3    
//   SAI1_SCK = PE5     SAI1_FS  = PE4    I2C1_SCL = PB6                                           
//
// MP34DT01 Digital MEMS microphone 
//    Audio_CLK = PE9   Audio_DIN = PE7
//
// LSM303C eCompass (a 3D accelerometer and 3D magnetometer module): 
//   MEMS_SCK  = PD1    MAG_DRDY = PC2    XL_CS  = PE0             
//   MEMS_MOSI = PD4    MAG_CS  = PC0     XL_INT = PE1       
//                      MAG_INT = PC1 
//
// L3GD20 Gyro (three-axis digital output): 
//   MEMS_SCK  = PD1    GYRO_CS   = PD7
//   MEMS_MOSI = PD4    GYRO_INT1 = PD2
//   MEMS_MISO = PD3    GYRO_INT2 = PB8
//
// ST-Link V2 (Virtual com port, Mass Storage, Debug port): 
//   USART_TX = PD5     SWCLK = PA14      MFX_USART3_TX   MCO
//   USART_RX = PD6     SWDIO = PA13      MFX_USART3_RX   NRST
//   PB3 = 3V3_REG_ON   SWO = PB5      
//
// Quad SPI Flash Memory (128 Mbit)
//   QSPI_CS  = PE11    QSPI_D0 = PE12    QSPI_D2 = PE14
//   QSPI_CLK = PE10    QSPI_D1 = PE13    QSPI_D3 = PE15
//
// LCD (24 segments, 4 commons, multiplexed 1/4 duty, 1/3 bias) on DIP28 connector
//   VLCD = PC3
//   COM0 = PA8     COM1  = PA9      COM2  = PA10    COM3  = PB9
//   SEG0 = PA7     SEG6  = PD11     SEG12 = PB5     SEG18 = PD8
//   SEG1 = PC5     SEG7  = PD13     SEG13 = PC8     SEG19 = PB14
//   SEG2 = PB1     SEG8  = PD15     SEG14 = PC6     SEG20 = PB12
//   SEG3 = PB13    SEG9  = PC7      SEG15 = PD14    SEG21 = PB0
//   SEG4 = PB15    SEG10 = PA15     SEG16 = PD12    SEG22 = PC4
//   SEG5 = PD9     SEG11 = PB4      SEG17 = PD10    SEG23 = PA6
// 
// USB OTG
//   OTG_FS_PowerSwitchOn = PC9    OTG_FS_VBUS = PC11    OTG_FS_DM = PA11  
//   OTG_FS_OverCurrent   = PC10   OTG_FS_ID   = PC12    OTG_FS_DP = PA12  
//
// PC14 = OSC32_IN      PC15 = OSC32_OUT
// PH0  = OSC_IN        PH1  = OSC_OUT 
// 
// PA4  = DAC1_OUT1 (NLMFX0 WAKEUP)   PA5 = DAC1_OUT2 (Joy Down)
// PA3  = OPAMP1_VOUT (Joy Up)        PB0 = OPAMP2_VOUT (LCD SEG21)
//
//****************************************************************************************************************
#define L3GD20_STATUS_REG_ADDR 	0x27
#define L3GD20_OUT_X_L_ADDR	0x28

#define L3GD20_CS_LOW 	GPIOD->ODR &= ~(1U << 7)
#define L3GD20_CS_HIGH 	GPIOD->ODR |= (1U << 7) 

//L3GD20 gyro
#define WHO_AM_I			0x0f
#define CTRL_REG1			0x20
#define CTRL_REG2			0x21
#define CTRL_REG3			0x22
#define CTRL_REG4			0x23
#define CTRL_REG5			0x24
#define REFERENCE			0x25
#define OUT_TEMP			0x26
#define STATUS_REG			0x27
#define OUT_X_L				0x28
#define OUT_X_H				0x29
#define OUT_Y_L				0x2a
#define OUT_Y_H				0x2b
#define OUT_Z_L				0x2c
#define OUT_Z_H				0x2d
#define FIFO_CTRL_REG 		0x2e
#define FIFO_SRC_REG 		0x2f
#define INT1_CFG 			0x30
#define INT1_SRC 			0x31
#define INT1_TSH_XH			0x32
#define INT1_TSH_XL			0x33
#define INT1_TSH_YH			0x34
#define INT1_TSH_YL			0x35
#define INT1_TSH_ZH			0x36
#define INT1_TSH_ZL			0x37
#define INT1_DURATION		0x38

void delay(uint32_t cycles){
	while(cycles > 0){cycles--;}
}

void SPI_Read(SPI_TypeDef * SPIx, uint8_t * rxBuffer, int size){
	int i = 0;
	for(i = 0; i < size; i++){
		//wait for the transmit buffer to be empty
		while((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE);
		
		//send a dummy  byte to start the spi clock
		SPIx->DR = 0xFF;
		//wait for data
		while((SPIx->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		rxBuffer[i] = SPIx->DR; //save byte to buffer
	}
	//wait for bsy flag to clear
	while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY);
}

void SPI_Write(SPI_TypeDef * SPIx, uint8_t * txBuffer, uint8_t * rxBuffer, int size){
	int i = 0;
	for(i = 0; i < size; i++){
		//wait for transmit buffer to be empty
		while((SPIx->SR & SPI_SR_TXE) != SPI_SR_TXE);
		SPIx->DR = txBuffer[i]; //send data
		
		//wait for recieve buffer not empty
		//while((SPIx->SR & SPI_SR_RXNE) != SPI_SR_RXNE);
		//rxBuffer[i] = SPIx->DR; //save input data
	}
	// wait for BSY flag to clear
	while((SPIx->SR & SPI_SR_BSY) == SPI_SR_BSY);
}

void USART2_INIT(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; //Enable USART 2 clock
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);  
	RCC->CCIPR |= (RCC_CCIPR_USART2SEL_0); //Select system clock as the clock source of USART 2
	
	USART2->CR1 &= ~USART_CR1_UE;			//Disable usart
	USART2->CR1 &= ~USART_CR1_M;			//Set data length to 8 bits
	USART2->CR2 &= ~USART_CR2_STOP;		//Select 1 stop bit
	USART2->CR1 &= ~USART_CR1_PCE;		//Set parity to no parity bit
	USART2->CR1 &= ~USART_CR1_OVER8;	//Set oversampling to 16
	USART2->BRR = (0x1A0);										//Set baud rate to 9600 using APB frequency of 4MHz
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE); 	//Enable transmission and reception
	USART2->CR1 |= USART_CR1_UE;	//Enable usart
	while((USART2->ISR & USART_ISR_TEACK) == 0);//Verify that the USART is ready for transmission
	while((USART2->ISR & USART_ISR_REACK) == 0);//Verify that USART is ready for reception
}

void pinSetup(){
	//UART setup
	RCC-> AHB2ENR |= RCC_AHB2ENR_GPIODEN ; //Enable GPIO clock D
	 
	GPIOD->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6); 		//clear	
	GPIOD->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1); 	//Set mode to  alternate function for PD5 and PD6
	GPIOD->AFR[0] |= 0x77 << (4*5); 														//Set alternate function type to USART
	GPIOD->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6); //Set output speed to high speed for PD5 and PD6
	
	//Set PD5 and PD6 to pull up
	GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6); //clear
	GPIOD->PUPDR |= (GPIO_PUPDR_PUPD5_0 | GPIO_PUPDR_PUPD6_0); //01 for pull up
	
	GPIOD->OTYPER &= ~(GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6); //Set PD5 and PD6 to push pull
	//end UART setup
	
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

void SPI_Init(){ 
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN; // enable the spi2 clock
	RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST; //reset the spi2
	delay(10);
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST; //clear the reset of SPI2
	
	//SPI2->CR1 |= SPI_CR1_SPE; //enable the spi
	SPI2->CR1 &= ~SPI_CR1_SPE; //disable the spi
	SPI2->CR1 &=~SPI_CR1_RXONLY; // set to full duplex mode. (‘1’ is receive only mode)
	SPI2->CR1 &= ~SPI_CR1_BIDIMODE; //2 line unidirectional mode
	SPI2->CR1 &=~SPI_CR1_BIDIOE; //disable the output
	
	//data format
	SPI2->CR2 &= ~SPI_CR2_DS; //clear. DS is set to 0111 when an invalid setting is applied
	//SPI2->CR2 = SPI_CR2_DS_0 | 	SPI_CR2_DS_1 | SPI_CR2_DS_2; // 0111: 8 bit

	//Bit order
	SPI2 -> CR1 &= ~SPI_CR1_LSBFIRST; //transmit and receive the msb first

	//clock phase
	SPI2->CR1 &= ~SPI_CR1_CPHA; //first clock transition is the first data capture edge

	//clock polarity
	SPI2->CR1 &= ~SPI_CR1_CPOL; //Polarity Low

	//Baud rate control
	// 000 = f/2	001 = f/4	010 = f/8 up to 111 = f/256
	SPI2->CR1 &= ~SPI_CR1_BR; //sets the spi clock to 4MHZ/2 = 2 MHZ //TODO: bump up speed if needed

	SPI2->CRCPR = 10; //CRC polynomial

	//CRC calculation disabled
	SPI2->CR1 &= ~SPI_CR1_CRCEN;

	SPI2->CR2 &= ~SPI_CR2_FRF; //frame format to spi motorola mode
	
	//NSSGPIO 1 = software slave management	0 = Hardware NSS management
	SPI2->CR1 |= SPI_CR1_SSM;

	//set as master
	SPI2->CR1 |= SPI_CR1_MSTR;

	//Manage slave selection by software
	SPI2->CR1 |= SPI_CR1_SSI;

	//Enable slave selection pulse management
	SPI2->CR2 |= SPI_CR2_NSSP;

	//change how the RXNE goes high and low
	SPI2->CR2 |= SPI_CR2_FRXTH;

	//Enable the SPI
	SPI2->CR1 |= SPI_CR1_SPE;
}

void GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadADDR, uint8_t size){
	uint8_t rxBuffer[32];
	
	//select read and multiple-byte mode
	uint8_t AddrByte = (ReadADDR | (1U << 7) );
	
	//if(size > 1) {AddrByte |= (1U << 6);}
	
	//set chip select low at the beginning of the transmission
	L3GD20_CS_LOW; // 0 = SPI
	delay(10);
	
	//send the address of the indexed register
	SPI_Write(SPI2, &AddrByte, rxBuffer, 1);
	
	//receive the data that will be read from the device (MSB first)
	SPI_Read(SPI2, pBuffer, size);

	//set chip select
	delay(10);
	L3GD20_CS_HIGH;
}

void GYRO_IO_write(uint8_t *pBuffer, uint8_t WriteADDR, uint8_t size){
	uint8_t rxBuffer[32];
	
	//if(size > 0x01){ //enables multiple byte write //TODO: check that this is correct
	//	WriteADDR |= 1U << 6;
	//}
	
	//set SPI interface
	L3GD20_CS_LOW;
	delay(10);
	
	//send the address of the indexed register
	SPI_Write(SPI2, &WriteADDR, rxBuffer, 1);
	
	//send data
	SPI_Write(SPI2, pBuffer, rxBuffer, size);

	//set chip select high
	delay(10);
	L3GD20_CS_HIGH;
}

void serial(uint8_t *txBuffer, uint8_t size){
	//while(!(USART2->ISR & USART_ISR_RXNE)); //wait until the receive register has a value
	//letter = USART2->RDR; //read the receive register and store the value into the array of characters
	for(int i = 0; i<size; i++){
		while(!(USART2->ISR & USART_ISR_TXE)); //wait until the hardware is ready to send a bit
		//if(letter >= 0x61 && letter <= 0x7A){
		//	USART2->TDR = (letter - 0x20) & 0xFF; //change the character from lowercase to uppercase
		USART2->TDR = txBuffer[i];
		while(!(USART2->ISR & USART_ISR_TC)); // wait for the transmission to be completed
		//}
		
		USART2->ICR |= USART_ICR_TCCF; //clear the transmission complete flag
	}
}

const char charList[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void SerialHex(uint8_t *txBuffer, uint8_t size){
	uint8_t hex[size*4];
	for(int i = 0; i<size*4; i += 4){
		hex[i] = ' ';
		hex[i+1] = 'x';
		uint8_t tens = txBuffer[i]/16;
		hex[i+2] = charList[tens%16];
		hex[i+3] = charList[txBuffer[i]%16];
	}
	
	serial(hex, size*4);
}

struct {
	float x;
	float y;
	float z;
} Gyro;

const uint8_t gyroDefaults[] = {0x07,0,0,0,0,0,1,1,1,1,1,1,1,1,0,1,0,1,0,0,0,0,0,0,0};

int main(void){
	//RCC->CR |= (RCC_CR_HSION);
	//while( (RCC->CR & RCC_CR_HSIRDY) == 0);
	
	pinSetup();
	SPI_Init();
	
	
	USART2_INIT();
	
	//uint8_t gyr[6], 
	uint8_t status = 0;
	
	uint8_t all[48] = {0};
	uint8_t testChar[4] = {' ', '|', '\n','\r'};
	
	//GYRO_IO_Read(all, 0x20, 25);
	
	/*struct {
		float x;
		float y;
		float z;
	} gyro;*/
	
	//int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
	uint32_t delayTime = 100000;
	
	while(1){
		//GYRO_IO_Read(&status, 0x20, 1);
		//delay(delayTime);
		
		//status = 0x00;
		//GYRO_IO_write(&status, CTRL_REG4, 1); // power up gyro at full speed, all axes.
		//delay(delayTime);
		
		
		GYRO_IO_Read(&status, WHO_AM_I, 1);
		
		SerialHex(&status,1);
		
		delay(delayTime);
		
		serial(testChar, 4);
		
		delay(delayTime);
		
		GYRO_IO_Read(all, CTRL_REG1, 48);
		
		SerialHex(all,48);
		
		delay(delayTime);
		
		serial(testChar, 4);
		
		delay(delayTime);
		
		//status = 0x3E;
		//GYRO_IO_write(&status, CTRL_REG1, 1); // power up gyro at full speed, all axes.
		//delay(delayTime);
		
		
		
		
		/*GYRO_IO_Read(all, 0x20, 25);
		delay(delayTime);
		
		status = (1<<3);
		GYRO_IO_write(&status, CTRL_REG3, 1); // Enable Int2 Data ready
		delay(delayTime);
		
		GYRO_IO_Read(all, 0x20, 25);
		delay(delayTime);*/
	}
	/*while(1){
		
		GYRO_IO_Read(&status, L3GD20_STATUS_REG_ADDR, 1);
		
		if((status & 0x08) == 0x08) {
			GYRO_IO_Read(gyr, L3GD20_OUT_X_L_ADDR, 6);
			gyro_x = (int16_t) ((uint16_t) (gyr[1] <<8) + gyr[0]);
			gyro_y = (int16_t) ((uint16_t) (gyr[3] <<8) + gyr[2]);
			gyro_z = (int16_t) ((uint16_t) (gyr[5] <<8) + gyr[4]);

			//for 2000dps, 1 unit equals 70 millidegrees per second
			gyro.x = (float) gyro_x * 0.070f;
			gyro.y = (float) gyro_y * 0.070f;
			gyro.z = (float) gyro_z * 0.070f;
		}	
	}*/
}
