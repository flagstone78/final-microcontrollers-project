#include "stm32l476xx.h"
#include "serial.h"
#include "spi.h"
#include "gyro.h"
#include "Accelerometer.h"
#include "stepper.h"
#include "systick.h"
#include <math.h>
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
volatile double angle = 0;
volatile double gyroAngle = 0;
volatile double accelAngle = 0;
volatile double intAngle = 0;

#define gyroSensitivity 250.0 //degrees per second
#define clkFreq 40000000 //Hz
#define timerFreq 500.0 //Hz
#define timerDiv clkFreq/timerFreq
#define degPersecondPerUnit gyroSensitivity/32768.0
#define PI 3.14159265358979323846
#define convertGryoToMilliRad (degPersecondPerUnit*PI*1000)/(180*timerFreq)
 
#define minTick 30
void setSpeed(double hertz){
	if(hertz < 0){ hertz = -hertz;}
	
	unsigned int ticks = (clkFreq/hertz);
	if(ticks < minTick){ticks = minTick;} //prevent too fast
	//else if(ticks > 0xFFFF){ticks = 0xFFFE;} //prevent overflow of LOAD register
	TIM2->ARR = ticks;
	if(TIM2->CNT > TIM2->ARR){
		TIM2->EGR |= TIM_EGR_UG;
	} //force event
} 
 
void timer3setup(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; //Enable timer 3 clock; 
	TIM3->CR1 &= ~TIM_CR1_DIR; //Set Count direction up; 
	TIM3->PSC &= ~TIM_PSC_PSC; //Clear timer prescaler; 
	TIM3->PSC |= 0 & TIM_PSC_PSC; //0 prescaler
	TIM3->ARR = timerDiv; //Set auto reload register 2ms for 500hz; 
	
	TIM3->DIER |= TIM_DIER_UIE; //TIM_DIER_CC1IE; //Enable timer 3 interrupt;
	TIM3->CR1 |= TIM_CR1_CEN; //Enable timer 3;
	//NVIC_SetPriority(TIM3_IRQn, (1<<__NVIC_PRIO_BITS)-1); //set systick priority to least important //check if nescessary
	NVIC_EnableIRQ(TIM3_IRQn);
}

uint8_t alldata[33];
volatile int16_t accel_x = 0;
volatile int16_t accel_y = 0;
volatile int16_t accel_z = 0;

void TIM3_IRQHandler(){//timer A interrupt (start ADC at 2ms)

	//if((TIM3->SR & TIM_SR_UIF) != 0){ //clear interrupt
	TIM3->SR &= ~TIM_SR_UIF;
	//}
	
	uint8_t status = 0;
	
	//GYRO
	double gyroRead = 0;
	
	GYRO_IO_Read(STATUS_REG, 1, &status);
	if((status & 0x08) == 0x08) {
		int16_t gyro_x = 0;
		uint8_t gyr[6] = {0};
		
		GYRO_IO_Read(OUT_X_L, 2, gyr);
		gyro_x = (int16_t) ((uint16_t) (gyr[1] <<8) + gyr[0])+59; //130
		
		gyroRead = ((double)gyro_x*convertGryoToMilliRad); //scale to degrees
		gyroAngle +=  gyroRead;
	}
	
	//ACCEL

	ACCEL_IO_Read(STATUS_REG_A, 1, &status);

	if((status & 0x08) == 0x08) {
		uint8_t acc[6] = {0};
		int16_t accel_y = 0;
		int16_t accel_z = 0;
		
		ACCEL_IO_Read(OUT_Y_L_A, 1, &acc[2]);
		
		ACCEL_IO_Read(OUT_Y_H_A, 1, &acc[3]);
		
		ACCEL_IO_Read(OUT_Z_L_A, 1, &acc[4]);
		
		ACCEL_IO_Read(OUT_Z_H_A, 1, &acc[5]);

		accel_y = (int16_t) ((uint16_t) (acc[3] <<8) + acc[2]);
		accel_z = (int16_t) ((uint16_t) (acc[5] <<8) + acc[4]);

		accelAngle = (atan2(-accel_z, accel_y)*1000);//-48; //tip forward is +
	}

	/*ACCEL_IO_Read(STATUS_REG_A, 1, &status);
	if((status & 0x08) == 0x08) {
		for(int i = 0; i<33; i++){
			ACCEL_IO_Read(ACT_THS_A+i, 1, &alldata[i]);
		}
		accel_x = (int16_t) (((uint16_t) (alldata[11]) <<8) + alldata[10]);
		accel_y = (int16_t) (((uint16_t) (alldata[13]) <<8) + alldata[12]);
		accel_z = (int16_t) (((uint16_t) (alldata[15]) <<8) + alldata[14]);
  }
	accelAngle = (atan2(-accel_z, accel_y)*1000);//-48; //tip forward is +
	*/
	angle = (995*(angle+gyroRead) + 5*accelAngle)/1000;
	intAngle += angle; //I part
	
	setDirection(angle);
	//setSpeed(abs((int)angle)*35);
	setSpeed((angle*100)+(intAngle*0.0)-(gyroRead*0));
}

void Timer2setup(){
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Enable timer 2 clock; 
	TIM2->CR1 &= ~TIM_CR1_DIR; //Set Count direction up; 
	TIM2->PSC &= ~TIM_PSC_PSC; //Clear timer prescaler; 
	TIM2->PSC |= (3 & TIM_PSC_PSC); //Set timer prescaler to 100; 
	TIM2->ARR = 2000; //Set auto reload register; 
	
	// TIM2->BDTR |= TIM_BDTR_MOE; //main output enable;
	// TIM2->CCMR1 &= ~TIM_CCMR1_OC1M; //Clear output and compare bits for channel 1;
	// TIM2->CCMR1 |= TIM_CCMR_OC1M_0 | TIM_CCMR_OC1M_1; //Select toggle mode;
	// TIM2 &= ~TIM1_CCER_CC1NP; //Select output polarity to active high;
	// TIM2->CCER |= TIM1_CCER_CC1NE; //enable output for channel 1 to complementary output;
	//NVIC_SetPriority(TIM2_IRQn,0);
	TIM2->DIER |= TIM_DIER_UIE; //TIM_DIER_CC1IE; //Enable timer 2 interrupt;
	TIM2->CR1 |= TIM_CR1_CEN; //Enable timer 2;
	NVIC_EnableIRQ(TIM2_IRQn); //Enable interrupt
	
}

void TIM2_IRQHandler(){//timer B interrupt (update buzzer frequency 500ms)
	//if((TIM2->SR & TIM_SR_UIF) != 0){ //clear interrupt flag
	TIM2->SR &= ~TIM_SR_UIF;
	//}
	step();
}



int main(void){
	//clock change
	
	//RCC->CR |= (RCC_CR_HSION);
	//while( (RCC->CR & RCC_CR_HSIRDY) == 0);
	//RCC->CR &= ~(RCC_CR_MSION);
	RCC->CR &= ~(RCC_CR_MSIRANGE);
	RCC->CR |= (RCC_CR_MSIRANGE_8);
	RCC->CR |= (RCC_CR_MSIRGSEL);
	
	stepper_Init();
	
	//SPI_Init();
	
	USART2_INIT();
	uint8_t hi[2] = {'h','i'};
	serial(hi,2);
	
	GYRO_Init();
	ACCEL_Init();
	
	timer3setup(); //reading values
	//Timer2setup(); //stepping
	
	while(1){
		//SerialHex(alldata, 33);
			
		//serialPrintGyro(accel_x, 'x');
		//serialPrintGyro(accel_y, 'y');
		//serialPrintGyro(accel_z, 'z');
		
		if(0){
			serialPrintGyro(accelAngle, 'c');
			serialPrintGyro(gyroAngle, 'g');
			serialPrintGyro(angle, 'a');
		}
		//printAllGyro();

		printAllAccel();
		
		newline();
		
		//printAllAccelAxis();
	}

}


