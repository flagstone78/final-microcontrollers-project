#include "stm32l476xx.h"
#include "serial.h"
#include "spi.h"
#include "gyro.h"
#include "Accelerometer.h"
#include "stepper.h"
#include "systick.h"
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
 
void timerAsetup(){
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; //Enable timer 3 clock; 
	TIM3->CR1 &= ~TIM_CR1_DIR; //Set Count direction up; 
	TIM3->PSC &= ~TIM_PSC_PSC; //Clear timer prescaler; 
	TIM3->PSC |= 1 & TIM_PSC_PSC; 
	TIM3->ARR = 8000; //Set auto reload register 2ms; 
	
	// TIM2->BDTR |= TIM_BDTR_MOE; //main output enable;
	// TIM2->CCMR1 &= ~TIM_CCMR1_OC1M; //Clear output and compare bits for channel 1;
	// TIM2->CCMR1 |= TIM_CCMR_OC1M_0 | TIM_CCMR_OC1M_1; //Select toggle mode;
	// TIM2 &= ~TIM1_CCER_CC1NP; //Select output polarity to active high;
	// TIM2->CCER |= TIM1_CCER_CC1NE; //enable output for channel 1 to complementary output;
	//NVIC_SetPriority(TIM3_IRQn, 1);
	
	TIM3->DIER |= TIM_DIER_UIE; //TIM_DIER_CC1IE; //Enable timer 3 interrupt;
	TIM3->CR1 |= TIM_CR1_CEN; //Enable timer 3;
	NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(){//timer A interrupt (start ADC at 2ms)
	int16_t gyro_x = 0;
	uint8_t gyr[6];
	//if((TIM3->SR & TIM_SR_UIF) != 0){ //clear interrupt
	TIM3->SR &= ~TIM_SR_UIF;
	//}
	uint8_t status = 0;
	GYRO_IO_Read(STATUS_REG, 1, &status);
	if((status & 0x08) == 0x08) {
		GYRO_IO_Read(OUT_X_L, 2, gyr);
			
			//GYRO_IO_Read(gyr, L3GD20_OUT_X_L_ADDR, 6);
		gyro_x = (int16_t) ((uint16_t) (gyr[1] <<8) + gyr[0])+130;
			//gyro_y = (int16_t) ((uint16_t) (gyr[3] <<8) + gyr[2])-80;
			//gyro_z = (int16_t) ((uint16_t) (gyr[5] <<8) + gyr[4]);

			//agyro_x = (((int32_t)gyro_x) + agyro_x*9)/10;
			//agyro_y = (((int32_t)gyro_y) + agyro_y*9)/10;
			//agyro_z = (((int32_t)gyro_z) + agyro_z*9)/10;
			

			
			//serialPrintGyro(angle, 'x');
			//serialPrintGyro(gyro_x, 'x');
			//serialPrintGyro(gyro_y, 'y');
			//serialPrintGyro(gyro_z, 'z');
			//newline();
		angle += ((double)gyro_x*.1525); //scale to degrees
		setDirection(angle);
		serialPrintGyro(angle, 'a');
		newline();
			//Systick_Freq_Update(abs(angle));
		}
}

int main(void){
	//clock change
	//RCC->CR |= (RCC_CR_HSION);
	//while( (RCC->CR & RCC_CR_HSIRDY) == 0);
	stepper_Init();
	
	//Systick_Initialization();
	
	SPI_Init();
	
	USART2_INIT();
	uint8_t hi[2] = {'h','i'};
	serial(hi,2);
	delay(100000);
	
	GYRO_Init();
	ACCEL_Init();
	timerAsetup();
	
	uint8_t status = 0;

	//raw gyro data
	uint8_t acc[6]; //raw accel data
	
	 //, gyro_y = 0, gyro_z = 0;
	int16_t accel_x = 0, accel_y = 0, accel_z = 0;
	//int32_t agyro_x = 0, agyro_y = 0, agyro_z = 0; //average
	
	setDirection(1); //make both wheels go the same direction
	
	
	
	while(1){
		//printAllGyro();
		//printAllAccel();
		
		//GYRO
	
		
		//ACCEL
		ACCEL_IO_Read(STATUS_REG_A, 1, &status);
		if((status & 0x08) == 0x08) {
			ACCEL_IO_Read(OUT_X_L_A, 1, &acc[0]);
			ACCEL_IO_Read(OUT_X_H_A, 1, &acc[1]);
			ACCEL_IO_Read(OUT_Y_L_A, 1, &acc[2]);
			ACCEL_IO_Read(OUT_Y_H_A, 1, &acc[3]);
			ACCEL_IO_Read(OUT_Z_L_A, 1, &acc[4]);
			ACCEL_IO_Read(OUT_Z_H_A, 1, &acc[5]);
			//SerialHex(acc, 6);
			//GYRO_IO_Read(gyr, L3GD20_OUT_X_L_ADDR, 6);
			accel_x = (int16_t) ((uint16_t) (acc[1] <<8) + acc[0]);
			accel_y = (int16_t) ((uint16_t) (acc[3] <<8) + acc[2]);
			accel_z = (int16_t) ((uint16_t) (acc[5] <<8) + acc[4]);

			//agyro_x = (((int32_t)gyro_x) + agyro_x*9)/10;
			//agyro_y = (((int32_t)gyro_y) + agyro_y*9)/10;
			//agyro_z = (((int32_t)gyro_z) + agyro_z*9)/10;

			//angle += (98*(int)angle + 2*(int)accel_y)/100;
			serialPrintGyro(angle, 'a');
			
			serialPrintGyro(accel_x, 'x');
			serialPrintGyro(accel_y, 'y');
			serialPrintGyro(accel_z, 'z');
			newline();
			
		}
		
	}
}
