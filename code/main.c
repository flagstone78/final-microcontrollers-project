#include "stm32l476xx.h"
#include "serial.h"
#include "spi.h"
#include "gyro.h"
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



int main(void){
	//clock change
	//RCC->CR |= (RCC_CR_HSION);
	//while( (RCC->CR & RCC_CR_HSIRDY) == 0);
	stepper_Init();
	Systick_Initialization();
	
	SPI_Init();
	
	USART2_INIT();
	uint8_t hi[2] = {'h','i'};
	serial(hi,2);
	delay(100000);
	
	//WriteGyroRegister(CTRL_REG1,0x3f); // power up gyro at full speed, all axes.
	//WriteGyroRegister(CTRL_REG3,BIT3); // Enable Int2 Data ready
  WriteGyroRegister(CTRL_REG1,0xff);    // power up gyro at full speed, all axes.
	
	uint32_t delayTime = 100;
	
	uint8_t status = 0;

	uint8_t gyr[6];
	int16_t gyro_x = 0, gyro_y = 0, gyro_z = 0;
	int32_t agyro_x = 0, agyro_y = 0, agyro_z = 0; //average
	
	while(1){
		
		GYRO_IO_Read(&status, L3GD20_STATUS_REG_ADDR, 1);
		
		if((status & 0x08) == 0x08) {
			for(int i = 0; i<6; i++){
				GYRO_IO_Read(&status, OUT_X_L+i, 1);
				//status = spi_read4Wire(OUT_X_L+i);
				//SerialHex(&status,1);
			
				delay(delayTime);
				gyr[i] = status;
			}
			//newline();
			
			//GYRO_IO_Read(gyr, L3GD20_OUT_X_L_ADDR, 6);
			gyro_x = (int16_t) ((uint16_t) (gyr[1] <<8) + gyr[0])+90;
			gyro_y = (int16_t) ((uint16_t) (gyr[3] <<8) + gyr[2])-80;
			gyro_z = (int16_t) ((uint16_t) (gyr[5] <<8) + gyr[4]);

			agyro_x = (((int32_t)gyro_x) + agyro_x*9)/10;
			agyro_y = (((int32_t)gyro_y) + agyro_y*9)/10;
			agyro_z = (((int32_t)gyro_z) + agyro_z*9)/10;
			
			//for 2000dps, 1 unit equals 70 millidegrees per second
			//for 250dps, 1 unit equals 3.814 millidegrees per second
			
			//serialPrintGyro(agyro_x, 'x');
			//serialPrintGyro(agyro_y, 'y');
			//serialPrintGyro(agyro_z, 'z');
			//newline();
			setDirection(agyro_x);
		}	
	}
}
