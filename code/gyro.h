#ifndef __gyro /* __gyro */
#define __gyro

#include "stm32l476xx.h" 
#include "spi.h"

#define L3GD20_STATUS_REG_ADDR 	0x27
#define L3GD20_OUT_X_L_ADDR	0x28

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

//void WriteGyroRegister(int , int);
//void GYRO_IO_Read(uint8_t *pBuffer, uint8_t ReadADDR, uint8_t size);
void GYRO_IO_Read(uint8_t readADDR, unsigned int size, uint8_t *rxBuffer);
void GYRO_IO_Write(uint8_t writeAddress, uint8_t size, uint8_t *txBuffer);
	
#endif /* __gyro */
