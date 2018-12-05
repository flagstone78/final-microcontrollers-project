#ifndef __accel /* __gyro */
#define __accel

#include "stm32l476xx.h" 
#include "spi.h"
#include "serial.h"

#define LSM303CTR_CS_LOW 	    GPIOE->ODR &= ~(1U << 0)
#define LSM303CTR_CS_HIGH 	  GPIOE->ODR |= (1U << 0) 

//L3GD20 gyro
#define WHO_AM_I			0x0f
#define ACT_THS_A			0x1E
#define ACT_DUR_A			0x1F
#define CTRL_REG1_A		0x20
#define CTRL_REG2_A		0x21
#define CTRL_REG3_A		0x22
#define CTRL_REG4_A		0x23
#define CTRL_REG5_A		0x24
#define CTRL_REG6_A		0x25
#define CTRL_REG7_A		0x26
#define STATUS_REG_A	0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2a
#define OUT_Y_H_A			0x2b
#define OUT_Z_L_A			0x2c
#define OUT_Z_H_A			0x2d
#define FIFO_CTRL 		0x2e
#define FIFO_SRC	 		0x2f
#define IG_CFG1_A 		0x30
#define IG_SRC1_A 		0x31
#define IG_THS_X1_A		0x32
#define IG_THS_Y1_A		0x33
#define IG_THS_Z1_A		0x34
#define IG_DUR1_A			0x35
#define IG_CFG2_A			0x36
#define IG_SRC2_A			0x37
#define IG_THS2_A			0x38
#define IG_DUR2_A			0x39
#define XL_REFERENCE	0x3A
#define XH_REFERENCE	0x3B
#define YL_REFERENCE	0x3C
#define YH_REFERENCE	0x3D
#define ZL_REFERENCE	0x3E
#define ZH_REFERENCE	0x3F

void ACCEL_Init(void);
void ACCEL_IO_Read(uint8_t readADDR, unsigned int size, uint8_t *rxBuffer);
void ACCEL_IO_Write(uint8_t writeAddress, uint8_t size, uint8_t *txBuffer);
void printAllAccel(void);
	
#endif /* __accel */
