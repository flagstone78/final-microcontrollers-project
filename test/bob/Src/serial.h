#ifndef __serials /* __serial */
#define __serials

//#include "stm32l476xx.h"
#include "stm32l4xx_hal.h"
#include "peripheralHandles.h"



void UARTPinSetup(void);
void USART2_INIT(void);
void serial(uint8_t*, uint8_t);
void newline(void);
void SerialHex(uint8_t *txBuffer, uint8_t size);
void SerialNum(int32_t, char);

#endif /* __serial */
