#ifndef __serial /* __serial */
#define __serial

#include "stm32l476xx.h"

void UARTPinSetup(void);
void USART2_INIT(void);
void serial(uint8_t*, uint8_t);
void newline(void);
void SerialHex(uint8_t *txBuffer, uint8_t size);
void serialPrintGyro(int32_t, char);

#endif /* __serial */
