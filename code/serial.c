#include "serial.h"
const char charList[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
uint8_t newL[] = {' ', ' ', '\n', '\r'};

void UARTPinSetup(){
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
	
}

void USART2_INIT(){
	UARTPinSetup();
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; //Enable USART 2 clock
	RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);  
	RCC->CCIPR |= (RCC_CCIPR_USART2SEL_0); //Select system clock as the clock source of USART 2
	
	USART2->CR1 &= ~USART_CR1_UE;			//Disable usart
	USART2->CR1 &= ~USART_CR1_M;			//Set data length to 8 bits
	USART2->CR2 &= ~USART_CR2_STOP;		//Select 1 stop bit
	USART2->CR1 &= ~USART_CR1_PCE;		//Set parity to no parity bit
	USART2->CR1 &= ~USART_CR1_OVER8;	//Set oversampling to 16
	USART2->BRR = (0x8A);										//Set baud rate to 9600 using APB frequency of 4MHz
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE); 	//Enable transmission and reception
	USART2->CR1 |= USART_CR1_UE;	//Enable usart
	while((USART2->ISR & USART_ISR_TEACK) == 0);//Verify that the USART is ready for transmission
	while((USART2->ISR & USART_ISR_REACK) == 0);//Verify that USART is ready for reception
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

void newline(){
	serial(newL,4);
}

void SerialHex(uint8_t *txBuffer, uint8_t size){
	uint8_t hex[size*4];
	for(int i = 0; i<size; i ++){
		hex[i*4] = ' ';
		hex[i*4+1] = 'x';
		uint8_t tens = txBuffer[i]/16;
		hex[i*4+2] = charList[tens%16];
		hex[i*4+3] = charList[txBuffer[i]%16];
	}
	serial(hex, size*4);
}

void serialPrintGyro(int32_t in, char label ){
	uint8_t hex[14];
	hex[0] = label;
	hex[1] = ':';
	hex[2] = ' ';
	if(in < 0){
		in = -in; //negate
		hex[3] = '-'; //minus sign
	} else {
		hex[3] = ' ';
	}
	
	uint8_t charNum;
	for(int i = 9; i>3; i--){
		charNum = in % 10;
		hex[i] = charList[charNum];
		in /= 10;
	}
	hex[10] = ' ';
	hex[11] = ' ';
	
	serial(hex, 12);
}
