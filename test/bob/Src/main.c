
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define MPU6050_ADDR 0x69<<1
#define gyroSensitivity 250.0 //degrees per second
#define clkFreq 40000000 //Hz
#define timerFreq 1000.0 //Hz
#define timerDiv clkFreq/timerFreq
#define degPersecondPerUnit gyroSensitivity/32768.0
#define PI 3.14159265358979323846
#define convertGryoToMilliRad (degPersecondPerUnit*PI*1000)/(180*timerFreq)

uint8_t initializarionMSG[7] = "Hello\r\n";
const char charList[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
uint8_t newL[] = {' ', ' ', '\n', '\r'};

volatile int16_t accel_x, accel_y, accel_z;
volatile int16_t gyro_x, gyro_y, gyro_z;
uint8_t rawData[14];

double gyroAngle;
volatile double accelAngle, gyroRead, ierror = 0, derror = 0, olda = 0, angle = 0, speed = 0;

uint8_t txData[1] = {0x28}; //out_x_l for accel and gyro

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int deadzone = 0;
void setDirection(int dir){
	if(dir > deadzone){
		GPIOE->ODR |= (1U << 15 | 1U << 13); //set dir pin PE15 abd 13	
	} else if(dir < -deadzone) {
		GPIOE->ODR &= ~(1U << 15 | 1U << 13); //clear dir pin E15 and 13
	}
}

#define minTick 40
void setSpeed(double hertz){
	if(hertz < 0){ hertz = -hertz;}
	if(hertz == 0){hertz += 1;}
	
	unsigned int ticks = (clkFreq/(10*hertz));
	if(ticks < minTick){ticks = minTick;} //prevent too fast
	//else if(ticks > 0xFFFF){ticks = 0xFFFE;} //prevent overflow of LOAD register
	TIM2->ARR = ticks;
	if(TIM2->CNT > TIM2->ARR){
		TIM2->EGR |= TIM_EGR_UG;
	} //force event
} 

void step(void){
	GPIOE->ODR ^= ((1U << 12) | (1U << 14)); //toggle step pin PE12 and 14
}

void newline(){
	HAL_UART_Transmit(&huart2, newL, 4, 10); //init complete
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
	HAL_UART_Transmit(&huart2, hex, size*4, size*4); //init complete
}

void SerialNum(int32_t in, char label ){
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
	
	HAL_UART_Transmit(&huart2, hex, 12, 12); //init complete
}


void initMPU(void){
	uint8_t data = 0; //write 0 to power management register to wake MPU6050
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

void initBot(){
	HAL_UART_Transmit(&huart2, initializarionMSG, 7, 10); //init complete
	initMPU(); //INITALIZE MPU6050
	HAL_TIM_Base_Start_IT(&htim2); //start timer 2
	HAL_TIM_Base_Start_IT(&htim3); //start timer 3
}

void processData(void){
	//translate data
	accel_x = (int16_t) (((uint16_t) (rawData[0]) <<8) + rawData[1]);
	//accel_y = (int16_t) (((uint16_t) (rawData[2]) <<8) + rawData[3]);
	accel_z = (int16_t) (((uint16_t) (rawData[4]) <<8) + rawData[5]);
	
	//gyro_x = (int16_t) (((uint16_t) (rawData[8]) <<8) + rawData[9])+210;
	gyro_y = (int16_t) (((uint16_t) (rawData[10]) <<8) + rawData[11]);
	//gyro_z = (int16_t) (((uint16_t) (rawData[12]) <<8) + rawData[13])+550;
	
	//read data from MPU6050 non blocking so that it is ready for next time
	HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, rawData, 14);
	
	//data processing
	//accelAngle = 0.5*accelAngle + 0.5*(atan2(-accel_x, accel_z)*1000)+0; //tip forward is +
	accelAngle = 0.99*accelAngle + 0.01*((-(accel_x*1000)/17000)+20); //tip forward is +
	gyroRead = ((double)gyro_y*convertGryoToMilliRad); //scale to degrees
	gyroAngle +=  gyroRead;

	
	angle = (995*(angle+gyroRead) + 5*accelAngle)/1000;
	derror = .9*derror + 0.1*(olda-angle)*timerFreq; //derror = (new error - old error)/dt
	olda = angle;
	
	ierror += angle/timerFreq; //ierror += (new error - old error)*dt
	if(ierror > 1){ierror = 1;}
	else if(ierror < -1){ierror = -1;}
	
	speed = (angle*220)+(derror*2)+(ierror*6000); //220 2 6000
	setDirection(speed);
	//setSpeed(abs((int)angle)*35);
	//setSpeed((angle*240)+(derror*3)+(ierror*0)); //with just angle
	setSpeed(speed); //currently just p
}

void printStuff(void){
	//processData();
	//SerialNum(accel_x, 'x');
	////SerialNum(accel_y, 'y');
	//SerialNum(accel_z, 'z');
	
	////SerialNum(gyro_x, 'u');
	//SerialNum(gyro_y, 'g');
	////SerialNum(gyro_z, 'w');
	
	//SerialNum(gyroAngle, 'g');
	//SerialNum(accelAngle, 'c');
	//SerialNum(angle, 'a');
	
	SerialNum(angle*220, 'p');
	SerialNum(ierror*6000, 'i');
	SerialNum(derror*2, 'd');
	SerialNum(speed, 's');
	newline();
}

//timer events
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		step();
	}
	if(htim->Instance == TIM3){		
		processData();
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	initBot();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//uint8_t whoami = 0;
	//int HALst = 0;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		printStuff();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00301347;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 39;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE12 PE13 PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
