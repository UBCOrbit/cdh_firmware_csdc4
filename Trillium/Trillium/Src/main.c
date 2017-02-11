/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
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

/**
 *
 * A UBC Orbit team software production
 * Project: Trillium Architecture V1
 *
 * This software is protected under a Creative Commons
 * Attribution-ShareAlike 4.0 license, summarized here:
 * http://creativecommons.org/licenses/by-sa/4.0/
 *
 * Description: Software for controlling radiation redundency in the Trillium architechture.
 *        Code designed for implementation on ATmega2560 chips operating at 16MHz.
 *              Additional functionality for host to send commands to the core to enable error
 *        simulation functionality
 *        This code is to be run on all three STMs, changing the STM_ID variable
 *        per STM (1, 2, 3)
 *
 * Original Author: Divya Bhudihal, Ro-ee Tal, Chenyi Zheng
 * Date Created: 21/01/2017
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define TRUE 1  //definition for boolean variables used in conditional statements
#define FALSE 0

//These characters are appended to the beginning and end of each string sent from the host
//In the below case, this represents the sequence AA0000000000000000YZ
#define CHAR1 'A'  //First Character
#define CHAR2 'A'  //Second Character
#define SECOND_CHAR 'Y' //second-to-last character (must be different than END_CHAR)
#define END_CHAR 'Z' //last character

#define STM_ID 1
#define BUFFER_SIZE 8
#define WAIT_TIME 50
#define RESET_TIME 500

#define timeOut 0xFFFF

////using buffers until structs working
//uint8_t bufferA[BUFFER_SIZE];
//uint8_t bufferB[BUFFER_SIZE];
//uint8_t bufferC[BUFFER_SIZE];

//Struct for each board, so only id changes for each board. Not using yet.
struct board{
	uint8_t data[BUFFER_SIZE];
	UART_HandleTypeDef *huart;
	int reset_Pin;
	char letter;
}STM_A, STM_B, STM_C;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
void STM_BOARD_Init(void);
void clearArray(uint8_t *buffer);
void votingArray(void);
int compare(uint8_t *bufferIn, char compare_cluster);
int readBoard(UART_HandleTypeDef *huart, uint8_t *buffer);
void resetPin(int pin);
void printStringToConsole(char message[]);
void printBufferToConsole(uint8_t *pData);
void printCompare(char compare_cluster, int comparison);
void writeOthers(void);
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	STM_BOARD_Init();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();

	/* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int count =0;
	while (1)
	{
		/* USER CODE END WHILE */

		while(count<3){
			votingArray();
			count++;

		}
//		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
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
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	//PA8: Sync_T
	/*Configure GPIO pins : LD2_Pin PA8 */
	GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Sync_R
	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void STM_BOARD_Init(void){
	switch(STM_ID) {
	case 1  :
		//board 1
		STM_A.reset_Pin = 1;
		STM_A.letter = 'A';
		STM_A.huart = &huart2;
		clearArray(STM_A.data);
		//board 2
		STM_B.reset_Pin = 2;
		STM_B.letter = 'B';
		STM_B.huart = &huart1;
		clearArray(STM_B.data);
		//board 3
		STM_C.reset_Pin = 3;
		STM_C.letter = 'C';
		STM_C.huart = &huart6;
		clearArray(STM_C.data);
		break; /* optional */
	case 2  :
		//board 2
		STM_A.reset_Pin = 2;
		STM_A.letter = 'A';
		STM_A.huart = &huart2;
		clearArray(STM_A.data);
		//board 3
		STM_B.reset_Pin = 3;
		STM_B.letter = 'B';
		STM_B.huart = &huart1;
		clearArray(STM_B.data);
		//board 1
		STM_C.reset_Pin = 1;
		STM_C.letter = 'C';
		STM_C.huart = &huart6;
		clearArray(STM_C.data);
		break; /* optional */
	case 3  :
		//board 3
		STM_A.reset_Pin = 3;
		STM_A.letter = 'A';
		STM_A.huart = &huart2;
		clearArray(STM_A.data);
		//board 2
		STM_B.reset_Pin = 2;
		STM_B.letter = 'B';
		STM_B.huart = &huart1;
		clearArray(STM_B.data);
		//board 1
		STM_C.reset_Pin = 1;
		STM_C.letter = 'C';
		STM_C.huart = &huart6;
		clearArray(STM_C.data);
		break; /* optional */
	}
}

void votingArray(void){
	//clear each of the buffers
	clearArray(STM_A.data);
	clearArray(STM_B.data);
	clearArray(STM_C.data);

	writeOthers();
	HAL_Delay(WAIT_TIME);

	if(readBoard(STM_B.huart, STM_B.data)){
		printStringToConsole("Got Data from B: ");
		printBufferToConsole(STM_B.data);
		printStringToConsole("\n");
	}else{
		printStringToConsole("Error reading buffer B.\n");
	}

	if(readBoard(STM_C.huart, STM_C.data)){
		printStringToConsole("Got Data from C: ");
		printBufferToConsole(STM_C.data);
		printStringToConsole("\n");
	}else{
		printStringToConsole("Error reading buffer C.\n");
	}

	int ab = compare(STM_B.data, STM_B.letter);
	int ac = compare(STM_C.data, STM_C.letter);

	//Sync_R high
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)>0.5){
		ab=TRUE;
		ac=TRUE;
	}

	if(ab==FALSE){
		resetPin(STM_B.reset_Pin);
	}

	if(ac==FALSE){
		resetPin(STM_C.reset_Pin);
	}

	//If there is a difference between the data received in any Arduino, then a reset was triggered.
	//We must wait for the other Arduinos to reset.
	if (ab || ac) {
		HAL_Delay(RESET_TIME); //Give time for the Arduinos to reset
	}

	//Drive reset pins back low
	//	digitalWrite(STM_B.reset_Pin, LOW);
	//	digitalWrite(STM_C.reset_Pin, LOW);

}

void writeOthers(void){
	HAL_UART_Transmit(STM_B.huart, STM_A.data, BUFFER_SIZE, timeOut);
	HAL_UART_Transmit(STM_C.huart, STM_A.data, BUFFER_SIZE, timeOut);
}

void printStringToConsole(char message[]){
	if(HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), timeOut)==HAL_OK){
	}
}

void printBufferToConsole(uint8_t *pData){
	if(HAL_UART_Transmit(&huart2, pData, BUFFER_SIZE, timeOut)==HAL_OK){
	}
}

void resetPin(int pin){
	char output[25];
	sprintf(output, "Driving pin to high: %d\n", pin);
	printStringToConsole(output);
	//	digitalWrite(pin, HIGH);
}

int compare(uint8_t *bufferIn, char compare_cluster){
	int out=TRUE;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		STM_A.data[i]='a';
		if (STM_A.data[i] != bufferIn[i]){
			out = FALSE;
		}
	}
	printCompare(compare_cluster, out);
	return out;
}

void printCompare(char compare_cluster, int comparison){
	char output[25];
	sprintf(output, "Compared A and %c", compare_cluster);
	printStringToConsole(output);
	if(comparison==TRUE){
		printStringToConsole(": Match.\n");
	}else{
		printStringToConsole(": No Match.\n");
	}
}

int readBoard(UART_HandleTypeDef *huart, uint8_t *buffer) {
	int out=FALSE;
	clearArray(buffer);
	if(HAL_UART_Receive(huart, buffer, BUFFER_SIZE, timeOut)==HAL_OK) {
//		if (buffer[0] == CHAR1 && buffer[1] == CHAR2&&buffer[BUFFER_SIZE-2]== SECOND_CHAR&&buffer[BUFFER_SIZE-1]==END_CHAR) {
			out = TRUE;
//		}
	}

	return out;
}

//ClearArray: This function writes null bytes to the buffer array passed to it
//Input: a is an array of characters (string)
//Outputs: none
void clearArray(uint8_t *buffer){
	for (int i = 0; i < BUFFER_SIZE; i++) {
		buffer[i] = '\0';
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
