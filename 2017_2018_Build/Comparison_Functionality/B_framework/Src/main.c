/**
******************************************************************************
* File Name          : main.c --- STM B Code
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
* Project: Trillium Architecture V2
*
* This software is protected under a Creative Commons
* Attribution-ShareAlike 4.0 license, summarized here:
* http://creativecommons.org/licenses/by-sa/4.0/
*
* Description: Software for controlling radiation redundency in the Trillium architechture.
*        Code designed for implementation on STM32 F401RE chips.
*              Additional functionality for host to send commands to the core to enable error
*        simulation functionality
*        This code is to be run on all three STMs, changing the STM_ID variable
*        per board (1, 2, 3)

* D1 and D0 - 2 is designated for receiving signals from source.
*
* GPIOB 4 (D5) - Send signal to reset it's B
* GPIOB 10 (D6) - Send signal to reset it's C
*
* SyncT - PA8
* SyncR - PB5
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
//Not in use at the moment.
ADC_HandleTypeDef hadc1;

// Communication Channels
UART_HandleTypeDef huart1; // STM_A
UART_HandleTypeDef huart2; // STM_C (Arduino for testing)
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//definition for boolean variables used in conditional statements
#define TRUE 1
#define FALSE 0

//Each board must have a different ID, and will have certain settings based on that ID.
//The settings are declared in a switch case in the STM_BOARD_Init
#define STM_ID 1

//Standard definitions for system
#define BUFFER_SIZE 64
#define WAIT_TIME 50
#define RESET_TIME 500
//Time used in Serial communication reading and sending
#define timeOut 0x0FFF

//Structure declaring board settings, which may vary depending on the board. Determined using STM_ID
struct board {
	uint8_t data[BUFFER_SIZE];
	UART_HandleTypeDef *huart;
	GPIO_TypeDef  *PinPort; //- No voting module now
	uint16_t Reset_Pin;
	char letter;
}STM_A, STM_B, STM_C;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // May need change ***
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void STM_BOARD_Init(void);
void clearArray(uint8_t *buffer);
void compareData(void);
void setPinLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void setPinHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void printStringToConsole(char message[]);
void printBufferToConsole(uint8_t *pData);
/* USER CODE END PFP */

// Compare data to B. Send result to C.
// Input: base index of memory, number of bytes to be compared
// Side-Effects: Request data from B. Receive and store B's data in virtual STM board (stm_B). Send result to C. Get power cycled.
// Output: none
// Assumptions: -Upon booting, A copies its data from C.
//				-A, B, and C have the exact same data[] array
void compareData(){
	char tempOutput[30];
	char temp[2] = "\0";
	int reqBytes = 2;
	uint8_t tempBuffer[BUFFER_SIZE];
	clearArray(tempBuffer);
	int count = 0;
	int baseIndex, numBytes;
	char baseIndex_s[8];
	char numBytes_s[8];

	// Wait until A sends the request string --------------------------
	int received = 0;
	while(received == 0){
		printStringToConsole("B: Waiting for A..\n");
		if(HAL_UART_Receive(&huart1, tempBuffer, reqBytes+1, timeOut) == HAL_OK)
			received = 1;
	}

	printStringToConsole("B: Received data from A\n");

	// Parse data request string ------------------------
	baseIndex_s[0] = tempBuffer[0];
	baseIndex_s[1] = '\0';

	/* Debug Messages */
	sprintf(tempOutput, "B: Base Address: %s\n", baseIndex_s);
	printStringToConsole(tempOutput);
	tempOutput[0] = '\0';


	// Store numBytes
	numBytes_s[0] = tempBuffer[1];
	numBytes_s[1] = '\0';

	/* Debug Messages */
	sprintf(tempOutput, "B: Size: %s\n", numBytes_s);
	printStringToConsole(tempOutput);
	tempOutput[0] = '\0';

	// Convert from bytes to int
	baseIndex = atoi(baseIndex_s);
	numBytes = atoi(numBytes_s);

	// Generate data array ------------------------
	uint8_t reqData[numBytes];

	// Transfer data from internal storage to msg buffer
	for(count = 0; count < numBytes; count++){
		reqData[count] = STM_B.data[baseIndex+count];
		temp[0] = (char)reqData[count];

		/* Debug Messages
		printStringToConsole("Letter stored.\n");
		printStringToConsole(temp);
		*/
	}

	// Delay for A to get ready
	HAL_Delay(500);

	// Send data to A ----------------------------
	printStringToConsole("B:Beginning transmission\n");
	if(HAL_UART_Transmit(&huart1, reqData, sizeof(reqData), timeOut) == HAL_OK){
		printStringToConsole("B: Sent data to A!\n");
	}
	else{
		printStringToConsole("B: Message could not be sent to A.\n");
		//return;
	}
	// Get reset or not --------------------------
}

/* USER CODE BEGIN 0 */
int main(void)
{
	//Set Tx pin high on boot
	setPinHigh(GPIOA, GPIO_PIN_8);

	//Initialize stm board settings on boot
	STM_BOARD_Init();

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

	//Set Tx pin low at the beginning of loop
	setPinLow(GPIOA, GPIO_PIN_8);
	printStringToConsole("B begin.\n");

	/* Operational Loop --------------------------------------- */
	// Initialize B's memory
	char *testString = "Ello!\n";
	int i;
	for(i=0; i<strlen(testString); i++){
		STM_B.data[i] = testString[i];
	}
	printStringToConsole("B initialized.\n");

	while (1)
	{
		//printStringToConsole("B: Comparison begun!\n");
		compareData();
	}
}

void printStringToConsole(char message[]) {
	if (HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), timeOut) == HAL_OK) {
	}
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

	/*Configure GPIO pins : LD2_Pin PA8 - SyncT pin*/
	GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB10 PB4 - Reset Pins*/
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 - SyncR Pin*/
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void STM_BOARD_Init(void){
	//Declare letter to identify boards for output messages.
	STM_A.letter = 'A';
	STM_B.letter = 'B';
	STM_C.letter = 'C';
	//Initialize serial ports for how the boards communicate.
	STM_A.huart = &huart2;
	STM_B.huart = &huart1;
	STM_C.huart = &huart6;
	//Clear data buffers.
	clearArray(STM_A.data);
	clearArray(STM_B.data);
	clearArray(STM_C.data);
	//Initialize reset ports and pins. A is not set.
	STM_B.Reset_Pin = GPIO_PIN_4;
	STM_B.PinPort = GPIOB;
	STM_C.Reset_Pin = GPIO_PIN_10;
	STM_C.PinPort = GPIOB;
}

//printBufferToConsole(): Prints the data in one of the buffers to the console.
//Input: the data buffer to print.
//Output: None.
void printBufferToConsole(uint8_t *pData){
	if(HAL_UART_Transmit(&huart2, pData, BUFFER_SIZE, timeOut)==HAL_OK){
	}
}

//setPinHigh(): sets a GPIO pin to High.
//Input: the pin and port.
//Output: None.
void setPinHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	char output[25];
	sprintf(output, "Driving pin to high: %u\n", GPIO_Pin);
	printStringToConsole(output);
}

//setPinLow(): sets a GPIO pin to low.
//Input: the pin and port.
//Output: None.
void setPinLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	char output[25];
	sprintf(output, "Driving pin to low: %u\n", GPIO_Pin);
	printStringToConsole(output);
}

//clearArray(): This function writes null bytes to the buffer array passed to it
//Input: buffer that needs to be cleared
//Output: None
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
