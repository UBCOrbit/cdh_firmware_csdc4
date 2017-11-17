/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//definition for boolean variables used in conditional statements
#define TRUE 1
#define FALSE 0

//Each board must have a different ID, and will have certain settings based on that ID.
//The settings are declared in a switch case in the STM_BOARD_Init
#define STM_ID 1

//Standard definitions for system wait time
#define BUFFER_SIZE 64
#define WAIT_TIME 50
#define RESET_TIME 500
//Time used in Serial communication reading and sending
#define timeOut 0x0FFF

//Structure declaring board settings, which may vary depending on the board. Determined using STM_ID
struct board {
	uint8_t data[BUFFER_SIZE];
	char letter;
}STM_A, STM_B, STM_C;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void STM_BOARD_Init(void);
void printStringToConsole(char message[]);
int getSignalData(char id, int baseIndex, int numBytes);
void compareData(int baseIndex, int numBytes);
void clearArray(uint8_t *buffer);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  /* USER CODE BEGIN 2 */
  char *testString = "Hello!\n";
  int i;
  for(i=0; i<strlen(testString); i++){
  	STM_A.data[i] = testString[i];
  }
  printStringToConsole("STM A Initialized!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  compareData(0, strlen("Hello!\n")*sizeof(char));
	  printStringToConsole("Comparison complete.\n");
	  HAL_Delay(500);
  /* USER CODE BEGIN 3 */

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
    _Error_Handler(__FILE__, __LINE__);
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

   /*Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

/* USER CODE BEGIN 4 */
void STM_BOARD_Init(void){
	//Declare letter to identify boards for output messages.
	STM_A.letter = 'A';
	STM_B.letter = 'B';
	STM_C.letter = 'C';
	//Clear data buffers.
	clearArray(STM_A.data);
	clearArray(STM_B.data);
	clearArray(STM_C.data);
}

void printStringToConsole(char message[]) {
	if (HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), timeOut) == HAL_OK) {
	}
}

// Description: Compare data with B. Send result over to C for reset if needed.
// Input: base index of memory, number of bytes to be compared
// Side-Effects: Request data from B. Receive and store B's data. Send result to C. Get power cycled.
// Assumptions: -Upon booting, A copies its data from C.
//				-A, B, and C have the exact same data[] array
void compareData(int baseIndex, int numBytes){
	printStringToConsole("A: Comparison begun.\n");
	char tempOutput[30];

	// Generate request string for B
	char addressString[8] = "";
	itoa(baseIndex, addressString, 10);
	char sizeString[8] = "";
	itoa(numBytes, sizeString, 10);
	char fullString[16] = "";

	strcpy(fullString, addressString);
	strcat(fullString, sizeString);

	// Send request to B
	int sending = 0;
	while (sending == 0) {
		if (HAL_UART_Transmit(&huart1, (uint8_t*)fullString, (uint16_t)sizeof(fullString), timeOut) == HAL_OK)
			sending = 1;
		HAL_Delay(50);
	}

	// Wait for data from B
	int received = 0;
	while(received == 0){
		printStringToConsole("A: Waiting for B data..\n");
		if(getSignalData('B', baseIndex, numBytes))
			received = 1;
	}

	printStringToConsole("A: Received B data\n");

	// Compare received B data with A data
	int i = 0;
	int comp_result = 1;
	for(i=0; i<numBytes; i++){
		if(STM_A.data[baseIndex + i] != STM_B.data[baseIndex + i])
			comp_result = 0;
	}

	// General result message for C
	char result[2];
	itoa(comp_result, result, 10);
	result[1] = '\0';

	// Send result string to C
	if(HAL_UART_Transmit(&huart6, (uint8_t*)result, strlen(result), timeOut) == HAL_OK){
		printStringToConsole(tempOutput);
		tempOutput[0] = '\0';
	}
	else
		printStringToConsole("A: Could not transmit message.\n ");
}

// Receive data from another STM
// Input: stm id, index to store the data at
// Side-Effects: Read the UART status register to check if bytes have been received. Try to read numBytes of them.
// Output: 0 if data is not read successfully. 1 if success.
int getSignalData(char id, int baseIndex, int numBytes) {
	//initial variable setup
	// Wait to initialize variables
	uint8_t tempBuffer[BUFFER_SIZE];
	clearArray(tempBuffer);
	/*
	int endCheck = FALSE;
	int startCheck = FALSE;
	*/
	int stmCount = 0;

	char byte_s[8];
	itoa(numBytes, byte_s, 10);

	// Check status register for numBytes bytes, store them in tempBuffer.
	// If successfully received specified number of bytes, return HAL_OK
	// Wait HAL_MAX_DELAY ms before timeOut, then return HAL_TIMEOUT
	// **change back to huart1 for B
	if (HAL_UART_Receive(&huart1, tempBuffer, numBytes, timeOut) == HAL_OK) {

		// Store address and numbytes
		printStringToConsole("A: Inside receive..\n");
		while (stmCount <= numBytes + 1) {
			STM_B.data[baseIndex + stmCount] = tempBuffer[stmCount];
			//printStringToConsole("One byte stored.\n");
			char currChar[2] = "\0";
			currChar[0] = (char)tempBuffer[stmCount];
			printStringToConsole(currChar);
			stmCount++;
		}
	}
	else {
		// printStringToConsole("A: Did not receive correct number of bytes.\n");
		return FALSE;
	}

	// Append null char
	STM_B.data[baseIndex + stmCount] = '\0';
	printStringToConsole("A: Finished comparison.\n");
	//if the buffer also had the last two check chars (which means it had to have had the first two), then proceed.
	return TRUE;
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
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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