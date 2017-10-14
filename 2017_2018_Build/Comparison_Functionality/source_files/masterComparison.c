/**
******************************************************************************
* File Name          : masterComparison.c
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
* Description: Comparison function for the Master STM.
* Original Author: Carter Fang
* Date Created: 2017/10/14
*
* Relevant Pins:
* D2 - uart1 RX
* D8 - uart1 TX
* D# - uart2 RX
* D# - uart2 TX
* <other data connections for COMMS, ADCS, PAYLOAD, etc..>

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
//Not in use at the moment.
ADC_HandleTypeDef hadc1;

// Communication Channels - fill this in
UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//definition for boolean variables used in conditional statements
#define TRUE 1
#define FALSE 0

//These characters are appended to the beginning and end of each string sent from the host this is just
//In the below case, this represents the sequence AA0000000000000000YZ
#define INITCHAR1 'X'
#define INITCHAR2 'X'
#define MIDCHAR1 'Y'
#define MIDCHAR2 'Y'
#define ENDCHAR1 'Z'
#define ENDCHAR2 'Z'

//Each board must have a different ID, and will have certain settings based on that ID.
//The settings are declared in a switch case in the STM_BOARD_Init
#define STM_ID 1

//Standard definitions for system
#define BUFFER_SIZE 128 // Expand this maybe?
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
int compare(uint8_t *bufferIn, char compare_cluster);
int readBoard(UART_HandleTypeDef *huart, uint8_t *buffer);
void setPinLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void setPinHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void printStringToConsole(char message[]);
void printBufferToConsole(uint8_t *pData);
void printCompare(char compare_cluster, int comparison);
int getSignalData(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	//Set Tx pin high on boot
	setPinHigh(GPIOA, GPIO_PIN_8);
	//Initialize stm board settings on boot
	STM_BOARD_Init();
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
	//Set Tx pin low at the beginning of loop
	setPinLow(GPIOA, GPIO_PIN_8);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		//Run voting array
		if (getSignalData() == TRUE) {
			printStringToConsole("Received Data.");
			//printBufferToConsole("");
		}
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */

}

/* USER CODE END 0 */

void compareData(int baseIndex, int numBytes){
	// Format of string from A->B, XX--address--YY--size--ZZ
	// Format of string from B->A, XX--data--ZZ
	// Format of string from C->A, XX--data-ZZ

	// Form string
	char* addressString = itoa(baseIndex);
	char* sizeString = itoa(numBytes);
	char* fullString = (char*)malloc(sizeof(addressString) + 3*sizeof("XX") + sizeof(sizeString));
	fullString = "XX";

	// Append address
	strcat(fullString, addressString);
	strcat(fullString,"YY");

	// Append numBytes
	strncat(fullString,sizeString);
	strcat(fullString,"ZZ");

	// Send data to B
	HAL_UART_Transmit(&huart, fullString, sizeof(fullString), timeOut);
	free(fullString);

	// Receive data from B
	int received = 0;
	while(!received){
		received = getSignalData('B');
	}

	// Compare data
	int i = 0;
	int comp_result = 1;
	for(i=0; i<numBytes; i++){
		if(stm_A.data[baseIndex + i] != stm_B.data[baseIndex + i])
			comp_result = 0;
	}

	// Format result string
	char* result = itoa(comp_result);
	char* resString = (char*)malloc(sizeof(result) + 2*sizeof("XX"));
	resString = "XX";
	strcat(resString, result);
	strcat(resString, "ZZ");

	// Send result string to C
	HAL_UART_Transmit()
	// Get reset or not
}

int getSignalData(char id, int baseIndex) {
	//initial algorithm setup
	HAL_Delay(WAIT_TIME);
	uint8_t tempBuffer[BUFFER_SIZE];
	clearArray(tempBuffer);
	int endCheck = FALSE;
	int midCheck = FALSE;
	int startCheck = FALSE;
	int count = 1;
	int stmCount = 0;
	uint8_t huart*;

	// Check if receiving data from B or C
	if(id == 'B')
		huart = huart1;
	else
		huart = huart2;

	//check if receiving serial communication from sub-system
	//Assumed to be board B
	if (readBoard(&huart, tempBuffer)) {
		//loop through read buffer
		while (count<BUFFER_SIZE - 1 && endCheck != TRUE) {
			//INITCHAR detection
			if (tempBuffer[count - 1] == INITCHAR1 && tempBuffer[count] == INITCHAR2) {
				startCheck = TRUE;
			}
			else if (startCheck) {
				//MIDCHAR detection
				if (tempBuffer[count] == ENDCHAR1 && tempBuffer[count + 1] == ENDCHAR2) {
					endCheck = TRUE;
				}

				else{
					STM_B.data[baseIndex + stmCount] = tempBuffer[count];
					stmCount++;
				}
			}
			count++;
		}
	}

	// Append null char
	STM_B.data[baseIndex + stmCount] = '\0';
	//if the buffer also had the last two check chars (which means it had to have had the first two), then proceed.
	if (endCheck == TRUE) {
		printStringToConsole("Got data from B");
		printBufferToConsole(STM_B.data[baseIndex]);
		printStringToConsole("\n");
	}
	else {
		printStringToConsole("Error receiving data from source!");
	}
	return endCheck;
}


void printStringToConsole(char message[]) {
	if (HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), timeOut) == HAL_OK) {
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

void votingArray(void){
	//clear each of the buffers
	clearArray(STM_A.data);
	clearArray(STM_B.data);
	clearArray(STM_C.data);
	//First check if the host STM received data from a sub-system.
	if(getSignalData()==TRUE){
		//send host data to other STMs
		writeOthers();
		//Read what the other STMs received
		int readB = readBoard(STM_B.huart, STM_B.data);
		int readC = readBoard(STM_C.huart, STM_C.data);
		if(readB){
			printStringToConsole("Got Data from B: ");
			printBufferToConsole(STM_B.data);
			printStringToConsole("\n");
		}else{
			printStringToConsole("Error reading buffer B.\n");
		}
		if(readC){
			printStringToConsole("Got Data from C: ");
			printBufferToConsole(STM_C.data);
			printStringToConsole("\n");
		}else{
			printStringToConsole("Error reading buffer C.\n");
		}
		//if there was no error reading STMs, continue.
		if(readB&&readC){
			//Compare host data to received data.
			int ab = compare(STM_B.data, STM_B.letter);
			int ac = compare(STM_C.data, STM_C.letter);
			//Check if Sync_R is high: Means other board is booting, so must not reset again.
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)>0.5){
				ab=TRUE;
				ac=TRUE;
			}
			//send reset if discrepancy in data comparison.
			if(ab==FALSE){
				setPinHigh(STM_B.PinPort, STM_B.Reset_Pin);
			}
			if(ac==FALSE){
				setPinHigh(STM_C.PinPort, STM_C.Reset_Pin);
			}
			//If there is a difference between the data received in any STMs, then a reset was triggered.
			//We must wait for the other STMs to reset.
			if (ab==FALSE || ac==FALSE) {
				HAL_Delay(RESET_TIME); //Give time for the STMs to reset
			}
			//Drive reset pins back low
			setPinLow(STM_B.PinPort, STM_B.Reset_Pin);
			setPinLow(STM_C.PinPort, STM_C.Reset_Pin);
		}
	}
}

//getSignalData(): this function reads the data from the host
//Inputs: none
//Outputs: Boolean value indicating whether signal data was successfully read or not
//Has not been tested yet.

//writeOthers(): Sends the data in buffer to other boards.
//Input: None.
//Output None.
void writeOthers(void){
	HAL_UART_Transmit(STM_B.huart, STM_A.data, BUFFER_SIZE, timeOut);
	HAL_UART_Transmit(STM_C.huart, STM_A.data, BUFFER_SIZE, timeOut);
	HAL_Delay(WAIT_TIME);
}


//printBufferToConsole(): Prints the data in one of the buffers to the console.
//Input: the data buffer to print.
//Output: None.
void printBufferToConsole(uint8_t *pData){
	if(HAL_UART_Transmit(&huart6, pData, BUFFER_SIZE, timeOut)==HAL_OK){
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

//compare():compares data buffer with that of another stm.
//Input: Which other data buffer it is comparing with and an identification char.
//Output: Boolean value - True: Comparison was a match.
int compare(uint8_t *bufferIn, char compare_cluster){
	int out=TRUE;
	for (int i = 0; i < strlen((const char*)STM_A.data)+1; i++) {
		if (STM_A.data[i] != bufferIn[i]){
			out = FALSE;
		}
	}
	printCompare(compare_cluster, out);
	return out;
}

//printCompare(): Prints the results of the comparison done in the voting array.
//Input: Which two stms were compared and the result.
//Output: nothing.
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

//readBoard(): Writes incoming serial communicated to data buffer.
//Input: The serial communication channel and the buffer to be saved to.
//Output: Boolean value - True: Successfully read to buffer.
int readBoard(UART_HandleTypeDef *huart, uint8_t *buffer) {
	if(HAL_UART_Receive(huart, buffer, BUFFER_SIZE, timeOut)==HAL_OK){
		return TRUE;
	}else{
		return FALSE;
	}
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

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
