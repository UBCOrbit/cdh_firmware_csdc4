/*Description: Given a queue of commands received from the ground station, forwards the commands one at a time
 * 			   to payload. Waits for a response and parses the reply. Errors are stored in a log with their
 * 			   error code and the message data.
 * Author: Carter Fang
 * Date: 2018-06-02
 * To-do: (1) Test Download - currently shasum isn't received
 * 		  (2) Remove placeholders for saving functions
 * 		  (3) Figure out what to do with error log
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "payloadsync.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Operation Variables*/
uint32_t upload_index = 0; // upload progress tracker
Queue *commandQue;
Queue *errors;
Message *command;
uint8_t shasum[32];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

/* Function Definitions ---------------------------------------------------------------- */

void heartbeatListen(){
	return; // To-do: remove this when pins are set
	GPIO_PinState currState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
	HAL_Delay(1000);

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == currState){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(TX2_BOOT_DELAY); // To-do: adjust boot-time for TX2
	}
	else
		return;
}

/*Queue Functions*/
Queue *initQueue(){
  Queue *que = malloc(sizeof(Queue));
  que->numMessages = 0;
  que->front = NULL;
  que->back = NULL;
  return que;
}

Message *peekQueue(Queue *que){
	return que->front;
}

void enqueue(Message *newMessage, Queue *que){
	if(que->numMessages == 0){
		que->front = newMessage;
		que->back = newMessage;
	}
	else{
		(que->back)->next = newMessage;
		que->back = newMessage;
	}

	que->numMessages++;
}

void dequeue(Queue *que){
	Message *temp = que->front;
	que->front = temp->next;
	free(temp);
	que->numMessages--;
}

/* Message Functions*/
Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data){
	Message *newMessage = (Message*)malloc(sizeof(Message)); // Command type and parameter

	newMessage->code = command_code;
	newMessage->payloadLen = data_len;
	newMessage->payload = data;
	newMessage->err = 0;
	newMessage->next = NULL;

	return newMessage;
}

void debugWrite(char *debug_msg){
	while(HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), TX_DELAY) != HAL_OK){
		HAL_Delay(TX_DELAY);
	}
	HAL_Delay(100);
}

void sendData(uint8_t *data, int dataLen){
	  while(HAL_UART_Transmit(&huart6, data, dataLen, TX_DELAY) != HAL_OK){
		  HAL_Delay(TX_DELAY); // Wait 10ms before retry
		  heartbeatListen(); // Check if still alive
	  }
}

void sendmHeader(Message *msg){
	uint8_t mHeader[3];
	mHeader[0] = msg->code; // message code
	mHeader[1] = (msg->payloadLen >> 0) & 0xFF;
	mHeader[2] = (msg->payloadLen >> 8) & 0xFF;
	sendData(mHeader, sizeof(mHeader));
}

uint8_t handleError(Queue *errQue, Message *command, uint8_t *reply){
	if(*reply == 0)
		return FALSE; // No error
	else{
		command->err = *reply;
		enqueue(command, errQue);
		return TRUE; // Notify of error
	}

}

void receiveData(uint8_t *reply, int numBytes){
/* Wait for a response -----------*/
	while(HAL_UART_Receive(&huart6, reply, numBytes, RX_DELAY) != HAL_OK){
		HAL_Delay(RX_DELAY);
		heartbeatListen(); // Check if still alive
	}
}

uint8_t memory[64];
uint32_t memIndex = 0;

uint8_t saveData(uint8_t *data, uint8_t dataLen){
	for(int i = 0; i < dataLen; i++){
		memory[memIndex + i] = data[i];
		memIndex++;

		if(memIndex + i > 255)
			return FALSE;
	}
	return TRUE;
}


int main(void)
{
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

  /* Allocate and Initialize Queue */
  commandQue = NULL;
  commandQue = initQueue(commandQue);
  command = NULL;

  errors = NULL;
  errors = initQueue(errors);

  //char buffer[100] = "";

  while (1)
  {
  /* USER CODE END WHILE */

	  /* Check if TX2 is alive --------------------*/
	  heartbeatListen();

	  /* Check if command queue is empty */
	  if(commandQue->numMessages > 0)
		  command = peekQueue(commandQue);
	  else{
		  HAL_Delay(1000); // Empty command queue - exit loop
		  break;
	  }

	  /* Save message details */
	  uint8_t comcode = command->code;
	  uint16_t datalen = command->payloadLen;

	  /* Send Header -----------------*/
	  sendmHeader(command);

	  /* Send Data ----------------*/
	  if(datalen > 0)
		  sendData(command->payload, datalen);

	  /* File transfer variables */
	  uint8_t packetLenArr[2];
	  uint16_t packetLen;
	  uint8_t *data;

	  /* Command-Specific Response Handling --------------------*/
	  // Receive reply -> Parse Reply -> Handle Errors -> Dequeue message
	  uint8_t reply;
	  switch(comcode){
		  case START_DOWNLOAD:
			  // To-do: Extract filename from filepath

			  // Receive and parse success/error
			  receiveData(&reply, 1);
			  if(!handleError(errors, command, &reply)){
				  // If no error, receive 32 byte shasum
				  receiveData(shasum, 32);
			  }
			  break;

		  case START_UPLOAD:
			  receiveData(&reply, 1);
			  if(handleError(errors, command, &reply))
				  break;
			  // Refresh upload index to match start of the file
			  upload_index = 0;
			  break;

		  case REQUEST_PACKET:
			  receiveData(&reply, 1);
			  if(handleError(errors, command, &reply))
				  break;
			  else{
				  // If no error, parse packet length
				  receiveData(packetLenArr, 2);
				  packetLen = packetLenArr[1] << 8;
				  packetLen = packetLen & (packetLenArr[0]);
				  data = malloc(packetLen);

				  // Use packet length to receive incoming data
				  receiveData(data, packetLen);

				  // Save data
				  saveData(data, packetLen); // To-do: Change saving function (currently a dummy)
				  free(data);
			  }
			  break;

		  case SEND_PACKET:
			  receiveData(&reply, 1);
			  if(handleError(errors, command, &reply))
				  break;
			  else // If no error, increment index of file being read (track progress of reading)
				  upload_index += datalen;
			  break;

		  case CANCEL_UPLOAD:
			  receiveData(&reply, 1);
			  handleError(errors, command, &reply);
			  break;

		  case FINALIZE_UPLOAD:
			  receiveData(&reply, 1);
			  break;

		  case TAKE_PHOTO:
			  receiveData(&reply, 1);
			  handleError(errors, command, &reply);
			  break;

		  case EXECUTE_COMMAND:
			  receiveData(&reply, 1);
			  handleError(errors, command, &reply);
			  break;
	  }
	  dequeue(commandQue);
	  HAL_Delay(COMMAND_DELAY);
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
