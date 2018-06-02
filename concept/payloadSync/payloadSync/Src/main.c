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

// TO-DOs:
// (1) Error handling. Retries, error code logging
// (2) Update boot-time for TX2 in heatbeatListen()
// (3) Handle data after download is finished
// (4) Review timeout periods for tx, rx
// (5) Review command-specific behaviours

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* USER CODE BEGIN Includes */
#define TRUE 1
#define FALSE 0
#define WAIT_TIME 250
#define TX_DELAY 50
#define RX_DELAY 100
// To-do: Different time-out durations for different commands

// Command Types
#define POWEROFF        0
#define START_DOWNLOAD  1
#define START_UPLOAD    2
#define REQUEST_PACKET  3
#define SEND_PACKET     4
#define CANCEL_UPLOAD   5
#define CANCEL_DOWNLOAD 6
#define FINALIZE_UPLOAD 7
#define TAKE_PHOTO      8
#define EXECUTE_COMMAND 9

// Size Constants
#define PACKETLEN_SIZE 2
uint32_t UPLOAD_PACKETLEN = 256;
uint32_t DOWNLOAD_PACKETLEN = 256;

uint8_t error_codes[] = {
		129, // photo-passed time
		130, // photo-out of storage
		131, // photo-other
		132, // exe-other
		133, // down-invalid path
		134, // down-other
		137, // up-out of storage
		138, // up-corrupt
		139, // up-invalid path
		140, // up-other
		135, // dlt-invalid path
		136, // dlt-other
		141, // status-invalid req
		142 // status-other
};

/* Structures -----------------------------------------------*/

typedef struct Message {
	uint8_t code;
	uint16_t payloadLen;
	uint8_t *payload;
	struct Message *next;
} Message;

typedef struct Queue {
	Message *front;
	Message *back;
	uint8_t numMessages;
} Queue;


/* Function Prototypes -----------------------------------------------*/
int saveData(uint8_t* arr, uint8_t index, uint8_t dataLen); // Dummy Memory
void readData(uint8_t* storage, uint8_t index, uint16_t dataLen);

void heartbeatListen();
Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data); // Queue operations

void debugWrite(char *debug_msg, int msgLen);
void sendData(uint8_t *data, int dataLen);
void sendmHeader(Message *msg);
uint8_t receiveData(uint8_t *reply, uint8_t numBytes);
uint8_t isError(uint8_t comcode, uint8_t reply);

void initQueue(Queue *que);
Message *peekQueue(Queue *que);
int emptyQueue(Queue *que);
void enqueue(Message *newMessage, Queue *que);
void dequeue(Queue *que);

/* Temp Memory Functions -----------------------------------------------*/
uint8_t memory[256];
uint8_t memIndex = 0;

int saveData(uint8_t *data, uint8_t dataLen){
	for(int i = 0; i < dataLen; i++){
		if(memIndex > 255)
			return FALSE;
		memory[memIndex] = data[i];
		memIndex++;
	}
	return TRUE;
}

void readData(uint8_t *storage, uint8_t index, uint16_t dataLen){
	for(int i = 0; i < dataLen; i++){
		if(index > 255)
			break;
		else{
			storage[i] = memory[index+i];
		}
	}
}

/* Private variables and Prototypes ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

/* Heartbeat Monitor ---------------------------------------------*/
// Listens for a heartbeat (half-period of 1 second) and power cycles if absent
void heartbeatListen(){

	GPIO_PinState currState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	HAL_Delay(1000);

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == currState){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(10000); // To-do: adjust boot-time for TX2
	}
	else
		return;
}

/* Queue Functions -----------------------------------------------*/
void initQueue(Queue *que){
  que->numMessages = 0;
  que->front = NULL;
  que->back = NULL;
}

Message *peekQueue(Queue *que){
	return que->front;
}

int emptyQueue(Queue *que){
	if(que->numMessages == 0)
		return 1;
	else
		return 0;
}

void enqueue(Message *newMessage, Queue *que){
	if(emptyQueue(que)){
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

/* Message Functions  -----------------------------------------------*/

Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data){
	Message *newMessage = (Message*)malloc(sizeof(Message)); // Command type and parameter

	newMessage->code = command_code;
	newMessage->payloadLen = data_len;
	newMessage->payload = data;
	newMessage->next = NULL;

	return newMessage;
}

void debugWrite(char *debug_msg, int msgLen){
	while(HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, msgLen, TX_DELAY) != HAL_OK){
		HAL_Delay(TX_DELAY);
	}
	HAL_Delay(100);
}

void sendData(uint8_t *data, int dataLen){
	  char buffer[50];
	  sprintf(buffer, "Sending message of length %i\n", dataLen);
	  debugWrite(buffer, sizeof(buffer));
	  while(HAL_UART_Transmit(&huart6, data, dataLen, TX_DELAY) != HAL_OK){
		  HAL_Delay(TX_DELAY); // Wait 10ms before retry
		  heartbeatListen();
	  }
}

void sendmHeader(Message *msg){
	uint8_t mHeader[3];
	mHeader[0] = msg->code; // message code
	mHeader[1] = (msg->payloadLen >> 0) & 0xFF;
	mHeader[2] = (msg->payloadLen >> 8) & 0xFF;
	sendData(mHeader, sizeof(mHeader));
}

uint8_t isError(uint8_t comcode, uint8_t reply){
	if(reply == 0)
		return FALSE;
	else
		return TRUE;
}

uint8_t receiveData(uint8_t *reply, uint8_t numBytes){
/* Wait for a response -----------*/
	char buffer[30];

	while(HAL_UART_Receive(&huart6, reply, numBytes, RX_DELAY) != HAL_OK){
		sprintf(buffer, "Waiting for reply..\n");
		debugWrite(buffer, sizeof(buffer));
		HAL_Delay(RX_DELAY);
		heartbeatListen();
	}
	sprintf(buffer, "Reply received!\n");
	debugWrite(buffer, sizeof(buffer));
	return TRUE;
}


/* Operation Variables -----------------------------------------------*/
uint32_t upload_index = 0; // upload progress tracker

/* Testing Variables -------------------------------*/
char sampleData[] = "abcd";
uint8_t sha256sum[] = { 0x88,0xD4,0x26,0x6F,0xD4,0xE6,0x33,0x8D,0x13,0xB8,0x45,0xFC,0xF2,0x89,0x57,0x9D,0x20,0x9C,0x89,0x78,0x23,0xB9,0x21,0x7D,0xA3,0xE1,0x61,0x93,0x6F,0x03,0x15,0x89 }; // pre-computed

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  /* Allocate and Initialize Queue */
  Queue *commandQue = (Queue*)malloc(sizeof(Queue));
  initQueue(commandQue);

  Queue *errors = (Queue*)malloc(sizeof(Queue));
  initQueue(errors);

  debugWrite("Initialized queue\n", sizeof("Initialized queue\n"));
  Message *command;
  uint8_t shasum[32];
  char buffer[100] = "";

  /* File Upload debug ---------------------*/
  /*
  Message *command = createMessage(CANCEL_UPLOAD, 0, NULL);
  enqueue(command, commandQue);
  command = createMessage(START_UPLOAD, sizeof(sha256sum), sha256sum);
  enqueue(command, commandQue);
  command = createMessage(SEND_PACKET, sizeof(sampleData), (uint8_t*)sampleData);
  enqueue(command, commandQue);

  char *uploadPath = "uploadtest.txt";
  command = createMessage(FINALIZE_UPLOAD, sizeof(*uploadPath),(uint8_t*)uploadPath);
  enqueue(command, commandQue);
    */

  /* Take photo debug ---------------------*/
  //Message *command = createMessage(TAKE_PHOTO, 0, NULL);

  /* Operational Loop ---------------- */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Check if TX2 is alive */
	  heartbeatListen();

	  /* Check if command queue is empty */
	  if(commandQue->numMessages > 0)
		  command = peekQueue(commandQue);
	  else{
		  HAL_Delay(1000);
		  sprintf(buffer, "Empty command queue\n");
		  debugWrite(buffer, sizeof(buffer));
		  break;
	  }

	  /* Write message details to console */
	  uint8_t comcode = command->code;
	  uint16_t datalen = command->payloadLen;
	  sprintf(buffer, "Debug message created with code %i and payload-len %i \n", comcode, datalen);
	  debugWrite(buffer, sizeof(buffer));

	  /* Send Header -----------------*/
      sendmHeader(command);
	  debugWrite("Message sent\n", sizeof("Message sent\n"));

	  /* Send Data ----------------*/
	  if(datalen > 0)
		  sendData(command->payload, datalen);

	  /* File transfer variables */
	  uint8_t *header;
	  uint16_t packetLen;
	  uint8_t *data;

	  /* Response Handling --------------------*/
	  uint8_t reply;
	  switch(comcode){
		  case START_DOWNLOAD:
			  // To-do: Extract filename from filepath

			  // Receive and parse success/error
			  receiveData(&reply, 1);

			  if(isError(START_DOWNLOAD, reply)){
				  enqueue(command, errors);
			  } else{
				  // Receive 32 byte shasum
				  receiveData(shasum, 32);
			  }
			  break;

		  case START_UPLOAD:
			  receiveData(&reply, 1);

			  if(isError(START_UPLOAD, reply)){
				  enqueue(command, errors);
				  break;
			  }
			  upload_index = 0;
			  dequeue(commandQue);
			  break;

		  case REQUEST_PACKET:
			  // Receive 1 byte response code
			  receiveData(&reply, 1);
			  if(isError(REQUEST_PACKET, reply)){
				  enqueue(command, errors);
			  }
			  else{
				  // Receive packet length and packet data
				  receiveData(&packetLen, 2);
				  data = malloc(packetLen);
				  receiveData(data, packetLen);

				  // Save data
				  saveData(data, packetLen);
				  free(packetLen);
			  }
			  break;

		  case SEND_PACKET:
			  receiveData(&reply, 1);
			  if(isError(SEND_PACKET, reply)){
				  enqueue(command, errors);
				  break;
			  }
			  upload_index += datalen;
			  break;

		  case CANCEL_UPLOAD:
			  receiveData(&reply, 1);
			  dequeue(commandQue);
			  break;

		  case FINALIZE_UPLOAD:
			  receiveData(&reply, 1);
			  dequeue(commandQue);
			  break;

		  case TAKE_PHOTO:
			  receiveData(&reply, 1);
			  if(isError(TAKE_PHOTO, reply)){
				  enqueue(command, errors);
				  debugWrite("Error Occurred\n", sizeof("Error Occurred\n"));
			  }
			  break;

		  case EXECUTE_COMMAND:
			  receiveData(&reply, 1);
			  if(isError(EXECUTE_COMMAND, reply)){
				  enqueue(command, errors);
			  }
			  break;
	  }
	  dequeue(commandQue);
	  sprintf(buffer, "Command finished ---------\n");
	  debugWrite(buffer, sizeof(buffer));
	  HAL_Delay(500);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration*/
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
