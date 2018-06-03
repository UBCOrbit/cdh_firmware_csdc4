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
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);


/* USER CODE BEGIN Includes */
#define TRUE 1
#define FALSE 0
#define WAIT_TIME 250
#define TX_DELAY 50
#define RX_DELAY 100
#define TX2_BOOT_DELAY 10000
// Delay between commands
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

// Description: Payload message format. Consists of the command code, data length, and actual data.
typedef struct Message {
	uint8_t code;
	uint16_t payloadLen;
	uint8_t *payload;
	uint8_t err;
	struct Message *next;
} Message;

// Description: Container for messages received from ground-station. Messages are handled in a FIFO
//				manner when forwarded to the TX2
typedef struct Queue {
	Message *front;
	Message *back;
	uint8_t numMessages;
} Queue;

/* Function Prototypes -----------------------------------------------*/


/* Description: Appends a fixed number of bytes to the global memory array.
 * Inputs: Pointer to the data to be appended, amount of data to be appended
 * Output: 8-bit value either True or False depending on success of the memory transfer
 */
uint8_t saveData(uint8_t *data, uint8_t dataLen); // Dummy Memory


/* Description: TX2 has a heartbeat that inverts its signal every 1second.
 * 				This function checks for two consecutive beats over PCA8 and resets the TX2
 * 				over PCA6 if it's not observed.
 */
//void heartbeatListen();


/* Description: Populate a message struct with given command code, data length and data pointer
 * Inputs: 8-bit command code, 16-bit data length, pointer to byte-formatted data array
 * Output: Pointer to newly created message
 * 		   Needs to be freed after transmitted to TX2.
 */
Message *createMessage(uint8_t command_code, uint16_t data_len, uint8_t *data); // Queue operations


/* Description: Send a debug message via serial on UART2
 * Input: Pointer to character array to be transmitted, 16-bit length of message
 * Output: Pointer to newly created message.
 */
void debugWrite(char *debug_msg);


/* Description: Send data over UART6
 * Input: Pointer to the 8-bit formatted data array, 64-bit data length
 * Output: Pointer to newly created message.
 */
void sendData(uint8_t *data, int dataLen);


/* Description: Send data-header over UART6 by extracting
 * 				the header from the message. Header contains (1) command code
 * 				and (2) payload length
 * Input: Pointer to the message to be transmitted.
 */
void sendmHeader(Message *msg);


/* Description: Receive data over UART6 and store it in the passed container.
 * Input: Pointer to the container for the response, and 8-bit length of
 * 		  the data to be received
 */
void receiveData(uint8_t *reply, int numBytes);


/* Description: Reads the reply and documents the error in the error queue if needed.
 * 				If error, reads the command, creates a similar error object appending the reply code,
 * 				then adds it to the error queue
 * Input: Pointer to the error queue, pointer to the reply, and pointer to the command
 * Output: Returns 8-bit value of True or False depending on whether error was observed
 */
uint8_t handleError(Queue *errQue, Message *command, uint8_t *reply);


/* Description: Initialize and return a new queue by nullifying front and end pointers. Zero-ing number of queue items.
 * Output: Returns pointer to newly allocated queue. Needs to be freed when no longer in use.
 */
Queue* initQueue();


/* Description: For a given queue, returns pointer to front message.
 * Output: Pointer to the front message. When freeing messages peeked from the queue, use the dequeue(queue) function.
 * 		   This ensures that the queue maintains front pointer and number of items properly.
 */
Message *peekQueue(Queue *que);


/* Description: Given a queue and a message, appends the message, updating queue pointers and params (num messages).
 */
void enqueue(Message *newMessage, Queue *que);


/* Description: Free the first message from the queue and update queue pointers and params (num messages)
 */
void dequeue(Queue *que);


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
	  char buffer[50];
	  sprintf(buffer, "Sending message of length %i\n", dataLen);
	  debugWrite(buffer);
	  while(HAL_UART_Transmit(&huart6, data, dataLen, TX_DELAY) != HAL_OK){
		  HAL_Delay(TX_DELAY); // Wait 10ms before retry
		  //heartbeatListen();
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
	char buffer[30];

	while(HAL_UART_Receive(&huart6, reply, numBytes, RX_DELAY) != HAL_OK){
		sprintf(buffer, "Waiting for reply..\n");
		debugWrite(buffer);
		HAL_Delay(RX_DELAY);
		//heartbeatListen();
	}
	sprintf(buffer, "Reply received!\n");
	debugWrite(buffer);
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
/* Operation Variables*/
uint32_t upload_index = 0; // upload progress tracker

/* Testing Variables*/
char sampleData[] = "abcd";
uint8_t sha256sum[] = { 0x88,0xD4,0x26,0x6F,0xD4,0xE6,0x33,0x8D,0x13,0xB8,0x45,0xFC,0xF2,0x89,0x57,0x9D,0x20,0x9C,0x89,0x78,0x23,0xB9,0x21,0x7D,0xA3,0xE1,0x61,0x93,0x6F,0x03,0x15,0x89 }; // pre-computed

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
  Queue *commandQue = NULL;
  commandQue = initQueue(commandQue);
  Message *command = NULL;

  Queue *errors = NULL;
  errors = initQueue(errors);

  debugWrite("Initialized queue\n");
  //Message *command;
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
  command = createMessage(CANCEL_UPLOAD, 0, NULL);
  enqueue(command, commandQue);
  buffer[0] = '\0';
  */
  //sprintf(buffer, "Commands entered.\n");
  //debugWrite(buffer, sizeof(buffer));

  //char *uploadPath = "uploadtest.txt";
  //command = createMessage(FINALIZE_UPLOAD, sizeof(*uploadPath),(uint8_t*)uploadPath);
  //enqueue(command, commandQue);


  /* Take photo debug ---------------------*/
  //Message *command = createMessage(TAKE_PHOTO, 0, NULL);
  //enqueue(command, commandQue);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	  //debugWrite("Testing..\n",sizeof("Testing..\n"));
	  //HAL_Delay(1000);
	  /* Check if TX2 is alive --------------------*/
	  //heartbeatListen();
	  //debugWrite("Entered loop..\n",sizeof("Entered loop..\n"));
	  /* Check if command queue is empty */
	  if(commandQue->numMessages > 0)
		  command = peekQueue(commandQue);
	  else{
		  HAL_Delay(1000);
		  buffer[0] = '\0';
		  sprintf(buffer, "Empty command queue\n");
		  debugWrite(buffer);
		  break;
	  }

	  /* Write message details to console */
	  uint8_t comcode = command->code;
	  uint16_t datalen = command->payloadLen;
	  sprintf(buffer, "Debug message created with code %i and payload-len %i \n", comcode, datalen);
	  debugWrite(buffer);

	  /* Send Header -----------------*/
	  sendmHeader(command);
	  buffer[0] = '\0';
	  sprintf(buffer, "Message sent\n");
	  debugWrite(buffer);

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
	  HAL_Delay(250);
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
