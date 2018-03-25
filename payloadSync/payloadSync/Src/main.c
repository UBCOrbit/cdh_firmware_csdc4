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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* USER CODE BEGIN Includes */
#define TRUE 1
#define FALSE 0
#define WAIT_TIME 500
#define TX_DELAY 50
#define RX_DELAY 50

// Command Types
#define EXECUTE_COMMAND 9
#define POWEROFF        0
#define START_DOWNLOAD  1
#define START_UPLOAD    2
#define REQUEST_PACKET  3
#define SEND_PACKET     4
#define CANCEL_UPLOAD   5
#define CANCEL_DOWNLOAD 6
#define FINALIZE_UPLOAD 7
#define TAKE_PHOTO      8

// Size Constants
#define PACKETLEN_SIZE 2
#define UPLOAD_PACKETLEN 256
uint8_t DOWNLOAD_PACKETLEN = 256;

// Success and error codes
uint8_t success_codes[] = {
		65, // photo
		66, // execute
		67, // download
		68, // delete
		69, // upload
		70 // status query
};

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

// Dummy Memory
uint8_t memory[256];
uint8_t memIndex = 0;

int saveData(uint8_t *data, uint8_t index, uint8_t dataLen){
	for(int i = 0; i < dataLen; i++){
		if(memIndex > 255)
			return FALSE;
		memory[memIndex] = data[index+i];
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6; // Payload communication pin
// To-do: pick a GPIO pin for powering on TX2
// To-do: pick a GPIO pin for timeout monitoring

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Structures -----------------------------------------------*/
typedef struct {
	uint8_t code;
	uint16_t payloadLen;
	uint8_t *payload;
} Message;

typedef struct Queue_Item {
	  Message command;
	  struct Queue_Item *next;
} Queue_Item;

typedef struct {
	Queue_Item *front;
	Queue_Item *back;
	  uint8_t numQueue_Items;
} Queue;

void getPacketLen(uint8_t *header, uint16_t *packetLen)
void buildHeader(uint8_t *header, uint16_t packetLen)
int saveData(uint8_t* arr, uint8_t index, uint8_t dataLen);
void readData(uint8_t* storage, uint8_t index, uint16_t dataLen);
Queue_Item *createQueue_Item(uint8_t command_code, uint16_t data_len, uint8_t *data);
void initQueue(Queue *que);
Queue_Item *peekQueue(Queue *que);
int emptyQueue(Queue *que);
void enqueue(Queue_Item *newQueue_Item, Queue *que);
void dequeue(Queue *que);


/* Queue Functions */
Queue_Item *createQueue_Item(uint8_t command_code, uint16_t data_len, uint8_t *data){
	Queue_Item *newQueue_Item = (Queue_Item*)malloc(sizeof(Queue_Item)); // Command type and parameter

	memncpy(newQueue_Item, &command_code, 1);
	memncpy(newQueue_Item + 1, &data_len, 2);
	memncpy(newQueue_Item + 3, data, data_len);
	newQueue_Item->next = NULL;

	return newQueue_Item;
}

void initQueue(Queue *que){
  que->numQueue_Items = 0;
  que->front = NULL;
  que->back = NULL;
}

Queue_Item *peekQueue(Queue *que){
	return que->front;
}

int emptyQueue(Queue *que){
	if(que->numQueue_Items == 0)
		return 1;
	else
		return 0;
}

void enqueue(Queue_Item *newQueue_Item, Queue *que){
	if(emptyQueue(que)){
		que->front = newQueue_Item;
		que->back = newQueue_Item;
	}
	else{
		que->back = newQueue_Item;
	}

	que->numQueue_Items++;
}

void dequeue(Queue *que){
	Queue_Item *temp = que->front;
	que->front = que->front->next;
	free(temp);
	que->numQueue_Items--;
}

/* Data Receiving and Sending */
void sendData(uint8_t *data){
	  while(HAL_UART_Transmit(&huart6, data, sizeof(*data), TX_DELAY) != HAL_OK){
		  HAL_Delay(TX_DELAY); // Wait 10ms before retry
	  }
}

uint8_t receiveData(uint8_t *reply, uint8_t numBytes){
/* Wait for a response -----------*/
	  while(HAL_UART_Receive(&huart6, reply, numBytes, RX_DELAY) != HAL_OK){
		  /*if(timeOut()){
			  return FALSE;
			  // Power cycle them
		  }*/
	  }
	  return TRUE;
}

/* Packet Header Functions */
void getPacketLen(uint8_t *header, uint16_t *packetLen){
	uint16_t temp = header[2];
	temp = temp << 8;
	temp = temp & header[1];

	*packetLen = temp;
}

void buildHeader(uint8_t *header, uint16_t packetLen){
	header = malloc(1+UPLOAD_PACKETLEN);
	header[0] = SEND_PACKET;
	header[2] = packetLen >> 8; // high 8 bits
	header[1] = packetLen & 00001111;
}

/* MAIN ----------------------------- */
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();

  /* Allocate and Initialize Queue */
  Queue *commands = (Queue*)malloc(sizeof(Queue));
  initQueue(commands);

  Queue *errors = (Queue*)malloc(sizeof(Queue));
  initQueue(errors);

  /* Operational Loop ---------------- */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Power on Payload ----------------------*/
	  // To-do, GPIO power on

	  /* Extract command from queue and send it -------------*/
	  Queue_Item *currItem = peekQueue(commands);
	  Message *command = &(currItem->command);
	  uint8_t commandCode = command->code;

	  sendData((uint8_t*)command);

	  /* Code-specific behavior */
	  uint8_t error = 0;
	  uint8_t reply;

	  uint8_t *header;
	  uint8_t *packet;
	  uint16_t packetLen;
	  uint8_t *data;
	  uint16_t upload_index = 0;

	  switch(commandCode){
		  case START_DOWNLOAD:
			  // Receive and parse success/error
			  receiveData(&reply, 1);
			  if(error){
				  enqueue(currItem, errors);
				  dequeue(commands);
				  break;
			  }
			  dequeue(commands);
			  break;

		  case START_UPLOAD:
			  // Receive and parse success/error
			  receiveData(&reply, 1);
			  if(error){
				  enqueue(currItem, errors);
				  dequeue(commands);
				  break;
			  }
			  // Initialize memory index of file
			  // To-do: figure out where this comes from?
			  upload_index = 0;
			  dequeue(commands);
			  break;

		  case REQUEST_PACKET:
			  // Extract packetLen
			  header = malloc(1+PACKETLEN_SIZE);
			  receiveData(header, 1+PACKETLEN_SIZE);
			  getPacketLen(header, &packetLen);

			  // Receive and save data
			  data = malloc(packetLen);
			  receiveData(data, packetLen);
			  saveData(data, memIndex, packetLen);

			  // Free data and header, dequeue the task
			  free(data);
			  free(header);
			  dequeue(commands);
			  break;

		  case SEND_PACKET:
			  // Construct and send header
			  header = malloc(1+PACKETLEN_SIZE);
			  buildHeader(header, (uint16_t)UPLOAD_PACKETLEN);
			  sendData(header);

			  // Construct and send packet
			  packet = malloc(UPLOAD_PACKETLEN);
			  readData(packet, upload_index, UPLOAD_PACKETLEN);
			  sendData(packet);

			  // Free header and packet, increment memory index based on packet length
			  free(header);
			  free(packet);
			  upload_index += UPLOAD_PACKETLEN;
			  dequeue(commands);
			  break;

		  case CANCEL_UPLOAD:
			  upload_index = 0;
			  dequeue(commands);
			  break;

		  case FINALIZE_UPLOAD:
			  upload_index = 0;
			  dequeue(commands);
			  break;

		  case TAKE_PHOTO:
			  receiveData(&reply, 1);
			  if(error){
				  queueError(currItem);
				  dequeue(commands);
			  }
			  break;

		  case EXECUTE_COMMAND:
			  receiveData(&reply, 1);
			  if(error){
				  queueError(currItem);
				  dequeue(commands);
			  }
			  break;
	  }

  }


  /* USER CODE END 3 */

}

/*
void handleError(errCode){
	switch(errCode){
		case
	}
}
*/

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
