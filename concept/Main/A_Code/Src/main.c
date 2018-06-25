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
#include "a_comparison.h"
#include "comms.h"
#include "payload.h"
#include "eps.h"
#include <stdlib.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2; //Ground timer
TIM_HandleTypeDef htim5; //Picture timer

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define BUS_FREQUENCY 10500000
#define PRESCALER 65534

#define ONDELAY 500

struct tempBuffers {
	uint8_t telemEPS;
	uint8_t telemEPS_lastIndex;

	uint8_t telemADCS;
	uint8_t telemADCS_lastIndex;

	uint8_t telemPayload;
	uint8_t telemPayload_lastIndex;

	uint8_t telemComms;
	uint8_t telemComms_lastIndex;

	uint8_t codeNavigation;
} tempBuffers;
int volatile payloadOn = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void seconds_to_timer_period(uint16_t seconds, int timer);
void MX_TIM2_Change_Period(int period);
void MX_TIM5_Change_Period(int period);
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
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_SPI1_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	MX_TIM2_Init();
	MX_TIM5_Init();

	/* USER CODE BEGIN 2 */
	commandQue = initQueue(commandQue);
	errors = initQueue(errors);
	uint8_t packetLenArr[2];
	uint16_t packetLen = 0;
	uint8_t *data;


/*	uint8_t lost_connection = 0;
	char received = 0;
	uint8_t *buf = malloc(PACKET_SIZE);
	uint8_t adr = 0;
	uint8_t flg = 0;
	uint8_t len_command = 0;
	uint8_t *data = malloc(PACKET_SIZE - 2);

	char *commandPayload;
	uint8_t payloadReply = 1;
	uint16_t time_until_picture;*/

	//Start from point where we turn on COMMS
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);

	//Wait for heartbeat from COMMS
//	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != 1) {
//		HAL_Delay(ONDELAY);
//	}
	printStringToConsole("STM_A Start\n");



	//Wait for COMMS to make connection to ground
	//Not yet in place to deal with missing connection from ground
	//while(HAL_UART_Receive(&huart1, (uint8_t*)&received, 2, 0x0FFF) != HAL_OK);

	//printStringToConsole(&received);
	/* USER CODE END 2 */

	/* Over Ground loop*/
/*	while (1) {
		//Check receive packet
		receive_packet(buf);

		printStringToConsole("Packet was received\n");
		if(parse_packet(buf, &adr, &flg, &len_command, data)) {
			printStringToConsole("Packet was parsed\n");
			switch(adr) {
			//CDH Command
			case 0x00:
				switch(len_command) {
				case 0x00:
					printStringToConsole("Lost connection with ground\n");
					lost_connection = 1;
					break;
				}
				break;

			//Payload Command
			case 0x01:
				if(flg == 0) {
					switch(len_command) {
					case 0x00:
						printStringToConsole("Take picture command received\n");
						//time_until_picture = ((uint16_t)data[0] * 256) + (uint16_t)data[1];
						//seconds_to_timer_period(time_until_picture, 5);

						//Add take picture command to payload queue
						enqueue(createMessage(TAKE_PHOTO, 8, (data)), payloadCommands);
						break;
					case 0x01:
						printStringToConsole("Execute script command received\n");
						commandPayload = "exec/identify_fire ";
						commandPayload[19] = data[0] + '0';
						enqueue(createMessage(EXECUTE_COMMAND, strlen(commandPayload), (uint8_t *)commandPayload), payloadCommands);
						break;
					case 0x02:
						printStringToConsole("File download command received\n");
						break;
					}
				} else;
				break;

			//ADCS Command
			case 0x02:
				break;

			//EPS Command
			case 0x03:
				break;
			}
		}
		break;

		if(lost_connection == 1)
			break;
	}*/

	char *path = "output/fire0.txt";
	enqueue(createMessage(START_DOWNLOAD, strlen(path), (uint8_t*)path), commandQue);

	 while (1) {
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
	  printStringToConsole("Sent command to payload\n");

	  /* Send Data ----------------*/
	  if(datalen > 0)
		  sendData(command->payload, datalen);

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
			  if(reply == 7)
				  printStringToConsole("Downloaded");
			  else if(handleError(errors, command, &reply))
				  break;
			  else{
				  // If no error, parse packet length
				  receiveData(packetLenArr, 2);

				  uint16_t packetLen;
				 // memcpy(&packetLen, packetLenArr, 2);

				  packetLen = packetLenArr[1];
				  packetLen = packetLen | ((uint16_t)packetLenArr[0] << 8);
				  data = malloc(packetLen);

				  // Use packet length to receive incoming data
				  receiveData(data, packetLen-5);

				  // Save data
				  saveData(data, packetLen); // To-do: Change saving function (currently a dummy)
				  free(data);
				  printStringToConsole("One Packet Downloaded\n");
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
	  printStringToConsole("Enqueue one packet\n");
	  enqueue(createMessage(REQUEST_PACKET, 0, (uint8_t*)path), commandQue);
	  HAL_Delay(COMMAND_DELAY);
  	}

	/*Wait for picture loop*/
	/*while(1) {
	//deal with next patch packets

		if(payloadOn == 1) {
			while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1)
				HAL_Delay(ONDELAY);
			break;
		}
	}*/
	/*char *path = "output/fire0.txt";
	uint8_t response[3];
	uint8_t dataResp[20];
	uint16_t packetLen;
	uint8_t packetLenArr[2];

	enqueue(createMessage(START_DOWNLOAD, strlen(path), (uint8_t*)path), payloadCommands);
	sendmHeader(peekQueue(payloadCommands));
	sendData((uint8_t*)path, strlen(path));

	printStringToConsole("Sent command to payload\n");
	receiveData(&payloadReply, 1);
	receiveData(data, 32);

	while(1) {
		if(payloadReply == 0) {
			dequeue(payloadCommands);

			printStringToConsole("Enqueue one packet\n");
			enqueue(createMessage(REQUEST_PACKET, 0, (uint8_t*)path), payloadCommands);

			sendmHeader(peekQueue(payloadCommands));
			receiveData(&payloadReply, 1);

			receiveData(packetLenArr, 2);
			packetLen = packetLenArr[1] << 8;
			packetLen = packetLen & (packetLenArr[0]);

			receiveData(dataResp,packetLen);
			printStringToConsole("One Packet Downloaded\n");
		}
		else if(payloadReply == 7) {
			printStringToConsole("Done Downloading\n");
			break;
		}
		else {
			handleError(errorPayload, peekQueue(payloadCommands), &payloadReply);
			printStringToConsole("Error\n");
		}
	}
*/


	/*Payload picture & analyze*/
/*	while(!queueIsEmpty(payloadCommands)) {
		sendmHeader(peekQueue(payloadCommands));
		sendData((uint8_t *)path, strlen(path));
		printStringToConsole("Sent command to payload\n");
		receiveData(&payloadReply, 1);

		if(payloadReply == 0) {
			dequeue(payloadCommands);
			enqueue(createMessage(START_DOWNLOAD, 0, (uint8_t*)path), payloadCommands);
			printStringToConsole("One Packet Downloaded\n");
		}
		else {
			handleError(errorPayload, peekQueue(payloadCommands), &payloadReply);
			printStringToConsole("Fail taking photo\n");
		}
	}*/


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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK)
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
  htim2.Init.Prescaler = 65534;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9613;						//Default timer time is 1min
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 65534;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 9613;						//Default timer time is 1min
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_9, GPIO_PIN_SET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_5
                           |GPIO_PIN_8 , GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC4 PC5 
                           PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



/*
 * Changes the timer period to trigger after a set number of seconds
 * Params: seconds - number of seconds between each interrupt
 */
void seconds_to_timer_period(uint16_t seconds, int timer) {
    int period = seconds*BUS_FREQUENCY/PRESCALER;

    if(timer == 2)
    	MX_TIM2_Change_Period(period);
    else if(timer == 5)
    	MX_TIM5_Change_Period(period);
}


void MX_TIM2_Change_Period(int period) {
	htim2.Init.Period = period;
	HAL_TIM_Base_Start_IT(&htim2);
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) //somehow necessary, not sure why
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

void MX_TIM5_Change_Period(int period) {
	htim5.Init.Period = period;
	HAL_TIM_Base_Start_IT(&htim5);
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) //somehow necessary, not sure why
	{
		_Error_Handler(__FILE__, __LINE__);
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
