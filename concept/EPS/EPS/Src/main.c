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

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//I2C Addresses for EPS and Battery
#define EPS_ADDRESS 0x2B
#define BAT_ADDRESS 0x2A

//Common Commands
#define DEFAULT_DATA 0x00
#define GET_STATUS 0x01
#define GET_ERROR 0x03
#define GET_VERSION 0x04
#define GET_CHECKSUM 0x05
#define GET_TELEMETRY 0x10
#define GET_BROWNOUT_RESETS 0x31
#define GET_AUTO_SOFTWARE_RESETS 0x32
#define GET_MANUAL_RESETS 0x33
#define MANUAL_RESET 0x80

//EPS Commands
#define EPS_GET_WATCHDOG_PERIOD 0x20
#define EPS_SET_WATCHDOG_PERIOD 0x21
#define EPS_RESET_WATCHDOG 0x22
#define EPS_GET_WATCHDOG_RESETS 0x34
#define EPS_ALLPDM_ON 0x40
#define EPS_ALLPDM_OFF 0x41
#define EPS_GET_ALLPDM_STATE 0x42
#define EPS_GET_ALLPDM_INITIAL_STATE 0x44
#define EPS_SET_ALLPDM_TO_INITIAL_STATE 0x45
#define EPS_SET_PDM_ON 0x50
#define EPS_SET_PDM_OFF 0x51
#define EPS_SET_PDM_INITIAL_STATE_ON 0x52
#define EPS_SET_PDM_INITIAL_STATE_OFF 0x53
#define EPS_GET_PDM_STATUS 0x54
#define EPS_PCM_RESET 0x70

//EPS Get Telemetry Commands
#define I_IDIODE 0xE284
#define V_IDIODE 0xE280
#define I_3V3_DRW 0xE205
#define I_5V_DRW 0xE215
#define I_PCM12V 0xE234
#define V_PCM12V 0xE230
#define I_PCMBATV 0xE224
#define V_PCMBATV 0xE220
#define I_PCM5V 0xE214
#define V_PCM5V 0xE210
#define I_PCM3V3 0xE204
#define V_PCM3V3 0xE200

#define V_SW1 0xE410
#define I_SW1 0xE414
#define V_SW2 0xE420
#define I_SW2 0xE424
#define V_SW3 0xE430
#define I_SW3 0xE434
#define V_SW4 0xE440
#define I_SW4 0xE444
#define V_SW5 0xE440
#define I_SW5 0xE454
#define V_SW6 0xE460
#define I_SW6 0xE464
#define V_SW7 0xE470
#define I_SW7 0xE474
#define V_SW8 0xE480
#define I_SW8 0xE484
#define V_SW9 0xE490
#define I_SW9 0xE494
#define V_SW10 0xE4A0
#define I_SW10 0xE4A4
#define MBRD_TEMP 0xE308

#define V_BCR1 0xE110
#define I_BCR1A 0xE114
#define I_BCR1B 0xE115
#define BCR1A_TEMP 0xE118
#define BCR1B_TEMP 0xE119
#define BCR1A_SUN_DETECT 0xE11C
#define BCR1B_SUN_DETECT 0xE11D

#define V_BCR2 0xE120
#define I_BCR2A 0xE124
#define I_BCR2B 0xE125
#define BCR2A_TEMP 0xE128
#define BCR2B_TEMP 0xE129
#define BCR2A_SUN_DETECT 0xE12C
#define BCR2B_SUN_DETECT 0xE12D

#define V_BCR3 0xE130
#define I_BCR3A 0xE134
#define I_BCR3B 0xE135
#define BCR3A_TEMP 0xE138
#define BCR3B_TEMP 0xE139
#define BCR3A_SUN_DETECT 0xE13C
#define BCR3B_SUN_DETECT 0xE13D

//BAT Commands
#define BAT_GET_HEATER_CONTROLLER_STATUS 0x90
#define BAT_SET_HEATER_CONTROLLER_STATUS 0x91
#define HEATER_OFF 0x00
#define HEATER_ON 0x01

//BAT Telemetry Commands
#define V_BAT 0xE280
#define I_BAT 0xE284
#define I_DIR_BAT 0xE28E
#define MBRD_TMP 0xE308
#define I_PCM5V 0xE214
#define V_PCM5V 0xE210
#define I_PCM3V3 0xE204
#define V_PCM3V3 0xE200
#define DBAT1_TEMP 0xE398
#define DBAT1_HEATER_STATUS 0xE239F
#define DBAT2_TEMP 0xE3A8
#define DBAT2_HEATER_STATUS 0xE3AF
#define DBAT3_TEMP 0xE3B8
#define DBAT3_HEATER_STATUS 0xE3CF

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void getStatus(uint8_t address, uint8_t data_received[]);
void sendCommand(uint8_t address, uint8_t command, uint8_t data_sent, uint8_t data_received[], uint8_t bytes_received, int delay);
void getError(uint8_t address, uint8_t data_received[]);
void getTelemetry(uint8_t address, uint8_t data1, uint8_t data0, uint8_t data_received[], uint8_t bytes_received);
void manualReset(uint8_t address);
uint16_t convertBATADC(uint8_t data[], uint8_t data1, uint8_t data0 );
uint16_t convertEPSADC(uint8_t data[], uint8_t data1, uint8_t data0 );

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  uint8_t returnData[4] = {0,0,0,0};
	  uint8_t data1 = 0;
	  uint8_t data0 = 0;
	  uint8_t command = 0;
	  uint8_t select = 0;
	  uint16_t address= 0 ;
	  uint8_t returnBytes = 0;
	  uint8_t delay = 0;

	  while(HAL_UART_Receive(&huart2, (uint8_t*)&select, 1, 50) != HAL_OK);
	  HAL_UART_Transmit(&huart2, (uint8_t*)&select, 1, 50);
	  while(HAL_UART_Receive(&huart2,(uint8_t *)&address, 2, 50) != HAL_OK);
	  HAL_UART_Transmit(&huart2, (uint8_t *)&address, 2, 50);
	  //Read Select and Address

	  if (select == 's'){
		  getStatus(address, returnData);
		  /*
		  Testing code
		  uint8_t data0 = 0x10;
		  uint8_t data1 = 0xE2;
		  uint8_t data2= 0x80;
		  HAL_I2C_Master_Transmit(&hi2c1, 0x54, (uint8_t*)&data0, 1, 50);
		  HAL_I2C_Master_Transmit(&hi2c1, 0x54, (uint8_t*)&data1, 1, 50);
		  HAL_I2C_Master_Transmit(&hi2c1, 0x54, (uint8_t*)&data2, 1, 50);
		  HAL_Delay(15);

		  HAL_I2C_Master_Receive(&hi2c1, 0x54, returnData, 2, 50);
		  uint16_t received = (uint16_t)((returnData[0] + (returnData[1]*256))* 0.008993);
		  uint8_t send= 0b01000010;
		  HAL_UART_Transmit(&huart2, (uint8_t*)&send, 1, 50);
		   */
	  }

	  else if (select == 'e'){

		  getError(address,returnData);

	  }

	  else if (select == 'r'){

		  manualReset(address);
	  }

	  else if (select == 't'){

		  while(HAL_UART_Receive(&huart2, (uint8_t*)&data1, 2, 50) != HAL_OK);
		  HAL_UART_Transmit(&huart2, (uint8_t*)&data1, 2, 50);
		  while(HAL_UART_Receive(&huart2,(uint8_t *)&data0, 2, 50) != HAL_OK);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&data0, 2, 50);
		  while(HAL_UART_Receive(&huart2,(uint8_t *)&returnBytes, 2, 50) != HAL_OK);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&returnBytes, 2, 50);
		  while(HAL_UART_Receive(&huart2,(uint8_t *)&delay, 4, 50) != HAL_OK);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&delay, 4, 50);

		  //Read command,data1,data0,returnbytes,delay
		  uint16_t convertedADC;

		  getTelemetry(address,data1,data0,returnData,returnBytes);

		  if (address == EPS_ADDRESS){
			  convertedADC = convertEPSADC(returnData,data1,data0);
		  }
		  else if (address == BAT_ADDRESS) {
			  convertedADC = convertBATADC(returnData,data1,data0);
		  }

	  }

	  else if (select == 'c'){
		  // Read command,data1,returnbytes,delay
		  while(HAL_UART_Receive(&huart2, (uint8_t*)&data1, 2, 50) != HAL_OK);
		  HAL_UART_Transmit(&huart2, (uint8_t*)&data1, 2, 50);
		  while(HAL_UART_Receive(&huart2,(uint8_t *)&returnBytes, 2, 50) != HAL_OK);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&returnBytes, 2, 50);
		  while(HAL_UART_Receive(&huart2,(uint8_t *)&delay, 4, 50) != HAL_OK);
		  HAL_UART_Transmit(&huart2, (uint8_t *)&delay, 4, 50);

		  sendCommand(address,command,data1,returnData,returnBytes,delay);

	  }
	  else if (select == 'p'){
		  // Read Timer Limit and PDM


	  }

	  else {
		  // command not recognized
	  }
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

    /**Initializes the CPU, AHB and APB busses clocks     */
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

}

/* USER CODE BEGIN 4 */


/*
 * 	Purpose:
 * 	This function will get the current status of either the EPS or Battery module
 *
 *	Reference:
 *	Section 12.4.1 for Battery, 11.3 for EPS
 *
 * 	Parameters:
 * 		address - target address (either EPS or Battery)
 * 		data_received - array to hold received bytes
 */
void getStatus(uint8_t address, uint8_t data_received[]) {
	uint8_t command = GET_STATUS;
	uint8_t data1 = DEFAULT_DATA;

	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) address<<1, (uint8_t*)&command, 1, 50) != HAL_OK);
	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) address<<1, (uint8_t*)&data1, 1, 50) != HAL_OK);
	HAL_Delay(1);

	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t) (address<<1) | 0x01, data_received, 2, 50) != HAL_OK);
}


/*
 * 	Purpose:
 *	This function will send a command to either the EPS or Battery module to request data
 *
 *	Reference:
 * 	Section 12 for Battery, 11 for EPS
 *
 *	Parameters:
 *		address : target address (either EPS or Battery)
 *		command : request command to send
 *		data_sent : additional data byte to be sent along with the command
 *		data_received : array to hold received bytes
 *		bytes_received: number of bytes to request from the module
 *		delay: delay required before a response can be requested, in milliseconds
 */
void sendCommand(uint8_t address, uint8_t command, uint8_t data_sent, uint8_t data_received[], uint8_t bytes_received, int delay){
	while( HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) address<<1, (uint8_t*) &command, 1, 50) != HAL_OK );
	while( HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) address<<1, (uint8_t*) &data_sent, 1, 50) != HAL_OK );

	HAL_Delay(delay);

	if(bytes_received != 0){
		while( HAL_I2C_Master_Receive(&hi2c1, (uint16_t) (address<<1) | 0x01, data_received, (uint16_t) bytes_received, 50) != HAL_OK);
	}
}


/*
 * 	Purpose:
 * 	This function will get the last error message of either the EPS or Batt module
 *
 * 	Reference:
 * 	Section 12.4.2 for Battery, 11.3 for EPS.
 *
 * 	Parameters:
 * 		address - target address (either EPS or Battery)
 * 		data_received - array to hold received bytes
 */
void getError(uint8_t address, uint8_t data_received[]) {
	uint8_t command = GET_ERROR;
	uint8_t data1 = DEFAULT_DATA;

	while( HAL_I2C_Master_Transmit(&hi2c1,  (uint16_t) address<<1, (uint8_t*)&command, 2, 50) != HAL_OK );
	while( HAL_I2C_Master_Transmit(&hi2c1,  (uint16_t) address<<1, (uint8_t*)&data1, 2, 50) != HAL_OK );

	HAL_Delay(1);

	while( HAL_I2C_Master_Receive(&hi2c1,  (uint16_t) (address<<1) & 0x01, data_received, (uint16_t) 2, (uint32_t) 50) != HAL_OK);
}


/* 	Purpose:
 * 	This function will retrieve a specific piece of telemetry information for either the battery or EPS module
 *
 * 	Reference:
 * 	Section 12.6 for Battery, 11.4 for EPS
 *
 * 	Parameters:
 * 		address: target address (either EPS or Battery)
 * 		command: telemetry request command (0x10)
 * 		data1: first telemetry code to send
 * 		data0: second telemetry code to send
 * 		data_received: array to hold received bytes
 * 		bytes_received: number of bytes to request from the module
 */
void getTelemetry(uint8_t address, uint8_t data1, uint8_t data0, uint8_t data_received[], uint8_t bytes_received){
	uint8_t command = GET_TELEMETRY;

	while( HAL_I2C_Master_Transmit(&hi2c1,  (uint16_t) address<<1, (uint8_t*)&command, 3, 50) != HAL_OK );
	while( HAL_I2C_Master_Transmit(&hi2c1,  (uint16_t) address<<1, (uint8_t*)&data1, 3,  50) != HAL_OK );
	while( HAL_I2C_Master_Transmit(&hi2c1,  (uint16_t) address<<1, (uint8_t*)&data0, 3, 50) != HAL_OK );
	HAL_Delay(15);

	if(bytes_received != 0){
		while( HAL_I2C_Master_Receive(&hi2c1,  (uint16_t) (address<<1) & 0x01, data_received, (uint16_t)bytes_received, 50) != HAL_OK);
	}
}


/*
 * 	Purpose:
 * 	This function will manually reset either the EPS or Batt module
 *
 * 	Reference:
 * 	Section 12.4.5 for Battery, 11.3 for EPS
 *
 * 	Parameters:
 * 		address - the target I2C Address. Either EPS or Batt in this case
 */
void manualReset(uint8_t address){
	uint8_t command = MANUAL_RESET;
	uint8_t data0 = DEFAULT_DATA;

	while( HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) address<<1, (uint8_t*)&command, 2, 50) != HAL_OK );
	while( HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) address<<1, (uint8_t*)&data0, 2, 50) != HAL_OK );
}


/*
 * 	Purpose:
 * 	Converts raw ADC values into proper values through equations found on battery datasheet
 *
 * 	Parameters:
 * 		ADCdata: The raw ADC value given from the EPS or BAT module
 * 		data1: The Most significant piece of TLE Code
 * 		data0: The Least significant piece of TLE Code
 *
 * 	Return:
 * 		converted: The converted ADC value
 */
uint16_t convertBATADC(uint8_t ADCdata[], uint8_t data1, uint8_t data0 ) {
	uint16_t telem_code = (uint16_t)data0 + ((uint16_t)data1 * 256);
	uint16_t adcResponse = (uint16_t)ADCdata[0] + ((uint16_t)ADCdata[1] * 256);
	uint16_t converted = 0;

	switch (telem_code) {
		case 0xE280: //battery output voltage
	      converted = adcResponse * 0.008993;
	      break;
	    case 0xE284: //battery current magnitude
	      converted = adcResponse * 14.662757;
	      break;
	    case 0xE28E: //battery current direction
	      if (adcResponse < 512)
	        converted = 1.0;
	      else
	        converted = 0.0;
	      break;
	    case 0xE308: //motherboard temperature
	      converted = 0.372434 * adcResponse - 273.15;
	      break;
	    case 0xE214: //current draw of 5V bus
	      converted = 1.327547 * adcResponse;
	      break;
	    case 0xE210: //output voltage of 5V bus
	      converted = 0.005965 * adcResponse;
	      break;
	    case 0xE204: //current draw of 3.3V bus
	      converted = 1.327547 * adcResponse;
	      break;
	    case 0xE200: //output voltage of 3.3V bus
	      converted = 0.004311 * adcResponse;
	      break;
	    case 0xE398: //daughterboard 1 temperature
	      converted = 0.397600 * adcResponse - 238.57;
	      break;
	    case 0xE39F: //daughterboard 1 heater status
	      if (adcResponse < 512)
	        converted = 1.0;
	      else
	        converted = 0.0;
	      break;
	    case 0xE3A8: //daughterboard 2 temperature
	      converted = 0.397600 * adcResponse - 238.57;
	      break;
	    case 0xE3AF: //daughterboard 2 heater status
	      if (adcResponse < 512)
	        converted = 1.0;
	      else
	        converted = 0.0;
	      break;
	    case 0xE3B8: //daughterboard 3 temperature
	      converted = 0.397600 * adcResponse - 238.57;
	      break;
	    case 0xE3BF: //daughterboard 3 heater status
	      if (adcResponse < 512)
	        converted = 1.0;
	      else
	        converted = 0.0;
	      break;
	    default:
	      converted = 0.0;
	      break;
	}

	return converted;
}

/*
 * 	Purpose:
 * 	Converts raw ADC values into proper values through equations found on EPS datasheet
 *
 * 	Parameters:
 * 		ADCdata: The raw ADC value given from the EPS or BAT module
 * 		data1: The Most significant piece of TLE Code
 * 		data0: The Least significant piece of TLE Code
 *
 * 	Return:
 * 		converted: The converted ADC value
 */
uint16_t convertEPSADC(uint8_t ADCdata[], uint8_t data1, uint8_t data0 ){
	uint16_t telem_code = (uint16_t)data0 + ((uint16_t)data1 * 256);
	uint16_t adcResponse = (uint16_t)ADCdata[0] + ((uint16_t)ADCdata[1] * 256);
	uint16_t converted = 0;

	switch (telem_code) {
		case 0xE284: //BCR output current
	      converted = 14.662757 * adcResponse;
	      break;
	    case 0xE280: //BCR output voltage
	      converted = 0.008993157 * adcResponse;
	      break;
	    case 0xE205: //3.3V current draw
	      converted = 0.001327547 * adcResponse;
	      break;
	    case 0xE215: //5V current draw
	      converted = 0.001327547 * adcResponse;
	      break;
	    case 0xE234: //output current of 12V bus
	      converted = 0.00207 * adcResponse;
	      break;
	    case 0xE230: //output voltage of 12V bus
	      converted = 0.01349 * adcResponse;
	      break;
	    case 0xE224: //output current of battery bus
	      converted = 0.005237 * adcResponse;
	      break;
	    case 0xE220: //output voltage of battery bus
	      converted = 0.008978 * adcResponse;
	      break;
	    case 0xE214: //output current of 5V bus
	      converted = 0.005237 * adcResponse;
	      break;
	    case 0xE210: //output voltage of 5V bus
	      converted = 0.005865 * adcResponse;
	      break;
	    case 0xE204: //output current of 3.3V bus
	      converted = 0.005237 * adcResponse;
	      break;
	    case 0xE200: //output voltage of 3.3V bus
	      converted = 0.004311 * adcResponse;
	      break;
	    case 0xE410: //output voltage switch 1
	      converted = 0.01349 * adcResponse;
	      break;
	    case 0xE414: //output current switch 1
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE420: //output voltage switch 2
	      converted = 0.01349 * adcResponse;
	      break;
	    case 0xE424: //output current switch 2
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE430: //output voltage switch 3
	      converted = 0.008993 * adcResponse;
	      break;
	    case 0xE434: //output current switch 3
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE440: //output voltage switch 4
	      converted = 0.008993 * adcResponse;
	      break;
	    case 0xE444: //output current switch 4
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE450: //output voltage switch 5
	      converted = 0.005865 * adcResponse;
	      break;
	    case 0xE454: //output current switch 5
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE460: //output voltage switch 6
	      converted = 0.005865 * adcResponse;
	      break;
	    case 0xE464: //output current switch 6
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE470: //output voltage switch 7
	      converted = 0.005865 * adcResponse;
	      break;
	    case 0xE474: //output current switch 7
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE480: //output voltage switch 8
	      converted = 0.004311 * adcResponse;
	      break;
	    case 0xE484: //output current switch 8
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE490: //output voltage switch 9
	      converted = 0.004311 * adcResponse;
	      break;
	    case 0xE494: //output current switch 9
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE4A0: //output voltage switch 10
	      converted = 0.004311 * adcResponse;
	      break;
	    case 0xE4A4: //output current switch 10
	      converted = 0.001328 * adcResponse;
	      break;
	    case 0xE308: //motherboard temperature
	      converted = 0.372434 * adcResponse - 273.15;
	      break;
	    case 0xE110: //voltage feeding BCR1
	      converted = 0.0249 * adcResponse;
	      break;
	    case 0xE114: //current BCR1, connector SA1A
	      converted = 0.0009775 * adcResponse;
	      break;
	    case 0xE115: //current BCR1, connector SA1B
	      converted = 0.0009775 * adcResponse;
	      break;
	    case 0xE120: //voltage feeding BCR2
	      converted = 0.0249 * adcResponse;
	      break;
	    case 0xE124: //current BCR2, connector SA2A
	      converted = 0.0009775 * adcResponse;
	      break;
	    case 0xE125: //current BCR2, connector SA2B
	      converted = 0.0009775 * adcResponse;
	      break;
	    case 0xE130: //voltage feeding BCR3
	      converted = 0.0099706 * adcResponse;
	      break;
	    case 0xE134: //current BCR3, connector SA3A
	      converted = 0.0009775 * adcResponse;
	      break;
	    case 0xE135: //current BCR3, connector SA3B
	      converted = 0.0009775 * adcResponse;
	      break;
	    default:
	      converted = 0.0;
	      break;
	}
	return converted;
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
