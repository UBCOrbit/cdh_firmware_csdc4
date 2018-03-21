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
#define EPS_ADDRESS 0x2B
#define BAT_ADDRESS 0x2A

#define DEFAULT_DATA 0x00

//EPS Module Telemetry Commands

#define EPS_BOARD_STATUS_COMMAND 0x01
#define EPS_ERROR_COMMAND 0x03
#define EPS_GET_VERSION_COMMAND 0x04
#define EPS_GET_CHECKSUM_COMMAND 0x05
#define EPS_GET_TELEMETRY 0x10
#define EPS_GET_COMMS_WATCHDOG 0x20
#define EPS_SET_COMMS_WATCHDOG 0x21
#define EPS_RESET_COMMS_WATCHDOG 0x22
#define EPS_GET_BROWNOUT_RESETS 0x31
#define EPS_GET_AUTO_SOFTWARE_RESETS 0x32
#define EPS_GET_MANUAL_RESETS 0x33
#define EPS_GET_COMMS_WATCHDOG_RESETS 0x34
#define EPS_ALLPDM_ON 0x40
#define EPS_ALLPDM_OFF 0x41
#define EPS_GET_ACTUAL_STATE_ALLPDM 0x42
#define EPS_GET_EXPECTED_STATE_ALLPDM 0x43
#define EPS_GET_INITIAL_STATE_ALLPDM 0x44
#define EPS_SET_ALLPDM_TO_INITIAL_STATE 0x45
#define EPS_SET_PDMN_ON 0x50
#define EPS_SET_PDMN_OFF 0x51
#define EPS_SET_PDMN_INITIAL_STATE_ON 0x52
#define EPS_SET_PDMN_INITIAL_STATE_OFF 0x53
#define EPS_GET_PDMN_ACTUAL_STATUS 0x54
#define EPS_SET_PDMN_TIMER_LIMIT 0x60
#define EPS_GET_PDMN_TIMER_LIMIT 0x61
#define EPS_GET_PDMN_CURRENT_TIMER_LIMIT 0x62
#define EPS_PCM_RESET_COMMAND 0x70
#define EPS_MANUAL_RESET_COMMAND 0x80

//EPS get telemetry data values
#define IIDIODE_OUT_DATA1 0xE2
#define IIDIODE_OUT_DATA0 0x84
#define VIDIODE_OUT_DATA1 0xE2
#define VIDIODE_OUT_DATA0 0x80
#define I3V3_DRW_DATA1 0xE2
#define I3V3_DRW_DATA0 0x05
#define I5V_DRW_DATA1 0xE2
#define I5V_DRW_DATA0 0x15
#define IPCM12V_DATA1 0xE2
#define IPCM12V_DATA0 0x34
#define VPCM12V_DATA1 0xE2
#define VPCM12V_DATA0 0x30
#define IPCMBATV_DATA1 0xE2
#define IPCMBATV_DATA0 0x24
#define VPCMBATV_DATA1 0xE2
#define VPCMBATV_DATA0 0x20
#define IPCM5V_DATA1 0xE2
#define IPCM5V_DATA0 0x14
#define VPCM5V_DATA1 0xE2
#define VPCM5V_DATA0 0x10
#define IPCM3V3_DATA1 0xE2
#define IPCM3V3_DATA0 0x04
#define VPCM3V3_DATA1 0xE2
#define VPCM3V3_DATA0 0x00

#define VSW1_DATA1 0xE4
#define VSW1_DATA0 0x10
#define ISW1_DATA1 0xE4
#define ISW1_DATA0 0x14
#define VSW2_DATA1 0xE4
#define VSW2_DATA0 0x20
#define ISW2_DATA1 0xE4
#define ISW2_DATA0 0x24
#define VSW3_DATA1 0xE4
#define VSW3_DATA0 0x30
#define ISW3_DATA1 0xE4
#define ISW3_DATA0 0x34
#define VSW4_DATA1 0xE4
#define VSW4_DATA0 0x40
#define ISW4_DATA1 0xE4
#define ISW4_DATA0 0x44
#define VSW5_DATA1 0xE4
#define VSW5_DATA0 0x50
#define ISW5_DATA1 0xE4
#define ISW5_DATA0 0x54
#define VSW6_DATA1 0xE4
#define VSW6_DATA0 0x60
#define ISW6_DATA1 0xE4
#define ISW6_DATA0 0x64
#define VSW7_DATA1 0xE4
#define VSW7_DATA0 0x70
#define ISW7_DATA1 0xE4
#define ISW7_DATA0 0x74
#define VSW8_DATA1 0xE4
#define VSW8_DATA0 0x80
#define ISW8_DATA1 0xE4
#define ISW8_DATA0 0x84
#define VSW9_DATA1 0xE4
#define VSW9_DATA0 0x90
#define ISW9_DATA1 0xE4
#define ISW9_DATA0 0x94
#define VSW10_DATA1 0xE4
#define VSW10_DATA0 0xA0
#define ISW10_DATA1 0xE4
#define ISW10_DATA0 0xA4
#define TBRD_DATA1 0xE3
#define TBRD_DATA0 0x08

#define VBCR1_DATA1 0xE1
#define VBCR1_DATA0 0x10
#define IBCR1A_DATA1 0xE1
#define IBCR1A_DATA0 0x14
#define IBCR1B_DATA1 0xE1
#define IBCR1B_DATA0 0x15
#define TBCR1A_DATA1 0xE1
#define TBCR1A_DATA0 0x18
#define TBCR1B_DATA1 0xE1
#define TBCR1B_DATA0 0x19
#define SDBCR1A_DATA1 0xE1
#define SDBCR1A_DATA0 0x1C
#define SDBCR1B_DATA1 0xE1
#define SDBCR1B_DATA0 0x1D

#define VBCR2_DATA1 0xE1
#define VBCR2_DATA0 0x20
#define IBCR2A_DATA1 0xE1
#define IBCR2A_DATA0 0x24
#define IBCR2B_DATA1 0xE1
#define IBCR2B_DATA0 0x25
#define TBCR2A_DATA1 0xE1
#define TBCR2A_DATA0 0x28
#define TBCR2B_DATA1 0xE1
#define TBCR2B_DATA0 0x29
#define SDBCR2A_DATA1 0xE1
#define SDBCR2A_DATA0 0x2C
#define SDBCR2B_DATA1 0xE1
#define SDBCR2B_DATA0 0x2D

#define VBCR3_DATA1 0xE1
#define VBCR3_DATA0 0x30
#define IBCR3A_DATA1 0xE1
#define IBCR3A_DATA0 0x34
#define IBCR3B_DATA1 0xE1
#define IBCR3B_DATA0 0x35
#define TBCR3A_DATA1 0xE1
#define TBCR3A_DATA0 0x38
#define TBCR3B_DATA1 0xE1
#define TBCR3B_DATA0 0x39
#define SDBCR3A_DATA1 0xE1
#define SDBCR3A_DATA0 0x3C
#define SDBCR3B_DATA1 0xE1
#define SDBCR3B_DATA0 0x3D

//PCM Control Data Bytes
#define BATV_RESET_DATA 0x01
#define V5_RESET_DATA 0x02
#define V3_RESET_DATA 0x04
#define V12_RESET_DATA 0x08

//BAT Module Telemetry Commands
#define BAT_BOARD_STASUS_COMMAND 0x01
#define BAT_ERROR_COMMAND 0x03
#define BAT_GET_VERSION_COMMAND 0x04
#define BAT_GET_CHECKSUM_COMMAND 0x05
#define BAT_GET_TELEMETRY_COMMAND 0x10
#define BAT_GET_BROWNOUT_COMMAND 0x31
#define BAT_GET_AUTO_SOFTWARE_RESET_COMMAND 0x32
#define BAT_GET_MANUAL_RESET_COMMAND 0x33
#define BAT_GET_HEATER_CONTROLLER_STATUS 0x90
#define BAT_SET_HEATER_CONTROLLER_STATUS 0x91
#define BAT_MANUAL_RESET 0x80

//HEATER CONTROLLER MODES
#define HEATER_THERMOSTAT_OFF 0x00
#define HEATER_THERMOSTAT_ON 0x01

//BAT Telemetry Data Values
#define VBAT_DATA1 0xE2
#define VBAT_DATA0 0x80
#define IBAT_DATA1 0xE2
#define IBAT_DATA0 0x84
#define IDIRBAT_DATA1 0xE2
#define IDIRBAT_DATA2 0x8E
#define TBRD_DATA1 0xE3
#define TBRD_DATA0 0x08
#define IPCM5V_DATA1 0xE2
#define IPCM5V_DATA0 0x14
#define VPCM5V_DATA1 0xE2
#define VPCM5V_DATA0 0x10
#define IPCM3V3_DATA1 0xE2
#define IPCM3V3_DATA0 0x04
#define VPCM3V3_DATA1 0xE2
#define VPCM3V3_DATA0 0x00
#define TBAT1_DATA1 0xE3
#define TBAT1_DATA0 0x98
#define HBAT1_DATA1 0xE3
#define HBAT1_DATA0 0x9F
#define TBAT2_DATA1 0xE3
#define TBAT2_DATA0 0xA8
#define HBAT2_DATA1 0xE3
#define HBAT2_DATA0 0xAF
#define TBAT3_DATA1 0xE3
#define TBAT3_DATA0 0xB8
#define HBAT3_DATA1 0xE3
#define HBAT3_DATA0 0xBF




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void getStatus(uint8_t address,uint8_t data_received[]);
void sendCommand(uint8_t address, uint8_t command, uint8_t data_sent, uint8_t data_received[], int bytes_returned, int delay);
void getError(uint8_t address,uint8_t data_received[]);
void getTelemetry(uint8_t address, uint8_t command, uint8_t data1, uint8_t data0, uint8_t data_received[], int bytes_returned, int delay);
void setPDMTimerLimit(uint8_t timerLimit, uint8_t selectedPDM);
void manualReset(uint8_t address);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  uint8_t address;
	  uint8_t returnData[4] = {0b00000000, 0b00000000, 0b00000000, 0b00000000};
	  uint8_t data1 = 0b00000000;
	  uint8_t data0 = 0b00000000;
	  uint8_t command = 0b00000000;

	  char select = 0;
	  int returnBytes = 0;
	  int delay = 0;
	  int timer = 0;
	  int PDM = 0;


	  //Read Select and Address

	  if (select == 's'){

		  getStatus(address,returnData);

	  }

	  else if (select == 'e'){

		  getError(address,returnData);

	  }

	  else if (select == 'r'){

		  manualReset(address);
	  }

	  else if (select == 't'){
		  //Read command,data1,data0,returnbytes,delay
		  getTelemetry(address,command,data1,data0,returnData,returnBytes,delay);
	  }

	  else if (select == 'c'){
		  // Read command,data1,returnbytes,delay
		  sendCommand(address,command,data1,returnData,returnBytes,delay);

	  }
	  else if (select == 'p'){
		  // Read Timer Limit and PDM
		  setPDMTimerLimit(timer,PDM);
	  }

	  else {
		  printf("Command not recognized");
	  }

	  //print the array

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
