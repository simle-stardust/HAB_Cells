/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "LTC2983.h"
#define ARM_MATH_CM3
#include "arm_math.h"
#define LED_CONTROL_ADDR  (0x40 << 1)
#define DS3231Addr (0b1101000 << 1)

#define SET_TEMPERATURE  36.6f

#define PID_PARAM_KP  1.0f
#define PID_PARAM_KI  0.01f
#define PID_PARAM_KD  0.01f

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId SaveTaskHandle;
osThreadId WatchdogTaskHandle;
osThreadId HeatingTaskHandle;
osThreadId ControlTaskHandle;
osMessageQId qToHeatingTaskHandle;
osMessageQId qToSaveTaskHandle;
osMessageQId qToWatchdogTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

FATFS my_fatfs;
FIL my_file;
UINT my_error;

typedef struct saveData_t_def {
	int32_t Temps[30];
	uint8_t Signals[12];
} saveData_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
void StartSaveTask(void const * argument);
void StartWatchdogTask(void const * argument);
void StartHeatingTask(void const * argument);
void StartControlTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
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
  MX_ADC_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of SaveTask */
  osThreadDef(SaveTask, StartSaveTask, osPriorityNormal, 0, 3060);
  SaveTaskHandle = osThreadCreate(osThread(SaveTask), NULL);

  /* definition and creation of WatchdogTask */
  osThreadDef(WatchdogTask, StartWatchdogTask, osPriorityNormal, 0, 128);
  WatchdogTaskHandle = osThreadCreate(osThread(WatchdogTask), NULL);

  /* definition and creation of HeatingTask */
  osThreadDef(HeatingTask, StartHeatingTask, osPriorityRealtime, 0, 128);
  HeatingTaskHandle = osThreadCreate(osThread(HeatingTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityHigh, 0, 1024);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of qToHeatingTask */
  osMessageQDef(qToHeatingTask, 4, 12);
  qToHeatingTaskHandle = osMessageCreate(osMessageQ(qToHeatingTask), NULL);

  /* definition and creation of qToSaveTask */
  osMessageQDef(qToSaveTask, 2, saveData_t);
  qToSaveTaskHandle = osMessageCreate(osMessageQ(qToSaveTask), NULL);

  /* definition and creation of qToWatchdogTask */
  osMessageQDef(qToWatchdogTask, 5, uint8_t);
  qToWatchdogTaskHandle = osMessageCreate(osMessageQ(qToWatchdogTask), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LTC1_RST_Pin|LTC1_CS_Pin|LTC2_CS_Pin|LTC3_RST_Pin 
                          |LTC3_CS_Pin|LTC5_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LowerHeater1_Pin|UpperHeater1_Pin|LowerHeater2_Pin|UpperHeater2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LTC2_RST_Pin|LTC6_CS_Pin|LTC6_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LowerHeater3_Pin|UpperHeater3_Pin|LowerHeater4_Pin|UpperHeater4_Pin 
                          |LowerHeater5_Pin|UpperHeater5_Pin|WiFi_GPIO0_Pin|WiFI_CH_PD_Pin 
                          |WiFi_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LTC4_CS_Pin|LTC4_RST_Pin|LTC5_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LowerHeater6_Pin|UpperHeater6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LTC1_RST_Pin LTC1_CS_Pin LTC2_CS_Pin LTC3_RST_Pin 
                           LTC3_CS_Pin LowerHeater6_Pin UpperHeater6_Pin LTC5_RST_Pin */
  GPIO_InitStruct.Pin = LTC1_RST_Pin|LTC1_CS_Pin|LTC2_CS_Pin|LTC3_RST_Pin 
                          |LTC3_CS_Pin|LowerHeater6_Pin|UpperHeater6_Pin|LTC5_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LowerHeater1_Pin UpperHeater1_Pin LowerHeater2_Pin UpperHeater2_Pin 
                           LTC2_RST_Pin LTC6_CS_Pin LTC6_RST_Pin */
  GPIO_InitStruct.Pin = LowerHeater1_Pin|UpperHeater1_Pin|LowerHeater2_Pin|UpperHeater2_Pin 
                          |LTC2_RST_Pin|LTC6_CS_Pin|LTC6_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LowerHeater3_Pin UpperHeater3_Pin LTC4_CS_Pin LTC4_RST_Pin 
                           LowerHeater4_Pin UpperHeater4_Pin LTC5_CS_Pin LowerHeater5_Pin 
                           UpperHeater5_Pin WiFi_GPIO0_Pin WiFI_CH_PD_Pin WiFi_RST_Pin */
  GPIO_InitStruct.Pin = LowerHeater3_Pin|UpperHeater3_Pin|LTC4_CS_Pin|LTC4_RST_Pin 
                          |LowerHeater4_Pin|UpperHeater4_Pin|LTC5_CS_Pin|LowerHeater5_Pin 
                          |UpperHeater5_Pin|WiFi_GPIO0_Pin|WiFI_CH_PD_Pin|WiFi_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static uint16_t WriteToSD(uint8_t* buf, uint8_t len)
{
	char my_file_name[20];
	static uint8_t datalogNum = 0;
	uint16_t returnCode = 0;
	FRESULT fresult;
	uint32_t fileSize;

	memset(my_file_name, ' ', sizeof(my_file_name));
	sprintf(my_file_name, "cells%d.txt", datalogNum);
	__disable_irq();
	fresult = f_open(&my_file, my_file_name, FA_WRITE | FA_OPEN_ALWAYS);
	fileSize = f_size(&my_file);
	if (fileSize > 1000000)
	{
		datalogNum++;
		sprintf(my_file_name, "cells%d.txt", datalogNum);
	}
	fresult = f_close(&my_file);
	__enable_irq();
	__disable_irq();
	fresult = f_open(&my_file, my_file_name, FA_WRITE | FA_OPEN_ALWAYS);
	if (fresult == FR_OK)
	{
		fresult = f_lseek(&my_file, fileSize);
		fresult = f_write(&my_file, buf, len, &my_error);
		if (fresult == FR_OK)
		{
			fresult = f_close(&my_file);
			if (fresult != FR_OK)
			{
				returnCode = 1;
			}
		}
		else
		{
			returnCode = 2;
		}
	}
	else
	{
		returnCode = 3;
	}
	__enable_irq();
	osDelay(10);
	return returnCode;
}

void arm_pid_init_f32(
  arm_pid_instance_f32 * S,
  int32_t resetStateFlag)
{

  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;

  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float32_t) 2.0 * S->Kd);

  /* Derived coefficient A2 */
  S->A2 = S->Kd;

  /* Check whether state needs reset or not */
  if(resetStateFlag)
  {
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(S->state, 0, 3u * sizeof(float32_t));
  }

}
/* USER CODE END 4 */

/* StartSaveTask function */
void StartSaveTask(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
	const char initMessage[] = "\r\nRESET\r\n";
	uint8_t SDBuffer[64];
	uint8_t SDBufLen = 0;
	saveData_t data;

	f_mount(&my_fatfs, SD_Path, 1);
	WriteToSD((uint8_t*) initMessage, sizeof(initMessage));
	memset(SDBuffer, 0, sizeof(SDBuffer));
	/* Infinite loop */
	for (;;)
	{
		// Fetch values from all other tasks
		if (xQueueReceive(qToSaveTaskHandle, &data, 0) == pdTRUE)
		{
			// Write to SD and WiFi

			SDBufLen = sprintf((char*)SDBuffer, "@MarcinSetValuesKom:");
			HAL_UART_Transmit_IT(&huart1, SDBuffer, SDBufLen);
			// If write failed send signal to watchdog task to reset the program
			for (uint32_t i = 0; i < 30; i+=5)
			{

				SDBufLen = sprintf((char*)SDBuffer, "%ld, %ld, %ld, %ld, %ld, ",
						data.Temps[i], data.Temps[i+1], data.Temps[i+2], data.Temps[i+3], data.Temps[i+4]);
				if (SDBufLen > 0)
				{
					WriteToSD(SDBuffer, SDBufLen);
					HAL_UART_Transmit_IT(&huart3, SDBuffer, SDBufLen);
					HAL_UART_Transmit_IT(&huart1, SDBuffer, SDBufLen);
				}
			}
			for (uint32_t i = 0; i < 12; i+=6)
			{
				SDBufLen = sprintf((char*)SDBuffer, "%hu, %hu, %hu, %hu, %hu, %hu, ",
						data.Signals[i], data.Signals[i+1], data.Signals[i+2], data.Signals[i+3], data.Signals[i+4], data.Signals[i+5]);
				if (SDBufLen > 0)
				{
					WriteToSD(SDBuffer, SDBufLen);
					HAL_UART_Transmit_IT(&huart3, SDBuffer, SDBufLen);
					HAL_UART_Transmit_IT(&huart1, SDBuffer, SDBufLen);
				}
			}
			SDBufLen = sprintf((char*)SDBuffer, "\r\n");
			WriteToSD(SDBuffer, SDBufLen);
			HAL_UART_Transmit_IT(&huart3, SDBuffer, SDBufLen);
			SDBufLen = sprintf((char*)SDBuffer, "!\r\n");
			HAL_UART_Transmit_IT(&huart1, SDBuffer, SDBufLen);

			// UART and SD transactions done, send data to WiFi now
			// set up flags




		}

		osDelay(100);
	}
  /* USER CODE END 5 */ 
}

/* StartWatchdogTask function */
void StartWatchdogTask(void const * argument)
{
  /* USER CODE BEGIN StartWatchdogTask */
	/* Infinite loop */
	for (;;)
	{
		osDelay(100);
	}
  /* USER CODE END StartWatchdogTask */
}

/* StartHeatingTask function */
void StartHeatingTask(void const * argument)
{
  /* USER CODE BEGIN StartHeatingTask */
	uint8_t ControlValues[12] =
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	GPIO_TypeDef * HeatingPorts[12] =
	{
	LowerHeater1_GPIO_Port, LowerHeater2_GPIO_Port, LowerHeater3_GPIO_Port,
	LowerHeater4_GPIO_Port, LowerHeater5_GPIO_Port, LowerHeater6_GPIO_Port,
	UpperHeater1_GPIO_Port, UpperHeater2_GPIO_Port, UpperHeater3_GPIO_Port,
	UpperHeater4_GPIO_Port, UpperHeater5_GPIO_Port, UpperHeater6_GPIO_Port };
	uint16_t HeatingPins[12] =
	{
	LowerHeater1_Pin, LowerHeater2_Pin, LowerHeater3_Pin, LowerHeater4_Pin,
	LowerHeater5_Pin, LowerHeater6_Pin, UpperHeater1_Pin, UpperHeater2_Pin,
	UpperHeater3_Pin, UpperHeater4_Pin, UpperHeater5_Pin, UpperHeater6_Pin };

	/* Infinite loop */
	for (;;)
	{
		// Get values from the control task, if there are any new
		xQueueReceive(qToHeatingTaskHandle, ControlValues, 0);

		// Lower Sample 1 heating
		for (uint8_t i = 0; i < 12; i++)
		{
			if (ControlValues[i] > 0)
			{
				HAL_GPIO_WritePin(HeatingPorts[i], HeatingPins[i],
						GPIO_PIN_SET);
				osDelay(ControlValues[i]);
				HAL_GPIO_WritePin(HeatingPorts[i], HeatingPins[i],
						GPIO_PIN_RESET);
			}
			osDelay(100 - ControlValues[i]);
		}
	}
  /* USER CODE END StartHeatingTask */
}

/* StartControlTask function */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */

	// array that holds current temperatures of each sample
	// indexes 0 - 5   LowerSample1,2,3,4,5,6
	// indexes 6 - 11  UpperSample1,2,3,4,5,6
	// indexes 12 - 17 LowerHeater1,2,3,4,5,6
	// indexes 18 - 23 UpperHeater1,2,3,4,5,6
	// indexes 24 - 29 Ambient1,2,3,4,5,6
	float Temperatures[30];

	saveData_t data;

	// mean temperatures that will be used as current temperature for PID algorithms, Lower6 then Upper6
	float CalculatedTemps[12];
	// outputs from PID for every sample, Lower6 then Upper6
	float ControlSignals[12];
	// converted outputs to 0-100% value to send to heating task, Lower6 than Upper6

	/* ARM PID Instance, float_32 format */
	arm_pid_instance_f32 PID[12];

	for (uint32_t i = 0; i < 12; i++)
	{
		PID[i].Kp = PID_PARAM_KP; /* Proporcional */
		PID[i].Ki = PID_PARAM_KI; /* Integral */
		PID[i].Kd = PID_PARAM_KD; /* Derivative */
		arm_pid_init_f32(&PID[i], 1);
	}

	// Configure LTCs
	ConfigureLTCs();

	/* Infinite loop */
	for (;;)
	{
		// Read LTCs, store readouts in the array
		// To do: return status bitmask and check for errors
		//
		ReadLTCs(&Temperatures[0], UPPER_SAMPLE_MODE);
		ReadLTCs(&Temperatures[6], LOWER_SAMPLE_MODE);
		ReadLTCs(&Temperatures[12], UPPER_HEATER_MODE);
		ReadLTCs(&Temperatures[18], LOWER_HEATER_MODE);
		ReadLTCs(&Temperatures[24], AMBIENT_MODE);

		// Calculate control signals and send them to heating task
		for (uint32_t i = 0; i < 6; i++)
		{
			// first lower samples, 0.9 * sample + 0.1 * heater
			CalculatedTemps[i] = 0.9f * Temperatures[i]
					+ 0.1f * Temperatures[12 + i];
		}
		for (uint32_t i = 6; i < 12; i++)
		{
			// then upper samples
			CalculatedTemps[i] = 0.9f * Temperatures[6 + i]
					+ 0.1f * Temperatures[18 + i];
		}
		for (uint32_t i = 0; i < 12; i++)
		{
			// PID calculations now
			ControlSignals[i] = arm_pid_f32(&PID[i],
					SET_TEMPERATURE - CalculatedTemps[i]);
			if (ControlSignals[i] > 100.0f)
			{
				ControlSignals[i] = 100.0f;
			}
			else if (ControlSignals[i] < 0.0f)
			{
				ControlSignals[i] = 0.0f;
			}

			// convert to integral value
			data.Signals[i] = (uint8_t) ControlSignals[i];
		}

		for (uint32_t i = 0; i < 30; i++)
		{
			data.Temps[i] = (int32_t)(Temperatures[i] * 1000);
		}

		// Send current control to HeatingTask
		xQueueSend(qToHeatingTaskHandle, &data.Signals, 1);

		// Send current control and temperatures to Save task
		xQueueSend(qToSaveTaskHandle, &data, 1);

		osDelay(1000);
	}
  /* USER CODE END StartControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1)
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
