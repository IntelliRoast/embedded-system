/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
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
#include "stm32f2xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
#include "max31855.h"
#include "he_pid.h"
#include "data_types.h"
#include "DefaultRoasts.h"
#include "jsmn.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId RoastTaskHandle;
osThreadId commTaskHandle;
osMessageQId transmittQueueHandle;
osMessageQId recieveQueueHandle;
osMutexId wireSerial_mutexHandle;
osMutexId btSerial_mutexHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static profile_t (*Profile)[];
static progress_t Progress;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);
void StartRoastTask(void const * argument);
void StartCommTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	Progress.State = idle;
	Progress.time = 0;
	Progress.bt = 0;
	Progress.st = 0;
	Progress.et = 0;
	Progress.dc = 0;
	Profile = &TestRoast;

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_SPI1_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of wireSerial_mutex */
	osMutexDef(wireSerial_mutex);
	wireSerial_mutexHandle = osMutexCreate(osMutex(wireSerial_mutex));

	/* definition and creation of btSerial_mutex */
	osMutexDef(btSerial_mutex);
	btSerial_mutexHandle = osMutexCreate(osMutex(btSerial_mutex));

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
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);



	/* definition and creation of commTask */
	osThreadDef(commTask, StartCommTask, osPriorityBelowNormal, 0, 128);
	commTaskHandle = osThreadCreate(osThread(commTask), NULL);
	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s)
	 definition and creation of transmittQueue
	 what about the sizeof here??? cd native code
	osMessageQDef(transmittQueue, 16, struct progress);
	transmittQueueHandle = osMessageCreate(osMessageQ(transmittQueue), NULL);

	 definition and creation of recieveQueue
	 what about the sizeof here??? cd native code
	osMessageQDef(recieveQueue, 16, struct profile);
	recieveQueueHandle = osMessageCreate(osMessageQ(recieveQueue), NULL);
*/
	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 13;
	RCC_OscInitStruct.PLL.PLLN = 195;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* RTC init function */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/**Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/* SPI1 init function */
static void MX_SPI1_Init(void) {

	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 60000;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 4096;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 4096;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART3 init function */
void MX_USART3_UART_Init(void) {

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
									;

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, SPI2_CS0_Pin | SPI2_CS1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI2_CS0_Pin SPI2_CS1_Pin */
	GPIO_InitStruct.Pin = SPI2_CS0_Pin | SPI2_CS1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period,
		uint16_t pulse) {
	HAL_TIM_PWM_Stop(&timer, channel);
	// stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period;
	// set the period duration
	HAL_TIM_PWM_Init(&timer); // reinititialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	// set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel);
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument) {

	/* USER CODE BEGIN 5 */
	HAL_UART_Transmit_DMA(&huart3, "IntelliRoast initializing\n\n", 29);

	unsigned int Connected = 0;

	osDelay(3000);
	/* definition and creation of RoastTask */
	osThreadDef(RoastTask, StartRoastTask, osPriorityAboveNormal, 0, 256);
	RoastTaskHandle = osThreadCreate(osThread(RoastTask), NULL);
	/* Infinite loop */
	for (;;) {
		char temp_msg[100] = {0};
		HAL_UART_Receive(&huart2,temp_msg, 100, 0xFFF);
		if (strlen(temp_msg) != 0) {
			HAL_UART_Transmit(&huart3, '1',1,0xFFF);
			jsmn_parser parser;
			jsmntok_t tokens[100];
			jsmn_init(&parser);

			// js - pointer to JSON string
			// tokens - an array of tokens available
			// 10 - number of tokens available
			jsmn_parse(&parser, temp_msg, strlen(temp_msg), tokens, 100);
		}
		osDelay(10);
	}

	/* USER CODE END 5 */
}

/* StartRoastTask function */
void StartRoastTask(void const * argument) {
	/* USER CODE BEGIN StartRoastTask */

	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, SPI2_CS0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, SPI2_CS1_Pin, GPIO_PIN_SET);

	uint8_t spi_data[4] = { 0 };
	float slope;
	uint16_t heDutyCycle;
	char temp_msg[80] = { 0 };

	Progress.State = roasting;

	//TODO: Set Fan Speed to Agitation

	/* Infinite loop */
	for (int current_point = 1; current_point < 6; current_point++) {
		for (int current_time = 0; Progress.time < (*Profile)[current_point].time; current_time++) {
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			//calculate slope
			int delta_temp = (*Profile)[current_point].temp - (*Profile)[current_point-1].temp;
			int delta_time = (*Profile)[current_point].time - (*Profile)[current_point-1].time;
			slope = (float) delta_temp / delta_time;
			Progress.st = (slope * current_time) + (*Profile)[current_point-1].temp;

			//Gather Bean Temp
			HAL_GPIO_WritePin(GPIOF, SPI2_CS0_Pin, GPIO_PIN_RESET);
			HAL_SPI_Receive(&hspi1, spi_data, 4, 0xFF);
			HAL_Delay(1);
			HAL_GPIO_WritePin(GPIOF, SPI2_CS0_Pin, GPIO_PIN_SET);
			if (max31855_Error(spi_data)) {
				if (max31855_Disconnected(spi_data)) {
					sprintf(temp_msg,
							"ERROR: Bean Thermocouple Disconnected\n");
				} else if (max31855_ShortVCC(spi_data)) {
					sprintf(temp_msg,
							"ERROR: Bean Thermocouple Shorted to VCC\n");
				} else if (max31855_ShortGND(spi_data)) {
					sprintf(temp_msg,
							"ERROR: Bean Thermocouple Shorted to GND\n");
				} else {
					sprintf(temp_msg,
							"ERROR: Bean Thermocouple has unknown error\n");
				}
				HAL_UART_Transmit(&huart3, (uint8_t*) temp_msg,
						strlen(temp_msg), 0xFFF);
			}
			Progress.bt = max31855toCelcius(spi_data);

			//Gather Heating Element Temp
			HAL_GPIO_WritePin(GPIOF, SPI2_CS1_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_SPI_Receive(&hspi1, spi_data, 4, 0xFF);
			HAL_GPIO_WritePin(GPIOF, SPI2_CS1_Pin, GPIO_PIN_SET);
			if (max31855_Error(spi_data)) {
				if (max31855_Disconnected(spi_data)) {
					sprintf(temp_msg,
							"ERROR: HE Thermocouple Disconnected\n");
				} else if (max31855_ShortVCC(spi_data)) {
					sprintf(temp_msg,
							"ERROR: HE Thermocouple Shorted to VCC\n");
				} else if (max31855_ShortGND(spi_data)) {
					sprintf(temp_msg,
							"ERROR: HE Thermocouple Shorted to GND\n");
				} else {
					sprintf(temp_msg,
							"ERROR: HE Thermocouple has unknown error\n");
				}
				HAL_UART_Transmit(&huart3, (uint8_t*) temp_msg,
						strlen(temp_msg), 0xFFF);
			}
			Progress.et = max31855toCelcius(spi_data);

			heDutyCycle = HE_PID(Progress.bt, Progress.st, 0);
			Progress.dc = ((float) heDutyCycle * 100) / 65536;
			setPWM(htim2, TIM_CHANNEL_1, 999, heDutyCycle);

			//sprintf(temp_msg, "Time: %d, BT: %d, ST: %d, ET: %d, DC: %d\n", time, Progress.bt, Progress.st, Progress.et, (int) heDutyCyclePercentage);
			sprintf(temp_msg, "%d,%d,%d,%d,%d\n",Progress.time, Progress.bt, Progress.st, Progress.et, Progress.dc);
			HAL_UART_Transmit(&huart3, (uint8_t*) temp_msg, strlen(temp_msg), 0xFFF); //Serial Port

			if ((Progress.bt >= (*Profile)[5].temp) || (Progress.time >= (*Profile)[5].time)) {
				Progress.State = cooling;
				break;
			}

			Progress.time++;
			osDelay(1000);
		}

		if (Progress.State != roasting) {
			break;
		}
	}

	/* Finish up roast by cooling off beans */
	setPWM(htim2, TIM_CHANNEL_1, 2000, 0); // cut off heater
	setPWM(htim3, TIM_CHANNEL_1, 2048, COOLING_DC); //TODO: Find correct percentage.

	do{
		HAL_GPIO_WritePin(GPIOF, SPI2_CS0_Pin, GPIO_PIN_RESET);
		HAL_SPI_Receive(&hspi1, spi_data, 4, 0xFF);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOF, SPI2_CS0_Pin, GPIO_PIN_SET);
		if (max31855_Error(spi_data)) {
			if (max31855_Disconnected(spi_data)) {
				sprintf(temp_msg, "ERROR: Bean Thermocouple Disconnected\n");
			} else if (max31855_ShortVCC(spi_data)) {
				sprintf(temp_msg,
						"ERROR: Bean Thermocouple Shorted to VCC\n");
			} else if (max31855_ShortGND(spi_data)) {
				sprintf(temp_msg,
						"ERROR: Bean Thermocouple Shorted to GND\n");
			} else {
				sprintf(temp_msg,
						"ERROR: Bean Thermocouple has unknown error\n");
			}
			HAL_UART_Transmit(&huart3, (uint8_t*) temp_msg,
					strlen(temp_msg), 0xFFF);
		}
		Progress.bt = max31855toCelcius(spi_data);
	} while(Progress.bt > 40 && Progress.State == cooling);

	setPWM(htim3, TIM_CHANNEL_1, 2048, EJECT_DC); //Set fan to full power to eject.

	sprintf(temp_msg, "ROAST_FINISHED\n");
	HAL_UART_Transmit(&huart3, (uint8_t*) temp_msg, strlen(temp_msg), 0xFFF);
	/* USER CODE END StartRoastTask */
}

/* StartCommTask function */
void StartCommTask(void const * argument) {
	/* USER CODE BEGIN StartCommTask */
	char temp_msg[80] = {0};
	char state_str[20] = {0};
	/* Infinite loop */
	for (;;) {
		osSignalWait(0x1, osWaitForever);
		switch(Progress.State){
		case idle:
			sprintf(state_str,"Idle");
			break;
		case roasting:
			sprintf(state_str,"Roasting");
			break;
		case cooling:
			sprintf(state_str,"Cooling");
			break;
		case ejecting:
			sprintf(state_str,"Ejecting");
			break;
		}
		sprintf(temp_msg, "{\"state\":\"%s\",\"T\":%d,\"BT\":%d,\"ST\":%d,\"ET\":%d,\"DC\":%d}",
							state_str,
							Progress.time,
							Progress.bt,
							Progress.st,
							Progress.et,
							Progress.dc);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t *) temp_msg, strlen(temp_msg));
	}
	/* USER CODE END StartCommTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
