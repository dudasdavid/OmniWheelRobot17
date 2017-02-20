/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId sensorTaskHandle;
osThreadId controlTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static volatile float radius = 0.1;
static volatile float kaka = 0.0;

static volatile int16_t rawWheel1 = 0;
static volatile int16_t rawWheel2 = 0;
static volatile int16_t rawWheel3 = 0;

static float toMeterFactor = 0.0082;
static float posWheel1 = 0.0;
static float posWheel2 = 0.0;
static float posWheel3 = 0.0;
static float posWheel1_prev = 0.0;
static float posWheel2_prev = 0.0;
static float posWheel3_prev = 0.0;
static volatile float spdWheel1 = 0.0;
static volatile float spdWheel2 = 0.0;
static volatile float spdWheel3 = 0.0;
static volatile float spdWheel1prev = 0.0;
static volatile float spdWheel2prev = 0.0;
static volatile float spdWheel3prev = 0.0;
static float spdWheel1Filtered = 0.0;
static float spdWheel2Filtered = 0.0;
static float spdWheel3Filtered = 0.0;
static float spdWheel1Norm = 0.0;
static float spdWheel2Norm = 0.0;
static float spdWheel3Norm = 0.0;
static int8_t signWheel1 = 1;
static int8_t signWheel2 = 1;
static int8_t signWheel3 = 1;

static uint8_t receiveBuffer[3]={0};
static volatile float referenceSpeed = 0;
static volatile float referenceAngle = 0;
static volatile float referenceTurn = 0;

static volatile float referenceSpeedWheel1 = 0;
static volatile float referenceSpeedWheel2 = 0;
static volatile float referenceSpeedWheel3 = 0;

static volatile float spdErrorWheel1 = 0;
static volatile float spdErrorWheel1Before = 0;
static volatile float spdIntErrorWheel1 = 0;
static int16_t forceWheel1 = 0;
static int16_t forceWheel1BeforeSaturation = 0;
static volatile float spdDerivatedErrorWheel1 = 0;
static int16_t absForceWheel1 = 0;

static volatile float spdErrorWheel2 = 0;
static volatile float spdErrorWheel2Before = 0;
static volatile float spdIntErrorWheel2 = 0;
static int16_t forceWheel2 = 0;
static int16_t forceWheel2BeforeSaturation = 0;
static volatile float spdDerivatedErrorWheel2 = 0;
static int16_t absForceWheel2 = 0;

static volatile float spdErrorWheel3 = 0;
static volatile float spdErrorWheel3Before = 0;
static volatile float spdIntErrorWheel3 = 0;
static int16_t forceWheel3 = 0;
static int16_t forceWheel3BeforeSaturation = 0;
static volatile float spdDerivatedErrorWheel3 = 0;
static int16_t absForceWheel3 = 0;

static volatile float antiWindup = 1;
static volatile float ctrlP = 50;
static volatile float ctrlI = 0.05;
static volatile float ctrlD = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void StartSensorTask(void const * argument);
void StartControlTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define ABS(x)         (x < 0) ? (-x) : x
#define PI 3.14159265
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_4);
  
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_2);
  
  //motor1
  TIM4->CCR1 = 0*2400/100;
  TIM4->CCR2 = 0*2400/100;
  //motor2
  TIM4->CCR3 = 0*2400/100;
  TIM4->CCR4 = 0*2400/100;
  //motor3
  TIM15->CCR1 = 0*2400/100;
  TIM15->CCR2 = 0*2400/100;
  
  
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  
  
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI); 
  BSP_LED_Init(LED4);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED7);
  BSP_LED_Init(LED9);
  BSP_LED_Init(LED10);
  BSP_LED_Init(LED8);
  BSP_LED_Init(LED6);
  
  BSP_GYRO_Init();
  BSP_ACCELERO_Init();
  MAGNET_Init();
  
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityHigh, 0, 128);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 3;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 3;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 3;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2400;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 2400;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim15);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
     PA11   ------> USB_DM
     PA12   ------> USB_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin 
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin 
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin 
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin 
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin 
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin 
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DM_Pin DP_Pin */
  GPIO_InitStruct.Pin = DM_Pin|DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF14_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float filter2 (float avg, float input, float alpha) {
  avg = (alpha * input) + (1.0 - alpha) * avg;
  return avg;
}

float absFloat(float num){
  if (num >= 0) return num;
  else return num*-1;
}

void saturateInteger(int16_t* i, int16_t min, int16_t max) {
  int16_t val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}
void saturateFloat(float* i, float min, float max) {
  float val = *i;
  if (val < min) val = min;
  else if (val > max) val = max;
  *i = val;
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Receive_IT(&huart1,receiveBuffer,3);
    
    if ((receiveBuffer[0] == 0xFF) && (receiveBuffer[1] <=100) && (receiveBuffer[2] <=180)){
      referenceSpeed = receiveBuffer[1] / 60.0;
      referenceAngle = receiveBuffer[2] * 2.0;
    }
    else {
      referenceSpeed = 0;
      referenceAngle = 0;
      referenceTurn = 0;
    }
    
    osDelay(20);
  }
  /* USER CODE END 5 */ 
}

/* StartSensorTask function */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
  uint16_t taskCounter = 0;
  /* Infinite loop */
  for(;;)
  {
    //rawWheel1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
    taskCounter++;
    if (taskCounter % 5 == 0){
      rawWheel1 = TIM1->CNT;
      rawWheel2 = TIM2->CNT;
      rawWheel3 = TIM3->CNT;
      
      posWheel1_prev = posWheel1;
      posWheel2_prev = posWheel2;
      posWheel3_prev = posWheel3;
      
      posWheel1 = rawWheel1*toMeterFactor;
      posWheel2 = rawWheel2*toMeterFactor;
      posWheel3 = rawWheel3*toMeterFactor;
       
      spdWheel1 = absFloat(posWheel1 - posWheel1_prev) * signWheel1;
      spdWheel2 = absFloat(posWheel2 - posWheel2_prev) * signWheel2;
      spdWheel3 = absFloat(posWheel3 - posWheel3_prev) * signWheel3;
      
      spdWheel1Filtered = filter2(spdWheel1Filtered, spdWheel1, 0.35);
      spdWheel2Filtered = filter2(spdWheel2Filtered, spdWheel2, 0.35);
      spdWheel3Filtered = filter2(spdWheel3Filtered, spdWheel3, 0.35);
      
      spdWheel1Norm = spdWheel1Filtered * 20;
      spdWheel2Norm = spdWheel2Filtered * 20;
      spdWheel3Norm = spdWheel3Filtered * 20;
    }
    /*
    if ((ABS(spdWheel1Filtered - spdWheel1) > 4.0)){
      kaka = ABS(spdWheel1Filtered - spdWheel1);
      spdWheel1Filtered = filter2(spdWheel1Filtered, spdWheel1, 0.1);
    }
    else {
      spdWheel1Filtered = filter2(spdWheel1Filtered, spdWheel1, 0.35);
    }
    if ((ABS(spdWheel2Filtered - spdWheel2) > 4.0)){
      kaka = ABS(spdWheel2Filtered - spdWheel2);
      spdWheel2Filtered = filter2(spdWheel2Filtered, spdWheel2, 0.1);
    }
    else {
      spdWheel2Filtered = filter2(spdWheel2Filtered, spdWheel2, 0.35);
    }
    if ((ABS(spdWheel3Filtered - spdWheel3) > 4.0)){
      kaka = ABS(spdWheel3Filtered - spdWheel3);
      spdWheel3Filtered = filter2(spdWheel3Filtered, spdWheel3, 0.1);
    }
    else {
      spdWheel3Filtered = filter2(spdWheel3Filtered, spdWheel3, 0.35);
    }
    */
    

    
    osDelay(10);
  }
  /* USER CODE END StartSensorTask */
}

/* StartControlTask function */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
    
    referenceSpeedWheel1 = referenceSpeed * cosf(referenceAngle * PI / 180.0) + radius * referenceTurn;
    referenceSpeedWheel2 = referenceSpeed * (-0.5 * cosf(referenceAngle * PI / 180.0) + 0.866 * sinf(referenceAngle * PI / 180.0)) + radius * referenceTurn;
    referenceSpeedWheel3 = referenceSpeed * (-0.5 * cosf(referenceAngle * PI / 180.0) - 0.866 * sinf(referenceAngle * PI / 180.0)) + radius * referenceTurn;
    
    spdErrorWheel1Before = spdErrorWheel1;
    spdErrorWheel1 = referenceSpeedWheel1 - spdWheel1Norm;
    spdIntErrorWheel1 = spdIntErrorWheel1 + spdErrorWheel1 + antiWindup * (forceWheel1 - forceWheel1BeforeSaturation);
    spdDerivatedErrorWheel1 = spdErrorWheel1 - spdErrorWheel1Before;
      
    forceWheel1 = (int)(ctrlP * spdErrorWheel1 + ctrlI * spdIntErrorWheel1 + ctrlD * spdDerivatedErrorWheel1);
    forceWheel1BeforeSaturation = forceWheel1;
    
    spdErrorWheel2Before = spdErrorWheel2;
    spdErrorWheel2 = referenceSpeedWheel2 - spdWheel2Norm;
    spdIntErrorWheel2 = spdIntErrorWheel2 + spdErrorWheel2 + antiWindup * (forceWheel2 - forceWheel2BeforeSaturation);
    spdDerivatedErrorWheel2 = spdErrorWheel2 - spdErrorWheel2Before;
      
    forceWheel2 = (int)(ctrlP * spdErrorWheel2 + ctrlI * spdIntErrorWheel2 + ctrlD * spdDerivatedErrorWheel2);
    forceWheel2BeforeSaturation = forceWheel2;
    
    spdErrorWheel3Before = spdErrorWheel3;
    spdErrorWheel3 = referenceSpeedWheel3 - spdWheel3Norm;
    spdIntErrorWheel3 = spdIntErrorWheel3 + spdErrorWheel3 + antiWindup * (forceWheel3 - forceWheel3BeforeSaturation);
    spdDerivatedErrorWheel3 = spdErrorWheel3 - spdErrorWheel3Before;
      
    forceWheel3 = (int)(ctrlP * spdErrorWheel3 + ctrlI * spdIntErrorWheel3 + ctrlD * spdDerivatedErrorWheel3);
    forceWheel3BeforeSaturation = forceWheel3;
    
    //if ((absFloat(referenceSpeed) < 0.05) && (absFloat(spdErrorWheel1) < 0.1)){
    //  spdIntErrorWheel1-=spdIntErrorWheel1*0.05;
    //}
    
    ////// PWM OUT //////
    saturateInteger(&forceWheel1, -80, 80);
    absForceWheel1 = ABS(forceWheel1);
    saturateInteger(&forceWheel2, -80, 80);
    absForceWheel2 = ABS(forceWheel2);
    saturateInteger(&forceWheel3, -80, 80);
    absForceWheel3 = ABS(forceWheel3);
    
    /*
    absForceWheel1 = 0;
    absForceWheel2 = 0;
    absForceWheel3 = 0;
    */
    
    if (forceWheel1 >= 0) {
      TIM4->CCR1 = 0;
      TIM4->CCR2 = absForceWheel1*2400/100;
      signWheel1 = 1;
    }
    else {
      TIM4->CCR1 = absForceWheel1*2400/100;
      TIM4->CCR2 = 0;
      signWheel1 = -1;
    }
    if (forceWheel2 >= 0) {
      TIM4->CCR3 = 0;
      TIM4->CCR4 = absForceWheel2*2400/100;
      signWheel2 = 1;
    }
    else {
      TIM4->CCR3 = absForceWheel2*2400/100;
      TIM4->CCR4 = 0;
      signWheel2 = -1;
    }
    if (forceWheel3 >= 0) {
      TIM15->CCR1 = 0;
      TIM15->CCR2 = absForceWheel3*2400/100;
      signWheel3 = 1;
    }
    else {
      TIM15->CCR1 = absForceWheel3*2400/100;
      TIM15->CCR2 = 0;
      signWheel3 = -1;
    }
    
    
    
    osDelay(100);
  }
  /* USER CODE END StartControlTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
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
