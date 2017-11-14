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
#include <stdlib.h>
#include <tgmath.h>
#include "filter.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;

IRDA_HandleTypeDef hirda4;
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

static float toMeterFactor = 0.00294;
static float posWheel1 = 0.0;
static float posWheel2 = 0.0;
static float posWheel3 = 0.0;
static float posWheel1_prev = 0.0;
static float posWheel2_prev = 0.0;
static float posWheel3_prev = 0.0;
static volatile float spdWheel1 = 0.0;
static volatile float spdWheel2 = 0.0;
static volatile float spdWheel3 = 0.0;
static volatile float spdWheel1_prev = 0.0;
static volatile float spdWheel2_prev = 0.0;
static volatile float spdWheel3_prev = 0.0;
static float spdWheel1Filtered = 0.0;
static float spdWheel2Filtered = 0.0;
static float spdWheel3Filtered = 0.0;
static float spdWheel1Norm = 0.0;
static float spdWheel2Norm = 0.0;
static float spdWheel3Norm = 0.0;


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
static volatile int16_t forceWheel1BeforeSaturation = 0;
static volatile float spdDerivatedErrorWheel1 = 0;
static int16_t absForceWheel1 = 0;

static volatile float spdErrorWheel2 = 0;
static volatile float spdErrorWheel2Before = 0;
static volatile float spdIntErrorWheel2 = 0;
static int16_t forceWheel2 = 0;
static volatile int16_t forceWheel2BeforeSaturation = 0;
static volatile float spdDerivatedErrorWheel2 = 0;
static int16_t absForceWheel2 = 0;

static volatile float spdErrorWheel3 = 0;
static volatile float spdErrorWheel3Before = 0;
static volatile float spdIntErrorWheel3 = 0;
static int16_t forceWheel3 = 0;
static volatile int16_t forceWheel3BeforeSaturation = 0;
static volatile float spdDerivatedErrorWheel3 = 0;
static int16_t absForceWheel3 = 0;

static volatile float antiWindup = 2;
static volatile float ctrlP = 20;
static volatile float ctrlI = 1;
static volatile float ctrlD = 3;

static volatile float gyroX,gyroY,gyroZ = 0;
static volatile float compassX,compassY,compassZ = 0;
static volatile float compassX_Max,compassY_Max,compassZ_Max = 0;
static volatile float compassX_Min,compassY_Min,compassZ_Min = 0;
static volatile uint8_t initialCompassCalib = 1;

static volatile float compassX_Range,compassY_Range,compassZ_Range = 0;
static volatile float compassX_Norm,compassY_Norm,compassZ_Norm = 0;
static volatile float compassX_NormFiltered,compassY_NormFiltered,compassZ_NormFiltered = 0;
static volatile float compassX_NormFilteredTilted,compassY_NormFilteredTilted = 0.0f;
static volatile float roll,pitch,heading = 0;
static volatile float temperature,temperatureGyro = 0;
static volatile float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f;
static volatile float accX,accY,accZ = 0;
static volatile float accXFiltered,accYFiltered,accZFiltered = 0;
static volatile float accXFilteredKalman,accYFilteredKalman,accZFilteredKalman = 0;
static volatile float accXFilteredLPF,accYFilteredLPF,accZFilteredLPF = 0;

static volatile uint16_t rawADC = 0;
static volatile float batteryVoltage = 0;
static volatile float cellVoltage = 0;
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
static void MX_UART4_IRDA_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void StartSensorTask(void const * argument);
void StartControlTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define PI 3.14159265
#define COMPASS_X_MAX 310
#define COMPASS_X_MIN -800
#define COMPASS_Y_MAX 250
#define COMPASS_Y_MIN -830
#define COMPASS_Z_MAX 2500
#define COMPASS_Z_MIN 1100

#define BATTERYCELLS 3
#define NORMALVOLTAGE 3.6
#define LOWVOLTAGE 3.0
#define CRITICALVOLTAGE 2.6
#define INDICATORBRIGHTNESS 0.1
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
  MX_UART4_IRDA_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();

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
  
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_4);
  
  //motor1
  TIM4->CCR1 = 0*2400/100;
  TIM4->CCR2 = 0*2400/100;
  //motor2
  TIM4->CCR3 = 0*2400/100;
  TIM4->CCR4 = 0*2400/100;
  //motor3
  TIM15->CCR1 = 0*2400/100;
  TIM15->CCR2 = 0*2400/100;
  
  //led R
  TIM8->CCR2 = 0*2400/100;
  //led B
  TIM8->CCR3 = 0*2400/100;
  //led G
  TIM8->CCR4 = 0*2400/100;
  
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  
  HAL_ADC_Start(&hadc1);
  
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the ADC multi-mode 
    */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

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

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 3;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2400;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim8);

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

/* UART4 init function */
static void MX_UART4_IRDA_Init(void)
{

  hirda4.Instance = UART4;
  hirda4.Init.BaudRate = 115200;
  hirda4.Init.WordLength = IRDA_WORDLENGTH_8B;
  hirda4.Init.Parity = IRDA_PARITY_NONE;
  hirda4.Init.Mode = IRDA_MODE_TX_RX;
  hirda4.Init.Prescaler = 1;
  hirda4.Init.PowerMode = IRDA_POWERMODE_NORMAL;
  if (HAL_IRDA_Init(&hirda4) != HAL_OK)
  {
    Error_Handler();
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

void saturateVolatileFloat(volatile float* i, float min, float max) {
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
  uint16_t taskCounter = 0;
  uint16_t heartBeatCounter = 0;
  
  float voltageFullRange = NORMALVOLTAGE - LOWVOLTAGE;
  float voltageInRange = 0;
  float voltageRatio = 0;
  /* Infinite loop */
  for(;;)
  {
    taskCounter++;
    
    HAL_UART_Receive_IT(&huart1,receiveBuffer,3);
    
    if ((receiveBuffer[0] == 0xFF) && (receiveBuffer[1] <=100) && (receiveBuffer[2] <=180)){
      referenceSpeed = receiveBuffer[1] / 6.0;
      referenceAngle = receiveBuffer[2] * 2.0 - 60 + 180;
      referenceTurn = 0;
    }
    else {
      referenceSpeed = 0;
      referenceAngle = 0;
      referenceTurn = 0;
    }
    
    CompassLED(heading);
    
    if (cellVoltage >= NORMALVOLTAGE) {
      TIM8->CCR2 = (int)(0*2400/100*INDICATORBRIGHTNESS); //led R
      TIM8->CCR4 = (int)(100*2400/100*INDICATORBRIGHTNESS); //led G
      TIM8->CCR3 = (int)(0*2400/100*INDICATORBRIGHTNESS); //led B
    }
    else if (cellVoltage < NORMALVOLTAGE && cellVoltage >= LOWVOLTAGE) {
      voltageInRange = cellVoltage - LOWVOLTAGE;
      voltageRatio = voltageInRange / voltageFullRange;
      
      TIM8->CCR2 = (int)(100*(1-voltageRatio)*2400/100*INDICATORBRIGHTNESS); //led R
      TIM8->CCR4 = (int)(100*voltageRatio*2400/100*INDICATORBRIGHTNESS); //led G
      TIM8->CCR3 = (int)(0*2400/100*INDICATORBRIGHTNESS); //led B
    }
    else if (cellVoltage < LOWVOLTAGE) {
      TIM8->CCR4 = (int)(0*2400/100*INDICATORBRIGHTNESS); //led G
      TIM8->CCR3 = (int)(0*2400/100*INDICATORBRIGHTNESS); //led B
      if (taskCounter % 4 == 0) {
        if (heartBeatCounter == 0) TIM8->CCR2 = (int)(100*2400/100*INDICATORBRIGHTNESS); //led R;
        else if (heartBeatCounter == 3) TIM8->CCR2 = (int)(0*2400/100*INDICATORBRIGHTNESS); //led R;
        else if (heartBeatCounter == 5) TIM8->CCR2 = (int)(100*2400/100*INDICATORBRIGHTNESS); //led R;
        else if (heartBeatCounter == 8) TIM8->CCR2 = (int)(0*2400/100*INDICATORBRIGHTNESS); //led R;
        else if (heartBeatCounter == 12) heartBeatCounter = -1;
        heartBeatCounter++;
      }
    }
    else {
      TIM8->CCR2 = (int)(100*2400/100*INDICATORBRIGHTNESS); //led R
      TIM8->CCR4 = (int)(100*2400/100*INDICATORBRIGHTNESS); //led G
      TIM8->CCR3 = (int)(100*2400/100*INDICATORBRIGHTNESS); //led B
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
  float gyroBuffer[3];
  float compassBuffer[3];
  float accelerometerBuffer[3] = {0};
  //float temperatureBuffer[1] = {0};
  float temperatureBufferGyro[1] = {0};
  /* Infinite loop */
  for(;;)
  {
    taskCounter++;
    
    rawWheel2 = TIM1->CNT;
    rawWheel3 = (TIM2->CNT)*-1;
    rawWheel1 = (TIM3->CNT)*-1;
    
    posWheel1_prev = posWheel1;
    posWheel2_prev = posWheel2;
    posWheel3_prev = posWheel3;
    
    spdWheel1_prev = spdWheel1;
    spdWheel2_prev = spdWheel2;
    spdWheel3_prev = spdWheel3;
    
    posWheel1 = rawWheel1*toMeterFactor;
    posWheel2 = rawWheel2*toMeterFactor;
    posWheel3 = rawWheel3*toMeterFactor;
    
    
    if (fabs(posWheel1-posWheel1_prev) > 100) spdWheel1 = spdWheel1_prev;
    else spdWheel1 = (posWheel1 - posWheel1_prev);
    if (fabs(posWheel2-posWheel2_prev) > 100) spdWheel2 = spdWheel2_prev;
    else spdWheel2 = (posWheel2 - posWheel2_prev);
    if (fabs(posWheel3-posWheel3_prev) > 100) spdWheel3 = spdWheel3_prev;
    else spdWheel3 = (posWheel3 - posWheel3_prev);
   
    
    spdWheel1Filtered = filter2(spdWheel1Filtered, spdWheel1, 0.35);
    spdWheel2Filtered = filter2(spdWheel2Filtered, spdWheel2, 0.35);
    spdWheel3Filtered = filter2(spdWheel3Filtered, spdWheel3, 0.35);
    
    spdWheel1Norm = spdWheel1Filtered * 200;
    spdWheel2Norm = spdWheel2Filtered * 200;
    spdWheel3Norm = spdWheel3Filtered * 200;

    
    if (taskCounter % 4 == 0){
      /* Read Gyro Angular data */
      BSP_GYRO_GetXYZ(gyroBuffer);
      
      gyroX = gyroBuffer[0];
      gyroY = gyroBuffer[1];
      gyroZ = gyroBuffer[2];
      
      BSP_GYRO_GetTemp(temperatureBufferGyro);
      temperatureGyro = temperatureBufferGyro[0];
      
      /* Read Acceleration*/
      BSP_ACCELERO_GetXYZ(accelerometerBuffer);

      accX = accelerometerBuffer[0];
      accY = accelerometerBuffer[1];
      accZ = accelerometerBuffer[2];
      
      accXFiltered = filter2(accXFiltered, accX, 0.2);
      accYFiltered = filter2(accYFiltered, accY, 0.2);
      accZFiltered = filter2(accZFiltered, accZ, 0.2);
      
      accZFilteredKalman = kalman_single(accZ, 60, 10);
      accZFilteredLPF    = LPF(accZ,1,40);
      //roll  = atan2(accYFiltered, sqrt(accXFiltered*accXFiltered + accZFiltered*accZFiltered));
      //pitch = atan2(accXFiltered, sqrt(accYFiltered*accYFiltered + accZFiltered*accZFiltered));
      
      /* Read Magnetometer*/
      LSM303DLHC_MagReadXYZ(compassBuffer);
      compassX = compassBuffer[0];
      compassY = compassBuffer[1];
      compassZ = compassBuffer[2];
      
      if (initialCompassCalib == 1){
        compassX_Max = compassX;
        compassX_Min = compassX;
        compassY_Max = compassY;
        compassY_Min = compassY;
        compassZ_Max = compassZ;
        compassZ_Min = compassZ;
        initialCompassCalib = 0;
      }
      
      if (compassX > compassX_Max) compassX_Max = compassX;
      if (compassY > compassY_Max) compassY_Max = compassY;
      if (compassZ > compassZ_Max) compassZ_Max = compassZ;
      if (compassX < compassX_Min) compassX_Min = compassX;
      if (compassY < compassY_Min) compassY_Min = compassY;
      if (compassZ < compassZ_Min) compassZ_Min = compassZ;
      
      
      //The code you quoted first subtracts the minimum raw value from the reading,
      //then divides it by the range between the minimum and maximum raw values,
      //which puts the reading into the range of 0 to 1.
      //It then multiplies the reading by 2 and subtracts 1 to put it in the range of -1 to 1.
      
      saturateVolatileFloat(&compassX, COMPASS_X_MIN, COMPASS_X_MAX);
      saturateVolatileFloat(&compassY, COMPASS_Y_MIN, COMPASS_Y_MAX);
      saturateVolatileFloat(&compassZ, COMPASS_Z_MIN, COMPASS_Z_MAX);
      
      compassX_Norm = (compassX - COMPASS_X_MIN) / (COMPASS_X_MAX - COMPASS_X_MIN) * 2 - 1.0;
      compassY_Norm = (compassY - COMPASS_Y_MIN) / (COMPASS_Y_MAX - COMPASS_Y_MIN) * 2 - 1.0;
      compassZ_Norm = (compassZ - COMPASS_Z_MIN) / (COMPASS_Z_MAX - COMPASS_Z_MIN) * 2 - 1.0;
      
      compassX_NormFiltered = filter2(compassX_NormFiltered, compassX_Norm, 0.3);
      compassY_NormFiltered = filter2(compassY_NormFiltered, compassY_Norm, 0.3);
      compassZ_NormFiltered = filter2(compassZ_NormFiltered, compassZ_Norm, 0.3);
      
      
      fNormAcc = sqrt((accXFiltered*accXFiltered)+(accYFiltered*accYFiltered)+(accZFiltered*accZFiltered));
        
      fSinRoll = accYFiltered/fNormAcc;
      fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
      fSinPitch = -accXFiltered/fNormAcc;
      fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
        
      if ( fSinRoll >0) {
        if (fCosRoll>0) roll = acos(fCosRoll)*180/PI;
        else            roll = acos(fCosRoll)*180/PI + 180;
      }
      else {
        if (fCosRoll>0) roll = acos(fCosRoll)*180/PI + 360;
        else            roll = acos(fCosRoll)*180/PI + 180;
      }
       
      if ( fSinPitch >0) {
        if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI;
        else             pitch = acos(fCosPitch)*180/PI + 180;
      }
      else {
        if (fCosPitch>0) pitch = acos(fCosPitch)*180/PI + 360;
        else             pitch = acos(fCosPitch)*180/PI + 180;
      }

      if (roll >=360)  roll = 360 - roll;
      if (pitch >=360) pitch = 360 - pitch;
      
      compassX_NormFilteredTilted = compassX_NormFiltered*fCosPitch+compassZ_NormFiltered*fSinPitch;
      compassY_NormFilteredTilted = compassX_NormFiltered*fSinRoll*fSinPitch+compassY_NormFiltered*fCosRoll-compassZ_NormFiltered*fSinRoll*fCosPitch;
      //xh=cx*cos(ayf)+cy*sin(ayf)*sin(axf)-cz*cos(axf)*sin(ayf);
      //yh=cy*cos(axf)+cz*sin(axf);
      
      //compassX_NormFilteredTilted = compassX_NormFiltered*fCosRoll + compassY_NormFiltered*fSinRoll*fSinPitch - compassZ_NormFiltered*fCosPitch*fSinRoll;
      //compassY_NormFilteredTilted = compassY_NormFiltered*fCosPitch + compassZ_NormFiltered*fSinPitch;
      
      heading = (float) ((atan2f((float)compassY_NormFilteredTilted,(float)compassX_NormFilteredTilted))*180)/PI;
      heading += 180;
      
      if (heading < 0) heading = heading + 360;
      
      rawADC = HAL_ADC_GetValue(&hadc1);
      batteryVoltage = rawADC/198.8;
      cellVoltage = batteryVoltage / BATTERYCELLS;
      //cellVoltage = 2.9;
    }
    
    osDelay(5);
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
    saturateInteger(&forceWheel1, -100, 100);
    absForceWheel1 = ABS(forceWheel1);
    saturateInteger(&forceWheel2, -100, 100);
    absForceWheel2 = ABS(forceWheel2);
    saturateInteger(&forceWheel3, -100, 100);
    absForceWheel3 = ABS(forceWheel3);
    
    /*
    absForceWheel1 = 0;
    absForceWheel2 = 0;
    absForceWheel3 = 0;
    */
    
    if (forceWheel1 <= 0) {
      TIM4->CCR1 = 0;
      TIM4->CCR2 = absForceWheel1*2400/100;
    }
    else {
      TIM4->CCR1 = absForceWheel1*2400/100;
      TIM4->CCR2 = 0;
    }
    if (forceWheel2 <= 0) {
      TIM4->CCR3 = 0;
      TIM4->CCR4 = absForceWheel2*2400/100;
    }
    else {
      TIM4->CCR3 = absForceWheel2*2400/100;
      TIM4->CCR4 = 0;
    }
    if (forceWheel3 <= 0) {
      TIM15->CCR1 = 0;
      TIM15->CCR2 = absForceWheel3*2400/100;
    }
    else {
      TIM15->CCR1 = absForceWheel3*2400/100;
      TIM15->CCR2 = 0;
    }
    
    /*
    TIM4->CCR1 = 100*2400/100;
    TIM4->CCR2 = 0*2400/100;
    TIM4->CCR3 = 100*2400/100;
    TIM4->CCR4 = 0*2400/100;
    TIM15->CCR1 = 100*2400/100;
    TIM15->CCR2 = 0*2400/100;
    */
    osDelay(5);
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
