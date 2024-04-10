/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LIS3DSH.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define flag_msg_user 		(0x00)
#define flag_msg_rtc 		(0x01)
#define flag_msg_acc		(0x02)

//#define RXBUFF_SIZE 512
//#define MAINBUFF_SIZE 2048
#define RXBUFF_SIZE 11

#define sampling_rate 1600 / 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for myUserMsg */
osThreadId_t myUserMsgHandle;
const osThreadAttr_t myUserMsg_attributes = {
  .name = "myUserMsg",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myRTCTime */
osThreadId_t myRTCTimeHandle;
const osThreadAttr_t myRTCTime_attributes = {
  .name = "myRTCTime",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myGetAccData */
osThreadId_t myGetAccDataHandle;
const osThreadAttr_t myGetAccData_attributes = {
  .name = "myGetAccData",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myIntTask */
osThreadId_t myIntTaskHandle;
const osThreadAttr_t myIntTask_attributes = {
  .name = "myIntTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myfreeRTOSManager */
osThreadId_t myfreeRTOSManagerHandle;
const osThreadAttr_t myfreeRTOSManager_attributes = {
  .name = "myfreeRTOSManager",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myProcessAccData */
osThreadId_t myProcessAccDataHandle;
const osThreadAttr_t myProcessAccData_attributes = {
  .name = "myProcessAccData",
  .stack_size = 2096 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myProcessCMD */
osThreadId_t myProcessCMDHandle;
const osThreadAttr_t myProcessCMD_attributes = {
  .name = "myProcessCMD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LIS3DSH_DataScaled */
osMessageQueueId_t LIS3DSH_DataScaledHandle;
const osMessageQueueAttr_t LIS3DSH_DataScaled_attributes = {
  .name = "LIS3DSH_DataScaled"
};
/* Definitions for LIS3DSH_ProcessedData */
osMessageQueueId_t LIS3DSH_ProcessedDataHandle;
const osMessageQueueAttr_t LIS3DSH_ProcessedData_attributes = {
  .name = "LIS3DSH_ProcessedData"
};
/* Definitions for RTC_Data */
osMessageQueueId_t RTC_DataHandle;
const osMessageQueueAttr_t RTC_Data_attributes = {
  .name = "RTC_Data"
};
/* Definitions for CMD_ProcData */
osMessageQueueId_t CMD_ProcDataHandle;
const osMessageQueueAttr_t CMD_ProcData_attributes = {
  .name = "CMD_ProcData"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
void MY_User_Message(void *argument);
void MY_RTC_Time(void *argument);
void MY_Get_Acc_Data(void *argument);
void my_Int_Task(void *argument);
void MY_freeRTOS_Task_Manager(void *argument);
void MY_Process_AccData(void *argument);
void MY_Process_CMD(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile unsigned long ulHighFrequencyTimerTicks;

struct My_TaskManager{
	volatile int freemem;
	char taskListBuffer [1024];
	char taskTimingBuffer [1024];
};

struct My_TaskManager task_manager;

//RTC_DateTypeDef sDate;
//RTC_TimeTypeDef sTime;

typedef struct {
	RTC_DateTypeDef sDate;
	RTC_TimeTypeDef sTime;
}My_RTC_Data;

uint8_t flag = flag_msg_user;

LIS3DSH_DataScaled g_myLIS3DSH;
LIS3DSH_DataScaled sg_myLIS3DSH;

uint8_t Rx_Buf[RXBUFF_SIZE];
uint16_t oldPos, newPos = 0;
uint8_t msg_buffer[1024] = {0};
uint32_t packet_counter = 0;



typedef struct {
	uint8_t cmd_id[3];
    uint8_t cmd_data[6];
} Command;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Rx_Buf, RXBUFF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of LIS3DSH_DataScaled */
  LIS3DSH_DataScaledHandle = osMessageQueueNew (3, sizeof(LIS3DSH_DataScaled), &LIS3DSH_DataScaled_attributes);

  /* creation of LIS3DSH_ProcessedData */
  LIS3DSH_ProcessedDataHandle = osMessageQueueNew (3, sizeof(LIS3DSH_DataScaled), &LIS3DSH_ProcessedData_attributes);

  /* creation of RTC_Data */
  RTC_DataHandle = osMessageQueueNew (3, sizeof(My_RTC_Data), &RTC_Data_attributes);

  /* creation of CMD_ProcData */
  CMD_ProcDataHandle = osMessageQueueNew (3, sizeof(Command), &CMD_ProcData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myUserMsg */
  myUserMsgHandle = osThreadNew(MY_User_Message, NULL, &myUserMsg_attributes);

  /* creation of myRTCTime */
  myRTCTimeHandle = osThreadNew(MY_RTC_Time, NULL, &myRTCTime_attributes);

  /* creation of myGetAccData */
  myGetAccDataHandle = osThreadNew(MY_Get_Acc_Data, NULL, &myGetAccData_attributes);

  /* creation of myIntTask */
  myIntTaskHandle = osThreadNew(my_Int_Task, NULL, &myIntTask_attributes);

  /* creation of myfreeRTOSManager */
  myfreeRTOSManagerHandle = osThreadNew(MY_freeRTOS_Task_Manager, NULL, &myfreeRTOSManager_attributes);

  /* creation of myProcessAccData */
  myProcessAccDataHandle = osThreadNew(MY_Process_AccData, NULL, &myProcessAccData_attributes);

  /* creation of myProcessCMD */
  myProcessCMDHandle = osThreadNew(MY_Process_CMD, NULL, &myProcessCMD_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == MEMS_INT1_Pin){
		osThreadFlagsSet(myGetAccDataHandle, 0x0001);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
}

int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if (huart->Instance == USART2) {
		osThreadFlagsSet(myProcessCMDHandle, 0x0001);
		HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *) Rx_Buf, RXBUFF_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}
/** ----------------------------------------------------------------------------------------------
* @brief
* This function calculates the pitch and roll angles based on the accelerometer readings.
*
* @param
* ax: Accelerometer reading along the X-axis.
* ay: Accelerometer reading along the Y-axis.
* az: Accelerometer reading along the Z-axis.
* @param
* pitch: Pointer to a variable where the calculated pitch angle will be stored.
* roll: Pointer to a variable where the calculated roll angle will be stored.
*
* @returns
* This function does not return a value directly, but it updates the values pointed to by 'pitch' and 'roll'
* with the calculated pitch and roll angles, respectively.
*/
void calculate_pitch_roll(float ax, float ay, float az, float *pitch, float *roll) {
//	float norm = sqrt(ax*ax + ay*ay + az*az);
//	ax /= norm;
//	ay /= norm;
//	az /= norm;
    *pitch = atan2(ax, sqrt(ay*ay + az*az));
    *roll = atan2(-ay, sqrt(ax*ax + az*az));

    *pitch = *pitch * 180.0 / M_PI;
    *roll = *roll * 180.0 / M_PI;
}
/** ----------------------------------------------------------------------------------------------
* @brief
* This function applies a moving average filter to the input accelerometer data and returns the filtered data.
*
* @param
* myLIS3DSH: The input accelerometer data to be filtered.
* mov_avg: A 2D array representing the moving average buffer for each axis (x, y, z).
* fill: A pointer to a variable indicating the current fill level of the moving average buffer.
*
* @returns
* Returns the filtered accelerometer data scaled to the LIS3DSH_DataScaled structure.
*/
LIS3DSH_DataScaled moving_average_filter(LIS3DSH_DataScaled myLIS3DSH, float mov_avg[3][sampling_rate], uint16_t* fill) {
    double avg[3] = {0};
    LIS3DSH_DataScaled LIS3DSH;

    if (*fill >= sampling_rate)
        *fill = 0;

    mov_avg[0][*fill] = myLIS3DSH.x;
    mov_avg[1][*fill] = myLIS3DSH.y;
    mov_avg[2][*fill] = myLIS3DSH.z;

    for (uint16_t i = 0; i < sampling_rate; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            avg[j] += mov_avg[j][i];
        }
    }

    LIS3DSH.x = avg[0] / sampling_rate;
	LIS3DSH.y = avg[1] / sampling_rate;
	LIS3DSH.z = avg[2] / sampling_rate;

    (*fill)++;

    return LIS3DSH;
}
/** ----------------------------------------------------------------------------------------------
* @brief
* Checks and parses a command received via UART from a buffer.
*
* @param
* rx_buf: Pointer to the buffer containing the received command.
*
* @returns
* Returns a Command structure representing the parsed command. If the received command
* meets the expected format, the parsed command is returned with its components filled accordingly.
* Otherwise, a default Command structure with "UN" as cmd_id and "KNOWN" as cmd_data is returned,
* and the rx_buf is cleared.
*/
Command check_command_from_uart(uint8_t *rx_buf){
	Command cmd = {0};
	if ((rx_buf[0] == '<') && (rx_buf[9] == '>') && (rx_buf[3] == ' ')) {
		memcpy(cmd.cmd_id, &rx_buf[1], 2);
		memcpy(cmd.cmd_data, &rx_buf[4], 5);
	} else {
		memcpy(cmd.cmd_id, "UN", 2);
		memcpy(cmd.cmd_data, "KNOWN", 5);
		memset(rx_buf, 0, RXBUFF_SIZE);
	}
	return cmd;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_MY_User_Message */
/**
  * @brief  Function implementing the myUserMsg thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_MY_User_Message */
void MY_User_Message(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  LIS3DSH_DataScaled myLIS3DSH;
  My_RTC_Data RTC_data;
  My_RTC_Data RTC_time_to_set;
  Command cmd;
  size_t size = 0;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(LIS3DSH_ProcessedDataHandle, &myLIS3DSH, NULL, osWaitForever);
	osMessageQueueGet(RTC_DataHandle, &RTC_data, NULL, osWaitForever);
	osMessageQueueGet(CMD_ProcDataHandle, &cmd, NULL, osWaitForever);
	if(strcmp((char *)cmd.cmd_id, "RO") == 0) {
		if(strcmp((char *)cmd.cmd_data, "ACCEL") == 0){
			size = sprintf ((char *)msg_buffer, "Time: %02d-%02d-%02d %02d:%02d:%02d\nPitch: %f\nRoll: %f\n", RTC_data.sDate.Date, RTC_data.sDate.Month, RTC_data.sDate.Year, RTC_data.sTime.Hours, RTC_data.sTime.Minutes, RTC_data.sTime.Seconds, myLIS3DSH.pitch, myLIS3DSH.roll);
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		} else if(strcmp((char *)cmd.cmd_data, "RAWAC") == 0) {
			size = sprintf ((char *)msg_buffer, "Time: %02d-%02d-%02d %02d:%02d:%02d\nRaw x: %f\nRaw y: %f\nRaw z: %f\n", RTC_data.sDate.Date, RTC_data.sDate.Month, RTC_data.sDate.Year, RTC_data.sTime.Hours, RTC_data.sTime.Minutes, RTC_data.sTime.Seconds, myLIS3DSH.x, myLIS3DSH.y, myLIS3DSH.z);
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		} else if(strcmp((char *)cmd.cmd_data, "RTCDA") == 0) {
			size = sprintf ((char *)msg_buffer, "%02d-%02d-20%02d\n", RTC_data.sDate.Date, RTC_data.sDate.Month, RTC_data.sDate.Year);
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		} else if(strcmp((char *)cmd.cmd_data, "RTCTI") == 0) {
			size = sprintf ((char *)msg_buffer, "%02d:%02d:%02d\n", RTC_data.sTime.Hours, RTC_data.sTime.Minutes, RTC_data.sTime.Seconds);
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		} else if(strcmp((char *)cmd.cmd_data, "RTCHO") == 0) {
			size = sprintf ((char *)msg_buffer, "%02d-%02d-20%02d %02d:%02d:%02d\n",RTC_data.sDate.Date, RTC_data.sDate.Month, RTC_data.sDate.Year, RTC_data.sTime.Hours, RTC_data.sTime.Minutes, RTC_data.sTime.Seconds);
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		} else if(strcmp((char *)cmd.cmd_data, "-HELP") == 0) {
			size = sprintf ((char *)msg_buffer, "Available read commands\nRTCDA - get DD/MM/YY\nRTCTI - get HH/MM/SS\nRTCHO - get both time and date\nACCEL - get boards angles relative to sensors measurement\n");
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		} else {
			size = sprintf ((char *)msg_buffer, "Unknown read command. Available command can view with -HELP \nCMD_DATA: %s\n", (char *) cmd.cmd_data);
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		}
	} else if(strcmp((char *)cmd.cmd_id, "WO") == 0) {
		if(cmd.cmd_data[0] == 'T' && cmd.cmd_data[1] == 'S' && cmd.cmd_data[2] == ':'){
			cmd.cmd_data[3] = (cmd.cmd_data[3] - '0') * 10;
			cmd.cmd_data[4] = cmd.cmd_data[4] - '0' + cmd.cmd_data[3];
			if(cmd.cmd_data[4] <= 60 && cmd.cmd_data[4] >= 0)
				RTC_time_to_set.sTime.Seconds = ((cmd.cmd_data[4] / 10) << 4) | (cmd.cmd_data[4] % 10);
		} else if(cmd.cmd_data[0] == 'T' && cmd.cmd_data[1] == 'M' && cmd.cmd_data[2] == ':') {
			cmd.cmd_data[3] = (cmd.cmd_data[3] - '0') * 10;
			cmd.cmd_data[4] = cmd.cmd_data[4] - '0' + cmd.cmd_data[3];
			if(cmd.cmd_data[4] <= 60 && cmd.cmd_data[4] >= 0)
				RTC_time_to_set.sTime.Minutes = ((cmd.cmd_data[4] / 10) << 4) | (cmd.cmd_data[4] % 10);
		} else if(cmd.cmd_data[0] == 'T' && cmd.cmd_data[1] == 'H' && cmd.cmd_data[2] == ':') {
			cmd.cmd_data[3] = (cmd.cmd_data[3] - '0') * 10;
			cmd.cmd_data[4] = cmd.cmd_data[4] - '0' + cmd.cmd_data[3];
			if(cmd.cmd_data[4] <= 24 && cmd.cmd_data[4] >= 0)
				RTC_time_to_set.sTime.Hours = ((cmd.cmd_data[4] / 10) << 4) | (cmd.cmd_data[3] + cmd.cmd_data[4] % 10);
		} else if(cmd.cmd_data[0] == 'D' && cmd.cmd_data[1] == 'D' && cmd.cmd_data[2] == ':'){
			cmd.cmd_data[3] = (cmd.cmd_data[3] - '0') * 10;
			cmd.cmd_data[4] = cmd.cmd_data[4] - '0' + cmd.cmd_data[3];
			if(cmd.cmd_data[4] <= 31 && cmd.cmd_data[4] >= 0)
				RTC_time_to_set.sDate.Date = ((cmd.cmd_data[4] / 10) << 4) | (cmd.cmd_data[4] % 10);
		} else if(cmd.cmd_data[0] == 'D' && cmd.cmd_data[1] == 'M' && cmd.cmd_data[2] == ':') {
			cmd.cmd_data[3] = (cmd.cmd_data[3] - '0') * 10;
			cmd.cmd_data[4] = cmd.cmd_data[4] - '0' + cmd.cmd_data[3];
			if(cmd.cmd_data[4] <= 12 && cmd.cmd_data[4] >= 0)
				RTC_time_to_set.sDate.Month = ((cmd.cmd_data[4] / 10) << 4) | (cmd.cmd_data[4] % 10);
		} else if(cmd.cmd_data[0] == 'D' && cmd.cmd_data[1] == 'W' && cmd.cmd_data[2] == ':') {
			cmd.cmd_data[3] = (cmd.cmd_data[3] - '0') * 10;
			cmd.cmd_data[4] = cmd.cmd_data[4] - '0' + cmd.cmd_data[3];
			if(cmd.cmd_data[4] <= 7 && cmd.cmd_data[4] >= 0)
				RTC_time_to_set.sDate.WeekDay = ((cmd.cmd_data[4] / 10) << 4) | (cmd.cmd_data[4] % 10);
		}else if(cmd.cmd_data[0] == 'D' && cmd.cmd_data[1] == 'Y' && cmd.cmd_data[2] == ':') {
			cmd.cmd_data[3] = (cmd.cmd_data[3] - '0') * 10;
			cmd.cmd_data[4] = cmd.cmd_data[4] - '0' + cmd.cmd_data[3];
			if(cmd.cmd_data[4] <= 100 && cmd.cmd_data[4] >= 0)
				RTC_time_to_set.sDate.Year = ((cmd.cmd_data[4] / 10) << 4) | (cmd.cmd_data[4] % 10);
		} else if(strcmp((char *)cmd.cmd_data, "TCONF") == 0) {
			HAL_RTC_SetDate(&hrtc, &RTC_time_to_set.sDate, RTC_FORMAT_BCD);
			HAL_RTC_SetTime(&hrtc, &RTC_time_to_set.sTime, RTC_FORMAT_BCD);
		} else if(strcmp((char *)cmd.cmd_data, "-HELP") == 0) {
			size = sprintf ((char *)msg_buffer, "Available write commands\nSETDA - set DD/MM/YY\nSETTI - set HH/MM/SS\nSETHO - set both time and date\n");
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		} else {
			size = sprintf ((char *)msg_buffer, "Unknown write command. Available command can view with -HELP \nCMD_DATA: %s\n", (char *) cmd.cmd_data);
			HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
			printf("%s", msg_buffer);
		}
	} else {
		size = sprintf ((char *)msg_buffer, "Unknown command.\nCMD ID: %s\nCMD_DATA: %s\n", (char *)cmd.cmd_id, (char *) cmd.cmd_data);
		HAL_UART_Transmit(&huart2, msg_buffer, size, 2000);
		printf("%s", msg_buffer);
	}

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MY_RTC_Time */
/**
* @brief Function implementing the myRTCTime thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MY_RTC_Time */
void MY_RTC_Time(void *argument)
{
  /* USER CODE BEGIN MY_RTC_Time */
  My_RTC_Data RTC_data;
  /* Infinite loop */
  for(;;)
  {
	osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
	HAL_RTC_GetTime(&hrtc, &RTC_data.sTime, RTC_FORMAT_BCD);
	RTC_data.sTime.Hours = ((RTC_data.sTime.Hours & 0xF0) >> 4) * 10 + (RTC_data.sTime.Hours & 0x0F);
	RTC_data.sTime.Minutes = ((RTC_data.sTime.Minutes & 0xF0) >> 4) * 10 + (RTC_data.sTime.Minutes & 0x0F);
	RTC_data.sTime.Seconds = ((RTC_data.sTime.Seconds & 0xF0) >> 4) * 10 + (RTC_data.sTime.Seconds & 0x0F);
	HAL_RTC_GetDate(&hrtc, &RTC_data.sDate, RTC_FORMAT_BCD);
	RTC_data.sDate.Year = ((RTC_data.sDate.Year & 0xF0) >> 4) * 10 + (RTC_data.sDate.Year & 0x0F);
	RTC_data.sDate.Month = ((RTC_data.sDate.Month & 0xF0) >> 4) * 10 + (RTC_data.sDate.Month & 0x0F);
	RTC_data.sDate.Date = ((RTC_data.sDate.Date & 0xF0) >> 4) * 10 + (RTC_data.sDate.Date & 0x0F);
	osMessageQueuePut(RTC_DataHandle, &RTC_data, 10, 1);
  }
  /* USER CODE END MY_RTC_Time */
}

/* USER CODE BEGIN Header_MY_Get_Acc_Data */
/**
* @brief Function implementing the myGetAccData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MY_Get_Acc_Data */
void MY_Get_Acc_Data(void *argument)
{
  /* USER CODE BEGIN MY_Get_Acc_Data */
	LIS3DSH_InitTypeDef MY_LIS3DSH_Configuration;
	MY_LIS3DSH_Configuration.dataRate = LIS3DSH_DATARATE_1600;
	MY_LIS3DSH_Configuration.fullScale = LIS3DSH_FULLSCALE_4;
	MY_LIS3DSH_Configuration.antiAliasingBW = LIS3DSH_FILTER_BW_50;
	MY_LIS3DSH_Configuration.enableAxes = LIS3DSH_XYZ_ENABLE;
	MY_LIS3DSH_Configuration.interruptEnable = true;
	LIS3DSH_Init(&hspi1, &MY_LIS3DSH_Configuration);
	LIS3DSH_DataScaled myLIS3DSH;
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
	  myLIS3DSH = LIS3DSH_GetDataScaled();
	  g_myLIS3DSH = myLIS3DSH;
	  osMessageQueuePut(LIS3DSH_DataScaledHandle, &myLIS3DSH, 10, 1);
  }
  /* USER CODE END MY_Get_Acc_Data */
}

/* USER CODE BEGIN Header_my_Int_Task */
/**
* @brief Function implementing the myIntTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_my_Int_Task */
void my_Int_Task(void *argument)
{
  /* USER CODE BEGIN my_Int_Task */
  /* Infinite loop */
  for(;;)
  {
	  vTaskSuspend(NULL);
//	  if(++flag > 2)
//			flag = 0;
//		while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)){
//			osDelay(50);
//		}
  }
  /* USER CODE END my_Int_Task */
}

/* USER CODE BEGIN Header_MY_freeRTOS_Task_Manager */
/**
* @brief Function implementing the myfreeRTOSManager thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MY_freeRTOS_Task_Manager */
void MY_freeRTOS_Task_Manager(void *argument)
{
  /* USER CODE BEGIN MY_freeRTOS_Task_Manager */
  /* Infinite loop */
  for(;;)
  {
	  task_manager.freemem = xPortGetFreeHeapSize();
	  vTaskList(task_manager.taskListBuffer);
	  vTaskGetRunTimeStats(task_manager.taskTimingBuffer);
	  osDelay(1000);
  }
  /* USER CODE END MY_freeRTOS_Task_Manager */
}

/* USER CODE BEGIN Header_MY_Process_AccData */
/**
* @brief Function implementing the myProcessAccData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MY_Process_AccData */
void MY_Process_AccData(void *argument)
{
  /* USER CODE BEGIN MY_Process_AccData */
	LIS3DSH_DataScaled myLIS3DSH;
	LIS3DSH_DataScaled myLIS3DSH_after_filter;
	uint16_t fill = 0;
	uint8_t fill_buff = 0;
	float moving_avg[3][sampling_rate] = {{0}};
  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(LIS3DSH_DataScaledHandle, &myLIS3DSH, NULL, osWaitForever);
	if(fill < sampling_rate && fill_buff == 0){
		moving_avg[0][fill] = myLIS3DSH.x;
		moving_avg[1][fill] = myLIS3DSH.y;
		moving_avg[2][fill] = myLIS3DSH.z;
		fill++;
		if(fill == sampling_rate)
			fill_buff = 1;
	} else {
		myLIS3DSH_after_filter = moving_average_filter(myLIS3DSH, moving_avg, &fill);
		sg_myLIS3DSH = myLIS3DSH_after_filter;
		calculate_pitch_roll(myLIS3DSH_after_filter.x, myLIS3DSH_after_filter.y , myLIS3DSH_after_filter.z, &myLIS3DSH_after_filter.pitch, &myLIS3DSH_after_filter.roll);
		osMessageQueuePut(LIS3DSH_ProcessedDataHandle, &myLIS3DSH_after_filter, 10, 1);
	}
  }
  /* USER CODE END MY_Process_AccData */
}

/* USER CODE BEGIN Header_MY_Process_CMD */
/**
* @brief Function implementing the myProcessCMD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MY_Process_CMD */
void MY_Process_CMD(void *argument)
{
  /* USER CODE BEGIN MY_Process_CMD */
	Command cmd;
  /* Infinite loop */
  for(;;)
  {
	osThreadFlagsWait(0x0001, osFlagsWaitAll, osWaitForever);
	cmd = check_command_from_uart(Rx_Buf);
	if (strcmp((char *)cmd.cmd_id, "UN") != 0 && strcmp((char *)cmd.cmd_data, "KNOWN") != 0) {
		osMessageQueuePut(CMD_ProcDataHandle, &cmd, 10, 1);
	}
  }
  /* USER CODE END MY_Process_CMD */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim->Instance == TIM3){
		ulHighFrequencyTimerTicks++;
	}
	if(htim->Instance == TIM2){
		osThreadFlagsSet(myRTCTimeHandle, 0x0001);
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
