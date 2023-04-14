/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Creating a structure to specify the SPI communication with MAX7301 will be 2 byte. One is the register address and the second is the data send to the MAX7301
typedef struct
{
	uint8_t address;
	uint8_t value;
} ConfigParam_t;

//Creating a structure to hold the state of each antenna
typedef struct
{
	uint8_t antEnable;
}ConfigAnt;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for Enable */
osThreadId_t EnableHandle;
const osThreadAttr_t Enable_attributes = {
  .name = "Enable",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Init */
osThreadId_t InitHandle;
const osThreadAttr_t Init_attributes = {
  .name = "Init",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Test */
osThreadId_t TestHandle;
const osThreadAttr_t Test_attributes = {
  .name = "Test",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Update */
osThreadId_t UpdateHandle;
const osThreadAttr_t Update_attributes = {
  .name = "Update",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myBinarySem */
osSemaphoreId_t myBinarySemHandle;
const osSemaphoreAttr_t myBinarySem_attributes = {
  .name = "myBinarySem"
};
/* USER CODE BEGIN PV */
//Declaration of an array with enable value for 2 daisy chained MAX7301
static const ConfigParam_t arrayConfig[] = {{.address = 0x04, .value = 0x01},
											{.address = 0x04, .value = 0x01}};

//Declaration of an array with the initialization parameters. Port P4 to P27 set has output.
static const ConfigParam_t arrayInitialize[] = {{.address = 0x09, .value = 0x55},
												{.address = 0x0A, .value = 0x55},
												{.address = 0x0B, .value = 0x55},
												{.address = 0x0C, .value = 0x55},
												{.address = 0x0D, .value = 0x55},
												{.address = 0x0E, .value = 0x55}};
//Declaration of each antenna state. Each bit in each byte is to enable one antenna.
//Ex : ConfigRISA[0] control antenna 1 to 8, ConfigRISA[1] control antenna 9 to 17
// ConfigRISA[0].antEnable = 0000 0001 -> enable antenna 1 				and disable all others from this byte
// ConfigRISA[3].antEnable = 1000 1001 -> enable antenna 27, 30 and 34	and disable all others from this byte
static ConfigAnt ConfigRISA[] = 	{{.antEnable = 0xFF},		// [0] Antenna 1 	to 8
										 {.antEnable = 0xFF},		// [1] Antenna 9	to 16
										 {.antEnable = 0xFF},		// [2] Antenna 17 	to 24
										 {.antEnable = 0xFF},		// [3] Antenna 25 	to 32
										 {.antEnable = 0xFF},		// [4] Antenna 33 	to 40
										 {.antEnable = 0xFF}};		// [5] Antenna 41 	to 48

static ConfigAnt ConfigRISB[] = 	{{.antEnable = 0xFF},		// [0] Antenna 1 	to 8
										 {.antEnable = 0xFF},		// [1] Antenna 9	to 16
										 {.antEnable = 0xFF},		// [2] Antenna 17 	to 24
										 {.antEnable = 0xFF},		// [3] Antenna 25 	to 32
										 {.antEnable = 0xFF},		// [4] Antenna 33 	to 40
										 {.antEnable = 0xFF}};		// [5] Antenna 41 	to 48
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
void StartEnable(void *argument);
void StartInit(void *argument);
void StartTest(void *argument);
void StartUpdate(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of myBinarySem */
  myBinarySemHandle = osSemaphoreNew(1, 1, &myBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Enable */
  EnableHandle = osThreadNew(StartEnable, NULL, &Enable_attributes);

  /* creation of Init */
  InitHandle = osThreadNew(StartInit, NULL, &Init_attributes);

  /* creation of Test */
  TestHandle = osThreadNew(StartTest, NULL, &Test_attributes);

  /* creation of Update */
  UpdateHandle = osThreadNew(StartUpdate, NULL, &Update_attributes);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|SPI1_CS_Pin|SPI2_CS_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin SPI1_CS_Pin SPI2_CS_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|SPI1_CS_Pin|SPI2_CS_Pin
                          |LD2_Pin;
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEnable */
/**
  * @brief  Function implementing the Enable thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEnable */
void StartEnable(void *argument)
{
  /* USER CODE BEGIN 5 */
	// CS_LOW
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	// RISA
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[0].address, 1, 10);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[0].value, 1, 10);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[1].address, 1, 10);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayConfig[1].value, 1, 10);
	// RISB
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[0].address, 1, 10);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[0].value, 1, 10);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[1].address, 1, 10);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayConfig[1].value, 1, 10);
	// CS_HIGHT
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	uint8_t data[]= "Enable completed\n";
	HAL_UART_Transmit(&huart3, data, sizeof(data),500);
    /* Give semaphore to Task 2 */
	osSemaphoreRelease(myBinarySemHandle);
    /* Delete Task 1 */
    vTaskDelete(EnableHandle);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartInit */
/**
* @brief Function implementing the Init thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInit */
void StartInit(void *argument)
{
  /* USER CODE BEGIN StartInit */
    /* Wait for semaphore from Task 1 */
	osSemaphoreAcquire(myBinarySemHandle, osWaitForever);
	uint8_t i = 0;
	uint8_t data[]= "Init completed\n";
	for(i = 0; i < 6;i++)
	{
	// CS_LOW
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	// RISA
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].address, 1, 10);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].value, 1, 10);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].address, 1, 10);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)&arrayInitialize[i].value, 1, 10);
	// RISB
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].address, 1, 10);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].value, 1, 10);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].address, 1, 10);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)&arrayInitialize[i].value, 1, 10);
	// CS_HIGHT
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	HAL_UART_Transmit(&huart3, data, sizeof(data),500);
	}
    /* Give semaphore to Task 3 */
	osSemaphoreRelease(myBinarySemHandle);
    /* Delete Task 2 */
    vTaskDelete(InitHandle);
  /* USER CODE END StartInit */
}

/* USER CODE BEGIN Header_StartTest */
/**
* @brief Function implementing the Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTest */
void StartTest(void *argument)
{
  /* USER CODE BEGIN StartTest */
  /* Wait for semaphore from Task 2 */
	osSemaphoreAcquire(myBinarySemHandle, osWaitForever);
	uint8_t buffer[] = {0x24, 0x01};
	uint8_t i = 0;
	for(i = 0; i < 25; i++)
	{
		buffer[0] = 0x24 + i;
		buffer[1] = 0x01;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

		HAL_SPI_Transmit(&hspi1, (uint8_t *)&buffer, 2, 10);
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&buffer, 2, 10);

		HAL_SPI_Transmit(&hspi2, (uint8_t *)&buffer, 2, 10);
		HAL_SPI_Transmit(&hspi2, (uint8_t *)&buffer, 2, 10);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		vTaskDelay(100);
		//HAL_Delay(100);
		buffer[1] = 0x00;
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

		HAL_SPI_Transmit(&hspi1, (uint8_t *)&buffer, 2, 10);
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&buffer, 2, 10);

		HAL_SPI_Transmit(&hspi2, (uint8_t *)&buffer, 2, 10);
		HAL_SPI_Transmit(&hspi2, (uint8_t *)&buffer, 2, 10);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

		uint8_t data[]= "Test completed\n";
		HAL_UART_Transmit(&huart3, data, sizeof(data),500);
	}

  /* Delay for a longer period of time to simulate a longer execution time */
 // vTaskDelay(pdMS_TO_TICKS(5000));
  osSemaphoreRelease(myBinarySemHandle);
  /* Delete Task 3 */
  vTaskDelete(TestHandle);
  /* USER CODE END StartTest */
}

/* USER CODE BEGIN Header_StartUpdate */
/**
* @brief Function implementing the Update thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUpdate */
void StartUpdate(void *argument)
{
  /* USER CODE BEGIN StartUpdate */
  /* Infinite loop */
  for(;;)
  {       osSemaphoreAcquire(myBinarySemHandle, osWaitForever);
		//This part Configure 8 antenna (P4 to P11 on each MAX7301)
		uint8_t bufferRISA1[2] = {0x44, ConfigRISA[0].antEnable};
		uint8_t bufferRISA2[2] = {0x44, ConfigRISA[3].antEnable};

		uint8_t bufferRISB1[2] = {0x44, ConfigRISB[0].antEnable};
		uint8_t bufferRISB2[2] = {0x44, ConfigRISB[3].antEnable};

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

		HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

		HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
		HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);


		//This part Configure 8 antenna (P12 to P19 on each MAX7301)
		bufferRISA1[0] = 0x4C;
		bufferRISA1[1] = ConfigRISA[1].antEnable;
		bufferRISA2[0] = 0x4C;
		bufferRISA2[1] = ConfigRISA[4].antEnable;

		bufferRISB1[0] = 0x4C;
		bufferRISB1[1] = ConfigRISB[1].antEnable;
		bufferRISB2[0] = 0x4C;
		bufferRISB2[1] = ConfigRISB[4].antEnable;

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

		HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

		HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
		HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

		//This part Configure 8 antenna (P20 to P27 on each MAX7301)
		bufferRISA1[0] = 0x54;
		bufferRISA1[1] = ConfigRISA[2].antEnable;
		bufferRISA2[0] = 0x54;
		bufferRISA2[1] = ConfigRISA[5].antEnable;

		bufferRISB1[0] = 0x54;
		bufferRISB1[1] = ConfigRISB[2].antEnable;
		bufferRISB2[0] = 0x54;
		bufferRISB2[1] = ConfigRISB[5].antEnable;

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);

		HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA2, 2, 10);
		HAL_SPI_Transmit(&hspi1, (uint8_t *)&bufferRISA1, 2, 10);

		HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB2, 2, 10);
		HAL_SPI_Transmit(&hspi2, (uint8_t *)&bufferRISB1, 2, 10);

		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);

		uint8_t data[]= "Update completed\n";
		HAL_UART_Transmit(&huart3, data, sizeof(data),500);
    //osDelay(5000);
  }
  /* USER CODE END StartUpdate */
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
