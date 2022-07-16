/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "retarget.h"
#include "LoRa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// ************** Below is used to define if the device is a slave or master *******************
//#define SLAVE_DEVICE
#define MASTER_DEVICE
// *********************************************************************************************

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
UART_HandleTypeDef* USB_UART = &huart1;
SPI_HandleTypeDef* RF_SPI = &hspi1;
TIM_HandleTypeDef* Poll_Timer = &htim2;

#define HEADER_STRING_SIZE 4
#define MAX_RF_PACKET_SIZE 256
#define MAX_SERIAL_PACKET_SIZE(MAX_RF_PACKET_SIZE) (MAX_RF_PACKET_SIZE*2)
#define PREAMBLE_SIZE(HEADER_STRING_SIZE) (HEADER_STRING_SIZE+sizeof(uint16_t))
#define MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE) (MAX_RF_PACKET_SIZE -  PREAMBLE_SIZE(HEADER_STRING_SIZE))
#define MAX_UART_DATA_SIZE 240

// Variable to store number of available bytes to read
uint8_t RF_available_bytes = 0;
// RF RX buffer
uint8_t RF_RX_Buff[MAX_RF_PACKET_SIZE];
uint8_t RF_TX_Buff[MAX_RF_PACKET_SIZE];
//uint8_t UART_Buff[MAX_RF_PACKET_SIZE-PREAMBLE_SIZE(HEADER_STRING_SIZE)];
uint8_t UART_Buff[2*MAX_UART_DATA_SIZE];
uint8_t UART_PACKET_SIZE = sizeof(UART_Buff)/2;
uint8_t USB_Buff[MAX_RF_PACKET_SIZE-PREAMBLE_SIZE(HEADER_STRING_SIZE)];
uint8_t RF_transmit_buffer[2*MAX_RF_PACKET_SIZE];
uint16_t RF_transmit_buff_offset = 0;
bool UART_READY = false;
bool POLL_READY = false;
bool MASTER_READY = false;
uint8_t USB_rx_data_len = 0; 	// Number of bytes read by usb

// static void MX_DMA_Init(void); MUST BE INIT BEFORE static void MX_USART1_UART_Init(void);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);

static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LoRa LoRaClass;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  RetargetInit(USB_UART);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  //***************************** DMA MUST INIT BEFORE UART ***********************
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // Polling timer init
  HAL_TIM_Base_Start_IT(Poll_Timer);
  // UART interrupt init
  HAL_UART_Receive_DMA (USB_UART, UART_Buff, sizeof(UART_Buff));

  // LoRa Class definitions
  LoRaClass = newLoRa();
  LoRaClass.hSPIx                 = RF_SPI;
  LoRaClass.CS_port               = RF_SPI_NSS_GPIO_Port;
  LoRaClass.CS_pin                = RF_SPI_NSS_Pin;
  LoRaClass.reset_port            = RESET_RF_GPIO_Port;
  LoRaClass.reset_pin             = RESET_RF_Pin;
  LoRaClass.DIO0_port			  = IO0_RF_GPIO_Port;
  LoRaClass.DIO0_pin			  = IO0_RF_Pin;

  LoRaClass.frequency             = 915;
  LoRaClass.spredingFactor        = SF_7;						// default = SF_7
  LoRaClass.bandWidth			  = BW_125KHz;				  	// default = BW_125KHz
  LoRaClass.crcRate				  = CR_4_5;						// default = CR_4_5
  LoRaClass.power			      = POWER_20db;					// default = 20db
  LoRaClass.overCurrentProtection = 120; 						// default = 100 mA
  LoRaClass.preamble			  = 10;		  					// default = 8;

  HAL_GPIO_WritePin(RF_SPI_NSS_GPIO_Port, RF_SPI_NSS_Pin, GPIO_PIN_SET);

  LoRa_reset(&LoRaClass);
  uint32_t result = LoRa_init(&LoRaClass);

  if(result == LORA_NOT_FOUND) {
	  Blocking_LED_Blink(1);
  }
  else if(result == LORA_UNAVAILABLE) {
	  Blocking_LED_Blink(1);
  }

  // START CONTINUOUS RECEIVING -----------------------------------
  LoRa_startReceiving(&LoRaClass);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(RF_available_bytes) {
		  // Bytes in RF RX buffer to read
		  // Read bytes into buffer
		  LoRa_receive(&LoRaClass, RF_RX_Buff, RF_available_bytes);
		  // Buffer to contain data to be transmitted over UART and USB
		  uint8_t serialBuff[MAX_SERIAL_PACKET_SIZE(MAX_RF_PACKET_SIZE)];

		  // Check packet identifier
#ifdef MASTER_DEVICE
		  if(!memcmp(RF_RX_Buff, (uint8_t*)"SRSP", HEADER_STRING_SIZE)) {
			  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
			  // Header byte is packet type 2 (Slave response)
			  uint16_t data_len = (uint16_t)(RF_RX_Buff[HEADER_STRING_SIZE]<<8 | RF_RX_Buff[HEADER_STRING_SIZE+1]);

			  if(data_len > MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE)) {
				  RF_available_bytes = 0;
				  // Data spans over two packets
				  // Put all available bytes into serialBuff
				  memcpy(serialBuff,  &RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE));
				  memset(RF_RX_Buff, '\0', sizeof(RF_RX_Buff));
				  // Wait for next packet
				  while(memcmp(RF_RX_Buff, (uint8_t*)"SRSP", HEADER_STRING_SIZE) != 0) {
					  LoRa_receive(&LoRaClass, RF_RX_Buff, data_len - MAX_RF_PACKET_SIZE);
				  }
				  data_len = (uint16_t)(RF_RX_Buff[HEADER_STRING_SIZE]<<8 | RF_RX_Buff[HEADER_STRING_SIZE+1]);
				  // Add new data to end of previous packet
				  memcpy(&serialBuff[MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE)], &RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], data_len);
				  // Transmit concatenated buffer over USB and UART
				  CDC_Transmit_HS(serialBuff, MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE) + data_len);
				  HAL_UART_Transmit(USB_UART, serialBuff, MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE) + data_len, 1000);
				  RF_available_bytes = 0;
			  }
			  else {
				  // Data is all contained within one packet
				  // Transmit bytes from RF_RX_Buff over UART not including packet identifier or data length field
				  HAL_UART_Transmit(USB_UART, &RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], RF_available_bytes-PREAMBLE_SIZE(HEADER_STRING_SIZE), 1000);
				  // Transmit bytes from RF_RX Buff over USB not including packet identifier
				  CDC_Transmit_HS(&RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], RF_available_bytes-PREAMBLE_SIZE(HEADER_STRING_SIZE));
				  RF_available_bytes = 0;
			  }
			  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
		  }

#elif defined SLAVE_DEVICE
		  if(!memcmp(RF_RX_Buff, (uint8_t*)"MREQ", HEADER_STRING_SIZE)) {
			  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
			  MASTER_READY = true;
			  // Header byte is packet type 1 (Master request)
			  uint16_t data_len = (uint16_t)(RF_RX_Buff[HEADER_STRING_SIZE]<<8 | RF_RX_Buff[HEADER_STRING_SIZE+1]);
			  if(data_len > MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE)) {
				  RF_available_bytes = 0;
				  // Data spans over two packets
				  // Put all available bytes into serialBuff
				  memcpy(serialBuff,  &RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE));
				  memset(RF_RX_Buff, '\0', sizeof(RF_RX_Buff));
				  // Wait for next packet
				  while(memcmp(RF_RX_Buff, (uint8_t*)"SRSP", HEADER_STRING_SIZE) != 0) {
					  LoRa_receive(&LoRaClass, RF_RX_Buff, data_len - MAX_RF_PACKET_SIZE);
				  }
				  data_len = (uint16_t)(RF_RX_Buff[HEADER_STRING_SIZE]<<8 | RF_RX_Buff[HEADER_STRING_SIZE+1]);
				  // Add new data to end of previous packet
				  memcpy(&serialBuff[MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE)], &RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], data_len);
				  // Transmit concatenated buffer over USB and UART
				  CDC_Transmit_HS(serialBuff, MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE) + data_len);
				  HAL_UART_Transmit(USB_UART, serialBuff, MAX_DATA_SIZE(MAX_RF_PACKET_SIZE, HEADER_STRING_SIZE) + data_len, 1000);
				  RF_available_bytes = 0;
			  }
			  else if (data_len > 0) {
				  // Data is all contained within one packet
				  // Transmit bytes from RF_RX_Buff over UART not including packet identifier or data length field
				  HAL_UART_Transmit(USB_UART, &RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], RF_available_bytes-PREAMBLE_SIZE(HEADER_STRING_SIZE), 1000);
				  // Transmit bytes from RF_RX Buff over USB not including packet identifier
				  CDC_Transmit_HS(&RF_RX_Buff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], RF_available_bytes-PREAMBLE_SIZE(HEADER_STRING_SIZE));
				  RF_available_bytes = 0;
			  }
			  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
		  }
		  else RF_available_bytes = 0;
#endif
	  }

#ifdef MASTER_DEVICE
	  if (POLL_READY) {
		  if(UART_READY) {
			  UART_READY = false;
			  uint8_t sendBuff[MAX_RF_PACKET_SIZE];
			  // Add packet identifier
			  memcpy(&sendBuff, (uint8_t*)"MREQ", sizeof("MREQ"));
			  // Add data length field
			  memcpy(&sendBuff[sizeof("MREQ")], &UART_PACKET_SIZE, sizeof(uint16_t));
			  // Add data to sendBuff
			  memcpy(&sendBuff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], &RF_transmit_buffer, UART_PACKET_SIZE);
			  // Transmit USB buffer over RF
			  if (!LoRa_transmit(&LoRaClass, sendBuff, RF_transmit_buff_offset+PREAMBLE_SIZE(HEADER_STRING_SIZE), 1000)) {
						  // Print error msg
			  }
			  RF_transmit_buff_offset = 0;
			  LoRa_startReceiving(&LoRaClass);
		  }
		  if(!UART_READY && !USB_rx_data_len) {
			  uint8_t sendBuff[MAX_RF_PACKET_SIZE];
			  // Send empty data packet
			  // Add packet identifier
			  memcpy(&sendBuff, (uint8_t*)"MREQ\0\0", sizeof("MREQ\0\0"));
			  if (!LoRa_transmit(&LoRaClass, sendBuff, PREAMBLE_SIZE(HEADER_STRING_SIZE), 1000)) {
				  // Print error msg
			  }
		  }
	  }
	  POLL_READY = false;

#elif defined SLAVE_DEVICE
	  if(MASTER_READY) {
		  MASTER_READY = false;
		  if(UART_READY) {
			  UART_READY = false;
			  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
			  uint8_t sendBuff[MAX_RF_PACKET_SIZE];
			  // Add packet identifier
			  memcpy(&sendBuff, (uint8_t*)"SRSP", sizeof("SRSP"));
			  // Add data length field
			  memcpy(&sendBuff[sizeof("SRSP")], &UART_PACKET_SIZE, sizeof(uint16_t));
			  // Add data to sendBuff
			  memcpy(&sendBuff[PREAMBLE_SIZE(HEADER_STRING_SIZE)], &RF_transmit_buffer, UART_PACKET_SIZE);
			  // Transmit USB buffer over RF
			  if (!LoRa_transmit(&LoRaClass, sendBuff, UART_PACKET_SIZE+PREAMBLE_SIZE(HEADER_STRING_SIZE), 1000)) {
						  // Print error msg
			  }
			  RF_transmit_buff_offset = 0;
			  LoRa_startReceiving(&LoRaClass);
			  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
		  }
	  }
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  htim2.Init.Prescaler = 2500;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RF_SPI_NSS_GPIO_Port, RF_SPI_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_RF_GPIO_Port, RESET_RF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RF_SPI_NSS_Pin */
  GPIO_InitStruct.Pin = RF_SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_SPI_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_RF_Pin */
  GPIO_InitStruct.Pin = RESET_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_RF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IO0_RF_Pin IO1_RF_Pin IO2_RF_Pin IO3_RF_Pin
                           IO4_RF_Pin */
  GPIO_InitStruct.Pin = IO0_RF_Pin|IO1_RF_Pin|IO2_RF_Pin|IO3_RF_Pin
                          |IO4_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IO5_RF_Pin */
  GPIO_InitStruct.Pin = IO5_RF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IO5_RF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INDICATOR_LED_Pin */
  GPIO_InitStruct.Pin = INDICATOR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INDICATOR_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == LoRaClass.DIO0_pin){
		RF_available_bytes = LoRa_received_bytes(&LoRaClass);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_DMA (USB_UART, UART_Buff, sizeof(UART_Buff));
	UART_READY = true;
	// Add second half of data to sendBuff
	memcpy(RF_transmit_buffer, &UART_Buff[UART_PACKET_SIZE], UART_PACKET_SIZE);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	UART_READY = true;
	// Add first half of data to sendBuff
	memcpy(RF_transmit_buffer, &UART_Buff, UART_PACKET_SIZE);
}


#ifdef MASTER_DEVICE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if(htim == Poll_Timer) {
    	// Poll slave device
    	POLL_READY = true;
    }
}
#endif

void Blocking_LED_Blink(uint8_t freq) {
	while(1) {
		HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(1000/freq);
		HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(1000/freq);
	}
}

/* USER CODE END 4 */

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
