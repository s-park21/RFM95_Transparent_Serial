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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
UART_HandleTypeDef* USB_UART = &huart1;
SPI_HandleTypeDef* RF_SPI = &hspi1;

// Variable to store number of available bytes to read
uint8_t RF_available_bytes = 0;
// RF RX buffer
uint8_t RF_RX_Buff[256];
uint8_t UART_Buff[12];
bool UART_READY = false;

uint8_t USB_Buff[256];
bool USB_READY = false;
uint8_t USB_rx_data_len = 0; 	// Number of bytes read by usb

// Header string identifier used to verify that received packet is correct
char header_string[5] = "FM3DR";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // UART interrupt init
  HAL_UART_Receive_IT (USB_UART, UART_Buff, sizeof(UART_Buff));

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
//	  RF_available_bytes = LoRa_received_bytes(&LoRaClass);

//	  uint8_t outputArr[10];
//	  memset(outputArr, '\0', sizeof(outputArr));
//	  LoRa_readReg(&LoRaClass, RegVersion, sizeof(RegVersion), outputArr, 4);
//	  CDC_Transmit_HS(outputArr, sizeof(outputArr));

	  if(RF_available_bytes) {
		  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
		  // Bytes in RF RX buffer to read
		  // Read bytes into buffer
		  LoRa_receive(&LoRaClass, RF_RX_Buff, RF_available_bytes);
		  // Check packet identifier
		  if(!memcmp(RF_RX_Buff, (uint8_t*)header_string, 5)) {
			  // Header byte is correct
			  // Transmit bytes from RF_RX_Buff over UART not including packet identifier
			  HAL_UART_Transmit(USB_UART, &RF_RX_Buff[sizeof(header_string)], RF_available_bytes-sizeof(header_string), 1000);
			  // Transmit bytes from RF_RX Buff over USB not including packet identifier
			  CDC_Transmit_HS(&RF_RX_Buff[sizeof(header_string)], RF_available_bytes-sizeof(header_string));
		  }

		  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
		  RF_available_bytes = 0;
	  }

	  if(UART_READY) {
		  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
		  UART_READY = false;
		  uint8_t sendBuff[256+5];
		  // Add packet identifier
		  memcpy(&sendBuff, (uint8_t*)header_string, sizeof(header_string));
		  memcpy(&sendBuff[sizeof(header_string)], &UART_Buff, sizeof(UART_Buff));
		  // Transmit UART buffer over RF
		  if (!LoRa_transmit(&LoRaClass, sendBuff, (uint8_t)(sizeof(header_string)+sizeof(UART_Buff)), 1000)) {
			  // Print error msg
		  }
		  LoRa_startReceiving(&LoRaClass);
		  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
	  }

	  if(USB_READY) {
		  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_SET);
		  uint8_t sendBuff[256+5];
		  // Add packet identifier
		  memcpy(&sendBuff, (uint8_t*)header_string, sizeof(header_string));
		  memcpy(&sendBuff[sizeof(header_string)], &USB_Buff, USB_rx_data_len);
		  // Transmit USB buffer over RF
		  if (!LoRa_transmit(&LoRaClass, sendBuff, USB_rx_data_len+sizeof(header_string), 1000)) {
		  			  // Print error msg
		  }
		  USB_READY = false;
		  USB_rx_data_len = 0;
		  LoRa_startReceiving(&LoRaClass);
		  HAL_GPIO_WritePin(INDICATOR_LED_GPIO_Port, INDICATOR_LED_Pin, GPIO_PIN_RESET);
	  }

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
  huart1.Init.BaudRate = 115200;
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
  huart2.Init.BaudRate = 115200;
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
    HAL_UART_Receive_IT(USB_UART, UART_Buff, sizeof(UART_Buff));
    UART_READY = true;
}

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
