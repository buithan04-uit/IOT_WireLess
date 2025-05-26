/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "fonts.h"
#include "testimg1.h"
#include "A.h"
#include "D.h"
#include "E.h"
#include "stm32f1xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "delay.h"
#include "gps.h"
#include "DHT.h"
#include "PMS.h"
#include "MAX30100_PulseOximeter.h"
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define UART_BUFFER_SIZE 2048
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void Send_AT_Commands(UART_HandleTypeDef *huart);
void Send_AT_Command(UART_HandleTypeDef *huart, const char *command, uint32_t timeout);
void Send_AT_Command1(UART_HandleTypeDef *huart, const char *command, uint32_t timeout);
void TestUart ();
void ReadDataSensorNew ();
void Max30100 ();
void TestGPS ();
void TestDHT22();
void TestUart1 ();
void onBeatDetected(void) {
    // Xử lý khi phát hiện nhịp tim (bật LED, gửi UART, ...)
	ILI9341_WriteString(0, 0, "Beat!" , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char data[1024];
float temperature = 1;
float humidity = 1;
//
UART_HandleTypeDef huart1;
PMS_HandleTypeDef pms;
PMS_DATA dataPM;

PulseOximeter pox;
uint32_t tsLastReport = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1) GPS_UART_CallBack();
}

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();
  ILI9341_FillScreen(ILI9341_BLACK);

//  MAX30100_Init(&hi2c1, &huart1);
//  MAX30100_SetSpO2SampleRate(MAX30100_SPO2SR_DEFAULT);
//  MAX30100_SetLEDPulseWidth(MAX30100_LEDPW_DEFAULT);
//  MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_DEFAULT, MAX30100_LEDCURRENT_DEFAULT);
//  MAX30100_SetMode(MAX30100_SPO2_MODE);

  GPS_Init();
  PMS_Init(&pms, &huart3);
  PMS_WakeUp(&pms);
  HAL_Delay(1000); // �?ợi cảm biến ổn định

  PulseOximeter_Init(&pox);
  PulseOximeter_SetOnBeatDetectedCallback(&pox, onBeatDetected);

  if (!PulseOximeter_Begin(&pox, PULSEOXIMETER_DEBUGGINGMODE_NONE)) {
	  ILI9341_WriteString(0, 0, "Initializing!!" , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
      while (1); // Lỗi khởi tạo, dừng lại
  } else {
	  ILI9341_WriteString(0, 0, "Init Successfull !!" , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
  }
  // Sau khi PulseOximeter_Begin(&pox, ...);
  // Sau khi PulseOximeter_Begin(&pox, ...);
  uint8_t spo2cfg = MAX30100_ReadRegister(MAX30100_REG_SPO2_CONFIGURATION);
  spo2cfg |= (1 << 6); // Bật EN_SPO2
  MAX30100_WriteRegister(MAX30100_REG_SPO2_CONFIGURATION, spo2cfg);

  // Đọc lại để kiểm tra
  spo2cfg = MAX30100_ReadRegister(MAX30100_REG_SPO2_CONFIGURATION);
  sprintf(data, "SPO2CFG:0x%02X", spo2cfg);
  ILI9341_WriteString(0, 15, data, Font_7x10, ILI9341_RED, ILI9341_BLACK);
  MAX30100_WriteRegister(MAX30100_REG_INTERRUPT_STATUS, 0x00);
  uint8_t mode = MAX30100_ReadRegister(MAX30100_REG_MODE_CONFIGURATION);
  char dbg[32];
  sprintf(dbg, "MODE:0x%02X", mode);
  ILI9341_WriteString(0, 30, dbg, Font_7x10, ILI9341_RED, ILI9341_BLACK);

  uint8_t status = MAX30100_ReadRegister(MAX30100_REG_INTERRUPT_STATUS);
  sprintf(dbg, "INT:0x%02X", status);
  ILI9341_WriteString(0, 45, dbg, Font_7x10, ILI9341_RED, ILI9341_BLACK);

  uint8_t part_id = MAX30100_GetPartId(&pox.hrm);
  sprintf(data, "PartID: 0x%02X", part_id);
  ILI9341_WriteString(0, 60, data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);

  HAL_Delay(5000);
  tsLastReport = HAL_GetTick();
  ILI9341_FillScreen(ILI9341_BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  TestGPS();
//	  HAL_Delay(2000);
//	  TestUart();
	  Max30100();
//	  ReadDataSensorNew();
//	  TestDHT22();
//	  HAL_Delay(7000);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TestUart1 (){
    Send_AT_Command(&huart4, "AT1234567123", 15000 );
}

void TestDHT22(){
		DHT_ReadData(&temperature, &humidity);
		sprintf(data, "Temp(C)= %.1f Humidity(%%)= %.1f", temperature, humidity);
		ILI9341_WriteString(0, 0 , data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
}
void TestGPS (){
	sprintf(data, "Lat : %3.6f", GPS.dec_latitude);
	ILI9341_WriteString(0, 0 , data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
	sprintf(data, "Long : %3.6f", GPS.dec_longitude);
	ILI9341_WriteString(0, 15, data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
}

float get_avg(uint16_t *data, int len) {
    float sum = 0;
    for (int i = 0; i < len; i++) sum += data[i];
    return sum / len;
}

float get_ac(uint16_t *data, int len) {
    uint16_t max_val = data[0], min_val = data[0];
    for (int i = 1; i < len; i++) {
        if (data[i] > max_val) max_val = data[i];
        if (data[i] < min_val) min_val = data[i];
    }
    return (float)(max_val - min_val);
}

float calculate_spo2(uint16_t *ir, uint16_t *red, int len) {
    float dc_ir = get_avg(ir, len);
    float dc_red = get_avg(red, len);
    float ac_ir = get_ac(ir, len);
    float ac_red = get_ac(red, len);

    float R = (ac_red / dc_red) / (ac_ir / dc_ir);
    float spo2 = 110 - 25 * R;

    if (spo2 > 100) spo2 = 100;
    if (spo2 < 0) spo2 = 0;

    return spo2;
}
void Max30100() {
    PulseOximeter_Update(&pox);
    // Định kỳ báo cáo nhịp tim và SpO2
    if (HAL_GetTick() - tsLastReport > 500) {
    	float heartRate = PulseOximeter_GetHeartRate(&pox);
    	uint8_t spo2 = PulseOximeter_GetSpO2(&pox);

        sprintf(data, "HR: %.1f bpm", heartRate);
        ILI9341_WriteString(0, 15, data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
        sprintf(data, "SpO2: %d %%", spo2);
        ILI9341_WriteString(0, 30, data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);

        tsLastReport = HAL_GetTick();
    }
    HAL_Delay(10);
}
void ReadDataSensorNew (){
    if (PMS_ReadUntil(&pms, &dataPM, 2000)) {
    	sprintf(data, "PM1.0: %u ug/m3", dataPM.PM_AE_UG_1_0);
    	ILI9341_WriteString(0, 0, data , Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
    	sprintf(data, "PM2.5: %u ug/m3", dataPM.PM_AE_UG_2_5);
    	ILI9341_WriteString(0, 20, data , Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
    	sprintf(data, "PM10.0: %u ug/m3", dataPM.PM_AE_UG_10_0);
    	ILI9341_WriteString(0, 40, data , Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
    } else {
        ILI9341_WriteString(0, 0, "Timeout" , Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
    }
}


void TestUart (){
	// GPS
	sprintf(data, "Lat : %3.6f", GPS.dec_latitude);
	ILI9341_WriteString(0, 0 , data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
	sprintf(data, "Long : %3.6f", GPS.dec_longitude);
	ILI9341_WriteString(0, 15, data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);

	//DHT22
	DHT_ReadData(&temperature, &humidity);
	sprintf(data, "Temp(C)= %.1f", temperature);
	ILI9341_WriteString(0, 30 , data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
	sprintf(data, "Humidity(%%)= %.1f", humidity);
	ILI9341_WriteString(0, 45 , data, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);

	//PM2.5
    if (PMS_ReadUntil(&pms, &dataPM, 2000)) {
    	sprintf(data, "PM1.0: %u ug/m3", dataPM.PM_AE_UG_1_0);
    	ILI9341_WriteString(0, 60, data , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    	sprintf(data, "PM2.5: %u ug/m3", dataPM.PM_AE_UG_2_5);
    	ILI9341_WriteString(0, 75, data , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    	sprintf(data, "PM10.0: %u ug/m3", dataPM.PM_AE_UG_10_0);
    	ILI9341_WriteString(0, 90, data , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    } else {
        ILI9341_WriteString(0, 60, "Timeout" , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    }

	//Send data

	Send_AT_Commands(&huart4);
}

void Send_AT_Commands(UART_HandleTypeDef *huart) {
	sprintf(data, "TEMP=%.1f;HUM=%.1f;PM1.0=%d;PM2.5=%d;PM10.0=%d;LAT=%.6f;LON=%.6f", temperature, humidity, dataPM.PM_AE_UG_1_0,dataPM.PM_AE_UG_2_5,dataPM.PM_AE_UG_10_0, GPS.dec_latitude, GPS.dec_longitude );
    Send_AT_Command(huart, data , 10000 );
}

void Send_AT_Command1(UART_HandleTypeDef *huart, const char *command, uint32_t timeout) {
    // Gửi lệnh qua UART

    HAL_UART_Transmit(huart, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

    // Ch�? phản hồi từ ESP
    uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
    memset(uart_rx_buffer, 0, UART_BUFFER_SIZE); // Xóa buffer
    HAL_UART_Receive(huart, uart_rx_buffer, UART_BUFFER_SIZE, timeout);
	ILI9341_WriteString(0, 130, data , Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
}
void Send_AT_Command(UART_HandleTypeDef *huart, const char *command, uint32_t timeout) {
    char full_command[128];
    snprintf(full_command, sizeof(full_command), "%s\n", command);

    ILI9341_WriteString(0, 110, full_command, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    HAL_UART_Transmit(huart, (uint8_t *)full_command, strlen(full_command), HAL_MAX_DELAY);
    HAL_Delay(10);
//    uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
//    memset(uart_rx_buffer, 0, UART_BUFFER_SIZE);
//
//    HAL_StatusTypeDef status = HAL_UART_Receive(huart, uart_rx_buffer, UART_BUFFER_SIZE - 1, timeout);
//
//    if (status == HAL_OK) {
//        uart_rx_buffer[UART_BUFFER_SIZE - 1] = '\0'; // �?ảm bảo kết thúc chuỗi
//        ILI9341_WriteString(0, 130, (char *)uart_rx_buffer, Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
//    } else {
//        ILI9341_WriteString(0, 130, "UART Timeout or Error", Font_7x10, ILI9341_RED, ILI9341_BLACK);
//    }
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
