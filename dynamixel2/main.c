/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
#include "app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  DXL_DIAG_OK = 0,
  DXL_DIAG_TX_FAIL,
  DXL_DIAG_RX_TIMEOUT,
  DXL_DIAG_BAD_HEADER,
  DXL_DIAG_BAD_ID,
  DXL_DIAG_BAD_LENGTH,
  DXL_DIAG_BAD_CHECKSUM
} DXL_DiagStatus;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INST_PING             0x01
#define INST_READ             0x02
#define INST_WRITE            0x03

#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_MOVING_SPEED     32
#define ADDR_PRESENT_POS      36

#define TORQUE_ON             1
#define TORQUE_OFF            0

#define DXL_MIN_POS           0
#define DXL_MAX_POS           1023

#define MICRO_DELTA           8
#define TEST_SPEED            40
#define MOVE_WAIT_MS          500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DXL_TX_ENABLE()       HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_SET)
#define DXL_TX_DISABLE()      HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_RESET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static const uint8_t test_ids[] = {
  1
//  2, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17
};
static const uint8_t scan_ids[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t dxl_checksum(uint8_t id, uint8_t length, uint8_t instruction, uint8_t *params);
static uint8_t dxl_status_checksum(uint8_t *status, uint8_t packet_len);
static void dxl_flush_rx(void);
static HAL_StatusTypeDef dxl_send_packet(uint8_t id, uint8_t instruction, uint8_t *params, uint8_t param_len);
static HAL_StatusTypeDef dxl_receive_exact(uint8_t *buffer, uint8_t len, uint32_t timeout_ms);
static DXL_DiagStatus dxl_ping_diag(uint8_t id);
static DXL_DiagStatus dxl_scan_read_diag(uint8_t *found_id);
static HAL_StatusTypeDef dxl_write_1byte(uint8_t id, uint8_t address, uint8_t value);
static HAL_StatusTypeDef dxl_write_2byte(uint8_t id, uint8_t address, uint16_t value);
static HAL_StatusTypeDef dxl_read_2byte(uint8_t id, uint8_t address, uint16_t *out_value);
static uint16_t clamp_position(int32_t pos);
static void DXL_ShowDiagStatus(DXL_DiagStatus status);
static void dxl_micro_rotate_test_one(uint8_t id);
static void DXL_RunMicroRotationTest(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t dxl_checksum(uint8_t id, uint8_t length, uint8_t instruction, uint8_t *params)
{
  uint16_t sum = id + length + instruction;

  for (uint8_t i = 0; i < length - 2; i++) {
    sum += params[i];
  }

  return (uint8_t)(~sum);
}

static uint8_t dxl_status_checksum(uint8_t *status, uint8_t packet_len)
{
  uint16_t sum = 0;

  for (uint8_t i = 2; i < packet_len - 1; i++) {
    sum += status[i];
  }

  return (uint8_t)(~sum);
}

static void dxl_flush_rx(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart6);

  while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET) {
    (void)huart6.Instance->DR;
  }
}

static HAL_StatusTypeDef dxl_send_packet(uint8_t id, uint8_t instruction, uint8_t *params, uint8_t param_len)
{
  uint8_t packet[32];
  uint8_t length = param_len + 2;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = length;
  packet[4] = instruction;

  for (uint8_t i = 0; i < param_len; i++) {
    packet[5 + i] = params[i];
  }

  packet[5 + param_len] = dxl_checksum(id, length, instruction, params);

  DXL_TX_ENABLE();
  HAL_Delay(1);

  HAL_StatusTypeDef ret = HAL_UART_Transmit(&huart6, packet, 6 + param_len, 100);

  uint32_t tickstart = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET) {
    if ((HAL_GetTick() - tickstart) > 100) {
      DXL_TX_DISABLE();
      return HAL_TIMEOUT;
    }
  }

  DXL_TX_DISABLE();

  return ret;
}

static HAL_StatusTypeDef dxl_receive_exact(uint8_t *buffer, uint8_t len, uint32_t timeout_ms)
{
  for (uint8_t i = 0; i < len; i++) {
    if (HAL_UART_Receive(&huart6, &buffer[i], 1, timeout_ms) != HAL_OK) {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

static DXL_DiagStatus dxl_ping_diag(uint8_t id)
{
  uint8_t status[6];

  if (dxl_send_packet(id, INST_PING, NULL, 0) != HAL_OK) {
    return DXL_DIAG_TX_FAIL;
  }

  if (dxl_receive_exact(status, sizeof(status), 100) != HAL_OK) {
    return DXL_DIAG_RX_TIMEOUT;
  }

  if (status[0] != 0xFF || status[1] != 0xFF) {
    return DXL_DIAG_BAD_HEADER;
  }

  if (status[2] != id) {
    return DXL_DIAG_BAD_ID;
  }

  if (status[3] != 0x02) {
    return DXL_DIAG_BAD_LENGTH;
  }

  if (status[5] != dxl_status_checksum(status, sizeof(status))) {
    return DXL_DIAG_BAD_CHECKSUM;
  }

  return DXL_DIAG_OK;
}

static DXL_DiagStatus dxl_scan_read_diag(uint8_t *found_id)
{
  uint16_t present_pos;

  for (uint8_t i = 0; i < sizeof(scan_ids); i++) {
    if (dxl_read_2byte(scan_ids[i], ADDR_PRESENT_POS, &present_pos) == HAL_OK) {
      *found_id = scan_ids[i];
      return DXL_DIAG_OK;
    }
  }

  return DXL_DIAG_RX_TIMEOUT;
}

static HAL_StatusTypeDef dxl_write_1byte(uint8_t id, uint8_t address, uint8_t value)
{
  uint8_t params[2];

  params[0] = address;
  params[1] = value;

  return dxl_send_packet(id, INST_WRITE, params, 2);
}

static HAL_StatusTypeDef dxl_write_2byte(uint8_t id, uint8_t address, uint16_t value)
{
  uint8_t params[3];

  params[0] = address;
  params[1] = value & 0xFF;
  params[2] = (value >> 8) & 0xFF;

  return dxl_send_packet(id, INST_WRITE, params, 3);
}

static HAL_StatusTypeDef dxl_read_2byte(uint8_t id, uint8_t address, uint16_t *out_value)
{
  uint8_t params[2];
  uint8_t status[8];

  params[0] = address;
  params[1] = 2;

  if (dxl_send_packet(id, INST_READ, params, 2) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_UART_Receive(&huart6, status, sizeof(status), 100) != HAL_OK) {
    return HAL_ERROR;
  }

  if (status[0] != 0xFF || status[1] != 0xFF || status[2] != id) {
    return HAL_ERROR;
  }

  *out_value = status[5] | ((uint16_t)status[6] << 8);

  return HAL_OK;
}

static uint16_t clamp_position(int32_t pos)
{
  if (pos < DXL_MIN_POS) {
    return DXL_MIN_POS;
  }

  if (pos > DXL_MAX_POS) {
    return DXL_MAX_POS;
  }

  return (uint16_t)pos;
}

static void DXL_ShowDiagStatus(DXL_DiagStatus status)
{
  if (status == DXL_DIAG_OK) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(500);
  } else {
    uint8_t pulses = (uint8_t)status;

    for (uint8_t i = 0; i < pulses; i++) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(100);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(100);
    }

    HAL_Delay(700);
  }
}

static void dxl_micro_rotate_test_one(uint8_t id)
{
	dxl_write_1byte(id, ADDR_TORQUE_ENABLE, TORQUE_ON);
  HAL_Delay(100);

  dxl_write_2byte(id, ADDR_MOVING_SPEED, 100);
  HAL_Delay(100);

  dxl_write_2byte(id, ADDR_GOAL_POSITION, 300);
  HAL_Delay(2000);

  dxl_write_2byte(id, ADDR_GOAL_POSITION, 700);
  HAL_Delay(2000);

  dxl_write_2byte(id, ADDR_GOAL_POSITION, 300);
  HAL_Delay(2000);

//  dxl_write_1byte(id, ADDR_TORQUE_ENABLE, TORQUE_OFF);
//  HAL_Delay(100);
}

static void DXL_RunMicroRotationTest(void)
{
  for (uint8_t i = 0; i < sizeof(test_ids); i++) {
    dxl_micro_rotate_test_one(test_ids[i]);
    HAL_Delay(200);
  }
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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  App_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Loop();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(DXL_DIR_GPIO_Port, DXL_DIR_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DXL_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DXL_DIR_GPIO_Port, &GPIO_InitStruct);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    for (volatile uint32_t i = 0; i < 500000; i++) {
    }
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
