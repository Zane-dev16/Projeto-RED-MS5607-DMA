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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MS5607.h"
#include "LSM6DSO32.h"
#include <string.h>
#include <stdio.h>
#include "MS5607_DMA.h"
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

/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
#define MS5607_ADDR 0x76 << 1

void XferCpltCallback(DMA_HandleTypeDef *hdma);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  struct ms5607_dev ms5607_sensor;
  ms5607_dev_init(&ms5607_sensor);
  if(ms5607_init(&ms5607_sensor)==0) {
	  while(1) {
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  HAL_Delay(100);
	  }
  }
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);
  LSM6DSO32_Init();

  float p;
  float t;
  int prev_time;
  int i = 0;
  int j = 0;
  int start_flag = 1;
  uint8_t read_adc[1] = {0x00};
  uint8_t ms_d2_convert[1] = {0x44};


  prev_time = HAL_GetTick();

  hdma_usart2_tx.XferCpltCallback=&XferCpltCallback;
  char p_str[50];

  huart2.Instance->CR3 |= USART_CR3_DMAT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	i++;
	if (i == 20 || start_flag == 1) {

		if (start_flag == 0) {
		  printf(" Frequencia: %d, t: %f \r\n", 20000/(HAL_GetTick() - prev_time), t);
		  prev_time = HAL_GetTick();
		}

		ms5607_dma_prep_temp();
		HAL_Delay(2);
		ms5607_dma_request_data();
		HAL_Delay(1);
		ms5607_dma_read_temp(&ms5607_sensor);
		HAL_Delay(1);

		i = 0;
		start_flag = 0;
	}
	// preparar pressão
	HAL_I2C_Master_Transmit_DMA(ms5607_sensor.i2c_bus, MS5607_ADDR, ms_d2_convert, 1);

	sprintf(p_str, "p: %f \r\n", p);
	while (hdma_usart2_tx.State != HAL_DMA_STATE_READY);
	HAL_DMA_Start_IT(&hdma_usart2_tx, (uint32_t)p_str, (uint32_t)&huart2.Instance->TDR, 19);

	// escrever dados ao terminal


	while (HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	while (HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);
	HAL_Delay(2);

	// pedir dados
    HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5607_ADDR, read_adc, 1);

    // ler dados
	while (HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);
	HAL_Delay(0.0000001);
    uint8_t raw_pressure[3];
    HAL_I2C_Master_Receive_DMA(&hi2c1, MS5607_ADDR, raw_pressure, 3);

    // converter pressão
	while (HAL_DMA_GetState(&hdma_i2c1_rx) != HAL_DMA_STATE_READY);
	ms5607_sensor.D1 = (uint32_t)(raw_pressure[0] << 16) | (uint32_t)(raw_pressure[1] << 8) | (uint32_t)raw_pressure[2];
	ms5607_convert(&ms5607_sensor, &p, &t);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void XferCpltCallback(DMA_HandleTypeDef *hdma) {
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_13) {
    HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
  } else {
      __NOP();
  }
}

void ms5607_dma_request_data()
{
    uint8_t read_adc[1] = {0x00};
    HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5607_ADDR, read_adc, 1);
    while (HAL_DMA_GetState(&hdma_i2c1_tx) != HAL_DMA_STATE_READY);
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
