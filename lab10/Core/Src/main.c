/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t SIZE_ONE_SECTOR = 0x4000;		// 16KB
uint32_t ADDR_FLASH_SECTOR = 0x0800C000; 	// address of 3rd sector
uint32_t val_000 = 0x01234567;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void flash_erase_one_sector(uint8_t flash_sector);
void flash_write_uint32(uint32_t, uint32_t);
uint32_t flash_read_uint32(uint32_t);
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
  HAL_Delay(2000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  flash_erase_one_sector(FLASH_SECTOR_3);
  flash_write_uint32(ADDR_FLASH_SECTOR, val_000);
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void flash_erase_one_sector(uint8_t flash_sector)
{
	FLASH_EraseInitTypeDef EraseInitStruct; // структура для очищення флеша

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.Sector = flash_sector;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

	uint32_t sector_error = 0; // змінна в яку запишеться адреса сторінки, при невдалому очищенні

	char str[64] = { 0, };

	//////// ОЧИЩЕННЯ Flash-пам'яті ////////
	HAL_FLASH_Unlock(); // розблокування Flash-пам'яті

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &sector_error) != HAL_OK)
	{
		// сталась помилка при очищенні сторінки
		uint32_t er = HAL_FLASH_GetError();
		snprintf(str, 64, "Error Flash_Erase(): %lu\n\r", er);
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 100);
		while (1)
		{
		}
	}

	// очищення сторінки виконане успішно
	snprintf(str, 64, "Erase Flash sector %d OK\n\r", flash_sector);
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 100);

	HAL_FLASH_Lock(); // заблокування Flash-пам'яті
}

void flash_write_uint32(uint32_t sector_address, uint32_t val)
{
	char str[64] = { 0, };

	///////////// ЗАПИС у Flash-пам'ять ///////////////
	HAL_FLASH_Unlock(); // розблокування Flash-пам'яті

	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, sector_address, val) != HAL_OK)
	{
		uint32_t er = HAL_FLASH_GetError();
		snprintf(str, 64, "Error Flash_Write(): %lu\n\r", er);
		HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 100);
		while (1)
		{
		}
	}

	snprintf(str, 64, "Write 32 bits (Hex: 0x%08lX) to address 0x%08lX OK\n\r", val, sector_address);
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 100);

	HAL_FLASH_Lock(); // заблокування Flash-пам'яті
}

uint32_t flash_read_uint32(uint32_t MYADDR)
{
	char str[64] = { 0, };

	/////////////// ЧИТАЄМО Flash-пам'ять ///////////////////
	uint32_t dig32 = *(uint32_t*) MYADDR; // читання числа за його адресою

	snprintf(str, 64, "Read 32 bits (Dec: %lu \tHex: 0x%08lX)", dig32, dig32);
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 100);
	snprintf(str, 64, " from address 0x%08lX \n\r", MYADDR);
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), 100);

	return dig32;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
