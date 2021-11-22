/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "math.h"
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t str[256];
int n;
uint32_t input_delta;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  n = sprintf((char*) str, "Project G - Ultrasound by Chris Abajian (cxa6282@rit.edu)\r\n");
  HAL_UART_Transmit(&huart2, str, n, 100);
  n = sprintf((char*) str, "Performing POST...");
  HAL_UART_Transmit(&huart2, str, n, 100);
  int res = ultrasonic_post();
//  while (!res) {}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// Capture a single measurement.
	n = sprintf((char*) str, "\r\nCapturing 1 measurement...\r\n");
	HAL_UART_Transmit(&huart2, str, n, 100);
	int dist = ultrasound_capture();
	if (dist > 0) {
	  n = sprintf((char*) str, "Distance: %d\r\n", dist);
	} else {
      n = sprintf((char*) str, "Distance: ***\r\n");
	}
	HAL_UART_Transmit(&huart2, str, n, 100);

	// Wait to continue.
	n = sprintf((char*) str, "Press any key to continue...\r\n");
	HAL_UART_Transmit(&huart2, str, n, 100);
	HAL_UART_Receive(&huart2, str, 1, 1000000);

	// Capture 100 measurements.
    n = sprintf((char*) str, "Capturing 100 measurements...\r\n");
    HAL_UART_Transmit(&huart2, str, n, 100);
    int i = 0;
    int measurements[100] = {0};
    while (i < 100) {
      int dist = ultrasound_capture();
      if (huart2.Instance->ISR & USART_ISR_RXNE) {
    	// Keystroke pressed, discard this sample and exit.
    	n = sprintf((char*) str, "Keyboard interrupt received\r\n");
		HAL_UART_Transmit(&huart2, str, n, 100);
    	break;
      } else {
    	// Save this datapoint if it's valid.
    	if (dist > 0) {
    	  measurements[i] = dist;
    	}
      }
      i++;
    }

    // Display the datapoints and calculate statistics.
	n = sprintf((char*) str, "Displaying results:\r\n");
	HAL_UART_Transmit(&huart2, str, n, 100);
	int min = 0;
	int max = 0;
	int mean = 0;
	int median = (measurements[i/2-1] + measurements[i/2])/2;
	for (int j = 0; j < i; j++) {
	  int val = measurements[j];
	  n = sprintf((char*) str, "%d,%d\r\n", j+1, val);
	  HAL_UART_Transmit(&huart2, str, n, 100);
	  min = (val < min ? val : min);
	  max = (val > max ? val : max);
	  mean += val;
	}
	mean /= i;
	float stddev = 0;
	for (int j = 0; j < i; j++) {
	  int val = measurements[j];
	  stddev += (val - mean) * (val - mean);
	}
	stddev = sqrt(stddev/i);

    // Display the statistics.
	n = sprintf((char*) str, "Maximum,%d\r\n", max);
	HAL_UART_Transmit(&huart2, str, n, 100);
	n = sprintf((char*) str, "Minimum,%d\r\n", min);
	HAL_UART_Transmit(&huart2, str, n, 100);
	n = sprintf((char*) str, "Mean,%d\r\n", mean);
	HAL_UART_Transmit(&huart2, str, n, 100);
	n = sprintf((char*) str, "Median,%d\r\n", median);
	HAL_UART_Transmit(&huart2, str, n, 100);
	n = sprintf((char*) str, "Stddev,%0.2f\r\n", stddev);
	HAL_UART_Transmit(&huart2, str, n, 100);

	// Wait to continue.
	n = sprintf((char*) str, "Press any key to continue...\r\n");
	HAL_UART_Transmit(&huart2, str, n, 100);
	HAL_UART_Receive(&huart2, str, 1, 1000000);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_INACTIVE;
  sConfigOC.Pulse = 9;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  // NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
  // Clear interrupts.
  htim2.Instance->SR = 0;
  // Enable interrupts.
  htim2.Instance->DIER |= TIM_DIER_CC2IE;
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
int ultrasonic_post() {
	while (1) {
		ultrasound_capture();
		// Check delta and confirm pass/fail.
		if ((input_delta >= 300) && (input_delta <= 1000)) {
			// Success.
			n = sprintf((char*) str, "success\r\n");
			HAL_UART_Transmit(&huart2, str, n, 100);
			return 1;
		} else {
			// Fail.
			n = sprintf((char*) str, "fail. ");
			HAL_UART_Transmit(&huart2, str, n, 100);
			if (input_delta < 300) {
				n = sprintf((char*) str, "Object too close. Retry? (y/n): ");
			} else {
				n = sprintf((char*) str, "Object too far. Retry? (y/n): ");
			}
			HAL_UART_Transmit(&huart2, str, n, 100);
			uint8_t c;
			HAL_UART_Receive(&huart2, &c, 1, 1000000);
			n = sprintf((char*) str, "%c\r\n", c);
			HAL_UART_Transmit(&huart2, str, n, 100);
			if ((c == 'n') || (c == 'N')) {
				return 0;
			}
		}
	}
}

int ultrasound_capture() {
	// Clear input delta.
	input_delta = 0;
	// Start the timer.
	htim2.Instance->CNT = 0;
	htim2.Instance->SR = 0;
	HAL_TIM_Base_Start(&htim2);
	// Wait 100ms.
	while (htim2.Instance->CNT < 100000) {}
	// Stop the timer.
	HAL_TIM_Base_Stop(&htim2);
	// Return the distance if in-bounds.
	int dist = (float)input_delta * 0.346f;
    if (dist == 0) {
    	// Timeout.
    	return 0;
    } else if ((dist < 50) || (dist > 1000)) {
    	// Out-of-bounds.
    	return -1;
    } else {
    	// Valid.
    	return dist;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
