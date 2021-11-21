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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "teller_task.h"
#include "customer_task.h"
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
RNG_HandleTypeDef hrng;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for availTellersMutex */
osMutexId_t availTellersMutexHandle;
const osMutexAttr_t availTellersMutex_attributes = {
  .name = "availTellersMutex"
};
/* Definitions for customerQueueMutex */
osMutexId_t customerQueueMutexHandle;
const osMutexAttr_t customerQueueMutex_attributes = {
  .name = "customerQueueMutex"
};
/* USER CODE BEGIN PV */
uint8_t availTellers = 0x00;
int bank_closed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void status_task(void *params);
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
  MX_RNG_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of availTellersMutex */
  availTellersMutexHandle = osMutexNew(&availTellersMutex_attributes);

  /* creation of customerQueueMutex */
  customerQueueMutexHandle = osMutexNew(&customerQueueMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  // Create customer queue.

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Initialize tellers.
  teller_task_init(NUM_TELLERS);
  // Initialize customer queue.
  customer_queue_init();
  // Real-time status printout task.
  BaseType_t err = xTaskCreate(status_task, "StatusTask", 256, 0, 12, NULL);
  assert(err == pdPASS);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  // Enable channel interrupts.
  htim2.Instance->DIER |= (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);
  // NVIC
  NVIC_EnableIRQ(TIM2_IRQn);
  // Enable channel output compares.
  htim2.Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
  // Generate update.
  htim2.Instance->EGR |= TIM_EGR_UG;
  // Enable the timer.
  htim2.Instance->CR1 |= TIM_CR1_CEN;
  /* USER CODE END TIM2_Init 2 */

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

}

/* USER CODE BEGIN 4 */
/**
 * Generates a random number in the range [min,max].
 */
int random_dist(int min, int max) {
	uint32_t rnum = 0;
	HAL_RNG_GenerateRandomNumber(&hrng, &rnum);
	return min + (max-min)*(double)(rnum/4294967295.0);
}

/**
 * Prints out the status of the simulation time, tellers, and queue.
 */
static void status_task(void *params) {
	char buffer[128];
	int len = 0;
	len = sprintf(buffer, "BANK IS OPEN.");
	HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
	while(1) {
		// Get current 24 hour time.
		int sim_time_sec = REAL_US_TO_SIM_SEC(htim2.Instance->CNT);
		if (!bank_closed && (sim_time_sec > BANK_HOURS*60*60)) {
			// Bank is closed.
			bank_closed = 1;
			// Delete customer queue task so no more customers are queued.
			vTaskDelete(customer_queue.handle);
			// Print status.
			len = sprintf(buffer, "\r\nBANK IS CLOSED.");
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
		}
		if (bank_closed && customer_queue.length == 0 && availTellers == 0x07) {
			// Print final statistics.
			len = sprintf(buffer, "\r\nFINAL METRICS:");
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tTotal Customers Served: %d", customer_queue.num_served);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tCustomers Served by Teller: T1:%d, T2:%d, T3:%d", tellers[0].customers_served, tellers[1].customers_served, tellers[2].customers_served);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tAverage Time Customers Spent in Queue: %.2lfsec", (double)customer_queue.total_wait_time/(double)customer_queue.num_served);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tAverage Time Customers Spent with Tellers: T1:%.2lfsec, T2:%.2lfsec, T3:%.2lfsec", (double)tellers[0].total_transaction_time/(double)tellers[0].customers_served,
																															(double)tellers[1].total_transaction_time/(double)tellers[1].customers_served,
																															(double)tellers[2].total_transaction_time/(double)tellers[2].customers_served);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tAverage Time Tellers Waited for Customers: T1:%.2lfsec, T2:%.2lfsec, T3:%.2lfsec", (double)tellers[0].total_wait_time/(double)tellers[0].customers_served,
																															(double)tellers[1].total_wait_time/(double)tellers[1].customers_served,
																															(double)tellers[2].total_wait_time/(double)tellers[2].customers_served);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tMax Customer Wait Time in Queue: %dsec", customer_queue.max_wait_time);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tMax Teller Wait Time for Customer: T1:%dsec, T2:%dsec, T3:%dsec", tellers[0].max_wait_time, tellers[1].max_wait_time, tellers[2].max_wait_time);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tMax Teller Transaction Time: T1:%dsec, T2:%dsec, T3:%dsec", tellers[0].max_transaction_time, tellers[1].max_transaction_time, tellers[2].max_transaction_time);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tMax Customer Queue Depth: %d", customer_queue.max_length);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			HAL_Delay(10);
			len = sprintf(buffer, "\r\n\tFinal idle count: %ld", app_idle_cnt);
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			// Delete teller tasks.
			for (int i = 0; i < NUM_TELLERS; i++) {
				vTaskDelete(tellers[i].handle);
			}
			// Delete this task.
			vTaskDelete(NULL);
		} else {
			// Print timestamped status of customer queue and tellers.
			int hour = sim_time_sec/60/60;
			int min = sim_time_sec/60 - hour*60;
			int sec = sim_time_sec - hour*60*60 - min*60;
			hour += BANK_OPEN;
			len = sprintf(buffer, "\r\n%02d:%02d:%02d\tWaiting:%d", hour,min,sec, customer_queue.length);
			for (int i = 0; i < NUM_TELLERS; i++) {
				len += sprintf(buffer+len, "\t|T%d,Served:%d,%s|", tellers[i].instance+1, tellers[i].customers_served, AVAIL_BUSY(availTellers & (uint8_t)(0x01 << i)));
			}
			HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, ~0);
			vTaskDelay(100);
		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
