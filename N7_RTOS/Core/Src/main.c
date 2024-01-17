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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "dht11.h"
#include "delay_timer.h"
#include "lcd_i2c.h"
#include "hrf05.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef enum
{
	LCD_DISTANCE,
	LCD_TEM,
	LCD_HUMI,
	LCD_FULL
} LCD_Mode;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_RX_LEN 				15
#define DISPLAY_COMMAND_LEN 	13
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DHT11 */
osThreadId_t DHT11Handle;
const osThreadAttr_t DHT11_attributes = {
  .name = "DHT11",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for DIST */
osThreadId_t DISTHandle;
const osThreadAttr_t DIST_attributes = {
  .name = "DIST",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for HandleInterrupt */
osThreadId_t HandleInterruptHandle;
const osThreadAttr_t HandleInterrupt_attributes = {
  .name = "HandleInterrupt",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for LCD */
osThreadId_t LCDHandle;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for DataSem */
osSemaphoreId_t DataSemHandle;
const osSemaphoreAttr_t DataSem_attributes = {
  .name = "DataSem"
};
/* Definitions for IRQSem */
osSemaphoreId_t IRQSemHandle;
const osSemaphoreAttr_t IRQSem_attributes = {
  .name = "IRQSem"
};
/* USER CODE BEGIN PV */
typedef struct
{
	float humi;
	float dist;
	float temp;
} senseData_t;
DHT11_Sensor dht11;
LCD_I2C_Name lcd;
SRF05_Device_Name hrf05;

/* Buffers and flags */
uint8_t count;
DHT11_Status dhtStatus;
uint8_t rxData[20]; /* Data receive buffer */
uint8_t rxDataId = 0;
char temp[20];
LCD_Mode LCDMode = LCD_FULL;
uint32_t dhtInterval = 1000;
uint32_t distInterval = 1000;
/* USER CODE END PV */
senseData_t senseData;
osStatus_t status;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void Task_DHT(void *argument);
void Task_Dist(void *argument);
void Task_InterruptHandler(void *argument);
void Task_LCD(void *argument);

/* USER CODE BEGIN PFP */
int __io_putchar(int a)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&a, 1, HAL_MAX_DELAY);
	return a;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		if (rxData[rxDataId] == '\n')
		{
			rxData[rxDataId] = '\0';
			printf("\nCommand: %s\r\n", rxData);
			rxDataId = 0;
			if (osThreadGetPriority(HandleInterruptHandle) != osPriorityHigh)
			{
				osThreadSetPriority(HandleInterruptHandle, osPriorityHigh);
			}
			osSemaphoreRelease(IRQSemHandle);
		}
		else
		{
			rxDataId ++;
		}
	}
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData[rxDataId], 1);
}
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
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	printf("run\r\n\n");
	DHT11_Init(&dht11, DHT_GPIO_Port, DHT_Pin, &htim4);
	LCD_Init(&lcd, &hi2c2, LDC_DEFAULT_ADDRESS, 20, 4);
	SRF05_Init(&hrf05, ECHO_GPIO_Port, ECHO_Pin, TRIG_GPIO_Port, TRIG_Pin);
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData[rxDataId], 1);
	LCD_SetCursor(&lcd, 0, 0);
	LCD_WriteString(&lcd, "Watting");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of DataSem */
  DataSemHandle = osSemaphoreNew(1, 0, &DataSem_attributes);

  /* creation of IRQSem */
  IRQSemHandle = osSemaphoreNew(1, 0, &IRQSem_attributes);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DHT11 */
  DHT11Handle = osThreadNew(Task_DHT, NULL, &DHT11_attributes);

  /* creation of DIST */
  DISTHandle = osThreadNew(Task_Dist, NULL, &DIST_attributes);

  /* creation of HandleInterrupt */
  HandleInterruptHandle = osThreadNew(Task_InterruptHandler, NULL, &HandleInterrupt_attributes);

  /* creation of LCD */
  LCDHandle = osThreadNew(Task_LCD, NULL, &LCD_attributes);

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  huart2.Init.BaudRate = 115200;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|DHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIG_Pin DHT_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

/* USER CODE BEGIN Header_Task_DHT */
/**
* @brief Function implementing the DHT11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_DHT */
void Task_DHT(void *argument)
{
	int32_t tick = osKernelGetTickCount();
  /* USER CODE BEGIN Task_DHT */
  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreGetCount(DataSemHandle) != 0)
	  		{
	  			osSemaphoreAcquire(DataSemHandle, osWaitForever);
	  		}
	  		tick = tick + dhtInterval;
	  		printf("TASK_DHT11 I: %ld\r\n", osKernelGetTickCount());
	  		SRF05_Read(&hrf05);
	  		senseData.dist = hrf05.Distance;
	  		printf("TASK_DHT11 O: %ld\r\n\n", osKernelGetTickCount());
	  		dhtStatus = DHT11_GetData(&dht11);
	  		switch(dhtStatus)
	  		{
	  		case DHT11_ERR_CHECKSUM:
	  			printf(" ERROR DHT11 CHECKSUM\r\n");
	  			break;
	  		case DHT11_ERR_RESPONSE:
	  			printf(" ERROR DHT11 RESPONSE\r\n");
	  			break;
	  		default:
	  			senseData.temp = dht11.Temp;
	  			senseData.humi = dht11.Humi;
	  			break;
	  		}

	  		osSemaphoreRelease(DataSemHandle);
	  		osDelayUntil(tick);

  }
  /* USER CODE END Task_DHT */
}

/* USER CODE BEGIN Header_Task_Dist */
/**
* @brief Function implementing the DIST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Dist */
void Task_Dist(void *argument)
{
	int32_t tk = osKernelGetTickCount();
  /* USER CODE BEGIN Task_Dist */
  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreGetCount(DataSemHandle) != 0)
	  		{
	  			osSemaphoreAcquire(DataSemHandle, osWaitForever);
	  		}
	  		tk = tk + distInterval;
	  		printf("TASK_DIST I: %ld\r\n", osKernelGetTickCount());
	  		SRF05_Read(&hrf05);
	  		senseData.dist = hrf05.Distance;
	  		printf("TASK_DIST O: %ld\r\n\n", osKernelGetTickCount());

	  		osSemaphoreRelease(DataSemHandle);

	  		osDelayUntil(tk);
  }
  /* USER CODE END Task_Dist */
}

/* USER CODE BEGIN Header_Task_InterruptHandler */
/**
* @brief Function implementing the HandleInterrupt thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_InterruptHandler */
void Task_InterruptHandler(void *argument)
{
  /* USER CODE BEGIN Task_InterruptHandler */
  /* Infinite loop */
  for(;;)
  {
	  /* Temporary pause others tasks */
	  		osSemaphoreAcquire(IRQSemHandle, osWaitForever);
	  		osThreadSuspend(DISTHandle);
	  		osThreadSuspend(DHT11Handle);
	  		osThreadSuspend(LCDHandle);
	  		/* Interrupt handler */
	  		char *command = strtok((char*)rxData, " ");
	  		char *time = strtok(NULL, " ");
	  		if (NULL == time){
	  			if (strcmp((const char*)rxData, "lcdtemp") == 0)
	  			{
	  				LCDMode = LCD_TEM;
	  				printf("Change Mode to LCD_TEM\r\n\n");
	  			}
	  			else if (strcmp((const char*)rxData, "lcdhumi") == 0)
	  			{
	  				LCDMode = LCD_HUMI;
	  				printf("Change Mode to LCD_HUMI\r\n\n");
	  			}
	  			else if (strcmp((const char*)rxData, "lcddist") == 0)
	  			{
	  				LCDMode = LCD_DISTANCE;
	  				printf("Change Mode to LCD_DIST\r\n\n");
	  			}
	  			else if (strcmp((const char*)rxData, "lcdboth") == 0)
	  			{
	  				LCDMode = LCD_FULL;
	  				printf("Change Mode to LCD_FUL\r\n\n");
	  			}
	  			else
	  			{
	  				printf("Error Syntax\r\n\n");
	  			}
	  		}
	  		else {
	  			uint32_t T = atoi(time);
	  			if (strcmp((const char*)command, "timedht11") == 0)
	  			{
	  				dhtInterval = T;
	  				printf("Change period of dht11 time to %ld\r\n", T);
	  			}
	  			else if (strcmp((const char*)command, "timedist") == 0)
	  			{
	  				distInterval = T;
	  				printf("Change period of dist time to %ld\r\n", T);
	  			}
	  			else
	  			{
	  				printf("Error Command Syntax\r\n\n");
	  			}
	  		}
	  		/* let continue others */
	  		osThreadResume(DISTHandle);
	  		osThreadResume(DHT11Handle);
	  		osThreadResume(LCDHandle);

  }
  /* USER CODE END Task_InterruptHandler */
}

/* USER CODE BEGIN Header_Task_LCD */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_LCD */
void Task_LCD(void *argument)
{
  /* USER CODE BEGIN Task_LCD */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(DataSemHandle, osWaitForever);
	  		printf("TASK_LCD IN: %ld\r\n", osKernelGetTickCount());

	  		if (dhtStatus == DHT11_OK)
	  		{
	  			LCD_Clear(&lcd);
	  			switch (LCDMode){
	  			case LCD_HUMI:
	  				sprintf(temp, "H: %.2f", senseData.humi);
	  				LCD_SetCursor(&lcd, 0, 0);
	  				LCD_WriteString(&lcd, temp);
	  				printf("Humi: %.2f\r\n", senseData.humi);
	  				break;
	  			case LCD_TEM:
	  				sprintf(temp, "T: %.2f", senseData.temp);
	  				LCD_SetCursor(&lcd, 0, 0);
	  				LCD_WriteString(&lcd, temp);
	  				printf("Tem: %.2f\r\n", senseData.temp);
	  				break;
	  			case LCD_DISTANCE:
	  				sprintf(temp, "D: %.2f", senseData.dist);
	  				LCD_SetCursor(&lcd, 0, 0);
	  				LCD_WriteString(&lcd, temp);
	  				printf("Dist: %.2f\r\n", senseData.dist);
	  				break;

	  			case LCD_FULL:
	  				sprintf(temp, "T: %.2f H: %.2f", senseData.temp, senseData.humi);
	  				LCD_SetCursor(&lcd, 0, 0);
	  				LCD_WriteString(&lcd, temp);
	  				printf("Temperature: %.2f\r\n", senseData.temp);
	  				printf("Humidity: %.2f\r\n", senseData.humi);
	  				sprintf(temp, "D: %.2f", senseData.dist);
	  				LCD_SetCursor(&lcd, 0, 1);
	  				LCD_WriteString(&lcd, temp);
	  				printf("Distance: %.2f\r\n", senseData.dist);
	  				break;
	  			}
	  		}
	  		printf("TASK_LCD O: %ld\r\n\n", osKernelGetTickCount());
  }
  /* USER CODE END Task_LCD */
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
