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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "delay_timer.h"
#include "dht11.h"
#include "lcd_i2c.h"
#include "hrf05.h"
#include "Task_Scheduler.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
typedef enum
{
	LCD_DISTANCE,
	LCD_TEM,
	LCD_HUMI,
	LCD_FULL
} LCD_MODE;
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_RX_LEN				15
#define DISPLAY_COMMAND_LEN 	13
#define TIME_COMMAND_LEN 		10
#define FRAME_TICK_LEN 	300 //ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
DHT11_Sensor dht11;

LCD_I2C_Name lcd;

SRF05_Device_Name hrf05;
/* USER CODE END PV */
sTaskTCB* TASK_DHT;
sTaskTCB* TASK_DIST;
sTaskTCB* TASK_LCD;
sTaskTCB* TASK_INTERRUPT_HANDLER;

float dhtTem;
float dhtHumi;
float distance;

uint8_t dem;
DHT11_Status dhtStatus;
char temp[20],h[20];
uint8_t rxData[20]; /* Data receive buffer */
uint8_t rxDataId = 0;
volatile uint8_t IsInterrup = 0;
LCD_MODE lcdMode = LCD_FULL;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int a)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&a, 1, HAL_MAX_DELAY);
	return a;
}

/* USER CODE END 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart2.Instance)
	{
		if (rxData[rxDataId] == '\n')
		{
			rxData[rxDataId] = '\0';
			IsInterrup = 1;
			rxDataId = 0;
		}
		else
		{
			rxDataId ++;
		}
	}
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData[rxDataId], 1);
}

void TASK_UART_Handler(){
	if (IsInterrup){
		char *command = strtok((char*)rxData, " ");
		char *time = strtok(NULL, " ");
		printf("UART Handler IN: %ld\r\n", uwTick);
		NVIC_DisableIRQ(USART2_IRQn);

		if (NULL == time){
			if (strcmp((const char*)rxData, "lcdtemp") == 0)
			{
				lcdMode = LCD_TEM;
				printf("Change LCD Mode to LCD_TEM\r\n\n");
			}
			else if (strcmp((const char*)rxData, "lcdhumi") == 0)
			{
				lcdMode = LCD_HUMI;
				printf("Change LCD Mode to LCD_HUMI\r\n\n");
			}
			else if (strcmp((const char*)rxData, "lcddist") == 0)
			{
				lcdMode = LCD_DISTANCE;
				printf("Change LCD Mode to DISPLAY_DIST\r\n\n");
			}
			else if (strcmp((const char*)rxData, "lcdboth") == 0)
			{
				lcdMode = LCD_FULL;
				printf("Change LCD Mode to LCD_FULL\r\n\n");
			}
			else
			{
				printf("Error Command Syntax\r\n\n");
			}
		}
		else {
			uint32_t pTime = atoi(time);
			if (strcmp((const char*)command, "timedht1") == 0)
			{
				TASK_DHT->Period = pTime/FRAME_TICK_LEN;
				printf("Change period of temp time to %ld\r\n", pTime);
			}
			else if (strcmp((const char*)command, "timedist") == 0)
			{
				TASK_DIST->Period = pTime/FRAME_TICK_LEN;
				printf("Change period of dist time to %ld\r\n", pTime);
			}
			else
			{
				printf("Error Command Syntax\r\n\n");
			}
			TASK_LCD->Period = TASK_DHT->Period + TASK_DIST->Period;

		}
		NVIC_EnableIRQ(USART2_IRQn);
		printf("UART Handler OUT: %ld\r\n", uwTick);
	}
	IsInterrup = 0;

}
void TASK_Dht_Sense(){
	printf("TASK_DHT IN: %ld\r\n", uwTick);
	dhtStatus = DHT11_GetData(&dht11);
	dhtTem = dht11.Temp;
	dhtHumi = dht11.Humi;
	printf("TASK_DHT OUT: %ld\r\n", uwTick);
}

void TASK_Dist_Sense(){
	printf("TASK_DIST IN: %ld\r\n", uwTick);
	SRF05_Read(&hrf05);
	distance = hrf05.Distance;
	printf("TASK_DIST OUT: %ld\r\n", uwTick);
}


void TASK_lcd_Measurement(){
	printf("TASK_LCD IN: %ld\r\n", uwTick);
	LCD_Clear(&lcd);
	switch (lcdMode){

	case LCD_TEM:
		sprintf(temp, "T: %.2f", dhtTem);
		LCD_SetCursor(&lcd, 0, 0);
		LCD_WriteString(&lcd, temp);
		printf("Tem: %.2f\r\n", dhtTem);
		break;
	case LCD_HUMI:
		sprintf(temp, "H: %.2f", dhtHumi);
		LCD_SetCursor(&lcd, 0, 0);
		LCD_WriteString(&lcd, temp);
		printf("Humi: %.2f\r\n", dhtHumi); // @suppress("Float formatting support")
		break;
	case LCD_DISTANCE:
		sprintf(temp, "D: %.2f", distance);
		LCD_SetCursor(&lcd, 0, 0);
		LCD_WriteString(&lcd, temp);
		printf("Distance: %.2f\r\n", distance);
		break;
	case LCD_FULL:
		sprintf(temp, "T: %.2f D: %.2f", dhtTem, distance);
		LCD_SetCursor(&lcd, 0, 0);
		LCD_WriteString(&lcd, temp);
		sprintf(h, "H: %.2f", dhtHumi);
		LCD_SetCursor(&lcd, 0, 1);
		LCD_WriteString(&lcd, h);
		printf("Distance: %.2f\r\n", distance);
		printf("Humidity: %.2f\r\n", dhtHumi);
		printf("Temperature: %.2f\r\n", dhtTem);
		break;
	}
	printf("TASK_LCD OUT: %ld\r\n", uwTick);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1){
		SCH_Update();
	}
}
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
	LCD_Init(&lcd, &hi2c2, LDC_DEFAULT_ADDRESS, 20, 4);
	DHT11_Init(&dht11, DHT_GPIO_Port, DHT_Pin, &htim4);
	SRF05_Init(&hrf05, ECHO_GPIO_Port, ECHO_Pin, TRIG_GPIO_Port, TRIG_Pin);
	HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxData[rxDataId], 1);
	printf("run\r\n");
		hSCH_Init();
  /* Infinite loop */
		TASK_DHT= SCH_Add_Task((TASK_SCH *)TASK_Dht_Sense,0,1000/FRAME_TICK_LEN,1,1);

			TASK_DIST= SCH_Add_Task((TASK_SCH *)TASK_Dist_Sense,0,1000/FRAME_TICK_LEN,1,1);

			TASK_LCD= SCH_Add_Task((TASK_SCH *)TASK_lcd_Measurement,0,1000/FRAME_TICK_LEN,1,1);

			TASK_INTERRUPT_HANDLER= SCH_Add_Task((TASK_SCH *)TASK_UART_Handler,0,1000/FRAME_TICK_LEN,1,1);

			SCH_Start(&htim1);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  hSCH_Dispatch_Tasks();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 893;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim4.Init.Prescaler = 83;
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

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin DHT_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
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
