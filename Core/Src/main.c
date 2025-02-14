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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

//#define VOLTAGE_TEST 1
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  STATE_STOPPED,
  STATE_STARTING,
  STATE_STARTED
} SystemState;

#define RX_BUF_LEN  2048     //
typedef struct
{
    unsigned char rx_buf[RX_BUF_LEN];
    uint16_t data_length;
} rxStruct;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t aRxBuffer2, aRxBuffer3;			
rxStruct txS = {
        .rx_buf = {0}, 
        .data_length = 0 
};
rxStruct rxS2 = {
        .rx_buf = {0}, 
        .data_length = 0 
};

SystemState currentState = STATE_STOPPED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1); //
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1); //
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

#ifdef VOLTAGE_TEST
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // ??PA5??
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1000);
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#ifndef VOLTAGE_TEST
//TIM??????
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)//????
{
 
	if(htim->Instance == TIM1)//??????1??????
	{
    if (currentState == STATE_STARTING) {
      currentState = STATE_STARTED;
      HAL_UART_Transmit(&huart3, (uint8_t *)rxS2.rx_buf, rxS2.data_length,6);
      rxS2.data_length = 0;
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);//?????TIM?????????????
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);//?????TIM??
      __HAL_TIM_SET_COUNTER(&htim1, 10000-10000); // ????1s
      HAL_TIM_Base_Start_IT(&htim1);//??????????TIM??
    }
    else if (currentState == STATE_STARTED) {
      currentState = STATE_STOPPED;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // ??PA5??
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
  */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
  if(huart->Instance == USART2)
  {
    if ((currentState == STATE_STOPPED) || (currentState == STATE_STARTING)) {
      if ((rxS2.data_length >= RX_BUF_LEN-1)) //
      {
        rxS2.data_length = 0;
      }
      rxS2.rx_buf[rxS2.data_length++] = aRxBuffer2; //
    }
    
    if (currentState == STATE_STOPPED) {
      currentState = STATE_STARTING;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // ??PA5??
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);//?????TIM?????????????
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);//?????TIM??
      __HAL_TIM_SET_COUNTER(&htim1, 10000-50); // ????
      HAL_TIM_Base_Start_IT(&htim1);//??????????TIM??
    }
    else if (currentState == STATE_STARTED) {
      __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);//??TIM??
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);//??TIM??
      __HAL_TIM_SET_COUNTER(&htim1, 10000-10000); // ????
      HAL_TIM_Base_Start_IT(&htim1);//??????????TIM??
      HAL_UART_Transmit(&huart3, (uint8_t *)&aRxBuffer2, 1, 0); //
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer2, 1); //
  }
  if (huart->Instance == USART3)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)&aRxBuffer3, 1, 0); //
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer3, 1); //
  }
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
}

#endif
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
