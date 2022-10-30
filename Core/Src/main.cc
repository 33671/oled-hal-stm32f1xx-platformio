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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <ArduinoJson.h>
extern "C"
{
#include <u8g2.h>
#include "stm32_u8g2.h"
#include "oled_test.h"
#include <Utils.h>
}
using namespace std;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void ButtonDect();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char wrongMessage[] = "error";
StaticJsonDocument<200> doc;
char buffer[200];
char buffer_timer[200];
RTC_TimeTypeDef sTime;
u8g2_t u8g2;
uint16_t get_adc()
{
  // start ADC1
  HAL_ADC_Start(&hadc1);
  // wait ADC finishment，timeout:100ms
  HAL_ADC_PollForConversion(&hadc1, 100);
  // check conversion state
  if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
  {
    return HAL_ADC_GetValue(&hadc1);
  }
  return 0;
}
int time = 0;
int temp_time = 0;
bool falled = false;
bool started = false;
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
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_TIM_Base_Start(&htim1); // enable us delay timer
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)buffer, 10);

  u8g2Init(&u8g2);
  HAL_Delay(1000);
  uint16_t pwmVal = 0; // PWM占空�??

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  u8g2_FirstPage(&u8g2);
  while (1)
  {
    while (pwmVal < 500)
    {
      pwmVal++;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal); //修改比较值，修改占空�??
                                                           //		  TIM3->CCR1 = pwmVal;    与上方相�??
      HAL_Delay(2);
    }
    while (pwmVal)
    {
      pwmVal--;
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal); //修改比较值，修改占空�??
                                                           //		  TIM3->CCR1 = pwmVal;     与上方相�??
      HAL_Delay(2);
    }
    HAL_Delay(1000);
    if (!started && temp_time == 0 && time >= 5000)
    {
      testDrawXBM(&u8g2);
    }
    // int a = get_adc();
    // // HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    // // // auto str = fmt::format("Time:{}", sTime.Seconds);
    // // auto str = "Time:" + to_string(sTime.Hours) + ":" + to_string(sTime.Seconds);
    // auto c = (char *)(to_string(a) + "\n").c_str();
    // // drawStr(&u8g2, (char *)to_string(a).c_str());
    // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)c, strlen(c));
    // HAL_Delay(1000);
    // do
    // {

    //   u8g2DrawTest(&u8g2);
    // } while (u8g2_NextPage(&u8g2));
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_Delay(2500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  HAL_Delay(20);
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET)
  {
    falled = true;
  }
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET && falled)
  {
    doc.clear();
    if (!started)
    {
      started = true;
      time = 0;
      doc["time"] = time;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
      doc["message"] = "Start";
      serializeJsonPretty(doc, buffer_timer, 200);
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer_timer, strlen(buffer_timer));
      drawStr(&u8g2, (char *)"Recording...");
    }
    else
    {
      temp_time = time;
      doc["message"] = "End";
      doc["time"] = temp_time;
      serializeJsonPretty(doc, buffer_timer, 200);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer_timer, strlen(buffer_timer));
      auto str = "Duration:" + to_string(temp_time) + "ms";
      drawStr(&u8g2, (char *)str.c_str());
      time = 0;
      temp_time = 0;
      started = false;
    }
    falled = false;
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // HAL_UART_Transmit(&huart1, (uint8_t *)wrongMessage, strlen(wrongMessage), 0xFFFF);
  if (htim->Instance == TIM2)
  {
    if (time >= 99999)
      time = 0;
    time++;
    if (started && time % 200 == 0)
    {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
    }
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  drawStr(&u8g2, buffer);
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buffer, strlen(buffer));
  HAL_UART_Receive_IT(&huart1, (uint8_t *)buffer, 10);
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
    HAL_UART_Transmit(&huart1, (uint8_t *)wrongMessage, strlen(wrongMessage), 0xFFFF);
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
