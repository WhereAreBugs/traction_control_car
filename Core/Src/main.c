/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pidcontrol.h"
#include "adrc.h"
#include <math.h>
#include <stdlib.h>
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
float adc_value[4] = {0}; // 保存ADC采样值
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
//Q: 请问adrc算法的数学原理是什么？

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    RollingFilter filter0;
    float *data0 = malloc(sizeof(float) * 10);
    slideFilteringInit(&filter0, data0, 10);
    RollingFilter filter1;
    float *data1 = malloc(sizeof(float) * 10);
    slideFilteringInit(&filter1, data1, 10);
    RollingFilter filter2;
    float *data2 = malloc(sizeof(float) * 10);
    slideFilteringInit(&filter2, data2, 10);
    RollingFilter filter3;
    float *data3 = malloc(sizeof(float) * 10);
    slideFilteringInit(&filter3, data3, 10);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    PID piddata; // PID数据
    PID_init(&piddata, KP, KI, KD, 0); // 初始化PID
    // 位置式PID

    //数据1，2是左侧，3，4是右侧
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1) {
        //处理采集回来的adc数据
        //其中adc_value[0]和adc_value[1]是左侧的数据，adc_value[2]和adc_value[3]是右侧的数据
        //采用滚动均值滤波算法进行处理,并计算左右两侧的差值
        float dat0 = slideFilteringCalculate(&filter0, adc_value[0]);
        float dat1 = slideFilteringCalculate(&filter1, adc_value[1]);
        float dat2 = slideFilteringCalculate(&filter2, adc_value[2]);
        float dat3 = slideFilteringCalculate(&filter3, adc_value[3]);
        //计算PID
        float result = PID_calculate(&piddata, (A*(dat0 - dat3) + B * (dat1 - dat2)) / (A * (dat0 + dat3) + fabsf(C * (dat1 - dat2))));
        //输出PWM
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, result);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        //Q: 位置式PID和增量式PID的区别？
        //A: 位置式PID是根据当前位置和目标位置计算出需要的增量，然后再加上当前位置，得到目标位置
        //   增量式PID是根据当前位置和目标位置计算出需要的增量，然后直接加上当前位置，得到目标位置
        //   位置式PID的优点是可以减少积分误差，缺点是增量式PID的响应速度更快
        //Q: 为什么要用Kalman滤波？
        //A: 因为ADC的采样值会有一定的噪声，而且ADC的采样值是不连续的，所以需要用滤波器来平滑ADC的采样值
        //Q: 平滑ADC的采样值有什么好处？
        //A: 1. 使得PID的输出更加平滑
        //   2. 使得电机的转速更加平滑
        //   3. 使得电机的转速更加稳定
        //Q: 为什么要用PID？
        //A: 因为PID可以使得电机的转速更加平滑，更加稳定
        //Q: 针对SG90舵机，应该采用哪种PWM值？
        //A: 500-2500，500是最小值，2500是最大值
        //Q: 在psc=72,arr=500,主频=72MHZ的情况下，PWM的周期是多少？
        //A: 1/144HZ=6.94ms
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
    while (1) {

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
