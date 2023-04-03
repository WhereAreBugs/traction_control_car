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
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "relocated.h" //重定向printf到串口
#include "servo.h"
#include "queue.h"
#include "stringProcess.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#define RXBUFFERSIZE 32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float adc_value[5] = {0}; // 保存ADC采样值
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
_Bool initFlag = 0;
_Bool crossFlag = 0;
_Bool roundaboutFlag = 0;
_Bool forkFlag = 0;
short stopFlag = 0;
char rxBuffer;
char RxBuffer[RXBUFFERSIZE];

float middleValue[10] = {0};
int Uart1_Rx_Cnt = 0;
int middleValueCount = 0;
Queue middleValueQueue;
float middleResult = 0;
PID piddata;
float speed = 0;

//过滤后的数据
float dat0 = 0;
float dat1 = 0;
float dat2 = 0;
float dat3 = 0;
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
    RollingFilter filter4;
    slideFilteringInit(&filter4, middleValue, 10);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    RetargetInit(&huart1);
    HAL_UART_Receive_IT(&huart1, (uint8_t *) &rxBuffer, 1);
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
    HAL_ADCEx_Calibration_Start(&hadc1); // 启动ADC校准
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_value, 5); // 启动ADC采样
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 启动PWM输出
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500); // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_Base_Start_IT(&htim4); // 启动定时器4中断
    HAL_UART_Receive_IT(&huart1, (uint8_t *) &rxBuffer, 1);

    //初始化PID
    PID_init(&piddata, KP, KI, KD, A, B, C, 0);
    // 位置式PID
    // 数据1，2是左侧，3，4是右侧
    QueueInit(&middleValueQueue);
    //WARNING: 以下代码将会禁用全局的速度
    speed = 100;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    while (1) {
        //处理采集回来的adc数据
        //其中adc_value[0]和adc_value[1]是左侧的数据，adc_value[2]和adc_value[3]是右侧的数据
        //采用滚动均值滤波算法进行处理,并计算左右两侧的差值
        if (initFlag) {
            servo_control(+50);
            HAL_Delay(500);
            servo_control(0);
            initFlag = 0;
        }
        dat0 = slideFilteringCalculate(&filter0, adc_value[0]);
        dat1 = slideFilteringCalculate(&filter1, adc_value[1]);
        dat2 = slideFilteringCalculate(&filter2, adc_value[2]);
        dat3 = slideFilteringCalculate(&filter3, adc_value[3]);
        middleResult = slideFilteringCalculate(&filter4, adc_value[4]);
        //计算PID
        float result = PID_calculate(&piddata, (piddata.a * (dat0 - dat3) + piddata.b * (dat1 - dat2)) /
                                               (piddata.a * (dat0 + dat3) + fabsf(piddata.c * (dat1 - dat2))));
        //输出PWM
        servo_control(result);
        speed_control(speed);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        //Q: 位置式PID和增量式PID的区别？
        //A: 位置式PID是根据当前位置和目标位置计算出需要的增量，然后再加上当前位置，得到目标位置
        //   增量式PID是根据当前位置和目标位置计算出需要的增量，然后直接加上当前位置，得到目标位置
        //   位置式PID的优点是可以减少积分误差，缺点是增量式PID的响应速度更快
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
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        //采用队列的方式保存中间传感器经过过滤后的值
        QueuePush(&middleValueQueue, middleResult);
        middleValueCount++;
        if (middleValueCount >= 5) {
            QueuePop(&middleValueQueue);
            middleValueCount -= 1;
        }
        if (QueueBack(&middleValueQueue) <= 10)
            //处在十字路口
        {
            if (crossFlag == 0) {
                crossFlag = 1;
                //执行进入十字路口的处理
                HAL_Delay(10);
            } else {
                crossFlag = 0;
                //执行离开十字路口的处理
                HAL_Delay(10);
            }
        }
        if (QueueBack(&middleValueQueue) <= 2 * QueueGet(&middleValueQueue, middleValueCount - 1) + 10 &&
            QueueBack(&middleValueQueue) >= 2 * QueueGet(&middleValueQueue, middleValueCount - 1) - 10)
            //处在环岛区域
        {
            if (roundaboutFlag == 0) {
                roundaboutFlag = 1;
                servo_control(+30.0f);
            } else {
                roundaboutFlag = 0;
                //执行离开环岛区域的处理
            }
        }
        //进入分叉路口
        //这边的方差的值正确吗？
        if (QueueVariance(&middleValueQueue) >= 100) {
            if (forkFlag == 0) {
                forkFlag = 1;
                //执行进入分叉路口1的处理
                servo_control(+30.0f);
                HAL_Delay(10);
            } else {
                forkFlag = 0;
                //执行进入分叉路口2的处理
                servo_control(-30.0f);
                HAL_Delay(10);
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BEED_Pin) {
        stopFlag++;
        if (stopFlag >= 2) {
            // 停车
            servo_control(30.0f);
            HAL_Delay(10);
            servo_control(-20.0f);
            HAL_Delay(10);
            servo_control(0.0f);
            speed = 0;
        }
    }

}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err34-c"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (Uart1_Rx_Cnt >= RXBUFFERSIZE)  //溢出判断?
    {
        UNUSED(huart);
        Uart1_Rx_Cnt = 0;
        memset(RxBuffer, 0x00, sizeof(RxBuffer));
        HAL_UART_Transmit(&huart1, (uint8_t *) "数据溢出", 12, 0xFFFF);

    } else {


        RxBuffer[Uart1_Rx_Cnt++] = rxBuffer;   //接收数据转存
//        printf("开始处理数据");
//        HAL_UART_Transmit(&huart1, (uint8_t *) "开始处理数据", 12, 0xFFFF);
        if (RxBuffer[Uart1_Rx_Cnt-1] == '#') //判断结束位
        {
            if (strstr(RxBuffer, "stop") != NULL) {
                //执行停车操作
                servo_control(30.0f);
                HAL_Delay(10);
                servo_control(-20.0f);
                HAL_Delay(10);
                servo_control(0.0f);
                speed = 0;


            } else if (strstr(RxBuffer, "setKP") != NULL) { // NOLINT(bugprone-branch-clone)
                //修改PID参数-比例系数
                if (!getSet(RxBuffer, 5, &piddata.kd)) {
//                    printf("数据错误\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误\n", 9, 0xFFFF);
                } else {
//                    printf("数据正确\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据正确\n", 9, 0xFFFF);
                }
            } else if (strstr(RxBuffer, "setKI") != NULL) {
                //修改PID参数-积分系数
                if (!getSet(RxBuffer, 5, &piddata.ki)) {
//                    printf("数据错误\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误\n", 9, 0xFFFF);
                } else {
//                    printf("数据正确\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据正确\n", 9, 0xFFFF);
                }
            } else if (strstr(RxBuffer, "setKD") != NULL) {
                //修改PID参数-微分系数
                if (!getSet(RxBuffer, 5, &piddata.kd)) {
//                    printf("数据错误\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误\n", 9, 0xFFFF);
                } else {
//                    printf("数据正确\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据正确\n", 9, 0xFFFF);
                }

            } else if (strstr(RxBuffer, "setA") != NULL) {
                if (!getSet(RxBuffer, 4, &piddata.a)) {
//                    printf("数据错误\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误\n", 9, 0xFFFF);
                } else {

                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据正确\n", 9, 0xFFFF);
                }
            } else if (strstr(RxBuffer, "setB") != NULL) {
                if (!getSet(RxBuffer, 4, &piddata.b)) {
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误\n", 9, 0xFFFF);
                } else {
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据正确\n", 9, 0xFFFF);
                }

            } else if (strstr(RxBuffer, "setC") != NULL) {
                if (!getSet(RxBuffer, 4, &piddata.c)) {
//                    if(sscanf("setC%f",RxBuffer,&piddata.c)!=1){
//                    printf("数据错误\n");
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误\n", 9, 0xFFFF);
                } else {
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据正确\n", 9, 0xFFFF);
                }

            } else if (strstr(RxBuffer, "setSpeed") != NULL) {
                //修改速度
                if (!getSet(RxBuffer, 4, &speed)) {
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误\n", 9, 0xFFFF);
                } else {
                    HAL_UART_Transmit(&huart1, (uint8_t *) "数据正确\n", 9, 0xFFFF);
                }
            } else if (strstr(RxBuffer, "start") != NULL) {
                // 开始运行

            } else if (strstr(RxBuffer, "what") != NULL) {
                //返回当前的参数
                printf("当前参数为:\r\n");
                printf("速度:%f\r\n", speed);
                printf("a:%f\r\n", piddata.a);
                printf("b:%f\r\n", piddata.b);
                printf("c:%f\r\n", piddata.c);
                printf("kp:%f\r\n", piddata.kp);
                printf("ki:%f\r\n", piddata.ki);
                printf("kd:%f\r\n", piddata.kd);

            } else if (strstr(RxBuffer, "help") != NULL) {
                //显示帮助信息
                HAL_UART_Transmit(&huart1, (uint8_t *) "帮助信息:\r\n", 16, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "stop:停止\r\n", 13, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "start:开始\r\n", 15, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "setSpeed:设置速度\r\n", 24, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "setA:设置a\r\n", 15, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "setB:设置b\r\n", 15, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "setC:设置c\r\n", 15, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "setKP:设置kp\r\n", 17, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "setKI:设置ki\r\n", 17, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "setKD:设置kd\r\n", 17, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "what:返回当前参数\r\n", 26, 0xFFFF);
                HAL_UART_Transmit(&huart1, (uint8_t *) "help:显示帮助信息\r\n", 26, 0xFFFF);
            }
            else if (strstr(RxBuffer, "data") != NULL) {
                // 获取当前传感器数据
                printf("当前传感器数据为:\r\n");
                printf("左侧传感器1:%f\r\n", dat0);
                printf("左侧传感器2:%f\r\n", dat1);
                printf("右侧传感器1:%f\r\n", dat2);
                printf("右侧传感器2:%f\r\n", dat3);
                printf("中间传感器:%f\r\n", middleResult);
            }
            else {
                //数据错误
                HAL_UART_Transmit(&huart1, (uint8_t *) "数据错误", 12, 0xFFFF);
            }
            Uart1_Rx_Cnt = 0;
            memset(RxBuffer, 0x00, sizeof(RxBuffer)); //清空数组
        }
    }
    HAL_UART_Receive_IT(&huart1, (uint8_t *) &rxBuffer, 1);   //再开启接收中断
}

#pragma clang diagnostic pop

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
