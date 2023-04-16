//
// Created by 神奇bug在哪里 on 3/23/23.
//

#include "servo.h"
#include "main.h"
#include "tim.h"
void servo_control(float angle)
/*
 * 舵机控制函数
 * 输入参数：角度
 * 正数为右转的角度，负数为左转的角度
 */
{

    if (angle > 20)
    {
        angle = 20;
    }
    else if (angle < -20)
    {
        angle = -20;
    }
    angle = 1850 + (angle-1) * (50/45.0f);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3, angle);
    __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4, angle);
}
void speed_control(float speed)
/*
 * 电机控制函数
 * 输入参数：速度
 * 正数为正转，负数为反转
 * 0为停止
 * 100为最大速度
 */
{

    if (speed > 0)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
    }else if (speed < 0)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
    }
    if (speed < 0)
    {
        speed = -speed;
    }
    if (speed > 100)
    {
        speed = 100;
    }
    speed = speed*20;
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1, speed);
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2, speed);
}