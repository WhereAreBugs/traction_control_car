//
// Created by 神奇bug在哪里 on 3/20/23.
//

#include "pidcontrol.h"

void PID_init(PID *pid, float kp, float ki, float kd, float a, float b, float c, float setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->a = a;
    pid->b = b;
    pid->c = c;
    pid->setpoint = setpoint;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;

}

float PID_calculate(PID *pid, float input) {
    pid->error = pid->setpoint - input; //误差
    pid->integral += pid->error; //积分
    pid->derivative = pid->error - pid->last_error; //微分
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative; //输出
    pid->last_error = pid->error; //更新误差
    return pid->output;
}


void RollingMeanFilter_init(RollingMeanFilter *filter, float *pBuf, unsigned char ucItemNum)
{
    filter->pBuf = pBuf;
    filter->ucItemNum = ucItemNum;
    filter->ucItemCnt = 0;
    filter->ucItemIdx = 0;
    filter->fSum = 0;
}

float RollingMeanFilter_calculate(RollingMeanFilter *filter, float input)
{
    float fRet = 0;
    if (filter->ucItemCnt < filter->ucItemNum)
    {
        filter->ucItemCnt++;
    }
    filter->fSum -= filter->pBuf[filter->ucItemIdx];
    filter->pBuf[filter->ucItemIdx] = input;
    filter->fSum += input;
    filter->ucItemIdx++;
    if (filter->ucItemIdx >= filter->ucItemNum)
    {
        filter->ucItemIdx = 0;
    }
    fRet = filter->fSum / filter->ucItemCnt;
    return fRet;
}
float KalmanFilter(float inData) {
    float R = 0.01f; //测量噪声协方差
    float Q = 0.0001f; //过程噪声协方差
    float x_last = 0; //上一次的状态值
    float p_last = 0; //上一次的估计误差协方差
    float kg = 0; //卡尔曼增益
    float x_mid = 0; //估计的状态值
    float x_now = 0; //当前最优的状态值
    float p_mid = 0; //估计的误差协方差
    float p_now = 0; //当前最优的误差协方差
    x_mid = x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid = p_last + Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
    kg = p_mid / (p_mid + R); //kg为Kg(k),R为噪声
    x_now = x_mid + kg * (inData - x_mid); //估计出的最优值
    p_now = (1 - kg) * p_mid; //最优值对应的covariance
    x_last = x_now; //更新上一次的状态值
    p_last = p_now; //更新上一次的估计误差协方差
    return x_now; //返回最优值

}
//模糊控制
void FuzzyPID_init(FuzzyPID *pid, float kp, float ki, float kd, float setpoint)
{
    pid->kp = kp; //默认Kp
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}
float FuzzyPID_calculate(FuzzyPID *pid, float input)
{
    //计算模糊的Kp
    float Kp;
    if (pid->error > 0)
    {
        Kp = pid->kp * (1 - pid->error / pid->setpoint);
    }
    else
    {
        Kp = pid->kp * (1 + pid->error / pid->setpoint);
    }

    pid->error = pid->setpoint - input; //误差
    pid->integral += pid->error; //积分
    pid->derivative = pid->error - pid->last_error; //微分
    pid->output = Kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative; //输出
    pid->last_error = pid->error; //更新误差
    return pid->output;
}

void L1_adaptive_PID_init(L1_adaptive_PID *pid, float kp, float ki, float kd, float setpoint)
{
    pid->kp = kp; //默认Kp
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->error = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->derivative = 0;
    pid->output = 0;
}
float L1_adaptive_PID_calculate(L1_adaptive_PID *pid, float input)
{
    //计算模糊的Kp
    float Kp;
    if (pid->error > 0)
    {
        Kp = pid->kp * (1 - pid->error / pid->setpoint);
    }
    else
    {
        Kp = pid->kp * (1 + pid->error / pid->setpoint);
    }

    pid->error = pid->setpoint - input; //误差
    pid->integral += pid->error; //积分
    pid->derivative = pid->error - pid->last_error; //微分
    pid->output = Kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative; //输出
    pid->last_error = pid->error; //更新误差
    return pid->output;
}


