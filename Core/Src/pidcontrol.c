//
// Created by 神奇bug在哪里 on 3/20/23.
//

#include "pidcontrol.h"

void PID_init(PID *pid, float kp, float ki, float kd, float setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
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
    //Q: 这里的微分是怎么计算的？
    //A: 这里的微分是误差的微分，误差的微分就是当前误差减去上一次误差
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->derivative; //输出
    pid->last_error = pid->error; //更新误差
    return pid->output;
}


void slideFilteringInit(RollingFilter *indata, float *pBuf, int bufSize) {
    indata->pBuf = pBuf;
    indata->bufSize = bufSize;
    indata->index = 0;
    indata->sum = 0;
}

float slideFilteringCalculate(RollingFilter *indata, float inData) {
    //滑动平均滤波
    //滑动平均滤波器的原理是将一组数据按照一定的顺序排列，然后取中间的数据作为滤波后的数据。
    //滑动平均滤波器的优点是简单，缺点是对于突变的数据不敏感。
    //这里采用的队列滑动平均滤波器，队列长度为5
    indata->sum -= indata->pBuf[indata->index];
    indata->pBuf[indata->index] = inData;
    indata->sum += indata->pBuf[indata->index];
    indata->index++;
    if (indata->index >= indata->bufSize) {
        indata->index = 0;
    }
    return indata->sum / (float)indata->bufSize;

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

// Path: Core/Src/main.c


