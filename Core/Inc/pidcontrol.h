//
// Created by 神奇bug在哪里 on 3/20/23.
//

#ifndef SMART_PIDCONTROL_H
#define SMART_PIDCONTROL_H
//PID调参
#define KP 1
#define KI 0
#define KD 0.5
#define A 1
#define B 0
#define C 0
//PID控制器
typedef struct {
    float kp;
    float ki;
    float kd;
    float setpoint;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;
} PID;
//PID初始化
void PID_init(PID *pid, float kp, float ki, float kd, float setpoint);
//PID计算
float PID_calculate(PID *pid, float input);
//滚动滤波算法
typedef struct
{
    float *pBuf; // 滤波缓冲区
    int bufSize; // 缓冲区大小
    int index; // 缓冲区索引
    float sum; // 缓冲区和
} RollingFilter;
//卡尔曼滤波算法
/**
  ******************************************************************************
  * @brief  卡尔曼滤波器 函数
  * @param  inData - 输入值
  * @return 滤波后的值
  * @note   r值固定，q值越大，代表越信任测量值，q值无穷大，代表只用测量值。
  *                  q值越小，代表越信任模型预测值，q值为0，代表只用模型预测值。
  *         q:过程噪声，q增大，动态响应变快，收敛稳定性变坏；反之。控制误差
  *         r:测量噪声，r增大，动态响应变慢，收敛稳定性变好；反之。控制响应速度
  ******************************************************************************
  */
float KalmanFilter(float inData);
void slideFilteringInit(RollingFilter *indata, float *pBuf, int bufSize);
float slideFilteringCalculate(RollingFilter *indata, float inData);
#endif //SMART_PIDCONTROL_H
