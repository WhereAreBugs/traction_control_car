//
// Created by 神奇bug在哪里 on 3/20/23.
//

#ifndef SMART_PIDCONTROL_H
#define SMART_PIDCONTROL_H
#define KP -27.00000
#define KI 0.0000010
#define KD 12.000000
//PID的参数
#define A 2.890000
#define B 2.360000
#define C 2.566000
//差比和公式的参数
#define Fuzzy_KP 1
#define Fuzzy_KI 0
#define Fuzzy_KD 0
//模糊PID的参数
//L1_adaptive
#define L1_KP 1
#define L1_KI 0
#define L1_KD 0
//L1自适应PID的参数
//PID结构体
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
} L1_adaptive_PID;


typedef struct {
    float kp;
    float ki;
    float kd;
    float a;
    float b;
    float c;
    float setpoint;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;
} PID;
//PID初始化
void PID_init(PID *pid, float kp, float ki, float kd, float a, float b, float c, float setpoint);
//PID计算
float PID_calculate(PID *pid, float input);
//滚动均值滤波算法
typedef struct {
    float *pBuf;
    unsigned char ucItemNum;
    unsigned char ucItemCnt;
    unsigned char ucItemIdx;
    float fSum;
} RollingMeanFilter;
//滚动均值滤波算法初始化
void RollingMeanFilter_init(RollingMeanFilter *filter, float *pBuf, unsigned char ucItemNum);
//滚动均值滤波算法计算
float RollingMeanFilter_calculate(RollingMeanFilter *filter, float input);




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
//滑动平均滤波算法

//模糊PID
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
} FuzzyPID;
//模糊PID初始化
void FuzzyPID_init(FuzzyPID *pid, float kp, float ki, float kd, float setpoint);
//模糊PID计算
float FuzzyPID_calculate(FuzzyPID *pid, float input);
//模糊PID调参
//Q: 模糊PID与PID的区别
//A: 模糊PID是PID的一种改进, 通过模糊控制, 使得PID的控制更加平滑, 更加稳定, 但是模糊PID的计算量更大, 速度更慢


#endif //SMART_PIDCONTROL_H
