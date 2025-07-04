#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"
typedef struct {
    float kp;          /* 比例系数 */
    float ki;          /* 积分系数 */
    float kd;          /* 微分系数 */
    float integral;    /* 积分累加 */
    float prev_err;    /* 上一次偏差，用于微分 */
    float out_min;     /* 输出下限 */
    float out_max;     /* 输出上限 */
} PIDController;

/**
 * @brief  初始化 PID 控制器
 * @param  pid       控制器实例
 * @param  kp,ki,kd  三个系数
 * @param  out_min   输出最小值（反向饱和）
 * @param  out_max   输出最大值
 */
static inline void PID_Init(PIDController *pid,
                            float kp, float ki, float kd,
                            float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
}

/**
 * @brief  执行一次 PID 计算
 * @param  pid         控制器实例
 * @param  setpoint    目标值
 * @param  measurement 实际测量值
 * @param  dt          两次调用间隔（s）
 * @return 控制输出
 */
static inline float PID_Update(PIDController *pid,
                               float setpoint,
                               float measurement,
                               float dt)
{
    /* 1) 计算偏差 */
    float err = setpoint - measurement;

    /* 2) 积分累加并限幅（防止积分风暴） */
    pid->integral += err * dt;
    if (pid->integral * pid->ki > pid->out_max) {
        pid->integral = pid->out_max / pid->ki;
    } else if (pid->integral * pid->ki < pid->out_min) {
        pid->integral = pid->out_min / pid->ki;
    }

    /* 3) 微分 */
    float derivative = (err - pid->prev_err) / dt;

    /* 4) PID 公式 */
    float output = pid->kp * err
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    /* 5) 输出限幅 */
    if (output > pid->out_max) {
        output = pid->out_max;
    } else if (output < pid->out_min) {
        output = pid->out_min;
    }

    /* 6) 保存偏差，供下次微分使用 */
    pid->prev_err = err;

    return output;
}

#endif /* __PID_H */
