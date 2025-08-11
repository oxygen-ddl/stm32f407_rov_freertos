#include "move_control.h"
#include <math.h>
#include <stdint.h>
#include "jy901p_uart.h"
#include "ms5837_uart.h"
#include "transmit_power_board.h"
#include "move_drv.h"
#include "tim.h"
#include "math.h"

#define ABS(x) ((x) > 0 ? (x) : -(x)) // 绝对值宏
S_handle handle = {0};                // 手柄数据
// 平移
int16_t go_forward = 0; // 正数向前，负数向后
int16_t go_left = 0;    // 正数向左，负数向右
int16_t go_up = 0;      // 正数上升，负数下降
// 角度
int16_t move_yaw = 0;   // 正数向左，负数向右
int16_t move_pitch = 0; // 正数向上，负数向下
int16_t move_roll = 0;  // 正数顺时针，负数逆时针
// pid
pid_set pid_out_parameter[6];
pid_set pid_in_parameter[8]; // 内环电流控制参数，按设定推进器编号排布
pid_set depth_pid;
// 功能
S_mode mode = {0};
S_mode mode_last = {0}; // 上一次的模式数据

S_pid_roll pid_roll = {0};

uint8_t move_lock_flag = 0;                  // 运动停止标志位
float roll_target, pitch_target, yaw_target; // 目标值

static void rov_move_data_process(float rov_move_data[6])
{
    rov_move_data[0] = roll_total; // 旋转后方向
    rov_move_data[1] = pitch_total;
    rov_move_data[2] = yaw_total;
}

static void motor_output(void)
{
    // motor_set有限幅
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, pid_in_parameter[i].out_data);
    }
}
//! 0-roll 1-pitch 2-yaw 3-向前 4-左移 5-向上（正方向定义）
/**
 * 计算pid并输出
 */
void Pid_Out_Calculate(void)
{
    float rov_move_data[6];
    rov_move_data_process(rov_move_data); // 姿态传感器->pid实际值

    for (uint8_t i = 0; i < 3; ++i)
    {
        // 更新控制状态
        pid_out_parameter[i].actual_value = rov_move_data[i];
        // 计算目标值与当前值的差值
        pid_out_parameter[i].err = pid_out_parameter[i].target_value - pid_out_parameter[i].actual_value;
        // 进行积分运算
        pid_out_parameter[i].integral_value += pid_out_parameter[i].err;
        // 进行积分限幅
        if (pid_out_parameter[i].integral_value > pid_out_parameter[i].integral_value_limit)
        {
            pid_out_parameter[i].integral_value = pid_out_parameter[i].integral_value_limit;
        }
        else if (pid_out_parameter[i].integral_value < -pid_out_parameter[i].integral_value_limit)
        {
            pid_out_parameter[i].integral_value = -pid_out_parameter[i].integral_value_limit;
        }
        // 进行pid运算
        pid_out_parameter[i].out_data = pid_out_parameter[i].parameter.kp * pid_out_parameter[i].err + pid_out_parameter[i].parameter.ki * pid_out_parameter[i].integral_value + pid_out_parameter[i].parameter.kd * (pid_out_parameter[i].err - pid_out_parameter[i].err_last);

        // 对运算结果进行限幅
        if (pid_out_parameter[i].out_data > pid_out_parameter[i].out_data_limit)
        {
            pid_out_parameter[i].out_data = pid_out_parameter[i].out_data_limit;
        }
        else if (pid_out_parameter[i].out_data < -pid_out_parameter[i].out_data_limit)
        {
            pid_out_parameter[i].out_data = -pid_out_parameter[i].out_data_limit;
        }
        // 更新上次差值
        pid_out_parameter[i].err_last = pid_out_parameter[i].err;
    }

    // 对前进与左移进行控制
    for (uint8_t i = 0 + 3; i < 3 + 2; ++i)
    {
        // 进行pd运算
        pid_out_parameter[i].out_data = pid_out_parameter[i].parameter.kp * pid_out_parameter[i].err + pid_out_parameter[i].parameter.kd * (pid_out_parameter[i].err - pid_out_parameter[i].err_last);
        // 对运算结果进行限幅
        if (pid_out_parameter[i].out_data > pid_out_parameter[i].out_data_limit)
        {
            pid_out_parameter[i].out_data = pid_out_parameter[i].out_data_limit;
        }
        else if (pid_out_parameter[i].out_data < -pid_out_parameter[i].out_data_limit)
        {
            pid_out_parameter[i].out_data = -pid_out_parameter[i].out_data_limit;
        }
        // 更新上次差值
        pid_out_parameter[i].err_last = pid_out_parameter[i].err;
    }

    // 对深度（上升）进行控制
    pid_out_parameter[5].actual_value = ms5837_depth; // 深度传感器->pid实际值
    // 计算目标值与当前值的差值
    pid_out_parameter[5].err = (pid_out_parameter[5].target_value - pid_out_parameter[5].actual_value)*100;
    // 进行积分运算
    pid_out_parameter[5].integral_value += pid_out_parameter[5].err;
    // 进行积分限幅
    if (pid_out_parameter[5].integral_value > pid_out_parameter[5].integral_value_limit)
    {
        pid_out_parameter[5].integral_value = pid_out_parameter[5].integral_value_limit;
    }
    else if (pid_out_parameter[5].integral_value < -pid_out_parameter[5].integral_value_limit)
    {
        pid_out_parameter[5].integral_value = -pid_out_parameter[5].integral_value_limit;
    }
    // 进行pid运算
    pid_out_parameter[5].out_data = pid_out_parameter[5].parameter.kp * pid_out_parameter[5].err + pid_out_parameter[5].parameter.ki * pid_out_parameter[5].integral_value + pid_out_parameter[5].parameter.kd * (pid_out_parameter[5].err - pid_out_parameter[5].err_last);

    // 对运算结果进行限幅
    if (pid_out_parameter[5].out_data > pid_out_parameter[5].out_data_limit)
    {
        pid_out_parameter[5].out_data = pid_out_parameter[5].out_data_limit;
    }
    else if (pid_out_parameter[5].out_data < -pid_out_parameter[5].out_data_limit)
    {
        pid_out_parameter[5].out_data = -pid_out_parameter[5].out_data_limit;
    }
    // 更新上次差值
    pid_out_parameter[5].err_last = pid_out_parameter[5].err;

    pid_out_parameter[5].out_data = - (pid_out_parameter[5].out_data);

    static float a, b, c, d , e;
    a = 1;
    b = 1;
    c = 1;
    d = 1;
    e = 1;
    // 判断pitch(运动限幅)
    if ((roll > -90)&& (roll < 90))//
    {
        a = 1; // roll
        b = (ABS(cosf(roll))*0.5f + 0.5f); // pitch
        c = 1; // yaw
        d = ABS(cosf(roll))*0.5f+0.5f; // 深度
        e = 1; //向左
    }
    else if ((roll > 90 ) || (roll < -90))
    {
        a = 1; // roll
        b = -(ABS(cosf(roll))*0.5f+0.5f); // pitch
        c = -1; // yaw
        d = -(ABS(cosf(roll))*0.5f+0.5f); // 深度
        e = -1; //向左
    }
    else
    {
        a = 1; // roll
        b = 0; // pitch
        c = 0; // yaw
        d = 0; // 深度
        e = 0; //向左
    }
    if (pitch > 80 || pitch < -80)
    {
        a = 0; // roll
        b = 0; // pitch
        c = 0; // yaw
        d = 0; // 深度
        e = 0; //向左
    }
    

    pid_in_parameter[0].out_data = +a * pid_out_parameter[0].out_data + b * pid_out_parameter[1].out_data + d  *pid_out_parameter[5].out_data;
    pid_in_parameter[1].out_data = -a * pid_out_parameter[0].out_data + b * pid_out_parameter[1].out_data + d*pid_out_parameter[5].out_data;
    pid_in_parameter[2].out_data = +a * pid_out_parameter[0].out_data - b * pid_out_parameter[1].out_data + d*pid_out_parameter[5].out_data;
    pid_in_parameter[3].out_data = -a * pid_out_parameter[0].out_data - b * pid_out_parameter[1].out_data + d*pid_out_parameter[5].out_data;

    pid_in_parameter[4].out_data = -c * pid_out_parameter[2].out_data - pid_out_parameter[3].out_data - e*pid_out_parameter[4].out_data;
    pid_in_parameter[5].out_data = +c * pid_out_parameter[2].out_data - pid_out_parameter[3].out_data + e*pid_out_parameter[4].out_data;
    pid_in_parameter[6].out_data = +c * pid_out_parameter[2].out_data + pid_out_parameter[3].out_data - e*pid_out_parameter[4].out_data;
    pid_in_parameter[7].out_data = -c * pid_out_parameter[2].out_data + pid_out_parameter[3].out_data + e*pid_out_parameter[4].out_data;

    motor_output();
}

// 电流顺序与推进器编号对应
void current_convert(void)
{
    motor_current_actual[0] = adc_data_after_filter[8 - 1];
    motor_current_actual[1] = adc_data_after_filter[7 - 1];
    motor_current_actual[2] = adc_data_after_filter[2 - 1];
    motor_current_actual[3] = adc_data_after_filter[4 - 1];
    motor_current_actual[4] = adc_data_after_filter[9 - 1];
    motor_current_actual[5] = adc_data_after_filter[6 - 1];
    motor_current_actual[6] = adc_data_after_filter[3 - 1];
    motor_current_actual[7] = adc_data_after_filter[1 - 1];
}

// 解锁电机
void motor_init(void)
{
    // 推进器初始化
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, 0);
    }
    HAL_Delay(8000); // 等待8秒
    // 检查是否能动
    //  motor_set(1, 50); //
    //  HAL_Delay(5000);
    //  motor_set(1, 0);

    // motor_set(2, 50); //
    // HAL_Delay(5000);
    // motor_set(2, 0);

    // motor_set(3, 100); // 负
    // HAL_Delay(5000);
    // motor_set(3, 0);

    // motor_set(4, 100); // 负
    // HAL_Delay(5000);
    // motor_set(4, 0);

    // motor_set(5,100);//为正  逆时针
    // HAL_Delay(5000);
    // motor_set(5,0);

    // motor_set(6,100);//为负
    // HAL_Delay(5000);
    // motor_set(6,0);

    // motor_set(7,100);//为正
    // HAL_Delay(5000);
    // motor_set(7,0);

    // motor_set(8,100);//为正
    // HAL_Delay(5000);
    // motor_set(8,0);
    // for (uint8_t i = 0; i < 8; ++i)
    // {
    //     motor_set(i + 1, 80);
    // }
    // HAL_Delay(2000); // 等待2秒
    // for (uint8_t i = 0; i < 8; ++i)
    // {
    //     motor_set(i + 1, 0); // 停止
    // }
}

//! 对于rov的控制，控制频率几十到一百多就可以了

void Move_Control_Task_Init(void)
{
    // 设置输出限幅幅值，积分限幅幅值,变积分指数值误差分界值
    for (uint8_t i = 0; i < 3; ++i)
    {
        pid_out_parameter[i].out_data_limit = 600.0;
        pid_out_parameter[i].integral_value_limit = 100.0;
    }
    for (uint8_t i = 3; i < 6; ++i)
    {
        pid_out_parameter[i].out_data_limit = 600.0;
        pid_out_parameter[i].integral_value_limit = 100.0;
    }

    pid_out_parameter[0].parameter.kp = 20;
    pid_out_parameter[1].parameter.kp = 10;
    pid_out_parameter[2].parameter.kp = 35;
    pid_out_parameter[3].parameter.kp = 15;
    pid_out_parameter[4].parameter.kp = 15;
    pid_out_parameter[5].parameter.kp = 15;

    pid_out_parameter[0].parameter.ki = 0;
    pid_out_parameter[1].parameter.ki = 0;
    pid_out_parameter[2].parameter.ki = 0;
    pid_out_parameter[3].parameter.ki = 0;
    pid_out_parameter[4].parameter.ki = 0;
    pid_out_parameter[5].parameter.ki = 0.5;

    pid_out_parameter[0].parameter.kd = 2;
    pid_out_parameter[1].parameter.kd = 0;
    pid_out_parameter[2].parameter.kd = 0;
    pid_out_parameter[3].parameter.kd = 0;
    pid_out_parameter[4].parameter.kd = 0;
    pid_out_parameter[5].parameter.kd = 2;

    for (int v = 0; v < 8; ++v)
    {
        pid_in_parameter[v].out_data_limit = 1000;
    }
    //翻滚pid参数
    pid_roll.kd = 40;
    pid_roll.ki = 2;
    pid_roll.kp = 5;
}

//! 现在对pid数据的编号进行定义

//! 0-roll 1-pitch 2-yaw 3-向前 4-左移 5-向上（正方向定义）
// actual_value为加入运算
// 特别注意翻滚数据突变带来的影响

// 用来处理控制数据的线程
void Handle_Control_Task(void *pvParameters)
{
    for (;;)
    {

        pid_out_parameter[0].target_value += (float)move_roll * 0.005f;

        pid_out_parameter[1].target_value += (float)move_pitch * 0.0015f;

        pid_out_parameter[2].target_value += (float)move_yaw * 0.004f;

        pid_out_parameter[3].err = -(float)go_forward * 0.5f;

        pid_out_parameter[4].err = (float)go_left * 0.5f;

        pid_out_parameter[5].target_value -= (float)go_up * 0.00001f; // 1s下潜5cm

        if (pid_out_parameter[5].target_value < 0)
        {
            pid_out_parameter[5].target_value = 0.0001; // 深度不能为负
        }

        if (mode.light_on == 0x02) // 关灯
        {
            light_set(0);
        }
        if (mode.light_on == 0x01 && mode_last.light_on == 0x02) // 开灯
        {
            light_set(15); // 亮度
        }

        if (mode.unlock == 0x02) // 关闭电机
        {
            move_lock_flag = 0; // 停止运动
            for (uint8_t i = 0; i < 8; ++i)
            {
                pid_in_parameter[i].out_data = 0; // 停止输出
                motor_set(i + 1, 0);
            }
        }
        if (mode.unlock == 0x01 && mode_last.unlock == 0x02)
        {
            move_lock_flag = 1; // 开启运动

            pid_out_parameter[0].target_value = roll_total;// 翻滚角复位
            pid_out_parameter[1].target_value = pitch_total;// 俯仰角复位
            pid_out_parameter[2].target_value = yaw_total; // 偏航角复位
        }
        if (mode.electromagnet == 0x02) // 电磁铁关闭
        {
            servo_electromagnet_set(0);
        }
        if (mode.electromagnet == 0x01) // 电磁铁开启
        {
            servo_electromagnet_set(1);
        }

        if (mode.push_rod == 0x02) // 推杆关闭
        {
            push_rod_set(0);
        }
        if (mode.push_rod == 0x01) // 推杆开启
        {
            push_rod_set(1);
        }

        if (mode.autotrip == 0x01) // 定速开启（巡航模式）
        {
            pid_out_parameter[4].target_value = 20; // 左右平移
        }

        // if(mode.autorolling == 0x01) // 翻滚开启
        // {
        //     pid_out_parameter[0].parameter.kp = pid_roll.kp;
        //     pid_out_parameter[0].parameter.ki = pid_roll.ki;
        //     pid_out_parameter[0].parameter.kd = pid_roll.kd; 
        // }
        mode_last = mode; // 更新上一次的模式数据
        if (move_lock_flag == 1)
        {
            Pid_Out_Calculate(); // 计算pid并输出
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
