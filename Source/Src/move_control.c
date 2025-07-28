#include "move_control.h"
#include <math.h>
#include <stdint.h>
#include "jy901p_uart.h"
#include "ms5837_uart.h"
#include "transmit_power_board.h"
#include "move_drv.h"
#include "tim.h"
#include "math.h"

S_handle handle = {0}; // 手柄数据
// 平移
int16_t go_forward = 0; // 正数向前，负数向后
int16_t go_left = 0;    // 正数向左，负数向右
int16_t go_up = 0;      // 正数上升，负数下降
// 角度
int16_t move_yaw = 0;   // 正数向左，负数向右
int16_t move_pitch = 0; // 正数向上，负数向下
int16_t move_roll = 0;  // 正数顺时针，负数逆时针
//pid
pid_set pid_out_parameter[6];
pid_set pid_in_parameter[8]; // 内环电流控制参数，按设定推进器编号排布
pid_set depth_pid;
//功能
S_mode mode = {0};
S_mode mode_last = {0}; // 上一次的模式数据

S_pid_depth pid_depth = {0};

float err_thea;
uint16_t depth_target_value;
uint8_t move_lock_flag = 0; //运动停止标志位
float roll_target, pitch_target, yaw_target; // 目标值

static void rov_move_data_process(float rov_move_data[6])
{
    rov_move_data[0] = roll_total; // 旋转后方向
    rov_move_data[1] = pitch_total;
    rov_move_data[2] = yaw_total;
}

static void motor_output(void)
{
     //motor_set有限幅
    for(uint8_t i=0;i<8;++i)
    {
        motor_set(i+1,pid_in_parameter[i].out_data);
    }
}

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

        if (mode.autorolling == 1)
        {
            pid_out_parameter[2].err = err_thea;
        }

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

    //对平动进行控制
    for (uint8_t i = 0 + 3; i < 3 + 3; ++i)
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

    /*定深pd*/
    if (depth_target_value != 0)
    {
        depth_pid.actual_value = ms5837_depth;
        depth_pid.err = depth_pid.target_value - depth_pid.actual_value;

        depth_pid.integral_value += depth_pid.err;
        // 对积分结果进行限幅
        if (depth_pid.integral_value > depth_pid.integral_value_limit)
        {
            depth_pid.integral_value = depth_pid.integral_value_limit;
        }
        else if (depth_pid.integral_value < -depth_pid.integral_value_limit)
        {
            depth_pid.integral_value = -depth_pid.integral_value_limit;
        }

        depth_pid.out_data = depth_pid.parameter.kp * depth_pid.err + depth_pid.parameter.ki * depth_pid.integral_value + depth_pid.parameter.kd * (depth_pid.err - depth_pid.err_last);
        // 对运算结果进行限幅
        if (depth_pid.out_data > depth_pid.out_data_limit)
        {
            depth_pid.out_data = depth_pid.out_data_limit;
        }
        else if (depth_pid.out_data < -depth_pid.out_data_limit)
        {
            depth_pid.out_data = -depth_pid.out_data_limit;
        }
        // 更新上次差值
        depth_pid.err_last = depth_pid.err;

        // 定深执行，直接给定上下浮动的差值
        pid_out_parameter[5].err = -depth_pid.out_data;//因为上升为正，下潜为负，
    }
    // 0-翻滚 1-俯仰
    static float a, b, c;
    a = 1;
    b = 1;
    c = 1;
    // 判断pitch(运动限幅)
    if (pitch_total > 80 || pitch_total < -80)
    {
        a = 0;//roll
        b = 0;//pitch
        c = 0;//yaw
    }
    else
    {
        a = 1;
        b = 1,
        c = 1;
    }

    pid_in_parameter[0].out_data = + a*pid_out_parameter[0].out_data + b*pid_out_parameter[1].out_data + pid_out_parameter[5].out_data;
    pid_in_parameter[1].out_data = - a*pid_out_parameter[0].out_data + b*pid_out_parameter[1].out_data + pid_out_parameter[5].out_data;
    pid_in_parameter[2].out_data = + a*pid_out_parameter[0].out_data - b*pid_out_parameter[1].out_data + pid_out_parameter[5].out_data;
    pid_in_parameter[3].out_data = - a*pid_out_parameter[0].out_data - b*pid_out_parameter[1].out_data + pid_out_parameter[5].out_data;

    pid_in_parameter[4].out_data = - c*pid_out_parameter[2].out_data - pid_out_parameter[3].out_data - pid_out_parameter[4].out_data;
    pid_in_parameter[5].out_data = + c*pid_out_parameter[2].out_data - pid_out_parameter[3].out_data + pid_out_parameter[4].out_data;
    pid_in_parameter[6].out_data = + c*pid_out_parameter[2].out_data + pid_out_parameter[3].out_data - pid_out_parameter[4].out_data;
    pid_in_parameter[7].out_data = - c*pid_out_parameter[2].out_data + pid_out_parameter[3].out_data + pid_out_parameter[4].out_data;

    motor_output();
}


// 用于校准方向
void rov_move_uppper_process(void *pvParameters)
{
    for (;;)
    {
        if (go_forward == 100)
        {
            motor_set(1, 400);
        }
        if (go_forward == -100)
        {
            motor_set(2, 400);
        }
        if (go_left == 100)
        {
            motor_set(3, 400);
        }
        if (go_left == -100)
        {
            motor_set(4, 400);
        }
        if (go_up == 100)
        {
            motor_set(5, 400);
        }
        if (go_up == -100)
        {
            motor_set(6, 400);
        }
        if (move_yaw == 100)
        {
            motor_set(7, 400);
        }
        if (move_yaw == -100)
        {
            motor_set(8, 400);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_set(1, 500);
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_set(1,-500);
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_set(1, 0);
    }
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
    pwm_init(); // 初始化PWM
    // 推进器初始化
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, 0);
    }
    HAL_Delay(10000); // 等待10秒
    // 检查是否能动
    motor_set(1,180);//为负
    HAL_Delay(5000);
    motor_set(1,0);

    motor_set(2,180);//为负
    HAL_Delay(5000);
    motor_set(2,0);

    motor_set(3,180);//为负
    HAL_Delay(5000);
    motor_set(3,0);

    motor_set(4,180);//为正
    HAL_Delay(5000);
    motor_set(4,0);

    // motor_set(5,180);//为正  逆时针
    // HAL_Delay(5000);
    // motor_set(5,0);

    // motor_set(6,180);//为负     
    // HAL_Delay(5000); 
    // motor_set(6,0);

    // motor_set(7,180);//为正   
    // HAL_Delay(5000);
    // motor_set(7,0);

    // motor_set(8,180);//为正
    // HAL_Delay(5000);
    // motor_set(8,0);    
    // for (uint8_t i = 0; i < 8; ++i)
    // {
    //     motor_set(i + 1, 180);
    // }
    HAL_Delay(2000); // 等待5秒
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, 0);
    }
}

/* 全局信号量句柄，外部可用来在 PID 线程等待 */
SemaphoreHandle_t xTimer11Semaphore = NULL;
SemaphoreHandle_t xTimer14Semaphore = NULL;

//! 对于rov的控制，控制频率几十到一百多就可以了
//采用14定时器处理上位机数据
// 采用11定时器处理PID数据，并输出到推进器

void Move_Control_Task_Init(void)
{
    /* 创建信号量，供 TIM 回调给 PID 线程发信号 */
    xTimer11Semaphore = xSemaphoreCreateBinary();
    xTimer14Semaphore = xSemaphoreCreateBinary();
    configASSERT(xTimer11Semaphore);
    configASSERT(xTimer14Semaphore);

    /*  启动硬件定时器中断 */
    HAL_TIM_Base_Start_IT(&htim14);//控制数据+运动控制中断

    //设置输出限幅幅值，积分限幅幅值,变积分指数值误差分界值
    for(uint8_t i=0;i<6;++i)
    {
        pid_out_parameter[i].out_data_limit=500.0;
        pid_out_parameter[i].integral_value_limit=15.0;
    }

    pid_out_parameter[0].parameter.kp=12;
    pid_out_parameter[1].parameter.kp=12;
    pid_out_parameter[2].parameter.kp=12;
    pid_out_parameter[3].parameter.kp=15;
    pid_out_parameter[4].parameter.kp=15;
    pid_out_parameter[5].parameter.kp=15;

    pid_out_parameter[0].parameter.ki=2;
    pid_out_parameter[1].parameter.ki=2;
    pid_out_parameter[2].parameter.ki=2;
    pid_out_parameter[3].parameter.ki=0;
    pid_out_parameter[4].parameter.ki=0;
    pid_out_parameter[5].parameter.ki=0;

    pid_out_parameter[0].parameter.kd=30;
    pid_out_parameter[1].parameter.kd=30;
    pid_out_parameter[2].parameter.kd=30;
    pid_out_parameter[3].parameter.kd=30;
    pid_out_parameter[4].parameter.kd=30;
    pid_out_parameter[5].parameter.kd=30;

    depth_pid.parameter.kp=2.5;
    depth_pid.parameter.ki=0.05;
    depth_pid.parameter.kd=1.3;
    depth_pid.out_data_limit=100;
    depth_pid.integral_value_limit=1;

   for(int v=0;v<8;++v)
    {
       pid_in_parameter[v].out_data_limit=1000;
    }    
}

//! 现在对pid数据的编号进行定义

//! 0-roll 1-pitch 2-yaw 3-向前 4-左移 5-向上（正方向定义）
//actual_value为加入运算
//特别注意翻滚数据突变带来的影响

//用来处理控制数据的线程
void Handle_Control_Task(void *pvParameters)
{
    for (;;)
    {
        // 等待定时器信号量
        if (xSemaphoreTake(xTimer14Semaphore, portMAX_DELAY) == pdTRUE)
        {
            if (fabsf(move_roll)>2)
            {
                pid_out_parameter[0].target_value += (float)move_roll*0.04f;
            }
            if (fabsf(move_pitch)>2)
            {
                pid_out_parameter[1].target_value += (float)move_pitch*0.04f;
            }
            if (fabsf(move_yaw)>2)
            {
                pid_out_parameter[2].target_value += (float)move_yaw*0.04f;
            }

            if (fabsf(go_forward)>2)
            {
                pid_out_parameter[3].err = (float)go_forward*0.5f;
            }
            if (fabsf(go_left)>2)
            {
                if (mode.autotrip == 0x02)
                {
                    pid_out_parameter[4].err = (float)go_left*0.5f;
                }
            }
            if (fabsf(go_up)>2)
            {
                if (mode.lockangle == 0x02) // 姿态未锁定
                {
                    pid_out_parameter[5].err = (float)go_up*0.05f; // 上升为正，下潜为负
                }
                else if (mode.lockangle == 0x01 && mode_last.lockangle == 0x02) // 姿态锁定
                {
                    pid_out_parameter[5].err = 0; // 锁定时不允许上下浮动
                    depth_target_value = ms5837_depth;
                }
                else if(mode.lockangle == 0x01) // 姿态锁定
                {
                    pid_out_parameter[5].err = 0; // 锁定时不允许上下浮动
                    depth_target_value = depth_target_value - (float)go_up*0.05f; // 上升为正，下潜为负
                }
            }

            if (mode.light_on == 0x02)//关灯
            {
                light_set(0);
            }
            if (mode.light_on == 0x01 && mode_last.light_on == 0x02)//开灯
            {
                light_set(50);//一半亮度
            }

            if (mode.unlock == 0x02)//关闭电机
            {
                move_lock_flag = 0; // 停止运动
                for (uint8_t i = 0; i < 8; ++i)
                {
                    motor_set(i + 1, 0);
                }

            }
            if (mode.unlock == 0x01 && mode_last.unlock == 0x02)//开启电机
            {
                move_lock_flag = 1;
            }
            
            if (mode.electromagnet == 0x02)//电磁铁关闭
            {
                electromagnet_set(0);
            }
            if (mode.electromagnet == 0x01 && mode_last.electromagnet == 0x02)//电磁铁开启
            {
                electromagnet_set(1);
            }

            if (mode.push_rod == 0x02)//推杆关闭
            {
                push_rod_set(0);
            }
            if (mode.push_rod == 0x01 && mode_last.push_rod == 0x02)//推杆开启
            {
                push_rod_set(1);
            }
            
            if (mode.autotrip == 0x01)//定速开启（巡航模式）
            {
                pid_out_parameter[4].target_value = 20; // 左右平移
            }
              
            mode_last = mode; // 更新上一次的模式数据
        }
        if (move_lock_flag == 1)
        {
            Pid_Out_Calculate(); // 计算pid并输出
        }
        
    }
}

