#include "move_control.h"
#include <math.h>
#include <stdint.h>
#include "jy901p_uart.h"
#include "ms5837_iic.h"
#include "transmit_power_board.h"
#include "move_drv.h"
#include "tim.h"


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
S_light light = {0};
S_mode mode = {0};
S_pid_depth pid_depth = {0};

float err_thea;
uint16_t depth_target_value;

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


static void Pid_Out_Calculate(void *pvParameters)
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
        pid_out_parameter[4].err = -depth_pid.out_data;
    }
    // 0-翻滚 1-俯仰
    static float a, b, c;
    a = 1;
    b = 1, c = 1;
    // 判断roll
    if (roll > 45 || roll < -45)
    {
        a = 0;
        b = 0;
        c = 0;
    }
    else
    {
        a = 1;
        b = 1, c = 1;
    }
    pid_in_parameter[0].out_data = +pid_out_parameter[0].out_data - a * pid_out_parameter[1].out_data - c * pid_out_parameter[4].out_data; // 推进器电流期望值
    pid_in_parameter[1].out_data = -pid_out_parameter[0].out_data - a * pid_out_parameter[1].out_data - c * pid_out_parameter[4].out_data;
    pid_in_parameter[2].out_data = +pid_out_parameter[0].out_data + a * pid_out_parameter[1].out_data - c * pid_out_parameter[4].out_data;
    pid_in_parameter[3].out_data = -pid_out_parameter[0].out_data + a * pid_out_parameter[1].out_data - c * pid_out_parameter[4].out_data;
    pid_in_parameter[4].out_data = +b * pid_out_parameter[2].out_data - pid_out_parameter[3].out_data - pid_out_parameter[5].out_data;
    pid_in_parameter[5].out_data = -b * pid_out_parameter[2].out_data - pid_out_parameter[3].out_data + pid_out_parameter[5].out_data;
    pid_in_parameter[6].out_data = -b * pid_out_parameter[2].out_data + pid_out_parameter[3].out_data - pid_out_parameter[5].out_data;
    pid_in_parameter[7].out_data = +b * pid_out_parameter[2].out_data + pid_out_parameter[3].out_data + pid_out_parameter[5].out_data;

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
    pwm_init(); // 初始化PWM
    // 推进器初始化
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // 等待5秒
    // 推进器初始化
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, 180);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待1秒
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 等待1秒
    for (uint8_t i = 0; i < 8; ++i)
    {
        motor_set(i + 1, 500); // 设置推进器初始值
    }
}

/* 全局信号量句柄，外部可用来在 PID 线程等待 */
SemaphoreHandle_t xTimer11Semaphore = NULL;
SemaphoreHandle_t xTimer14Semaphore = NULL;

void Move_Control_Task(void *pvParameters)
{
    motor_init();
    /* 创建信号量，供 TIM 回调给 PID 线程发信号 */
    xTimer11Semaphore = xSemaphoreCreateBinary();
    xTimer14Semaphore = xSemaphoreCreateBinary();
    configASSERT(xTimer11Semaphore);
    configASSERT(xTimer14Semaphore);

    /*  启动硬件定时器中断 */
    HAL_TIM_Base_Start_IT(&htim11);
    HAL_TIM_Base_Start_IT(&htim14);
}


void Move_PID_TASK(void *pvParameters)
{
    for(;;)
    {
        // 等待定时器信号量
        if (xSemaphoreTake(xTimer11Semaphore, portMAX_DELAY) == pdTRUE)
        {
            Pid_Out_Calculate(NULL);
        }
    }
}
