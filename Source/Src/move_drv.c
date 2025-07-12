#include "move_drv.h"
#include "tim.h"

void pwm_init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}
/*
 *默认50hz
 * pwm占空比设置函数
 * num 1,2,3代表htim1，htim2，htim3
 * channel表示通道 可选1,2,3,4
 * data为pause，最高为2000
 */
void pwm_set(uint8_t num, uint8_t channel, uint16_t data)
{
    switch (num)
    {
    case 1:
        switch (channel)
        {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, data);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, data);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, data);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, data);
            break;
        default:
            break;
        }
        break;
    case 2:
        switch (channel)
        {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, data);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, data);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, data);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, data);
            break;
        default:
            break;
        }
        break;
    case 3:
        switch (channel)
        {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, data);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, data);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, data);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, data);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}
/*
 * data -1000--+1000
 */
void motor_set(uint8_t num, float data)
{
    float limit = 600;
    if (data > limit)
    {
        data = limit;
    }
    if (data < -limit)
    {
        data = -limit;
    }
    // 5%-7.5%-10% 100-150-200
    uint16_t data_temp_forward = (uint16_t)((data + 1000) / 20 + 100); // 100-200
    uint16_t data_temp_backward = (uint16_t)((-data + 1000) / 20 + 100); // 100-200
    switch (num)
    {
    case 1:
        pwm_set(1, 4, data_temp_forward); // up-l-f
        break;
    case 2:
        pwm_set(1, 3, data_temp_forward); // up-r-f
        break;
    case 3:
        pwm_set(2, 4, data_temp_forward); // up-l-b
        break;
    case 4:
        pwm_set(2, 3, data_temp_forward); // up-r-b
        break;
    case 5:
        pwm_set(1, 2, data_temp_forward); // down-l-f
        break;
    case 6:
        pwm_set(1, 1, data_temp_forward); // down-r-f
        break;
    case 7:
        pwm_set(2, 2, data_temp_forward); // down-l-b
        break;
    case 8:
        pwm_set(2, 1, data_temp_forward); // down-r-b
        break;
    default:
        break;
    }
}

/*设置探照灯的亮度
 * 0-100
 * */
void light_set(uint16_t data)
{
    uint16_t data_set;
    if (data > 100)
        data = 100;
    data_set = data + 100;
    pwm_set(3, 3, data_set);
}

void move_putt_init(void)
{
    pwm_set(3,1,2000);
    pwm_set(3,2,100);
}
/*****************************************仅完成基本运动**************************************** */
//  1           2
//      5   6
//
//
//      7   8
//3            4      
//自行定义推进器转动方向
//对于推进器1-4，其中，向下推送为正
//对于推进器5-8，其中，向外推送为正

#include "FreeRTOS.h"
#include "semphr.h"
#include "tim.h"
#include "move_control.h"
#include "jy901p_uart.h"
#include "pid.h"
#include "ms5837_uart.h"

/* 全局信号量句柄，外部可用来在 PID 线程等待 */
SemaphoreHandle_t xTimer10Semaphore = NULL;

PIDController depth_PID;
PIDController roll_pid;
PIDController pitch_pid;
PIDController yaw_pid;


void Move_basic_Init(void)
{
    // 创建信号量
    xTimer10Semaphore = xSemaphoreCreateBinary();
    configASSERT(xTimer10Semaphore);
    PID_Init(&depth_PID,2.0f,0.5f,0.1f,-100,100);
    PID_Init(&roll_pid,2.0f,0.5f,0.1f,-100,100);
    PID_Init(&yaw_pid,2.0f,0.5f,0.1f,-100,100);
    PID_Init(&pitch_pid,2.0f,0.5f,0.1f,-100,100);

    // 启动定时器，定时发送信号量
    HAL_TIM_Base_Start_IT(&htim10);
}
typedef struct
{
    float go_forward_num;     // 前进后退
    float go_left_num;   // 左右移动
    float go_up_num;     // 上下移动

    float move_yaw_num;    // 左右转动
    float move_pitch_num;  // 上下转动
    float move_roll_num;   // 左右翻滚
} MOVE_NUMBER;


int16_t motor_set_data[8];

MOVE_NUMBER move_err = {0}; 

float actual_roll,actual_pitch,actual_yaw;
static void jy901_to_actual(void)
{
    //!jy901的默认角度中，roll是实际中的pitch
    actual_pitch = roll_total;
    actual_roll = actual_pitch;
    actual_yaw = yaw_total;
}

static void motor_basic_output(void)
{
    motor_set(3,motor_set_data[0]);
    motor_set(5,motor_set_data[1]);
 
 
    motor_set(4,motor_set_data[2]);
    motor_set(1,motor_set_data[3]);
    motor_set(7,motor_set_data[4]);
    motor_set(6,motor_set_data[5]);
    motor_set(8,motor_set_data[6]);
    motor_set(2,motor_set_data[7]);
    
}



// !需要根据实际更改motor_set中的对应关系
//俯仰、翻滚、偏航，前后、上下、左右
void upper_move_process(void *pvParameters)
{
    for (;;)
    {
        // 等待定时器信号量
        if (xSemaphoreTake(xTimer10Semaphore, portMAX_DELAY) == pdTRUE)
        {
            jy901_to_actual();

            move_err.go_forward_num = go_forward*2;
            move_err.go_left_num = go_left*2;

            //需要加入pid进行处理

            move_err.go_up_num = go_up*2;
            move_err.move_pitch_num = move_pitch*2;
            move_err.move_roll_num = move_roll*2;
            move_err.move_yaw_num = move_yaw*2;
            // move_err.go_up_num = PID_Update(&depth_PID, (ms5837_depth+(float)go_up*0.0001f), ms5837_depth, 0.02f);
            // move_err.move_roll_num = PID_Update(&roll_pid,(actual_roll+(float)move_roll*0.1f),actual_roll,0.02f);
            // move_err.move_yaw_num = PID_Update(&yaw_pid,(actual_yaw+(float)actual_yaw*0.1f),actual_yaw,0.02f);


            motor_set_data[0] = + move_err.move_pitch_num  + move_err.move_roll_num + move_err.go_up_num;
            motor_set_data[1] = + move_err.move_pitch_num - move_err.move_roll_num + move_err.go_up_num;
            motor_set_data[2] = - move_err.move_pitch_num  +move_err.move_roll_num + move_err.go_up_num;
            motor_set_data[3] = -move_err.move_pitch_num - move_err.move_roll_num + move_err.go_up_num;

            motor_set_data[4] = + move_err.move_yaw_num + move_err.go_forward_num - move_err.go_left_num;
            motor_set_data[5] = - move_err.move_yaw_num + move_err.go_forward_num + move_err.go_left_num;
            motor_set_data[6] = - move_err.move_yaw_num - move_err.go_forward_num - move_err.go_left_num;
            motor_set_data[7] = + move_err.move_yaw_num - move_err.go_forward_num + move_err.go_left_num;

    
            motor_basic_output();
        }
    }
}
#include "FreeRTOS.h"
#include "task.h"
void pull_use_electric(void *pvParameters)
{
    for (;;)
    {
        if (mode.electromagnet == 0x1)//退出
        {
            pwm_set(3,1,100);
            pwm_set(3,2,1100);
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);


        }
        else if(mode.electromagnet == 0x2)//收回
        {
            pwm_set(3,1,1800);
            pwm_set(3,2,100);

            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
            vTaskDelay(pdMS_TO_TICKS(10000));

        }
        else if(mode.electromagnet == 0x0)
        {
            pwm_set(3,1,0);
            pwm_set(3,2,0);

            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
        }
        
            
        vTaskDelay(pdMS_TO_TICKS(300));

    }
}
