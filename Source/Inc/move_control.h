#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

typedef struct 
{
    float kp;
    float ki;
    float kd;
}pid_rov;

typedef struct 
{
    float target_value;         //预期值
    float actual_value;         //实际值
    float err;                  //偏差值
    float err_last;             //上一次偏差值
    float integral_value;       //积分值
    float integral_value_limit; //积分值上限
    pid_rov parameter;   //pid参数 kp、ki、kd
    float out_data;             //输出值
    float out_data_limit;       //输出值限幅（正）
    float index;                //变积分指数
    float index_of_err_up;      //误差高于此值后积分指数为0
    float index_of_err_down;    //误差低于此值后积分指数为1
}pid_set;

typedef struct
{
     int16_t go;
     int16_t move;
     int16_t up;
     int16_t yaw;
     int16_t pitch;
     int16_t roll;
}S_handle;

typedef struct
{
     uint8_t lockangle;//姿态锁定
     uint8_t autotrip;//开启定速
     uint8_t autovertical;//自动对正
     uint8_t autorolling;//翻滚
     uint8_t defogging;//待定
     uint8_t unlock;//开启电机
     uint8_t electromagnet;//电磁铁
     uint8_t push_rod;//推杆
     uint8_t light_on;//开启灯光

}S_mode;

typedef struct
{
   struct
   {
     float kp[6];
   }p;
   struct
   {
    float ki[6];
   }i;
   struct
   {
    float kd[6];
   }d;
}S_pid_outcircle;

typedef struct
{
       float kp;
       float ki;
       float kd;
}S_pid_depth;


extern S_handle handle;

extern int16_t go_forward;
extern int16_t go_left;
extern int16_t go_up;

extern int16_t move_yaw;
extern int16_t move_pitch;
extern int16_t move_roll;

extern S_mode mode;
extern S_pid_depth pid_depth;


//初始化各个pid数据结构体
//为代码简洁方便，用数组表示；分别为俯仰、翻滚、偏航，前后、上下、左右
extern  pid_set pid_out_parameter[6];
//8个推进器
extern  pid_set pid_in_parameter[8];
extern  pid_set depth_pid;


/** TIM11 到期信号量，供 PID 线程等待 */
extern SemaphoreHandle_t xTimer11Semaphore;
/** TIM14 到期信号量，供 PID 线程等待 */
extern SemaphoreHandle_t xTimer14Semaphore;
void motor_init(void);
void Move_Control_Task(void *pvParameters);






#endif /* MOVE_CONTROL_H */

