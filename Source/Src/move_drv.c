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
/*
探照灯: 正极（红色）、负极（黑色）、黄色为信号线(调亮度)信号可以不接，不接就最大功率Pwm 就是舵机的pwm，直接插上电，红黑有电就有输出，然后根据pwm 来调节亮度。50hz 的pwm 波，占空比5%最暗，占空比10%最亮
探照灯12-24V供电，5v，3.3v都可以，我们内部有做限幅，直接接24v的信号也没事。
正极（红色）、负极（黑色）、黄色为信号线。信号可以不接，不接就最大功率亮
*/


/*设置探照灯的亮度
 * 0-100对应5%-10%占空比
 * */
void light_set(uint16_t data)
{
    uint16_t data_set;
    if (data > 100)
        data = 100;
    data_set = data + 100;
    pwm_set(3, 3, data_set);
}

/**
 * @brief 设置电磁铁的状态
 * @param data 0 关闭电磁铁，1 开启电磁铁
 */
void electromagnet_set(uint8_t data)
{
    if (data == 0)
    {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_SET); // 关闭电磁铁)
    }
    else if (data == 1)
    {
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1, GPIO_PIN_RESET); // 开启电磁铁
    }
}
/**
 * @brief 设置推杆的状态
 * @param data 0 推杆拉回，1 推杆推出
 */
void push_rod_set(uint8_t data)
{
    if (data == 0)//推杆拉回
    {
        pwm_set(3, 3, 1500);
        pwm_set(3, 4, 0); 

    }
    else if (data == 1)//推杆推出
    {
        pwm_set(3, 3, 0);
        pwm_set(3, 4, 1500); 
    }
}
