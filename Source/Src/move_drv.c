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
    float limit = 800;
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
