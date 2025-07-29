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
    case 2:
        pwm_set(1, 4, data_temp_forward); // up-l-f   //2
        break;
    case 8:
        pwm_set(1, 3, data_temp_forward); // up-r-f   //8
        break;
    case 4:
        pwm_set(2, 4, data_temp_forward); // up-l-b   //!
        break;
    case 5:
        pwm_set(2, 3, data_temp_forward); // up-r-b    //5
        break;
    case 7:
        pwm_set(1, 2, data_temp_forward); // down-l-f   //7
        break;
    case 3:
        pwm_set(1, 1, data_temp_forward); // down-r-f   //!
        break;
    case 1:
        pwm_set(2, 2, data_temp_forward); // down-l-b    //1
        break;
    case 6:
        pwm_set(2, 1, data_temp_forward); // down-r-b   //6
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
void light_set(uint16_t light_data)
{
    static uint8_t light_data_last = 5;// 上一次亮度
    if (light_data != light_data_last) // 如果亮度改变了
    {
        light_data_last = light_data; // 更新上一次亮度
        if (light_data > 100)
            light_data = 100; // 限制最大值
        pwm_set(3, 1, (uint16_t)(light_data + 100)); // 设置占空比
    }
}

/**
 * @brief 设置电磁铁的状态
 * @param data 0 关闭电磁铁，1 开启电磁铁（低电平触发）
 */
void electromagnet_set(uint8_t electromagnet_data)
{
    static uint8_t electromagnet_data_last = 5; // 上一次电磁铁状态
    if (electromagnet_data != electromagnet_data_last) // 如果电磁铁状态
    {
        electromagnet_data_last = electromagnet_data; // 更新上一次状态
        if (electromagnet_data == 0)
        {
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_SET); // 关闭电磁铁)
        }
        if (electromagnet_data == 1) 
        {
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0, GPIO_PIN_RESET); // 开启电磁铁
        }
    }

}
/**
 * @brief 设置推杆的状态
 * @param data 0 推杆拉回，1 推杆推出
 */
void push_rod_set(uint8_t push_rod_data)
{
    static uint8_t push_rod_data_last = 5; // 上一次推杆状态
    if (push_rod_data != push_rod_data_last) // 如果推杆状态
    {
        push_rod_data_last = push_rod_data; // 更新上一次状态
        if (push_rod_data == 0) // 推杆拉回
        {
            pwm_set(3, 3, 1500); // 设置推杆拉回
            pwm_set(3, 4, 0); // 停止推出
        }
        else if (push_rod_data == 1) // 推杆推出
        {
            pwm_set(3, 3, 0); // 停止拉回
            pwm_set(3, 4, 1500); // 设置推杆推出
        }
    }
}

void servo_set(uint8_t servo_data)
{
    if (servo_data > 180) servo_data = 180;

    // 线性映射 0–180° → 100–200 ticks
    // (100/180 ≈ 0.5556)
    uint16_t data = (uint16_t)(servo_data * (100.0f / 180.0f) + 100.5f);

    pwm_set(3, 2, data);  // 频道 3，通道 2 只是示例
}
/**
 * @brief 电磁铁处于0x01状态时，设置舵机的pwm，进行旋转90度操作，然后
 */
void servo_electromagnet_set(uint8_t servo_electromagnet_data)
{      
    static uint8_t servo_electromagnet_data_last = 0; // 上一次舵机数据
    static uint8_t servo_data = 0; // 舵机数据

    if (servo_electromagnet_data != servo_electromagnet_data_last) // 如果舵机数据改变了
    {
        servo_electromagnet_data_last = servo_electromagnet_data; // 更新上一次数据
        if (servo_electromagnet_data == 0) // 关闭电磁铁
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // 关闭电磁铁
        }
        else if (servo_electromagnet_data == 1) // 开启电磁铁
        {
            servo_set(servo_data+90); // 设置舵机旋转90度
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // 开启电磁铁
        }
    }
}

