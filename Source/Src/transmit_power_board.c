#include "transmit_power_board.h"
#include <stdio.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

#define data_buffer_size 25

uint16_t current_adc_data[9];
uint16_t adc_data_after_filter[9];
uint16_t motor_current_actual[8]; // adc采集的推进器电流数据，由电源板采集后传入主控

uint8_t electronic_switch[8];
uint16_t current_adc_data[9];
float temperature_power_board;
float RH_power_board;
uint8_t switch_send_flag;

uint8_t Tx_buffer_5[data_buffer_size];

uint8_t uart5_it_flag = 0;   // 0表示没有新数据，1表示有新数据的到来
Power_board_Msg_t uart5_msg; // 作为全局变量，在中断与处理函数中传递接收到的数据以及长度

/*清空发送缓冲区*/
static void tx_buffer_clear(void)
{
    for (uint8_t i = 0; i < data_buffer_size; ++i)
    {
        Tx_buffer_5[i] = 0;
    }
}

static void send_data_assembly(void)
{
    tx_buffer_clear();
    uint8_t i = 0, data_temp[8];
    Tx_buffer_5[0] = 0xaa;
    Tx_buffer_5[1] = 0xa2;
    for (uint8_t j = 0; j < 8; ++j)
    {
        data_temp[j] = electronic_switch[j];
        data_temp[j] |= 0x80;
    }
    for (i = 2; i < 8 + 2; ++i)
    {
        Tx_buffer_5[i] = data_temp[i - 2];
    }
    Tx_buffer_5[i + 1] = 0xaf;
    Tx_buffer_5[i] = electronic_switch[0] + electronic_switch[2] + electronic_switch[7] + 0xaf;
    Tx_buffer_5[i + 2] = 0xff;
}
/*
 * @brief  解析一段缓存区里的完整帧（帧头 0xAA，帧尾 0xFF）
 * @param  buf    指向接收到的数据数组
 * @param  len    数组长度
 */
void parsePowerBoardFrame(const uint8_t *buf, uint8_t len)
{
    uint8_t head = 0, tail = 0, data_head = 0, data_tail = 0;
    uint8_t data_check = 0;

    // 1) 找到 head/data_head
    for (uint8_t i = 0; i + 1 < len; ++i)
    {
        if (buf[i] == 0xAA &&
            (buf[i + 1] == 0xA1 || buf[i + 1] == 0xA3 ||
             buf[i + 1] == 0xA4 || buf[i + 1] == 0xA5))
        {
            head = i;
            data_head = i + 1;
            break;
        }
    }
    // 2) 找到 data_tail/tail
    for (uint8_t i = data_head + 2; i < len; ++i)
    {
        if (buf[i] == 0xFF && buf[i - 1] == 0xAF)
        {
            tail = i;
            data_tail = i - 1;
            break;
        }
    }
    // 检查合法性
    if (head >= tail || data_head >= data_tail)
    {
        //HAL_UART_Transmit_DMA(&huart6, &buf[data_head], 1);
    }

    // 根据 data_head 指针判断包类型
    switch (buf[data_head])
    {
    case 0xA1: // ADC 推进器电流包，payload 长度=18
    {
        if (data_tail - data_head - 1 < 18)
            return;
        // data bytes start at buf[data_head+1]
        const uint8_t *p = buf + data_head + 1;
        // 校验：data_temp[0]+data_temp[2]+data_temp[15] + buf[data_tail]
        data_check = p[0] + p[2] + p[15] + buf[data_tail];
        if ((uint8_t)(data_check) != buf[data_tail - 1])
            return;

        // 拆 9 路 16 位电流
        for (uint8_t i = 0; i < 9; ++i)
        {
            current_adc_data[i] = (uint16_t)(p[2 * i] << 8) | p[2 * i + 1];
        }
        // 电压异常剔除
        if (current_adc_data[4] == 0)
            current_adc_data[4] = current_adc_data[4]; // old value 保持不变
        break;
    }
    case 0xA3: // 温湿度包，payload=4
    {
        if (data_tail - data_head - 1 < 4)
            return;
        const uint8_t *p = buf + data_head + 1;
        data_check = p[0] + p[3] + buf[data_tail];
        if ((uint8_t)(data_check) != buf[data_tail - 1])
            return;

        RH_power_board = (uint16_t)(p[0] << 8) | p[1];
        temperature_power_board = (uint16_t)(p[2] << 8) | p[3];
        break;
    }
    case 0xA4: // 活动校验包，payload=1
    {
        uint8_t tx[6] = {0xAA, 0xA4, 0xCC, (uint8_t)(0xCC + 0xAF), 0xAF, 0xFF};
        HAL_UART_Transmit_DMA(&huart5, tx, sizeof(tx));
        break;
    }
    case 0xA5: // 开关设置成功包，payload=1
    {
        if (data_tail - data_head - 1 < 1)
            return;
        uint8_t v = buf[data_head + 1];
        data_check = v + buf[data_tail];
        if ((uint8_t)(data_check) != buf[data_tail - 1] || v != 0x65)
            return;

        switch_send_flag = 1;
        break;
    }
    default:
        break;
    }
}

/*
 * 推进器电子开关设置
 * choose:0-7 set:1 or 0
 * */
void propeller_switch_set(uint8_t choose, uint8_t set)
{
    if (choose > 7)
    {
        printf("propeller_switch_set num err\n");
        return;
    }
    if (set != 0 && set != 1)
    {
        printf("propeller_switch_set set err\n");
        return;
    }
    if (choose < 7)
        electronic_switch[choose] = set;
    else
    {
        if (set == 1)
            electronic_switch[choose] |= 0x01; // set 1
        else
        {
            electronic_switch[choose] &= ~0x01; // set 0
        }
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}

/*舵机开关设置
 * 1-on 0-off
 * */
void server_switch_set(uint8_t data)
{
    uint8_t flag = (electronic_switch[7] >> 1) & 0x01;
    // 进行比较判断，相同则直接返回，不同则改变
    if (flag == data)
        return;
    // 操作第二位
    if (data == 0)
        electronic_switch[7] &= ~(1 << 1); // 第二位置零
    else if (data == 1)
    {
        electronic_switch[7] |= 1 << 1;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}

/*电机开关设置
 * 1-on 0-off
 * */
void motor_switch_set(uint8_t data)
{
    uint8_t flag = (electronic_switch[7] >> 2) & 0x01;
    // 进行比较判断，相同则直接返回，不同则改变
    if (flag == data)
        return;
    // 操作第三位
    if (data == 0)
        electronic_switch[7] &= ~(1 << 2); // 第三位置零
    else if (data == 1)
    {
        electronic_switch[7] |= 1 << 2;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}

/*电机转向设置
 * 1-反转 0-正转
 * */
void motor_dir_set(uint8_t data)
{
    // 操作第4位
    if (data == 0)
        electronic_switch[7] &= ~(1 << 3); // 第4位置零xxxx 0xxx
    else if (data == 1)
    {
        electronic_switch[7] |= 1 << 3;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}

/*
 * 探照灯电子开关
 * 1-on 0-off
 * */
void light_switch_set(uint8_t data)
{
    // 操作第5位
    if (data == 0)
        electronic_switch[7] &= ~(1 << 4); // 第5位置零xxx0 xxxx
    else if (data == 1)
    {
        electronic_switch[7] |= 1 << 4;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}

/************************************************************数据发送解析任务********************************************************************* */
#include "move_control.h"

uint8_t uart5_buf[power_board_max_len];

void Uart5_Parse_Init(void)
{
    HAL_UART_Receive_DMA(&huart5, uart5_buf, power_board_max_len);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
}
S_mode mode_last = {0};
//对电子开关进行设置，同时对接收到的上位机控制命令下达
void Uart5_Parse_Task(void *pvParameters)
{
    motor_switch_set(1);
    mode.light_on = 0x2;
    for (;;)
    {
        //开关灯设置
        if (mode.light_on != mode_last.light_on)
        {
            if (mode.light_on == 0x01)
            {
                light_switch_set(1);
            }
            else
            {
                light_switch_set(0);
            }
            mode_last.light_on = mode.light_on;
        }
        if (mode_last.unlock != mode.unlock)
        {
            if (mode.unlock == 0x01)
            {
                motor_switch_set(1);
            }
            else
            {
                motor_switch_set(0);
            }
        }
        
        if (uart5_it_flag == 1)
        {
            parsePowerBoardFrame(uart5_msg.data,uart5_msg.len);
            //printf("%d",uart5_msg.len);
            //HAL_UART_Transmit_DMA(&huart6, uart5_msg.data, 32);
            uart5_it_flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 数据发送——通过串口5
void switch_Process_Task(void *pvParameters)
{
    switch_send_flag = 0;
    for (;;)
    {
        while (switch_send_flag == 0)
        {
            send_data_assembly();
            HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, 13);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void UART5_IT_TASK(void)
{
    // 空闲检测
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);
        // 停 DMA，算长度
        HAL_UART_DMAStop(&huart5);
        uint16_t len = power_board_max_len - __HAL_DMA_GET_COUNTER(huart5.hdmarx);

        if (uart5_it_flag == 0)
        {
            uart5_msg.len = len;
            memcpy(uart5_msg.data, uart5_buf, len);
            uart5_it_flag = 1;
        }
        // 重启 DMA 
        HAL_UART_Receive_DMA(&huart5, uart5_buf, power_board_max_len);
    }
}
