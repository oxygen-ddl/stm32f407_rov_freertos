#include "ms5837_uart.h"
#include "usart.h"    // CubeMX 生成的 huart4 句柄
#include "stm32f4xx_hal.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"


UART4_Msg_t uart4_msg;
uint8_t uart4_it_flag;
uint8_t uart4_rx_buffer[UART4_DMA_BUF_SIZE];

/* 协议里“温度”中文前缀的 GBK 编码 */
static const uint8_t PREFIX_TEMP[4]  = {0xCE,0xC2,0xB6,0xC8};
/* 协议里“深度”中文前缀的 GBK 编码 */
static const uint8_t PREFIX_DEPTH[4] = {0xC9,0xEE,0xB6,0xC8};


/* 解析结果 */
float ms5837_temperature = 0.0f;
float ms5837_depth       = 0.0f;
float ms5837_pressure = 0.0f; 

/**
 * @brief  从 buf 中以“前缀 + ':' + ASCII 数字 + '.' + ASCII 数字”格式解析一个数值
 * @param  buf   起始地址
 * @param  len   buf 可读长度
 * @param  prefix 4 字节前缀编码
 * @param  out   结果（带符号，减去 0x30）
 * @return  >=0 成功解析到并写入 *out，返回消耗的字节数；否则返回 -1
 */
static int parse_value(const uint8_t *buf, int len,
                       const uint8_t prefix[4],
                       float *out)
{
    int i = 0;
    // 找前缀
    for (; i + 5 < len; i++)
    {
        if (memcmp(buf + i, prefix, 4) == 0 && buf[i+4] == 0x3A /*':'*/)
            break;
    }
    if (i + 5 >= len) return -1;
    int j = i + 5;
    int sign = 1;
    int iv = 0, fv = 0, fd = 0;
    // 整数部分
    while (j < len && buf[j] != 0x2E /*'.'*/)
    {
        if (buf[j] == 0x2D /*'-'*/) { sign = -1; }
        else if (buf[j] >= '0' && buf[j] <= '9')
        {
            iv = iv * 10 + (buf[j] - '0');
        }
        else break;
        j++;
    }
    if (j >= len || buf[j] != 0x2E) return -1;
    j++; // 跳过 '.'
    // 小数两位
    for (fd = 0; fd < 2 && j < len; fd++, j++)
    {
        if (buf[j] >= '0' && buf[j] <= '9')
            fv = fv * 10 + (buf[j] - '0');
        else break;
    }
    // 组装
    *out = sign * (iv + fv / (float)(fd == 0 ? 1 : (fd==1?10:100)));
    return j - i;
}

/**
 * @brief  接收一段 DMA 数据后一次性调用，提取“温度”和“深度”
 */
void parse_frame(const uint8_t *buf, int len)
{
    // 解析温度
    parse_value(buf, len, PREFIX_TEMP, &ms5837_temperature);
    // 解析深度
    parse_value(buf, len, PREFIX_DEPTH, &ms5837_depth);
}

/**
 * @brief  初始化，启动 DMA + 空闲中断接收
 */
void Parser4_Init(void)
{
    // 启动 DMA 接收
    HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, UART4_DMA_BUF_SIZE);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
}

void UART4_IT_TASK(void)
{
    // 空闲检测
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        // 停 DMA，算长度
        HAL_UART_DMAStop(&huart4);
        uint16_t len = UART4_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart4.hdmarx);

        if (uart4_it_flag == 0)
        {
            uart4_msg.len = len;
            memcpy(uart4_msg.data, uart4_rx_buffer, len);
            uart4_it_flag = 1;
        }
        // 重启 DMA
        HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, UART4_DMA_BUF_SIZE);
    }
}

//
void MS5837_ProcessTask(void *pvParameters)
{
    for (;;)
    {
        if (uart4_it_flag == 1)
        {
            parse_frame(uart4_msg.data, uart4_msg.len);
            uart4_it_flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

