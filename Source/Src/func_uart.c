#include "func_uart.h"
#include "usart.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define MY_PRINTF_BUF_SIZE 256
static uint8_t _tx_buf[MY_PRINTF_BUF_SIZE];
static volatile bool _dma_busy = false;

/**
 * @brief DMA 发送完成回调（在中断上下文）
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        _dma_busy = false;
    }
}
/**
 * @brief 非阻塞 printf 实现
 */
bool my_printf(const char *fmt, ...)
{
    if (_dma_busy)
    {
        // 上一次发送尚未完成，丢弃本次
        return false;
    }

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf((char *)_tx_buf, MY_PRINTF_BUF_SIZE, fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return false;
    }
    if (len > MY_PRINTF_BUF_SIZE)
    {
        len = MY_PRINTF_BUF_SIZE;
    }

    // 启动 DMA 发送
    if (HAL_UART_Transmit_DMA(&huart2, _tx_buf, len) != HAL_OK)
    {
        // 发送启动失败
        return false;
    }

    _dma_busy = true;
    return true;
}
#define FLOAT_BUF_LEN 32

static uint8_t _float_buf[FLOAT_BUF_LEN];
bool uart_send_float(float f)
{
    if (_dma_busy)
    {
        // 上一次还在发送，直接丢弃
        return false;
    }
    // 格式化浮点数（保留 3 位小数，并换行）
    int len = snprintf((char *)_float_buf, FLOAT_BUF_LEN, "%.3f\r\n", f);
    if (len <= 0)
        return false;
    if (len > FLOAT_BUF_LEN)
        len = FLOAT_BUF_LEN;

    // 启动 DMA 非阻塞发送
    if (HAL_UART_Transmit_DMA(&huart2, _float_buf, len) != HAL_OK)
    {
        return false;
    }
    _dma_busy = true;
    return true;
}

#define MAX_FLOAT_COUNT 20
#define TX_BUF_SIZE (MAX_FLOAT_COUNT * 4 + 4)

bool uart_just_float_send(uint8_t count, ...)
{
    if (_dma_busy || count == 0 || count > MAX_FLOAT_COUNT)
    {
        return false;
    }

    va_list args;//声明一个保存可变参数状态的对象
    va_start(args, count);//用函数最后一个固定参数 count 来初始化 args，此后可通过 va_arg 逐个取出实参。
    uint8_t *p = _tx_buf;
    for (uint8_t i = 0; i < count; i++)
    {
        // float 在可变参数里会被提升为 double
        double tmp = va_arg(args, double);//所有 float 型的实参在传入 ... 时会自动以 double 形式压入栈；
        float v = (float)tmp;//要用 va_arg(args, double) 来取出，然后再 显式 转回 float （(float)tmp），才能得到原始 32-bit 浮点数。
        memcpy(p, &v, sizeof(v)); // 小端复制
        p += sizeof(v);
    }
    va_end(args);//va_end(args);：用完可变参数后务必调用，清理内部状态。

    // 末尾追加帧尾 0x00,0x00,0x80,0x7F
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    size_t len = p - _tx_buf;//计算要发送的总字节长度：(结束位置 – 缓冲起始地址)
    if (HAL_UART_Transmit_DMA(&huart2, _tx_buf, len) != HAL_OK)
    {
        return false;
    }
    _dma_busy = true;
    return true;
}



#define RX_BUF_SIZE 32

uint8_t rx_buf[RX_BUF_SIZE];
uint8_t rx_ready = 0;

/**
 * @brief  启动 USART2 的 DMA 循环接收
 */
void UART2_DMA_StartReceive(void)
{
    // 如果前一次接收已停止，可以重新启动，否则第一次调用即可
    HAL_UART_Receive_DMA(&huart2, rx_buf, RX_BUF_SIZE);
}

/**
 * @brief  这个回调由 HAL_UART_IRQHandler 在 DMA 环境中自动触发
 *         当 DMA 完成整个缓冲区接收时调用
 */
static void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) 
    {
        rx_ready = true;
    }
}

/**
 * @brief  解析一帧 “set_angle=XXX” 字符串
 */
void Parse_SetAngle(float* set_angle)
{
    // 以 '\n' 结尾
    rx_buf[RX_BUF_SIZE-1] = '\n';
    // 找前缀
    const char *p = strstr((char*)rx_buf, "set_angle=");
    if (p) 
    {
        // 指向数字部分
        p += strlen("set_angle=");
        // 转 float
        float v = strtof(p, NULL);
        *set_angle = v;
    }
    memset(rx_buf, 0, RX_BUF_SIZE); // 清空接收缓冲区
}


