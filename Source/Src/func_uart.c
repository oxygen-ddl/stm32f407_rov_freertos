#include "func_uart.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define MAX_FLOAT_COUNT 20
#define TX_BUF_SIZE (MAX_FLOAT_COUNT * 4 + 4)

static uint8_t _tx_buf[TX_BUF_SIZE];
static volatile bool _dma_busy = false;

/**
 * @brief UART DMA 发送完成回调，清除忙标志
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        _dma_busy = false;
    }
}

bool UART_SendFloats_DMA(uint8_t count, ...)
{
    if (_dma_busy || count == 0 || count > MAX_FLOAT_COUNT)
    {
        return false;
    }

    va_list args;
    va_start(args, count);

    uint8_t *p = _tx_buf;
    for (uint8_t i = 0; i < count; i++)
    {
        // float 在可变参数里会被提升为 double
        double tmp = va_arg(args, double);
        float v = (float)tmp;
        memcpy(p, &v, sizeof(v)); // 小端复制
        p += sizeof(v);
    }
    va_end(args);

    // 末尾追加帧尾 0x00,0x00,0x80,0x7F
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    size_t len = p - _tx_buf;
    if (HAL_UART_Transmit_DMA(&huart6, _tx_buf, len) != HAL_OK)
    {
        return false;
    }
    _dma_busy = true;
    return true;
}

bool UART_SendFloats_Blocking(uint8_t count, ...)
{
    if (count == 0 || count > MAX_FLOAT_COUNT)
    {
        return false;
    }

    va_list args;
    va_start(args, count);

    uint8_t *p = _tx_buf;
    for (uint8_t i = 0; i < count; i++)
    {
        // float 在可变参数里会被提升为 double
        double tmp = va_arg(args, double);
        float v = (float)tmp;
        memcpy(p, &v, sizeof(v)); // 小端复制
        p += sizeof(v);
    }
    va_end(args);

    // 末尾追加帧尾 0x00,0x00,0x80,0x7F
    *p++ = 0x00;
    *p++ = 0x00;
    *p++ = 0x80;
    *p++ = 0x7F;

    size_t len = p - _tx_buf;
    if (HAL_UART_Transmit(&huart6, _tx_buf, len, HAL_MAX_DELAY) != HAL_OK)
    {
        return false;
    }
    return true;
}



int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart6, &ch, 1, 0xffff);
  return ch;
}

/*************************************变量观测**************************************/
#include "ms5837_iic.h"
#include "cmsis_os.h"

static osThreadId_t view_variables_TaskHandle;//是 CMSIS-RTOS2 （在 STM32CubeMX/FreeRTOS V10+ 中默认启用）的创建线程接口
//xTaskCreate()是老版本任务创建
void view_variables_Task(void *argument)
{
    for (;;)
    {
        // 发送深度和压力数据
        UART_SendFloats_DMA(2, ms5837_depth, ms5837_pressure);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void view_variables_StartTask(void)
{
    const osThreadAttr_t attr = {
        .name = "view_variables",
        .stack_size = 256,
        .priority = osPriorityLow2,
    };
    view_variables_TaskHandle = osThreadNew(view_variables_Task, NULL, &attr);//调用 MS5837_Task 作为任务入口函数；传入 NULL 作为该任务的参数；用 &attr 里指定的名字、堆栈大小和优先级来创建线程；

}

