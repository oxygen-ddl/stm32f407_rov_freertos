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
#include "move_control.h"
#include "jy901p_uart.h"
#include "chat_with_upper.h"
#include "ms5837_uart.h"
#include "ath20_bmp280.h"
#include "transmit_power_board.h"

void view_variables_Task(void *argument)
{
    for (;;)
    {
        //UART_SendFloats_DMA(3, (float)roll, (float)pitch,(float)yaw);
        //UART_SendFloats_DMA(9, (float)accx, (float)accy, (float)accz,  (float)angx, (float)angy, (float)angz, (float)roll, (float)pitch, (float)yaw);
        //UART_SendFloats_DMA(6,(float)go_forward, (float)go_left,(float)go_up,(float) move_yaw,(float) move_pitch, (float)move_roll);
        //UART_SendFloats_DMA(2,ms5837_temperature, ms5837_depth);
        //UART_SendFloats_DMA(4, (float)bmp280_pressure, (float)bmp280_temperature, (float)ath20_humidity, (float)ath20_temperature);
        //UART_SendFloats_DMA(3,pitch_total,roll_total,yaw_total);
        //UART_SendFloats_DMA(4,(float)current_adc_data[1],(float)current_adc_data[2],(float)current_adc_data[4],(float)current_adc_data[7]);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


