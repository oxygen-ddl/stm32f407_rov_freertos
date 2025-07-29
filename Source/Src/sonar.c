#include "sonar.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

UART1_Msg_t uart1_msg;
uint8_t uart1_it_flag;


void sonar_init(void)
{

    // 启动 DMA 接收
    HAL_UART_Receive_DMA(&huart1, uart1_msg.data, UART1_DMA_BUF_SIZE);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

void UART1_IT_TASK(void)
{
    // 空闲检测
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        // 停 DMA，算长度
        HAL_UART_DMAStop(&huart1);
        uint16_t len = UART1_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);

        if (uart1_it_flag == 0)
        {
            uart1_msg.len = len;
            uart1_it_flag = 1;
        }
        // 重启 DMA
        HAL_UART_Receive_DMA(&huart1, uart1_msg.data, UART1_DMA_BUF_SIZE);
    }
}

void Sonar_Process_Task(void *pvParameters)
{
    for (;;)
    {
        
        if (uart1_it_flag == 1)
        {

            uart1_it_flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 延时100毫秒
    }
}

