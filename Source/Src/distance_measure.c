#include "distance_measure.h"
#include "software_uart.h"
#include "FreeRTOS.h"
#include "task.h"

// 由超声波避障传感器测量的距离数据
float wave_distance[2] = {0.0f, 0.0f};


uint8_t ch_0;
uint8_t ch_1;

uint8_t muart2_cnt = 0;
uint8_t muart3_cnt = 0;
uint8_t muart2_rx_buf[5] = {0};
uint8_t muart3_rx_buf[5] = {0};
// 接收到字节处理函数
void Handle_Muart_Task(void *pvParameters)
{

   for (;;)
   {

       if (SoftUART_GetChar(&ch_0, 0))
       {
            //SoftUART_PutChar(0,ch_0);//回调测试
           if (ch_0 != 0xff)
           {
               muart2_cnt = 0;
               continue;
           }
           muart2_rx_buf[muart2_cnt++] = ch_0;
           if (muart2_cnt < 4)
           {
               continue;
           }
           else
           {
               uint8_t sum_2 = (muart2_rx_buf[0] + muart2_rx_buf[1] + muart2_rx_buf[2]) & 0xFF;
               if (sum_2 == muart2_rx_buf[3])
               {
                   wave_distance[1] = (muart2_rx_buf[1] << 8) | muart2_rx_buf[2];
               }
           }
       }
       if (SoftUART_GetChar(&ch_1, 1))
       {
           if (ch_1 != 0xff)
           {
               muart3_cnt = 0;
               continue;
           }
           muart3_rx_buf[muart3_cnt++] = ch_1;
           if (muart3_cnt < 4)
           {
               continue;
           }
           else
           {
               uint8_t sum_3 = (muart3_rx_buf[0] + muart3_rx_buf[1] + muart3_rx_buf[2]) & 0xFF;
               if (sum_3 == muart3_rx_buf[3])
               {
                   wave_distance[1] = (muart3_rx_buf[1] << 8) | muart2_rx_buf[2];
               }
           }
       }
       vTaskDelay(pdMS_TO_TICKS(10));

   }
}

void Trigger_Distance_Mearsure_Task(void *pvParameters)
{
   for (;;)
   {
       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
       vTaskDelay(pdMS_TO_TICKS(10));
       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
       vTaskDelay(pdMS_TO_TICKS(500));

       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
       vTaskDelay(pdMS_TO_TICKS(10));
       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
       vTaskDelay(pdMS_TO_TICKS(500));
   }
}
