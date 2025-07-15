#include "distance_measure.h"
#include "software_uart.h"
#include "FreeRTOS.h"
#include "task.h"

// 由超声波避障传感器测量的距离数据
float wave_distance[2] = {0.0f, 0.0f};

SoftUART_Handle_t muart_2, muart_3;

void soft_uart_init(void)
{

   SoftUART_Config_t cfg2 = {
       .rxPort = GPIOC, .rxPin = GPIO_PIN_0, .txPort = GPIOC, .txPin = GPIO_PIN_1, .baudrate = 115200};
   SoftUART_Config_t cfg3 = {
       .rxPort = GPIOC, .rxPin = GPIO_PIN_2, .txPort = GPIOC, .txPin = GPIO_PIN_3, .baudrate = 115200};

   // 使用软件串口，节省资源消耗，基本上不使用tx发送功能，主要就是接收功能
   muart_2 = SoftUART_Init(&cfg2);
   muart_3 = SoftUART_Init(&cfg3);
}

uint8_t c = 0;
uint8_t muart2_cnt = 0;
uint8_t muart3_cnt = 0;
uint8_t muart2_rx_buf[5] = {0};
uint8_t muart3_rx_buf[5] = {0};
// 接收到字节处理函数
void Handle_Muart_Task(void *pvParameters)
{

   for (;;)
   {
       if (SoftUART_ReceiveByte(muart_2, &c, 100))
       {
           if (c != 0xff)
           {
               muart2_cnt = 0;
               continue;
           }
           muart2_rx_buf[muart2_cnt++] = c;
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
       if (SoftUART_ReceiveByte(muart_3, &c, 100))
       {
           if (c != 0xff)
           {
               muart3_cnt = 0;
               continue;
           }
           muart3_rx_buf[muart3_cnt++] = c;
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
