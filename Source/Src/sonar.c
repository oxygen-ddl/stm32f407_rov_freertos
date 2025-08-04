#include "sonar.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

UART1_Msg_t uart1_msg;
uint8_t uart1_it_flag;

float sonar_distance = 0.0f; // 存储测量的距离
float sonar_distance_deputy = 0.0f; // 存储副模组测量的距离
uint8_t sonar_status = 0; // 存储测量状态


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
    static uint8_t sonar_cnt = 0;
    for (;;)
    {
        sonar_cnt++;
        if (sonar_cnt >= 10) // 每10次循环处理一次
        {
            sonar_cnt = 0;
            //ff 01 08 01 06 02 00 00 01 49 b2 fe 
            // uint8_t test_data[10] = {0xFF, 0x01, 0x08, 0x01, 0x06, 0x02, 0x00, 0x00, 0x01, 0x49};
            // HAL_UART_Transmit_DMA(&huart1, test_data, 10); // 发送测试数据
            
            // 发送定位命令
            // uint8_t sonar_send_data[7] = {0x01,0x6C,0x07,0x01,0xFF,0x01,0x75};
            // HAL_UART_Transmit_DMA(&huart1, sonar_send_data, 7);
        }
        if (uart1_it_flag == 1)
        {
            uart1_it_flag = 0;
            // 处理接收到的数据
            if (uart1_msg.data[1]  == 0x8C)
            {
                //数据为01 8C 0B 00 00 06 00 7E 01 20 3D
                //主机发送0X6C功能码控制发射端进行定位，当发射端定位完成后会回应数据包给主机，其中0X0006表示当前定位距离6cm，0X007e表示飞行时间126us，0X0120表示温度28.8摄氏度
                // 这里可以添加对接收到数据的处理逻辑
                if (uart1_msg.len >= 7) // 确保数据长度足够
                {
                    
                    uint16_t distance = (uart1_msg.data[3] << 8) | uart1_msg.data[4]; // 获取距离数据
                    sonar_distance = (float)distance * 1.0f; // 单位cm
                    sonar_status = 1; // 设置状态为已测量
                }
            }
            if (uart1_msg.data[1] == 0x8D)
            {
                //数据为：018D0B 00010005 00070072 011932
                //说明：01表示主模组先接收到基站回复的定位数据，0X0005表示主模组到基站的距离为50cm，0X0007表示副模组到基站的距离位7cm，0x0072表示主模组和副模组定位时间差为114us，0X0119表示28.1摄氏度。
                uint16_t distance = (uart1_msg.data[3] << 8) | uart1_msg.data[4]; // 获取主模组距离数据
                sonar_distance = (float)distance * 1.0f; // 单位cm
                sonar_distance_deputy = (float)((uart1_msg.data[5] << 8) | uart1_msg.data[6]) * 1.0f; // 获取副模组距离数据
                sonar_status = 2; // 设置状态为已测量
                
            }
            

        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 延时100毫秒
    }
}

