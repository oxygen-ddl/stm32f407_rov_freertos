#include "jy901p_uart.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"

float accx, accy, accz, angx, angy, angz, magx, magy, magz;
float roll, pitch, yaw, temperature_jy901p;
float roll_last, pitch_last, yaw_last;

uint8_t transmit_jy901_choose = 0; // 0-解析 1-透传
int turns_of_pitch, turns_of_roll, turns_of_yaw;
float roll_total, pitch_total, yaw_total, yaw_total_start;
uint8_t yaw_flag = 0; // 用于判断是否第一次接收 yaw 数据

extern UART_HandleTypeDef huart2; // 接收 jy901p 的串口
extern UART_HandleTypeDef huart5; // 输出到上位机的串口

static StreamBufferHandle_t jy901p_sbHandle;  // StreamBuffer 句柄，发送字节流给处理任务

/* DMA 接收缓冲区 */
static uint8_t dma_rx_buffer[JY901P_RX_BUFFER_SIZE];

void jy901p_process(char *p);

//UART IDLE+DMA 中断回调
//CubeMX 生成的 stm32f4xx_it.c 里，在 USART2_IRQHandler()
//调用 HAL_UART_IRQHandler(&huart2) 后，会触发此回调。

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    /* 把本次接收到的 Size 字节，一次性发给 StreamBuffer */
    xStreamBufferSendFromISR(jy901p_sbHandle, dma_rx_buffer, Size, &xHigherPriorityTaskWoken);

    /* 重新启动 DMA 循环接收（HAL 会自动重装 DMA 数据长度） */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, JY901P_RX_BUFFER_SIZE);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void jy901p_Task(void *pv)
{
    uint8_t local[JY901P_RX_BUFFER_SIZE];
    size_t len;

    for (;;)
    {
        /* 阻塞直到有字节可读，借助最大等待 */
        len = xStreamBufferReceive(jy901p_sbHandle, local, JY901P_RX_BUFFER_SIZE, portMAX_DELAY);
        //portMAX_DELAY 意味着：只要缓冲区里没有数据，调用会让出 CPU 并且 阻塞，直到有数据写入或超时（如果启用了超时，否则永远等）
        if (len > 0)
        {
            /* 加个结束符，然后调用既有的解析函数 */
            local[len] = '\0';
            if (transmit_jy901_choose == 0)
            {
                jy901p_process((char *)local);
            }
            else
            {
                /* 透传给 uart5 */
                HAL_UART_Transmit_DMA(&huart5, local, len);
            }
        }
    }
}

void jy901p_uart_init(void)
{
    /* 1) 创建一个足够大的 StreamBuffer（RX_DMA_SIZE*2 保证不会溢出） */
    jy901p_sbHandle = xStreamBufferCreate(JY901P_RX_BUFFER_SIZE * 2, 1);
    configASSERT(jy901p_sbHandle);

    /* 2) 启动 DMA+IDLE 接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, dma_rx_buffer, JY901P_RX_BUFFER_SIZE);

    /* 3) 创建解析任务 */
    xTaskCreate(
        jy901p_Task,
        "jy901p",
        256, /* 栈深度 256 words (1KB) */
        NULL,
        tskIDLE_PRIORITY + 2, /* 适当的优先级 */
        NULL);
}

void jy901p_process(char *p)
{
    char *p_temp = p;
    char data_temp[4][11]; // 欧拉角，角速度，加速度，磁场
    char data_check[4] = {0};
    while (*p_temp != '\0')
    {
        if (*p_temp == 0x55 && *(p_temp + 1) == 0x53) // 角度数据
        {
            for (uint8_t i = 0; i < 11; ++i)
            {
                data_temp[0][i] = *(p_temp + i);
            }
        }
        if (*p_temp == 0x55 && *(p_temp + 1) == 0x52) // 角速度数据
        {
            for (uint8_t i = 0; i < 11; ++i)
            {
                data_temp[1][i] = *(p_temp + i);
            }
        }
        if (*p_temp == 0x55 && *(p_temp + 1) == 0x51) // 加速度数据
        {
            for (uint8_t i = 0; i < 11; ++i)
            {
                data_temp[2][i] = *(p_temp + i);
            }
        }
        if (*p_temp == 0x55 && *(p_temp + 1) == 0x54) // 磁场数据
        {
            for (uint8_t i = 0; i < 11; ++i)
            {
                data_temp[3][i] = *(p_temp + i);
            }
        }
        ++p_temp;
    }

    for (uint8_t i = 0; i < 4; ++i)
    {
        for (uint8_t j = 0; j < 10; ++j)
        {
            data_check[i] += data_temp[i][j];
        }
    }
    if (data_temp[0][10] == data_check[0]) // 角度数据
    {
        roll_last = roll;
        pitch_last = pitch;
        yaw_last = yaw;
        /*由于安装方向问题*/
        pitch = (float)((data_temp[0][3] << 8) | data_temp[0][2]) * 180 / 32768;
        roll = (float)((data_temp[0][5] << 8) | data_temp[0][4]) * 180 / 32768;
        yaw = (float)((data_temp[0][7] << 8) | data_temp[0][6]) * 180 / 32768;
        temperature_jy901p = (float)(((data_temp[0][9] << 8) | data_temp[0][8])) / 100;
        if (roll - roll_last > 270)
        {
            turns_of_roll -= 1;
        }
        else if (roll - roll_last < -270)
        {
            turns_of_roll += 1;
        }
        roll_total = roll + turns_of_roll * 360;
        if (pitch - pitch_last > 270)
        {
            turns_of_pitch -= 1;
        }
        else if (pitch - pitch_last < -270)
        {
            turns_of_pitch += 1;
        }
        pitch_total = pitch + turns_of_pitch * 360;
        if (yaw - yaw_last > 270)
        {
            turns_of_yaw -= 1;
        }
        else if (yaw - yaw_last < -270)
        {
            turns_of_yaw += 1;
        }
        yaw_total = yaw + turns_of_yaw * 360;
        if (yaw_flag == 0)
        {
            yaw_flag = 1;
            yaw_total_start = yaw_total;
        }
    }
    // if (data_temp[1][10] == data_check[1]) // 角速度数据
    // {
    //     data_transfer_buffer.data2[0] = data_temp[1][3];
    //     data_transfer_buffer.data2[1] = data_temp[1][2];
    //     angx = (float)data_transfer_buffer.data1 * 2000.0 / 32768.0;

    //     data_transfer_buffer.data2[0] = data_temp[1][5];
    //     data_transfer_buffer.data2[1] = data_temp[1][4];
    //     angy = (float)data_transfer_buffer.data1 * 2000.0 / 32768.0;

    //     data_transfer_buffer.data2[0] = data_temp[1][7];
    //     data_transfer_buffer.data2[1] = data_temp[1][6];
    //     angz = (float)data_transfer_buffer.data1 * 2000.0 / 32768.0;
    //     //
    //     //        angx=(float)((((short)data_temp[1][3]<<8)|data_temp[1][2]))*2000.0/32768.0;
    //     //        angy=(float)((((short)data_temp[1][5]<<8)|data_temp[1][4]))*2000.0/32768.0;
    //     //        angz=(float)((((short)data_temp[1][7]<<8)|data_temp[1][6]))*2000.0/32768.0;
    // }
    // if (data_temp[2][10] == data_check[2]) // 加速度数据
    // {
    //     data_transfer_buffer.data2[0] = data_temp[2][3];
    //     data_transfer_buffer.data2[1] = data_temp[2][2];
    //     accx = (float)data_transfer_buffer.data1 * 16.0 * 9.8 / 32768.0;

    //     data_transfer_buffer.data2[0] = data_temp[2][5];
    //     data_transfer_buffer.data2[1] = data_temp[2][4];
    //     accy = (float)data_transfer_buffer.data1 * 16.0 * 9.8 / 32768.0;

    //     data_transfer_buffer.data2[0] = data_temp[2][7];
    //     data_transfer_buffer.data2[1] = data_temp[2][6];
    //     accz = (float)data_transfer_buffer.data1 * 16.0 * 9.8 / 32768.0;

    //     //        accx=(float)((((short)data_temp[2][3]<<8)|data_temp[2][2]))*16.0*9.8/32768.0;
    //     //        accy=(float)((((short)data_temp[2][5]<<8)|data_temp[2][4]))*16.0*9.8/32768.0;
    //     //        accz=(float)((((short)data_temp[2][7]<<8)|data_temp[2][6]))*16.0*9.8/32768.0;
    // }
    // if (data_temp[3][10] == data_check[3]) // 磁场数据
    // {
    //     data_transfer_buffer.data2[0] = data_temp[3][3];
    //     data_transfer_buffer.data2[1] = data_temp[3][2];
    //     magx = (float)data_transfer_buffer.data1;

    //     data_transfer_buffer.data2[0] = data_temp[3][5];
    //     data_transfer_buffer.data2[1] = data_temp[3][4];
    //     magy = (float)data_transfer_buffer.data1;

    //     data_transfer_buffer.data2[0] = data_temp[3][7];
    //     data_transfer_buffer.data2[1] = data_temp[3][6];
    //     magz = (float)data_transfer_buffer.data1;

    //     //        magx=(float)((((short)data_temp[3][3]<<8)|data_temp[3][2]));
    //     //        magy=(float)((((short)data_temp[3][5]<<8)|data_temp[3][4]));
    //     //        magz=(float)((((short)data_temp[3][7]<<8)|data_temp[3][6]));
    // }
}

// 解锁
void send_unlock_uart(void)
{
    uint8_t cmd[5] = {0xff, 0xaa, 0x69, 0x88, 0xb5};
    HAL_UART_Transmit_DMA(&huart2, cmd, sizeof(cmd));
}

// 保存配置
void send_save_uart(void)
{
    uint8_t cmd[5] = {0xff, 0xaa, 0x00, 0x00, 0x00};
    HAL_UART_Transmit_DMA(&huart2, cmd, sizeof(cmd));
}

// 恢复出厂设置并保存
void factory_data_reset_uart(void)
{
    send_unlock_uart();
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    uint8_t cmd[5] = {0xff, 0xaa, 0x00, 0x01, 0x00};
    HAL_UART_Transmit_DMA(&huart2, cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    send_save_uart();
}
// 设置校准

// 0-退出校准 1-加计校准 2-磁场校准 3-高度置零 4-z轴角度归零（6轴）
void set_calibration_uart(char data)
{
    send_unlock_uart();
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    uint8_t cmd[5] = {0xff, 0xaa, 0x01, data, 0x00};
    HAL_UART_Transmit_DMA(&huart2, cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    send_save_uart();
}
// 算法转换
// 0-9轴 1-6轴
void algorithm_conversion_uart(char data)
{
    send_unlock_uart();
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    uint8_t cmd[5] = {0xff, 0xaa, 0x24, data, 0x00};
    HAL_UART_Transmit_DMA(&huart2, cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    send_save_uart();
}
// 设置回传内容(见说明书) 00011110 00000000
void set_send_back_content_uart(char datah, char datal)
{
    send_unlock_uart();
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    uint8_t cmd[5] = {0xff, 0xaa, 0x02, datal, datah};
    HAL_UART_Transmit_DMA(&huart2, cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    send_save_uart();
}

// led设置
// 0-off 1-on
void led_set_uart(char set)
{
    uint8_t cmd[5] = {0xff, 0xaa, 0x1b, set, 0x00};
    send_unlock_uart();
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    HAL_UART_Transmit_DMA(&huart2, cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(5)); // 延时5毫秒
    // 经试验可以不加保存命令
    send_save_uart();
}
