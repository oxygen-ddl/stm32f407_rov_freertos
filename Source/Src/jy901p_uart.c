#include "jy901p_uart.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
#include "cmsis_os.h"
#include "queue.h"
#include "string.h"
float accx, accy, accz, angx, angy, angz, magx, magy, magz;
float roll, pitch, yaw, temperature_jy901p;
float roll_last, pitch_last, yaw_last;
float roll_total, pitch_total, yaw_total, yaw_total_start;
float roll_start, pitch_start, yaw_start; // 用于记录初始角度


int turns_of_pitch, turns_of_roll, turns_of_yaw;
uint8_t yaw_flag = 0; // 用于判断是否第一次接收 yaw 数据
uint8_t start_set = 0;

/*—————— 内部类型 ——————*/
typedef struct
{
    int16_t a[3], T;
} SAcc;
typedef struct
{
    int16_t w[3], T;
} SGyro;
typedef struct
{
    int16_t Angle[3], T;
} SAngle;
typedef struct
{
    int16_t h[3], T;
} SMag;

/*—————— 模块静态数据 ——————*/
uint8_t jy901_rx_buffer[JY901_RX_BUFFER_SIZE];
QueueHandle_t jy901Queue;

/*——— 函数原型 ———*/
void JY901_ProcessTask(void *pvParameters);
void JY901_ParseData(const uint8_t *buf, uint16_t len);

/*—————— 公有接口 ——————*/
void JY901_UART_Init(void)
{
    /* 1) 创建队列 */
    jy901Queue = xQueueCreate(JY901_RX_QUEUE_LEN, sizeof(JY901_Msg_t));
    if (jy901Queue == NULL)
    {
        Error_Handler();
    }

    /* 3) 启动 DMA 接收 */
    HAL_UART_Receive_DMA(&huart2, jy901_rx_buffer, JY901_RX_BUFFER_SIZE);
    /* 4) 使能空闲中断 */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
}


/*—————— 静态实现 ——————*/

/**
 * @brief  后台任务：从队列读取一帧数据并调用解析函数
 */
void JY901_ProcessTask(void *pvParameters)
{
    JY901_Msg_t msg;
    for (;;)
    {
        if (xQueueReceive(jy901Queue, &msg, portMAX_DELAY) == pdPASS)
        {
            JY901_ParseData(msg.data, msg.len);
        }
    }
}

uint16_t version = 0; // JY901 版本号
/**
 * @brief  将原始字节流逐字节传给 CopeSerial2Data，
 *         最后更新全局变量 accx/angx/roll…
 */
/**
 * @brief  按协议解析一段原始字节流
 * @param  buf 原始接收缓冲区
 * @param  len 缓冲区长度
 */
void JY901_ParseData(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    while (i + 10 < len)
    {
        // 找到包头 0x55，并且包类型在 0x51~0x54
        if (buf[i] == 0x55 && (buf[i+1] >= 0x51 && buf[i+1] <= 0x54))
        {
            // 计算校验和：sum(buf[i] 到 buf[i+9])
            uint8_t sum = 0;
            for (uint8_t k = 0; k < 10; ++k)
            {
                sum += buf[i + k];
            }
            // 校验和通过
            if (sum == buf[i + 10])
            {
                // 指针 p 指向本包首地址
                const uint8_t *p = &buf[i];
                int16_t rawL, rawH;

                switch (p[1])
                {
                case 0x51:
                    // 加速度包
                    // Ax = ((AxH<<8)|AxL)/32768*16*g
                    rawL =  (int16_t)((p[3] << 8) | p[2]);
                    accx  = (float)rawL / 32768.0f * 16.0f * 9.8f;
                    rawL =  (int16_t)((p[5] << 8) | p[4]);
                    accy  = (float)rawL / 32768.0f * 16.0f * 9.8f;
                    rawL =  (int16_t)((p[7] << 8) | p[6]);
                    accz  = (float)rawL / 32768.0f * 16.0f * 9.8f;
                    // 温度 = ((TH<<8)|TL)/100 ℃
                    rawL =  (int16_t)((p[9] << 8) | p[8]);
                    temperature_jy901p = (float)rawL / 100.0f;
                    break;

                case 0x52:
                    // 角速度包
                    rawL =  (int16_t)((p[3] << 8) | p[2]);
                    angx  = (float)rawL / 32768.0f * 2000.0f;
                    rawL =  (int16_t)((p[5] << 8) | p[4]);
                    angy  = (float)rawL / 32768.0f * 2000.0f;
                    rawL =  (int16_t)((p[7] << 8) | p[6]);
                    angz  = (float)rawL / 32768.0f * 2000.0f;
                    // （如果需要读电压，可按协议解析 p[8]/p[9]）
                    break;

                case 0x53:
                    // 角度包
                    rawL =  (int16_t)((p[3] << 8) | p[2]);
                    roll  = (float)rawL / 32768.0f * 180.0f;
                    rawL =  (int16_t)((p[5] << 8) | p[4]);
                    pitch = (float)rawL / 32768.0f * 180.0f;
                    rawL =  (int16_t)((p[7] << 8) | p[6]);
                    yaw = (float)rawL / 32768.0f * 180.0f;
                    // 版本号=(VH<<8)|VL
                    rawH = p[9];
                    rawL = p[8];
                    version = (rawH << 8) | rawL;
                    // roll = 360-roll;
                    // yaw = 360-yaw;
                    // pitch -= pitch_start;
                    // roll -= roll_start;
                    //! pitch于roll 角色对调
                    yaw  = -yaw;

                    if (roll - roll_last > 270)
                    {
                        turns_of_roll += 1;
                    }
                    else if (roll - roll_last < -270)
                    {
                        turns_of_roll -= 1;
                    }
                    roll_total = roll + turns_of_roll * 360;


                    if (pitch - pitch_last > 270)
                    {
                        turns_of_pitch += 1;
                    }
                    else if (pitch - pitch_last < -270)
                    {
                        turns_of_pitch -= 1;
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

                    roll_last = roll;
                    pitch_last = pitch;
                    yaw_last = yaw;

                    if (yaw_flag == 0)
                    {
                        yaw_flag = 1;
                        yaw_total_start = yaw_total;
                    }

                    break;

                case 0x54:
                    // 磁场包
                    rawL =  (int16_t)((p[3] << 8) | p[2]);
                    magx  = (float)rawL;
                    rawL =  (int16_t)((p[5] << 8) | p[4]);
                    magy  = (float)rawL;
                    rawL =  (int16_t)((p[7] << 8) | p[6]);
                    magz  = (float)rawL;
                    // 温度同 0x51 包
                    rawL =  (int16_t)((p[9] << 8) | p[8]);
                    temperature_jy901p = (float)rawL / 100.0f;
                    break;

                default:
                    break;
                }
                if (start_set == 0)
                {
                    // 第一次接收数据，记录初始角度
                    roll_start = roll;
                    pitch_start = pitch;
                    yaw_start = yaw;
                    start_set = 1;
                }
                
            }
            // 跳过这个子包
            i += 11;
            continue;
        }
        // 非包头或校验失败，继续下一字节搜索
        ++i;
    }
}

