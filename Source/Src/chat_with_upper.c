#include "chat_with_upper.h"
#include "jy901p_uart.h"
#include "transmit_power_board.h"
#include "ms5837_uart.h"
#include "distance_measure.h"
#include "ath20_bmp280.h"
#include "move_control.h"
#include "shtc3.h"


#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
extern float accx, accy, accz, angx, angy, angz, magx, magy, magz; // 来自jy901p_uart.h
extern float roll, pitch, yaw, temperature_jy901p;                 // 来自jy901p_uart.h

uint8_t parse_rx_buf[PARSER_DMA_BUF_SIZE];
uint8_t uart3_it_flag = 0;   // 0表示没有新数据，1表示有新数据的到来
Parse_Msg_t uart3_msg; // 作为全局变量，在中断与处理函数中传递接收到的数据以及长度


// 发送
typedef union
{
    float value;
    unsigned char data[4];
} U_FloatData;

typedef union
{
    uint16_t value;
    unsigned char data[2];
} U_uint16Data;

// 接收
typedef union
{
    float value;
    unsigned char rxbuf[4]; // 使用其他数据类型，修改此处并在下一行设置相应的字节数即可
} R_FloatData;

typedef union
{
    uint16_t value;
    unsigned char rxbuf[2];
} R_uint16Data;

typedef union
{
    int16_t value;
    unsigned char rxbuf[2];
} R_int16Data;

static void float_pack(int num_data, int length, uint8_t data_ID, uint8_t buf[], U_FloatData conv[])
{
    int i, j, k;
    uint8_t buff = 0;

    buf[1] = data_ID; // 数据ID
    buf[2] = length;  // 数据长度

    for (j = 0; j < num_data; j++)
    {
        for (i = 3; i < 7; i++)
        {
            buf[i + j * 4] = conv[j].data[i - 3];
        }
    }

    for (k = 1; k < length + 3; k++)
    {
        buff += buf[k]; // 和校验
    }
    buf[length + 3] = buff & 0xFF;
}

void data_packup(uint8_t startbit)
{
    uint8_t buf[S_PACKAGE_LEN] = {0};

    // 用于拆分数据
    U_FloatData conv[10];
    buf[0] = 0xFF; // 帧头
    switch (startbit)
    {
    case (TX_StartBit_ACC):
        conv[0].value = accx;
        conv[1].value = accy;
        conv[2].value = accz;
        conv[3].value = angx;
        conv[4].value = angy;
        conv[5].value = angz;
        conv[6].value = roll;
        conv[7].value = pitch;
        conv[8].value = yaw;

        float_pack(9, 36, TX_StartBit_ACC, buf, conv);
        HAL_UART_Transmit(&huart3, buf, 40, 100); // 发送数据包到上位机
        break;

    case (TX_StartBit_DEP):
        conv[0].value = ms5837_depth;
        conv[1].value = ms5837_pressure;
        conv[2].value = wave_distance[0];
        conv[3].value = wave_distance[1];

        float_pack(4, 16, TX_StartBit_DEP, buf, conv);
        HAL_UART_Transmit(&huart3, buf, 20, 100); // 发送数据包到上位机
        break;

    case (TX_StartBit_TEM_WET):
        conv[0].value = sthc3_temperature;//主控温度
        conv[1].value = sthc3_humidity;//主控湿度
        conv[2].value = (float)temperature_power_board;//电源温度
        conv[3].value = (float)RH_power_board;//电源湿度

        float_pack(4, 16, TX_StartBit_TEM_WET, buf, conv);
        HAL_UART_Transmit(&huart3, buf, 20, 100); // 发送数据包到上位机
        break;

    case (TX_StartBit_PWM1_8_power):
        conv[0].value = (float)current_adc_data[0];
        conv[1].value = (float)current_adc_data[1];
        conv[2].value = (float)current_adc_data[2];
        conv[3].value = (float)current_adc_data[3];
        conv[4].value = (float)current_adc_data[4];
        conv[5].value = (float)current_adc_data[5];
        conv[6].value = (float)current_adc_data[6];
        conv[7].value = (float)current_adc_data[7];
        conv[8].value = (float)current_adc_data[8];

        float_pack(9, 36, TX_StartBit_PWM1_8_power, buf, conv);
        HAL_UART_Transmit(&huart3, buf, 40, 100); // 发送数据包到上位机
        break;

    default:
        break;
    }
}

/* 专门的发送任务，每 200ms 调一次 Send_allpack */
void SendAllPack_Task(void *pvParameters)
{
    for (;;)
    {
        data_packup(TX_StartBit_ACC);
        vTaskDelay(pdMS_TO_TICKS(50));

        data_packup(TX_StartBit_DEP);
        vTaskDelay(pdMS_TO_TICKS(50));

        data_packup(TX_StartBit_TEM_WET);
        vTaskDelay(pdMS_TO_TICKS(50));

        data_packup(TX_StartBit_PWM1_8_power);
        vTaskDelay(pdMS_TO_TICKS(50));

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**********************接收函数部分*************************/

int16_t int16_convert(uint8_t a[], int start, int end)
{
    R_int16Data convert;

    for (int i = start; i <= end; i++)
    {
        convert.rxbuf[i - start] = a[i];
    }

    return convert.value;
}

uint16_t uint16_convert(uint8_t a[], int start, int end)
{
    R_uint16Data convert;
    for (int i = start; i <= end; i++)
    {
        convert.rxbuf[i - start] = a[i];
    }

    return convert.value;
}

float float_convert(uint8_t a[], int start, int end)
{
    R_FloatData convert;
    for (int i = start; i <= end; i++)
    {
        convert.rxbuf[i - start] = a[i];
    }

    return convert.value;
}

/*************************************************************************** */


#include "queue.h"
#include "string.h"     // memcpy

/* 真正做 unpack 的任务 */
/* 解析并处理一帧数据 */
void parsePacket(uint8_t *buf, uint16_t len)
{
    if (len < 4) return;              // 至少要有头、ID、LEN、CHK
    if (buf[0] != 0xFF) return;       // 帧头不对
    uint8_t id  = buf[1];
    uint8_t N   = buf[2];
    if (3 + N + 1 > len) return;      // 数据不全，丢弃
    // 计算校验和
    uint8_t sum = 0;
    for (int i = 0; i < 3 + N; i++)
        sum += buf[i];
    if (sum != buf[3 + N]) return;    // 校验失败

    uint8_t *p = buf + 3;             // 指向载荷起始
    // 根据 ID 一次性解析
    switch (id)
    {
    case RX_StartBit_Handle_basic: // 0xB1, N == 12
        handle.go    = (int16_t)((p[1]<<8)|p[0]);
        handle.move  = (int16_t)((p[3]<<8)|p[2]);
        handle.up    = (int16_t)((p[5]<<8)|p[4]);
        handle.yaw   = (int16_t)((p[7]<<8)|p[6]);
        handle.pitch = (int16_t)((p[9]<<8)|p[8]);
        handle.roll  = (int16_t)((p[11]<<8)|p[10]);
        // 更新外部变量
        go_forward = handle.go;
        go_left    = handle.move;
        go_up      = handle.up;
        move_yaw   = handle.yaw;
        move_pitch = handle.pitch;
        move_roll  = handle.roll;
        break;

    case RX_StartBit_Handle_light: 
        light.on = (int16_t)((p[1]<<8)|p[0]);
        mode.lockangle = (int16_t)((p[3]<<8)|p[2]);
        mode.unknow = (int16_t)((p[5]<<8)|p[4]);
        mode.autotrip = (int16_t)((p[7]<<8)|p[6]);
        mode.defogging = (int16_t)((p[9]<<8)|p[8]);
        mode.electromagnet = (int16_t)((p[11]<<8)|p[10]);
        mode.push_rod = (int16_t)((p[13]<<8)|p[12]);
        mode.autovertical = (int16_t)((p[15]<<8)|p[14]);

        break;

    case RX_StartBit_PID: // 0xC1, N == 6*2*2 + ... 根据协议
        // 示例：第一个 float 占 4 字节，依次解析...
        for (uint8_t i = 0; i < 8; i++)
        {
            uint32_t tmp = ((uint32_t)p[i*4+3]<<24) |
                           ((uint32_t)p[i*4+2]<<16) |
                           ((uint32_t)p[i*4+1]<<8 ) |
                           ((uint32_t)p[i*4+0]<<0 );
            pid_in_parameter[i].parameter.kp = *(float*)&tmp;
            // 类似解析 ki、kd...
        }
        break;

    // ... 其它 ID 按需添加 ...

    default:
        break;
    }
}

/* 在 freeRTOS init 里调用一次 */
void Parser_Init(void)
{
    
    //启动dma接收
    HAL_UART_Receive_DMA(&huart3, parse_rx_buf, PARSER_DMA_BUF_SIZE);

    /* 3) 使能空闲中断 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}


void Parse_Task(void *pvParameters)
{
    for (;;)
    {
        if (uart3_it_flag == 1)
        {
            parsePacket(uart3_msg.data,uart3_msg.len);
            uart3_it_flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void UART3_IT_TASK(void)
{
    // 空闲检测
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        // 停 DMA，算长度
        HAL_UART_DMAStop(&huart3);
        uint16_t len = PARSER_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        if (uart3_it_flag == 0)
        {
            uart3_msg.len = len;
            memcpy(uart3_msg.data, parse_rx_buf, len);
            uart3_it_flag = 1;
        }
        // 重启 DMA 
        HAL_UART_Receive_DMA(&huart3, parse_rx_buf, PARSER_DMA_BUF_SIZE);
    }
}

