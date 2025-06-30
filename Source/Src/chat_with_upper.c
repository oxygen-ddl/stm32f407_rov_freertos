#include "chat_with_upper.h"
#include "jy901p_uart.h"
#include "transmit_power_board.h"
#include "mx5837_iic.h"
#include "distance_measure.h"
#include "ath20_bmp280.h"
#include "move_control.h"

#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "semphr.h"
extern float accx, accy, accz, angx, angy, angz, magx, magy, magz; // 来自jy901p_uart.h
extern float roll, pitch, yaw, temperature_jy901p;                 // 来自jy901p_uart.h

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
    uint8_t buf[S_PACKAGE_LEN]={0}; 

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
        conv[6].value = pitch;
        conv[7].value = yaw;
        conv[8].value = roll;

        float_pack(9, 36, TX_StartBit_ACC, buf, conv);
        HAL_UART_Transmit(&huart3, buf, 40,100); // 发送数据包到上位机
        break;

    case (TX_StartBit_DEP):
        conv[0].value = mx5837_depth;
        conv[1].value = mx5837_depth;
        conv[2].value = wave_distance[0];
        conv[3].value = wave_distance[1];

        float_pack(4, 16, TX_StartBit_DEP, buf, conv);
        HAL_UART_Transmit(&huart3, buf,20,100); // 发送数据包到上位机
        break;

    case (TX_StartBit_TEM_WET):
        conv[0].value = bmp280_pressure;
        conv[1].value = bmp280_temperature;
        conv[2].value = ath20_humidity;
        conv[3].value = ath20_tremperature;

        float_pack(4, 16, TX_StartBit_TEM_WET, buf, conv);
        HAL_UART_Transmit(&huart3, buf,20,100); // 发送数据包到上位机
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
        HAL_UART_Transmit(&huart3, buf,40,100); // 发送数据包到上位机
        break;

    default:
        break;
    }
}

/* 专门的发送任务，每 200ms 调一次 Send_allpack */
static void SendAllPack_Task(void *pvParameters)
{
    (void)pvParameters;
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

/**
 * @brief  创建并启动“发送整帧”任务
 * @note   调用时机：一般放在 main() 里，或者 FreeRTOS 初始化后立即调用
 */
void SendAllPack_TaskInit(void)
{
    BaseType_t ret;

    ret = xTaskCreate(
        SendAllPack_Task,     /* 任务入口函数 */
        "SendPack",           /* 任务名字 */
        512,                  /* 栈深度（字） */
        NULL,                 /* 传给任务的参数 */
        tskIDLE_PRIORITY + 3, /* 任务优先级 */
        NULL                  /* 不需要任务句柄就传 NULL */
    );
    configASSERT(ret == pdPASS);
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

/* 一帧最大负载长度 */
#define PARSER_MAX_PAYLOAD 128

/* 状态机状态 */
typedef enum
{
    S_SYNC,
    S_ID,
    S_LEN,
    S_BODY,
    S_CHK
} ParserState_t;

/* “帧”结构，用队列传给处理任务 */
typedef struct
{
    uint8_t id;
    uint8_t len;
    uint8_t payload[PARSER_MAX_PAYLOAD];
} Frame_t;

/* FreeRTOS 对象 */
static QueueHandle_t xFrameQueue;

/* 状态机上下文（static 只在本文件） */
static ParserState_t state = S_SYNC;
static uint8_t cur_id;
static uint8_t cur_len;
static uint8_t body_idx;
static uint8_t checksum;
static uint8_t body_buf[PARSER_MAX_PAYLOAD];

/* ISR 或 Rx 回调里，每收到一个字节就喂给它 */
void parser_feed(uint8_t ch)
{
    switch (state)
    {
    case S_SYNC:
        if (ch == 0xFF)
            state = S_ID;
        break;

    case S_ID:
        cur_id = ch;
        checksum = ch; // 从 ID 开始累加
        state = S_LEN;
        break;

    case S_LEN:
        cur_len = ch;
        checksum += ch;
        if (cur_len > PARSER_MAX_PAYLOAD)
        {
            /* 长度非法，丢弃 */
            state = S_SYNC;
        }
        else
        {
            body_idx = 0;
            state = (cur_len > 0 ? S_BODY : S_CHK);
        }
        break;

    case S_BODY:
        body_buf[body_idx++] = ch;
        checksum += ch;
        if (body_idx >= cur_len)
            state = S_CHK;
        break;

    case S_CHK:
        if ((uint8_t)(checksum & 0xFF) == ch)
        {
            /* 校验通过，把一帧投递到队列 */
            Frame_t frame;
            frame.id = cur_id;
            frame.len = cur_len;
            memcpy(frame.payload, body_buf, cur_len);
            xQueueSendFromISR(xFrameQueue, &frame, NULL);
        }
        /* 无论成功或失败，都重新回到 SYNC */
        state = S_SYNC;
        break;
    }
}

/* 真正做 unpack 的任务 */
static void vParserTask(void *pv)
{
    Frame_t frame;
    for (;;)
    {
        /* 阻塞直到有一帧到来 */
        if (xQueueReceive(xFrameQueue, &frame, portMAX_DELAY) == pdTRUE)
        {
            switch (frame.id)
            {
            case RX_StartBit_Handle_basic: // 0xB1 字节数为28，6*4+4
                handle.go = int16_convert(frame.payload, 0, 1);
                handle.move = int16_convert(frame.payload, 2, 3);
                handle.yaw = int16_convert(frame.payload, 4, 5);
                handle.pitch = int16_convert(frame.payload, 6, 7);
                handle.roll = int16_convert(frame.payload, 8, 9);
                handle.up = int16_convert(frame.payload, 10, 11);
                /* 更新全局运动量 */
                go_forward_back = handle.go;
                go_left_right = handle.move;
                turn_left_right = handle.yaw;
                move_pitch = handle.pitch;
                move_row = handle.roll;
                go_up_down = handle.up;
                break;

            case RX_StartBit_Handle_light:
                light.on = uint16_convert(frame.payload,0,1);
            case RX_StartBit_Handle_func_lockangle:
                mode.lockangle = uint16_convert(frame.payload,0,1);
                break;
            case RX_StartBit_Handle_func_start_move:
                mode.autotrip = uint16_convert(frame.payload,0,1);
                break;
            case RX_StartBit_Handle_func_autotrip:
                mode.autovertical = uint16_convert(frame.payload,0,1);
                break;
            case RX_StartBit_Handle_func_electromagnet:
                mode.electromagnet = uint16_convert(frame.payload,0,1);
                break;
            case RX_StartBit_Handle_func_push_rod:
                mode.push_rod = uint16_convert(frame.payload,0,1);
                break;
            case RX_StartBit_Handle_func_autorolling:
                mode.autorolling = uint16_convert(frame.payload,0,1);
                break;

            case RX_StartBit_PID:
            {
                // 1) 解析 8 路输入 PID
                for (uint8_t u = 0; u < 8; ++u)
                {
                    pid_in_parameter[u].parameter.kp = float_convert(frame.payload, 0, 3);
                    pid_in_parameter[u].parameter.ki = float_convert(frame.payload, 4, 7);
                    pid_in_parameter[u].parameter.kd = float_convert(frame.payload, 8, 11);
                }
                // 2) 解析 6 路输出 PID
                for (uint8_t i = 0; i < 6; ++i)
                {

                    pid_out_parameter[i].parameter.kp = float_convert(frame.payload, i * 12 + 0, i * 12 + 3);
                    pid_out_parameter[i].parameter.ki = float_convert(frame.payload, i * 12 + 4, i * 12 + 7);
                    pid_out_parameter[i].parameter.kd = float_convert(frame.payload, i * 12 + 8, i * 12 + 11);
                }
                // 3) 解析 深度 PID
                pid_depth.kp = float_convert(frame.payload, 84, 90);
                pid_depth.ki = float_convert(frame.payload, 91, 94);
                pid_depth.kd = float_convert(frame.payload, 95, 98);
            }

            break;

            default:
                break;
            }
        }
    }
}
static uint8_t rxByte;
/* 在 reeRTOS init 里调用一次 */
void Parser_Init(void)
{
    // 帧队列，深度 5 帧
    xFrameQueue = xQueueCreate(5, sizeof(Frame_t));
    // 启动第一次接收中断
    HAL_UART_Receive_IT(&huart3,&rxByte, 1);
    // Parser 任务 
    xTaskCreate(vParserTask,
                "Parser",
                512, NULL,
                tskIDLE_PRIORITY + 6,
                NULL);
}

/* 例：HAL USART Rx 完成中断里喂字节并触发解析 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    rxByte = (uint8_t)(huart->Instance->DR & 0xFF);

    if (huart == &huart3)
    {
        parser_feed(rxByte);
        /* 重启下一次中断接收 */
        HAL_UART_Receive_IT(&huart3, &rxByte, 1);
    }
}
