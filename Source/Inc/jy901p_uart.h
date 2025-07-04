#ifndef JY901P_UART_H
#define JY901P_UART_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "string.h"

extern float accx, accy, accz, angx, angy, angz, magx, magy, magz;
extern float roll,pitch,yaw,temperature_jy901s;
extern float roll_last, pitch_last, yaw_last;
extern float roll_total, pitch_total, yaw_total, yaw_total_start;
extern float roll_start, pitch_start, yaw_start; // 用于记录初始角度
extern int turns_of_pitch, turns_of_roll, turns_of_yaw;

      	
/** 接收缓冲区长度 */
#define JY901_RX_BUFFER_SIZE  64
/** 队列最大深度 */
#define JY901_RX_QUEUE_LEN    5

/** 队列消息：一帧数据及其长度 */
typedef struct {
    uint8_t  data[JY901_RX_BUFFER_SIZE];
    uint16_t len;
} JY901_Msg_t;


extern uint8_t    jy901_rx_buffer[JY901_RX_BUFFER_SIZE];
extern QueueHandle_t jy901Queue;
/**
 * @brief  初始化 JY901 串口（USART2）DMA + IDLE + Queue，创建处理任务
 * @note   在 HAL 库与 USART2 初始化完成后调用
 */
void JY901_UART_Init(void);
void JY901_ProcessTask(void *pvParameters);

#endif /* JY901P_UAYRT_H */
