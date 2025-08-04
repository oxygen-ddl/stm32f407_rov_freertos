#ifndef SONAR_H
#define SONAR_H

#include "stm32f4xx_hal.h"

/* DMA 接收缓冲区大小（要大于一帧最大长度） */
#define UART1_DMA_BUF_SIZE  32

typedef struct {
    uint8_t  data[UART1_DMA_BUF_SIZE];
    uint16_t len;
} UART1_Msg_t;


extern float sonar_distance; // 存储测量的距离
extern uint8_t sonar_status; // 存储测量状态
extern float sonar_distance_deputy; // 存储副模组测量的距离

void sonar_init(void);
void UART1_IT_TASK(void);
void Sonar_Process_Task(void *pvParameters);




#endif // SONAR_H
