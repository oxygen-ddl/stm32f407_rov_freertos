#ifndef SONAR_H
#define SONAR_H

#include "stm32f4xx_hal.h"

/* DMA 接收缓冲区大小（要大于一帧最大长度） */
#define UART1_DMA_BUF_SIZE  32

typedef struct {
    uint8_t  data[UART1_DMA_BUF_SIZE];
    uint16_t len;
} UART1_Msg_t;


#endif // SONAR_H
