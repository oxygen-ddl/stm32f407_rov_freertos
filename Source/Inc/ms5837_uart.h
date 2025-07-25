#ifndef MS5837_UART_H
#define MS5837_UART_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @brief  启动 UART4 DMA + 空闲中断接收，并解析温度/深度
 * @note   在 HAL 库初始化并 MX_USART4_UART_Init() 后调用
 */
void Parser4_Init(void);
void UART4_IT_TASK(void);
void MS5837_ProcessTask(void *pvParameters);

/* 解析得到的物理量，单位：℃、m */
extern float ms5837_temperature;
extern float ms5837_depth;
extern float ms5837_pressure;

/* DMA 接收缓冲区大小（要大于一帧最大长度） */
#define UART4_DMA_BUF_SIZE  32

typedef struct {
    uint8_t  data[UART4_DMA_BUF_SIZE];
    uint16_t len;
} UART4_Msg_t;


#endif /* MS5837_UART_H */
