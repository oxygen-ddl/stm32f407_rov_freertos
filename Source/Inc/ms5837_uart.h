#ifndef MS5837_UART_H
#define MS5837_UART_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @brief  ���� UART4 DMA + �����жϽ��գ��������¶�/���
 * @note   �� HAL ���ʼ���� MX_USART4_UART_Init() �����
 */
void Parser4_Init(void);

/* �����õ�������������λ���桢m */
extern float ms5837_temperature;
extern float ms5837_depth;
extern float ms5837_pressure;

/* DMA ���ջ�������С��Ҫ����һ֡��󳤶ȣ� */
#define UART4_DMA_BUF_SIZE  128
void parse_frame(const uint8_t *buf, int len);

#endif /* MS5837_UART_H */
