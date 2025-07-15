#ifndef SOFTWARE_UART_H
#define SOFTWARE_UART_H

#include "stm32f4xx_hal.h"
#include <string.h>

#define SOFTUART_MAX_CHANNELS  4
#define SU_RX_QUEUE_LEN       16
#define SU_TX_BUF_LEN         16

typedef uint8_t SoftUART_Handle_t;

/** 软件串口配置 */
typedef struct {
    GPIO_TypeDef *rxPort;   /**< RX 引脚端口 */
    uint16_t      rxPin;    /**< RX 引脚 Pin */
    GPIO_TypeDef *txPort;   /**< TX 引脚端口 */
    uint16_t      txPin;    /**< TX 引脚 Pin */
    uint32_t      baudrate; /**< 波特率 */
} SoftUART_Config_t;

/**
 * @brief  初始化一个软串口通道
 * @param  cfg  配置结构，GPIO 引脚和波特率
 * @retval 句柄（0…SOFTUART_MAX_CHANNELS-1）
 */
SoftUART_Handle_t SoftUART_Init(const SoftUART_Config_t *cfg);

/**
 * @brief  发送一个字节（非阻塞）
 * @retval HAL_OK 成功，HAL_ERROR 失败（缓冲区满或非法句柄）
 */
HAL_StatusTypeDef SoftUART_SendByte(SoftUART_Handle_t h, uint8_t b);

/**
 * @brief  接收一个字节（可阻塞）
 * @param  timeout_ms 等待超时，单位毫秒
 * @retval 1=收到 0=超时或错误
 */
uint8_t SoftUART_ReceiveByte(SoftUART_Handle_t h, uint8_t *out, uint32_t timeout_ms);

void My_TIM4_IRQHandler(void);
#endif //SOFTWARE_UART_H
