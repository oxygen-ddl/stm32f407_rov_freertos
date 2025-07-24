#ifndef SOFTWARE_UART_H
#define SOFTWARE_UART_H

#include "stm32f4xx_hal.h"
#include <string.h>

#define SOFTUART_MAX_CH      2    /* 使用 2 路 UART */

typedef enum {SU_IDLE, SU_TX, SU_RX_START, SU_RX_DATA} SU_State_t;

typedef struct
{
    GPIO_TypeDef *tx_port;
    uint16_t      tx_pin;

    GPIO_TypeDef *rx_port;
    uint16_t      rx_pin;
    uint32_t      rx_exti_line;   /* EXTI_Line 对应 */

    /* 发送缓存 */
    uint8_t  tx_buf[64];
    uint16_t tx_head;
    uint16_t tx_tail;
    uint16_t tx_cnt;

    /* 接收缓存 */
    uint8_t  rx_buf[64];
    uint16_t rx_head;
    uint16_t rx_tail;
    uint16_t rx_cnt;

    /* 运行时变量 */
    SU_State_t state;
    uint8_t    bit_cnt;
    uint8_t    cur_byte;
} SoftUART_HandleTypeDef;

/* ======= API ======= */
void SoftUART_Init(void);                                 /* 在 main.c -> MX_TIM4_Init 后调用 */
void SoftUART_PutChar(uint8_t ch, uint8_t ch_idx);        /* 发送 1 字节 */
int  SoftUART_GetChar(uint8_t *pch, uint8_t ch_idx);      /* 1 = 读取到数据 */

void My_TIM4_IRQHandler(void);


extern SoftUART_HandleTypeDef su[SOFTUART_MAX_CH];

#endif //SOFTWARE_UART_H
