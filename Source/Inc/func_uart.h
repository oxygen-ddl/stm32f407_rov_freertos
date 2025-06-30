#ifndef _FUNC_UART_H
#define _FUNC_UART_H

#include "stm32f4xx_hal.h" 
#include <stdio.h>
#include <stdbool.h>

#define UART_TEST huart6
#define UART_TEST_UART huart6

bool my_printf(const char *fmt, ...);
bool uart_send_float(float f);
bool uart_just_float_send(uint8_t count, ...);

extern uint8_t rx_ready;
void UART2_DMA_StartReceive(void);
void Parse_SetAngle(float* set_angle);
#endif // _FUNC_UART_H
