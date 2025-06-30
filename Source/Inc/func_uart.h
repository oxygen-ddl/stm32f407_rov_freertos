#ifndef FUNC_UART_H
#define FUNC_UART_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

bool UART_SendFloats_DMA(uint8_t count, ...);
bool UART_SendFloats_Blocking(uint8_t count, ...);


/*************************************变量观测**************************************/

void view_variables_StartTask(void);

#endif  /* FUNC_UART_H */
