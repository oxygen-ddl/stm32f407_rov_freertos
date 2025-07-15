#ifndef DISTANCE_MEASURE_H
#define DISTANCE_MEASURE_H


#include "stm32f4xx_hal.h"
extern float wave_distance[2]; // 由超声波避障传感器测量的距离数据

void soft_uart_init(void);
void Handle_Muart_Task(void *pvParameters);
void Trigger_Distance_Mearsure_Task(void *pvParameters);


#endif/* DISTANCE_MEASURE_H */

