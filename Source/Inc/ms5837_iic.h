#ifndef MX5837_IIC_H
#define MX5837_ICC_H

#include "stm32f4xx_hal.h"

extern float ms5837_depth; // 深度数据
extern float ms5837_pressure; // 压力数据

void MS5837_Init(void);
void MS5837_StartTask(void);



#endif/* MX5837_IIC_H */
