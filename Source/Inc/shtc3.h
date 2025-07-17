#ifndef SHTC3_H
#define SHTC3_H


#include "stm32f4xx_hal.h"


extern float sthc3_temperature; // 温度数据
extern float sthc3_humidity;    // 湿度数据

void Sthc3SensorI2c_Init(void);
void Sthc3SensorI2cRead_Task(void *argument);



#endif //SHTC3_H
