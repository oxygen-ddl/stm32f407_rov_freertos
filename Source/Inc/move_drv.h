#ifndef MOVE_DRV_H
#define MOVE_DRV_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t xTimer10Semaphore;
void pwm_init(void);
void pwm_set(uint8_t num,uint8_t channel,uint16_t data);
void motor_set(uint8_t num,float data);
void light_set(uint16_t data);
void move_putt_init(void);
void Move_basic_Init(void);
void upper_move_process(void *pvParameters);
void pull_use_electric(void *pvParameters);
#endif /* MOVE_DRV_H */
