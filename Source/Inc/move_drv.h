#ifndef MOVE_DRV_H
#define MOVE_DRV_H

#include "stm32f4xx_hal.h"


void pwm_init(void);
void pwm_set(uint8_t num,uint8_t channel,uint16_t data);
void motor_set(uint8_t num,float data);
void light_set(uint16_t data);


#endif /* MOVE_DRV_H */
