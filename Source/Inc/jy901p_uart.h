#ifndef JY901P_UART_H
#define JY901P_UART_H

#include "stm32f4xx_hal.h"


#define JY901P_RX_BUFFER_SIZE 128


extern float accx, accy, accz, angx, angy, angz, magx, magy, magz;
extern float roll,pitch,yaw,temperature_jy901s;
extern uint8_t transmit_jy901_choose;
extern int turns_of_pitch,turns_of_roll,turns_of_yaw;
extern float roll_total,pitch_total,yaw_total,yaw_total_start;


/* 初始化函数，在 MX_FREERTOS_Init() 中调用 */
void jy901p_uart_init(void);


#endif /* JY901P_UAYRT_H */
