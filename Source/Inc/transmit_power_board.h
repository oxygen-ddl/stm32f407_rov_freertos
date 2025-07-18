#ifndef TRANSMIT_POWER_BOARD_H
#define TRANSMIT_POWER_BOARD_H

#include "stm32f4xx_hal.h"

#define power_board_max_len 64
typedef struct
{
    uint8_t data[power_board_max_len];
    uint16_t len;
}Power_board_Msg_t;


void Uart5_Parse_Init(void);
void Uart5_Parse_Task(void *pvParameters);
void switch_Process_Task(void *pvParameters);
void UART5_IT_TASK(void);


extern uint16_t current_adc_data[9];    //8个推进器电流+总电流
extern uint16_t adc_data_after_filter[9];
extern uint16_t motor_current_actual[8];   //adc采集的推进器电流数据，由电源板采集后传入主控

extern float temperature_power_board;
extern float RH_power_board;

#endif /* TRANSMIT_POWER_BOARD_H */
