#ifndef TRANSMIT_POWER_BOARD_H
#define TRANSMIT_POWER_BOARD_H

#include "stm32f4xx_hal.h"

 extern uint16_t current_adc_data[9];//8个推进器电流+总电流
 extern uint16_t adc_data_after_filter[9];


 extern uint16_t motor_current_actual[8];   //adc采集的推进器电流数据，由电源板采集后传入主控


#endif /* TRANSMIT_POWER_BOARD_H */

