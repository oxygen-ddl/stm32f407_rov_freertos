#include "transmit_power_board.h"

uint16_t current_adc_data[9];
uint16_t adc_data_after_filter[9];
uint16_t motor_current_actual[8];   //adc采集的推进器电流数据，由电源板采集后传入主控

uint8_t  electronic_switch[8];
uint16_t current_adc_data[9];
float    temperature_power_board;
float    RH_power_board;

