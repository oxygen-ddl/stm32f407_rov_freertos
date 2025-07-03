#ifndef MX5837_IIC_H
#define MX5837_ICC_H

#include "stm32f4xx_hal.h"
#include "i2c.h"

extern float ms5837_depth; // 深度数据
extern float ms5837_pressure; // 压力数据


//Definitions:
#define MS5837_ADDR               0x76

#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A

//Models:
#define MS5837_30BA               0x00
#define MS5837_02BA				  0x01

#define waterDensity = 1029;

struct MS5837_t {
	//MS5837 model(Default MS5837_30BA)
	uint8_t model;
	//Fluid density (Default 1029)
	float fluidDensity;
	//Pressure unit (Default mBar)
	float temperture;
	float pressure;
};

//Values red from MS5837
struct MS5837_values_t{
	int32_t TEMP;
	int32_t P;
	uint16_t C[8];
	uint32_t D1;
	uint32_t D2;
};

uint8_t MS5837_init(I2C_HandleTypeDef *i2c_channel);
void MS5837_Task(void *argument);

#endif/* MX5837_IIC_H */
