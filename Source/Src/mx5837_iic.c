#include "mx5837_iic.h"
#include "cmsis_os.h"
#include <math.h>
#include "i2c.h"


float mx5837_depth = 0.0f; // 深度数据
float mx5837_pressure = 0.0f; // 压力数据

/*
C1  压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3  温度压力灵敏度系数 TCS
C4  温度系数的压力补偿 TCO
C5  参考温度 T|REF
C6  温度系数的温度 TEMPSENS
*/

#define MS5837_ADDR               0xEC	//0x76  
#define MS5837_RESET              0x1E
#define MS5837_ADC_READ           0x00
#define MS5837_PROM_READ          0xA0
#define MS5837_CONVERT_D1_8192    0x4A
#define MS5837_CONVERT_D2_8192    0x5A


