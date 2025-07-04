#ifndef ATH20_BMP280_H
#define ATH20_BMP280_H

#include "stm32f4xx_hal.h"

// bmp280大气压强与温度传感器
extern float bmp280_temperature; // 温度数据
extern float bmp280_pressure;    // 压力数据
// ath20 温湿度传感器数据
extern float ath20_humidity;     // 湿度数据
extern float ath20_temperature; // 温度数据


/**
 * @brief 初始化并读取一次各传感器的校准数据
 *        必须在 I2C2 初始化之后调用一次
 */
void Sensors_Init(void);

/**
 * @brief 读取 BMP280，更新 bmp280_temperature 与 bmp280_pressure
 */
HAL_StatusTypeDef BMP280_Read(void);

/**
 * @brief 读取 ATH20（SHT20），更新 ath20_temperature 与 ath20_humidity
 */
HAL_StatusTypeDef ATH20_Read(void);

void Sensors_Task(void *pvParameters);


#endif /* ATH20_BMP280_H */

