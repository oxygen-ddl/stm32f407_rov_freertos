#include "ath20_bmp280.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

//bmp280大气压强与温度传感器
float bmp280_temperature = 0.0f; // 温度数据
float bmp280_pressure = 0.0f; // 压力数据
//ath20 温湿度传感器数据
float ath20_humidity = 0.0f; // 湿度数据
float ath20_temperature = 0.0f; // 温度数据

/* BMP280 I2C 地址 */
#define BMP280_ADDR         (0x76 << 1)  // 或 0x77<<1，看你的硬件接线
/* BMP280 寄存器 */
#define BMP280_REG_CALIB00  0x88
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7  // 开始读 8 字节：press(3) + temp(3) + humidity(2)

/* BMP280 校准参数结构 */
static int32_t  dig_T1;
static int32_t  dig_T2;
static int32_t  dig_T3;
static int32_t  dig_P1;
static int32_t  dig_P2;
static int32_t  dig_P3;
static int32_t  dig_P4;
static int32_t  dig_P5;
static int32_t  dig_P6;
static int32_t  dig_P7;
static int32_t  dig_P8;
static int32_t  dig_P9;

/* BMP280 t_fine，用于温度和压力补偿 */
static int32_t t_fine;

/**
 * @brief 读取 BMP280 的校准数据
 */
static HAL_StatusTypeDef BMP280_ReadCalib(void)
{
    uint8_t buf[24];
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR, BMP280_REG_CALIB00,
                           I2C_MEMADD_SIZE_8BIT, buf, 24, 100);
    if (ret != HAL_OK) return ret;

    dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    dig_T2 = (int16_t) (buf[3] << 8 | buf[2]);
    dig_T3 = (int16_t) (buf[5] << 8 | buf[4]);
    dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
    dig_P2 = (int16_t) (buf[9] << 8 | buf[8]);
    dig_P3 = (int16_t) (buf[11]<< 8 | buf[10]);
    dig_P4 = (int16_t) (buf[13]<< 8 | buf[12]);
    dig_P5 = (int16_t) (buf[15]<< 8 | buf[14]);
    dig_P6 = (int16_t) (buf[17]<< 8 | buf[16]);
    dig_P7 = (int16_t) (buf[19]<< 8 | buf[18]);
    dig_P8 = (int16_t) (buf[21]<< 8 | buf[20]);
    dig_P9 = (int16_t) (buf[23]<< 8 | buf[22]);

    return HAL_OK;
}

/**
 * @brief 对 BMP280 原始温度做补偿，返回摄氏度*100
 */
static int32_t BMP280_Compensate_T(int32_t adc_T)
{
    int32_t var1 = ((((adc_T>>3) - (dig_T1<<1))) * (dig_T2)) >> 11;
    int32_t var2 = (((((adc_T>>4) - dig_T1) * ((adc_T>>4) - dig_T1)) >> 12) * dig_T3) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}

/**
 * @brief 对 BMP280 原始压力做补偿，返回 Pa
 */
static int32_t BMP280_Compensate_P(int32_t adc_P)
{
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * dig_P6;
    var2 = var2 + ((var1 * dig_P5) << 17);
    var2 = var2 + ((int64_t)dig_P4 << 35);
    var1 = ((var1 * var1 * dig_P3)>>8) + ((var1 * dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1)) * dig_P1 >> 33;
    if (var1 == 0) return 0; // 防止除零
    int64_t p = 1048576 - adc_P;
    p = (((p<<31) - var2) * 3125) / var1;
    var1 = (dig_P9 * (p>>13) * (p>>13)) >> 25;
    var2 = (dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)dig_P7<<4);
    return (int32_t)p;
}

HAL_StatusTypeDef BMP280_Read(void)
{
    uint8_t raw[8];
    HAL_StatusTypeDef ret;

    /* 发控制寄存器：温度+压力 oversampling x1, 模式正常 */
    uint8_t cmd = (1<<5)|(1<<2)|   // osrs_t = x1, osrs_p = x1
                  (3<<0);          // mode = normal
    ret = HAL_I2C_Mem_Write(&hi2c2, BMP280_ADDR, BMP280_REG_CTRL_MEAS,
                            I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
    if (ret != HAL_OK) return ret;

    /* 等待转换时间，大约 8ms */
    HAL_Delay(8);

    /* 连续读 8 字节：PRESS_MSB..TEM_MSB..TEM_LSB */
    ret = HAL_I2C_Mem_Read(&hi2c2, BMP280_ADDR, BMP280_REG_PRESS_MSB,
                           I2C_MEMADD_SIZE_8BIT, raw, 8, 100);
    if (ret != HAL_OK) return ret;

    int32_t adc_P = (raw[0]<<12)|(raw[1]<<4)|(raw[2]>>4);
    int32_t adc_T = (raw[3]<<12)|(raw[4]<<4)|(raw[5]>>4);

    /* 补偿计算 */
    int32_t t100 = BMP280_Compensate_T(adc_T);  // 温度 *100
    int32_t p32 = BMP280_Compensate_P(adc_P);  // 单位 Pa<<4

    bmp280_temperature = t100 / 100.0f;
    bmp280_pressure    = p32 / 256.0f;  // 右移 8 后得到 Pa
    return HAL_OK;
}

/* ATH20(SHT20) I2C 地址 */
#define ATH20_ADDR      (0x40<<1)

/**
 * @brief 读取 ATH20 温度（命令 0xF3）或湿度（命令 0xF5）
 */
static HAL_StatusTypeDef ATH20_ReadCmd(uint8_t cmd, uint16_t *raw)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Master_Transmit(&hi2c2, ATH20_ADDR, &cmd, 1, 100);
    if (ret != HAL_OK) return ret;

    /* 根据 datasheet，最大等待时长 ~85 ms */
    HAL_Delay(85);

    uint8_t buf[3];
    ret = HAL_I2C_Master_Receive(&hi2c2, ATH20_ADDR, buf, 3, 100);
    if (ret != HAL_OK) return ret;

    *raw = (buf[0]<<8)|buf[1]; // 忽略 CRC
    return HAL_OK;
}

HAL_StatusTypeDef ATH20_Read(void)
{
    uint16_t rawT, rawH;
    HAL_StatusTypeDef ret;

    ret = ATH20_ReadCmd(0xF3, &rawT);
    if (ret != HAL_OK) return ret;
    ret = ATH20_ReadCmd(0xF5, &rawH);
    if (ret != HAL_OK) return ret;

    /* 转换公式 */
    ath20_temperature = -46.85f + 175.72f * (rawT / 65536.0f);
    ath20_humidity    = -6.0f  + 125.0f * (rawH / 65536.0f);
    return HAL_OK;
}

void Sensors_Init(void)
{
    /* 读取 BMP280 校准数据 */
    BMP280_ReadCalib();
    /* 上面已经把 hi2c2、GPIO、CLK 都初始化好 */
}

void Sensors_Task(void *pvParameters)
{
    (void)pvParameters; // 避免未使用参数警告

    for(;;)
    {
        /* 读取 BMP280 */
        if (BMP280_Read() != HAL_OK)
        {
            bmp280_temperature = 0.0f;
            bmp280_pressure = 0.0f;
        }

        /* 读取 ATH20 */
        if (ATH20_Read() != HAL_OK)
        {
            ath20_temperature = 0.0f;
            ath20_humidity = 0.0f;
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒更新一次
    }
}

