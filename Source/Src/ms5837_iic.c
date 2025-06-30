#include "ms5837_iic.h"
#include "cmsis_os.h"
#include "task.h"
#include <math.h>
#include "i2c.h"
#include "usart.h"
#include "stdio.h"
/*
C1  压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3  温度压力灵敏度系数 TCS
C4  温度系数的压力补偿 TCO
C5  参考温度 T|REF
C6  温度系数的温度 TEMPSENS
*/

// MS5837-03BA I²C 地址
#define MS5837_ADDR (0x76 << 1) // 7-bit=0x76, HAL 要求左移1位

// 命令定义
#define CMD_RESET 0x1E
#define CMD_CONVERT_D1 0x48                  // OSR=4096 压力
#define CMD_CONVERT_D2 0x58                  // OSR=4096 温度
#define CMD_PROM_READ(i) (0xA0 + ((i) << 1)) // i=1..6

// PROM 校准系数
static uint16_t C[7]; // C[1]..C[6] 有效

// 任务句柄
static osThreadId_t ms5837TaskHandle;

// 计算得到的全局传感器数据
float ms5837_temperature; // °C
float ms5837_pressure;    // mbar
float ms5837_depth;       // m

// 内部使用变量
static int32_t dT;
static int64_t OFF, SENS;

// 前向声明
static void MS5837_Reset(void);
static HAL_StatusTypeDef MS5837_ReadPROM(void);
static HAL_StatusTypeDef MS5837_ReadADC(uint32_t cmd, uint32_t *adc);
static void MS5837_Compensate(uint32_t D1, uint32_t D2);

/**
 * @brief   MS5837 驱动初始化（在 FreeRTOS 启动前调用即可）
 */
void MS5837_Init(void)
{
    // 1. 复位
    MS5837_Reset();
    vTaskDelay(pdMS_TO_TICKS(20));
    // 2. 读取 PROM 校准系数
    if (MS5837_ReadPROM() != HAL_OK)
    {
        // TODO: 错误处理
        HAL_UART_Transmit_DMA(&huart6, (uint8_t *)"MS5837 PROM read error!\n", 24);
        printf("MS5837 PROM: C1=%04X, C2=%04X, C3=%04X, C4=%04X, C5=%04X, C6=%04X\n",
               C[1], C[2], C[3], C[4], C[5], C[6]);
    }
    printf("MS5837 PROM: C1=%04X, C2=%04X, C3=%04X, C4=%04X, C5=%04X, C6=%04X\n",
           C[1], C[2], C[3], C[4], C[5], C[6]);
}

/**
 * @brief   MS5837 循环任务
 */
static void MS5837_Task(void *argument)
{
    uint32_t D1, D2;

    (void)argument;
    for (;;)
    {
        // 1. 发起压力转换，读取 ADC
        MS5837_ReadADC(CMD_CONVERT_D1, &D1);
        // 2. 发起温度转换，读取 ADC
        MS5837_ReadADC(CMD_CONVERT_D2, &D2);

        // 3. 补偿计算
        MS5837_Compensate(D1, D2);

        // 4. 暂停 200 ms（可根据实际需求调整）
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief   启动一个 FreeRTOS 任务，周期读取并计算传感器数据
 */
void MS5837_StartTask(void)
{
    MS5837_Init(); // 初始化传感器
    const osThreadAttr_t attr = {
        .name = "ms5837",
        .stack_size = 512,
        .priority = (osPriority_t) osPriorityLow1,
    };
    ms5837TaskHandle = osThreadNew(MS5837_Task, NULL, &attr);
}

/** @name   低级 I²C 操作封装
    @{ */

/**
 * @brief   发送复位命令
 */
static void MS5837_Reset(void)
{
    HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, (uint8_t[]){CMD_RESET}, 1, HAL_MAX_DELAY);
}

/**
 * @brief   读取 PROM 中的 6 组校准值
 */
static HAL_StatusTypeDef MS5837_ReadPROM(void)
{
    HAL_StatusTypeDef res;
    for (uint8_t i = 1; i <= 6; i++)
    {
        uint8_t buf[2];
        res = HAL_I2C_Mem_Read(&hi2c2, MS5837_ADDR, CMD_PROM_READ(i), I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
        if (res != HAL_OK)
            return res;
        C[i] = (buf[0] << 8) | buf[1];
    }
    return HAL_OK;
}

/**
 * @brief   发起一次转换并读取 ADC
 * @param   cmd   转换命令（CMD_CONVERT_D1 / CMD_CONVERT_D2）
 * @param   adc   接收返回的 24bit 原始值指针
 */
static HAL_StatusTypeDef MS5837_ReadADC(uint32_t cmd, uint32_t *adc)
{
    HAL_StatusTypeDef res;
    uint8_t raw[3];

    // 1) 发起转换
    res = HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, (uint8_t[]){(uint8_t)cmd}, 1, HAL_MAX_DELAY);
    if (res != HAL_OK)
        return res;

    // 2) 等待最大转换时间约 10ms
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3) 读 ADC 寄存器（0x00）
    res = HAL_I2C_Master_Transmit(&hi2c2, MS5837_ADDR, (uint8_t[]){0x00}, 1, HAL_MAX_DELAY);
    if (res != HAL_OK)
        return res;
    res = HAL_I2C_Master_Receive(&hi2c2, MS5837_ADDR, raw, 3, HAL_MAX_DELAY);
    if (res != HAL_OK)
        return res;

    // 24 位拼接
    *adc = (uint32_t)raw[0] << 16 | (uint32_t)raw[1] << 8 | raw[2];
    return HAL_OK;
}
/** @} */

/**
 * @brief   一阶／二阶补偿计算
 * @param   D1  原始压力 ADC
 * @param   D2  原始温度 ADC
 */
static void MS5837_Compensate(uint32_t D1, uint32_t D2)
{
    int64_t OFF2 = 0, SENS2 = 0;

    // dT = D2 - C5 × 2^8
    dT = (int32_t)D2 - ((int32_t)C[5] << 8);

    // TEMP = 20°C + dT × C6 / 2^23
    ms5837_temperature = 2000 + ((int64_t)dT * C[6]) / 8388608;
    // 初步 OFF / SENS
    OFF = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT >> 7);
    SENS = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT >> 8);

    // 二阶补偿
    if (ms5837_temperature < 2000)
    {
        int64_t t = ms5837_temperature - 2000;
        OFF2 = (3 * (t * t)) >> 1;
        SENS2 = (5 * (t * t)) >> 3;
        if (ms5837_temperature < -1500)
        {
            int64_t tt = ms5837_temperature + 1500;
            OFF2 += 7 * (tt * tt);
            SENS2 += (11 * (tt * tt)) >> 1;
        }
    }

    OFF -= OFF2;
    SENS -= SENS2;

    // 计算最终压力（单位 mbar）
    // Pressure = (D1 × SENS / 2^21 - OFF) / 2^15
    ms5837_pressure = (((D1 * SENS) >> 21) - OFF) / 32768.0f / 100;

    // 根据水的密度计算深度（单位 m，ρ≈1029kg/m³，g≈9.8）
    ms5837_depth = (ms5837_pressure - /*大气压*/ 1013.25f) * 100.0f / (1029.0f * 9.8f);
}
