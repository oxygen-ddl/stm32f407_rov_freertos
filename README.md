# STM32F407 ROV (Remotely Operated Vehicle) FreeRTOS 项目

## 项目概述

这是一个基于 **STM32F407VE** 微控制器和 **FreeRTOS** 实时操作系统的水下遥操作机器人(ROV)控制系统。该项目集成了多个传感器、电机驱动和通信模块，实现了对水下机器人的完整控制与监测。

---

## 主要特性

✅ **实时操作系统** - 基于FreeRTOS的多任务管理  
✅ **多传感器集成** - 支持多种温度、气压、湿度、IMU、压力、距离传感器  
✅ **电机控制** - 直流电机驱动与PID闭环控制  
✅ **通信系统** - 与上位机通信、硬件/软件UART/I2C支持  
✅ **模块化设计** - 清晰的代码结构，易于维护和扩展  

---

## 硬件配置

### 微控制器
- **芯片**: STM32F407VETx
- **内核**: ARM Cortex-M4
- **主频**: 168MHz
- **Flash**: 512KB
- **RAM**: 192KB

### 主要接口
- **I2C**: 用于传感器通信
- **UART**: 用于串口通信
- **SPI**: 可选扩展
- **GPIO**: 电机控制、状态指示
- **DMA**: 数据传输加速

### 传感器模块

| 传感器 | 功能 | 通信方式 | 文件 |
|--------|------|--------|------|
| ATH20 | 温湿度传感 | I2C | `ath20_bmp280.c` |
| BMP280 | 气压/温度传感 | I2C | `ath20_bmp280.c` |
| MS5837 | 水深(压力)传感 | I2C/UART | `ms5837_iic.c`/`ms5837_uart.c` |
| JY901P | 惯性测量(IMU) | UART | `jy901p_uart.c` |
| SHTC3 | 温湿度传感 | I2C | `shtc3.c` |
| 声纳 | 距离测量 | 自定义 | `sonar.c`/`distance_measure.c` |

### 执行器(电机驱动)
- **电机驱动**: PWM控制直流电机
- **驱动文件**: `move_drv.c`
- **PID控制**: 速度/位置闭环反馈

---

## 项目文件结构

```
stm32f407_rov_freertos/
├── Core/                          # STM32 HAL库配置
│   ├── Inc/                       # 头文件
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   ├── FreeRTOSConfig.h       # FreeRTOS配置
│   │   ├── dma.h, gpio.h, i2c.h, tim.h, usart.h
│   │   └── stm32f4xx_it.h         # 中断处理
│   └── Src/                       # 源文件
│       ├── main.c                 # 主程序入口
│       ├── freertos.c             # FreeRTOS任务创建
│       ├── stm32f4xx_hal_msp.c    # HAL MSP回调
│       └── stm32f4xx_it.c         # 中断服务程序
├── Source/                        # 自定义应用代码
│   ├── Inc/                       # 应用头文件
│   └── Src/                       # 应用源文件
│       ├── 📌 传感器模块
│       │   ├── ath20_bmp280.c/h   # ATH20温湿度 + BMP280气压
│       │   ├── ms5837_iic.c/h     # MS5837水深(I2C)
│       │   ├── ms5837_uart.c/h    # MS5837水深(UART)
│       │   ├── shtc3.c/h          # SHTC3温湿度
│       │   ├── jy901p_uart.c/h    # JY901P IMU(九轴传感器)
│       │   ├── sonar.c/h          # 声纳传感器
│       │   └── distance_measure.c/h # 距离测量处理
│       ├── 📌 电机控制模块
│       │   ├── move_drv.c/h       # 电机驱动
│       │   ├── move_control.c/h   # 运动控制逻辑
│       │   └── pid.c/h            # PID控制算法
│       ├── 📌 通信模块
│       │   ├── chat_with_upper.c/h # 与上位机通信协议
│       │   ├── func_uart.c/h      # UART功能函数
│       │   ├── software_uart.c/h  # 软件UART实现
│       │   ├── software_iic.c/h   # 软件I2C实现
│       │   └── transmit_power_board.c/h # 电源板控制
│       └── ...
├── Drivers/                       # STM32驱动库
│   ├── CMSIS/                     # ARM CMSIS标准库
│   └── STM32F4xx_HAL_Driver/      # STM32 HAL驱动
├── Middlewares/                   # 中间件库
│   └── Third_Party/
├── MDK-ARM/                       # Keil MDK工程文件
│   ├── stm32f407_rov_freertos.uvprojx  # MDK项目文件
│   ├── DebugConfig/               # 调试配置
│   └── RTE/                       # 运行时接整配置
└── stm32f407_rov_freertos.ioc     # STM32CubeMX配置文件
```

---

## 核心模块说明

### 1. **电机驱动与控制** (`move_drv.c` / `move_control.c`)

```c
// PID控制结构
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
} pid_rov;

typedef struct {
    float target_value;           // 目标值
    float actual_value;           // 实际值
    float err;                    // 偏差
    float integral_value;         // 积分值
    float out_data_limit;         // 输出限幅
} pid_set;
```

**主要功能:**
- PWM调速控制
- PID闭环反馈控制
- 多电机协调运动

### 2. **传感器数据采集** (`ath20_bmp280.c` / `ms5837_iic.c` 等)

```c
// 外部数据变量
extern float bmp280_temperature;    // BMP280温度
extern float bmp280_pressure;       // BMP280气压/高度
extern float ath20_humidity;        // ATH20湿度
extern float ath20_temperature;     // ATH20温度
```

**主要功能:**
- I2C/UART传感器数据读取
- 传感器校准与数据处理
- FreeRTOS任务调度

### 3. **通信系统** (`chat_with_upper.c`)

**功能:**
- 与地面控制站通信
- 实时数据上传
- 指令接收与解析
- 支持硬件UART与软件UART实现

### 4. **PID控制** (`pid.c`)

**功能:**
- 标准PID算法实现
- 积分限幅与变系数控制
- 闭环稳定性设计

---

## 工作流程

```
启动 → HAL初始化 → GPIO/UART/I2C/Timer初始化 
   ↓
FreeRTOS任务创建
   ↓
├─ 传感器采集任务(10Hz)  → 读取各传感器数据
├─ 电机控制任务(50Hz)    → PID运算 → 输出PWM
├─ 通信任务(10Hz)        → 数据上传 ↔ 指令接收
├─ IMU处理任务           → 姿态计算
└─ 系统监控任务          → 状态检查
```

---

## 开发环境

### 必需工具
- **IDE**: Keil MDK-ARM v5.37+
- **编译器**: ARM Compiler 5/6
- **ST-Link**: 调试器/编程器
- **CubeMX**: STM32配置工具(可选)

### 依赖库
- STM32F4xx HAL库
- FreeRTOS v10+
- ARM CMSIS库

---

## 构建与编译

### 使用Keil MDK:
1. 打开 `MDK-ARM/stm32f407_rov_freertos.uvprojx`
2. 点击 **Build** 编译项目
3. 点击 **Download** 烧写到设备
4. 点击 **Debug** 启动调试

### 命令行编译(可选):
```bash
# 使用arm-gcc工具链
arm-none-eabi-gcc -c [sources] -I[includes] -DSTM32F407xx ...
arm-none-eabi-ld -o output.elf [object files]
```

---

## 配置说明

### FreeRTOS配置 (`Core/Inc/FreeRTOSConfig.h`)
- 定义了任务栈大小、队列大小等参数
- 可根据实际需求调整

### I2C/UART配置 (`Core/Src/main.c`)
- I2C2: 传感器通信(ATH20/BMP280/MS5837/SHTC3)
- UART1: 上位机通信
- UART2/3: 其他传感器(JY901P/MS5837)

### GPIO配置
- 各电机通过GPIO的PWM或GPIO反向控制
- LED指示灯状态

---

## 打包/烧写

### 生成的输出文件
- `MDK-ARM/stm32f407_rov_freertos/stm32f407_rov_freertos.axf` - 调试文件
- `MDK-ARM/stm32f407_rov_freertos/stm32f407_rov_freertos.hex` - 十六进制固件

### 烧写到设备
```
ST-Link → USB → 电脑
        ↓
     MDK IDE
        ↓
     Download (烧写到Flash)
```

---

## 常见问题 (FAQ)

**Q: 编译错误 "hal.h not found"?**  
A: 确保 `Drivers/STM32F4xx_HAL_Driver` 路径正确

**Q: FreeRTOS栈溢出?**  
A: 调整 `FreeRTOSConfig.h` 中的 `configMINIMAL_STACK_SIZE` 和任务栈大小

**Q: 传感器读取不到数据?**  
A: 检查I2C/UART连线、地址配置和初始化顺序

**Q: 电机不转?**  
A: 检查电机驱动引脚配置、PWM时钟和电源供应

---

## 扩展与自定义

### 添加新传感器
1. 在 `Source/Src/` 创建 `xxx_sensor.c/h`
2. 实现初始化和读取函数
3. 在 `main.c` 中创建FreeRTOS任务
4. 集成到通信协议中

### 修改通信协议
编辑 `Source/Src/chat_with_upper.c` 中的数据包格式

### 调整PID参数
修改 `Source/Src/move_control.c` 中的 `pid_rov` 结构体参数

---

## 许可证

本项目使用以下开源库:
- STM32 HAL库 (STMicroelectronics License)
- FreeRTOS (MIT License)
- ARM CMSIS (Apache 2.0 License)

---

## 联系方式

- **作者**: [Oxgen-ddl|R&S lab]
- **邮箱**: [2023212693@stu.hit.edu.cn]

---

## 更新日志

### v1.0 (2025-04)
- ✅ 项目初始化
- ✅ 基础传感器驱动集成
- ✅ 电机控制系统
- ✅ 上位机通信协议

---

**最后更新**: 2025年4月  
**项目状态**: 🟢 活跃开发中
