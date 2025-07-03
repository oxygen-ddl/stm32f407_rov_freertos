#ifndef CHAT_WITH_UPPER_H
#define CHAT_WITH_UPPER_H

#include "stm32f4xx_hal.h"

#define USER_MAIN_DEBUG    //不需要打印信息时时注释掉该行宏

#ifdef USER_MAIN_DEBUG
  /* 简单输出一行，自动加 “\r\n” */
  #define user_printf(format, ...)       printf(format "\r\n", ##__VA_ARGS__)
  /* 带模块前缀的 INFO/DEBUG/ERROR */
  #define user_main_info(format, ...)    printf("[ main ] INFO : " format "\r\n", ##__VA_ARGS__)
  #define user_main_debug(format, ...)   printf("[ main ] DEBUG: " format "\r\n", ##__VA_ARGS__)
  #define user_main_error(format, ...)   printf("[ main ] ERROR: " format "\r\n", ##__VA_ARGS__)

#else
  /* 关掉所有日志 */
  #define user_printf(format, ...)
  #define user_main_info(format, ...)
  #define user_main_debug(format, ...)
  #define user_main_error(format,...)
#endif


#define S_PACKAGE_LEN                         50          //发送缓冲区长度
#define R_PACKAGE_LEN                         40          //接收缓冲区长度


#define TX_StartBit_ACC                     0xA1        //向上位机传输jy901s数据
#define TX_StartBit_DEP                     0xA2        //向上位机传输MS5837数据
#define TX_StartBit_TEM_WET                 0xA3        //向上位机传输shtc3数据
#define TX_StartBit_PWM1_8_power            0xA4        //向上位机传输推进器电流数据


/**************************************数据包帧头信息***************************************** */
/***     用于PID调参          ***/
#define RX_StartBit_PID                     0xC6       //下位机接收pid参数
/***       手柄            ***/
#define RX_StartBit_Handle_basic            0xB1        //下位机接收手柄基本运动数据
#define RX_StartBit_Handle_light            0xB2        //下位机接收手柄照明数据
#define RX_StartBit_Handle_func_lockangle   0xB3        //下位机接收按键功能控制-锁定角度
#define RX_StartBit_Handle_func_start_move      0xB4        //下位机接收按键功能控制-开启运动
#define RX_StartBit_Handle_func_autotrip      0xB5       //下位机接收按键功能控制-定速
#define RX_StartBit_Handle_func_electromagnet  0xB6     // 下位机接收按键功能控制-电磁铁
#define RX_StartBit_Handle_func_push_rod  0xB7          // 下位机接收按键功能控制-推杆
#define RX_StartBit_Handle_func_autorolling   0xB8       // 下位机接收按键功能控制-自动对正

#define RX_StartBit_Handle_func_defogging     0xC5

void parser_feed(uint8_t ch);
void SendAllPack_Task(void *pvParameters);
void Parser_Init(void);
void vParserTask(void *pvParameters);







#endif /* CHAT_WITH_UPPER_H */

