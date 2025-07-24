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

#define TX_StartBit_ACC                     0xA1        //向上位机传输jy901s数据
#define TX_StartBit_DEP                     0xA2        //向上位机传输MS5837数据
#define TX_StartBit_TEM_WET                 0xA3        //向上位机传输shtc3数据
#define TX_StartBit_PWM1_8_power            0xA4        //向上位机传输推进器电流数据


/**************************************数据包帧头信息***************************************** */
/***     用于PID调参          ***/
#define RX_StartBit_PID                     0xC6       //下位机接收pid参数
/***       手柄            ***/
#define RX_StartBit_Handle_basic            0xB1        //下位机接收手柄基本运动数据
#define RX_StartBit_Handle_light            0xB2        //下位机接收手柄功能按键数据

#define RX_StartBit_Handle_func_defogging     0xC5

#define PARSER_DMA_BUF_SIZE   256   // DMA 缓冲区大小，>= 你可能一帧的最大长度
#define SEND_BUF_SIZE    128

typedef struct
{
  uint8_t data[PARSER_DMA_BUF_SIZE];
  uint16_t len;
}Parse_Msg_t;

extern 


void SendAllPack_Task(void *pvParameters);
void Parser_Init(void);
void Parse_Task(void *pvParameters);
void UART3_IT_TASK(void);






#endif /* CHAT_WITH_UPPER_H */

