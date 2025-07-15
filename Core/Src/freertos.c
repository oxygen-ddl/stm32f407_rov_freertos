/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "jy901p_uart.h"
#include "chat_with_upper.h"
#include "ms5837_iic.h"
#include "func_uart.h"
#include "move_control.h"
#include "ms5837_uart.h"
#include "move_drv.h"
#include "ath20_bmp280.h"
#include "distance_measure.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t view_variables_TaskHandle; // 变量观测任务句柄
osThreadId_t SendAllPack_TaskHandle; // 发送任务句柄
osThreadId_t Jy901p_Uart_TaskHandle; // jy901p UART任务句柄
osThreadId_t Move_Control_TaskHandle; // 推进器控制任务句柄
osThreadId_t MS5837_IIC_TaskHandle; // MS5837 IIC任务句柄
osThreadId_t Ath20_Bmp280_TaskHandle; // 循环LED任务句柄
osThreadId_t tuigan_TaskHandle;
osThreadId_t Wave_Distance_Trigger_TaskHandle;//避障传感器触发任务句柄
osThreadId_t Wave_Distance_Handle_TaskHandle;//避障传感器解析任务句柄


const osThreadAttr_t view_variables_attributes = {
  .name = "view_variables",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t SendAllPack_attributes = {
  .name = "SendAllPack",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};

const osThreadAttr_t Jy901p_Uart_attributes = {
  .name = "Jy901p_Uart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};

const osThreadAttr_t Move_Control_attributes = {
  .name = "Move_Control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};

const osThreadAttr_t MS5837_IIC_attributes = {
  .name = "MS5837_IIC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

const osThreadAttr_t ath20_bmp280_attributes = {
  .name = "Ath20_Bmp280",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};

const osThreadAttr_t tuigan_attributes = {
  .name = "tuigan",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};

const osThreadAttr_t wave_distance_trigger_attributes = {
  .name = "wave_distance__trigger",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};

const osThreadAttr_t wave_distance_handle_attributes = {
  .name = "wave_distance_handle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Cycle_led */
osThreadId_t Cycle_ledHandle;
const osThreadAttr_t Cycle_led_attributes = {
  .name = "Cycle_led",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Cycle_led_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  Parser_Init(); // 初始化解析器
  JY901_UART_Init(); // 启动jy901p DMA空闲检测
  //Move_Control_Task(NULL); // 启动推进器控制任务
  Parser4_Init(); // 启动MS5837 UART解析任务
  Move_basic_Init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Cycle_led */
  Cycle_ledHandle = osThreadNew(Cycle_led_Task, NULL, &Cycle_led_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  Move_Control_TaskHandle = osThreadNew(upper_move_process, NULL, &Move_Control_attributes); // 启动推进器控制任务

  //tuigan_TaskHandle = osThreadNew(pull_use_electric,NULL,&tuigan_attributes);

  Jy901p_Uart_TaskHandle = osThreadNew(JY901_ProcessTask, NULL, &Jy901p_Uart_attributes); // 启动jy901p UART任务
  
  SendAllPack_TaskHandle = osThreadNew(SendAllPack_Task, NULL,&SendAllPack_attributes); // 启动发送任务 

  view_variables_TaskHandle = osThreadNew(view_variables_Task, NULL, &view_variables_attributes);// 启动变量观测任务

  Ath20_Bmp280_TaskHandle = osThreadNew(Sensors_Task, NULL, &ath20_bmp280_attributes); // 启动循环LED任务

  Wave_Distance_Trigger_TaskHandle = osThreadNew(Trigger_Distance_Mearsure_Task, NULL, &wave_distance_trigger_attributes);//启动避障传感器周期性触发任务

  Wave_Distance_Trigger_TaskHandle = osThreadNew(Handle_Muart_Task,NULL,&wave_distance_handle_attributes);//启动避障传感器周期性触发任务


  


  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Cycle_led_Task */
/**
* @brief Function implementing the Cycle_led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cycle_led_Task */
void Cycle_led_Task(void *argument)
{
  /* USER CODE BEGIN Cycle_led_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2); // 切换LED状态
  }
  /* USER CODE END Cycle_led_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

