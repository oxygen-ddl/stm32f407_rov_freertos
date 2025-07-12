#include "transmit_power_board.h"
#include <stdio.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

#define data_buffer_size 25

uint16_t current_adc_data[9];
uint16_t adc_data_after_filter[9];
uint16_t motor_current_actual[8]; // adc采集的推进器电流数据，由电源板采集后传入主控

uint8_t electronic_switch[8];
uint16_t current_adc_data[9];
float temperature_power_board;
float RH_power_board;
uint8_t switch_send_flag;

uint8_t Tx_buffer_5[data_buffer_size];

/*****************************************环形缓冲区********************************************************* */
/*简易环形缓冲区*/
static struct
{
    uint8_t Rx_buffer_5[data_buffer_size]; // 存储数据的数组
    uint8_t head;                          // 指向下一个要写入的位置
    uint8_t tail;                          // 指向下一个要读取的位置
    uint8_t size;                          // 缓冲区的大小
} CircularRxBuffer;

void CircularRxBufferInit(void)
{
    CircularRxBuffer.head = 0;
    CircularRxBuffer.tail = 0;
    CircularRxBuffer.size = 0;
}
// 从环形缓存读取数据
uint8_t CircularRxBuffer_Read(uint8_t *data)
{
    if (CircularRxBuffer.size == 0)
        return 0; // 如果缓冲区为空，则返回false

    *data = CircularRxBuffer.Rx_buffer_5[CircularRxBuffer.head];
    // 缓冲区到达末尾
    if (CircularRxBuffer.head == data_buffer_size - 1)
    {
        CircularRxBuffer.head = 0;
    }
    else
    {
        CircularRxBuffer.head++;
    }
    CircularRxBuffer.size--;
    return 1;
}
// 写入数据到环形缓存
uint8_t CircularRxBuffer_Write(uint8_t data)
{
    // 写入数据
    CircularRxBuffer.Rx_buffer_5[CircularRxBuffer.tail] = data;
    // 缓冲区到达末尾
    if (CircularRxBuffer.tail == data_buffer_size - 1)
    {
        CircularRxBuffer.tail = 0;
    }
    else
    {
        CircularRxBuffer.tail++;
    }
    // 如果缓冲区已满
    if (CircularRxBuffer.tail == CircularRxBuffer.head)
    {
        if (CircularRxBuffer.head == data_buffer_size - 1)
        {
            CircularRxBuffer.head = 0;
        }
        else
        {
            CircularRxBuffer.head++;
        }
        CircularRxBuffer.size--;
    }
    CircularRxBuffer.size++;
    return 1;
}
// 清空环形缓存
void CircularRxBuffer_Clear(void)
{
    CircularRxBufferInit();
}
// 获得数据大小
uint8_t GetCircularRxBufferSize(void)
{
    return CircularRxBuffer.size;
}
/*****************************************环形缓冲区********************************************************* */

/*清空发送缓冲区*/
static void tx_buffer_clear(void)
{
    for (uint8_t i = 0; i < data_buffer_size; ++i)
    {
        Tx_buffer_5[i] = 0;
    }
}

static void send_data_assembly(void)
{
    tx_buffer_clear();
    uint8_t i = 0, data_temp[8];
    Tx_buffer_5[0] = 0xaa;
    Tx_buffer_5[1] = 0xa2;
    for (uint8_t j = 0; j < 8; ++j)
    {
        data_temp[j] = electronic_switch[j];
        data_temp[j] |= 0x80;
    }
    for (i = 2; i < 8 + 2; ++i)
    {
        Tx_buffer_5[i] = data_temp[i - 2];
    }
    Tx_buffer_5[i + 1] = 0xaf;
    Tx_buffer_5[i] = electronic_switch[0] + electronic_switch[2] + electronic_switch[7] + 0xaf;
    Tx_buffer_5[i + 2] = 0xff;
}
//数据发送——通过串口5
void switch_Process_Task(void *pvParameters)
{
    switch_send_flag = 0;
    for (;;)
    {
        while (switch_send_flag == 0)
        {
            send_data_assembly();
            HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
//数据解析
#include "queue.h"
uint8_t uart5_buf[power_board_max_len];
QueueHandle_t uart5_parse_queue;

void Uart5_Parse_Init(void)
{
    uart5_parse_queue = xQueueCreate(power_board_max_len,sizeof(uint8_t));
    if (uart5_parse_queue == NULL)
    {
        Error_Handler();
    }
    HAL_UART_Receive_DMA(&huart5,uart5_buf,power_board_max_len);
    __HAL_UART_ENABLE_IT(&huart5,UART_IT_IDLE);

}

void Uart5_Parse_Task(void *pvParameters)
{
    Power_board_Msg_t temp;
    for(;;)
    {
        if (xQueueReceive(uart5_parse_queue,&temp,portMAX_DELAY) == pdPASS)
        {
            //! 
            
        }
    }
}



/*数据接收处理*/
static void data_receive_process(void)
{
    uint8_t Rx_buffer_5[data_buffer_size] = {0};
    uint8_t length = 0;
    while (CircularRxBuffer_Read(&Rx_buffer_5[length]))
    {
        ++length;
    }
    uint8_t head = 0, tail = 0, data_head = 0, data_tail = 0, data_check = 0;
    for (uint8_t i = 0; i < length; ++i)
    {
        if (Rx_buffer_5[i] == 0xaa)
        {
            if (Rx_buffer_5[i + 1] == 0xa1 || Rx_buffer_5[i + 1] == 0xa3 || Rx_buffer_5[i + 1] == 0xa4 || Rx_buffer_5[i + 1] == 0xa5)
            {
                head = i;
                data_head = i + 1;
            }
        }
        if (Rx_buffer_5[i] == 0xff && Rx_buffer_5[i - 1] == 0xaf)
        {
            tail = i;
            data_tail = i - 1;
        }
    }
    // rt_kprintf("head:%d\t",head);
    // rt_kprintf("tail:%d\n",tail);
    if (tail != 0 && tail != head && head < tail)
    {
        // rt_kprintf("come in 1\n");
        if (Rx_buffer_5[data_head] == 0xa1) // adc
        {
            uint8_t data_temp[18] = {0};
            uint16_t data_adc_temp[9];
            for (uint8_t i = 0; i < 18; ++i)
            {
                data_temp[i] = Rx_buffer_5[i + data_head + 1];
            }
            data_check = data_temp[0] + data_temp[2] + data_temp[15] + Rx_buffer_5[data_tail];
            // rt_kprintf("here\n");
            if (data_check == Rx_buffer_5[data_tail - 1])
            {
                for (uint8_t i = 0; i < 9; ++i)
                {
                    data_adc_temp[i] = (uint16_t)(data_temp[2 * i] << 8) | (data_temp[2 * i + 1]);
                }
                if (data_adc_temp[4] == 0) // 电源电压，不可能为零；剔除异常数据
                {
                    data_adc_temp[4] = current_adc_data[4];
                }
                for (uint8_t i = 0; i < 9; ++i)
                {
                    current_adc_data[i] = data_adc_temp[i];
                }
                // multiple_average_filtering(current_adc_data);
                // rt_kprintf("come in\n");
                // rt_kprintf("%s",current_adc_data);
            }
            else
            {
                for (uint8_t i = 0; i < length; ++i)
                {
                    CircularRxBuffer_Write(Rx_buffer_5[i]);
                }
                return;
            }
        }
        if (Rx_buffer_5[data_head] == 0xa3) // shtc3
        {
            uint8_t data_temp[4] = {0};
            for (uint8_t i = 0; i < 4; ++i)
            {
                data_temp[i] = Rx_buffer_5[i + data_head + 1];
            }
            data_check = data_temp[0] + data_temp[3] + Rx_buffer_5[data_tail];
            // rt_kprintf("here\n");
            if (data_check == Rx_buffer_5[data_tail - 1])
            {
                RH_power_board = (uint16_t)(data_temp[0] << 8) | (data_temp[1]);
                temperature_power_board = (uint16_t)(data_temp[2] << 8) | (data_temp[3]);
                // rt_kprintf("come in\n");
                // rt_kprintf("rh:%d,tem:%d",(int)RH,(int)temperature);
            }
            else
            {
                for (uint8_t i = 0; i < length; ++i)
                {
                    CircularRxBuffer_Write(Rx_buffer_5[i]);
                }
                return;
            }
        }
        if (Rx_buffer_5[data_head] == 0xa4) // 活动校验
        {
            uint8_t data_temp = Rx_buffer_5[data_head + 1];
            data_check = data_temp + Rx_buffer_5[data_tail];
            if (data_check == Rx_buffer_5[data_tail - 1] && data_temp == 0xbb)
            {
                // rt_kprintf("here");
                // 发送活动校验
                tx_buffer_clear();
                Tx_buffer_5[0] = 0xaa;
                Tx_buffer_5[1] = 0xa4;
                Tx_buffer_5[2] = 0xcc;
                Tx_buffer_5[3] = (uint8_t)(0xcc + 0xaf);
                Tx_buffer_5[4] = 0xaf;
                Tx_buffer_5[5] = 0xff;
                HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
            }
            else
            {
                for (uint8_t i = 0; i < length; ++i)
                {
                    CircularRxBuffer_Write(Rx_buffer_5[i]);
                }
                return;
            }
        }
        if (Rx_buffer_5[data_head] == 0xa5) // 开关设置成功
        {
            // rt_kprintf("here1\n");
            uint8_t data_temp = Rx_buffer_5[data_head + 1];
            // rt_kprintf("%c\n",data_temp);
            data_check = data_temp + Rx_buffer_5[data_tail]; //
            if (data_check == Rx_buffer_5[data_tail - 1] && data_temp == 0x65)
            {
                switch_send_flag = 1;
                // rt_kprintf("%d",switch_send_flag);
                printf("switch set ok\n");
            }
            else
            {
                for (uint8_t i = 0; i < length; ++i)
                {
                    CircularRxBuffer_Write(Rx_buffer_5[i]);
                }
                return;
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < length; ++i)
        {
            CircularRxBuffer_Write(Rx_buffer_5[i]);
        }
        return;
    }
}

/*
 * 推进器电子开关设置
 * choose:0-7 set:1 or 0
 * */
void propeller_switch_set(uint8_t choose, uint8_t set)
{
    if (choose > 7)
    {
        printf("propeller_switch_set num err\n");
        return;
    }
    if (set != 0 && set != 1)
    {
        printf("propeller_switch_set set err\n");
        return;
    }
    if (choose < 7)
        electronic_switch[choose] = set;
    else
    {
        if (set == 1)
            electronic_switch[choose] |= 0x01; // set 1
        else
        {
            electronic_switch[choose] &= ~0x01; // set 0
        }
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}


/*舵机开关设置
 * 1-on 0-off
 * */
void server_switch_set(uint8_t data)
{
    uint8_t flag=(electronic_switch[7]>>1)&0x01;
    //进行比较判断，相同则直接返回，不同则改变
    if(flag==data)
        return ;
    //操作第二位
    if(data==0)
        electronic_switch[7]&=~(1<<1);//第二位置零
    else if (data==1)
    {
        electronic_switch[7]|=1<<1;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}

/*电机开关设置
 * 1-on 0-off
 * */
void motor_switch_set(uint8_t data)
{
    uint8_t flag=(electronic_switch[7]>>2)&0x01;
    //进行比较判断，相同则直接返回，不同则改变
    if(flag==data)
        return ;
    //操作第三位
    if(data==0)
        electronic_switch[7]&=~(1<<2);//第三位置零
    else if (data==1)
    {
        electronic_switch[7]|=1<<2;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}

/*电机转向设置
 * 1-反转 0-正转
 * */
void motor_dir_set(uint8_t data)
{
    //操作第4位
    if(data==0)
        electronic_switch[7]&=~(1<<3);//第4位置零xxxx 0xxx
    else if (data==1)
    {
        electronic_switch[7]|=1<<3;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}


/*
 * 探照灯电子开关
 * 1-on 0-off
 * */
void light_switch_set(uint8_t data)
{
    // 操作第5位
    if (data == 0)
        electronic_switch[7] &= ~(1 << 4); // 第5位置零xxx0 xxxx
    else if (data == 1)
    {
        electronic_switch[7] |= 1 << 4;
    }
    send_data_assembly();
    HAL_UART_Transmit_DMA(&huart5, Tx_buffer_5, sizeof(Tx_buffer_5));
}
