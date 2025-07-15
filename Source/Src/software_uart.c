#include "software_uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "tim.h"

#define SU_TIMER_HANDLE (&htim4)
#define SU_TIMER_CHANNEL TIM_CHANNEL_1

/* 每路软串口的内部状态 */
typedef enum
{
    SU_IDLE,
    SU_RX,
    SU_TX
} SU_State_t;

typedef struct
{
    /* 用户配置 */
    GPIO_TypeDef *rxPort;
    uint16_t rxPin;
    GPIO_TypeDef *txPort;
    uint16_t txPin;
    uint32_t bitTime; /* 定时器滴答数 */

    /* FreeRTOS 同步 */
    QueueHandle_t rxQueue;
    SemaphoreHandle_t txMutex;

    /* RX 状态 */
    SU_State_t state;
    uint8_t rxByte;
    uint8_t rxBitIdx;

    /* TX 状态 */
    uint8_t txBuf[SU_TX_BUF_LEN];
    uint8_t txHead, txTail;
    uint8_t txByte;
    uint8_t txBitIdx;
} SoftUART_Channel_t;

static SoftUART_Channel_t su_ch[SOFTUART_MAX_CHANNELS];
static uint8_t su_count = 0;

/* Helper: 获取当前定时器计数 */
static inline uint32_t SU_TimerNow(void)
{
    return SU_TIMER_HANDLE->Instance->CNT;
}

/* 初始化软串口 */
SoftUART_Handle_t SoftUART_Init(const SoftUART_Config_t *cfg)
{
    configASSERT(su_count < SOFTUART_MAX_CHANNELS);
    SoftUART_Channel_t *ch = &su_ch[su_count];

    /* 复制配置 */
    ch->rxPort = cfg->rxPort;
    ch->rxPin = cfg->rxPin;
    ch->txPort = cfg->txPort;
    ch->txPin = cfg->txPin;
    /* 计算 bitTime = timer_clock / baudrate */
    uint32_t clk = HAL_RCC_GetPCLK1Freq() * 2; // APB1 x2
    ch->bitTime = clk / cfg->baudrate;

    /* 初始化状态机 */
    ch->state = SU_IDLE;
    ch->rxBitIdx = 0;
    ch->txHead = ch->txTail = 0;

    /* 创建 FreeRTOS 对象 */
    ch->rxQueue = xQueueCreate(SU_RX_QUEUE_LEN, sizeof(uint8_t));
    ch->txMutex = xSemaphoreCreateMutex();
    configASSERT(ch->rxQueue && ch->txMutex);

    /* 配置 TX 引脚为推挽高电平（空闲）——CubeMX 预先设置即可 */
    HAL_GPIO_WritePin(ch->txPort, ch->txPin, GPIO_PIN_SET);

    /* 第一次注册时，启动 Timer 并使能 CC 中断 */
    if (su_count == 0)
    {
        HAL_TIM_Base_Start(SU_TIMER_HANDLE);
        __HAL_TIM_CLEAR_FLAG(SU_TIMER_HANDLE, TIM_FLAG_CC1);
        __HAL_TIM_DISABLE_IT(SU_TIMER_HANDLE, TIM_IT_CC1);
        HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }

    /* 外部中断 RX 需要用户在 CubeMX 或代码中配置好 */
    /* EXTI 回调会调用 HAL_GPIO_EXTI_Callback */

    return su_count++;
}

/* 发送一个字节到 TX 缓冲 */
HAL_StatusTypeDef SoftUART_SendByte(SoftUART_Handle_t h, uint8_t b)
{
    if (h >= su_count)
        return HAL_ERROR;
    SoftUART_Channel_t *ch = &su_ch[h];

    if (xSemaphoreTake(ch->txMutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return HAL_ERROR;

    /* 环形写入 */
    uint8_t next = (ch->txHead + 1) % SU_TX_BUF_LEN;
    if (next == ch->txTail)
    {
        xSemaphoreGive(ch->txMutex);
        return HAL_ERROR; /* 满 */
    }
    ch->txBuf[ch->txHead] = b;
    ch->txHead = next;

    /* 若空闲，立刻开始发送 */
    if (ch->state == SU_IDLE)
    {
        ch->state = SU_TX;
        ch->txBitIdx = 0;
        ch->txByte = b;
        /* 起始位 */
        HAL_GPIO_WritePin(ch->txPort, ch->txPin, GPIO_PIN_RESET);
        /* 安排下一个 bit 时间 */
        uint32_t t = SU_TimerNow() + ch->bitTime;
        __HAL_TIM_SET_COMPARE(SU_TIMER_HANDLE, SU_TIMER_CHANNEL, t);
        __HAL_TIM_ENABLE_IT(SU_TIMER_HANDLE, TIM_IT_CC1);
    }
    xSemaphoreGive(ch->txMutex);
    return HAL_OK;
}

/* 接收一个字节（带超时） */
uint8_t SoftUART_ReceiveByte(SoftUART_Handle_t h, uint8_t *out, uint32_t tmo)
{
    if (h >= su_count)
        return 0;
    if (xQueueReceive(su_ch[h].rxQueue, out, pdMS_TO_TICKS(tmo)) == pdTRUE)
        return 1;
    return 0;
}

/* 外部中断回调：检测到 RX 起始位下降沿 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (int i = 0; i < su_count; i++)
    {
        SoftUART_Channel_t *ch = &su_ch[i];
        if (GPIO_Pin == ch->rxPin && ch->state == SU_IDLE)
        {
            ch->state = SU_RX;
            ch->rxBitIdx = 0;
            ch->rxByte = 0;
            /* 安排半个 bit 后采样首位 */
            uint32_t t = SU_TimerNow() + ch->bitTime / 2;
            __HAL_TIM_SET_COMPARE(SU_TIMER_HANDLE, SU_TIMER_CHANNEL, t);
            __HAL_TIM_ENABLE_IT(SU_TIMER_HANDLE, TIM_IT_CC1);
            break;
        }
    }
}

/* 定时器中断：按位处理所有通道的 RX/TX */
void My_TIM4_IRQHandler(void)
{
    if (!__HAL_TIM_GET_FLAG(SU_TIMER_HANDLE, TIM_FLAG_CC1))
        return;
    __HAL_TIM_CLEAR_IT(SU_TIMER_HANDLE, TIM_IT_CC1);
    uint32_t now = SU_TimerNow();

    /* RX 处理 */
    for (int i = 0; i < su_count; i++)
    {
        SoftUART_Channel_t *ch = &su_ch[i];
        if (ch->state == SU_RX)
        {
            /* 采样 */
            GPIO_PinState bit = HAL_GPIO_ReadPin(ch->rxPort, ch->rxPin);
            ch->rxByte |= (bit == GPIO_PIN_SET) << ch->rxBitIdx;
            ch->rxBitIdx++;
            if (ch->rxBitIdx >= 8)
            {
                /* 完成一个字节 */
                ch->state = SU_IDLE;
                BaseType_t w = pdFALSE;
                xQueueSendFromISR(ch->rxQueue, &ch->rxByte, &w);
                portYIELD_FROM_ISR(w);
            }
            else
            {
                /* 下一个 bit */
                __HAL_TIM_SET_COMPARE(SU_TIMER_HANDLE, SU_TIMER_CHANNEL, now + ch->bitTime);
            }
        }
    }

    /* TX 处理 */
    for (int i = 0; i < su_count; i++)
    {
        SoftUART_Channel_t *ch = &su_ch[i];
        if (ch->state == SU_TX)
        {
            ch->txBitIdx++;
            if (ch->txBitIdx < 8)
            {
                /* 输出数据位 */
                uint8_t b = (ch->txByte >> ch->txBitIdx) & 1;
                HAL_GPIO_WritePin(ch->txPort, ch->txPin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
                __HAL_TIM_SET_COMPARE(SU_TIMER_HANDLE, SU_TIMER_CHANNEL, now + ch->bitTime);
            }
            else if (ch->txBitIdx == 8)
            {
                /* 输出停止位 */
                HAL_GPIO_WritePin(ch->txPort, ch->txPin, GPIO_PIN_SET);
                __HAL_TIM_SET_COMPARE(SU_TIMER_HANDLE, SU_TIMER_CHANNEL, now + ch->bitTime);
            }
            else
            {
                /* 检查下一个字节 */
                xSemaphoreTakeFromISR(ch->txMutex, NULL);
                if (ch->txHead != ch->txTail)
                {
                    ch->txByte = ch->txBuf[ch->txTail++];
                    if (ch->txTail >= SU_TX_BUF_LEN)
                        ch->txTail = 0;
                    ch->txBitIdx = 0;
                    HAL_GPIO_WritePin(ch->txPort, ch->txPin, GPIO_PIN_RESET);
                    __HAL_TIM_SET_COMPARE(SU_TIMER_HANDLE, SU_TIMER_CHANNEL, now + ch->bitTime);
                }
                else
                {
                    /* 无数据，回空闲 */
                    ch->state = SU_IDLE;
                    __HAL_TIM_DISABLE_IT(SU_TIMER_HANDLE, TIM_IT_CC1);
                }
                xSemaphoreGiveFromISR(ch->txMutex, NULL);
            }
        }
    }
}
