#include "software_uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "tim.h"

/* ------- 全局对象 ------- */
SoftUART_HandleTypeDef su[SOFTUART_MAX_CH];

/* ------- 用户修改区：GPIO & EXTI 对应 ------- */
static void SU_GPIO_Config(void)
{
    /* 仅示例：PA0/PA1/PA2/PA3 做 TX，PB0/1/2/3 做 RX */
    su[0].tx_port = GPIOA; su[0].tx_pin = GPIO_PIN_0;
    su[0].rx_port = GPIOB; su[0].rx_pin = GPIO_PIN_0; su[0].rx_exti_line = EXTI_LINE_0;

    su[1].tx_port = GPIOA; su[1].tx_pin = GPIO_PIN_1;
    su[1].rx_port = GPIOB; su[1].rx_pin = GPIO_PIN_1; su[1].rx_exti_line = EXTI_LINE_1;

}
/* ------------------------ */

void SoftUART_Init(void)
{
    SU_GPIO_Config();
    /* 默认 TX 空闲 = 1 */
    for (int i = 0; i < SOFTUART_MAX_CH; i++)
    {
        HAL_GPIO_WritePin(su[i].tx_port, su[i].tx_pin, GPIO_PIN_SET);
        su[i].state = SU_IDLE;
        su[i].tx_head = su[i].tx_tail = su[i].tx_cnt = 0;
        su[i].rx_head = su[i].rx_tail = su[i].rx_cnt = 0;
    }
    /* 使能定时器 */
    HAL_TIM_Base_Start_IT(&htim4);
}

/* ---------- 发送队列接口 ---------- */
void SoftUART_PutChar(uint8_t ch, uint8_t idx)
{
    if (idx >= SOFTUART_MAX_CH) return;
    if (su[idx].tx_cnt >= sizeof(su[idx].tx_buf)) return; /* 满 */

    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    su[idx].tx_buf[su[idx].tx_head++] = ch;
    su[idx].tx_head &= (sizeof(su[idx].tx_buf) - 1);
    su[idx].tx_cnt++;
    __set_PRIMASK(prim);
}

/* ---------- 取 1 字节 ---------- */
int SoftUART_GetChar(uint8_t *pch, uint8_t idx)
{
    if (idx >= SOFTUART_MAX_CH || su[idx].rx_cnt == 0) return 0;
    uint32_t prim = __get_PRIMASK();
    __disable_irq();
    *pch = su[idx].rx_buf[su[idx].rx_tail++];
    su[idx].rx_tail &= (sizeof(su[idx].rx_buf) - 1);
    su[idx].rx_cnt--;
    __set_PRIMASK(prim);
    return 1;
}

/* ========== TIM4 中断回调 ========== */
void My_TIM4_IRQHandler(void)
{
    for (uint8_t i = 0; i < SOFTUART_MAX_CH; i++)
    {
        switch (su[i].state)
        {
        case SU_IDLE:     /* 空闲：有数据就启动发送 */
            if (su[i].tx_cnt)
            {
                su[i].cur_byte = su[i].tx_buf[su[i].tx_tail++];
                su[i].tx_tail &= (sizeof(su[i].tx_buf) - 1);
                su[i].tx_cnt--;
                su[i].bit_cnt = 0;
                HAL_GPIO_WritePin(su[i].tx_port, su[i].tx_pin, GPIO_PIN_RESET); /* 起始位0 */
                su[i].state = SU_TX;
            }
            break;

        case SU_TX:       /* 正在发送 */
            su[i].bit_cnt++;
            if (su[i].bit_cnt <= 8)
            {   /* 发送数据位 LSB→MSB */
                uint8_t bit = (su[i].cur_byte >> (su[i].bit_cnt - 1)) & 0x01;
                HAL_GPIO_WritePin(su[i].tx_port, su[i].tx_pin,
                                  bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            else if (su[i].bit_cnt == 9)
            {   /* 停止位 1 */
                HAL_GPIO_WritePin(su[i].tx_port, su[i].tx_pin, GPIO_PIN_SET);
            }
            else
            {   /* 10 位结束 */
                su[i].state = SU_IDLE;
            }
            break;

        case SU_RX_START: /* 起始半位 -> 等 0.5 bit 再采样 */
            su[i].bit_cnt++;
            if (su[i].bit_cnt == 1)
            {
                su[i].bit_cnt = 0;
                su[i].state = SU_RX_DATA;
            }
            break;

        case SU_RX_DATA:  /* 按 1 bit 周期采样 */
            su[i].bit_cnt++;
            if (su[i].bit_cnt <= 8)
            {
                uint8_t bit = HAL_GPIO_ReadPin(su[i].rx_port, su[i].rx_pin);
                su[i].cur_byte |= (bit << (su[i].bit_cnt - 1));
            }
            else if (su[i].bit_cnt == 9)
            {   /* 停止位：可检查 */
            }
            else
            {   /* 一帧结束，推入 FIFO */
                if (su[i].rx_cnt < sizeof(su[i].rx_buf))
                {
                    su[i].rx_buf[su[i].rx_head++] = su[i].cur_byte;
                    su[i].rx_head &= (sizeof(su[i].rx_buf) - 1);
                    su[i].rx_cnt++;
                }
                su[i].state = SU_IDLE;
            }
            break;
        default: break;
        }
    }
}

/* ========== EXTI 回调：检测起始位 ========== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    for (uint8_t i = 0; i < SOFTUART_MAX_CH; i++)
    {
        if (GPIO_Pin == su[i].rx_pin && su[i].state == SU_IDLE)
        {
            /* 下降沿触发：检测到起始位 */
            su[i].state   = SU_RX_START;
            su[i].bit_cnt = 0;
            su[i].cur_byte = 0;
            /* 把定时器计数器装到 half bit 延迟处 ≈ 0.5 bit */
            __HAL_TIM_SET_COUNTER(&htim4, (TIM4->ARR + 1) / 2);
        }
    }
}
