#include "ms5837_uart.h"
#include "usart.h"    // CubeMX ���ɵ� huart4 ���
#include "stm32f4xx_hal.h"
#include <string.h>



/* Э����¶ȡ�����ǰ׺�� GBK ���� */
static const uint8_t PREFIX_TEMP[4]  = {0xCE,0xC2,0xB6,0xC8};
/* Э�����ȡ�����ǰ׺�� GBK ���� */
static const uint8_t PREFIX_DEPTH[4] = {0xC9,0xEE,0xB6,0xC8};

static uint8_t dma_rx_buf[UART4_DMA_BUF_SIZE];

/* ������� */
float ms5837_temperature = 0.0f;
float ms5837_depth       = 0.0f;
float ms5837_pressure = 0.0f; 
/**
 * @brief  �� buf ���ԡ�ǰ׺ + ':' + ASCII ���� + '.' + ASCII ���֡���ʽ����һ����ֵ
 * @param  buf   ��ʼ��ַ
 * @param  len   buf �ɶ�����
 * @param  prefix 4 �ֽ�ǰ׺����
 * @param  out   ����������ţ���ȥ 0x30��
 * @return  >=0 �ɹ���������д�� *out���������ĵ��ֽ��������򷵻� -1
 */
static int parse_value(const uint8_t *buf, int len,
                       const uint8_t prefix[4],
                       float *out)
{
    int i = 0;
    // ��ǰ׺
    for (; i + 5 < len; i++)
    {
        if (memcmp(buf + i, prefix, 4) == 0 && buf[i+4] == 0x3A /*':'*/)
            break;
    }
    if (i + 5 >= len) return -1;
    int j = i + 5;
    int sign = 1;
    int iv = 0, fv = 0, fd = 0;
    // ��������
    while (j < len && buf[j] != 0x2E /*'.'*/)
    {
        if (buf[j] == 0x2D /*'-'*/) { sign = -1; }
        else if (buf[j] >= '0' && buf[j] <= '9')
        {
            iv = iv * 10 + (buf[j] - '0');
        }
        else break;
        j++;
    }
    if (j >= len || buf[j] != 0x2E) return -1;
    j++; // ���� '.'
    // С����λ
    for (fd = 0; fd < 2 && j < len; fd++, j++)
    {
        if (buf[j] >= '0' && buf[j] <= '9')
            fv = fv * 10 + (buf[j] - '0');
        else break;
    }
    // ��װ
    *out = sign * (iv + fv / (float)(fd == 0 ? 1 : (fd==1?10:100)));
    return j - i;
}

/**
 * @brief  ����һ�� DMA ���ݺ�һ���Ե��ã���ȡ���¶ȡ��͡���ȡ�
 */
void parse_frame(const uint8_t *buf, int len)
{
    // �����¶�
    parse_value(buf, len, PREFIX_TEMP, &ms5837_temperature);
    // �������
    parse_value(buf, len, PREFIX_DEPTH, &ms5837_depth);
}

/**
 * @brief  ��ʼ�������� DMA + �����жϽ���
 */
void Parser4_Init(void)
{
    // ���� DMA ����
    HAL_UART_Receive_DMA(&huart4, dma_rx_buf, UART4_DMA_BUF_SIZE);
    // ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
}


