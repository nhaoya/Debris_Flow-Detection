/*
 * usart3.c - USART3 驱动（接 M100MG-B1 DTU，透传上传）
 *
 * PB10=TX、PB11=RX，115200 8N1（与 DTU TTL 波特率一致），
 * 中断接收 + 环形缓冲，用于把数据帧透传给 DTU 发布到 MQTT。
 */
#include "stm32f10x.h"                  // Device header
#include <string.h>
#include "usart3.h"

#define UART3_RX_BUF_SIZE 128

static u8  rx_buf[UART3_RX_BUF_SIZE];
static volatile u16 rx_head = 0;
static volatile u16 rx_tail = 0;

void uart3_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  //USART3时钟

    //USART3_TX   GPIOB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //USART3_RX   GPIOB.11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);   //开启接收中断
    USART_Cmd(USART3, ENABLE);

    rx_head = 0;
    rx_tail = 0;
}

void UART3_SendCode(u8 *DATA, u8 len)
{
    USART_ClearFlag(USART3, USART_FLAG_TC);
    while(len--)
    {
        USART_SendData(USART3, *DATA++);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TC) != SET);
    }
}

void UART3_SendString(const char *s)
{
    UART3_SendCode((u8 *)s, (u8)strlen(s));
}

u16 UART3_RxAvailable(void)
{
    return (u16)((rx_head - rx_tail + UART3_RX_BUF_SIZE) % UART3_RX_BUF_SIZE);
}

u8 UART3_ReadByte(void)
{
    u8 c = rx_buf[rx_tail];
    rx_tail = (u16)((rx_tail + 1) % UART3_RX_BUF_SIZE);
    return c;
}

void UART3_RxFlush(void)
{
    rx_head = 0;
    rx_tail = 0;
}

void USART3_IRQHandler(void)              //串口3中断服务程序
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        u8 c = (u8)USART_ReceiveData(USART3);
        u16 next = (u16)((rx_head + 1) % UART3_RX_BUF_SIZE);
        if(next != rx_tail)               //缓冲未满则写入
        {
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
}
