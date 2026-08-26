/*
 * usart2.c - USART2 驱动（接 DX-LR32 LoRa 模块）
 *
 * PA2=TX、PA3=RX，9600 8N1，中断接收 + 环形缓冲。
 */
#include "stm32f10x.h"                  // Device header
#include "usart2.h"

#define UART2_RX_BUF_SIZE 128

static u8  rx_buf[UART2_RX_BUF_SIZE];
static volatile u16 rx_head = 0;
static volatile u16 rx_tail = 0;

void uart2_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   //GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //USART2时钟

    //USART2_TX   GPIOA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART2_RX   GPIOA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //USART2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
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

    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);   //开启接收中断
    USART_Cmd(USART2, ENABLE);                       //使能串口2

    rx_head = 0;
    rx_tail = 0;
}

void UART2_SendCode(u8 *DATA, u8 len)
{
    USART_ClearFlag(USART2, USART_FLAG_TC);
    while(len--)
    {
        USART_SendData(USART2, *DATA++);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) != SET);
    }
}

u16 UART2_RxAvailable(void)
{
    return (u16)((rx_head - rx_tail + UART2_RX_BUF_SIZE) % UART2_RX_BUF_SIZE);
}

u8 UART2_ReadByte(void)
{
    u8 c = rx_buf[rx_tail];
    rx_tail = (u16)((rx_tail + 1) % UART2_RX_BUF_SIZE);
    return c;
}

void UART2_RxFlush(void)
{
    rx_head = 0;
    rx_tail = 0;
}

void USART2_IRQHandler(void)              //串口2中断服务程序
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        u8 c = (u8)USART_ReceiveData(USART2);
        u16 next = (u16)((rx_head + 1) % UART2_RX_BUF_SIZE);
        if(next != rx_tail)               //缓冲未满则写入
        {
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
}
