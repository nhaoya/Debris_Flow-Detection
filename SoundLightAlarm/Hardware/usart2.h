#ifndef __USART2_H
#define __USART2_H

void uart2_init(u32 bound);              //USART2初始化(PA2 TX / PA3 RX)，供 LoRa 使用
void UART2_SendCode(u8 *DATA, u8 len);   //发送
u16  UART2_RxAvailable(void);            //接收缓冲未读字节数
u8   UART2_ReadByte(void);               //读取一个字节
void UART2_RxFlush(void);                //清空接收缓冲

#endif
