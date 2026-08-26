#ifndef __USART3_H
#define __USART3_H

void uart3_init(u32 bound);              //USART3初始化(PB10 TX / PB11 RX)，接 M100MG-B1 DTU
void UART3_SendCode(u8 *DATA, u8 len);   //发送
void UART3_SendString(const char *s);    //发送字符串
u16  UART3_RxAvailable(void);            //接收缓冲未读字节数
u8   UART3_ReadByte(void);               //读取一个字节
void UART3_RxFlush(void);                //清空接收缓冲

#endif
