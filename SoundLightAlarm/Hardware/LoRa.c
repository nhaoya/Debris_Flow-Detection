/*
 * LoRa.c - 大夏龙雀 DX-LR32-433T22D LoRa 接收驱动（透明传输模式）
 *
 * 通过 USART2(PA2/PA3, 9600 8N1) 接收传感器节点数据，
 * 帧格式: {dev_id,type,event}
 *   dev_id: 节点标识(如 N1)
 *   type:   0=数据帧(心跳) 1=报警帧(事件开始/持续) 2=解除帧(事件结束)
 *   event:  事件字符串
 * 解析到完整帧后回调 LoRa_DataHandler()，由主程序处理上传与声光动作。
 */
#include "stm32f10x.h"                  // Device header
#include <string.h>
#include "usart2.h"
#include "LoRa.h"

#define LORA_RX_BUF       96
#define LORA_IDLE_POLLS   500     /* 帧尾无换行时，连续空闲 500 次循环后强制解析 */

static u8  rx_buf[LORA_RX_BUF];
static u8  rx_len = 0;
static u16 idle_cnt = 0;

/******************** 帧解析 ********************
 * 格式: {dev_id,type,event}
 *   例如 {N1,0,heartbeat}  数据帧(心跳)
 *   例如 {N1,1,landslide}  报警帧(事件开始/持续)
 *   例如 {N1,2,clear}      解除帧(事件结束)
 */
static void LoRa_ParseFrame(void)
{
    lora_frame_t f;
    u8 *p, *q, *end;
    u8 i, len;

    memset(&f, 0, sizeof(f));

    /* 搜索 '{' 定位帧头 */
    for(i = 0; i < rx_len; i++)
        if(rx_buf[i] == '{')
            break;
    if(i >= rx_len)
        return;
    end = rx_buf + rx_len;
    p = &rx_buf[i + 1];

    /* dev_id: 直到 ',' */
    q = p;
    while(q < end && *q != ',' && (q - p) < LORA_DEV_ID_MAX - 1)
        q++;
    if(q >= end)
        return;
    len = (u8)(q - p);
    memcpy(f.dev_id, p, len);
    f.dev_id[len] = '\0';
    p = q + 1;

    /* type: 直到 ','，0=数据 1=报警 2=解除 */
    q = p;
    while(q < end && *q != ',' && (q - p) < 2)
        q++;
    if(q >= end)
        return;
    if(q - p >= 1)
        f.type = (*p >= '0' && *p <= '9') ? (u8)(*p - '0') : 0;
    p = q + 1;

    /* event: 直到 '}' */
    q = p;
    while(q < end && *q != '}' && (q - p) < LORA_EVENT_MAX - 1)
        q++;
    if(q >= end)
        return;
    len = (u8)(q - p);
    memcpy(f.event, p, len);
    f.event[len] = '\0';

    LoRa_DataHandler(&f);
}

static void LoRa_Flush(void)
{
    if(rx_len > 0)
    {
        LoRa_ParseFrame();
        rx_len = 0;
    }
}

/******************** 对外接口 ********************/
void LoRa_Init(void)
{
    uart2_init(9600);               /* 模块默认波特率 9600 8N1 */
    rx_len = 0;
    idle_cnt = 0;
    UART2_RxFlush();
}

void LoRa_Task(void)
{
    u8 got = 0;

    while(UART2_RxAvailable() > 0)
    {
        u8 c = UART2_ReadByte();
        got = 1;

        if(c == '\n' || c == '\r')
        {
            LoRa_Flush();               /* 收到行结束符，立即解析 */
        }
        else if(rx_len < LORA_RX_BUF - 1)
        {
            rx_buf[rx_len++] = c;
        }
        else
        {
            LoRa_Flush();               /* 缓冲满，先解析再继续 */
        }
    }

    /* 帧尾无换行：空闲一段时间后解析（主循环需较快地调用本函数） */
    if(got)
        idle_cnt = 0;
    else if(rx_len > 0 && ++idle_cnt >= LORA_IDLE_POLLS)
    {
        LoRa_Flush();
        idle_cnt = 0;
    }
}

/* 帧回调默认实现：用户可在 main.c 中重新实现覆盖 */
__weak void LoRa_DataHandler(lora_frame_t *f)
{
    (void)f;
}
