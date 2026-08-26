#ifndef __LORA_H
#define __LORA_H

/* 大夏龙雀 DX-LR32-433T22D 接收端驱动（透明传输模式 MODE0）
 *
 * 帧格式: {dev_id,type,event}
 *   dev_id: 设备节点标识，如 N1
 *   type:   0=数据帧(心跳) 1=报警帧(事件开始/持续) 2=解除帧(事件结束)
 *   event:  事件字符串
 */

#define LORA_DEV_ID_MAX   8
#define LORA_EVENT_MAX    24

typedef struct {
    char dev_id[LORA_DEV_ID_MAX];   /* 节点标识 */
    u8   type;                      /* 0=数据 1=报警 2=解除 */
    char event[LORA_EVENT_MAX];     /* 事件字符串 */
} lora_frame_t;

void LoRa_Init(void);               /* USART2 初始化（9600 8N1） */
void LoRa_Task(void);               /* 主循环周期调用：收帧解析 */
void LoRa_DataHandler(lora_frame_t *f);  /* 解析到一帧时回调(在 main.c 实现) */

#endif
