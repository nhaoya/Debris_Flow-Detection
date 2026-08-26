#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include "Delay.h"
#include "LED.h"
#include "usart.h"
#include "usart3.h"
#include "JQ8X00.h"
#include "LoRa.h"

/* ================= 灯条控制 =================
 * 12V 红蓝灯条经 NCK4080K MOS 低边开关接 PA1：
 *   MOS 栅极高电平 = 导通 = 灯条亮
 * 如果你的接法是"低电平亮"（或直接接的灯珠），把下面改为 0
 */
#define LIGHTBAR_ACTIVE_HIGH 1

static void LightBar_Set(u8 on)
{
    if(on)
    {
#if LIGHTBAR_ACTIVE_HIGH
        GPIO_SetBits(GPIOA, GPIO_Pin_1);      /* 灯条亮 */
#else
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);
#endif
    }
    else
    {
#if LIGHTBAR_ACTIVE_HIGH
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);    /* 灯条灭 */
#else
        GPIO_SetBits(GPIOA, GPIO_Pin_1);
#endif
    }
}

/* ================= 报警声光控制 =================
 * 报警持续期间用语音模块"单曲循环"模式一直循环播放，
 * 收到解除帧(type=2)才停止。
 */
static u8 alarm_active = 0;     /* 报警进行中标志 */

static void Alarm_Start(void)
{
    if(alarm_active)
        return;                                 /* 已在报警中，不重复触发 */
    alarm_active = 1;
    JQ8x00_Command_Data(SetLoopMode, SingleCycle);  /* 单曲循环 */
    JQ8x00_Command_Data(AppointTrack, 1);           /* 播放报警语音(00001.mp3) */
    LightBar_Set(1);                                /* 灯条亮 */
}

static void Alarm_Stop(void)
{
    if(!alarm_active)
        return;
    alarm_active = 0;
    JQ8x00_Command(Stop);                           /* 停止播报 */
    JQ8x00_Command_Data(SetLoopMode, SingleStop);   /* 恢复单曲停止 */
    LightBar_Set(0);                                /* 灯条灭 */
}

/* ================= M100MG-B1 DTU 透传上传 =================
 * DTU 已在银尔达平台(dtu.yinerda.com)配置好 EMQX MQTT 连接：
 *   主机 m047bb7f.ala.dedicated.aliyun.emqxcloud.cn:1883
 *   账号 test01 / 123456，发布 topic = abc
 * 本机只需把数据帧通过串口发给 DTU，DTU 自动透传发布到 MQTT，后台订阅 abc 即可收到。
 */
#define DTU_BAUD  115200     /* 与 DTU TTL 串口波特率一致（之前 GPS 项目实测默认 115200） */

/* 组帧并发送: {dev_id,type,event}\r\n */
static void Dtu_Upload(lora_frame_t *f)
{
    char buf[48];
    u8 len = (u8)sprintf(buf, "{%s,%d,%s}\r\n", f->dev_id, (int)f->type, f->event);
    UART3_SendCode((u8 *)buf, len);
}

/* ================= LoRa 数据回调 =================
 * 帧格式: {dev_id,type,event}
 *   type=0 数据帧(心跳)：只上传后台，本地不动作
 *   type=1 报警帧(事件开始/持续)：上传后台 + 本地响喇叭报警
 *   type=2 解除帧(事件结束)：上传后台 + 本地停止播报
 * "是否全部节点报警"由后台根据收到的数据判断。
 */
void LoRa_DataHandler(lora_frame_t *f)
{
    /* 1. 上传后台（DTU 透传 -> MQTT） */
    Dtu_Upload(f);

    /* 2. 本地声光动作 */
    if(f->type == 1)
    {
        Alarm_Start();
    }
    else if(f->type == 2)
    {
        Alarm_Stop();
    }
    /* type=0 心跳帧：仅上传 */
}

int main(void)
{	
    LED_Init();
    JQ8x00_Init();//语音模块初始化

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    uart_init(9600);	 //串口初始化为9600
    uart3_init(DTU_BAUD);   //M100MG-B1 DTU 串口（透传上传）
    LoRa_Init();        //LoRa 模块初始化（USART2，透明接收）
    Delay_ms(500);              //等待模块稳定
    JQ8x00_Command_Data(SetVolume,30);         //设置音量，最大为30
    Delay_ms(10);                               //连续发送指令，加入一定的延时等待模块处理完成上一条指令
	while(1)
	{
		   LoRa_Task();          //轮询 LoRa 接收并解析
	}
}
