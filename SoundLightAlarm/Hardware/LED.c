/*
 * LED.c - 指示灯驱动（低电平点亮）
 *
 * LED1 = PA1：当前作为 12V 灯条 MOS 控制脚，
 *         灯条实际极性见 main.c 的 LIGHTBAR_ACTIVE_HIGH（高电平亮）。
 * LED2 = PB0：预留/调试，当前未使用。
 */
#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;//结构体变量
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_1;//LED1 = PA1
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;//输出速度
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化
	
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0;//LED2 = PB0
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化
	
	GPIO_SetBits(GPIOA,GPIO_Pin_1);//LED1关闭状态
	GPIO_SetBits(GPIOB,GPIO_Pin_0);//LED2关闭状态
}

void LED1_ON(void)//低电平亮
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}

void LED1_OFF(void)//高电平灭
{
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
}

void LED1_Turn(void)//取返
{
	if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_1)==0)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_1);//高电平灭
	}
	else
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);//低电平亮
	}
}

void LED2_ON(void)//低电平亮
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}

void LED2_OFF(void)//高电平灭
{
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
}

void LED2_Turn(void)//取返
{
	if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)==0)
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_0);//高电平灭
	}
	else
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_0);//低电平亮
	}
}

