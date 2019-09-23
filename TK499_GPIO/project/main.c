/****************************************Copyright (c)****************************************************
** 
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			main.c
** modified Date:  		2017-6-20
** Last Version:		V0.1
** Descriptions:		  main 函数调用
**
** 好钜润科技，芯片事业部----深圳龙华应用分部
*********************************************************************************************************/
#include "HAL_conf.h"



/********************************************************************************************************
**函数信息 ：int main (void)                          
**功能描述 ：
**输入参数 ：
**输出参数 ：
********************************************************************************************************/

int main(void)
{
	uint32_t i ,j;
	GPIO_InitTypeDef GPIO_InitStructure;//定义GPIO初始化结构体变量

	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	//配置连接LED的GPIO为推挽输出模式
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	while(1)//无限循环 
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_8); //PC8输出高电平，点亮LED
		for(i=0;i<2000000;i++);//延时
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_8);//PD8输出低电平，熄灭LED
		for(i=0;i<2000000;i++);//延时
	}

}



