/****************************************Copyright (c)****************************************************
** 
**                                      
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			main.c
** modified Date:  		2017-6-20
** Last Version:		V0.1
** Descriptions:		  main ��������
**
** ������Ƽ���оƬ��ҵ��----��������Ӧ�÷ֲ�
*********************************************************************************************************/
#include "HAL_conf.h"



/********************************************************************************************************
**������Ϣ ��int main (void)                          
**�������� ��
**������� ��
**������� ��
********************************************************************************************************/

int main(void)
{
	uint32_t i ,j;
	GPIO_InitTypeDef GPIO_InitStructure;//����GPIO��ʼ���ṹ�����

	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

	//��������LED��GPIOΪ�������ģʽ
	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	while(1)//����ѭ�� 
	{
		GPIO_SetBits(GPIOD, GPIO_Pin_8); //PC8����ߵ�ƽ������LED
		for(i=0;i<2000000;i++);//��ʱ
		
		GPIO_ResetBits(GPIOD, GPIO_Pin_8);//PD8����͵�ƽ��Ϩ��LED
		for(i=0;i<2000000;i++);//��ʱ
	}

}



