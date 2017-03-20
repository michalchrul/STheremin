#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

int main(void)
{
	SystemInit();

	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2| GPIO_Pin_3 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//int x=0;
	int n = 0;
	int a = 0, b= 0, c = 0, d = 0;
	int i;

int main(void)
{

	while(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1));
			if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1))
			{
				a++;
				if(a==1)
				{
					GPIO_ResetBits(GPIOD,GPIO_Pin_15);
					GPIO_SetBits(GPIOD,GPIO_Pin_12);
				}
				else if(a==2)
				{
					GPIO_ResetBits(GPIOD,GPIO_Pin_12);
					GPIO_SetBits(GPIOD,GPIO_Pin_13);
				}
				else if(a==3)
				{
					GPIO_ResetBits(GPIOD,GPIO_Pin_13);
					GPIO_SetBits(GPIOD,GPIO_Pin_14);
				}
				else if(a==4)
				{
					GPIO_ResetBits(GPIOD,GPIO_Pin_14);
					GPIO_SetBits(GPIOD,GPIO_Pin_15);
					a=0;
				}
				else
				{a=0;}

				while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1));
				for(i=0;i<999999;i++);
			}
}}
