#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"

#include "tm_stm32f4_delay.c"
#include "tm_stm32f4_delay.h"

/*
 	Additional connections used:
		Sensor #1: Trigger PA0, Echo PA1
		Sensor #2: Trigger PA2, Echo PA3
*/

//Global variables
uint16_t TIM3_CNT, TIM4_CNT;
float Time_1, Time_2;
float Distance_1, Distance_2;

void TIM2_Config()
{

	//Timer responsible for sending the Trigger signals every 100ms

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 8399;
	TIM_TimeBaseStructure.TIM_Prescaler = 999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
}

void TIM_2_Interruption_Config()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		EXTI_Pin_1_Enable();
		EXTI_Pin_3_Enable();

		//Triger for Sensor #1
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
		Delay(2);
		GPIO_SetBits(GPIOA, GPIO_Pin_0);
		Delay(10);
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);

		//Triger for Sensor #2
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
		Delay(2);
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
		Delay(10);
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);

		TIM_SetCounter(TIM3, 0);
		TIM_SetCounter(TIM4, 0);

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void TIM3_Config()
{

	//Timer responsible for measuring the time of the Echo signal return (Sensor #1)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 42;
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM3, DISABLE);
}

void TIM4_Config()
{

	//Timer responsible for measuring the time of the Echo signal return (Sensor #2)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 42;
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM4, DISABLE);
}


void TIM5_Config(uint32_t PeriodValue)
{

	//Timer responsible for sending the sine sample to DAC (PeriodValue depends on the distance values)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = PeriodValue;
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM5, ENABLE);

}

void TIM_5_Interruption_Config()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
}

void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		//Send the sample here

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

void GPIOA_Init()
{

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	//Trigger signal pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Echo signal pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

//EXTI for [GPIO A Pin_1] Echo signal

		void EXTI_Pin_1_Enable()
		{

			EXTI_InitTypeDef EXTI_InitStructure;
			EXTI_InitStructure.EXTI_Line = EXTI_Line1;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);

			SYSCFG_EXTILineConfig(GPIOA, EXTI_PinSource1);
		}

		void EXTI_Pin_1_Disable()
		{
			EXTI_InitTypeDef EXTI_InitStructure;
			EXTI_InitStructure.EXTI_Line = EXTI_Line1;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			EXTI_InitStructure.EXTI_LineCmd = DISABLE;
			EXTI_Init(&EXTI_InitStructure);

			SYSCFG_EXTILineConfig(GPIOA, EXTI_PinSource1);
		}

		void EXTI_Pin_1_Init()
		{

			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			EXTI_Pin_1_Disable();
		}

		void EXTI1_IRQHandler(void)
		{
			if(EXTI_GetITStatus(EXTI_Line1) != RESET)
			{
				if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
				{
					TIM_Cmd(TIM3, ENABLE);

				}
				else if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
				{
					TIM_Cmd(TIM3, DISABLE);
					TIM3_CNT = TIM3->CNT;
					Distance_1 = (float)TIM3_CNT / 58;
					EXTI_Pin_1_Disable();
					EXTI_ClearITPendingBit(EXTI_Line1);
				}
				EXTI_ClearITPendingBit(EXTI_Line1);
			}

		}

//EXTI for [GPIO A Pin_3] Echo signal

		void EXTI_Pin_3_Enable()
		{

			EXTI_InitTypeDef EXTI_InitStructure;
			EXTI_InitStructure.EXTI_Line = EXTI_Line3;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);

			SYSCFG_EXTILineConfig(GPIOA, EXTI_PinSource3);
		}

		void EXTI_Pin_3_Disable()
		{
			EXTI_InitTypeDef EXTI_InitStructure;
			EXTI_InitStructure.EXTI_Line = EXTI_Line3;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			EXTI_InitStructure.EXTI_LineCmd = DISABLE;
			EXTI_Init(&EXTI_InitStructure);

			SYSCFG_EXTILineConfig(GPIOA, EXTI_PinSource3);
		}


		void EXTI_Pin_3_Init()
		{

			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			EXTI_Pin_3_Disable();

		}

		void EXTI3_IRQHandler(void)
		{
			if(EXTI_GetITStatus(EXTI_Line3) != RESET)
			{
				if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3))
				{
					TIM_Cmd(TIM4, ENABLE);
				}
				else if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3))
				{
					TIM_Cmd(TIM4, DISABLE);
					TIM4_CNT = TIM4->CNT;
					Distance_2 = (float)TIM4_CNT /58;
					EXTI_Pin_3_Disable();
					EXTI_ClearITPendingBit(EXTI_Line3);
				}
				EXTI_ClearITPendingBit(EXTI_Line3);
			}
		}

int main(void)
{
	SystemInit();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIOA_Init();

	TIM2_Config();
	TIM_2_Interruption_Config();
	TIM3_Config();
	EXTI_Pin_1_Init();
	TIM4_Config();
	EXTI_Pin_3_Init();
	TIM5_Config(42);
	TIM_5_Interruption_Config();








	while (1)
	{



	}


}
