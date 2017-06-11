#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "misc.h"
#include "main.h"

/*
 	Digital Theremin Project for STM32f407VG (Discovery)
 	Michal Chrul
 	michalchrul@gmail.com

 	This program contains code written by Andreas Finkelmeyer which was shared here:
 	http://www.mind-dump.net/configuring-the-stm32f4-discovery-for-audio

	Additional hardware used:
		2x HC-SR04 Ultrasonic sensors

	Additional connections used:
		Sensor #1: Trigger <---> PE5 | Echo <---> PD0  (Distance_1 for the frequency)
		Sensor #2: Trigger <---> PE7 | Echo <---> PB12 (Distance_2 for the amplitude)
*/

//Global variables
uint16_t TIM3_CNT, TIM4_CNT;
float Time_1, Time_2;
float Distance_1, Distance_2;
int i = 0;
int PeriodValue, RoundedUpPeriodValue;
uint16_t Sample;
uint16_t hData, lData;
uint16_t test = (uint16_t)(0.05 * 0x798a);

uint16_t sine_table[256] =
{
   0x0000, 0x0324, 0x0647, 0x096a, 0x0c8b, 0x0fab, 0x12c8, 0x15e2,
   0x18f8, 0x1c0b, 0x1f19, 0x2223, 0x2528, 0x2826, 0x2b1f, 0x2e11,
   0x30fb, 0x33de, 0x36ba, 0x398c, 0x3c56, 0x3f17, 0x41ce, 0x447a,
   0x471c, 0x49b4, 0x4c3f, 0x4ebf, 0x5133, 0x539b, 0x55f5, 0x5842,
   0x5a82, 0x5cb4, 0x5ed7, 0x60ec, 0x62f2, 0x64e8, 0x66cf, 0x68a6,
   0x6a6d, 0x6c24, 0x6dca, 0x6f5f, 0x70e2, 0x7255, 0x73b5, 0x7504,
   0x7641, 0x776c, 0x7884, 0x798a, 0x7a7d, 0x7b5d, 0x7c29, 0x7ce3,
   0x7d8a, 0x7e1d, 0x7e9d, 0x7f09, 0x7f62, 0x7fa7, 0x7fd8, 0x7ff6,
   0x7fff, 0x7ff6, 0x7fd8, 0x7fa7, 0x7f62, 0x7f09, 0x7e9d, 0x7e1d,
   0x7d8a, 0x7ce3, 0x7c29, 0x7b5d, 0x7a7d, 0x798a, 0x7884, 0x776c,
   0x7641, 0x7504, 0x73b5, 0x7255, 0x70e2, 0x6f5f, 0x6dca, 0x6c24,
   0x6a6d, 0x68a6, 0x66cf, 0x64e8, 0x62f2, 0x60ec, 0x5ed7, 0x5cb4,
   0x5a82, 0x5842, 0x55f5, 0x539b, 0x5133, 0x4ebf, 0x4c3f, 0x49b4,
   0x471c, 0x447a, 0x41ce, 0x3f17, 0x3c56, 0x398c, 0x36ba, 0x33de,
   0x30fb, 0x2e11, 0x2b1f, 0x2826, 0x2528, 0x2223, 0x1f19, 0x1c0b,
   0x18f8, 0x15e2, 0x12c8, 0x0fab, 0x0c8b, 0x096a, 0x0647, 0x0324,
   0x0000, 0xfcdc, 0xf9b9, 0xf696, 0xf375, 0xf055, 0xed38, 0xea1e, // 0 is the 128th value (sine_table[127])
   0xe708, 0xe3f5, 0xe0e7, 0xdddd, 0xdad8, 0xd7da, 0xd4e1, 0xd1ef,
   0xcf05, 0xcc22, 0xc946, 0xc674, 0xc3aa, 0xc0e9, 0xbe32, 0xbb86,
   0xb8e4, 0xb64c, 0xb3c1, 0xb141, 0xaecd, 0xac65, 0xaa0b, 0xa7be,
   0xa57e, 0xa34c, 0xa129, 0x9f14, 0x9d0e, 0x9b18, 0x9931, 0x975a,
   0x9593, 0x93dc, 0x9236, 0x90a1, 0x8f1e, 0x8dab, 0x8c4b, 0x8afc,
   0x89bf, 0x8894, 0x877c, 0x8676, 0x8583, 0x84a3, 0x83d7, 0x831d,
   0x8276, 0x81e3, 0x8163, 0x80f7, 0x809e, 0x8059, 0x8028, 0x800a,
   0x8000, 0x800a, 0x8028, 0x8059, 0x809e, 0x80f7, 0x8163, 0x81e3,
   0x8276, 0x831d, 0x83d7, 0x84a3, 0x8583, 0x8676, 0x877c, 0x8894,
   0x89bf, 0x8afc, 0x8c4b, 0x8dab, 0x8f1e, 0x90a1, 0x9236, 0x93dc,
   0x9593, 0x975a, 0x9931, 0x9b18, 0x9d0e, 0x9f14, 0xa129, 0xa34c,
   0xa57e, 0xa7be, 0xaa0b, 0xac65, 0xaecd, 0xb141, 0xb3c1, 0xb64c,
   0xb8e4, 0xbb86, 0xbe32, 0xc0e9, 0xc3aa, 0xc674, 0xc946, 0xcc22,
   0xcf05, 0xd1ef, 0xd4e1, 0xd7da, 0xdad8, 0xdddd, 0xe0e7, 0xe3f5,
   0xe708, 0xea1e, 0xed38, 0xf055, 0xf375, 0xf696, 0xf9b9, 0xfcdc,
};

uint16_t sine_table_2[128]=
{
	2048, 2145, 2242, 2339, 2435, 2530, 2624, 2717, 2808, 2897,
	2984, 3069, 3151, 3230, 3307, 3381, 3451, 3518, 3581, 3640,
	3696, 3748, 3795, 3838, 3877, 3911, 3941, 3966, 3986, 4002,
	4013, 4019, 4020, 4016, 4008, 3995, 3977, 3954, 3926, 3894,
	3858, 3817, 3772, 3722, 3669, 3611, 3550, 3485, 3416, 3344,
	3269, 3191, 3110, 3027, 2941, 2853, 2763, 2671, 2578, 2483,
	2387, 2291, 2194, 2096, 1999, 1901, 1804, 1708, 1612, 1517,
	1424, 1332, 1242, 1154, 1068, 985, 904, 826, 751, 679,
	610, 545, 484, 426, 373, 323, 278, 237, 201, 169,
	141, 118, 100, 87, 79, 75, 76, 82, 93, 109,
	129, 154, 184, 218, 257, 300, 347, 399, 455, 514,
	577, 644, 714, 788, 865, 944, 1026, 1111, 1198, 1287,
	1378, 1471, 1565, 1660, 1756, 1853, 1950, 2047
};

void TIM2_Config()
{
	//Timer responsible for sending the Trigger signals and setting the PeriodValue of TIM5

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 4199; // 8399 for 10 Hz / 4199 for 20 Hz / 1679 for 50 Hz / 839 for 100 Hz refreshing rate
	TIM_TimeBaseStructure.TIM_Prescaler = 999;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
}

void TIM_2_Interrupt_Config()
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
		EXTI_Line_0_Enable();
		EXTI_Line_12_Enable();

		GPIO_ResetBits(GPIOE, GPIO_Pin_7);
		GPIO_ResetBits(GPIOE, GPIO_Pin_5);

		TIM_SetCounter(TIM7, 0);
		TIM_SetCounter(TIM3, 0);
		TIM_SetCounter(TIM4, 0);

		GPIO_SetBits(GPIOE, GPIO_Pin_7);
		GPIO_SetBits(GPIOE, GPIO_Pin_5);

		TIM_Cmd(TIM7, ENABLE);

		/*
		//Pseudo-stepless frequency adjustment

		PeriodValue = -120*(int)Distance_1 + 8120;
		if (PeriodValue > 8000)
		{
			PeriodValue = 8000;
		}
		if (PeriodValue <3680)
		{
			PeriodValue = 3680;
		}
		TIM5_Config(PeriodValue);
		TIM_5_Interrupt_Config();
		*/

		//F-Dur based frequency adjustment
		PeriodValue = -120*(int)Distance_1 + 8120;
		//F
		if (PeriodValue >= 7400)
		{
			PeriodValue = 7400;
		}
		//G
		if (PeriodValue >= 6680 && PeriodValue < 7400)
		{
			PeriodValue = 6680;
		}
		//A
		if (PeriodValue >= 5960 && PeriodValue < 6680)
		{
			PeriodValue = 5960;
		}
		//A#
		if (PeriodValue >= 5600 && PeriodValue < 5960)
		{
			PeriodValue = 5600;
		}
		//C
		if (PeriodValue >= 5000 && PeriodValue < 5600)
		{
			PeriodValue = 5000;
		}
		//D
		if (PeriodValue >= 4400 && PeriodValue < 5000)
		{
			PeriodValue = 4400;
		}
		//E
		if (PeriodValue >= 3920 && PeriodValue < 4400)
		{
			PeriodValue = 3920;
		}
		//F
		if (PeriodValue < 3920)
		{
			PeriodValue = 3680;
		}

		TIM5_Config(PeriodValue);
		TIM_5_Interrupt_Config();

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void TIM3_Config()
{
	//Timer responsible for measuring the length of the Echo signal return (Sensor #1)
	//f reload = 10 ms, f inc = 1 000 000 (T=1us)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM3, DISABLE);
}

void TIM4_Config()
{
	//Timer responsible for measuring the length of the Echo signal return (Sensor #2)
	//f reload = 10 ms, f inc = 1 000 000 (T=1us)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_Prescaler = 83;
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
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM5, ENABLE);

}

void TIM_5_Interrupt_Config()
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
		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
		{
					//for 256 element table
					lData = sine_table[i];
					hData = sine_table[i];
					hData >>= 8;
					lData = (uint16_t)(0.005 * lData);
					hData = (uint16_t)(0.005 * hData);
		    		SPI_I2S_SendData(CODEC_I2S, hData);
		    		SPI_I2S_SendData(CODEC_I2S, lData);

		    		i++;
		    		if(i == 255)
		    		{
		    			i=0;
		    		}

		    		/*
		    		//for 128 element table
		    		Sample = sine_table_2[i];
					SPI_I2S_SendData(CODEC_I2S, Sample);
					i++;
					if(i == 127)
					{
						i=0;
					}
					*/
		}

		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

void TIM7_Config()
{
	//Timer responsible for the trigger signal length
	//f reload = 50 000Hz => T = 20us (trigger signal has to be >= 10us)

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 839;
	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode =  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM7, ENABLE);
}

void TIM_7_Interrupt_Config()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}

void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		GPIO_ResetBits(GPIOE, GPIO_Pin_7);
		GPIO_ResetBits(GPIOE, GPIO_Pin_5);

		TIM_Cmd(TIM7, DISABLE);

		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}

void GPIO_ED_Init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;

	//Trigger signal pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//LEDs for EXTI test
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}


void Configure_PD0(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);

	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void EXTI_Line_0_Enable()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
}

void EXTI_Line_0_Disable()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);
}


void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
    	if (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0))
    	{
    		TIM_Cmd(TIM3, ENABLE);
    	}
    	else if (!GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0))
		{
    		TIM_Cmd(TIM3, DISABLE);
			TIM3_CNT = TIM3->CNT;
			Distance_1 = (float)TIM3_CNT / 58;
			EXTI_Line_0_Disable();
			EXTI_ClearITPendingBit(EXTI_Line1);
		}

        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

void Configure_PB12(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);

    EXTI_InitStruct.EXTI_Line = EXTI_Line12;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

void EXTI_Line_12_Enable()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
}

void EXTI_Line_12_Disable()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
}


void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
		{
			TIM_Cmd(TIM4, ENABLE);
		}
		else if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
		{
			TIM_Cmd(TIM4, DISABLE);
			TIM4_CNT = TIM4->CNT;
			Distance_2 = (float)TIM4_CNT /58;
			EXTI_Line_12_Disable();
			EXTI_ClearITPendingBit(EXTI_Line12);
		}
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}



int main(void)
{
	SystemInit();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	Configure_PD0();
	Configure_PB12();

	EXTI_Line_0_Disable();
	EXTI_Line_12_Disable();

	GPIO_ED_Init();
	TIM2_Config();
	TIM_2_Interrupt_Config();
	TIM3_Config();
	TIM4_Config();
	TIM7_Config();
	TIM_7_Interrupt_Config();
	TIM_5_Interrupt_Config();


	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	codec_init();
	codec_ctrl_init();
	I2S_Cmd(CODEC_I2S, ENABLE);

	Codec_VolumeCtrl(0x0000);
	//Codec_Mute(AUDIO_MUTE_ON);

	while (1)
	{

	}
}
