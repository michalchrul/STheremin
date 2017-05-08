#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_tim.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_delay.c"
#include "tm_stm32f4_hcsr04.h"
#include "tm_stm32f4_hcsr04.c"
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_gpio.c"
#include <stm32f4xx.h>
#include "audio.c"
#include "codec.h"
#include "main.h"



float Distance_1, Distance_2;

//#define   OUT_FREQ          5000                                 // czestotliwosc fali
//#define   SINE_RES          128                                  // rozdzielczosc
//#define   DAC_DHR12R1_ADDR  0x40007408                           // rejestr do ktorego dma wpisuje dane
//#define   CNT_FREQ          42000000                             // tim6 counter clock
//#define   TIM_PERIOD        ((CNT_FREQ)/((SINE_RES)*(OUT_FREQ))) // okres z uwzglednieniem cntfreq
//
//const uint16_t function[SINE_RES] = { 2048, 2145, 2242, 2339, 2435, 2530, 2624, 2717, 2808, 2897,
//                                      2984, 3069, 3151, 3230, 3307, 3381, 3451, 3518, 3581, 3640,
//                                      3696, 3748, 3795, 3838, 3877, 3911, 3941, 3966, 3986, 4002,
//                                      4013, 4019, 4020, 4016, 4008, 3995, 3977, 3954, 3926, 3894,
//                                      3858, 3817, 3772, 3722, 3669, 3611, 3550, 3485, 3416, 3344,
//                                      3269, 3191, 3110, 3027, 2941, 2853, 2763, 2671, 2578, 2483,
//                                      2387, 2291, 2194, 2096, 1999, 1901, 1804, 1708, 1612, 1517,
//                                      1424, 1332, 1242, 1154, 1068, 985, 904, 826, 751, 679,
//                                      610, 545, 484, 426, 373, 323, 278, 237, 201, 169,
//                                      141, 118, 100, 87, 79, 75, 76, 82, 93, 109,
//                                      129, 154, 184, 218, 257, 300, 347, 399, 455, 514,
//                                      577, 644, 714, 788, 865, 944, 1026, 1111, 1198, 1287,
//                                      1378, 1471, 1565, 1660, 1756, 1853, 1950, 2047 };
//
//static void TIM6_Config(void);
//static void DAC1_Config(void);
//
//static void TIM6_Config(void)
//{
//  TIM_TimeBaseInitTypeDef TIM6_TimeBase;
//
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
//
//  TIM_TimeBaseStructInit(&TIM6_TimeBase);
//  TIM6_TimeBase.TIM_Period        = (uint16_t)TIM_PERIOD;
//  TIM6_TimeBase.TIM_Prescaler     = 0;
//  TIM6_TimeBase.TIM_ClockDivision = 0;
//  TIM6_TimeBase.TIM_CounterMode   = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM6, &TIM6_TimeBase);
//  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
//
//  TIM_Cmd(TIM6, ENABLE);
//}
//
//static void DAC1_Config(void)
//{
//  DAC_InitTypeDef DAC_INIT;
//  DMA_InitTypeDef DMA_INIT;
//
//  DAC_INIT.DAC_Trigger        = DAC_Trigger_T6_TRGO;
//  DAC_INIT.DAC_WaveGeneration = DAC_WaveGeneration_None;
//  DAC_INIT.DAC_OutputBuffer   = DAC_OutputBuffer_Enable;
//  DAC_Init(DAC_Channel_1, &DAC_INIT);
//
//  DMA_DeInit(DMA1_Stream5);
//  DMA_INIT.DMA_Channel            = DMA_Channel_7;
//  DMA_INIT.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R1_ADDR;
//  DMA_INIT.DMA_Memory0BaseAddr    = (uint32_t)&function;
//  DMA_INIT.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
//  DMA_INIT.DMA_BufferSize         = SINE_RES;
//  DMA_INIT.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
//  DMA_INIT.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//  DMA_INIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//  DMA_INIT.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
//  DMA_INIT.DMA_Mode               = DMA_Mode_Circular;
//  DMA_INIT.DMA_Priority           = DMA_Priority_High;
//  DMA_INIT.DMA_FIFOMode           = DMA_FIFOMode_Disable;
//  DMA_INIT.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
//  DMA_INIT.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
//  DMA_INIT.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
//  DMA_Init(DMA1_Stream5, &DMA_INIT);
//
//  DMA_Cmd(DMA1_Stream5, ENABLE);
//  DAC_Cmd(DAC_Channel_1, ENABLE);
//  DAC_DMACmd(DAC_Channel_1, ENABLE);
//}

// a very crude FIR lowpass filter
float updateFilter(fir_8* filt, float val)
{
	uint16_t valIndex;
	uint16_t paramIndex;
	float outval = 0.0;

	valIndex = filt->currIndex;
	filt->tabs[valIndex] = val;

	for (paramIndex=0; paramIndex<8; paramIndex++)
	{
		outval += (filt->params[paramIndex]) * (filt->tabs[(valIndex+paramIndex)&0x07]);
	}

	valIndex++;
	valIndex &= 0x07;

	filt->currIndex = valIndex;

	return outval;
}

void initFilter(fir_8* theFilter)
{
	uint8_t i;

	theFilter->currIndex = 0;

	for (i=0; i<8; i++)
		theFilter->tabs[i] = 0.0;

	theFilter->params[0] = 0.01;
	theFilter->params[1] = 0.05;
	theFilter->params[2] = 0.12;
	theFilter->params[3] = 0.32;
	theFilter->params[4] = 0.32;
	theFilter->params[5] = 0.12;
	theFilter->params[6] = 0.05;
	theFilter->params[7] = 0.01;
}

int main(){

	SystemInit();

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,  ENABLE);


//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//
//	  GPIO_InitTypeDef gpio_A;
//	  gpio_A.GPIO_Pin  = GPIO_Pin_4;
//	  gpio_A.GPIO_Mode = GPIO_Mode_AN;
//	  gpio_A.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	  GPIO_Init(GPIOA, &gpio_A);
//
//	  TIM6_Config();
//	  DAC1_Config();





						//	// zegary dla dac
						//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1 | RCC_APB1Periph_SPI3, ENABLE);
						//
						//	RCC_PLLI2SCmd(ENABLE);
						//
						//	//piny dac
						//	GPIO_InitTypeDef PinInitStruct;
						//	PinInitStruct.GPIO_Pin = GPIO_Pin_4;
						//	PinInitStruct.GPIO_Mode = GPIO_Mode_OUT;
						//	PinInitStruct.GPIO_OType = GPIO_OType_PP;
						//	PinInitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
						//	PinInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
						//
						//	GPIO_Init(GPIOD, &PinInitStruct);
						//
						//	PinInitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_9;
						//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_SPI3); //laczenie pinu 4 portu A do SPI3
						//
						//	//konfiguracja I2S
						//	I2S_InitTypeDef I2S_InitType;
						//	I2S_InitType.I2S_AudioFreq = I2S_AudioFreq_48k;
						//	I2S_InitType.I2S_MCLKOutput = I2S_MCLKOutput_Enable;
						//	I2S_InitType.I2S_Mode = I2S_Mode_MasterTx;
						//	I2S_InitType.I2S_DataFormat = I2S_DataFormat_16b;
						//	I2S_InitType.I2S_Standard = I2S_Standard_Phillips;
						//	I2S_InitType.I2S_CPOL = I2S_CPOL_Low;
						//
						//	I2S_Init(SPI3, &I2S_InitType);
						//
						//	I2S_Cmd(SPI3, ENABLE);
						//
						//	//inicjalizacja interfejsu i2c
						//	I2C_InitTypeDef I2C_InitType;
						//	I2C_InitType.I2C_ClockSpeed = 100000;
						//	I2C_InitType.I2C_Mode = I2C_Mode_I2C;
						//	I2C_InitType.I2C_OwnAddress1 = 99;
						//	I2C_InitType.I2C_Ack = I2C_Ack_Enable;
						//	I2C_InitType.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
						//	I2C_InitType.I2C_DutyCycle = I2C_DutyCycle_2;
						//
						//	I2C_Init(I2C1, &I2C_InitType);
						//	I2C_Cmd(I2C1, ENABLE);
						//
						//	SPI_I2S_SendData(SPI3, rawAudio);
						//	SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE);


	SystemInit();

		fir_8 filt;

		//enables GPIO clock for PortD
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_Init(GPIOD, &GPIO_InitStructure);

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		codec_init();
		codec_ctrl_init();

		I2S_Cmd(CODEC_I2S, ENABLE);

		initFilter(&filt);



	//piny LED
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

	//piny czujnika 1
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

	//piny czujnika 2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

	TM_HCSR04_t Sensor_1;
	TM_HCSR04_t Sensor_2;

	//(echo, trigger)
	TM_HCSR04_Init(&Sensor_1, GPIOA, GPIO_Pin_1, GPIOA, GPIO_Pin_0);
	TM_HCSR04_Init(&Sensor_2, GPIOA, GPIO_Pin_3, GPIOA, GPIO_Pin_2);



	// 12 green, 13 orange, 14 red, 15 blue

	//Sensor_1
	/* Initialize distance sensor1 on pins; ECHO: PD0, TRIGGER: PC1 */
	if (!TM_HCSR04_Init(&Sensor_1, GPIOA, GPIO_PIN_1, GPIOA, GPIO_PIN_0)) {
		/* Sensor is not ready to use */
		/* Maybe wiring is incorrect */
		while (1) {

			Delayms(100);
		}
	}

	//Sensor_2
	/* Initialize distance sensor1 on pins; ECHO: PD0, TRIGGER: PC1 */
	if (!TM_HCSR04_Init(&Sensor_2, GPIOA, GPIO_Pin_3, GPIOA, GPIO_Pin_2)) {
		/* Sensor is not ready to use */
		/* Maybe wiring is incorrect */
		while (1) {

			Delayms(100);
		}
	}

	GPIO_SetBits(GPIOD,GPIO_Pin_15);
	GPIO_SetBits(GPIOD,GPIO_Pin_13);

	while (1) {



		if (SPI_I2S_GetFlagStatus(CODEC_I2S, SPI_I2S_FLAG_TXE))
			    	{
			    		SPI_I2S_SendData(CODEC_I2S, sample);

			    		//only update on every second sample to insure that L & R ch. have the same sample value
			    		if (sampleCounter & 0x00000001)
			    		{
			    			sawWave += NOTEFREQUENCY;
			    			if (sawWave > 1.0)
			    				sawWave -= 2.0;

			    			filteredSaw = updateFilter(&filt, sawWave);

			    			sample = (int16_t)(NOTEAMPLITUDE*filteredSaw);
			    		}
			    		sampleCounter++;
			    	}

			    	if (sampleCounter==48000)
			    	{
			    		//LED_BLUE_OFF;

			    	}
			    	else if (sampleCounter == 96000)
			    	{
			    		//LED_BLUE_ON;
			    		sampleCounter = 0;
			    	}

		Distance_1 = TM_HCSR04_Read(&Sensor_1);
		Distance_2 = TM_HCSR04_Read(&Sensor_2);










		/* Read distance from sensor 1 */
		/* Distance is returned in cm and also stored in structure */
		/* You can use both ways */
		TM_HCSR04_Read(&Sensor_1);

		/* Something is going wrong, maybe incorrect pinout */
		if (Sensor_1.Distance < 0) {
			GPIO_SetBits(GPIOD,GPIO_Pin_15);

		} else if (Sensor_1.Distance > 50) {
			/* Distance more than 50cm */

			GPIO_SetBits(GPIOD,GPIO_Pin_15);
		} else {
			/* Distance between 0 and 50cm */
			GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		}

		/* Give some time to sensor */
		Delayms(100);

		/* Read distance from sensor 1 */
		/* Distance is returned in cm and also stored in structure */
		/* You can use both ways */
		TM_HCSR04_Read(&Sensor_2);

		/* Something is going wrong, maybe incorrect pinout */
		if (Sensor_2.Distance < 0) {
			GPIO_SetBits(GPIOD,GPIO_Pin_13);

		} else if (Sensor_2.Distance > 50) {
			/* Distance more than 50cm */

			GPIO_SetBits(GPIOD,GPIO_Pin_13);
		} else {
			/* Distance between 0 and 50cm */
			GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		}

		/* Give some time to sensor */
		Delayms(100);










	}
}
