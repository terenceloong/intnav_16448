#include "stm32f4xx.h"
#include <math.h>

//static uint8_t uclock_Rx[250];
//static uint32_t uclock = 0; //ms of week

uint8_t uclock_Rx[250];
uint32_t uclock = 0; //ms of week

void UClock_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// CLOCK
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //EXIT
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	// EXIT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //*
	GPIO_Init(GPIOA, &GPIO_InitStructure); //* PA3, USART2_RX, Pin11

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3); //*
	EXTI_InitStructure.EXTI_Line = EXTI_Line3; //*
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// TIM
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 41999; //500us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	// USART
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//Tx		
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//Rx					  
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	USART_OverSampling8Cmd(UART5, ENABLE);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART5, &USART_InitStructure);

	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(UART5->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    /* Configure RX DMA */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)uclock_Rx;
	DMA_InitStructure.DMA_BufferSize = sizeof(uclock_Rx);
	DMA_Init(DMA1_Stream0, &DMA_InitStructure);
	
	// NVIC
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

	// Interrupt enable
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	
	// ENABLE
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream0, ENABLE);
    USART_Cmd(UART5, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}

/*-----------------------------------------------------------------------------*/
// PPS interrupt
void EXTI3_IRQHandler(void)
{
	EXTI->PR = EXTI_Line3; //clear interrupt flag
	uclock += round((double)TIM5->CNT/2000.0) * 1000;
	TIM5->CNT = 0;
}

void UART5_IRQHandler(void)
{
	uint8_t length;
	
	if(UART5->SR & USART_FLAG_IDLE)
    {
        UART5->DR; //Clear IDLE flag
        DMA1_Stream0->CR &= ~DMA_SxCR_EN; //DMA_Cmd(DMA1_Stream0, DISABLE)
        length = sizeof(uclock_Rx) - DMA1_Stream0->NDTR; //DMA_GetCurrDataCounter(DMA1_Stream0)
        DMA1->LIFCR = (uint32_t)(DMA_FLAG_TCIF0 & 0x0F7D0F7D); //DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0)
        DMA1->LIFCR = (uint32_t)(DMA_FLAG_HTIF0 & 0x0F7D0F7D); //DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_HTIF0)
        DMA1_Stream0->CR |= DMA_SxCR_EN; //DMA_Cmd(DMA1_Stream0, ENABLE)
		if(uclock_Rx[0]==2 && uclock_Rx[length-1]==3 && uclock_Rx[3]==(length-6)) //BD910 RT-17 Position Massage
		{
			uclock = (uint32_t)uclock_Rx[80]<<24 | (uint32_t)uclock_Rx[81]<<16 | (uint32_t)uclock_Rx[82]<<8 | (uint32_t)uclock_Rx[83]; //GPS ms of week
		}
    }
}
/*-----------------------------------------------------------------------------*/

uint32_t get_uclock(void)
{
	return uclock + TIM5->CNT / 2;
}
