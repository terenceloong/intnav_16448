#include "stm32f4xx.h"
#include "NEO6M.h"

uint8_t NEO6M_Rx[800];
uint8_t NEO6M_flag = 0;

nmeaINFO NEO6M_info;
nmeaPARSER parser;

void NEO6M_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	nmea_zero_INFO(&NEO6M_info);
	nmea_parser_init(&parser);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//Tx		
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//Rx					  
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_OverSampling8Cmd(USART3, ENABLE);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3, &USART_InitStructure);

    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART3->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    /* Configure RX DMA */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)NEO6M_Rx;
	DMA_InitStructure.DMA_BufferSize = sizeof(NEO6M_Rx);
	DMA_Init(DMA1_Stream1, &DMA_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(DMA1_Stream1, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

/*-----------------------------------------------------------------------------*/
static uint16_t length = 0;

void USART3_IRQHandler(void)
{
    if(USART3->SR & USART_FLAG_IDLE)
    {
        USART3->DR; //Clear IDLE flag
        DMA1_Stream1->CR &= ~DMA_SxCR_EN; //DMA_Cmd(DMA1_Stream1, DISABLE)
        length = sizeof(NEO6M_Rx) - DMA1_Stream1->NDTR; //DMA_GetCurrDataCounter(DMA1_Stream1)
        DMA1->LIFCR = (uint32_t)(DMA_FLAG_TCIF1 & 0x0F7D0F7D); //DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1)
        DMA1->LIFCR = (uint32_t)(DMA_FLAG_HTIF1 & 0x0F7D0F7D); //DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_HTIF1)
        DMA1_Stream1->CR |= DMA_SxCR_EN; //DMA_Cmd(DMA1_Stream1, ENABLE)
        if(NEO6M_Rx[0]=='$' && NEO6M_Rx[3] == 'R')
        {
            NEO6M_flag = 1;
        }
    }
}
/*-----------------------------------------------------------------------------*/

void NEO6M_read(void)
{
	nmea_parse(&parser, (char*)NEO6M_Rx, (int)length, &NEO6M_info);
}
