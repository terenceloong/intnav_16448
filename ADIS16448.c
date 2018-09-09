#include "stm32f4xx.h"
#include "ADIS16448.h"
#include "uclock.h"

#define CS_CLOCK      RCC_AHB1Periph_GPIOA
#define CS_GPIO       GPIOA
#define CS_PIN        GPIO_Pin_4

#define RST_CLOCK     RCC_AHB1Periph_GPIOC
#define RST_GPIO      GPIOC
#define RST_PIN       GPIO_Pin_4

#define CLK_CLOCK     RCC_AHB1Periph_GPIOA
#define CLK_GPIO      GPIOA
#define CLK_PIN       GPIO_Pin_1

#define IRQ_CLOCK     RCC_AHB1Periph_GPIOA
#define IRQ_GPIO      GPIOA
#define IRQ_PIN       GPIO_Pin_0
#define EXIT_PORT     EXTI_PortSourceGPIOA
#define EXIT_PIN      EXTI_PinSource0
#define EXIT_LINE     EXTI_Line0
#define IRQ_CHANNEL   EXTI0_IRQn

static uint8_t ADIS16448_state = 0;
static uint16_t ADIS16448_cmd[13] = {0x003E, 0, 0,0,0, 0,0,0, 0,0,0, 0,0};
ADIS16448_DATA ADIS16448_data;
uint8_t ADIS16448_flag = 0;
uint32_t ADIS16448_stamp = 0;

static void delay_us(uint32_t n)
{
	n *= 42;
	while((n--)>0);
}

void ADIS16448_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    SPI_InitTypeDef SPI_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// CLOCK
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //SPI
    RCC_AHB1PeriphClockCmd(CS_CLOCK, ENABLE); //CS
    RCC_AHB1PeriphClockCmd(RST_CLOCK, ENABLE); //RST
	RCC_AHB1PeriphClockCmd(CLK_CLOCK, ENABLE); //CLK
	RCC_AHB1PeriphClockCmd(IRQ_CLOCK, ENABLE); //IRQ
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); //EXIT
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	// GPIO
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; //SCK, MISO, MOSI
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = CS_PIN; //CS
    GPIO_Init(CS_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(CS_GPIO, CS_PIN);
	GPIO_InitStructure.GPIO_Pin = RST_PIN; //RST
    GPIO_Init(RST_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(RST_GPIO, RST_PIN);
	GPIO_InitStructure.GPIO_Pin = CLK_PIN; //CLK
    GPIO_Init(CLK_GPIO, &GPIO_InitStructure);
	GPIO_SetBits(CLK_GPIO, CLK_PIN);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = IRQ_PIN; //IRQ
	GPIO_Init(IRQ_GPIO, &GPIO_InitStructure);

	// EXIT
	SYSCFG_EXTILineConfig(EXIT_PORT, EXIT_PIN);
	EXTI_InitStructure.EXTI_Line = EXIT_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// TIM, ADIS16448 clk timer
	TIM_TimeBaseStructure.TIM_Period = 624; //625us, 1600Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 83; //1us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// SPI
    SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler	= SPI_BaudRatePrescaler_128; //656.25kHz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/* Configure TX DMA */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ADIS16448_cmd;
	DMA_InitStructure.DMA_BufferSize = 26;
	DMA_Init(DMA2_Stream3, &DMA_InitStructure);
	/* Configure RX DMA */
	DMA_InitStructure.DMA_Channel = DMA_Channel_3;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADIS16448_data;
	DMA_InitStructure.DMA_BufferSize = 26;
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);

	// NVIC
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = IRQ_CHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

	// Interrupt enable
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);

	// ENABLE
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_Cmd(SPI1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

/*-----------------------------------------------------------------------------*/
/* ADIS16448 clk */
void TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_FLAG_Update)
	{
		TIM3->SR = ~TIM_FLAG_Update;
		CLK_GPIO->ODR ^= CLK_PIN; //800Hz
	}
}

/* ADIS16448 read data */
void EXTI0_IRQHandler(void)
{
	EXTI->PR = EXTI_Line0; //clear interrupt flag
	if(ADIS16448_state == 1)
	{
		ADIS16448_stamp = get_uclock();
		CS_GPIO->BSRRH = CS_PIN;
		DMA2_Stream2->CR |= DMA_SxCR_EN;
		DMA2_Stream3->CR |= DMA_SxCR_EN;
	}
}

/* SPI1 Rx DMA complete interrupt */
void DMA2_Stream2_IRQHandler(void)
{
	DMA2->LIFCR = (uint32_t)(DMA_FLAG_TCIF2 & 0x0F7D0F7D);
	DMA2->LIFCR = (uint32_t)(DMA_FLAG_HTIF2 & 0x0F7D0F7D);
	DMA2->LIFCR = (uint32_t)(DMA_FLAG_TCIF3 & 0x0F7D0F7D);
	DMA2->LIFCR = (uint32_t)(DMA_FLAG_HTIF3 & 0x0F7D0F7D);
	CS_GPIO->BSRRL = CS_PIN;
	ADIS16448_flag = 1;
}
/*-----------------------------------------------------------------------------*/

void ADIS16448_Startup(void)
{
    delay_us(250000); //at least 205ms
    ADIS16448_Reset();
}

void ADIS16448_Reset(void)
{
    ADIS16448_state = 0;
	delay_us(500); //at least 320us
	RST_GPIO->BSRRH = RST_PIN;
    delay_us(20); //at least 10us
    RST_GPIO->BSRRL = RST_PIN;
	delay_us(100000); //at least 90ms, it's necessary
	ADIS16448_state = 1;
}

uint16_t ADIS16448_ReadReg(uint8_t reg)
{
	uint16_t dataH, dataL;

	ADIS16448_state = 0;
	delay_us(500); //at least 320us
	CS_GPIO->BSRRH = CS_PIN;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	SPI1->DR = reg;
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	SPI1->DR = 0;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	CS_GPIO->BSRRL = CS_PIN;
	delay_us(9);

	CS_GPIO->BSRRH = CS_PIN;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	SPI1->DR = reg;
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	SPI1->DR = 0;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	dataH = SPI1->DR;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	dataL = SPI1->DR;
	CS_GPIO->BSRRL = CS_PIN;
	delay_us(9);
	ADIS16448_state = 1;

	return (dataH << 8) | (dataL & 0x00FF);
}

void ADIS16448_WriteReg(uint8_t reg, uint16_t data)
{
	ADIS16448_state = 0;
	delay_us(500); //at least 320us
	CS_GPIO->BSRRH = CS_PIN;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	SPI1->DR = reg | 0x80;
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	SPI1->DR = data & 0x00FF;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	CS_GPIO->BSRRL = CS_PIN;
	delay_us(9);
	
	CS_GPIO->BSRRH = CS_PIN;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	SPI1->DR = (reg+1) | 0x80;
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	SPI1->DR = (data & 0xFF00) >> 8;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	CS_GPIO->BSRRL = CS_PIN;
	delay_us(9);
	ADIS16448_state = 1;
}

void ADIS16448_Save(void)
{
	ADIS16448_state = 0;
	delay_us(500); //at least 320us
	CS_GPIO->BSRRH = CS_PIN;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	SPI1->DR = 0xBE;
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	SPI1->DR = 0x08;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	CS_GPIO->BSRRL = CS_PIN;
	delay_us(80000); //at least 75ms
	ADIS16448_state = 1;
}

void ADIS16448_Restore(void)
{
	ADIS16448_state = 0;
	delay_us(500); //at least 320us
	CS_GPIO->BSRRH = CS_PIN;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	SPI1->DR = 0xBE;
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE));
	SPI1->DR = 0x02;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	SPI1->DR;
	CS_GPIO->BSRRL = CS_PIN;
	delay_us(80000); //at least 75ms
	ADIS16448_state = 1;
}
