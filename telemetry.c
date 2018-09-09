#include <string.h>
#include "stm32f4xx.h"
#include "main.h"
#include "flash.h"

// The computer doesn't send data continuously, so can't use DMA to receive data.

static uint8_t telemetry_Tx[100] = {0x55};
static uint8_t telemetry_Rx[100];
uint8_t CMD_flag = 0;

void Telemetry_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //Tx		
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //Rx				      
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_OverSampling8Cmd(USART1, ENABLE);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);

	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/* Configure TX DMA */
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)telemetry_Tx;
	DMA_InitStructure.DMA_BufferSize = sizeof(telemetry_Tx);
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

/*-----------------------------------------------------------------------------*/
/* USART1 Tx DMA complete interrupt */
void DMA2_Stream7_IRQHandler(void)
{
	DMA2->HIFCR = (uint32_t)(DMA_FLAG_TCIF7 & 0x0F7D0F7D); //DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7)
	DMA2->HIFCR = (uint32_t)(DMA_FLAG_HTIF7 & 0x0F7D0F7D); //DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_HTIF7)
}

/* USART1 Rx interrupt */
void USART1_IRQHandler(void)
{
    static uint8_t p = 0;
    uint8_t data;
    
    if(USART1->SR & USART_FLAG_RXNE)
    {
		data = (uint8_t)USART1->DR;
        if(p == 0)
        {
            if(data == 0x55)
                telemetry_Rx[p++] = data;    
        }
        else if(p == 1)
        {
            if(data <= sizeof(telemetry_Rx))
                telemetry_Rx[p++] = data;
            else
                p = 0;
        }
        else
        {
            telemetry_Rx[p++] = data;
            if(p == telemetry_Rx[1])
            {
                p = 0;
                CMD_flag = 1;
            }
        }
    }  
}
/*-----------------------------------------------------------------------------*/

#define R2D                 57.29577951308
#define D2R                 0.01745329251994

static uint8_t telemetry_mode = 1;
static uint8_t telemetry_period = 1;
static uint8_t telemetry_cnt = 0;

void telemetry_data(void)
{
    uint8_t i, sum = 0;
	float tmpf;
	double tmpd;
	
	if(telemetry_mode != 0)
	{
		if((++telemetry_cnt) == telemetry_period)
		{
			telemetry_cnt = 0;
			switch(telemetry_mode)
			{
				case 1: //original data
					imu_read(&ADIS16448_data, &imu);
					*(uint32_t*)(&telemetry_Tx[3]) = ADIS16448_stamp;
					memcpy(&telemetry_Tx[7], &imu, 44);
					if(NEO6M_flag == 0)
					{
						telemetry_Tx[1] = 52; //3 + 4 + 11*4 + 1
						telemetry_Tx[2] = 0; //code
					}
					else
					{
						telemetry_Tx[1] = 80; //3 + 4 + 11*4 + 2*8 + 3*4 + 1
						telemetry_Tx[2] = 1; //code
						*(uint32_t*)&telemetry_Tx[51] =  *(uint32_t*)&NEO6M_info.lat;
						*(uint32_t*)&telemetry_Tx[55] = *((uint32_t*)&NEO6M_info.lat + 1);
						*(uint32_t*)&telemetry_Tx[59] =  *(uint32_t*)&NEO6M_info.lon;
						*(uint32_t*)&telemetry_Tx[63] = *((uint32_t*)&NEO6M_info.lon + 1);
						tmpf = (float)NEO6M_info.elv;
						*(uint32_t*)&telemetry_Tx[67] = *(uint32_t*)&tmpf;
						tmpf = (float)(NEO6M_info.speed / 3.6);
						*(uint32_t*)&telemetry_Tx[71] = *(uint32_t*)&tmpf;
						tmpf = (float)NEO6M_info.direction;
						*(uint32_t*)&telemetry_Tx[75] = *(uint32_t*)&tmpf;
					}
					break;
				case 2: //gyro and acc
					telemetry_Tx[1]  = 34; //3 + 6*5 + 1
					telemetry_Tx[2]  = telemetry_mode; //code
					telemetry_Tx[3]  = 1;
					*(uint32_t*)&telemetry_Tx[4]  = *(uint32_t*)&imu.gyro[0];
					telemetry_Tx[8]  = 2;
					*(uint32_t*)&telemetry_Tx[9]  = *(uint32_t*)&imu.gyro[1];
					telemetry_Tx[13] = 3;
					*(uint32_t*)&telemetry_Tx[14] = *(uint32_t*)&imu.gyro[2];
					telemetry_Tx[18] = 4;
					*(uint32_t*)&telemetry_Tx[19] = *(uint32_t*)&imu.acc[0];
					telemetry_Tx[23] = 5;
					*(uint32_t*)&telemetry_Tx[24] = *(uint32_t*)&imu.acc[1];
					telemetry_Tx[28] = 6;
					*(uint32_t*)&telemetry_Tx[29] = *(uint32_t*)&imu.acc[2];
					break;
				case 3: //mag, baro and temp
					telemetry_Tx[1]  = 29; //3 + 5*5 + 1
					telemetry_Tx[2]  = telemetry_mode; //code
					telemetry_Tx[3]  = 1;
					*(uint32_t*)&telemetry_Tx[4]  = *(uint32_t*)&imu.mag[0];
					telemetry_Tx[8]  = 2;
					*(uint32_t*)&telemetry_Tx[9]  = *(uint32_t*)&imu.mag[1];
					telemetry_Tx[13] = 3;
					*(uint32_t*)&telemetry_Tx[14] = *(uint32_t*)&imu.mag[2];
					telemetry_Tx[18] = 4;
					*(uint32_t*)&telemetry_Tx[19] = *(uint32_t*)&imu.baro;
					telemetry_Tx[23] = 5;
					*(uint32_t*)&telemetry_Tx[24] = *(uint32_t*)&imu.temp;
					break;
				case 4: //run time
					telemetry_Tx[1] = 9; //3 + 1*5 +1
					telemetry_Tx[2] = telemetry_mode; //code
					telemetry_Tx[3] = 1;
					tmpf = (float)run_time;
					*(uint32_t*)&telemetry_Tx[4]  = *(uint32_t*)&tmpf;
					break;
				case 5: //attitude
					telemetry_Tx[1]  = 49; //3 + 9*5 + 1
					telemetry_Tx[2]  = telemetry_mode; //code
					telemetry_Tx[3]  = 1;
					*(uint32_t*)&telemetry_Tx[4]  = *(uint32_t*)&iner.att_am[0];
					telemetry_Tx[8]  = 2;
					*(uint32_t*)&telemetry_Tx[9]  = *(uint32_t*)&iner.att_am[1];
					telemetry_Tx[13] = 3;
					*(uint32_t*)&telemetry_Tx[14] = *(uint32_t*)&iner.att_am[2];
					telemetry_Tx[18] = 1;
					*(uint32_t*)&telemetry_Tx[19] = *(uint32_t*)&iner.att[0];
					telemetry_Tx[23] = 2;
					*(uint32_t*)&telemetry_Tx[24] = *(uint32_t*)&iner.att[1];
					telemetry_Tx[28] = 3;
					*(uint32_t*)&telemetry_Tx[29] = *(uint32_t*)&iner.att[2];
					telemetry_Tx[33] = 4;
					tmpf = (float)(att_filter.X[4] * R2D);
					*(uint32_t*)&telemetry_Tx[34] = *(uint32_t*)&tmpf;
					telemetry_Tx[38] = 5;
					tmpf = (float)(att_filter.X[5] * R2D);
					*(uint32_t*)&telemetry_Tx[39] = *(uint32_t*)&tmpf;
					telemetry_Tx[43] = 6;
					tmpf = (float)(att_filter.X[6] * R2D);
					*(uint32_t*)&telemetry_Tx[44] = *(uint32_t*)&tmpf;
					break;
				case 6: //GPS lla
					telemetry_Tx[1]  = 49; //3 + 5*9 + 1
					telemetry_Tx[2]  = telemetry_mode + 100; //code
					telemetry_Tx[3]  = 1;
					*(uint32_t*)&telemetry_Tx[4]  =  *(uint32_t*)&gps.pos_lla[0];
					*(uint32_t*)&telemetry_Tx[8]  = *((uint32_t*)&gps.pos_lla[0] +1 );
					telemetry_Tx[12] = 2;
					*(uint32_t*)&telemetry_Tx[13] =  *(uint32_t*)&gps.pos_lla[1];
					*(uint32_t*)&telemetry_Tx[17] = *((uint32_t*)&gps.pos_lla[1] + 1);
					telemetry_Tx[21] = 3;
					*(uint32_t*)&telemetry_Tx[22] =  *(uint32_t*)&gps.pos_lla[2];
					*(uint32_t*)&telemetry_Tx[26] = *((uint32_t*)&gps.pos_lla[2] + 1);
					telemetry_Tx[30] = 4;
					tmpd = (double)gps.vel_vd[0];
					*(uint32_t*)&telemetry_Tx[31] =  *(uint32_t*)&tmpd;
					*(uint32_t*)&telemetry_Tx[35] = *((uint32_t*)&tmpd + 1);
					telemetry_Tx[39] = 5;
					tmpd = (double)gps.vel_vd[1];
					*(uint32_t*)&telemetry_Tx[40] =  *(uint32_t*)&tmpd;
					*(uint32_t*)&telemetry_Tx[44] = *((uint32_t*)&tmpd + 1);
					break;
				case 7: //GPS xyz
					telemetry_Tx[1]  = 29; //3 + 5*5 + 1
					telemetry_Tx[2]  = telemetry_mode; //code
					telemetry_Tx[3]  = 1;
					*(uint32_t*)&telemetry_Tx[4]  = *(uint32_t*)&gps.pos_xyz[0];
					telemetry_Tx[8]  = 2;
					*(uint32_t*)&telemetry_Tx[9]  = *(uint32_t*)&gps.pos_xyz[1];
					telemetry_Tx[13] = 3;
					*(uint32_t*)&telemetry_Tx[14] = *(uint32_t*)&gps.pos_xyz[2];
					telemetry_Tx[18] = 4;
					*(uint32_t*)&telemetry_Tx[19] = *(uint32_t*)&gps.vel_vd[0];
					telemetry_Tx[23] = 5;
					*(uint32_t*)&telemetry_Tx[24] = *(uint32_t*)&gps.vel_vd[1];
					break;
				default:
					break;
			}
			for(i=0; i<(telemetry_Tx[1]-1); i++)
				sum ^= telemetry_Tx[i];
			telemetry_Tx[telemetry_Tx[1]-1] = sum;

			DMA2_Stream7->NDTR = telemetry_Tx[1];
			DMA2_Stream7->CR |= DMA_SxCR_EN; //USART1_TX, DMA_Cmd(DMA2_Stream7, ENABLE)
		}
	}
}

void telemetry_cmd(void)
{
	uint8_t i, sum = 0;

	for(i=0; i<telemetry_Rx[1]-1; i++)
        sum ^= telemetry_Rx[i];
	if(sum == telemetry_Rx[i])
	{
		switch(telemetry_Rx[2])
		{
			/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
			case 0:
				telemetry_mode = telemetry_Rx[2];
				break;
			case 1:
				telemetry_mode = telemetry_Rx[2];
				telemetry_period = 1;
				telemetry_cnt = 0;
				break;
			case 2:
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
				telemetry_mode = telemetry_Rx[2];
				telemetry_period = 10;
				telemetry_cnt = 0;
				break;
			/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
			case 31: //Gyro and acc bias calibration
				memcpy(&flash_buff[0], &telemetry_Rx[3], 24); //flash_buff[0]~flash_buff[5], 6*4
				memcpy(&bias.gyro, &flash_buff[0], 24);
				break;
			case 32: //Mag bias calibration
				memcpy(&flash_buff[6], &telemetry_Rx[3], 48); //flash_buff[6]~flash_buff[17], 12*4
				memcpy(&bias.mag_hard, &flash_buff[6], 48);
				break;
			/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
			case 40: //Restart
				att_filter.flag = 0;
				break;
			/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
			case 81: //Read parameter
			case 82:
				DMA2_Stream7->CR &= ~DMA_SxCR_EN; //USART1_TX, DMA_Cmd(DMA2_Stream7, DISABLE)
				telemetry_mode = 0;
				telemetry_Tx[2] = telemetry_Rx[2]; //code
				switch(telemetry_Rx[2])
				{
					case 81: //Read bias
						telemetry_Tx[1] = 76; //3 + 18*4 + 1
						memcpy(&telemetry_Tx[3], &flash_buff[0], telemetry_Tx[1]-4); //flash_buff[0]~flash_buff[17]
						break;
					case 82: //Read att filter
						telemetry_Tx[1] = 72; //3 + 17*4 + 1
						memcpy(&telemetry_Tx[3], &flash_buff[18], telemetry_Tx[1]-4); //flash_buff[18]~flash_buff[24]
						break;
					default:
						break;
				}
				sum = 0;
				for(i=0; i<(telemetry_Tx[1]-1); i++)
					sum ^= telemetry_Tx[i];
				telemetry_Tx[telemetry_Tx[1]-1] = sum;
				DMA2_Stream7->NDTR = telemetry_Tx[1];
				DMA2_Stream7->CR |= DMA_SxCR_EN; //USART1_TX, DMA_Cmd(DMA2_Stream7, ENABLE)
				break;
			/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
			case 91: //Write bias
				memcpy(&flash_buff[0], &telemetry_Rx[3], 72); //flash_buff[0]~flash_buff[17], 18*4
				memcpy(&bias, &flash_buff[0], 72);
				break;
			case 92: //Write att filter
				memcpy(&flash_buff[18], &telemetry_Rx[3], 68); //flash_buff[18]~flash_buff[24], 17*4
				break;
			/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
			case 0xFF: //Save
				flash_write(flash_buff, FLASH_BUFF_SIZE);
				break;
			default:
				break;
		}
	}
}
