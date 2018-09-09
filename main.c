#include <string.h>
#include <math.h>
#include "stm32f4xx.h"
#include "main.h"
#include "flash.h"
#include "telemetry.h"
#include "uclock.h"

// global variable
uint32_t flash_buff[FLASH_BUFF_SIZE];
uint32_t run_time = 0;
IMU_DATA imu;
IMU_BIAS bias;
INER_NAV iner;
GPS_NAV gps;
ATT_FILTER att_filter;

uint16_t reg = 0;
uint32_t time0, time1;

/*--------------------------------------------------------------------------------------*/
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

//	Timer_Config();
	Telemetry_Config();
	ADIS16448_Config();
//	NEO6M_Config();
//	Variable_Init();
	UClock_Config();
	
	ADIS16448_Startup();

/*~~~~~~~~~~~~~~~~~~~~~~~~~~Debug~~~~~~~~~~~~~~~~~~~~~~~~~~*/
//	ADIS16448_WriteReg(MSC_CTRL, 0x0006);
//	ADIS16448_WriteReg(SMPL_PRD, 0x0300);
//	ADIS16448_WriteReg(SENS_AVG, 0x0104);
	
//	reg =  ADIS16448_ReadReg(MSC_CTRL);
//	reg =  ADIS16448_ReadReg(SMPL_PRD);
//	reg =  ADIS16448_ReadReg(SENS_AVG);
//	reg =  ADIS16448_ReadReg(PROD_ID);
	
//	ADIS16448_Save();
	
//	memset(flash_buff, 0, sizeof(flash_buff));
//	flash_write(flash_buff, FLASH_BUFF_SIZE);
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

//	while(1)
//	{
//		if(CMD_flag == 1)
//		{
//			telemetry_cmd();
//			CMD_flag = 0;
//		}
//		
//		if(ADIS16448_flag == 1)
//		{
//			time0 = TIM2->CNT;

//			imu_read(&ADIS16448_data, &imu);
//			imu_compensate(&imu, &bias);
//			if(NEO6M_flag == 1)
//			{
//				NEO6M_read();
//				gps_read(&NEO6M_info, &gps);
//			}
//			att_solve(&imu, &iner, &att_filter, (float*)&flash_buff[18]);
//			telemetry_data();
//			ADIS16448_flag = 0;
//			NEO6M_flag = 0;

//			time1 = TIM2->CNT;
//			if(time1 > time0)
//			{
//				if((time1-time0) > 100)
//					run_time = time1 - time0;
//			}
//			else
//			{
//				if((time1+100000-time0) > 100)
//					run_time = time1 + 100000 - time0;
//			}
//		}
//	}

	while(1)
	{
		if(CMD_flag == 1)
		{
			telemetry_cmd();
			CMD_flag = 0;
		}
		
		if(ADIS16448_flag == 1)
		{
			imu_read(&ADIS16448_data, &imu);
			telemetry_data();
			ADIS16448_flag = 0;
		}
	}
}

/*--------------------------------------------------------------------------------------*/
void Timer_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 99999; //100ms
	TIM_TimeBaseStructure.TIM_Prescaler = 83; //1us
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_Cmd(TIM2, ENABLE);
}

void Variable_Init(void)
{
	flash_read(flash_buff, FLASH_BUFF_SIZE);
	memcpy(&bias, flash_buff, sizeof(IMU_BIAS));
	att_filter.flag = 0;
}

void imu_read(ADIS16448_DATA *raw, IMU_DATA *imu)
{
    int16_t data;

    data = (int16_t)((raw->gyrox<<8) | (raw->gyrox>>8));
    imu->gyro[0] = data * -0.01f;
    data = (int16_t)((raw->gyroy<<8) | (raw->gyroy>>8));
    imu->gyro[1] = data * 0.01f;
    data = (int16_t)((raw->gyroz<<8) | (raw->gyroz>>8));
    imu->gyro[2] = data * -0.01f;
    data = (int16_t)((raw->accx<<8) | (raw->accx>>8));
    imu->acc[0] = data / -1200.0f;
    data = (int16_t)((raw->accy<<8) | (raw->accy>>8));
    imu->acc[1] = data / 1200.0f;
    data = (int16_t)((raw->accz<<8) | (raw->accz>>8));
    imu->acc[2] = data / -1200.0f;
    data = (int16_t)((raw->magx<<8) | (raw->magx>>8));
    imu->mag[0] = data / -7.0f;
    data = (int16_t)((raw->magy<<8) | (raw->magy>>8));
    imu->mag[1] = data / 7.0f;
    data = (int16_t)((raw->magz<<8) | (raw->magz>>8));
    imu->mag[2] = data / -7.0f;
    data = (int16_t)((raw->temp<<8) | (raw->temp>>8));
    imu->temp = data * 0.07386f + 31.0f;
    imu->baro = (uint16_t)((raw->baro<<8) | (raw->baro>>8)) * 0.02f;
}

double latlon2deg(double pos)
{
	int16_t deg;
	double min;
	
	deg = (int16_t)(pos/100.0);
	min = pos - deg*100;
	
	return min/60.0 + deg;
}

#define D2R     0.01745329251994
double referpoint[5] = {45.741220, 126.627107, 126, 6368337, 4459053};

void gps_read(nmeaINFO *info, GPS_NAV *gps)
{
	gps->pos_lla[0] = latlon2deg(info->lat);
	gps->pos_lla[1] = latlon2deg(info->lon);
	gps->pos_lla[2] = info->elv;
	gps->pos_xyz[0] = (gps->pos_lla[0] - referpoint[0]) * D2R * referpoint[3];
	gps->pos_xyz[1] = (gps->pos_lla[1] - referpoint[1]) * D2R * referpoint[4];
	gps->pos_xyz[2] =  gps->pos_lla[2] - referpoint[2];
	gps->vel_vd[0]  = info->speed / 3.6;
	gps->vel_vd[1]  = info->direction;
	gps->vel_xyz[0] = info->speed * cos(info->direction * D2R);
	gps->vel_xyz[1] = info->speed * sin(info->direction * D2R);
}
