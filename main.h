#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "ADIS16448.h"
#include "NEO6M.h"
#include "inertial.h"

#define FLASH_BUFF_SIZE     80
extern uint32_t flash_buff[FLASH_BUFF_SIZE];
extern uint32_t run_time;
extern IMU_DATA imu;
extern IMU_BIAS bias;
extern INER_NAV iner;
extern GPS_NAV gps;
extern ATT_FILTER att_filter;

void Timer_Config(void);
void Variable_Init(void);
void imu_read(ADIS16448_DATA *raw, IMU_DATA *imu);
void gps_read(nmeaINFO *info, GPS_NAV *gps);

#ifdef __cplusplus
}
#endif

#endif
