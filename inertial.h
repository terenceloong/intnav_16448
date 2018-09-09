#ifndef __INERTIAL_H
#define __INERTIAL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float gyro[3];
    float acc[3];
    float mag[3];
    float baro;
    float temp;
}IMU_DATA;

typedef struct
{
    float gyro[3];
    float acc[3];
    float mag_hard[3];
    float mag_soft[3][3];
}IMU_BIAS;

typedef struct
{
    float  att[3];
    float  att_am[3];
    float  vel_vd[3];
    float  vel_xyz[3];
    double pos_lla[3];
    float  pos_xyz[3];
}INER_NAV;

typedef struct
{
    float  vel_vd[2];
    float  vel_xyz[2];
    double pos_lla[3];
    float  pos_xyz[3];
}GPS_NAV;

typedef struct
{
    uint8_t flag;
    uint32_t cnt;
    double X[7];
    double P[7][7];
    double Q[6];
    double R[4];
}ATT_FILTER;

void imu_compensate(IMU_DATA *imu, IMU_BIAS *bias);
void att_solve(IMU_DATA *imu, INER_NAV *iner, ATT_FILTER *filter, float *para);

#ifdef __cplusplus
}
#endif

#endif
