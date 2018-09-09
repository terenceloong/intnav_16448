#ifndef __ADIS16448_H
#define __ADIS16448_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"

#define XGYRO_OFF          0x1A
#define YGYRO_OFF          0x1C
#define ZGYRO_OFF          0x1E
#define XACCL_OFF          0x20
#define YACCL_OFF          0x22
#define ZACCL_OFF          0x24
#define XMAGN_HIC          0x26
#define YMAGN_HIC          0x28
#define ZMAGN_HIC          0x2A
#define XMAGN_SIC          0x2C
#define YMAGN_SIC          0x2E
#define ZMAGN_SIC          0x30
#define GPIO_CTRL          0x32
#define MSC_CTRL           0x34
#define SMPL_PRD           0x36
#define SENS_AVG           0x38
#define PROD_ID            0x56

typedef struct
{
    uint16_t invalid;
    uint16_t state;
    uint16_t gyrox;
    uint16_t gyroy;
    uint16_t gyroz;
    uint16_t accx;
    uint16_t accy;
    uint16_t accz;
    uint16_t magx;
    uint16_t magy;
    uint16_t magz;
    uint16_t baro;
    uint16_t temp;
}ADIS16448_DATA;

extern ADIS16448_DATA ADIS16448_data;
extern uint8_t ADIS16448_flag;
extern uint32_t ADIS16448_stamp;

void ADIS16448_Config(void);
void ADIS16448_Startup(void);
void ADIS16448_Reset(void);
uint16_t ADIS16448_ReadReg(uint8_t reg);
void ADIS16448_WriteReg(uint8_t reg, uint16_t value);
void ADIS16448_Save(void);
void ADIS16448_Restore(void);

#ifdef __cplusplus
}
#endif

#endif
