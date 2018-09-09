#ifndef __NEO6M_H
#define __NEO6M_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"
#include "nmea.h"

extern uint8_t NEO6M_flag;
extern nmeaINFO NEO6M_info;

void NEO6M_Config(void);
void NEO6M_read(void);

#ifdef __cplusplus
}
#endif

#endif
