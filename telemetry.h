#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#ifdef __cplusplus
 extern "C" {
#endif

extern uint8_t CMD_flag;

void Telemetry_Config(void);
void telemetry_data(void);
void telemetry_cmd(void);

#ifdef __cplusplus
}
#endif

#endif
