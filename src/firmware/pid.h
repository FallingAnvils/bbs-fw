#ifndef _PID_H
#define _PID_H

#include <stdint.h>

// decent
//#define PID_KP 0.42f
//#define PID_KI 0.f/750.0f
//#define PID_KD 0.40f

#define PID_KP 0.42f
#define PID_KI 1.f/32000.0f
#define PID_KD 0.40f


int32_t pid_integrate(int16_t error, int32_t integral, uint16_t dt);

float pid_differentiate(float de, uint16_t dt);

uint8_t pid_process_get_current_percent(int16_t speed, int16_t setpoint, int32_t *pid_integral, int16_t *pid_last_error, uint32_t *last_time);

#endif
