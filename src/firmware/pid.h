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

typedef struct {
	float kp;
	float ki;
	float kd;
	float k;
} pid_consts_t;

typedef struct {
	int32_t integral;
	int16_t last_error;
	uint32_t last_time;
} pid_vars_t;

int32_t pid_integrate(int16_t error, int32_t integral, uint16_t dt);

float pid_differentiate(float de, uint16_t dt);

uint8_t pid_process_get_current_percent(int16_t speed, int16_t setpoint, pid_consts_t *pid_consts, pid_vars_t *pid_vars);

void pid_load_consts(uint16_t kp_x1000, uint16_t ki_denom, uint16_t kd_x1000, uint16_t mult_x1000, pid_consts_t *pid_consts);

#endif
