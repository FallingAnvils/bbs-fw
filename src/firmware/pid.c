#include "pid.h"
#include "util.h"
#include "system.h"

int32_t pid_integrate(int16_t error, int32_t integral, uint16_t dt) {
	return integral + (error * (int16_t)dt);
}


float pid_differentiate(float de, uint16_t dt) {
	return de / (int16_t)dt;
}

uint8_t pid_process_get_current_percent(int16_t speed, int16_t setpoint, pid_consts_t *pid_consts, pid_vars_t *pid_vars) {

	uint32_t now = system_x100us();
	uint16_t dt = CLAMP(now - pid_vars->last_time, 0, 100); // assume the clock runs at least every 10ms...
	pid_vars->last_time = now;

	int16_t error = setpoint - speed;

	pid_vars->integral = pid_integrate(error, pid_vars->integral, dt);

	// TODO: realistic? windup limiter numbers
	pid_vars->integral = CLAMP(pid_vars->integral, -800000, 800000);

	int16_t de = error - pid_vars->last_error;
	pid_vars->last_error = error;

	float derivative = pid_differentiate(de, dt);

	int32_t requested = (int32_t)( pid_consts->k * ((pid_consts->kp * error) + (pid_consts->ki * pid_vars->integral) + (pid_consts->kd * derivative))  );

	return (uint8_t)CLAMP(requested, 0, 100);


}

void pid_load_consts(uint16_t kp_x1000, uint16_t ki_denom, uint16_t kd_x1000, uint16_t mult_x1000, pid_consts_t *pid_consts) {
	pid_consts->kp = kp_x1000 / 1000.f;
	pid_consts->ki = 1.f / ki_denom;
	pid_consts->kd = kd_x1000 / 1000.f;
	pid_consts->k = mult_x1000 / 1000.f;
}

