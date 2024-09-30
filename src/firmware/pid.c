#include "pid.h"
#include "util.h"

int32_t pid_integrate(int16_t error, int32_t integral, uint16_t dt) {
	return integral + (error * (int16_t)dt);
}


float pid_differentiate(float de, uint16_t dt) {
	return de / (int16_t)dt;
}

uint8_t pid_process_get_current_percent(int16_t speed, int16_t setpoint, int32_t *pid_integral, int16_t *pid_last_error, uint32_t *last_time) {

	uint32_t now = system_x100us();
	uint16_t dt = CLAMP(now - *last_time, 0, 100); // assume the clock runs at least every 10ms...
	*last_time = now;

	int16_t error = setpoint - speed;

	*pid_integral = pid_integrate(error, *pid_integral, dt);

	// TODO: realistic? windup limiter numbers
	*pid_integral = CLAMP(*pid_integral, -800000.0f, 800000.0f);

	int16_t de = error - *pid_last_error;
	*pid_last_error = error;

	float derivative = pid_differentiate(de, dt);

	int32_t requested = (int32_t)( (PID_KP * error) + (PID_KI * *pid_integral) + (PID_KD * derivative)  );

	return (uint8_t)CLAMP(requested, 0, 100);


}

