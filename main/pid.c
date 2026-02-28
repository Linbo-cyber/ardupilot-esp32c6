#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float out_min, float out_max, float i_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->i_max = i_max;
}

float pid_update(pid_t *pid, float error, float dt) {
    pid->integral += error * dt;
    // anti-windup
    if (pid->integral > pid->i_max) pid->integral = pid->i_max;
    if (pid->integral < -pid->i_max) pid->integral = -pid->i_max;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_reset(pid_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
