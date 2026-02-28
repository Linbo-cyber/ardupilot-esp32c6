#pragma once
#include <stdint.h>

void pwm_output_init(void);
void pwm_set_throttle(uint16_t us);
void pwm_set_steering(uint16_t us);
void pwm_failsafe(void);
