#pragma once
#include <stdint.h>

void battery_init(void);
float battery_read_voltage(void);
uint8_t battery_remaining_pct(void);
int battery_is_low(void);
int battery_is_critical(void);
