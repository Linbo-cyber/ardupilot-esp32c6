#pragma once
#include <stdint.h>

typedef struct {
    char name[16];
    float value;
    float min;
    float max;
    uint8_t type; // MAV_PARAM_TYPE
} param_entry_t;

void params_init(void);
void params_save(void);
int params_count(void);
param_entry_t *params_get_by_index(int idx);
param_entry_t *params_get_by_name(const char *name);
int params_set(const char *name, float value);
