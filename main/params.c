#include "params.h"
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>

static param_entry_t params[] = {
    {"CRUISE_SPEED",   1.0f,  0.0f, 10.0f, 9},
    {"CRUISE_THROT",   50.0f, 0.0f, 100.0f, 9},
    {"STEER_P",        1.0f,  0.0f, 10.0f, 9},
    {"STEER_I",        0.1f,  0.0f, 5.0f,  9},
    {"STEER_D",        0.01f, 0.0f, 2.0f,  9},
    {"THR_P",          0.5f,  0.0f, 10.0f, 9},
    {"THR_I",          0.05f, 0.0f, 5.0f,  9},
    {"THR_D",          0.0f,  0.0f, 2.0f,  9},
    {"BATT_MONITOR",   1.0f,  0.0f, 1.0f,  9},
    {"BATT_LOW_V",     10.5f, 0.0f, 25.0f, 9},
    {"BATT_CRT_V",     9.6f,  0.0f, 25.0f, 9},
    {"BATT_DIVIDER",   4.0f,  1.0f, 20.0f, 9},
    {"FS_THR_ENABLE",  1.0f,  0.0f, 2.0f,  9},
    {"FS_THR_VALUE",   950.0f,800.0f,1100.0f,9},
    {"RC3_MIN",        1000.0f,500.0f,2500.0f,9},
    {"RC3_MAX",        2000.0f,500.0f,2500.0f,9},
    {"RC1_MIN",        1000.0f,500.0f,2500.0f,9},
    {"RC1_MAX",        2000.0f,500.0f,2500.0f,9},
    {"SYSID_THISMAV",  1.0f,  1.0f, 255.0f, 9},
    {"WP_RADIUS",      2.0f,  0.1f, 100.0f, 9},
};

static int param_count = sizeof(params) / sizeof(params[0]);

void params_init(void) {
    nvs_handle_t h;
    if (nvs_open("params", NVS_READONLY, &h) == ESP_OK) {
        for (int i = 0; i < param_count; i++) {
            uint32_t val;
            if (nvs_get_u32(h, params[i].name, &val) == ESP_OK) {
                float *fp = (float *)&val;
                params[i].value = *fp;
            }
        }
        nvs_close(h);
    }
}

void params_save(void) {
    nvs_handle_t h;
    if (nvs_open("params", NVS_READWRITE, &h) == ESP_OK) {
        for (int i = 0; i < param_count; i++) {
            uint32_t *vp = (uint32_t *)&params[i].value;
            nvs_set_u32(h, params[i].name, *vp);
        }
        nvs_commit(h);
        nvs_close(h);
    }
}

int params_count(void) {
    return param_count;
}

param_entry_t *params_get_by_index(int idx) {
    if (idx < 0 || idx >= param_count) return NULL;
    return &params[idx];
}

param_entry_t *params_get_by_name(const char *name) {
    for (int i = 0; i < param_count; i++) {
        if (strncmp(params[i].name, name, 16) == 0) return &params[i];
    }
    return NULL;
}

int params_set(const char *name, float value) {
    param_entry_t *p = params_get_by_name(name);
    if (!p) return -1;
    if (value < p->min) value = p->min;
    if (value > p->max) value = p->max;
    p->value = value;
    return 0;
}
