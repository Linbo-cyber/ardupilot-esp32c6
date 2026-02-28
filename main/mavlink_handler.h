#pragma once
#include <stdint.h>
#include "mavlink_types.h"

// rover state shared with mavlink handler
typedef struct {
    uint8_t mode;           // current mode
    uint8_t armed;          // 0=disarmed, 1=armed
    uint16_t rc_throttle;   // RC channel 3 (us)
    uint16_t rc_steering;   // RC channel 1 (us)
    float batt_voltage;
    uint8_t batt_pct;
    uint32_t uptime_ms;
    uint8_t failsafe;
} rover_state_t;

void mavlink_handler_init(void);
void mavlink_handle_byte(uint8_t c, int link_id);  // 0=USB, 1=WiFi
void mavlink_send_heartbeat(rover_state_t *state);
void mavlink_send_sys_status(rover_state_t *state);
void mavlink_send_rc_channels(rover_state_t *state);
void mavlink_send_statustext(const char *text, uint8_t severity);
