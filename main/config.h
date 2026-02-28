#pragma once

// ── Vehicle ─────────────────────────────────────
#define VEHICLE_TYPE        10      // MAV_TYPE_GROUND_ROVER
#define SYSTEM_ID           1
#define COMPONENT_ID        1       // MAV_COMP_ID_AUTOPILOT1
#define FIRMWARE_VERSION    0x04060000  // 4.6.0

// ── Control Loop ────────────────────────────────
#define LOOP_RATE_HZ        50      // main control loop
#define PID_DT              (1.0f / LOOP_RATE_HZ)

// ── PWM Output ──────────────────────────────────
#define PWM_FREQ_HZ         50
#define PWM_THROTTLE_GPIO   4
#define PWM_STEER_GPIO      5
#define PWM_MIN_US          1000
#define PWM_MID_US          1500
#define PWM_MAX_US          2000

// ── Battery ADC ─────────────────────────────────
#define BATT_ADC_CHANNEL    ADC_CHANNEL_0   // GPIO0
#define BATT_ADC_ATTEN      ADC_ATTEN_DB_12
// Voltage divider: R1=30k, R2=10k → ratio = (30+10)/10 = 4.0
#define BATT_DIVIDER_RATIO  4.0f
#define BATT_CELLS          3
#define BATT_LOW_VOLTAGE    10.5f   // 3S low = 3.5V * 3
#define BATT_CRIT_VOLTAGE   9.6f    // 3S crit = 3.2V * 3

// ── WiFi (backup link) ─────────────────────────
#define WIFI_AP_SSID        "Rover-C6"
#define WIFI_AP_PASS        "ardupilot"
#define WIFI_AP_CHANNEL     6
#define WIFI_UDP_PORT       14550

// ── USB CDC MAVLink ─────────────────────────────
#define USB_CDC_BAUD        115200

// ── Modes ───────────────────────────────────────
#define MODE_MANUAL         0
#define MODE_HOLD           4
#define MODE_AUTO           10

// ── Params ──────────────────────────────────────
#define PARAM_COUNT_MAX     32
