#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

#include "config.h"
#include "mavlink_handler.h"
#include "battery.h"
#include "pwm_output.h"
#include "params.h"
#include "pid.h"
#include "wifi_link.h"

static rover_state_t rover = {
    .mode = MODE_MANUAL,
    .armed = 0,
    .rc_throttle = PWM_MID_US,
    .rc_steering = PWM_MID_US,
    .batt_voltage = 12.6f,
    .batt_pct = 100,
    .uptime_ms = 0,
    .failsafe = 0,
};

static pid_t steer_pid;
static pid_t throttle_pid;
static uint32_t last_rc_time = 0;

// ── USB CDC ─────────────────────────────────────
static void usb_cdc_send(const uint8_t *buf, int len) {
    tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, buf, len);
    tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
}

static void usb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    uint8_t buf[256];
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, sizeof(buf), &rx_size);
    if (ret == ESP_OK) {
        for (size_t i = 0; i < rx_size; i++) {
            mavlink_handle_byte(buf[i], 0);
        }
        last_rc_time = (uint32_t)(esp_timer_get_time() / 1000);
    }
}

// ── Control Loop Task ───────────────────────────
static void control_task(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / LOOP_RATE_HZ);

    uint32_t hb_counter = 0;
    uint32_t status_counter = 0;

    while (1) {
        rover.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // ── Battery ──
        rover.batt_voltage = battery_read_voltage();
        rover.batt_pct = battery_remaining_pct();

        // ── Failsafe: no MAVLink for 1.5s ──
        if (rover.uptime_ms - last_rc_time > 1500) {
            if (!rover.failsafe) {
                rover.failsafe = 1;
                mavlink_send_statustext("FAILSAFE: no RC input", MAV_SEVERITY_CRITICAL);
            }
            pwm_failsafe();
        } else {
            rover.failsafe = 0;
        }

        // ── Battery failsafe ──
        if (battery_is_critical()) {
            rover.armed = 0;
            pwm_failsafe();
        }

        // ── PWM output ──
        if (rover.armed && !rover.failsafe) {
            if (rover.mode == MODE_MANUAL) {
                pwm_set_throttle(rover.rc_throttle);
                pwm_set_steering(rover.rc_steering);
            } else if (rover.mode == MODE_HOLD) {
                pwm_failsafe();
            }
        } else {
            pwm_failsafe();
        }

        // ── WiFi MAVLink receive ──
        uint8_t wifi_buf[280];
        int n = wifi_link_receive(wifi_buf, sizeof(wifi_buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                mavlink_handle_byte(wifi_buf[i], 1);
            }
            last_rc_time = rover.uptime_ms;
        }

        // ── Heartbeat 1Hz ──
        hb_counter++;
        if (hb_counter >= LOOP_RATE_HZ) {
            hb_counter = 0;
            mavlink_send_heartbeat(&rover);
        }

        // ── SYS_STATUS + RC 4Hz ──
        status_counter++;
        if (status_counter >= LOOP_RATE_HZ / 4) {
            status_counter = 0;
            mavlink_send_sys_status(&rover);
            mavlink_send_rc_channels(&rover);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

// ── Arm/Disarm handler (called from mavlink_handler) ──
extern void mavlink_handler_set_state(rover_state_t *state);
extern void mavlink_handler_set_usb_send(void (*fn)(const uint8_t *, int));

void app_main(void) {
    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Init subsystems
    params_init();
    battery_init();
    pwm_output_init();
    mavlink_handler_init();
    mavlink_handler_set_state(&rover);

    // USB CDC
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
    };
    tinyusb_driver_install(&tusb_cfg);

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = &usb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL,
    };
    tusb_cdc_acm_init(&acm_cfg);
    mavlink_handler_set_usb_send(usb_cdc_send);

    // WiFi backup link
    wifi_link_init();

    // Start control loop
    xTaskCreate(control_task, "control", 8192, NULL, 5, NULL);

    mavlink_send_statustext("Rover C6 ready", MAV_SEVERITY_INFO);
}
