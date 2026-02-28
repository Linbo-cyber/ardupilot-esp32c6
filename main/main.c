#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "config.h"
#include "mavlink/common/mavlink.h"
#include "mavlink_handler.h"
#include "battery.h"
#include "pwm_output.h"
#include "params.h"
#include "pid.h"
#include "wifi_link.h"

#define UART_PORT       UART_NUM_0
#define UART_TX_PIN     16
#define UART_RX_PIN     17
#define UART_BUF_SIZE   1024

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

// ── UART send ───────────────────────────────────
static void uart_send(const uint8_t *buf, int len) {
    uart_write_bytes(UART_PORT, buf, len);
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

        // ── UART MAVLink receive ──
        uint8_t uart_buf[280];
        int n = uart_read_bytes(UART_PORT, uart_buf, sizeof(uart_buf), 0);
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                mavlink_handle_byte(uart_buf[i], 0);
            }
            last_rc_time = rover.uptime_ms;
        }

        // ── WiFi MAVLink receive ──
        uint8_t wifi_buf[280];
        n = wifi_link_receive(wifi_buf, sizeof(wifi_buf));
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

    // UART
    uart_config_t uart_cfg = {
        .baud_rate = USB_CDC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_cfg);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    mavlink_handler_set_usb_send(uart_send);

    // WiFi backup link
    wifi_link_init();

    // Start control loop
    xTaskCreate(control_task, "control", 8192, NULL, 5, NULL);

    mavlink_send_statustext("Rover C6 ready", MAV_SEVERITY_INFO);
}
