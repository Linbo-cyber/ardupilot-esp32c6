#include "pwm_output.h"
#include "config.h"
#include "driver/ledc.h"

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_14_BIT  // 16384 steps

static uint32_t us_to_duty(uint16_t us) {
    // period = 1/50Hz = 20000us, duty = us/20000 * 16384
    return (uint32_t)((float)us / 20000.0f * 16384.0f);
}

void pwm_output_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_throttle = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER,
        .gpio_num = PWM_THROTTLE_GPIO,
        .duty = us_to_duty(PWM_MID_US),
        .hpoint = 0,
    };
    ledc_channel_config(&ch_throttle);

    ledc_channel_config_t ch_steer = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER,
        .gpio_num = PWM_STEER_GPIO,
        .duty = us_to_duty(PWM_MID_US),
        .hpoint = 0,
    };
    ledc_channel_config(&ch_steer);
}

void pwm_set_throttle(uint16_t us) {
    if (us < PWM_MIN_US) us = PWM_MIN_US;
    if (us > PWM_MAX_US) us = PWM_MAX_US;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, us_to_duty(us));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}

void pwm_set_steering(uint16_t us) {
    if (us < PWM_MIN_US) us = PWM_MIN_US;
    if (us > PWM_MAX_US) us = PWM_MAX_US;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, us_to_duty(us));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
}

void pwm_failsafe(void) {
    pwm_set_throttle(PWM_MID_US);
    pwm_set_steering(PWM_MID_US);
}
