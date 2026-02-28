#include "battery.h"
#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static float voltage_filtered = 12.6f;

void battery_init(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, BATT_ADC_CHANNEL, &chan_cfg);

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
}

float battery_read_voltage(void) {
    int raw = 0;
    int mv = 0;
    adc_oneshot_read(adc_handle, BATT_ADC_CHANNEL, &raw);
    adc_cali_raw_to_voltage(cali_handle, raw, &mv);
    float v = (mv / 1000.0f) * BATT_DIVIDER_RATIO;
    // low-pass filter
    voltage_filtered = voltage_filtered * 0.95f + v * 0.05f;
    return voltage_filtered;
}

uint8_t battery_remaining_pct(void) {
    float cell_v = voltage_filtered / BATT_CELLS;
    // simple linear mapping: 3.2V=0%, 4.2V=100%
    float pct = (cell_v - 3.2f) / (4.2f - 3.2f) * 100.0f;
    if (pct > 100.0f) pct = 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    return (uint8_t)pct;
}

int battery_is_low(void) {
    return voltage_filtered < BATT_LOW_VOLTAGE;
}

int battery_is_critical(void) {
    return voltage_filtered < BATT_CRIT_VOLTAGE;
}
