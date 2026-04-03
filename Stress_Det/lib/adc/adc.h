#pragma once
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

esp_err_t init_adc(adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *adc_cali_handle);
esp_err_t read_battery_voltage(adc_oneshot_unit_handle_t *adc1_handle,
                               adc_cali_handle_t *adc1_cali_chan0_handle, int *adc_raw, int *voltage);
void log_battery_voltage(int *adc_raw, int *voltage);