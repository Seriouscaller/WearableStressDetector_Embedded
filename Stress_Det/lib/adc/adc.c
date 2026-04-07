// ADC - Used for sampling the voltagedivider to get battery voltage

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const static char *TAG = "ADC";
extern bool show_battery_log;

static esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                      adc_cali_handle_t *adc_cali_handle);

#define ADC_SAMPLES 10
#define MCU_MINIMUM_OP_VOLTAGE 3.4f
#define BATTERY_MAX_VOLTAGE 4.2f
#define V_DIVIDER_R1 33000.0f
#define V_DIVIDER_R2 100000.0f
#define V_DIVIDER_RATIO ((V_DIVIDER_R1 + V_DIVIDER_R2) / V_DIVIDER_R2)

esp_err_t init_adc(adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *adc_cali_handle)
{
    esp_err_t ret = ESP_FAIL;
    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, adc_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, ADC_CHANNEL_0, &config));

    //-------------ADC1 Calibration Init---------------//
    ret = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, adc_cali_handle);

    return ret;
}

// ADC Calibration
static esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                      adc_cali_handle_t *adc_cali_handle)
{
    esp_err_t ret = ESP_FAIL;

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, adc_cali_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibration Failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Calibration Success");
    return ret;
}

esp_err_t read_battery_voltage(adc_oneshot_unit_handle_t *adc1_handle,
                               adc_cali_handle_t *adc1_cali_chan0_handle, int *adc_raw, int *voltage_mV)
{
    int voltage_sum = 0;
    int num_of_samples = 0;
    // Taking multiple samples and averging to reduce noise in ADC reading
    for (int i = 0; i < ADC_SAMPLES; i++) {
        if (adc_oneshot_read(*adc1_handle, ADC_CHANNEL_0, adc_raw) == ESP_OK) {
            if (adc_cali_raw_to_voltage(*adc1_cali_chan0_handle, *adc_raw, voltage_mV) == ESP_OK) {
                voltage_sum += *voltage_mV;
                num_of_samples++;
            }
        }
    }
    if (num_of_samples > 0) {
        *voltage_mV = voltage_sum / num_of_samples;
        return ESP_OK;
    }
    return ESP_FAIL;
}

void log_battery_voltage(int *adc_raw, int *voltage_mV)
{
    float pin_v = *voltage_mV / 1000.0f;
    float battery_v = pin_v * V_DIVIDER_RATIO;

    float battery_percentage =
        ((battery_v - MCU_MINIMUM_OP_VOLTAGE) / (BATTERY_MAX_VOLTAGE - MCU_MINIMUM_OP_VOLTAGE) * 100);

    if (battery_percentage < 0)
        battery_percentage = 0;
    if (battery_percentage > 100)
        battery_percentage = 100;

    if (show_battery_log) {
        ESP_LOGI(TAG, "Raw: %d Pin: %.2fV Battery: %.2fV Charge: %.2f%%", *adc_raw, pin_v, battery_v,
                 battery_percentage);
    }
}