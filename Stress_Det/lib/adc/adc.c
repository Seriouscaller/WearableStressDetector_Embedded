// ADC - Used for sampling the voltagedivider to get battery voltage

#include "board_config.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "types.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const static char *TAG = "ADC";
extern bool show_battery_log;
extern device_control_t device_config;
extern float battery_percentage;

static esp_err_t adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                      adc_cali_handle_t *adc_cali_handle);

#define ADC_SAMPLES 10
#define MCU_MINIMUM_OP_VOLTAGE 3.4f
#define BATTERY_MAX_VOLTAGE 4.2f
#define V_DIVIDER_R1 33000.0f
#define V_DIVIDER_R2 100000.0f
#define V_DIVIDER_RATIO ((V_DIVIDER_R1 + V_DIVIDER_R2) / V_DIVIDER_R2)

/**
 * @brief Initializes ADC1 in oneshot mode and applies calibration.
 *
 * This function configures the ADC unit, sets up the specific channel with
 * 12dB attenuation (allowing for full-scale voltage swing up to ~3.1V),
 * and initializes the calibration handles required to convert raw digital
 * readings into accurate millivolt values.
 *
 * @note Designed for the Seeed XIAO ESP32-S3 (ADC_UNIT_1, ADC_CHANNEL_0).
 *
 * @param[out] adc_handle      Pointer to the ADC oneshot unit handle to be initialized.
 * @param[out] adc_cali_handle Pointer to the ADC calibration handle to be initialized.
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid arguments
 *      - ESP_ERR_NO_MEM: Out of memory
 *      - ESP_FAIL: Calibration initialization failed or other error
 */
esp_err_t init_adc(adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *adc_cali_handle)
{
    esp_err_t ret = ESP_FAIL;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, adc_handle));

    // ADC_ATTEN_DB_12 provides the widest input voltage range (approx. 0 - 3100mV)
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, ADC_CHANNEL_0, &config));

    ret = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, adc_cali_handle);

    return ret;
}

/**
 * @brief  Initializes the ADC calibration scheme using curve fitting.
 *
 * Creates a calibration handle based on the ADC unit, channel, and attenuation.
 * This is crucial for the ESP32-S3 to provide accurate voltage readings, as it
 * compensates for chip-to-chip variations in the internal reference voltage.
 *
 * @param[in]  unit            The ADC unit (e.g., ADC_UNIT_1).
 * @param[in]  channel         The ADC channel associated with the sensor.
 * @param[in]  atten           The attenuation level used for the channel configuration.
 * @param[out] adc_cali_handle Pointer to the handle where the created scheme will be stored.
 *
 * @return
 *      - ESP_OK: Calibration scheme created successfully.
 *      - ESP_FAIL: Calibration initialization failed (e.g., scheme not supported or invalid config).
 */
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

/**
 * @brief  Reads the battery voltage by averaging multiple ADC samples.
 *
 * Performs oversampling to mitigate electrical noise, providing a more stable
 * voltage reading. It reads raw data from the specified ADC channel and
 * converts it to millivolts using the provided calibration handle.
 *
 * @param[in]  adc1_handle             Handle for the ADC oneshot unit.
 * @param[in]  adc1_cali_chan0_handle  Handle for the ADC calibration scheme.
 * @param[out] adc_raw                 Pointer to store the last raw ADC reading.
 * @param[out] voltage_mV              Pointer to store the averaged voltage in millivolts.
 *
 * @return
 *      - ESP_OK:   Successfully sampled and averaged the voltage.
 *      - ESP_FAIL: Failed to acquire any valid samples.
 *
 * @note The constant 'ADC_SAMPLES' must be defined in your configuration (e.g., 16 or 64).
 */
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

/**
 * @brief  Calculates and logs the battery voltage and charge percentage.
 *
 * This function converts the measured pin voltage into the actual battery voltage
 * using the configured divider ratio. It then estimates the remaining charge
 * using a linear approximation between the maximum battery voltage and the
 * minimum operating voltage of the MCU.
 *
 * @param[in] adc_raw    Pointer to the most recent raw ADC digital value.
 * @param[in] voltage_mV Pointer to the calibrated millivolt reading at the GPIO pin.
 *
 * @note The calculation assumes a linear discharge curve. While not chemically
 *       perfect for LiPo batteries, it provides a sufficient estimate for UI purposes.
 *
 * @see V_DIVIDER_RATIO
 * @see MCU_MINIMUM_OP_VOLTAGE
 * @see BATTERY_MAX_VOLTAGE
 */
void log_battery_voltage(int *adc_raw, int *voltage_mV)
{
    float pin_v = *voltage_mV / 1000.0f;
    float battery_v = pin_v * V_DIVIDER_RATIO;

    battery_percentage =
        ((battery_v - MCU_MINIMUM_OP_VOLTAGE) / (BATTERY_MAX_VOLTAGE - MCU_MINIMUM_OP_VOLTAGE) * 100);

    if (battery_percentage < 0)
        battery_percentage = 0;
    if (battery_percentage > 100)
        battery_percentage = 100;

    if (device_config.show_battery_log) {
        ESP_LOGI(TAG, "Raw: %d Pin: %.2fV Battery: %.2fV Charge: %.2f%%", *adc_raw, pin_v, battery_v,
                 battery_percentage);
    }
}