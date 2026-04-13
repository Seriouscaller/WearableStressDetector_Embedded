// Extracts derived features from PPG and GSR values, such as
// HRV and tonic, phasic values.

#include "board_config.h"
#include "esp_log.h"
#include "ppg_hrv.h"
#include "ppg_peaks.h"
#include "types.h"
#include <stdio.h>

#define SAMPLE_RATE 200
#define TOTAL_SAMPLES 6000
#define US_PER_SAMPLE (1000000 / SAMPLE_RATE) // 5000uS
#define MAX_PEAKS 100                         // Maximum expected beats in 30s

typedef struct {
    uint32_t timestamps_us[MAX_PEAKS];
    uint32_t ibi_ms[MAX_PEAKS];
    int peak_count;
    float average_hr;
} pulse_results_t;

som_input_t calculate_features(raw_data_t history[], uint16_t window_size);
static void detect_peaks(raw_data_t *signal, pulse_results_t *results);

static const char *TAG = "SIGNAL_P";

// Template for feature calculations on PPG & GSR
som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    // Extract PPG & EDA features here. Append to the som_input
    // Dont forget to normalize into floats!

    som_input_t features = {0};

    pulse_results_t results = {0};

    detect_peaks(&history[0], &results);

    results.average_hr = results.peak_count * 2.0f;

    ppg_hrv_init();

    for (int i = 1; i < results.peak_count; i++) {

        uint32_t t1 = results.timestamps_us[i - 1];
        uint32_t t2 = results.timestamps_us[i];

        float rr_ms = (t2 - t1) / 1000.0f;
        float time_s = t2 / 1000000.0f;

        ESP_LOGI(TAG, "RR: %.1f ms", rr_ms);

        ppg_add_rr(rr_ms, time_s);

        // ESP_LOGI(TAG, "RR t1:%u t2:%u diff_us:%u", t1, t2, (t2 - t1));

        // ESP_LOGI(TAG, "IBI:%.1f", rr_ms);
    }

    float now_s = results.timestamps_us[results.peak_count - 1] / 1000000.0f;

    ppg_features_t hrv = ppg_compute_hrv(now_s);

    ESP_LOGI(TAG, "peaks:%d bpm:%.2f hr:%.2f rmssd:%.2f sdnn:%.2f", results.peak_count, results.average_hr,
             hrv.hr, hrv.rmssd, hrv.sdnn);

    return features;
}

static void detect_peaks(raw_data_t *signal, pulse_results_t *results)
{
    ppg_peaks_init();

    results->peak_count = 0;

    int last_peak_index = -1000;

    for (int i = 0; i < TOTAL_SAMPLES; i++) {

        float x = signal[i].ppg_filtered;

        /*if (ppg_detect_peak(x)) {

            results->timestamps_us[results->peak_count] = (uint32_t)i * US_PER_SAMPLE;
            results->peak_count++;

            if (results->peak_count >= MAX_PEAKS)
                break;
        }*/

        if (ppg_detect_peak(x)) {

            uint32_t timestamp = (uint32_t)i * US_PER_SAMPLE;

            ESP_LOGI(TAG, "PEAK i:%d val:%.2f", i, x);

            results->timestamps_us[results->peak_count] = timestamp;
            results->peak_count++;
            last_peak_index = i;
        }
    }
}
