// Extracts derived features from PPG and GSR values, such as
// HRV and tonic, phasic values.

#include "board_config.h"
#include "esp_log.h"
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
static float calculate_threshold(raw_data_t *signal, int length);
static void detect_peaks(raw_data_t *signal, float threshold, pulse_results_t *results);

static const char *TAG = "SIGNAL_P";

// Template for feature calculations on PPG & GSR
som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    // Extract PPG & EDA features here. Append to the som_input
    // Dont forget to normalize into floats!

    som_input_t features = {0};
    /*
    pulse_results_t results = {0};

    // 1. Get the dynamic threshold
    float threshold = calculate_threshold(&history[0], TOTAL_SAMPLES);
    printf(">Threshold: %.3f\n", threshold);

    // 2. Identify peaks
    detect_peaks(&history[0], threshold, &results);

    // 3. Extract Heart Rate
    // calculate_metrics(&results);

    // 4. Output
    printf("Detected %d beats. Average HR: %.2f BPM\n", results.peak_count, results.average_hr);
    for (int i = 0; i < results.peak_count - 1; i++) {
        printf(">IBI nr %d\n", i);
        printf(">IBI val: %lu ms\n", results.ibi_ms[i]);
    }*/

    return features;
}

static float calculate_threshold(raw_data_t *signal, int length)
{
    float max_val = signal[0].ppg_filtered;
    float min_val = signal[0].ppg_filtered;

    for (int i = 1; i < length; i++) {
        if (signal[i].ppg_filtered > max_val)
            max_val = signal[i].ppg_filtered;
        if (signal[i].ppg_filtered < min_val)
            min_val = signal[i].ppg_filtered;
    }

    // Set threshold at 60% of the peak-to-peak height above the minimum
    return min_val + (max_val - min_val) * 0.60f;
}

static void detect_peaks(raw_data_t *signal, float threshold, pulse_results_t *results)
{
    bool looking_for_peak = true;
    int refractory_samples = (300 * 1000) / US_PER_SAMPLE; // 300ms worth of samples
    int last_peak_index = -refractory_samples;

    results->peak_count = 0;

    for (int i = 1; i < TOTAL_SAMPLES - 1; i++) {
        // 1. Check if signal is above threshold and we are outside refractory period
        if (signal[i].ppg_filtered > threshold && (i - last_peak_index) > refractory_samples) {

            // 2. Local Maximum Check: Is this point higher than its neighbors?
            if (signal[i].ppg_filtered > signal[i - 1].ppg_filtered &&
                signal[i].ppg_filtered > signal[i + 1].ppg_filtered) {

                // Peak confirmed
                results->timestamps_us[results->peak_count] = (uint32_t)i * US_PER_SAMPLE;
                results->peak_count++;
                last_peak_index = i;

                if (results->peak_count >= MAX_PEAKS)
                    break;
            }
        }
    }
}