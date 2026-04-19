// Extracts derived features from PPG and GSR values, such as
// HRV and tonic, phasic values.

#include "signal_processing.h"
#include "board_config.h"
#include "esp_log.h"
#include "float.h"
#include "ppg_hrv.h"
#include "ppg_peaks.h"
#include "test_signal.h"
#include "types.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define MINIMUM_AMOUNT_OF_DATA (FIVE_SEC * SAMPLE_RATE)
#define MAXIMUM_AMOUNT_OF_DATA (THIRTY_SEC * SAMPLE_RATE)
#define FIVE_SEC 5
#define THIRTY_SEC 30
#define SAMPLE_RATE 200
#define MIN_SYSTOLIC_THRESHOLD 15
#define MAX_PEAKS 100
#define TOO_MUCH_TIME_BETWEEN_HEARTBEATS 300
#define TOO_LITTLE_TIME_BETWEEN_HEARTBEATS MIN_SAMPLES_BETWEEN_BEATS
#define MIN_SAMPLES_BETWEEN_BEATS TWO_HUNDRED_FIFTY_MS_IN_SMPLS
#define JITTER_THRESHOLD_ZERO_CROSSING 20.0f
#define MAX_RR_INVERVAL 1500
#define MIN_RR_INVERVAL 300
#define MIN_VALLEY_THRESHOLD -7
#define MIN_PEAKS_FOR_HEARTRATE 5
#define MIN_PEAKS_FOR_SDNN 25
#define MIN_VALID_RR_MS 600
#define MAX_VALID_RR_MS 1300

#define TWO_HUNDRED_FIFTY_MS_IN_SMPLS 50
#define THREE_HUNDRED_MS_IN_SMPLS 60
#define FIVE_HUNDRED_MS_IN_SMPLS 100
#define SECONDS_PER_MINUTE 60.0f

#define PEAK_AVG_ALPHA 0.1f
#define THRESHOLD_RATIO 0.5F
#define MIN_DYNAMIC_THRESHOLD 1.5f

#define PEAK_WINDOW 5

// 200 samples = 1 sec
// 100 samp 0.5 sec
//  50 samp 0.25 sec

bool debug_zero_cross_algo = false;
bool debug_peak_found = false;
bool debug_show_heartbeat_stats = true;
bool debug_zero_cross = false;
bool debug_hr_features = false;
bool debug_show_all_intervals = false;
bool show_new_intervals = false;

enum SignalState {
    ABOVE = 1,
    BELOW = -1,
};

typedef struct {
    float local_min;
    float local_max;
    float current_min_val;
    float current_max_val;
    uint16_t current_min_index;
    uint16_t current_max_index;
    uint16_t valley_count;
    uint16_t peaks_count;
    uint16_t valley_idx[MAX_PEAKS];
    uint16_t peaks_idx[MAX_PEAKS];
    uint16_t zero_crossings;
    uint16_t double_peaks;
} signal_data_t;

typedef struct {
    float rr_intervals[MAX_PEAKS];
    int32_t succ_diff[MAX_PEAKS];
    uint8_t num_of_diffs;
    float diff_mean;
    float avg_hr;
    float rmssd;
    float sdnn;
} heart_beat_stats_t;

som_input_t calculate_features(raw_data_t history[], uint16_t window_size);
static void calculate_heart_rate(heart_beat_stats_t *hb_data, signal_data_t *s_data, raw_data_t *history);
static void calculate_rmssd(heart_beat_stats_t *data);
static heart_beat_stats_t calculate_rr_intervals(signal_data_t *data, raw_data_t history[],
                                                 uint16_t window_size);
static signal_data_t peak_detector(raw_data_t history[], uint16_t window_size);
static bool is_signal_data_valid(signal_data_t *data);
static void calculate_sdnn(heart_beat_stats_t *data, signal_data_t *s_data);

static const char *TAG = "S_PR";

som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    som_input_t features = {0};

    signal_data_t peak_data = peak_detector(history, window_size);
    heart_beat_stats_t pulse_data = calculate_rr_intervals(&peak_data, history, window_size);
    calculate_heart_rate(&pulse_data, &peak_data, history);
    calculate_sdnn(&pulse_data, &peak_data);

    ESP_LOGI(TAG, "Peaks: %u HR: %.1f SDNN: %.1f", peak_data.peaks_count, pulse_data.avg_hr, pulse_data.sdnn);

    return features;
}

static signal_data_t peak_detector(raw_data_t history[], uint16_t window_size)
{
    enum SignalState signal_position;

    float running_peak_avg = 200.0f;
    float dynamic_threshold = 50.0f;

    // Init state
    if (history[0].ppg_filtered > 0) {
        signal_position = ABOVE;
    } else {
        signal_position = BELOW;
    }

    signal_data_t window_data = {0};
    window_data.local_min = FLT_MAX;
    window_data.local_max = -FLT_MAX;

    for (int i = 1; i < window_size; i++) {

        float sample = history[i].ppg_filtered;

        // ABOVE → BELOW peak done.
        if ((signal_position == ABOVE) && (sample < -JITTER_THRESHOLD_ZERO_CROSSING)) {

            window_data.zero_crossings++;
            signal_position = BELOW;

            uint16_t current_peak = window_data.current_max_index;

            // Check threshold, big enough to be a peak.
            if (window_data.local_max > dynamic_threshold && window_data.local_max < 500) {

                // 2. timing (refractory period)
                bool beat_detected_too_fast =
                    (window_data.peaks_count > 0) &&
                    (current_peak - window_data.peaks_idx[window_data.peaks_count - 1] <
                     TWO_HUNDRED_FIFTY_MS_IN_SMPLS);

                // 3. Peak found.
                if (!beat_detected_too_fast && window_data.peaks_count < MAX_PEAKS - 2) {
                    window_data.peaks_idx[window_data.peaks_count++] = current_peak;

                    if (debug_peak_found) {
                        ESP_LOGI(TAG, "Peak idx: %u Value: %.2f Threshold: %.2f", current_peak,
                                 window_data.local_max, dynamic_threshold);
                    }

                    // Exponential Moving Average (EMA).
                    // 0.9 of old value + 0.1 of new value (current peak) moving towards the new peak value
                    // over time.
                    running_peak_avg =
                        (1.0f - PEAK_AVG_ALPHA) * running_peak_avg + PEAK_AVG_ALPHA * window_data.local_max;

                    dynamic_threshold = THRESHOLD_RATIO * running_peak_avg;

                    // Safety check to prevent threshold from getting too low.
                    // DTH lower than 1.5 gives DTH 2
                    if (dynamic_threshold < MIN_DYNAMIC_THRESHOLD) {
                        dynamic_threshold = MIN_DYNAMIC_THRESHOLD;
                    }

                } else if (beat_detected_too_fast) {
                    window_data.double_peaks++;
                    ESP_LOGI(TAG, "Double peak ignored at idx: %u", current_peak);
                }
            }
        }

        // FALL: BELOW → ABOVE (start new peak)
        else if ((signal_position == BELOW) && (sample > JITTER_THRESHOLD_ZERO_CROSSING)) {

            window_data.zero_crossings++;
            signal_position = ABOVE;

            // Start tracking peak
            window_data.local_max = sample;
            window_data.current_max_index = i;
        }

        // Save MAX
        if ((signal_position == ABOVE) && (sample > window_data.local_max)) {

            window_data.local_max = sample;
            window_data.current_max_index = i;
        }
    }

    return window_data;
}

static void calculate_heart_rate(heart_beat_stats_t *hb_data, signal_data_t *s_data, raw_data_t *history)
{

    uint16_t first_beat_idx = 0, last_beat_idx = 0;
    if (s_data->peaks_count >= MIN_PEAKS_FOR_HEARTRATE) {
        first_beat_idx = s_data->peaks_idx[0];
        last_beat_idx = s_data->peaks_idx[s_data->peaks_count - 1];

        int64_t t_first_beat = 0, t_last_beat = 0;
        int64_t duration_us = 0;
        float duration_sec = 0;
        t_first_beat = history[first_beat_idx].time_stamp;
        t_last_beat = history[last_beat_idx].time_stamp;

        duration_us = t_last_beat - t_first_beat;
        duration_sec = duration_us / 1000000.0f;

        uint16_t intervals = (s_data->peaks_count - 1);

        hb_data->avg_hr = (float)(intervals / duration_sec) * SECONDS_PER_MINUTE;
        ESP_LOGI(TAG, "First: %u Last: %u Duration: %.2f Heartbeat: %.1f", first_beat_idx, last_beat_idx,
                 duration_sec, hb_data->avg_hr);
    }
}

static heart_beat_stats_t calculate_rr_intervals(signal_data_t *data, raw_data_t history[],
                                                 uint16_t window_size)
{
    heart_beat_stats_t pulse_data = {0};
    float interval_ms_sum = 0;
    for (int i = 0; i < data->peaks_count; i++) {
        if (pulse_data.rr_intervals[i] != 0) {
            interval_ms_sum += pulse_data.rr_intervals[i];
        }
    }

    // Calculate RR/IBI ( ms)
    for (int i = 1; i < data->peaks_count; i++) {

        int64_t sample_a_idx = data->peaks_idx[i - 1];
        int64_t sample_b_idx = data->peaks_idx[i];

        int64_t time_diff_us = history[sample_b_idx].time_stamp - history[sample_a_idx].time_stamp;
        float interval_ms = time_diff_us / 1000.0f;
        if (interval_ms < MAX_VALID_RR_MS && interval_ms > MIN_VALID_RR_MS) {
            pulse_data.rr_intervals[i - 1] = interval_ms;
        } else {
            pulse_data.rr_intervals[i - 1] = 0;
        }

        if (debug_hr_features) {
            ESP_LOGI(TAG, "RR[%d]: %.2f ms", i - 1, interval_ms);
        }
    }

    return pulse_data;
}

static void calculate_sdnn(heart_beat_stats_t *hb_data, signal_data_t *s_data)
{
    int8_t intervals = s_data->peaks_count - 1;
    if (intervals < MIN_PEAKS_FOR_SDNN) {
        return;
    }
    // Sum up HR-intervals
    double sum = 0;
    uint8_t invalid_intervals = 0;
    for (int i = 0; i < intervals; i++) {
        if (hb_data->rr_intervals[i] == 0) {
            invalid_intervals++;
            continue;
        }
        sum += hb_data->rr_intervals[i];
    }

    float mean = 0;
    if (intervals > invalid_intervals) {

    } else {
        return;
    }
    mean = (float)sum / (intervals - invalid_intervals);

    hb_data->sdnn = 0;
    double sum_sq_diff = 0;
    for (int i = 0; i < intervals; i++) {
        if (hb_data->rr_intervals[i] != 0) {
            float dev = hb_data->rr_intervals[i] - mean;
            sum_sq_diff += (double)(dev * dev);
        }
    }
    hb_data->sdnn = sqrt((double)sum_sq_diff / intervals);
}

static void calculate_rmssd(heart_beat_stats_t *data)
{
    int32_t total_sum = 0.0f;
    for (int i = 0; i < data->num_of_diffs; i++) {
        total_sum += data->succ_diff[i];
    }

    data->diff_mean = (float)total_sum / data->num_of_diffs;
    data->rmssd = (float)sqrt(data->diff_mean);
    if (debug_hr_features) {
        ESP_LOGI(TAG, "Sum: %.2f Diffs: %u", total_sum, data->num_of_diffs);
        ESP_LOGI(TAG, "Diff-Mean: %.3f", data->diff_mean);
    }
}

static bool is_signal_data_valid(signal_data_t *data)
{
    if (data->peaks_count < 3) {
        ESP_LOGI(TAG, "Insufficient peaks found! Peaks: %u", data->peaks_count);
        return false;
    }

    uint16_t double_peaks = 0;
    for (int peak = 1; peak < data->peaks_count; peak++) {

        uint16_t inverval_samples = data->peaks_idx[peak] - data->peaks_idx[peak - 1];

        // Larger gap than 1,5 seconds
        if (inverval_samples > TOO_MUCH_TIME_BETWEEN_HEARTBEATS) {
            ESP_LOGI(TAG, "Too much time between peaks at index %u! Sample interval: %u",
                     data->peaks_idx[peak], inverval_samples);
            return false;
        } else if (inverval_samples < TOO_LITTLE_TIME_BETWEEN_HEARTBEATS) {
            ESP_LOGI(TAG, "Too short amount of time between peaks! Sample interval: %u", inverval_samples);
            return false;
        }
    }
    return true;
}

/*
Starting up
 - Wait until we have atleast MINIMUM_AMOUNT_OF_DATA before we extract features. Process partial array
 - When 30 seconds of data arrived window

Data quality
 - Calculate quality of signal. If signal is not good, skip feature extraction.
   Using statistical measurements

Time domain
Systolic peaks = Big heartbeats
Diastolic notch = small coupled with systolic

Zero-crossing state - Reset logic when crossing the 0 value.

Noise resilience
 Look at three points. Keep comparing the three to each other. When the middle is the highest, peak found.
 Noise can complicate the algorithm
 Create a threshold of the ampliture. The beat has to be above a certain level to be counted.
 Once a beat is detected, we blind the algorithm for a short duration to prevent double detections.
 For example the dicrotic notch.

 Search strategy
  - Find slope
  - Validation
    1. value high enough?
    2. enough time passed?
    3. is slope rise consistent with a heartbeat peak?
*/

/*
Signal Quality
    Amplitude bounds
     P - P < lowthreshold = No skin contact
     P - P > High = Too much skin pressure

    Zero-crossing Rate (Noise Detector)
     Normal:1-4 crossings
     High noise: 15+
     Counting how many times the signal crosses the 0 boundrary

    Standard deviation
     If current std is much higher than rest of window, the
     user most likely moved.

    Slope consistency
     If the change between two points is inhumanely large, flag
     the sample as invalid.

    Quality Map
     Keep a separate array that keeps track of the quality of
     the data in the 1 second window. A bad window can be
     excluded from the feature extraction.

*/
