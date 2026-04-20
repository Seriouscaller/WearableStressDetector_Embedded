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

bool debug_show_heartbeat_stats = true;
bool debug_show_first_last_peaks = false;
static const char *TAG = "S_PR";

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
#define MIN_PEAK_FOR_HEARTRATE 5
#define MAX_PEAK_FOR_HEARTRATE 600
#define MIN_PEAKS_FOR_SDNN 25
#define MIN_VALID_RR_MS 600
#define MAX_VALID_RR_MS 1300
#define MAX_DIFF_MS_HEARTBEATS 400

#define TWO_HUNDRED_FIFTY_MS_IN_SMPLS 50
#define THREE_HUNDRED_MS_IN_SMPLS 60
#define FIVE_HUNDRED_MS_IN_SMPLS 100
#define SECONDS_PER_MINUTE 60.0f

#define PEAK_AVG_ALPHA 0.1f
#define THRESHOLD_RATIO 0.5F
#define MIN_DYNAMIC_THRESHOLD 1.5f

#define PEAK_WINDOW 5

enum SignalState {
    ABOVE = 1,
    BELOW = -1,
};

typedef struct {
    float local_max;
    float current_max_val;
    uint16_t current_max_index;
    uint16_t peaks_count;
    uint16_t peaks_idx[MAX_PEAKS];
    uint16_t zero_crossings;
    uint16_t double_peaks;
} peak_data_t;

typedef struct {
    float rr_intervals[MAX_PEAKS];
    int32_t succ_diff[MAX_PEAKS];
    uint8_t num_of_intervals;
    uint8_t num_of_diffs;
    float diff_mean;
    float avg_hr;
    float rmssd;
    float sdnn;
} heart_beat_stats_t;

som_input_t calculate_features(raw_data_t history[], uint16_t window_size);
static void calculate_heart_rate(heart_beat_stats_t *hb_data, peak_data_t *s_data, raw_data_t *history);
static void calculate_rmssd(heart_beat_stats_t *data);
static heart_beat_stats_t calculate_rr_intervals(peak_data_t *data, raw_data_t history[],
                                                 uint16_t window_size);
static peak_data_t peak_detector(raw_data_t history[], uint16_t window_size);
static void calculate_sdnn(heart_beat_stats_t *data, peak_data_t *s_data);

som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    som_input_t features = {0};

    peak_data_t peak_data = peak_detector(history, window_size);
    heart_beat_stats_t heart_beat_data = calculate_rr_intervals(&peak_data, history, window_size);
    calculate_heart_rate(&heart_beat_data, &peak_data, history);
    calculate_sdnn(&heart_beat_data, &peak_data);
    calculate_rmssd(&heart_beat_data);

    if (debug_show_heartbeat_stats)
        printf(">Peaks:%u\n>HR:%.0f\n>SDNN:%.0f\n>RMSSD:%.0f\n", peak_data.peaks_count,
               heart_beat_data.avg_hr, heart_beat_data.sdnn, heart_beat_data.rmssd);

    features.hr = heart_beat_data.avg_hr;
    features.hrv_sdnn = heart_beat_data.sdnn;
    features.hrv_rmssd = heart_beat_data.rmssd;

    return features;
}

static peak_data_t peak_detector(raw_data_t history[], uint16_t window_size)
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

    peak_data_t window_data = {0};
    window_data.local_max = -FLT_MAX;

    for (int i = 1; i < window_size; i++) {

        float sample = history[i].ppg_filtered;

        // ABOVE → BELOW peak done.
        if ((signal_position == ABOVE) && (sample < -JITTER_THRESHOLD_ZERO_CROSSING)) {

            window_data.zero_crossings++;
            signal_position = BELOW;

            uint16_t current_peak = window_data.current_max_index;

            // Check threshold, big enough to be a peak.
            if (window_data.local_max > dynamic_threshold && window_data.local_max < MAX_PEAK_FOR_HEARTRATE) {

                // 2. timing (refractory period)
                bool beat_detected_too_fast =
                    (window_data.peaks_count > 0) &&
                    (current_peak - window_data.peaks_idx[window_data.peaks_count - 1] <
                     TWO_HUNDRED_FIFTY_MS_IN_SMPLS);

                // 3. Peak found.
                if (!beat_detected_too_fast && window_data.peaks_count < MAX_PEAKS - 2) {
                    window_data.peaks_idx[window_data.peaks_count++] = current_peak;

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

static void calculate_heart_rate(heart_beat_stats_t *hb_data, peak_data_t *s_data, raw_data_t *history)
{

    uint16_t first_beat_idx = 0, last_beat_idx = 0;
    if (s_data->peaks_count >= MIN_PEAK_FOR_HEARTRATE) {
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
        if (debug_show_first_last_peaks)
            ESP_LOGI(TAG, "First: %u Last: %u Duration: %.2f Heartbeat: %.1f", first_beat_idx, last_beat_idx,
                     duration_sec, hb_data->avg_hr);
    }
}

static heart_beat_stats_t calculate_rr_intervals(peak_data_t *data, raw_data_t history[],
                                                 uint16_t window_size)
{
    heart_beat_stats_t pulse_data = {0};

    // Calculate RR/IBI ( ms)
    for (int i = 1; i < data->peaks_count; i++) {

        int64_t sample_a_idx = data->peaks_idx[i - 1];
        int64_t sample_b_idx = data->peaks_idx[i];

        int64_t time_diff_us = history[sample_b_idx].time_stamp - history[sample_a_idx].time_stamp;
        float interval_ms = time_diff_us / 1000.0f;
        if (interval_ms < MAX_VALID_RR_MS && interval_ms > MIN_VALID_RR_MS) {
            pulse_data.rr_intervals[i - 1] = interval_ms;
        } else {
            pulse_data.rr_intervals[i - 1] = 950;
        }
    }

    return pulse_data;
}

static void calculate_sdnn(heart_beat_stats_t *hb_data, peak_data_t *s_data)
{
    int8_t intervals = s_data->peaks_count - 1;
    if (intervals < MIN_PEAKS_FOR_SDNN) {
        hb_data->sdnn = 0.0f;
        return;
    }

    // Sum up HR-intervals
    double sum = 0;
    uint8_t valid_count = 0;
    for (int i = 0; i < intervals; i++) {
        if (hb_data->rr_intervals[i] > 0) {
            sum += hb_data->rr_intervals[i];
            valid_count++;
        }
    }
    hb_data->num_of_intervals = valid_count;

    if (valid_count < 2) {
        hb_data->sdnn = 0.0f;
        return;
    }

    float mean = (float)(sum / (intervals - 1));

    hb_data->sdnn = 0;
    double sum_sq_diff = 0;
    for (int i = 0; i < intervals; i++) {
        if (hb_data->rr_intervals[i] > 0) {
            float dev = hb_data->rr_intervals[i] - mean;
            sum_sq_diff += (double)(dev * dev);
        }
    }
    hb_data->sdnn = sqrtf((float)sum_sq_diff / valid_count);
}

static void calculate_rmssd(heart_beat_stats_t *data)
{
    if (data->num_of_intervals < 1) {
        data->rmssd = 0.0f;
        return;
    }

    for (int i = 0; i < MAX_PEAKS - 1; i++) {
        float current_rr = data->rr_intervals[i];
        float next_rr = data->rr_intervals[i + 1];

        // Only calculate if BOTH intervals are non-zero/valid
        if (current_rr > 0 && next_rr > 0) {
            float diff = next_rr - current_rr;
            if (fabsf(diff) < MAX_DIFF_MS_HEARTBEATS) {
                data->succ_diff[data->num_of_diffs] = (int32_t)diff;
                data->num_of_diffs++;
            }
        }
    }

    double sum_squared_diffs = 0.0;
    uint16_t actual_diffs = 0;
    for (int i = 0; i < data->num_of_intervals; i++) {
        float diff = data->succ_diff[i];
        sum_squared_diffs += (double)(diff * diff);
        actual_diffs++;
    }

    if (actual_diffs > 0) {
        float mean_square = (float)(sum_squared_diffs / actual_diffs);
        data->rmssd = sqrtf(mean_square);
    } else {
        data->rmssd = 0.0f;
    }
}
