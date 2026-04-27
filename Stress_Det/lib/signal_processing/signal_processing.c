// Extracts derived features from PPG and GSR values, such as
// HRV and tonic, phasic values.

#include "signal_processing.h"
#include "board_config.h"
#include "eda_clean.h"
#include "eda_filter.h"
#include "eda_peaks.h"
#include "esp_log.h"
#include "float.h"
#include "ppg_hrv.h"
#include "ppg_peaks.h"
#include "types.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

bool debug_show_heartbeat_stats = true;
bool debug_show_first_last_peaks = false;
bool debug_show_latest_interval = true;
bool debug_show_rmssd_calculations = true;
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
#define JITTER_THRESHOLD_ZERO_CROSSING 5.0f
#define MAX_RR_INVERVAL 1500
#define MIN_RR_INVERVAL 300
#define MIN_PEAK_FOR_HEARTRATE 5
#define MAX_PEAK_FOR_HEARTRATE 600
#define MIN_PEAKS_FOR_SDNN 25
#define MIN_VALID_RR_MS 600
#define MAX_VALID_RR_MS 1300
#define BASELINE_RR_MS 800
#define MAX_DIFF_MS_HEARTBEATS 200

#define TWO_HUNDRED_FIFTY_MS_IN_SMPLS 50
#define THREE_HUNDRED_MS_IN_SMPLS 60
#define FIVE_HUNDRED_MS_IN_SMPLS 100
#define SECONDS_PER_MINUTE 60.0f

#define PEAK_AVG_ALPHA 0.5f
#define THRESHOLD_RATIO 0.5F
#define MIN_DYNAMIC_THRESHOLD 20.0f

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
    int64_t invalid_data_us;
    int64_t valid_data_us;
} peak_data_t;

typedef struct {
    float rr_intervals[MAX_PEAKS];
    uint8_t num_of_intervals;
    uint8_t num_of_diffs;
    float avg_hr;
    float rmssd;
} heart_beat_stats_t;

som_input_t calculate_features(raw_data_t history[], uint16_t window_size);
static peak_data_t peak_detector_with_motion_detection(raw_data_t history[], uint16_t window_size);
static void calculate_heart_rate(heart_beat_stats_t *hb_data, peak_data_t *s_data, raw_data_t *history);
static heart_beat_stats_t calculate_rr_intervals(peak_data_t *data, raw_data_t history[],
                                                 uint16_t window_size);
static void calculate_rmssd(heart_beat_stats_t *data);
static int64_t sum_invalid_data_us(raw_data_t history[], uint16_t window_size);
static float calculate_clean_data_percentage(peak_data_t *peak_data);
static float process_eda_signal(raw_data_t history[], uint16_t window_size);

som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    som_input_t features = {0};

    peak_data_t peak_data = peak_detector_with_motion_detection(history, window_size);
    heart_beat_stats_t heart_beat_data = calculate_rr_intervals(&peak_data, history, window_size);
    calculate_heart_rate(&heart_beat_data, &peak_data, history);
    float clean_data_percentage = calculate_clean_data_percentage(&peak_data);
    calculate_rmssd(&heart_beat_data);

    if (debug_show_heartbeat_stats)
        printf(">Peaks:%u\n>HR:%.0f\n>Clean Data Ratio:%.0f\n>rmssd:%.1f\n", peak_data.peaks_count,
               heart_beat_data.avg_hr, clean_data_percentage, heart_beat_data.rmssd);

    features.hr = heart_beat_data.avg_hr;
    features.hrv_rmssd = heart_beat_data.rmssd;

    features.scr = process_eda_signal(history, window_size);
    ESP_LOGI("SCR", "Count: %.f", features.scr);
    return features;
}

static float process_eda_signal(raw_data_t history[], uint16_t window_size)
{
    float scr_rate = 0.0f;
    for (int i = 1; i < window_size; i++) {

        // 2. Clean the signal (Butterworth)
        history[i].gsr_clean = eda_clean_process(history[i].gsr_scaled);
        // 3. Process components
        eda_filter_process(history[i].gsr_clean);
        float phasic = eda_get_phasic();

        // 4. Detect peaks/SCR rate
        eda_peaks_process(phasic);
        scr_rate = eda_get_scr_rate();

        if (i % 200 == 0) {
            ESP_LOGI("eda_sig", "gsr_scaled: %.4f gsr_clean: %.4f scr_rate: %.4f phasic: %.4f",
                     history[i].gsr_scaled, history[i].gsr_clean, scr_rate, phasic);
        }
    }

    return eda_get_scr_rate();
}

static float calculate_clean_data_percentage(peak_data_t *peak_data)
{
    float valid_time_ms = (float)peak_data->valid_data_us / 1000.0f;
    float invalid_time_ms = (float)peak_data->invalid_data_us / 1000.0f;
    float total_time_ms = valid_time_ms + invalid_time_ms;
    if (total_time_ms > 0) {
        // Ratio of usable signal (0.0 to 1.0)
        return (valid_time_ms / total_time_ms) * 100.0f;
    } else {
        return 0.0f;
    }
};

static peak_data_t peak_detector_with_motion_detection(raw_data_t history[], uint16_t window_size)
{
    enum SignalState signal_position;

    float running_peak_avg = 100.0f;
    float dynamic_threshold = 30.0f;

    // Init state
    if (history[0].ppg_filtered > 0) {
        signal_position = ABOVE;
    } else {

        signal_position = BELOW;
    }

    peak_data_t window_data = {0};
    window_data.local_max = -FLT_MAX;

    window_data.invalid_data_us = sum_invalid_data_us(history, window_size);

    for (int i = 1; i < window_size; i++) {

        float sample = history[i].ppg_filtered;
        bool motion_detected = history[i].has_movement_artifact;

        if (motion_detected) {
            window_data.local_max = -FLT_MAX;
        }

        // ABOVE → BELOW peak done.
        if ((signal_position == ABOVE) && (sample < -JITTER_THRESHOLD_ZERO_CROSSING) && !motion_detected) {

            window_data.zero_crossings++;
            signal_position = BELOW;

            uint16_t current_peak = window_data.current_max_index;

            // Check threshold, big enough to be a peak.
            if (window_data.local_max > dynamic_threshold && window_data.local_max < MAX_PEAK_FOR_HEARTRATE &&
                !motion_detected) {

                // timing (refractory period)
                bool beat_detected_too_fast =
                    (window_data.peaks_count > 0) &&
                    (current_peak - window_data.peaks_idx[window_data.peaks_count - 1] <
                     TWO_HUNDRED_FIFTY_MS_IN_SMPLS);

                // Peak found.
                if (!beat_detected_too_fast && window_data.peaks_count < MAX_PEAKS - 2) {
                    window_data.peaks_idx[window_data.peaks_count++] = current_peak;

                    // Exponential Moving Average (EMA).
                    // 0.9 of old value + 0.1 of new value (current peak) moving towards the new peak
                    // value
                    running_peak_avg =
                        (1.0f - PEAK_AVG_ALPHA) * running_peak_avg + PEAK_AVG_ALPHA * window_data.local_max;

                    // new DTH
                    dynamic_threshold = THRESHOLD_RATIO * running_peak_avg;

                    // Safety check to prevent threshold from getting too low.
                    // DTH lower than 1.5 gives DTH 2
                    if (dynamic_threshold < MIN_DYNAMIC_THRESHOLD) {
                        dynamic_threshold = MIN_DYNAMIC_THRESHOLD;
                    }
                    // check for double peaks.
                } else if (beat_detected_too_fast) {
                    ESP_LOGW(TAG, "Double peak ignored at idx: %u", current_peak);
                }
            }
        }

        // FALL: BELOW → ABOVE (start new peak)
        else if ((signal_position == BELOW) && (sample > JITTER_THRESHOLD_ZERO_CROSSING) &&
                 !motion_detected) {

            window_data.zero_crossings++;
            signal_position = ABOVE;

            // Start tracking peak
            window_data.local_max = sample;
            window_data.current_max_index = i;
        }

        // Save MAX
        if ((signal_position == ABOVE) && (sample > window_data.local_max) && !motion_detected) {

            window_data.local_max = sample;
            window_data.current_max_index = i;
        }
    }
    // all heartbeats(peaks) in this window
    return window_data;
}

static int64_t sum_invalid_data_us(raw_data_t history[], uint16_t window_size)
{
    int64_t total_invalid_us = 0;
    int64_t block_start_time = -1;

    for (int i = 0; i < window_size; i++) {
        // If we find an artifact and we aren't currently tracking a block
        if (history[i].has_movement_artifact && block_start_time == -1) {
            block_start_time = history[i].time_stamp;
        }
        // If the artifact ends OR we hit the end of the array while in a block
        else if (!history[i].has_movement_artifact && block_start_time != -1) {
            total_invalid_us += (history[i - 1].time_stamp - block_start_time);
            block_start_time = -1; // Reset for next block
        }
    }

    // Handle case where the window ends while still in an artifact block
    if (block_start_time != -1) {
        total_invalid_us += (history[window_size - 1].time_stamp - block_start_time);
    }

    return total_invalid_us;
}

static void calculate_heart_rate(heart_beat_stats_t *hb_data, peak_data_t *s_data, raw_data_t *history)
{
    if (s_data->peaks_count < 10) {
        hb_data->avg_hr = 0.0f;
        return;
    }

    uint16_t first_beat_idx = 0, last_beat_idx = 0;
    if (s_data->peaks_count >= MIN_PEAK_FOR_HEARTRATE) {
        first_beat_idx = s_data->peaks_idx[0];
        last_beat_idx = s_data->peaks_idx[s_data->peaks_count - 1];

        int64_t t_first_beat = 0, t_last_beat = 0;
        t_first_beat = history[first_beat_idx].time_stamp;
        t_last_beat = history[last_beat_idx].time_stamp;

        int64_t duration_us = t_last_beat - t_first_beat;
        int64_t valid_duration_us = duration_us - s_data->invalid_data_us;
        if (valid_duration_us < 0) {
            valid_duration_us = 0;
        }
        s_data->valid_data_us = valid_duration_us;

        float valid_duration_sec = valid_duration_us / 1000000.0f;
        uint16_t intervals = (s_data->peaks_count - 1);
        if (valid_duration_sec <= 0) {
            hb_data->avg_hr = 0;
            ESP_LOGW(TAG, "calculate_heart_rate - Invalid Duration");
            return;
        }

        float beats_per_sec = (float)(intervals / valid_duration_sec);
        float heartrate = (float)beats_per_sec * SECONDS_PER_MINUTE;
        hb_data->avg_hr = heartrate;

        if (debug_show_first_last_peaks)
            ESP_LOGI(TAG, "First: %u Last: %u Valid Duration: %.2f Heartbeat: %.1f", first_beat_idx,
                     last_beat_idx, valid_duration_sec, hb_data->avg_hr);
    }
}

static heart_beat_stats_t calculate_rr_intervals(peak_data_t *data, raw_data_t history[],
                                                 uint16_t window_size)
{
    heart_beat_stats_t pulse_data = {0};
    if (data->peaks_count < 3) {
        return pulse_data;
    }
    uint16_t valid_intervals = 0;
    // Calculate RR/IBI (ms)
    for (int i = 1; i < data->peaks_count; i++) {

        int64_t sample_a_idx = data->peaks_idx[i - 1];
        int64_t sample_b_idx = data->peaks_idx[i];

        int64_t time_diff_us = history[sample_b_idx].time_stamp - history[sample_a_idx].time_stamp;
        float interval_ms = time_diff_us / 1000.0f;
        if (interval_ms < MAX_VALID_RR_MS && interval_ms > MIN_VALID_RR_MS) {
            pulse_data.rr_intervals[valid_intervals++] = interval_ms;
        }
    }
    pulse_data.num_of_intervals = valid_intervals;
    if (debug_show_latest_interval) {
        ESP_LOGI(TAG, "Latest Interval: %.1f", pulse_data.rr_intervals[valid_intervals - 1]);
    }

    return pulse_data;
}

static void calculate_rmssd(heart_beat_stats_t *data)
{
    if (data->num_of_intervals < 2) {
        data->rmssd = 0.0f;
        data->num_of_diffs = 0;
        return;
    }

    double sum_squared_diffs = 0.0;
    uint16_t valid_diff_count = 0;

    // Use the actual count of valid intervals identified earlier
    for (int i = 0; i < (data->num_of_intervals - 1); i++) {
        float current_rr = data->rr_intervals[i];
        float next_rr = data->rr_intervals[i + 1];

        float diff = next_rr - current_rr;
        if (fabsf(diff) < MAX_DIFF_MS_HEARTBEATS) {
            sum_squared_diffs += (double)(diff * diff);

            valid_diff_count++;
        }
    }

    data->num_of_diffs = valid_diff_count;

    if (valid_diff_count > 0) {
        // The core RMSSD formula: sqrt( mean( differences^2 ) )
        double mean_square = sum_squared_diffs / (double)valid_diff_count;
        if (debug_show_rmssd_calculations) {
            ESP_LOGI("RMSSD", "SumsqDiffs: %.f Validdiffcount: %u MeanSq: %.f RMSSD: %.f", sum_squared_diffs,
                     valid_diff_count, mean_square, (float)sqrt(mean_square));
        }
        data->rmssd = (float)sqrt(mean_square);
    } else {
        data->rmssd = 0.0f;
    }
}
