#include "ppg_processing.h"
#include "ppg_filter.h"
#include "ppg_hrv.h"
#include "ppg_peaks.h"
#include <stdio.h>

#define FS 200 //  MUST MATCH YOUR ACTUAL SAMPLING RATE
#define DEBUG_WINDOW_SEC 5

static int sample_index = 0;
static int step_counter = 0;
static int last_peak = -1000;
static int features_ready_flag = 0;

static float debug_filtered = 0;
static int debug_peak = 0;
static float debug_hr = 0;

// 🔹 Debug BPM via peak count
static int peak_count = 0;
static int window_start = 0;

static ppg_features_t features;

void ppg_processing_init()
{
    ppg_filter_init();
    ppg_peaks_init();
    ppg_hrv_init();

    sample_index = 0;
    step_counter = 0;
    last_peak = -1000;
    peak_count = 0;
    window_start = 0;
}

void ppg_process_sample(float raw)
{
    float normalized = raw * 0.0001f;
    // Filtrering
    float filtered = ppg_filter_process(normalized);

    filtered = filtered * 1000.0f; // För peak-detection och debug
    debug_filtered = filtered;

    sample_index++;
    step_counter++;

    // Peak detection
    int peak = ppg_detect_peak(filtered);
    debug_peak = peak ? (int)filtered : 0;

    if (peak) {

        int interval = sample_index - last_peak;

        printf("interval: %d samples\n", interval);

        // ignorera för täta peaks (< ~600 ms)
        if (interval < (FS * 0.4f)) {
            last_peak = sample_index;

            return;
        }

        if (last_peak > 0) {
            float rr = interval * 1000.0f / FS;

            printf("RR_raw: %f\n", rr);

            if (rr > 300 && rr < 2000) {

                peak_count++; // BPM debug counter

                printf(">RR:%f\n", rr);
                debug_hr = 60000.0f / rr;
                printf(">HR_debug:%f\n", debug_hr);

                float current_time = sample_index / (float)FS;
                ppg_add_rr(rr, current_time);
            } else {
                printf("RR rejected\n");
            }
        }

        last_peak = sample_index;
    }

    // 🔹 3. BPM debug (var 60 sek)

    if ((sample_index - window_start) >= (FS * DEBUG_WINDOW_SEC)) {

        int bpm = peak_count * (60 / DEBUG_WINDOW_SEC);

        printf("========== BPM DEBUG ==========\n");
        printf("Peaks last %ds: %d\n", DEBUG_WINDOW_SEC, peak_count);
        printf("\n>>> BPM DEBUG: %d <<<\n\n", bpm);
        printf("================================\n");

        peak_count = 0;
        window_start = sample_index;
    }

    // 🔹 4. HRV features (1 gång per sekund)
    if (step_counter >= FS) {
        step_counter = 0;

        float now = sample_index / (float)FS;

        features = ppg_compute_hrv(now);
        features_ready_flag = 1;

        float hr_smooth = ppg_compute_hr(now);
        debug_hr = hr_smooth;

        printf(">HR:%f\n", hr_smooth);
    }
}

int ppg_features_ready(void)
{
    if (features_ready_flag) {
        features_ready_flag = 0;
        return 1;
    }
    return 0;
}

ppg_features_t ppg_get_features()
{
    return features;
}

float ppg_get_filtered(void)
{
    return debug_filtered;
}

int ppg_get_peak(void)
{
    return debug_peak;
}

float ppg_get_hr(void)
{
    return debug_hr;
}
