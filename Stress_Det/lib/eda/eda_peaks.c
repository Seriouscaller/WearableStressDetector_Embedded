#include "eda_peaks.h"
#include "esp_timer.h"
#include <stdio.h>

#define WINDOW_SEC 60.0f
#define REFRACTORY_SEC 1.0f

static float fs = 200.0f;

// Peak detection state
static float prev = 0.0f;
static float curr = 0.0f;
static float next = 0.0f;
static float prev_prev = 0.0f;

static int initialized = 0;

// SC_PH
static float phasic_power_sum = 0.0f;
static int power_samples = 0;
static float sc_ph = 0.0f;

// Threshold (can tune later)
static float threshold = 0.01f;

// Refractory
static int refractory_samples = 0;
static int refractory_limit = 0;

// SC_RR
static int scr_count = 0;
static float sc_rr = 0.0f;
static int window_samples = 0;
static int window_size = 0;

// Init
void eda_peaks_init(float sampling_rate)
{
    fs = sampling_rate;

    refractory_limit = (int)(REFRACTORY_SEC * fs);
    window_size = (int)(WINDOW_SEC * fs);

    prev = curr = next = 0.0f;
    initialized = 0;

    refractory_samples = refractory_limit;
    scr_count = 0;
    window_samples = 0;

    phasic_power_sum = 0.0f;
    power_samples = 0;
    sc_ph = 0.0f;
    sc_rr = 0.0f;
    prev_prev = 0.0f;
}

// Process one sample
void eda_peaks_process(float phasic)
{
    // shift buffer
    prev_prev = prev;
    prev = curr;
    curr = next;
    next = phasic;

    if (!initialized) {
        initialized = 1;
        return;
    }

    float p = phasic * 10.0f;

    // Clipping
    float cap = 0.1f;
    if (p > cap)
        p = cap;

    phasic_power_sum += p * p;
    power_samples++;

    // refractory countdown
    if (refractory_samples < refractory_limit)
        refractory_samples++;

    // peak detection
    float diff = curr - prev;
    float prev_diff = prev - prev_prev;

    int peak_shape = (prev_diff > 0.0f && diff < 0.0f);
    int strong_enough = (curr > threshold);

    if (peak_shape && strong_enough && refractory_samples >= refractory_limit) {
        scr_count++;
        refractory_samples = 0;

        // DEBUG peak
        // printf("PEAK: %.4f\n", curr);
    }

    // window tracking
    window_samples++;
    if (window_samples >= window_size) {

        if (power_samples > 0)
            sc_ph = phasic_power_sum / power_samples;

        sc_rr = (float)scr_count / WINDOW_SEC;

        // DEBUG PRINT
        printf("DATA:%lld,%.6f,%.4f,%d\r\n", esp_timer_get_time() / 1000, sc_ph, sc_rr, scr_count);

        // reset
        window_samples = 0;
        scr_count = 0;
        phasic_power_sum = 0.0f;
        power_samples = 0;
    }
}

// SC_RR (peaks per second) same feature as SOM model uses for classification
float eda_get_scr_rate(void)
{
    return sc_rr;
}

int eda_get_scr_count(void)
{
    return scr_count;
}

// SC_PH (phasically filtered power) same feature as SOM model uses for classification
float eda_get_sc_ph(void)
{
    return sc_ph;
}