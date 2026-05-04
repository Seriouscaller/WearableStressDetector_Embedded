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

/**
 * @brief  Initializes the EDA peak detection and feature extraction variables.
 *
 * Sets up the timing constraints for peak detection, including the refractory
 * period (to prevent double-counting a single physiological response) and
 * windowing for power calculations. It resets the SCR (Skin Conductance Response)
 * counters and the history buffers used for slope detection.
 *
 * @param[in] sampling_rate The frequency at which the GSR sensor is polled (Hz).
 *
 * @note The constants 'REFRACTORY_SEC' and 'WINDOW_SEC' must be defined in your
 *       headers to set the biological and statistical timing limits.
 */
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

/**
 * @brief  Analyzes the phasic EDA signal to detect peaks and calculate metrics.
 *
 * This function performs three main tasks:
 * 1. **Peak Detection**: Uses a 4-point sliding window to find local maxima where
 *    the signal slope changes from rising to falling.
 * 2. **Refractory Logic**: Implements a lockout period after a peak is detected
 *    to align with biological sweat gland recovery times.
 * 3. **Feature Integration**: Calculates the average phasic power and the peak
 *    frequency (Response Rate) over a fixed time window.
 *
 * @param[in] phasic The current filtered and tonic-removed phasic sample.
 *
 * @note Metrics are printed to the console in a CSV-ready format for easy
 *       telemetry visualization.
 */
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

/**
 * @brief  Retrieves the current Skin Conductance Response Rate (SC-RR).
 *
 * This metric is calculated in 'eda_peaks_process' as the number of peaks
 * detected divided by the window duration. It provides a measure of
 * sympathetic nervous system activity frequency.
 *
 * @return float The response rate (peaks per unit of time).
 *
 * @note This value is updated only at the end of each sliding window
 *       (e.g., every 30 or 60 seconds).
 */
float eda_get_scr_rate(void)
{
    return sc_rr;
}

/**
 * @brief  Retrieves the total count of detected Skin Conductance Responses (SCR).
 *
 * Returns the current tally of peaks identified by the 'eda_peaks_process'
 * algorithm. This count is reset every time the 'window_samples' reaches
 * 'window_size', allowing for periodic density analysis of stress responses.
 *
 * @return int The number of valid Phasic peaks detected in the current window.
 *
 * @note In biofeedback applications, a high SCR count over a 60-second
 *       window typically correlates with high cognitive load or emotional stress.
 */
int eda_get_scr_count(void)
{
    return scr_count;
}

/**
 * @brief  Retrieves the Phasic Power (SC-Ph) of the EDA signal.
 *
 * Returns the mean squared value of the phasic component calculated in
 * 'eda_peaks_process'. This metric captures the overall intensity of
 * physiological arousal, accounting for both the frequency and the
 * amplitude of sweat gland activity.
 *
 * @return float The average phasic power (scaled units).
 *
 * @note This is a windowed metric. It remains constant throughout a
 *       window and only updates once the 'window_samples' threshold is met.
 */
float eda_get_sc_ph(void)
{
    return sc_ph;
}