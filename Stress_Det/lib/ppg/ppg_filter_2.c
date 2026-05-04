#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    float dc_est;
    float lpf_out;
    bool is_initialized;
} ppg_state_t;

static ppg_state_t ppg_state = {0.0f, 0.0f, false};

/**
 * @brief  Processes a raw PPG sample to isolate the pulsatile (AC) component.
 *
 * This pipeline performs three critical transformations:
 * 1. **DC Tracking**: Uses a slow EMA to estimate the static tissue reflection.
 * 2. **Jitter Filtering**: A 0.30 alpha EMA removes high-frequency I2C/optical noise.
 * 3. **Inversion**: Since blood volume increases (absorbing more light) during
 *    systole, the raw signal dips. We invert this so peaks represent heart beats.
 *
 * @param[in] raw_in The 18-bit raw intensity from the MAX30101 FIFO.
 *
 * @return float The cleaned, centered, and inverted AC signal for peak detection.
 */
float ppg_process_sample(uint32_t raw_in)
{
    // 1. Convert to float immediately [cite: 637]
    float x = (float)raw_in;

    // 2. Initialize DC estimate to the current baseline [cite: 523, 616]
    if (!ppg_state.is_initialized) {
        ppg_state.dc_est = x;
        ppg_state.is_initialized = true;
    }

    // 3. Track the DC baseline (the 75k offset)
    ppg_state.dc_est = (0.995f * ppg_state.dc_est) + (0.005f * x);

    // 4. Subtract DC to get AC signal (centered at 0)
    float ac_signal = x - ppg_state.dc_est;

    // 5. Apply Jitter Filter (0.30f for sharpness)
    float alpha = 0.30f;
    ppg_state.lpf_out = (alpha * ac_signal) + ((1.0f - alpha) * ppg_state.lpf_out);

    // 6. HARD INVERSION: Flip the signal so valleys point UP
    // We also add a small offset (e.g., 500) so the plotter definitely sees it as positive
    return (ppg_state.lpf_out * -1.0f) + 500.0f;
}

/**
 * @brief  Resets the initialization flag of the PPG signal processor.
 *
 * Calling this function forces the next sample passed to 'ppg_process_sample'
 * to be treated as a new baseline. This is critical for clearing out 'stale'
 * DC estimates after the sensor has been moved or the 5V boost pump has
 * been power-cycled.
 */
void ppg_filter_reset()
{
    ppg_state.is_initialized = false;
}