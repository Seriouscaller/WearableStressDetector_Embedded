#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    float dc_est;
    float lpf_out;
    bool is_initialized;
} ppg_state_t;

static ppg_state_t ppg_state = {0.0f, 0.0f, false};

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

void ppg_filter_reset()
{
    ppg_state.is_initialized = false;
}