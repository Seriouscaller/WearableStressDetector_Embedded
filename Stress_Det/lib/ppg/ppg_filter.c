#include "ppg_filter.h"

typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float x1, x2;
    float y1, y2;
} biquad_t;

static biquad_t hp, lp;

/**
 * @brief  Initializes the Biquad filter coefficients for PPG signal conditioning.
 *
 * Sets up a dual-stage filter architecture:
 * 1. **High-Pass (0.5 Hz)**: Removes DC offset and baseline wander caused by
 *    breathing or slow vasomotor activity.
 * 2. **Low-Pass (8.0 Hz)**: Removes 50/60Hz power line interference and
 *    high-frequency motion jitter from the green LED.
 *
 * @note Sampling Frequency (Fs) is assumed to be 200 Hz.
 */
void ppg_filter_init()
{
    // --- Stage 1: High-pass 0.5 Hz (Butterworth 2nd Order) ---
    // This removes the "75k offset" and centers the signal around zero.
    hp.b0 = 0.9845f;
    hp.b1 = -1.9690f;
    hp.b2 = 0.9845f;

    hp.a1 = -1.9689f;
    hp.a2 = 0.9691f;

    hp.x1 = hp.x2 = 0.0f;
    hp.y1 = hp.y2 = 0.0f;

    // --- Stage 2: Low-pass 8.0 Hz (Butterworth 2nd Order) ---
    // Selected for a sharp cutoff to isolate the primary systolic peak.
    lp.b0 = 0.0286f;
    lp.b1 = 0.0572f;
    lp.b2 = 0.0286f;
    lp.a1 = -1.4542f;
    lp.a2 = 0.5686f;

    lp.x1 = lp.x2 = 0.0f;
    lp.y1 = lp.y2 = 0.0f;
}

/**
 * @brief  Processes a single sample through a second-order Biquad IIR filter.
 *
 * This function implements the standard Direct Form I difference equation:
 * $$y[n] = b_0x[n] + b_1x[n-1] + b_2x[n-2] - a_1y[n-1] - a_2y[n-2]$$
 *
 * @param[in,out] f Pointer to the biquad_t struct containing coefficients and history.
 * @param[in]     x The current raw input sample.
 *
 * @return float  The filtered output sample.
 */
float biquad_process(biquad_t *f, float x)
{
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;

    f->x2 = f->x1;
    f->x1 = x;

    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

/**
 * @brief  Applies a complete band-pass filter to a PPG sample.
 *
 * This function chains two second-order Biquad filters in series.
 * Stage 1 (HP): 0.5 Hz Cutoff - Removes DC offset and breathing artifacts.
 * Stage 2 (LP): 8.0 Hz Cutoff - Smooths the signal for peak detection.
 *
 * @param[in] x The raw (or pre-conditioned) PPG sample.
 *
 * @return float The fully filtered AC signal, centered at 0.0.
 */
float ppg_filter_process(float x)
{
    float y;

    // High-Pass Filter: Remove the "75k" baseline and slow wander.
    // Input: Raw Sample -> Output: Centered AC Signal
    y = biquad_process(&hp, x);

    // Low-Pass Filter: Remove jitter and 50/60Hz noise.
    // Input: Centered AC -> Output: Clean Pulse Wave
    y = biquad_process(&lp, y);

    return y;
}