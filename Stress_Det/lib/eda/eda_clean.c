// 4:th order Butterworth low-pass filter for EDA signal cleaning
// Same as NK
#include "eda_clean.h"

static int clean_initialized = 0;

// Biquad structure
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} biquad_t;

// Two cascaded biquads = 4th order Butterworth
static biquad_t s1, s2;

/**
 * @brief Processes a single input sample through a Biquad filter stage.
 *
 * This function applies a Second-Order Section (SOS) filter to the input 'x'
 * using the Direct Form II Transposed structure. This structure is particularly
 * effective for real-time sensor filtering on the ESP32-S3 as it balances
 * computational speed with low quantization noise.
 *
 * @param[in,out] s Pointer to the biquad_t structure containing coefficients
 *                  (b0, b1, b2, a1, a2) and state variables (z1, z2).
 * @param[in]     x The raw input sample (e.g., raw GSR voltage or PPG intensity).
 *
 * @return float The filtered output sample 'y'.
 *
 * @note To implement higher-order filters (e.g., 4th order Butterworth),
 *       cascade multiple calls to this function with different biquad_t stages.
 */
static float biquad_process(biquad_t *s, float x)
{
    float y = s->b0 * x + s->z1;
    s->z1 = s->b1 * x - s->a1 * y + s->z2;
    s->z2 = s->b2 * x - s->a2 * y;
    return y;
}

/**
 * @brief Initializes a 4th-order Butterworth low-pass filter for EDA signals.
 *
 * Configures two cascaded biquad stages to achieve a 3Hz cutoff frequency at
 * a 200Hz sampling rate. This filter is used to clean raw GSR data from the
 * CJMCU 6701, removing high-frequency noise while preserving skin conductance
 * responses.
 *
 * @param[in] first_sample The initial sensor reading. Used to prime the delay
 *                         lines to prevent startup transients.
 *
 * @note Coefficients are calculated for:
 *       - Type: Butterworth Low-Pass
 *       - Sample Rate: 200 Hz
 *       - Cutoff: 3 Hz
 *       - Structure: Cascaded Second-Order Sections (SOS)
 */
void eda_clean_init(float first_sample)
{
    // Butterworth Low-pass
    // Coefficients depend on cutoff frequency and sampling rate. For 3Hz cutoff and 200Hz sampling:
    // Stage 1
    s1.b0 = 0.0009447f;
    s1.b1 = 0.0018894f;
    s1.b2 = 0.0009447f;
    s1.a1 = -1.911197f;
    s1.a2 = 0.914976f;
    s1.z1 = 0.0f;
    s1.z2 = 0.0f;

    s1.z1 = first_sample;
    s1.z2 = first_sample;

    // Stage 2
    s2.b0 = 0.0009447f;
    s2.b1 = 0.0018894f;
    s2.b2 = 0.0009447f;
    s2.a1 = -1.822694f;
    s2.a2 = 0.837181f;
    s2.z1 = first_sample;
    s2.z2 = first_sample;
}

float eda_clean_process(float x)
{
    if (!clean_initialized) {
        eda_clean_init(x);
        clean_initialized = 1;
    }

    float y = biquad_process(&s1, x);
    y = biquad_process(&s2, y);
    return y;
}
