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

// Process one biquad
static float biquad_process(biquad_t *s, float x)
{
    float y = s->b0 * x + s->z1;
    s->z1 = s->b1 * x - s->a1 * y + s->z2;
    s->z2 = s->b2 * x - s->a2 * y;
    return y;
}

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
