#include "eda_clean.h"

// Biquad structure (Direct Form II)
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

void eda_clean_init(void)
{
    // Butterworth Low-pass
    // fs = 200 Hz, fc = 3 Hz, order = 4

    // Stage 1
    s1.b0 = 0.0009447f;
    s1.b1 = 0.0018894f;
    s1.b2 = 0.0009447f;
    s1.a1 = -1.911197f;
    s1.a2 = 0.914976f;
    s1.z1 = 0.0f;
    s1.z2 = 0.0f;

    // Stage 2
    s2.b0 = 0.0009447f;
    s2.b1 = 0.0018894f;
    s2.b2 = 0.0009447f;
    s2.a1 = -1.822694f;
    s2.a2 = 0.837181f;
    s2.z1 = 0.0f;
    s2.z2 = 0.0f;
}

float eda_clean_process(float x)
{
    float y = biquad_process(&s1, x);
    y = biquad_process(&s2, y);
    return y;
}