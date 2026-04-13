#include "eda_filter.h"
#include <math.h>

#define FS 200.0f
#define FC 0.05f // cutoff for tonic (slow component)

// Outputs
static float tonic = 0.0f;
static float phasic = 0.0f;

// Internal state
static float lp = 0.0f;

void eda_filter_init(void)
{
    tonic = 0.0f;
    phasic = 0.0f;
    lp = 0.0f;
}

void eda_filter_process(float x)
{
    float dt = 1.0f / FS;
    float RC = 1.0f / (2.0f * M_PI * FC);

    float alpha = dt / (RC + dt);

    // Low-pass → tonic
    lp = lp + alpha * (x - lp);

    // High-pass → phasic
    tonic = lp;
    phasic = x - lp;
}

float eda_get_tonic(void)
{
    return tonic;
}

float eda_get_phasic(void)
{
    return phasic;
}