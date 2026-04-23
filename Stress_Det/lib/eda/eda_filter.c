#include "eda_filter.h"
#include <math.h>

#define FS 200.0f

static float b0 = 0.99843f;
static float b1 = -1.99686f;
static float b2 = 0.99843f;
static float a1 = -1.99685f;
static float a2 = 0.99687f;

static float z1 = 0.0f, z2 = 0.0f;
static float mean = 0.0f;
static float phasic = 0.0f;

// Init
void eda_filter_init(void)
{
    z1 = 0;
    z2 = 0;
    mean = 0;
    phasic = 0;
}

// Process
void eda_filter_process(float x)
{
    // ===== CENTERING =====
    mean += 0.01f * (x - mean);
    float centered = x - mean;

    // ===== BIQUAD HPF =====
    float y = b0 * centered + z1;
    z1 = b1 * centered - a1 * y + z2;
    z2 = b2 * centered - a2 * y;

    phasic = y;

    // ===== CLAMP =====
    if (phasic < 0.0f)
        phasic = 0.0f;
}

// Getter
float eda_get_phasic(void)
{
    return phasic;
}

/*#include "eda_filter.h"
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
}*/