#include "eda_filter.h"
#include <math.h>

#define FS 200.0f

static float b0 = 0.9995066f;
static float b1 = -1.9990132f;
static float b2 = 0.9995066f;
static float a1 = -1.9990129f;
static float a2 = 0.9990135f;

static float z1 = 0.0f, z2 = 0.0f;
static float mean = 0.0f;
static float phasic = 0.0f;
static float tonic = 0.0f;
static float phasic_smooth = 0.0f;
static int tonic_initialized = 0;

// Init
void eda_filter_init(void)
{
    z1 = 0;
    z2 = 0;
    mean = 0;
    phasic = 0;
    tonic = 0;
    phasic_smooth = 0;
    tonic_initialized = 0;
}

// Process
void eda_filter_process(float x)
{
    if (!tonic_initialized) {
        tonic = x; // Initialize tonic with the first sample
        tonic_initialized = 1;
    }

    tonic += 0.00005f * (x - tonic); // Simple IIR for tonic estimation
    float centered = x - tonic;

    // ===== BIQUAD HPF =====
    float y = b0 * centered + z1;
    z1 = b1 * centered - a1 * y + z2;
    z2 = b2 * centered - a2 * y;

    phasic = y;

    phasic_smooth += 0.1f * (phasic - phasic_smooth); // Smooth the phasic component
    phasic = phasic_smooth;

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