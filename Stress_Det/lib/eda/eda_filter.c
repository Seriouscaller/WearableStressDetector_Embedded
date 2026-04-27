#include "eda_filter.h"
#include <math.h>

#define FS 200.0f

// values depends on cutoff frequency and sampling rate. For 0.05Hz cutoff and 200Hz sampling:
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
    // Initialize tonic with the first sample
    if (!tonic_initialized) {
        tonic = x;
        tonic_initialized = 1;
    }
    // Update tonic with a slow moving average
    tonic += 0.00005f * (x - tonic);

    // Center the signal by removing the tonic component
    // find variation around the tonic level, which is the phasic component
    float centered = x - tonic;

    // high-pass filter to extract phasic component
    float y = b0 * centered + z1;
    z1 = b1 * centered - a1 * y + z2;
    z2 = b2 * centered - a2 * y;

    phasic = y;

    // simple low-pass to smooth the phasic component.

    phasic_smooth += 0.1f * (phasic - phasic_smooth);
    phasic = phasic_smooth;

    // take absolute value, positive deflection in GSR is what matters.
    if (phasic < 0.0f)
        phasic = 0.0f;
}

// Getter
float eda_get_phasic(void)
{
    return phasic;
}

float eda_get_tonic(void)
{
    return tonic;
}
