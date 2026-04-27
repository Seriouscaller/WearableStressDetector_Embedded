#include "eda_peaks.h"

#define WINDOW_SEC 60.0f
#define REFRACTORY_SEC 1.0f

static float fs = 200.0f;

// Peak detection state
static float prev = 0.0f;
static float curr = 0.0f;
static float next = 0.0f;

static int initialized = 0;

// Threshold (can tune later)
static float threshold = 0.003f;

// Refractory
static int refractory_samples = 0;
static int refractory_limit = 0;

// SC_RR
static int scr_count = 0;
static int window_samples = 0;
static int window_size = 0;

// Init
void eda_peaks_init(float sampling_rate)
{
    fs = sampling_rate;

    refractory_limit = (int)(REFRACTORY_SEC * fs);
    window_size = (int)(WINDOW_SEC * fs);

    prev = curr = next = 0.0f;
    initialized = 0;

    refractory_samples = refractory_limit;
    scr_count = 0;
    window_samples = 0;
}

// Process one sample
void eda_peaks_process(float phasic)
{
    // shift buffer
    prev = curr;
    curr = next;
    next = phasic;

    if (!initialized) {
        initialized = 1;
        return;
    }

    // refractory countdown
    if (refractory_samples < refractory_limit)
        refractory_samples++;

    // peak detection
    if (curr > prev && curr > next && curr > threshold && refractory_samples >= refractory_limit) {
        scr_count++;
        refractory_samples = 0;
    }

    // window tracking
    window_samples++;
    if (window_samples >= window_size) {
        window_samples = 0;
        scr_count = 0;
    }
}

// SC_RR (peaks per second) same feature as SOM model uses for classification
float eda_get_scr_rate(void)
{
    return (float)scr_count / WINDOW_SEC;
}

int eda_get_scr_count(void)
{
    return scr_count;
}