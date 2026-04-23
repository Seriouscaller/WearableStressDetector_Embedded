#include "eda_peaks.h"
#include <math.h>

#define AMP_THRESHOLD 0.01f
#define REFRACTORY_SEC 1.0f

typedef enum { SCR_IDLE, SCR_RISING } scr_state_t;

static scr_state_t state;
static float onset_value;
static float peak_value;
static float last_peak_time;

void eda_peaks_init()
{
    state = SCR_IDLE;
    onset_value = 0;
    peak_value = 0;
    last_peak_time = -10.0f;
}

int eda_detect_scr(float phasic, float time)
{
    static float prev = 0;
    int scr = 0;

    float diff = phasic - prev;

    switch (state) {
    case SCR_IDLE:
        // Detect start of rise
        if (diff > 0.001f) {
            state = SCR_RISING;
            onset_value = phasic;
            peak_value = phasic;
        }
        break;

    case SCR_RISING:
        // Track peak
        if (phasic > peak_value) {
            peak_value = phasic;
        }

        // Peak reached (slope goes negative)
        if (diff < 0) {
            float amplitude = fabsf(peak_value - onset_value);

            int valid_amp = (amplitude > AMP_THRESHOLD);
            int valid_time = ((time - last_peak_time) > REFRACTORY_SEC);

            if (valid_amp && valid_time) {
                scr = 1;
                last_peak_time = time;
            }

            state = SCR_IDLE;
        }
        break;
    }

    prev = phasic;
    return scr;
}
