#include "ppg_peaks.h"
#include <math.h>

#define FS 200
#define REFRACTORY_SEC 0.4f // 400 ms

// --- Intern state ---
static float prev1 = 0;
static float prev2 = 0;

static float smooth = 0;
static float env = 0;

static int refractory = 0;

void ppg_peaks_init()
{
    prev1 = prev2 = 0;
    smooth = 0;
    env = 0;
    refractory = 0;
}

int ppg_detect_peak(float x)
{
    int peak = 0;

    // 🔹 1. Smooth signal (minskar brus)
    smooth = 0.8f * smooth + 0.2f * x;

    // 🔹 2. Envelope (amplitud)
    env = 0.95f * env + 0.05f * fabsf(smooth);

    // 🔹 3. Adaptiv threshold
    float threshold = env * 0.3f;

    // 🔹 4. Refractory countdown
    if (refractory > 0)
        refractory--;

    // 🔹 5. Peak detection (lokal max + slope + threshold)
    if (prev1 > prev2 &&    
        prev1 > smooth &&      // lokal max (vänster)
        prev1 > threshold &&      // över threshold
        (prev1 - prev2) > 0.0001f )// slope (tar bort små spikes)
    {
        if (refractory == 0) {
            peak = 1;
            refractory = (int)(FS * REFRACTORY_SEC);
        }
    }

    // 🔹 6. Shift history
    prev2 = prev1;
    prev1 = smooth;

    return peak;
}
float ppg_get_env(void)
{
    return env;
}

