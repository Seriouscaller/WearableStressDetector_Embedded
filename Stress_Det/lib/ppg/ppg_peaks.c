#include "ppg_peaks.h"
/*Peak detection for PPG signals
3 samples for local maxima detection*/

static float prev1 = 0;
static float prev2 = 0;

static int refractory = 0;

void ppg_peaks_init()
{
    prev1 = prev2 = 0;
    refractory = 0;
}

int ppg_detect_peak(float x)
{
    int peak = 0;

    if(refractory > 0)
        refractory--;

    if(prev1 > prev2 && prev1 > x && prev1 > 0.02f)
    {
        if(refractory == 0)
        {
            peak = 1;
            refractory = 64;   // ~250 ms
        }
    }

    prev2 = prev1;
    prev1 = x;

    return peak;
}