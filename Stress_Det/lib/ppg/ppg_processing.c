#include "ppg_processing.h"
#include "ppg_filter.h"
#include "ppg_peaks.h"
#include "ppg_hrv.h"

#define FS 256

static int sample_index = 0;
static int step_counter = 0;
static int last_peak = -1000;

static ppg_features_t features;

void ppg_processing_init()
{
    ppg_filter_init();
    ppg_peaks_init();
    ppg_hrv_init();
}

void ppg_process_sample(float raw)
{
    float filtered = ppg_filter_process(raw);

    sample_index++;
    step_counter++;

    if(ppg_detect_peak(filtered))
    {
        int interval = sample_index - last_peak;

        if(last_peak > 0)
        {
            float rr = interval * 1000.0f / FS;

            if(rr > 300 && rr < 2000)
            {
                float current_time = sample_index / (float)FS;

                ppg_add_rr(rr, current_time);
            }
        }

        last_peak = sample_index;
    }

    if(step_counter >= FS)
    {
        step_counter = 0;

        float now = sample_index / (float)FS;

        features = ppg_compute_hrv(now);
    }
}

ppg_features_t ppg_get_features()
{
    return features;
}