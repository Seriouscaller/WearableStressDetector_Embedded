#include "ppg_processing.h"
#include "ppg_filter.h"
#include <stdio.h>

#define PPG_OFF_WRIST_THRESHOLD 20000

void ppg_processing_init()
{
    ppg_filter_init();
}

float ppg_process_sample(uint32_t raw)
{
    // If device is not worn, don't filter
    if (raw < PPG_OFF_WRIST_THRESHOLD) {
        return 0.0f;
    }
    float normalized = raw * 0.0001f;
    return ppg_filter_process(normalized) * 1000.0f;
}
