#pragma once
#include <stdint.h>

void ppg_peaks_init(void);
int ppg_detect_peak(float sample);
float ppg_get_threshold(void);

float ppg_get_env(void);