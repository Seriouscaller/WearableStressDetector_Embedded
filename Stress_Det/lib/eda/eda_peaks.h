#pragma once

void eda_peaks_init(float sampling_rate);
void eda_peaks_process(float phasic);

float eda_get_scr_rate(void); // SC_RR (peaks per second)
int eda_get_scr_count(void);
