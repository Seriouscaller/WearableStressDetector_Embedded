#pragma once
#include "ppg_hrv.h"

#define FS 200
#define WINDOW_SEC 30
#define STEP_SEC 1

void ppg_processing_init(void);

void ppg_process_sample(float sample);

int ppg_features_ready(void);

ppg_features_t ppg_get_features(void);

float ppg_get_filtered(void);

int ppg_get_peak(void);

float ppg_get_hr(void);
