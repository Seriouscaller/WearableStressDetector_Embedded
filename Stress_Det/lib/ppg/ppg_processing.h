#pragma once
#include "ppg_hrv.h"

#define FS 256
#define WINDOW_SEC 30
#define STEP_SEC 1

void ppg_processing_init(void);

void ppg_process_sample(float sample);

ppg_features_t ppg_get_features(void);