#pragma once

typedef struct {
    float hr;
    float rmssd;
    float sdnn;

} ppg_features_t;

void ppg_hrv_init(void);

void ppg_add_rr(float rr, float time);

ppg_features_t ppg_compute_hrv(float now);

float ppg_compute_hr(float now);