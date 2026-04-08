#pragma once

typedef struct
{
    float tonic;
    float phasic;
    int scr_count;

} eda_features_t;

void eda_processing_init(void);

void eda_process_sample(float raw, float time);

eda_features_t eda_get_features(void);