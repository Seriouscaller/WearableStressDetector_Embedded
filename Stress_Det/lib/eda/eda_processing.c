#include "eda_processing.h"
#include "eda_clean.h"
#include "eda_filter.h"
#include "eda_peaks.h"

#define FS 200
#define WINDOW_SEC 60
#define MAX_EDA (FS * WINDOW_SEC)

typedef struct {
    float tonic;
    float phasic;
    float time;
    int scr;
} eda_entry_t;

static eda_entry_t eda_buffer[MAX_EDA];
static int index = 0;
static int filled = 0;

static eda_features_t features;

void eda_processing_init()
{
    eda_clean_init();
    eda_filter_init();
    eda_peaks_init();

    index = 0;
    filled = 0;

    features.tonic = 0;
    features.phasic = 0;
    features.scr_count = 0;
}

void eda_process_sample(float raw, float time)
{
    // -------- STEP 1: CLEAN (NeuroKit-style) --------
    float clean = eda_clean_process(raw);

    // -------- STEP 2: TONIC / PHASIC --------
    eda_filter_process(clean);

    float tonic = eda_get_tonic();
    float phasic = eda_get_phasic();

    // -------- STEP 3: PEAK DETECTION --------
    int scr = eda_detect_scr(phasic, time);

    // -------- STEP 4: STORE IN RING BUFFER --------
    eda_buffer[index].tonic = tonic;
    eda_buffer[index].phasic = phasic;
    eda_buffer[index].time = time;
    eda_buffer[index].scr = scr;

    index = (index + 1) % MAX_EDA;

    if (filled < MAX_EDA)
        filled++;

    // -------- STEP 5: FEATURE EXTRACTION (TIME WINDOW) --------
    float tonic_sum = 0.0f;
    float phasic_sum = 0.0f;
    int scr_sum = 0;
    int count = 0;

    float window_start = time - WINDOW_SEC;

    for (int i = 0; i < filled; i++) {
        if (eda_buffer[i].time >= window_start) {
            tonic_sum += eda_buffer[i].tonic;
            phasic_sum += eda_buffer[i].phasic;
            scr_sum += eda_buffer[i].scr;
            count++;
        }
    }

    if (count > 0) {
        features.tonic = tonic_sum / count;
        features.phasic = phasic_sum / count;

        // SCR per window (kan även normaliseras per sekund)
        features.scr_count = scr_sum / WINDOW_SEC;
    }
}

eda_features_t eda_get_features()
{
    return features;
}