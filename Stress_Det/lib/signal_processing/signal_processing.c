// Extracts derived features from PPG and GSR values, such as
// HRV and tonic, phasic values.

#include "signal_processing.h"
#include "board_config.h"
#include "esp_log.h"
#include "ppg_hrv.h"
#include "ppg_peaks.h"
#include "test_signal.h"
#include "types.h"
#include <stdio.h>

#define MINIMUM_AMOUNT_OF_DATA (FIVE_SEC * SAMPLE_RATE)
#define MAXIMUM_AMOUNT_OF_DATA (THIRTY_SEC * SAMPLE_RATE)
#define FIVE_SEC 5
#define THIRTY_SEC 30
#define SAMPLE_RATE 200
#define TOTAL_SAMPLES 6000

bool debug_zero_cross = false;

som_input_t calculate_features(raw_data_t history[], uint16_t window_size);
static int count_zero_crossings(const float history[], uint16_t window_size);

static const char *TAG = "SIGNAL_PROCESSING";

enum SignalState {
    ABOVE = 1,
    BELOW = -1,
};

// Template for feature calculations on PPG & GSR
som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    // Extract PPG & EDA features here. Append to the som_input
    // Dont forget to normalize into floats!

    som_input_t features = {0};
    // SINE_WAVE_600 Amp:10 | 596 samples | Perfect wave | Zero-crossings: 5
    // NOISY_SINE_WAVE_600 Amp:10 | 598 samples | jitters and spikes | Zero-crossings: 5
    int crossings = count_zero_crossings(NOISY_SINE_WAVE_600, 598);
    return features;
}

static int count_zero_crossings(const float history[], uint16_t window_size)
{
    float THRESHOLD = 1.0f;
    enum SignalState state;

    if (history[0] > 0) {
        state = ABOVE;
    } else {
        state = BELOW;
    }

    uint16_t zero_crossings = 0;
    for (int i = 1; i < window_size; i++) {

        if ((state == ABOVE) && (history[i] < -THRESHOLD)) {
            zero_crossings++;
            state = BELOW;
            if (debug_zero_cross)
                ESP_LOGI(TAG, "Above to Below at [%d] val: %.2f", i, history[i]);
        } else if ((state == BELOW) && (history[i] > THRESHOLD)) {
            zero_crossings++;
            state = ABOVE;
            if (debug_zero_cross)
                ESP_LOGI(TAG, "Below to Above at [%d] val: %.2f", i, history[i]);
        }
        if (debug_zero_cross) {
            if (state == ABOVE) {
                ESP_LOGI(TAG, "Signal ABOVE 0. [%d] val: %.2f", i, history[i]);
            } else {
                ESP_LOGI(TAG, "Signal BELOW 0. [%d] val: %.2f", i, history[i]);
            }
        }
    }
    if (debug_zero_cross)
        ESP_LOGI(TAG, "Zerocrossings found: %d", zero_crossings);
    return zero_crossings;
}

/*
Starting up
 - Wait until we have atleast MINIMUM_AMOUNT_OF_DATA before we extract features. Process partial array
 - When 30 seconds of data arrived window

Data quality
 - Calculate quality of signal. If signal is not good, skip feature extraction.
   Using statistical measurements

Time domain
Systolic peaks = Big heartbeats
Diastolic peak = small coupled with systolic


Zero-crossing state - Reset logic when crossing the 0 value.

Noise resilience
 Look at three points. Keep comparing the three to each other. When the middle is the highest, peak found.
 Noise can complicate the algorithm
 Create a threshold of the ampliture. The beat has to be above a certain level to be counted.
 Once a beat is detected, we blind the algorithm for a short duration to prevent double detections.
 For example the dicrotic notch.

 Search strategy
  - Find slope
  - Validation
    1. value high enough?
    2. enough time passed?
    3. is slope rise consistent with a heartbeat peak?
*/

/*
Signal Quality
    Amplitude bounds
     P - P < lowthreshold = No skin contact
     P - P > High = Too much skin pressure

    Zero-crossing Rate (Noise Detector)
     Normal:1-4 crossings
     High noise: 15+
     Counting how many times the signal crosses the 0 boundrary

    Standard deviation
     If current std is much higher than rest of window, the
     user most likely moved.

    Slope consistency
     If the change between two points is inhumanely large, flag
     the sample as invalid.

    Quality Map
     Keep a separate array that keeps track of the quality of
     the data in the 1 second window. A bad window can be
     excluded from the feature extraction.

*/