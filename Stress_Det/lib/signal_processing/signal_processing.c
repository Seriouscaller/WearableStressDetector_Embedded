// Extracts derived features from PPG and GSR values, such as
// HRV and tonic, phasic values.

#include "board_config.h"
#include "esp_log.h"
#include "types.h"
#include <stdio.h>

static const char *TAG = "SIGNAL_P";

// Template for feature calculations on PPG & GSR
som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    // Extract PPG & EDA features here. Append to the som_input
    // Dont forget to normalize into floats!

    som_input_t features = {0};

    return features;
}
