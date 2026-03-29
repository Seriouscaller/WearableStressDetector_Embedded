#include "board_config.h"
#include "esp_log.h"
#include "types.h"
#include <stdio.h>

static const char *TAG = "PPG_PROC";

som_input_t calculate_features(raw_data_t history[], uint16_t window_size)
{
    // Extract PPG & EDA features here. Append to the som_input
    // Dont forget to normalize into floats!

    som_input_t features = {0};

    return features;
}
