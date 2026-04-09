#include "inference.h"
#include "esp_log.h"
#include "som_model_200hz.h"
#include <float.h>
#include <math.h>
#include <stdio.h>

/*
2. Pack floats in correct order in array
3. Normalize the data using robustscaler
    scaled = input - median / IQR
    (Possibility to clip data to prevent sensor spikes)
4. Finding the BMU
    4.1 Iterate through all 400 neurons
    4.2 Calculate eucledian distance between scaled inputvector and neurons 6 weights
    4.3 Neuron with smallest distance is selected.
5. Cross reference which class the neuron belongs to.
*/

#define INPUT_VECTORS 7
bool debug_som = true;

static void normalize(float *input, float *scaled_output);
uint8_t som_model_predict(som_input_t *features);

uint8_t som_model_predict(som_input_t *features)
{
    float input[INPUT_VECTORS];
    memset(input, 0, INPUT_VECTORS * sizeof(float));

    input[0] = features->hr;
    input[1] = features->hrv_rmssd;
    input[2] = features->hrv_sdnn;
    input[4] = features->tonic;
    input[5] = features->phasic;
    input[6] = features->scr;

    // Normalization
    float scaled_input[INPUT_VECTORS];
    memset(scaled_input, 0, INPUT_VECTORS * sizeof(float));
    normalize(input, scaled_input);

    if (debug_som) {
        ESP_LOGI("Raw Input", "0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f", input[0], input[1], input[2], input[3],
                 input[4], input[5], input[6]);

        ESP_LOGI("Normalized Input", "0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f", scaled_input[0], scaled_input[1],
                 scaled_input[2], scaled_input[3], scaled_input[4], scaled_input[5], scaled_input[6]);
    }
    // Calculate distance

    // Best Matching Unit

    // Get cluster

    uint8_t results = 0;
    return results;
}

static void normalize(float *input, float *scaled_output)
{
    // scaled = input - median / IQR
    for (int i = 0; i < INPUT_VECTORS; i++) {
        scaled_output[i] = (input[i] - scaler_median[i]) / scaler_iqr[i];

        if (debug_som) {
        }
    }
}
