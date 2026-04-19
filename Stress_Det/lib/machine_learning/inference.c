#include "inference.h"
#include "esp_log.h"
#include "som_model_200hz.h"
#include <float.h>
#include <math.h>
#include <stdio.h>

// Feature order in struct som_input_t and trained ML model:
// HR, HRV_RMSSD, HRV_SDNN, SCR_COUNT, EDA_TONIC, EDA_PHASIC

#define SOM_NEURONS 400
#define SOM_INPUT_LEN 3
bool debug_som = true;

static void normalize(float *input, float *scaled_output);
int classify_stress(som_input_t *features);
static uint16_t get_winning_neuron(float *scaled_input);

int classify_stress(som_input_t *features)
{
    /* Real data
    float input[SOM_INPUT_LEN] = {features->hr,  features->hrv_rmssd, features->hrv_sdnn,
                                  features->scr, features->tonic,     features->phasic};*/

    /* Test data */
    // float input[SOM_INPUT_LEN] = {-11.11f, 11.11f, -11.11f, 11.11f, 11.11f, 11.11f};

    float input[SOM_INPUT_LEN] = {features->hr, features->hrv_rmssd, features->hrv_sdnn};

    // Normalization
    float scaled_input[SOM_INPUT_LEN] = {0};
    normalize(input, scaled_input);

    uint16_t bmu_index = get_winning_neuron(scaled_input);
    if (SOM_NEURONS - 1 < bmu_index) {
        ESP_LOGE("Inference", "BMU index out of range!");
        return -1;
    }

    if (debug_som) {
        // ESP_LOGI("Raw Input", "0:%f 1:%f 2:%f ", input[0], input[1], input[2]);
        // ESP_LOGI("Normalized Input", "0:%f 1:%f 2:%f ", scaled_input[0], scaled_input[1], scaled_input[2]);
        ESP_LOGI("Results", "BMU idx: %u Class: %u", bmu_index, som_clusters[bmu_index]);
    }
    return (int)som_clusters[bmu_index];
}

/* Normalization using RobustScaler */
static void normalize(float *input, float *scaled_output)
{
    // scaled = input - median / IQR
    for (int i = 0; i < SOM_INPUT_LEN; i++) {
        scaled_output[i] = (input[i] - scaler_median[i]) / scaler_iqr[i];
    }
}

static uint16_t get_winning_neuron(float *scaled_input)
{
    uint16_t best_neuron = 0;
    float min_distance = FLT_MAX;

    for (int neuron = 0; neuron < SOM_NEURONS; neuron++) {
        float current_distance = 0;

        for (int feature = 0; feature < SOM_INPUT_LEN; feature++) {
            float diff = scaled_input[feature] - som_weights[neuron * SOM_INPUT_LEN + feature];
            current_distance += diff * diff;
        }
        if (current_distance < min_distance) {
            min_distance = current_distance;
            best_neuron = neuron;
        }
    }
    return best_neuron;
}
