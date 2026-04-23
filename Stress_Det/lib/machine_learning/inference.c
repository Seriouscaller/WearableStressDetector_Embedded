#include "inference.h"
#include "esp_log.h"
#include "som_model_200hz_ver9_ppg_center.h"
#include <float.h>
#include <math.h>
#include <stdio.h>

// Feature order in struct som_input_t and trained ML model:
// HR, HRV_RMSSD, HRV_SDNN, SCR_COUNT, EDA_TONIC, EDA_PHASIC

#define SOM_NEURONS 900
#define SOM_INPUT_LEN 3
bool debug_som = true;

static void normalize(float *input, float *scaled_output);
int classify_stress(som_input_t *features);
static uint16_t get_winning_neuron(float *scaled_input);

int classify_stress(som_input_t *features)
{
    /* Real data
    float input[SOM_INPUT_LEN] = {features->hr,  features->hrv_rmssd, features->hrv_sdnn,
                        (          features->scr, features->tonic,     features->phasic};*/

    /* Test data */
    // float input[SOM_INPUT_LEN] = {-11.11f, 11.11f, -11.11f, 11.11f, 11.11f, 11.11f};

    if ((features->hr == 0) || features->hrv_rmssd == 0) {
        return -1;
    }

    float input[SOM_INPUT_LEN] = {features->hr, features->hrv_rmssd, features->hrv_sdnn};

    // Normalization
    float scaled_output[SOM_INPUT_LEN] = {0};
    normalize(input, scaled_output);

    uint16_t bmu_index = get_winning_neuron(scaled_output);
    if (SOM_NEURONS - 1 < bmu_index) {
        ESP_LOGE("Inference", "BMU index out of range!");
        return -1;
    }

    if (debug_som) {
        // ESP_LOGI("Raw Input", "0:%f 1:%f 2:%f ", input[0], input[1], input[2]);
        // ESP_LOGI("Normalized Input", "0:%f 1:%f 2:%f ", scaled_output[0], scaled_output[1],
        // scaled_output[2]);

        // 0 = Neutral
        // 1 = Stress
        // 2 = Rest
        const char *cluster_name;
        switch (som_clusters[bmu_index]) {
        case 0:
            cluster_name = "NEUTRAL";
            break;
        case 1:
            cluster_name = "STRESS";
            break;
        case 2:
            cluster_name = "REST";
            break;
        default:
            cluster_name = "UNKNOWN";
        }

        ESP_LOGI("Results", "BMU idx: %u Class: %u %s", bmu_index, som_clusters[bmu_index], cluster_name);
    }
    return (int)som_clusters[bmu_index];
}

/*
*@brief Normalization using Min -
    Max Scaler *Maps input features to a[0, 1] range based on training data bounds.*/
static void normalize(float *input, float *scaled_output)
{
    for (int i = 0; i < SOM_INPUT_LEN; i++) {
        // formula: (x - min) / (max - min)
        scaled_output[i] = (input[i] - scaler_min[i]) / scaler_range[i];
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
