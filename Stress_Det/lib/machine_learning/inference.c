#include "inference.h"
#include "esp_log.h"
#include "som_model_4_features.h"
#include <float.h>
#include <math.h>
#include <stdio.h>

// Feature order in struct som_input_t and trained ML model:
// HR, HRV_RMSSD, SC_PH, SC_RR

#define SOM_NEURONS 484
#define SOM_INPUT_LEN 4
bool debug_som = true;

static void normalize(float *input, float *scaled_output);
int classify_stress(som_input_t *features);
static uint16_t get_winning_neuron(float *scaled_input);

/**
 * @brief  Classifies the user's stress state using a Self-Organizing Map.
 *
 * Takes synchronized features from both the PPG and GSR processing chains.
 * The algorithm normalizes the features to a common scale and calculates the
 * Euclidean distance to all neurons in the SOM to find the winner (BMU).
 *
 * @param[in] features Pointer to a struct containing HR, HRV_RMSSD, SC_PH, and SC_RR.
 *
 * @return
 *      - 0: REST (Low arousal, high HRV)
 *      - 1: NEUTRAL (Baseline state)
 *      - 2: STRESS (High arousal, low HRV, high GSR activity)
 *      - -1: Error (Missing data or indexing fault)
 */
int classify_stress(som_input_t *features)
{

    if ((features->hr == 0) || features->hrv_rmssd == 0) {
        return -1;
    }

    float input[SOM_INPUT_LEN] = {features->hr, features->hrv_rmssd, features->sc_ph, features->sc_rr};

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
            cluster_name = "REST";
            break;
        case 1:
            cluster_name = "NEUTRAL";
            break;
        case 2:
            cluster_name = "STRESS";
            break;
        default:
            cluster_name = "UNKNOWN";
        }

        ESP_LOGI("Results", "BMU idx: %u Class: %u %s", bmu_index, som_clusters[bmu_index], cluster_name);
    }
    return (int)som_clusters[bmu_index];
}

/**
 * @brief  Normalizes input features to a [0, 1] range for SOM inference.
 *
 * This is a standard Min-Max scaler implementation. It ensures that features
 * with different units (BPM for Heart Rate, µS for GSR) exert equal influence
 * on the Euclidean distance calculation during the BMU search.
 *
 * @param[in]  input         Raw feature vector [HR, HRV, SC_PH, SC_RR].
 * @param[out] scaled_output Normalized vector where each element is [0.0, 1.0].
 *
 * @note The 'scaler_min' and 'scaler_range' (max - min) arrays must be
 *       identical to those used during the offline training phase in
 *       order to maintain classification accuracy.
 */
static void normalize(float *input, float *scaled_output)
{
    for (int i = 0; i < SOM_INPUT_LEN; i++) {
        /**
         * Min-Max Formula: x_scaled = (x - x_min) / (x_max - x_min)
         * We use 'scaler_range' as the denominator to save a subtraction
         * operation during each iteration.
         */
        scaled_output[i] = (input[i] - scaler_min[i]) / scaler_range[i];
    }
}

/**
 * @brief  Finds the Best Matching Unit (BMU) in the Self-Organizing Map.
 *
 * Compares the 4-dimensional normalized input vector against the weights
 * of every neuron in the SOM. It uses the squared Euclidean distance
 * metric to determine similarity.
 *
 * @param[in] scaled_input  Normalized feature vector [HR, HRV, SC_PH, SC_RR].
 *
 * @return uint16_t The index of the winning neuron (0 to SOM_NEURONS - 1).
 *
 * @note This function avoids the 'sqrtf' call for performance, as
 *       if (a^2 < b^2), then (a < b) for all positive distances.
 */
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
