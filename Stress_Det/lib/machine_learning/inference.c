#include "inference.h"
#include <stdio.h>

/*
1. Feature extraction for PPG and EDA from 30 second windows
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

typedef struct {
    float x;
    float y;
} BMU_t;

enum STRESSTYPE { CALM, MODERATE, HIGH };

float robust_scaler_normalization(float *value, float median, float iqr)
{
    float scaled_value = 0.0f;

    return scaled_value;
}

float get_euclidean_distance(float *weights, float vector)
{
    float distance = 0.0f;

    return distance;
}

BMU_t find_BMU()
{
    BMU_t bmu = {.x = 99, .y = 99};

    return bmu;
}

enum STRESSTYPE classification()
{
    enum STRESSTYPE class;

    return class;
}