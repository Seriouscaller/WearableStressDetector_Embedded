#include "eda_filter.h"
#include <math.h>

#define FS 200.0f

// values depends on cutoff frequency and sampling rate. For 0.05Hz cutoff and 200Hz sampling:
static float b0 = 0.9995066f;
static float b1 = -1.9990132f;
static float b2 = 0.9995066f;
static float a1 = -1.9990129f;
static float a2 = 0.9990135f;

static float z1 = 0.0f, z2 = 0.0f;
static float mean = 0.0f;
static float phasic = 0.0f;
static float tonic = 0.0f;
static float phasic_smooth = 0.0f;
static int tonic_initialized = 0;

/**
 * @brief  Initializes the state variables for EDA signal decomposition.
 *
 * This function resets the filters and accumulators used to separate the raw
 * Skin Conductance (SC) into Tonic (Level) and Phasic (Response) components.
 * It must be called before the main sensor sampling loop begins to ensure
 * the baseline estimation (Tonic) starts from a neutral state.
 *
 * @note The variables initialized here (z1, z2, mean, etc.) are typically
 *       global or static variables used within the 'biquad_process' or
 *       custom EDA decomposition algorithms.
 */
void eda_filter_init(void)
{
    z1 = 0;
    z2 = 0;
    mean = 0;
    phasic = 0;
    tonic = 0;
    phasic_smooth = 0;
    tonic_initialized = 0;
}

/**
 * @brief  Decomposes the raw EDA signal into Tonic and Phasic components.
 *
 * This function implements a baseline-tracking algorithm. It uses a very slow
 * Exponential Moving Average (EMA) to identify the Tonic Skin Conductance
 * Level (SCL). The Phasic component is then derived by subtracting the Tonic
 * from the input 'x', isolating the Skin Conductance Responses (SCR).
 *
 * @param[in] x  The raw (or pre-filtered) skin conductance sample in microsiemens (µS).
 *
 * @note The smoothing coefficient 0.0002f at 200Hz provides a time constant
 *       of approximately 25 seconds, ensuring the Tonic level ignores
 *       rapid physiological spikes.
 *
 * @see tonic
 * @see phasic
 */
void eda_filter_process(float x)
{
    // Initialize tonic with the first sample
    if (!tonic_initialized) {
        tonic = x;
        tonic_initialized = 1;
    }
    // Update tonic with a slow moving average
    tonic += 0.0002f * (x - tonic);

    // Center the signal by removing the tonic component
    // find variation around the tonic level, which is the phasic component
    // float centered = x;

    // high-pass filter to extract phasic component
    /*float y = b0 * centered + z1;
    z1 = b1 * centered - a1 * y + z2;
    z2 = b2 * centered - a2 * y; */

    phasic = x - tonic;

    if (phasic < 0) {
        phasic = 0; // Only consider positive variations as phasic responses
    }

    phasic *= 0.5f; // Scale down the phasic component for better visualization
}

/**
 * @brief  Retrieves the current Phasic component of the EDA signal.
 *
 * This getter function provides access to the processed Phasic value calculated
 * in the 'eda_filter_process' routine. In a typical wearable workflow, this
 * value is polled by the BLE task to be sent as a notification to the central
 * device for real-time visualization.
 *
 * @return float The current phasic skin conductance response in scaled units.
 */
float eda_get_phasic(void)
{
    return phasic;
}

/**
 * @brief  Retrieves the current Tonic component (SCL) of the EDA signal.
 *
 * Returns the baseline skin conductance level calculated in 'eda_filter_process'.
 * The Tonic component is used in longitudinal studies to track shifts in
 * autonomic nervous system activity over minutes or hours.
 *
 * @return float The current tonic skin conductance level in microsiemens (µS).
 */
float eda_get_tonic(void)
{
    return tonic;
}
