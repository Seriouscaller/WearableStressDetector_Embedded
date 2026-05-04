#include "ppg_processing.h"
#include "ppg_filter.h"
#include <stdio.h>

#define PPG_OFF_WRIST_THRESHOLD 4000

/**
 * @brief  Top-level initialization for the PPG processing pipeline.
 *
 * Prepares the signal conditioning environment. Currently, this focuses on
 * initializing the Biquad filters (0.5Hz - 8Hz bandpass). This is required
 * to ensure that the filters do not start with 'garbage' data in their
 * delay lines, which could cause a system crash or massive signal spikes.
 */
void ppg_processing_init()
{
    ppg_filter_init();
}

/**
 * @brief  Pre-processes raw PPG data with off-wrist detection and scaling.
 *
 * Before filtering, the function checks if the signal intensity is below the
 * 'worn' threshold. If valid, it scales the 18-bit raw value down to prevent
 * floating-point precision issues in the Biquad stages, then extracts the
 * AC component using the cascaded HP/LP filters.
 *
 * @param[in] raw The 18-bit raw sample from the MAX30101 FIFO.
 *
 * @return float  The isolated AC pulse signal, or 0.0f if the device is off-wrist.
 */
float ppg_process_sample(uint32_t raw)
{
    /**
     * Off-Wrist Detection
     * When the MAX30101 is not against skin, the Green LED reflection drops
     * significantly. Returning 0.0f here prevents the Biquad filters from
     * accumulating noise-driven error in their delay lines.
     */
    if (raw < PPG_OFF_WRIST_THRESHOLD) {
        return 0.0f;
    }

    /**
     * Pre-Scaling
     * Scaling (raw * 0.0001f) brings 18-bit values (up to 262,144) into a
     * range of ~0.0 to 26.2. This prevents coefficient multiplication
     * from hitting extreme magnitudes inside the IIR logic.
     */
    float normalized = raw * 0.0001f;

    /**
     * Band-Pass Filtering
     * Stage 1: HP (0.5Hz) - Strips the DC offset.
     * Stage 2: LP (8.0Hz) - Smooths the pulse.
     */
    float ac_component = ppg_filter_process(normalized);

    /**
     * Post-Scaling
     * Multiplied by 1000.0f to restore signal amplitude for the peak detector,
     * making the tiny systolic variations easier to identify.
     */
    return ac_component * 1000.0f;
}
