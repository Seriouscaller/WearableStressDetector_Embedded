/**
 * @file board_config.h
 * @brief Global device state and feature toggles.
 *
 * This file contains the volatile flags used for inter-task synchronization
 * and the master configuration struct that dictates the operating mode
 * of the XIAO ESP32-S3 wearable.
 */

#include "board_config.h"
#include "types.h"
#include <stdio.h>

/**
 * @brief Global sampling control flag.
 *
 * Marked as 'volatile' to prevent compiler optimization, as this value is
 * modified by the BLE write callback (Core 0) and polled by the
 * sensor acquisition tasks (Core 1).
 */
volatile bool is_sampling_active = true;

device_control_t device_config = {
    .show_telemetry = true,      // Output formatted sensor data to Serial
    .show_logged_values = false, // Log raw values stored in flash
    .show_battery_log = false,   // Output battery voltage/percentage logs
    .show_gsr_debugging = false, // Output raw GSR SPI transactions
    .show_spiff_status = false,  // Log filesystem health and usage
    .enable_ppg = true,          // Enable MAX30101 (Green LED)
    .enable_gsr = true,          // Enable CJMCU 6701 (SPI)
    .enable_imu = true,          // Enable BMI260 (I2C)
    .enable_temp = true,         // Enable internal/external temperature sensing
};
