/**
 * @file types.h
 * @brief Data structure definitions for the XIAO ESP32-S3 Wearable Stress Monitor.
 *
 * Contains structures for sensor data, BLE payloads, SOM features,
 * and device configuration. Optimized for PSRAM storage and NimBLE transmission.
 */

#pragma once
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gatt.h"
#include <stdio.h>

#define BLE_NUM_OF_SAMPLES_PER_PAYLOAD 24
#define BLE_NUM_OF_BULK_PAYLOADS 8

/**
 * @brief Raw IMU data from BMI260.
 */
typedef struct {
    int16_t acc_x; /**< Accelerometer X-axis raw value */
    int16_t acc_y; /**< Accelerometer Y-axis raw value */
    int16_t acc_z; /**< Accelerometer Z-axis raw value */
    int16_t gyr_x; /**< Gyroscope X-axis raw value */
    int16_t gyr_y; /**< Gyroscope Y-axis raw value */
    int16_t gyr_z; /**< Gyroscope Z-axis raw value */
} bmi_data_t;      /* Total: 12 Bytes */

/**
 * @brief Minimal sample structure for bulk logging/BLE streaming.
 */
typedef struct __attribute__((packed)) {
    int64_t time_stamp; /**< Microseconds since boot (esp_timer_get_time) */
    uint32_t ppg_raw;   /**< Raw ADC/FIFO value from MAX30101 (Green LED) */
    float ppg_filtered; /**< Biquad/EMA filtered PPG signal */
    uint16_t gsr;       /**< Raw 12-bit ADC value from CJMCU 6701 SPI */
} raw_log_data_t;       /* Total: 18 Bytes (Updated from 16B due to int64 alignment) */

/**
 * @brief Internal high-resolution data point for real-time processing.
 */
typedef struct __attribute__((packed)) {
    int64_t time_stamp;         /**< esp_timer timestamp */
    uint32_t ppg_raw;           /**< Raw PPG sensor data */
    float ppg_filtered;         /**< Processed PPG AC component */
    uint16_t gsr;               /**< Raw GSR ADC value */
    float gsr_scaled;           /**< GSR converted to Voltage/Resistance */
    float gsr_clean;            /**< GSR Phasic/Filtered component */
    bmi_data_t bmi_data;        /**< Synchronized IMU snapshot */
    bool has_movement_artifact; /**< Flag set by detect_motion logic */
} raw_data_t;

/**
 * @brief Input vector for the Self-Organizing Map (SOM).
 */
typedef struct __attribute__((packed)) {
    float hr;        /**< Heart Rate in BPM */
    float hrv_rmssd; /**< Root Mean Square of Successive Differences */
    float sc_ph;     /**< Skin Conductance Phasic component (peaks) */
    float sc_rr;     /**< Skin Conductance Response Rate */
} som_input_t;       /* Total: 16 Bytes */

/**
 * @brief Structure for Transfer Learning snapshots saved to SPIFFS.
 */
typedef struct __attribute__((packed)) {
    som_input_t features;        /**< 16B feature vector */
    uint8_t experiment_phase;    /**< Phase ID for supervised labeling */
    uint8_t padding[3];          /**< Alignment padding for 32-bit CPU access */
} som_input_transfer_learning_t; /* Total: 20 Bytes */

/**
 * @brief Full 1-second data bundle intended for PSRAM storage.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;              /**< Start of window timestamp */
    raw_log_data_t raw_samples[200]; /**< 1 second of 200Hz raw data */
    som_input_t features;            /**< Extracted features for this second */
    uint8_t stress_class;            /**< Result: 0=Neutral, 1=Stress, 2=Rest */
    uint16_t num_of_classifications; /**< Running session counter */
    uint8_t experiment_phase;        /**< Current experimental context */
} complete_log_t;

/**
 * @brief Payload for NimBLE bulk data characteristics (A-H).
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;                                         /**< Sync timestamp */
    raw_log_data_t raw_samples[BLE_NUM_OF_SAMPLES_PER_PAYLOAD]; /**< Fragmented raw samples */
} ble_payload_bulk_t;                                           // Total: 424B

/**
 * @brief Payload for NimBLE summary characteristic (I).
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;              /**< Sync timestamp */
    raw_log_data_t raw_samples[8];   /**< Remaining samples to complete the second */
    float hr;                        /**< Heart Rate feature */
    float rmssd;                     /**< HRV feature */
    float sc_ph;                     /**< GSR Phasic feature */
    float sc_rr;                     /**< GSR Response Rate feature */
    uint8_t stress_class;            /**< Final ML classification result */
    uint16_t num_of_classifications; /**< Cumulative classification count */
    uint8_t experiment_phase;        /**< Active experiment phase */
} ble_payload_final_t;

/**
 * @brief Peripheral handles for I2C and SPI communication.
 */
typedef struct {
    i2c_master_dev_handle_t *max_handle; /**< MAX30101 I2C device handle */
    i2c_master_dev_handle_t *bmi_handle; /**< BMI260 I2C device handle */
    spi_device_handle_t *gsr_handle;     /**< CJMCU 6701 SPI device handle */
} sensor_handles_t;

/**
 * @brief NimBLE Value Handles for GATT characteristics.
 */
typedef struct {
    uint16_t ble_sensor_chr_a_val_handle;
    uint16_t ble_sensor_chr_b_val_handle;
    uint16_t ble_sensor_chr_c_val_handle;
    uint16_t ble_sensor_chr_d_val_handle;
    uint16_t ble_sensor_chr_e_val_handle;
    uint16_t ble_sensor_chr_f_val_handle;
    uint16_t ble_sensor_chr_g_val_handle;
    uint16_t ble_sensor_chr_h_val_handle;
    uint16_t ble_sensor_chr_i_val_handle; /**< Final summary characteristic handle */
    uint16_t ble_command_chr_val_handle;  /**< Command/Input characteristic handle */
} ble_sensor_handles_t;

/**
 * @brief Global flags for runtime device configuration and debugging.
 */
typedef struct {
    bool show_telemetry;     /**< Print raw data to serial for Teleplot */
    bool show_logged_values; /**< Verbose logging of processed features */
    bool show_battery_log;   /**< Print battery voltage/percentage status */
    bool show_gsr_debugging; /**< Print GSR specific ADC/Filtering data */
    bool show_spiff_status;  /**< Print flash storage usage on each save */
    bool enable_imu;         /**< Power on BMI260 */
    bool enable_ppg;         /**< Power on MAX30101 */
    bool enable_gsr;         /**< Power on CJMCU 6701 */
    bool enable_temp;        /**< Power on TMP117 */
} device_control_t;