#pragma once
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "gatt.h"
#include <stdio.h>

#define BLE_NUM_OF_SAMPLES_PER_PAYLOAD 40

typedef struct __attribute__((packed)) {
    int64_t time_stamp; // 8B
    uint32_t ppg_raw;   // 4B
    float ppg_filtered; // 4B
    uint16_t gsr;       // 2B
} raw_data_t;           // Tot: 10B

typedef struct __attribute__((packed)) {
    float hr;        // 4B
    float hrv_rmssd; // 4B
    float hrv_sdnn;  // 4B
    float scr;       // 4B
    float tonic;     // 4B
    float phasic;    // 4B
} som_input_t;       // Tot: 24B

typedef struct __attribute__((packed)) {
    som_input_t features;        // 24B
    uint8_t experiment_phase;    // 1B
    uint8_t padding[3];          // 3B (Padding for CPU)
} som_input_transfer_learning_t; // Tot: 28B

typedef struct __attribute__((packed)) {
    uint32_t timestamp;          // 4B
    raw_data_t raw_samples[200]; // 2000B
    som_input_t features;        // 24B
    uint8_t stress_class;        // 1B
    uint8_t experiment_phase;    // 1B
} complete_log_t;                // Tot: 2030B

/*
// Total BLE PACKET: 2042B
// 5 Payloads each around 400B each
// ble_payload_bulk_t * 4 = 1600B
// ble_payload_final_t = 426B
*/

typedef struct __attribute__((packed)) {
    uint32_t timestamp;                                     // 4 Bytes
    raw_data_t raw_samples[BLE_NUM_OF_SAMPLES_PER_PAYLOAD]; // First 40 samples (40 * 10B = 400 Bytes)
} ble_payload_bulk_t;                                       // Total: 404B

typedef struct __attribute__((packed)) {
    uint32_t timestamp;                                     // 4 Bytes
    raw_data_t raw_samples[BLE_NUM_OF_SAMPLES_PER_PAYLOAD]; // Last 40 samples (400 bytes)
    float hr, rmssd, sdnn, scr, tonic, phasic;              // 4 * 6 = 24 Bytes
    uint8_t stress_class;                                   // (1 byte)
    uint8_t experiment_phase;                               // (1 byte)
} ble_payload_final_t;                                      // Total: 430 bytes

typedef struct {
    int16_t acc_x; // 2B
    int16_t acc_y; // 2B
    int16_t acc_z; // 2B
    int16_t gyr_x; // 2B
    int16_t gyr_y; // 2B
    int16_t gyr_z; // 2B
} bmi_data_t;      // Tot: 12B

typedef struct {
    i2c_master_dev_handle_t *max_handle;
    spi_device_handle_t *gsr_handle;
} sensor_handles_t;

typedef struct {
    uint16_t ble_sensor_chr_a_val_handle;
    uint16_t ble_sensor_chr_b_val_handle;
    uint16_t ble_sensor_chr_c_val_handle;
    uint16_t ble_sensor_chr_d_val_handle;
    uint16_t ble_sensor_chr_e_val_handle;
    uint16_t ble_command_chr_val_handle;
} ble_sensor_handles_t;

typedef struct {
    bool show_telemetry;
    bool show_logged_values;
    bool show_battery_log;
    bool show_gsr_debugging;
    bool show_spiff_status;
    bool enable_imu;
    bool enable_ppg;
    bool enable_gsr;
    bool enable_temp;
} device_control_t;
