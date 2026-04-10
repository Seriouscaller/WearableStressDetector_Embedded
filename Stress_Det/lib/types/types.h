#pragma once
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>

#define BLE_NUM_OF_SAMPLES_PER_PAYLOAD 40

typedef struct __attribute__((packed)) {
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
    float hr, rmssd, sdnn, scr, tonic, phasic;              // (24 bytes)
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
