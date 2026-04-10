#pragma once
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>

#define BLE_NUM_OF_SAMPLES_PER_PAYLOAD 40

typedef struct __attribute__((packed)) {
    uint32_t ppg_raw;
    float ppg_filtered;
    uint16_t gsr;
} raw_data_t;

typedef struct __attribute__((packed)) {
    float hr;
    float hrv_rmssd;
    float hrv_sdnn;
    float scr;
    float tonic;
    float phasic;
} som_input_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    raw_data_t raw_samples[200];
    som_input_t features;
    uint8_t stress_class;
    uint8_t experiment_phase; // (1 byte)
} complete_log_t;

/*
// Total BLE PACKET: 2042B
// Payloads each around 400B each
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
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
} bmi_data_t;

typedef struct {
    i2c_master_dev_handle_t *max_handle;
    spi_device_handle_t *gsr_handle;
} sensor_handles_t;
