#pragma once
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdio.h>

typedef struct __attribute__((packed)) {
    uint32_t ppg;
    uint16_t gsr;
} raw_data_t;

typedef struct __attribute__((packed)) {
    float hrv_rmssd;
    float hrv_sdnn;
    float tonic;
    float phasic;
    float scr;
} som_input_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;
    raw_data_t raw_samples[200];
    som_input_t features;
    uint8_t stress_class;
    uint8_t experiment_phase; // (1 byte)
} complete_log_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // 4 Bytes
    raw_data_t raw_samples[75]; // First 75 Samples (450 Bytes)
} ble_payload_a_t;              // Total: 454 bytes

typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // 4 Bytes
    raw_data_t raw_samples[75]; // Middle 75 samples (450 bytes)
} ble_payload_b_t;              // Total: 454 bytes

typedef struct __attribute__((packed)) {
    uint32_t timestamp;                    // 4 Bytes
    raw_data_t raw_samples[50];            // Last 50 samples (300 bytes)
    float rmssd, sdnn, tonic, phasic, scr; // (20 bytes)
    uint8_t stress_class;                  // (1 byte)
    uint8_t experiment_phase;              // (1 byte)
} ble_payload_c_t;                         // Total: 326 bytes

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
