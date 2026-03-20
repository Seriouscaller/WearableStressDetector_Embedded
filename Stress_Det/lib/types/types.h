#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdio.h>

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint16_t gsr;
    uint16_t temp_raw;
    uint32_t ppg_green;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
} sensor_data_t;

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
} bmi_data_t;

typedef struct {
    sensor_data_t *data;    // Pointer to the PSRAM block
    uint32_t head;          // Next position to write
    uint32_t tail;          // Oldest data position
    uint32_t count;         // How many samples currently stored
    SemaphoreHandle_t lock; // Prevent read/write collisions
} psram_ring_buffer_t;

typedef struct {
    uint32_t *data;         // Pointer to the PSRAM block
    uint32_t head;          // Next position to write
    uint32_t count;         // How many samples currently stored
    SemaphoreHandle_t lock; // Prevent read/write collisions
} psram_ppg_ring_buffer_t;

typedef struct {
    bool enable_serial_output; // Master switch for Serial
    bool ppg_debug;            // MAX30101
    bool gsr_debug;            // CJMCU 6701
    bool imu_debug;            // BMI160
    bool temp_debug;           // TMP117
} sensor_debug_cfg_t;
