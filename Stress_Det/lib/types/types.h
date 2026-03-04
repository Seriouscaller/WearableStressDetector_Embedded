#pragma once
#include <stdint.h>

/**
 * @brief High-level App Structure (Internal Use)
 * Used in main.c for sensor processing and logic.
 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} bmi160_data_t;

typedef struct {
    uint32_t ppg_green;      // Raw value from MAX30101
    float temperature_c;     // High-precision float from TMP117
    bmi160_data_t imu;       // Nested IMU data from BMI160
    uint16_t gsr_raw;        // Raw SPI data from CJMCU 6701
} app_sensor_data_t;

/**
 * @brief Packed BLE Structure (Over-the-Air)
 * Designed to be exactly 26 bytes for efficient notification.
 */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;  
    uint16_t company_id; 
    uint16_t gsr;        
    uint16_t temp_raw;   // temperature_c * 100
    uint32_t ppg_green;  
    int16_t acc_x;       
    int16_t acc_y;       
    int16_t acc_z;       
    int16_t gyr_x;       
    int16_t gyr_y;       
    int16_t gyr_z;       
} ble_sensor_payload_t;