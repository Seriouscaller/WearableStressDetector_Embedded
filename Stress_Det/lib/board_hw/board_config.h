#pragma once

/*
I2C Devices:
0x48 TMP117 Temperature Sensor
0x57 MAX30101 PPG sensor
0x69 BMI160 IMU Accelerometer & Gyroscope
*/

// I2C Definitions for PPG, IMU and Temperature Sensors
#define I2C_MASTER_SDA_IO 5 // Physical Pin D4 (S3)
#define I2C_MASTER_SCL_IO 6 // Physical Pin D5 (S3)
#define I2C_MASTER_FREQ_HZ 400000

// SPI Definitions for GSR Sensor
#define SPI_NUM_MISO 8 // XIAO D9
#define SPI_NUM_CLK 7  // XIAO D8
#define SPI_NUM_CS 2
#define SPI_NUM_MOSI -1 // Not present on GSR Module
#define SPI_FREQ_HZ 1000000

// Sensor timings
#define IMU_SAMPLING_RATE_HZ ONEHUNDRED_HZ_IN_MS
#define PPG_SAMPLING_RATE_HZ FIFTY_HZ_IN_MS
#define GSR_SAMPLING_RATE_HZ TEN_HZ_IN_MS
#define TEMP_SAMPLING_RATE_HZ ONE_HZ_IN_MS
#define ONEHUNDRED_HZ_IN_MS 10
#define FIFTY_HZ_IN_MS 20
#define TEN_HZ_IN_MS 100
#define ONE_HZ_IN_MS 1000

// BLE update interval (how often we send data to the connected phone/PC)
#define BLE_NOTIFY_INTERVAL_MS 500

// Time before snapshot of ble payload is taken and sent to queue
#define SNAPSHOT_SYNC_RATE 50

// Number of samples that can be stored in PSRAM buffer.
// ~2 hours at 20Hz (20Hz * 60s * 120m) = 144000 samples
// 144000 * 24B (size of sensor_data_t) = 3.46MB
#define LOG_SAMPLES_COUNT 144000