#pragma once

/*
I2C Devices:
0x48 TMP117 Temperature Sensor
0x57 MAX30101 PPG sensor
0x68 BMI260 IMU Accelerometer & Gyroscope
*/

// I2C Master Definitions for PPG, IMU and Temperature Sensors
#define I2C_MASTER_SDA_IO 5 // Physical Pin D4 (S3)
#define I2C_MASTER_SCL_IO 6 // Physical Pin D5 (S3)
#define I2C_MASTER_FREQ_HZ 400000

// SPI Master Definitions for GSR Sensor
#define SPI_NUM_MISO 8 // XIAO D9
#define SPI_NUM_CLK 7  // XIAO D8
#define SPI_NUM_CS 2
#define SPI_NUM_MOSI -1 // Not present on GSR Module
#define SPI_FREQ_HZ 1000000

// Sensor timings
#define IMU_SAMPLING_RATE_IN_MS ONEHUNDRED_HZ_IN_MS
#define PPG_SAMPLING_RATE_IN_MS ONEHUNDRED_HZ_IN_MS
#define SAMPLING_RATE_IN_MS ONEHUNDRED_HZ_IN_MS
#define GSR_SAMPLING_RATE_IN_MS TEN_HZ_IN_MS
#define TEMP_SAMPLING_RATE_IN_MS ONE_HZ_IN_MS
#define ONEHUNDRED_HZ_IN_MS 10
#define FIFTY_HZ_IN_MS 20
#define TEN_HZ_IN_MS 100
#define ONE_HZ_IN_MS 1000

// BLE update interval (how often we send data to the connected phone/PC)
#define BLE_NOTIFY_INTERVAL_MS 500

// Time before snapshot of raw_data is taken and sent to queue
#define SNAPSHOT_SYNC_RATE 10

// Sliding window holds 90 seconds of ppg data
// Circular buffer
#define SLIDING_WINDOW_PPG_SAMPLES_COUNT 20000

// PPG Processing
#define PPG_SAMPLE_RATE 100
#define SNAPSHOT_LEN (30 * PPG_SAMPLE_RATE)  // 30s = 3000 samples
#define RING_BUF_CAPACITY (SNAPSHOT_LEN * 2) // 60s = 6000 samples

#define DATA_COLLECTION_SAMPLES_COUNT 144000

// (1Hz * 60s * 60m) = 3600
// Size: 3600 samples * 12B (som_data_t) = 43 200B
#define SOM_DATA_COLLECTION_SAMPLES_COUNT 3600

// (100Hz * 10s) = 1000 Samples
// Size: 1000 samples * 24B (full_data_t) = 24 000B
#define FULL_DATA_COLLECTION_SAMPLES_COUNT 1000

#define RING_BUF_SIZE (WINDOW_SIZE * sizeof(raw_data_t) + 1024) // +1024 for overhead
#define WINDOW_SIZE 3000