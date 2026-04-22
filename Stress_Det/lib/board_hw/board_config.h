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
#define SPI_NUM_CS 3
#define SPI_NUM_MOSI -1 // Not present on GSR Module
#define SPI_FREQ_HZ 1000000

// Sensor timings
#define PPG_AND_GSR_SAMPLING_RATE_IN_MS TWOHUNDRED_HZ_IN_MS
#define IMU_SAMPLING_RATE_IN_MS ONEHUNDRED_HZ_IN_MS
#define TEMP_SAMPLING_RATE_IN_MS ONE_HZ_IN_MS
#define TWOHUNDRED_HZ_IN_MS 5
#define ONEHUNDRED_HZ_IN_MS 10
#define FIFTY_HZ_IN_MS 20
#define TEN_HZ_IN_MS 100
#define ONE_HZ_IN_MS 1000

// BLE update interval (how often we send data to the connected phone/PC)
#define BLE_NOTIFY_INTERVAL_MS 1000

// Battery
#define BATTERY_SAMPLING_INTERVAL_MS 2000

// Ring buffer which keeps PPG and EDA values for future feature extraction
#define RING_BUF_SIZE (WINDOW_SIZE * sizeof(raw_data_t) + 1024) // +1024 for overhead

// PPG Processing
#define PPG_SAMPLE_RATE 200
#define WINDOW_SIZE (61 * PPG_SAMPLE_RATE) // 30s = 6000 samples

#define SAMPLES_PER_SECOND PPG_SAMPLE_RATE // 1 second @ 200Hz
