#pragma once

/*
I2C Devices:
0x48 TMP117 Temperature Sensor
0x57 MAX30101 PPG sensor
0x69 BMI160 IMU Accelerometer & Gyroscope
*/

#define I2C_MASTER_SDA_IO  5      // Physical Pin D4 (S3)
#define I2C_MASTER_SCL_IO  6      // Physical Pin D5 (S3)
#define I2C_MASTER_FREQ_HZ 400000

#define SPI_NUM_MISO  8     // XIAO D9
#define SPI_NUM_CLK   7     // XIAO D8
#define SPI_NUM_CS    2
#define SPI_NUM_MOSI -1     // Not present on GSR Module
#define SPI_FREQ_HZ   1000000

#define GSR_R_FIXED 100000.0f;
#define GSR_V_REF   3.3f;

// Sensor timings
#define ONE_HZ_IN_MS 1000
#define TWO_HZ_IN_MS 500
#define TEN_HZ_IN_MS 100
#define FIFTY_HZ_IN_MS 20
#define ONEHUNDRED_HZ_IN_MS 10
#define BLE_NOTIFY_INTERVAL_MS 500
#define SNAPSHOT_SYNC_RATE 50
