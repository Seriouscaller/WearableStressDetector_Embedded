/**
 * @file board_config.h
 * @brief Hardware abstraction layer and timing definitions for the XIAO S3.
 *
 * Defines GPIO mappings for I2C and SPI buses, specifies communication
 * frequencies, and establishes the sampling intervals for all bio-sensors.
 */

#pragma once

/* --- I2C Bus Configuration --- */
/**
 * @note Shared bus for:
 * - 0x48: TMP117 (High-precision temperature)
 * - 0x57: MAX30101 (PPG/Heart Rate)
 * - 0x68: BMI260 (6-Axis IMU)
 */
#define I2C_MASTER_SDA_IO 5 // Physical Pin D4 (S3)
#define I2C_MASTER_SCL_IO 6 // Physical Pin D5 (S3)
#define I2C_MASTER_FREQ_HZ 400000

/* --- SPI Bus Configuration (GSR Sensor) --- */
/**
 * @note CJMCU 6701 Skin Conductance Sensor mapping.
 * MOSI is disabled (-1) as the module is output-only.
 */
#define SPI_NUM_MISO 8 // XIAO D9
#define SPI_NUM_CLK 7  // XIAO D8
#define SPI_NUM_CS 3
#define SPI_NUM_MOSI -1 // Not present on GSR Module
#define SPI_FREQ_HZ 1000000

/* --- Sensor Sampling Intervals --- */
#define PPG_AND_GSR_SAMPLING_RATE_IN_MS TWOHUNDRED_HZ_IN_MS
#define IMU_SAMPLING_RATE_IN_MS ONEHUNDRED_HZ_IN_MS
#define TEMP_SAMPLING_RATE_IN_MS ONE_HZ_IN_MS
#define SAMPLES_PER_SECOND PPG_SAMPLE_RATE

#define TWOHUNDRED_HZ_IN_MS 5
#define ONEHUNDRED_HZ_IN_MS 10
#define FIFTY_HZ_IN_MS 20
#define TEN_HZ_IN_MS 100
#define ONE_HZ_IN_MS 1000

/* --- System Timing and Buffering --- */
#define BLE_NOTIFY_INTERVAL_MS 1000
#define BATTERY_SAMPLING_INTERVAL_MS 2000

/**
 * @brief Ring Buffer Sizing
 *
 * Calculated based on a 61-second window at 200Hz.
 * Used for storing BVP (Blood Volume Pulse) and EDA (Electrodermal Activity)
 * data for future DSP and feature extraction.
 */
#define RING_BUF_SIZE (WINDOW_SIZE * sizeof(raw_data_t) + 1024) // +1024 for overhead

#define PPG_SAMPLE_RATE 200
#define WINDOW_SIZE (61 * PPG_SAMPLE_RATE) // 30s = 6000 samples
