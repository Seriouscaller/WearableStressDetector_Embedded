#include "board_config.h"
#include "freertos/ringbuf.h"
#include "gatt.h"
#include "host/ble_hs.h"
#include "signal_processing.h"
#include "types.h"
#include <stdio.h>

// ==========================================
// FreeRTOS Handles
// ==========================================
TaskHandle_t xFeatureExtractionTaskHandle = NULL;
QueueHandle_t data_log_queue = NULL;
QueueHandle_t telemetry_queue = NULL;
RingbufHandle_t raw_data_ringbuf = NULL;

// ==========================================
// Sensor & System State
// ==========================================
/**
 * These small variables stay in Internal SRAM for fast access
 * by high-frequency sampling tasks (200Hz).
 */
raw_data_t raw_data = {0};
bmi_data_t imu_data = {0};
float battery_percentage = 0.0f;
float temperature = 0.0f;
uint16_t num_of_classifications = 0;

// ==========================================
// BLE Global State
// ==========================================
/**
 * Mutexes to prevent race conditions between the Processing task (Core 1)
 * and the BLE/Logging tasks (Core 0).
 */
SemaphoreHandle_t experiment_phase_mutex = NULL;
SemaphoreHandle_t ble_payload_mutex = NULL;

uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
uint16_t ble_command_chr_val_handle = 0;
uint8_t ble_received_packet[2] = {0, 0};
volatile uint8_t current_experiment_phase = 255;

ble_sensor_handles_t ble_val_handles = {0};

// ==========================================
// PSRAM Allocated Buffers (R8 Advantage)
// ==========================================

// Buffers for fragmented BLE notifications (Characteristics A-H and Final)
ble_payload_bulk_t ble_payloads_bulk[8] = {0};
ble_payload_final_t ble_payload_final = {0};

// The 30-second sliding window for SOM feature extraction (approx 360KB)
raw_data_t processing_buffer[WINDOW_SIZE];

// The batch buffer for SPIFFS training data persistence
som_input_transfer_learning_t transfer_learning_buffer[15] = {0};
