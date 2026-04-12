#include "board_config.h"
#include "freertos/ringbuf.h"
#include "gatt.h"
#include "host/ble_hs.h"
#include "signal_processing.h"
#include "types.h"
#include <stdio.h>

// FreeRTOS
TaskHandle_t xFeatureExtractionTaskHandle = NULL;

// Data Collection
raw_data_t raw_data = {0};
QueueHandle_t data_log_queue = NULL;
RingbufHandle_t raw_data_ringbuf = NULL;

// BLE
ble_payload_bulk_t ble_payloads_bulk[4] = {0};
ble_payload_final_t ble_payload_final = {0};
uint16_t ble_command_chr_val_handle = 0;
ble_sensor_handles_t ble_val_handles = {0};

uint16_t ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
uint8_t ble_received_packet[2] = {0, 0};
volatile uint8_t current_experiment_phase = 255;

SemaphoreHandle_t experiment_phase_mutex = NULL;
SemaphoreHandle_t ble_payload_mutex = NULL;

// PPG/GSR Processing
raw_data_t processing_buffer[WINDOW_SIZE]; // 30 seconds of data approx 12 KB in RAM

// Data collection on-device
som_input_transfer_learning_t transfer_learning_buffer[15] = {0};
