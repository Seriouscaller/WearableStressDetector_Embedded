#include "board_config.h"
#include "freertos/ringbuf.h"
#include "signal_processing.h"
#include "types.h"
#include <stdio.h>

// FreeRTOS
TaskHandle_t xFeatureExtractionTaskHandle = NULL;

// Data Collection
raw_data_t raw_data = {0};
SemaphoreHandle_t ble_payload_mutex = NULL;
QueueHandle_t data_log_queue = NULL;
RingbufHandle_t raw_data_ringbuf = NULL;

// BLE
ble_payload_a_t ble_payload_a = {0};
ble_payload_b_t ble_payload_b = {0};
ble_payload_c_t ble_payload_c = {0};
uint16_t ble_sensor_chr_a_val_handle;
uint16_t ble_sensor_chr_b_val_handle;
uint16_t ble_sensor_chr_c_val_handle;
uint16_t ble_conn_handle;

// PPG/GSR Processing
raw_data_t processing_buffer[WINDOW_SIZE]; // 30 seconds of data approx 12 KB in RAM
