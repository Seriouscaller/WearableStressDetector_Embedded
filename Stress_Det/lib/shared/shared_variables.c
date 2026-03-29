#include "board_config.h"
#include "freertos/ringbuf.h"
#include "signal_processing.h"
#include "types.h"
#include <stdio.h>

// Data Collection
QueueHandle_t storage_queue;
psram_ring_buffer_t sensor_log;
SemaphoreHandle_t sensor_data_mutex;
SemaphoreHandle_t raw_data_mutex;
raw_data_t raw_data;
QueueHandle_t data_log_queue;
RingbufHandle_t raw_data_ringbuf;

// BLE
sensor_data_t ble_sensor_payload;
uint16_t sensor_chr_val_handle;
uint16_t conn_handle;

// PPG/GSR Processing
psram_window_ring_buffer_t sliding_window;
TaskHandle_t xFeatureExtractionTaskHandle = NULL;
raw_data_t processing_buffer[SNAPSHOT_LEN]; // 30 seconds of data approx 12 KB in RAM
