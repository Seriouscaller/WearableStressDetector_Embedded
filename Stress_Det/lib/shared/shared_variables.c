#include "board_config.h"
#include "ppg_processing.h"
#include "types.h"
#include <stdio.h>

// Data Collection
QueueHandle_t storage_queue;
psram_ring_buffer_t sensor_log;
SemaphoreHandle_t sensor_data_mutex;

// BLE
sensor_data_t ble_sensor_payload;
uint16_t sensor_chr_val_handle;
uint16_t conn_handle;

// PPG Processing
psram_ppg_ring_buffer_t ppg_sliding_window;
TaskHandle_t xPpgProcessingTaskHandle = NULL;
uint32_t processing_buffer[SNAPSHOT_LEN]; // 30 seconds of data approx 12 KB in RAM

// Debugging
sensor_debug_cfg_t debug_cfg;
