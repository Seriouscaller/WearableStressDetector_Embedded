#include "board_config.h"
#include "ppg_processing.h"
#include "types.h"
#include <stdio.h>

// Data storage
QueueHandle_t storage_queue;
psram_ring_buffer_t sensor_log;
psram_ppg_ring_buffer_t ppg_sliding_window;

// Sensors
uint16_t sensor_chr_val_handle;
SemaphoreHandle_t sensor_data_mutex;

// BLE
sensor_data_t ble_sensor_payload;
uint16_t conn_handle;

// PPG Task
TaskHandle_t xPpgProcessingTaskHandle = NULL;
uint32_t processing_buffer[SNAPSHOT_LEN]; // 30 seconds of data approx 12 KB in RAM
