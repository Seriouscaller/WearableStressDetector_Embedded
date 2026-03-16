// Storage for collecting sensor data to a ring buffer on PSRAM.

#include "bmi160.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "gsr.h"
#include "i2c_common.h"
#include "max30101.h"
#include "spi_common.h"
#include "tmp117.h"
#include "types.h"
#include <stdio.h>
#include <string.h>

extern sensor_data_t ble_sensor_payload;
extern SemaphoreHandle_t sensor_data_mutex;
extern QueueHandle_t storage_queue;

psram_ring_buffer_t sensor_log;

static const char *TAG = "STORAGE";

// Initializes the ring buffer in PSRAM for storing sensor data. This function allocates a large
// block of memory in PSRAM to hold the sensor data samples, and sets up the necessary variables for
// managing the ring buffer. The buffer is designed to hold approximately 2 hours of data at a 20Hz
// sampling rate. Each sample consists of a snapshot of all sensor readings at a given time.
void init_psram_buffer()
{
    size_t buffer_size = LOG_SAMPLES_COUNT * sizeof(sensor_data_t);

    // Allocate the massive block in Octal PSRAM
    sensor_log.buffer = (sensor_data_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);

    if (sensor_log.buffer == NULL) {
        ESP_LOGE("PSRAM", "Failed to allocate log buffer!");
        return;
    }

    // Initialize buffer variables
    sensor_log.head = 0;
    sensor_log.tail = 0;
    sensor_log.count = 0;
    sensor_log.lock = xSemaphoreCreateMutex(); // Semaphore to protect buffer access

    ESP_LOGI("PSRAM", "Allocated %.2f MB for 2-hour log", (float)buffer_size / (1024 * 1024));
}
