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
extern psram_ring_buffer_t sensor_log;

static const char *TAG = "STORAGE";

// Initializes a PSRAM ringbuffer
esp_err_t init_psram_buffer(psram_ring_buffer_t *ring_buffer, uint32_t samples_count)
{
    size_t buffer_size = samples_count * sizeof(sensor_data_t);

    // Allocate the massive block in Octal PSRAM
    ring_buffer->data = (sensor_data_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);

    if (ring_buffer->data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate log buffer!");
        return ESP_FAIL;
    }

    // Initialize buffer variables
    ring_buffer->head = 0;
    ring_buffer->tail = 0;
    ring_buffer->count = 0;
    ring_buffer->lock = xSemaphoreCreateMutex(); // Semaphore to protect buffer access

    if (ring_buffer->lock == NULL) {
        ESP_LOGE(TAG, "Failed to create buffer lock!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Allocated %.2f MB for 2-hour log", (float)buffer_size / (1024 * 1024));
    return ESP_OK;
}

// Sync heartbeat task that runs according to SYNC_RATE to take a consistent snapshot of the
// current sensor data and send it to the storage task via a queue. This ensures that the data
// stored in flash is always consistent across all sensors, even if they are updated at different
// rates.
void sync_heartbeat_task(void *pvParameters)
{
    sensor_data_t snapshot;

    while (1) {
        // Take a snapshot of the CURRENT state of all sensors
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&snapshot, &ble_sensor_payload, sizeof(sensor_data_t));
            xSemaphoreGive(sensor_data_mutex);

            // Send this consistent snapshot to the storage task
            // If the queue is full, we drop the frame to keep the system real-time
            esp_err_t ret = xQueueSend(storage_queue, &snapshot, pdMS_TO_TICKS(5));
            if (ret != pdPASS) {
                ESP_LOGE(TAG, "Failed to send snapshot to storage queue");
            }

        } else {
            ESP_LOGE(TAG, "Could not take mutex to read sensor data for snapshot. Data sample lost!");
        }

        // Logging rate
        vTaskDelay(pdMS_TO_TICKS(SNAPSHOT_SYNC_RATE));
    }
}

// Task that gives status updates about PSRAM. How much of it is filled up, and how much remains
// etc.
void print_data_collection_status_task(void *pvParameters)
{

    while (1) {
        if (xSemaphoreTake(sensor_log.lock, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint32_t samples_stored = sensor_log.count;
            uint32_t head_pos = sensor_log.head;
            uint32_t tail_pos = sensor_log.tail;
            xSemaphoreGive(sensor_log.lock);

            float seconds_stored = samples_stored * (SNAPSHOT_SYNC_RATE / 1000.0f);
            float minutes_stored = seconds_stored / 60.0f;
            float fill_percentage = ((float)samples_stored / DATA_COLLECTION_SAMPLES_COUNT) * 100.0f;

            ESP_LOGI(TAG, "--- PSRAM Buffer Status ---");
            ESP_LOGI(TAG, "Samples: %lu / %d", samples_stored, DATA_COLLECTION_SAMPLES_COUNT);
            ESP_LOGI(TAG, "Time Stored: %.2f minutes (%.1f seconds)", minutes_stored, seconds_stored);
            ESP_LOGI(TAG, "Fill Level: %.2f%%", fill_percentage);
            ESP_LOGI(TAG, "Head Index: %lu | Tail Index: %lu", head_pos, tail_pos);

            if (samples_stored >= DATA_COLLECTION_SAMPLES_COUNT) {
                ESP_LOGW(TAG, "Buffer is LOOPING (2h limit reached, overwriting oldest data)");
            }
        } else {
            ESP_LOGE(TAG, "Semaphore timeout! Could not take buffer lock to print status");
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Print status every 5 seconds
    }
}

// Stores data in the ring buffer. Manage the head and tail pointers. When buffer is full, the head
// will wrap around and start overwriting the oldest data. Currently the storage holds 2h of
// sensordata.
void storage_task(void *pvParameters)
{
    sensor_data_t received_data;

    while (1) {
        // Block until the sync_heartbeat_task sends a new snapshot
        if (xQueueReceive(storage_queue, &received_data, portMAX_DELAY) == pdPASS) {

            // Lock the buffer to prevent a "Read" task from accessing mid-update
            if (xSemaphoreTake(sensor_log.lock, pdMS_TO_TICKS(5)) == pdTRUE) {

                // Insert data at the current Head
                sensor_log.data[sensor_log.head] = received_data;

                // Advance the Head (with wrap-around)
                sensor_log.head = (sensor_log.head + 1) % PS;

                // Manage the Tail and Count
                if (sensor_log.count < DATA_COLLECTION_SAMPLES_COUNT) {
                    sensor_log.count++;
                } else {
                    // Buffer is full; the Tail must move to stay ahead of the Head
                    // This means we are now overwriting the oldest data
                    sensor_log.tail = (sensor_log.tail + 1) % DATA_COLLECTION_SAMPLES_COUNT;
                }

                xSemaphoreGive(sensor_log.lock);
            } else {
                ESP_LOGE(TAG, "Semaphore timeout! Could not take buffer lock to store data");
            }
        }
    }
}
