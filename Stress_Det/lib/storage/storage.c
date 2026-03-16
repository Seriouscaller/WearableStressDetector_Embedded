#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "i2c_common.h"
#include "spi_common.h"
#include "max30101.h"
#include "tmp117.h"
#include "gsr.h"
#include "bmi160.h"
#include "types.h"
#include "board_config.h"

extern sensor_data_t ble_sensor_payload;
extern SemaphoreHandle_t sensor_data_mutex;
extern QueueHandle_t storage_queue;

psram_ring_buffer_t sensor_log;
#define LOG_SAMPLES_COUNT 144000 // ~2 hours at 20Hz (20 * 60 * 120)

static const char *TAG = "STORAGE";

// Initializes the ring buffer in PSRAM for storing sensor data. This function allocates a large block of memory 
// in PSRAM to hold the sensor data samples, and sets up the necessary variables for managing the ring buffer. 
// The buffer is designed to hold approximately 2 hours of data at a 20Hz sampling rate. Each sample consists
// of a snapshot of all sensor readings at a given time.
void init_psram_buffer() {
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
    sensor_log.lock = xSemaphoreCreateMutex();  // Semaphore to protect buffer access
    
    ESP_LOGI("PSRAM", "Allocated %.2f MB for 2-hour log", (float)buffer_size / (1024 * 1024));
}

// Sync heartbeat task that runs according to SYNC_RATE to take a consistent snapshot of the 
// current sensor data and send it to the storage task via a queue. This ensures that the data 
// stored in flash is always consistent across all sensors, even if they are updated at different rates.
void sync_heartbeat_task(void *pvParameters) {
    sensor_data_t snapshot;
    
    while (1) {
        // Take a snapshot of the CURRENT state of all sensors
        if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(&snapshot, &ble_sensor_payload, sizeof(sensor_data_t));
            xSemaphoreGive(sensor_data_mutex);

            // Send this consistent snapshot to the storage task
            // If the queue is full, we drop the frame to keep the system real-time
            xQueueSend(storage_queue, &snapshot, 0); 
        }

        // Logging rate
        vTaskDelay(pdMS_TO_TICKS(SNAPSHOT_SYNC_RATE)); 
    }
}

// Task that gives status updates about PSRAM. How much of it is filled up, and how much remains etc.
void print_buffer_status_task(void *pvParameters) {

    while(1) {
        if (xSemaphoreTake(sensor_log.lock, pdMS_TO_TICKS(10)) == pdTRUE) {
            
            // Calculate the time stored based on the 20Hz (50ms) sync rate
            float seconds_stored = sensor_log.count * (SNAPSHOT_SYNC_RATE / 1000.0f); 
            float minutes_stored = seconds_stored / 60.0f;
            float fill_percentage = ((float)sensor_log.count / LOG_SAMPLES_COUNT) * 100.0f;

            ESP_LOGI(TAG, "--- PSRAM Buffer Status ---");
            ESP_LOGI(TAG, "Samples: %lu / %d", sensor_log.count, LOG_SAMPLES_COUNT);
            ESP_LOGI(TAG, "Time Stored: %.2f minutes (%.1f seconds)", minutes_stored, seconds_stored);
            ESP_LOGI(TAG, "Fill Level: %.2f%%", fill_percentage);
            ESP_LOGI(TAG, "Head Index: %lu | Tail Index: %lu", sensor_log.head, sensor_log.tail);
            
            if (sensor_log.count >= LOG_SAMPLES_COUNT) {
                ESP_LOGW(TAG, "Buffer is LOOPING (2h limit reached, overwriting oldest data)");
            }
            
            xSemaphoreGive(sensor_log.lock);
        } else {
            ESP_LOGE(TAG, "Could not take buffer lock to print status");
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Print status every 5 seconds
    }
}

// Stores data in the ring buffer. Manage the head and tail pointers. When buffer is full, the head will 
// wrap around and start overwriting the oldest data. Currently the storage holds 2h of sensordata.
void storage_task(void *pvParameters) {
    sensor_data_t received_data;

    while (1) {
        // Block until the sync_heartbeat_task sends a new snapshot
        if (xQueueReceive(storage_queue, &received_data, portMAX_DELAY) == pdPASS) {
            
            // Lock the buffer to prevent a "Read" task from accessing mid-update
            if (xSemaphoreTake(sensor_log.lock, pdMS_TO_TICKS(5)) == pdTRUE) {
                
                // Insert data at the current Head
                sensor_log.buffer[sensor_log.head] = received_data;

                // Advance the Head (with wrap-around)
                sensor_log.head = (sensor_log.head + 1) % LOG_SAMPLES_COUNT;

                // Manage the Tail and Count
                if (sensor_log.count < LOG_SAMPLES_COUNT) {
                    sensor_log.count++;
                } else {
                    // Buffer is full; the Tail must move to stay ahead of the Head
                    // This means we are now overwriting the oldest data
                    sensor_log.tail = (sensor_log.tail + 1) % LOG_SAMPLES_COUNT;
                }

                xSemaphoreGive(sensor_log.lock);
            }
        }
    }
}
