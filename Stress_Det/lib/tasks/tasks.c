#include "bmi260.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "gatt.h"
#include "gsr.h"
#include "i2c_common.h"
#include "max30101.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "spi_common.h"
#include "tmp117.h"
#include "types.h"
#include <stdio.h>
#include <string.h>

void create_tasks(i2c_master_dev_handle_t tmp_handle, i2c_master_dev_handle_t max_handle,
                  i2c_master_dev_handle_t bmi_handle, spi_device_handle_t gsr_handle);
static void imu_bmi260_task(void *pvParameters);
static void temp_task(void *pvParameters);
static void ppg_task(void *pvParameters);
static void gsr_task(void *pvParameters);
static void ble_update_task(void *pvParameters);
static void sync_heartbeat_task(void *pvParameters);
static void print_buffer_status_task(void *pvParameters);
static void storage_task(void *pvParameters);

static const char *TAG = "SENSOR_TASKS";
uint16_t sensor_chr_val_handle;
extern sensor_data_t ble_sensor_payload;
extern SemaphoreHandle_t sensor_data_mutex;
extern QueueHandle_t storage_queue;
extern uint16_t conn_handle;
extern psram_ring_buffer_t sensor_log;

void create_tasks(i2c_master_dev_handle_t tmp_handle, i2c_master_dev_handle_t max_handle,
                  i2c_master_dev_handle_t bmi_handle, spi_device_handle_t gsr_handle)
{
    // Each task is pinned to core 1 to avoid conflicts with BLE stack on core 0.
    // Task priorities are set based on sensor read frequency and importance.

    // High-speed IMU Task (100Hz)
    xTaskCreatePinnedToCore(imu_bmi260_task, "imu_task", 4096, bmi_handle, 10, NULL, 1);
    // Slow Temperature Task (1Hz)
    xTaskCreatePinnedToCore(temp_task, "temp_task", 2048, tmp_handle, 2, NULL, 1);
    // Heart Rate Task (50Hz)
    xTaskCreatePinnedToCore(ppg_task, "ppg_task", 4096, max_handle, 9, NULL, 1);
    // GSR Task (10Hz)
    xTaskCreatePinnedToCore(gsr_task, "gsr_task", 4096, gsr_handle, 8, NULL, 1);
    // Send struct via BLE (100 ms)
    xTaskCreatePinnedToCore(ble_update_task, "ble_update_task", 4096, NULL, 4, NULL, 1);
    // Sync task that takes consistent snapshots of BLE_payload and sends to storage queue
    xTaskCreatePinnedToCore(sync_heartbeat_task, "sync_task", 4096, NULL, 5, NULL, 1);
    // Storage task that receives snapshots from queue and writes to PSRAM ring buffer
    xTaskCreatePinnedToCore(storage_task, "storage_task", 4096, NULL, 3, NULL, 1);
    // Print status of buffer every 5 seconds
    xTaskCreatePinnedToCore(print_buffer_status_task, "prt_bufr_status_tsk", 4096, NULL, 1, NULL, 1);
};

// Sampling of IMU sensor every 10 ms (100 Hz).
// Adds read data to shared ble_sensor_payload struct
static void imu_bmi260_task(void *pvParameters)
{
    i2c_master_dev_handle_t bmi_handle = (i2c_master_dev_handle_t)pvParameters;
    bmi_data_t imu_data = {0};

    while (1) {
        if (bmi260_read(bmi_handle, &imu_data) == ESP_OK) {
            // Update shared ble_sensor_payload with new IMU data. Mutex ensures that
            // only one task can access ble_sensor_payload at a time, preventing data corruption.
            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.acc_x = imu_data.acc_x;
                ble_sensor_payload.acc_y = imu_data.acc_y;
                ble_sensor_payload.acc_z = imu_data.acc_z;
                ble_sensor_payload.gyr_x = imu_data.gyr_x;
                ble_sensor_payload.gyr_y = imu_data.gyr_y;
                ble_sensor_payload.gyr_z = imu_data.gyr_z;

                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Ax: %d Ay: %d Az: %d Gx: %d Gy: %d Gz: %d", imu_data.acc_x, imu_data.acc_y,
                         imu_data.acc_z, imu_data.gyr_x, imu_data.gyr_y, imu_data.gyr_z);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read BMI260 sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLING_RATE_HZ));
    }
}

// Sampling of temperature sensor every 1000 ms (1 Hz).
// Adds read data to shared ble_sensor_payload struct
static void temp_task(void *pvParameters)
{
    i2c_master_dev_handle_t tmp_handle = (i2c_master_dev_handle_t)pvParameters;
    float current_temp = 0.0f;

    while (1) {
        if (tmp117_read_temp(tmp_handle, &current_temp) == ESP_OK) {

            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.temp_raw = (uint16_t)(current_temp * 100);
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read temp: %.2f", current_temp);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TEMP_SAMPLING_RATE_HZ));
    }
}

// Sampling of PPG (blood) sensor every 20 ms (50 Hz).
// Adds read data to shared ble_sensor_payload struct
static void ppg_task(void *pvParameters)
{
    i2c_master_dev_handle_t max_handle = (i2c_master_dev_handle_t)pvParameters;
    uint32_t current_ppg = 0;

    while (1) {
        if (max30101_read_fifo(max_handle, &current_ppg) == ESP_OK) {

            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.ppg_green = current_ppg;
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read PPG: %lu", current_ppg);
            }

        } else {
            ESP_LOGW(TAG, "Failed to read MAX30101 sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(PPG_SAMPLING_RATE_HZ));
    }
}

// Sampling of GSR (sweat) sensor every 100 ms (10 Hz).
// Adds read data to shared ble_sensor_payload struct
static void gsr_task(void *pvParameters)
{
    spi_device_handle_t gsr_handle = (spi_device_handle_t)pvParameters;
    uint16_t current_gsr = 0;

    while (1) {
        if (gsr_sensor_read_raw(gsr_handle, &current_gsr) == ESP_OK) {

            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                ble_sensor_payload.gsr = current_gsr;
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read GSR: %u", current_gsr);
            }

        } else {
            ESP_LOGW(TAG, "Failed to read GSR sensor.");
        }
        vTaskDelay(pdMS_TO_TICKS(GSR_SAMPLING_RATE_HZ));
    }
}

// Update BLE message buffer every 500 ms, and notify connected phone.
static void ble_update_task(void *pvParameters)
{
    while (1) {
        // Only send if a phone is connected
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {

            // Is ble_sensor_payload free from producers?
            if (xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                struct os_mbuf *om = ble_hs_mbuf_from_flat(&ble_sensor_payload, sizeof(ble_sensor_payload));

                // Notify connected phone with new sensor data. If om is NULL, it means
                // there was an error creating the mbuf.
                if (om != NULL) {
                    ble_gatts_notify_custom(conn_handle, sensor_chr_val_handle, om);
                }
                xSemaphoreGive(sensor_data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_INTERVAL_MS));
    }
}

// Sync heartbeat task that runs according to SYNC_RATE to take a consistent snapshot of the
// current sensor data and send it to the storage task via a queue. This ensures that the data
// stored in flash is always consistent across all sensors, even if they are updated at different
// rates.
static void sync_heartbeat_task(void *pvParameters)
{
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

// Task that gives status updates about PSRAM. How much of it is filled up, and how much remains.
static void print_buffer_status_task(void *pvParameters)
{

    while (1) {
        if (xSemaphoreTake(sensor_log.lock, pdMS_TO_TICKS(10)) == pdTRUE) {

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

// Stores data in the ring buffer. Manage the head and tail pointers. When buffer is full, the head
// will wrap around and start overwriting the oldest data. Currently the storage holds 2h of
// sensordata.
static void storage_task(void *pvParameters)
{
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
