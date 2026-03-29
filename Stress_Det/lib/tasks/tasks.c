#include "bmi260.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "gatt.h"
#include "gsr.h"
#include "i2c_common.h"
#include "inference.h"
#include "max30101.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "shared_variables.h"
#include "signal_processing.h"
#include "spi_common.h"
#include "storage.h"
#include "tmp117.h"
#include "types.h"
#include <stdio.h>
#include <string.h>

void ble_update_task(void *pvParameters);

static const char *TAG = "TASKS";
extern bool show_telemetry;
extern uint16_t sensor_chr_val_handle;
extern sensor_data_t ble_sensor_payload;
extern SemaphoreHandle_t sensor_data_mutex;
extern uint16_t conn_handle;
extern bool enable_imu;
extern bool enable_ppg;
extern bool enable_gsr;
extern bool enable_temp;
extern raw_data_t raw_data;
extern RingbufHandle_t raw_data_ringbuf;
extern QueueHandle_t data_log_queue;
QueueHandle_t ml_queue;

#define NEW_DATA_SIZE 100 // 1 second @ 100Hz
#define WINDOW_SIZE 3000

void producer_task(void *pvParameters)
{
    sensor_handles_t *sensors = (sensor_handles_t *)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLING_RATE_IN_MS); // 100 Hz
    static raw_data_t bundle[100];
    int samples_collected = 0;

    while (1) {
        raw_data_t current_sample = {0};

        bool ppg_ok = (max30101_read_fifo(*sensors->max_handle, &current_sample.ppg) == ESP_OK);
        bool gsr_ok = (gsr_sensor_read_raw(*sensors->gsr_handle, &current_sample.gsr) == ESP_OK);

        if (ppg_ok && gsr_ok) {
            bundle[samples_collected++] = current_sample;
        } else {
            ESP_LOGW(TAG, "Skipped reading sensors!");
        }

        if (samples_collected >= 100) {
            if (xRingbufferSend(raw_data_ringbuf, bundle, sizeof(bundle), 0) == pdTRUE) {
                samples_collected = 0;
            } else {
                ESP_LOGE(TAG, "Ringbuffer full!");
                samples_collected = 0;
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void feature_extraction_task(void *pvParameters)
{
    raw_data_t *history = (raw_data_t *)heap_caps_malloc(WINDOW_SIZE * sizeof(raw_data_t), MALLOC_CAP_SPIRAM);

    if (history == NULL) {
        ESP_LOGE(TAG, "Failed to allocate history buffer");
        vTaskDelete(NULL);
        return;
    }
    memset(history, 0, WINDOW_SIZE * sizeof(raw_data_t));

    while (1) {
        size_t item_size;

        // Wait here until 1 bundle of samples (100 raw_data_t) has arrived
        raw_data_t *new_samples =
            (raw_data_t *)xRingbufferReceive(raw_data_ringbuf, &item_size, pdMS_TO_TICKS(1500));

        if (new_samples != NULL) {
            // Shift the 29 seconds of "old" data to the front
            // Moving (3000 - 100) elements * size of each element
            memmove(history, &history[NEW_DATA_SIZE], (WINDOW_SIZE - NEW_DATA_SIZE) * sizeof(raw_data_t));

            // Copy the 1 second of "new" data to the very end
            memcpy(&history[WINDOW_SIZE - NEW_DATA_SIZE], new_samples, NEW_DATA_SIZE * sizeof(raw_data_t));

            // Return the memory to the Ring Buffer immediately
            vRingbufferReturnItem(raw_data_ringbuf, (void *)new_samples);

            // Running patriks feature extraction functions in here.
            // Along with normalization into floats
            som_input_t features = calculate_features(history, WINDOW_SIZE);

            uint8_t result = som_model_predict(&features);

            complete_log_t *final_log = malloc(sizeof(complete_log_t));
            if (final_log) {
                memcpy(final_log->raw_samples, new_samples, 100 * sizeof(raw_data_t));
                final_log->features = features;
                final_log->stress_class = result;
                final_log->timestamp = xTaskGetTickCount();

                xQueueSend(data_log_queue, &final_log, 0);
            }
        }
    }
}

void logging_task(void *pvParameters)
{
    complete_log_t *received_log;
    while (1) {
        if (xQueueReceive(data_log_queue, &received_log, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "%lu, %f, %f, %u", received_log->timestamp, received_log->features.hrv_rmssd,
                     received_log->features.tonic, received_log->stress_class);
            free(received_log);
        }
    }
}

// Update BLE message buffer every 500 ms, and notify connected phone.
void ble_update_task(void *pvParameters)
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
