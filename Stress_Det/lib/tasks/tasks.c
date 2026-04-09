#include "adc.h"
#include "board_config.h"
#include "eda_processing.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "gatt.h"
#include "gsr.h"
#include "inference.h"
#include "max30101.h"
#include "ppg_processing.h"
#include "shared_variables.h"
#include "signal_processing.h"
#include <stdio.h>

static const char *TAG = "TASKS";
extern SemaphoreHandle_t ble_payload_mutex;
extern uint16_t ble_conn_handle;
extern bool is_sampling_active;
extern bool show_telemetry;
extern bool show_logged_values;
extern bool enable_imu;
extern bool enable_ppg;
extern bool enable_gsr;
extern bool enable_temp;
extern RingbufHandle_t raw_data_ringbuf;
extern QueueHandle_t data_log_queue;
extern ble_payload_a_t ble_payload_a;
extern ble_payload_b_t ble_payload_b;
extern ble_payload_c_t ble_payload_c;
extern volatile uint8_t current_experiment_phase;
extern SemaphoreHandle_t experiment_phase_mutex;

void sensor_sampling_task(void *pvParameters)
{
    sensor_handles_t *sensors = (sensor_handles_t *)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PPG_AND_GSR_SAMPLING_RATE_IN_MS);
    static raw_data_t bundle[PPG_SAMPLE_RATE]; // Collects 200 ppg & gsr samples in array
    int samples_collected = 0;

    ppg_processing_init();
    eda_processing_init();

    while (1) {
        if (is_sampling_active) {

            raw_data_t current_sample = {0};

            bool ppg_ok = (max30101_read_fifo(*sensors->max_handle, &current_sample.ppg_raw) == ESP_OK);
            bool gsr_ok = (gsr_sensor_read_raw(*sensors->gsr_handle, &current_sample.gsr) == ESP_OK);

            // Adds raw_data_t to static array
            if (ppg_ok && gsr_ok) {
                current_sample.ppg_filtered = ppg_process_sample(current_sample.ppg_raw);
                bundle[samples_collected++] = current_sample;
            } else {
                ESP_LOGW(TAG, "sensor_sampling_task - Skipped reading sensors!");
            }

            if (show_telemetry) {

                printf(">ppg raw: %lu\n", current_sample.ppg_raw);
                printf(">ppg filt:%f\n", current_sample.ppg_filtered);
                printf(">gsr: %u\n", current_sample.gsr);
            }

            // Once 200 samples (1 sec) is accumulated, bundle is sent off to Ringbuffer
            if (samples_collected >= PPG_SAMPLE_RATE) {
                if (xRingbufferSend(raw_data_ringbuf, bundle, sizeof(bundle), 0) == pdTRUE) {
                    samples_collected = 0;
                } else {
                    ESP_LOGE(TAG, "sensor_sampling_task - Ringbuffer full!");
                    samples_collected = 0;
                }
            }
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Extracts 30 seconds of raw_data_t from ringbuffer. Copies data to static array (history).
// Runs feature extraction to get features for ML model. ML model runs prediction
// on these features. Rawdata + features + ML result are added to complete_log_t.
// The complete_log_t is added to data_log_queue for BLE transmission.
void feature_extraction_task(void *pvParameters)
{
    raw_data_t *history = (raw_data_t *)heap_caps_malloc(WINDOW_SIZE * sizeof(raw_data_t), MALLOC_CAP_SPIRAM);

    if (history == NULL) {
        ESP_LOGE(TAG, "feature_extraction_task - Failed to allocate history buffer");
        vTaskDelete(NULL);
        return;
    }
    memset(history, 0, WINDOW_SIZE * sizeof(raw_data_t));

    while (1) {
        size_t item_size;

        // Wait here until 1 bundle of samples (200 raw_data_t) has arrived
        raw_data_t *new_samples =
            (raw_data_t *)xRingbufferReceive(raw_data_ringbuf, &item_size, pdMS_TO_TICKS(1500));

        if (new_samples != NULL) {
            // Shift the 29 seconds of "old" data to the front
            // Moving (3000 - 100) elements * size of each element
            memmove(history, &history[SAMPLES_PER_SECOND],
                    (WINDOW_SIZE - SAMPLES_PER_SECOND) * sizeof(raw_data_t));

            // Copy the 1 second of "new" data to the very end
            memcpy(&history[WINDOW_SIZE - SAMPLES_PER_SECOND], new_samples,
                   SAMPLES_PER_SECOND * sizeof(raw_data_t));

            // Running patriks feature extraction functions in here.
            som_input_t features = calculate_features(history, WINDOW_SIZE);

            // Inference using SOM model. Outputs class as single digit
            uint8_t result = classify_stress(&features);

            complete_log_t *final_log =
                (complete_log_t *)heap_caps_malloc(sizeof(complete_log_t), MALLOC_CAP_SPIRAM);
            if (final_log) {
                memcpy(final_log->raw_samples, new_samples, PPG_SAMPLE_RATE * sizeof(raw_data_t));
                final_log->features = features;
                final_log->stress_class = result;
                final_log->timestamp = xTaskGetTickCount();
                if (xSemaphoreTake(experiment_phase_mutex, pdMS_TO_TICKS(10))) {
                    final_log->experiment_phase = current_experiment_phase;
                    xSemaphoreGive(experiment_phase_mutex);
                } else {
                    ESP_LOGW(TAG, "feature_extraction_task - Failed to take semaphore. Exp. Phase not set!");
                }

                if (xQueueSend(data_log_queue, &final_log, 0) != pdTRUE) {
                    ESP_LOGE(TAG, "feature_extraction_task - Failed to send to queue!");
                    heap_caps_free(final_log);
                };

            } else {
                ESP_LOGE(TAG, "feature_extraction_task - Failed to allocate mem for final_log!");
            }
            vRingbufferReturnItem(raw_data_ringbuf, (void *)new_samples);
        } else if (new_samples == NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// Receives complete_log_t over queue each second. Max BLE Max Transmissible
// Unit is 512B, so complete_log_t is split into 3 parts. A, B, and C.
// A: timestamp 4B + raw_data[0-75] 450B = 454B
// B: timestamp + raw_data[75-150] 450B = 454B
// C: timestamp + raw_data[150-200] 300B + 5 floats (features) 20B +  class 1B = 325B
// Each part has it's own BLE char. Data is combined on the receiving end.
void logging_task(void *pvParameters)
{
    complete_log_t *received_log;
    while (1) {
        if (xQueueReceive(data_log_queue, &received_log, portMAX_DELAY) == pdTRUE) {
            if (show_logged_values) {
                ESP_LOGI(TAG, "t:%lu ppg:%lu gsr:%u rm:%f ton:%f cl:%u ph:%u", received_log->timestamp,
                         received_log->raw_samples[0].ppg_raw, received_log->raw_samples[0].gsr,
                         received_log->features.hrv_rmssd, received_log->features.tonic,
                         received_log->stress_class, received_log->experiment_phase);
            }

            if (xSemaphoreTake(ble_payload_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                uint32_t sync_time = (uint32_t)(esp_timer_get_time() / 1000);

                // Filling Part A
                ble_payload_a.timestamp = sync_time;
                memcpy(ble_payload_a.raw_samples, &received_log->raw_samples[0], sizeof(raw_data_t) * 75);

                // Filling Part B
                ble_payload_b.timestamp = sync_time;
                memcpy(ble_payload_b.raw_samples, &received_log->raw_samples[75], sizeof(raw_data_t) * 75);

                // Filling Part C
                ble_payload_c.timestamp = sync_time;
                memcpy(ble_payload_c.raw_samples, &received_log->raw_samples[150], sizeof(raw_data_t) * 50);

                ble_payload_c.rmssd = received_log->features.hrv_rmssd;
                ble_payload_c.sdnn = received_log->features.hrv_sdnn;
                ble_payload_c.tonic = received_log->features.tonic;
                ble_payload_c.phasic = received_log->features.phasic;
                ble_payload_c.scr = received_log->features.scr;
                ble_payload_c.stress_class = received_log->stress_class;
                ble_payload_c.experiment_phase = received_log->experiment_phase;

                xSemaphoreGive(ble_payload_mutex);
                heap_caps_free(received_log);
            } else {
                ESP_LOGW(TAG, "logging_task - Failed to take ble_payload semaphore, data lost!");
                heap_caps_free(received_log);
            }
        }
    }
}

// Update BLE message buffer every BLE_NOTIFY_INTERVAL_MS, and notify connected phone.
// Task responsible for sending data every second over BLE
void ble_update_task(void *pvParameters)
{
    while (1) {
        // Only send if a phone is connected
        if (ble_conn_handle != BLE_HS_CONN_HANDLE_NONE) {

            // Is ble_sensor_payload free from producers?
            if (xSemaphoreTake(ble_payload_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

                // Send Part A
                struct os_mbuf *om_a = ble_hs_mbuf_from_flat(&ble_payload_a, sizeof(ble_payload_a));
                ble_gatts_notify_custom(ble_conn_handle, ble_sensor_chr_a_val_handle, om_a);

                // Send Part B
                struct os_mbuf *om_b = ble_hs_mbuf_from_flat(&ble_payload_b, sizeof(ble_payload_b));
                ble_gatts_notify_custom(ble_conn_handle, ble_sensor_chr_b_val_handle, om_b);

                // Send Part C
                struct os_mbuf *om_c = ble_hs_mbuf_from_flat(&ble_payload_c, sizeof(ble_payload_c));
                ble_gatts_notify_custom(ble_conn_handle, ble_sensor_chr_c_val_handle, om_c);
                xSemaphoreGive(ble_payload_mutex);
            } else {
                ESP_LOGW(TAG, "ble_update_task - Failed to take ble_payload semaphore!");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_INTERVAL_MS));
    }
}

void battery_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    ESP_ERROR_CHECK(init_adc(&adc1_handle, &adc1_cali_chan0_handle));
    static int adc_raw;
    static int voltage_mV;

    while (1) {
        esp_err_t ret = read_battery_voltage(&adc1_handle, &adc1_cali_chan0_handle, &adc_raw, &voltage_mV);
        if (ret == ESP_OK) {
            log_battery_voltage(&adc_raw, &voltage_mV);
        } else {
            ESP_LOGE(TAG, "Failed to read battery Voltage!");
        }
        vTaskDelay(pdTICKS_TO_MS(2000));
    }
}