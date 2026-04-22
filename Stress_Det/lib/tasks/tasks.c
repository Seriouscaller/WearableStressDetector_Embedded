#include "adc.h"
#include "bmi260.h"
#include "board_config.h"
#include "eda_processing.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/ringbuf.h"
#include "gatt.h"
#include "gsr.h"
#include "inference.h"
#include "max30101.h"
#include "ppg_filter_2.h"
#include "ppg_processing.h"
#include "shared_variables.h"
#include "signal_processing.h"
#include "storage.h"
#include <math.h>
#include <stdio.h>

#define STORAGE_BUFFER_SIZE 10
#define PRINT_EVERY_N_SAMPLE 10
#define WINDOW_STEP_SIZE_SEC 1
#define MOVEMENT_THRESHOLD 4.0f
#define MOTION_COOLDOWN_SAMPLES 300

static void send_ble_payload(uint16_t handle, void *data, uint16_t len);
static void collect_training_data(complete_log_t *log, uint16_t *buff_index);
static void store_training_data(uint16_t *buff_index);
static void fragment_ble_payloads(complete_log_t *log);
static void collect_data_final_log(complete_log_t *final_log, raw_data_t *new_samples, som_input_t *features,
                                   uint8_t result);
static bool detect_motion(bmi_data_t *sample);

static const char *TAG = "TASKS";
extern uint16_t ble_conn_handle;
extern ble_sensor_handles_t ble_val_handles;
extern bool is_sampling_active;
extern device_control_t device_config;
extern RingbufHandle_t raw_data_ringbuf;
extern QueueHandle_t data_log_queue;
extern QueueHandle_t telemetry_queue;
extern ble_payload_bulk_t ble_payloads_bulk[];
extern ble_payload_final_t ble_payload_final;
extern SemaphoreHandle_t ble_payload_mutex;
extern SemaphoreHandle_t experiment_phase_mutex;
extern volatile uint8_t current_experiment_phase;
extern som_input_transfer_learning_t transfer_learning_buffer[];
extern device_control_t device_config;
extern i2c_master_dev_handle_t bmi_handle;
extern i2c_master_dev_handle_t max_handle;
extern spi_device_handle_t gsr_handle;
extern bmi_data_t imu_data;
extern SemaphoreHandle_t imu_data_mutex;

void sensor_sampling_task(void *pvParameters)
{
    sensor_handles_t *sensors = (sensor_handles_t *)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PPG_AND_GSR_SAMPLING_RATE_IN_MS);
    static raw_data_t bundle[PPG_SAMPLE_RATE];
    int samples_collected = 0;
    static uint32_t motion_cooldown_counter = 0;

    ppg_processing_init();

    while (1) {
        uint8_t samples_available = 0;
        if (is_sampling_active) {

            raw_data_t current_sample = {0};

            if (max30101_get_fifo_count(*sensors->max_handle, &samples_available) == ESP_OK) {
                for (int i = 0; i < samples_available; i++) {
                    bool ppg_ok =
                        (max30101_read_fifo(*sensors->max_handle, &current_sample.ppg_raw) == ESP_OK);
                    bool gsr_ok = (gsr_sensor_read_raw(*sensors->gsr_handle, &current_sample.gsr) == ESP_OK);
                    bool imu_ok = (bmi260_read(bmi_handle, &current_sample.bmi_data) == ESP_OK);

                    if (ppg_ok && gsr_ok && imu_ok) {
                        bool motion_now = detect_motion(&current_sample.bmi_data);

                        if (motion_now) {
                            motion_cooldown_counter = MOTION_COOLDOWN_SAMPLES;
                        }

                        if (motion_cooldown_counter > 0) {
                            current_sample.has_movement_artifact = true;
                            current_sample.ppg_filtered = 500.0f;
                            motion_cooldown_counter--;
                        } else {
                            current_sample.has_movement_artifact = false;
                            current_sample.ppg_filtered =
                                ppg_filter_process(current_sample.ppg_raw) * (-1.0f);
                        }

                        current_sample.time_stamp = esp_timer_get_time();
                        bundle[samples_collected++] = current_sample;

                        if (device_config.show_telemetry && samples_collected % PRINT_EVERY_N_SAMPLE == 0) {
                            if (xQueueSend(telemetry_queue, &current_sample, 0) != pdTRUE) {
                                ESP_LOGE(TAG, "show_telemetry - Failed to send to queue!");
                            }
                        }
                    } else {
                        ESP_LOGE(TAG, "Sampling - Failed to sample PPG & GSR!");
                    }
                }
            }
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

    som_input_t features = {0};
    uint8_t result = 255;
    while (1) {
        size_t item_size;
        static uint32_t seconds_of_samples_collected = 0;

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

            // Extract features every 15 seconds
            // Run inference every 15 seconds
            seconds_of_samples_collected++;

            if ((seconds_of_samples_collected % WINDOW_STEP_SIZE_SEC == 0)) {
                features = calculate_features(history, WINDOW_SIZE);
                // Inference using SOM model. Outputs class as single digit
                result = classify_stress(&features);
            }
            // 0 = Neutral
            // 1 = Stress
            // 2 = Rest

            complete_log_t *final_log =
                (complete_log_t *)heap_caps_malloc(sizeof(complete_log_t), MALLOC_CAP_SPIRAM);
            if (final_log) {
                collect_data_final_log(final_log, new_samples, &features, result);

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
    static uint16_t buffer_index = 0;

    while (1) {
        if (xQueueReceive(data_log_queue, &received_log, portMAX_DELAY) == pdTRUE) {
            if (device_config.show_logged_values) {
                ESP_LOGI(TAG,
                         "t:%8lu ppg:%8lu ppgf:%3.1f gsr:%8u hr: %3.1f rmssd:%3.2f sdnn:%3.2f ton:%3.2f "
                         "phas: %3.2f "
                         "Str.cl:%3u Ex.ph:%3u",
                         received_log->timestamp, received_log->raw_samples[0].ppg_raw,
                         received_log->raw_samples[0].ppg_filtered, received_log->raw_samples[0].gsr,
                         received_log->features.hr, received_log->features.hrv_rmssd,
                         received_log->features.hrv_sdnn, received_log->features.tonic,
                         received_log->features.phasic, received_log->stress_class,
                         received_log->experiment_phase);
            }

            if (xSemaphoreTake(ble_payload_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                fragment_ble_payloads(received_log);
                xSemaphoreGive(ble_payload_mutex);
                collect_training_data(received_log, &buffer_index);
                heap_caps_free(received_log);

            } else {
                ESP_LOGW(TAG, "logging_task - Failed to take ble_payload semaphore, data lost!");
                heap_caps_free(received_log);
            }
            store_training_data(&buffer_index);
        }
    }
}

// Update BLE message buffer every BLE_NOTIFY_INTERVAL_MS, and notify connected phone.
// Task responsible for sending data every second over BLE
void ble_update_task(void *pvParameters)
{
    uint16_t handles[] = {
        ble_val_handles.ble_sensor_chr_a_val_handle, ble_val_handles.ble_sensor_chr_b_val_handle,
        ble_val_handles.ble_sensor_chr_c_val_handle, ble_val_handles.ble_sensor_chr_d_val_handle,
        ble_val_handles.ble_sensor_chr_e_val_handle, ble_val_handles.ble_sensor_chr_f_val_handle,
        ble_val_handles.ble_sensor_chr_g_val_handle, ble_val_handles.ble_sensor_chr_h_val_handle,
        ble_val_handles.ble_sensor_chr_i_val_handle};
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BLE_NOTIFY_INTERVAL_MS));

        // Only send if a phone is connected, and sampling is enabled
        if (is_sampling_active && ble_conn_handle != BLE_HS_CONN_HANDLE_NONE) {

            // Is ble_sensor_payload free from producers?
            if (xSemaphoreTake(ble_payload_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

                // Send Complete log 2042B. Split into 400B payloads
                for (int i = 0; i < BLE_NUM_OF_BULK_PAYLOADS; i++) {
                    send_ble_payload(handles[i], &ble_payloads_bulk[i], sizeof(ble_payload_bulk_t));
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                send_ble_payload(handles[BLE_NUM_OF_BULK_PAYLOADS], &ble_payload_final,
                                 sizeof(ble_payload_final_t));

                xSemaphoreGive(ble_payload_mutex);
            } else {
                ESP_LOGW(TAG, "ble_update_task - Failed to take ble_payload semaphore!");
            }
        }
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
        vTaskDelay(pdTICKS_TO_MS(BATTERY_SAMPLING_INTERVAL_MS));
        esp_err_t ret = read_battery_voltage(&adc1_handle, &adc1_cali_chan0_handle, &adc_raw, &voltage_mV);
        if (ret == ESP_OK) {
            log_battery_voltage(&adc_raw, &voltage_mV);
        } else {
            ESP_LOGE(TAG, "Failed to read battery Voltage!");
        }
    }
}

static void fragment_ble_payloads(complete_log_t *log)
{
    uint32_t sync_time = log->timestamp;

    // Filling 8 bulk payloads
    for (int i = 0; i < BLE_NUM_OF_BULK_PAYLOADS; i++) {
        ble_payloads_bulk[i].timestamp = sync_time;
        memcpy(ble_payloads_bulk[i].raw_samples, &log->raw_samples[i * BLE_NUM_OF_SAMPLES_PER_PAYLOAD],
               sizeof(raw_data_t) * BLE_NUM_OF_SAMPLES_PER_PAYLOAD);
    }

    // Filling Final
    ble_payload_final.timestamp = sync_time;
    memcpy(ble_payload_final.raw_samples,
           &log->raw_samples[BLE_NUM_OF_SAMPLES_PER_PAYLOAD * BLE_NUM_OF_BULK_PAYLOADS],
           sizeof(raw_data_t) * 8);

    ble_payload_final.hr = log->features.hr;
    ble_payload_final.rmssd = log->features.hrv_rmssd;
    ble_payload_final.sdnn = log->features.hrv_sdnn;
    ble_payload_final.scr = log->features.scr;
    ble_payload_final.tonic = log->features.tonic;
    ble_payload_final.phasic = log->features.phasic;
    ble_payload_final.stress_class = log->stress_class;
    ble_payload_final.experiment_phase = log->experiment_phase;
}

static void send_ble_payload(uint16_t handle, void *data, uint16_t len)
{
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) {
        int rc = ble_gatts_notify_custom(ble_conn_handle, handle, om);
        if (rc != 0) {
            ESP_LOGE(TAG, "send_ble_payload - Notification failed!");
        }
    }
}

static void collect_training_data(complete_log_t *log, uint16_t *buff_index)
{
    transfer_learning_buffer[*buff_index].features = log->features;
    transfer_learning_buffer[*buff_index].experiment_phase = log->experiment_phase;

    if (*buff_index < STORAGE_BUFFER_SIZE) {
        (*buff_index)++;
    } else {
        ESP_LOGW(TAG, "Buffer overflowing! Data lost!");
        *buff_index = 0;
    }
}

static void store_training_data(uint16_t *buff_index)
{
    if (*buff_index >= STORAGE_BUFFER_SIZE) {
        FILE *f = fopen(LOG_FILE_PATH, "ab");
        if (f == NULL) {
            ESP_LOGE(TAG, "logging_task - Failed to open file for appending!");
        } else {
            fwrite(transfer_learning_buffer, sizeof(som_input_transfer_learning_t), STORAGE_BUFFER_SIZE, f);
            fclose(f);
            *buff_index = 0;

            if (device_config.show_spiff_status) {
                ESP_LOGI(TAG, "Saved %d samples to SPIFFS", STORAGE_BUFFER_SIZE);
                check_spiffs_status(PARTITION_NAME);
            }
        }
    }
}

static void collect_data_final_log(complete_log_t *final_log, raw_data_t *new_samples, som_input_t *features,
                                   uint8_t result)
{
    memcpy(final_log->raw_samples, new_samples, PPG_SAMPLE_RATE * sizeof(raw_data_t));
    final_log->features = *features;
    final_log->stress_class = result;
    final_log->timestamp = xTaskGetTickCount();
    if (xSemaphoreTake(experiment_phase_mutex, pdMS_TO_TICKS(10))) {
        final_log->experiment_phase = current_experiment_phase;
        xSemaphoreGive(experiment_phase_mutex);
    } else {
        ESP_LOGW(TAG, "feature_extraction_task - Failed to take semaphore. Exp. Phase not set!");
    }
}

static bool detect_motion(bmi_data_t *sample)
{
    const float GRAVITY_LSB = 2050.0f;

    float fx = (float)sample->acc_x / GRAVITY_LSB;
    float fy = (float)sample->acc_y / GRAVITY_LSB;
    float fz = (float)sample->acc_z / GRAVITY_LSB;

    float gx = (float)sample->gyr_x / 16.4f;
    float gy = (float)sample->gyr_y / 16.4f;
    float gz = (float)sample->gyr_z / 16.4f;

    float gyro_mag = sqrtf(gx * gx + gy * gy + gz * gz);
    float total_mag = sqrtf(fx * fx + fy * fy + fz * fz);
    float movement_intensity = fabsf(total_mag - 1.0f);
    float total_movement_score = (movement_intensity * 100.0f) + (gyro_mag * 0.5f);

    if (total_movement_score > MOVEMENT_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}

void telemetry_task(void *pvParameters)
{
    raw_data_t sample;
    while (1) {
        if (xQueueReceive(telemetry_queue, &sample, portMAX_DELAY)) {
            printf(">Raw:%lu\n", sample.ppg_raw);
            printf(">Filt:%.2f\n", sample.ppg_filtered);
            printf(">Movement:%d\n", sample.has_movement_artifact);
        }
    }
}