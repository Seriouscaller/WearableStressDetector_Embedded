#include "adc.h"
#include "bmi260.h"
#include "board_config.h"
#include "eda_clean.h"
#include "eda_filter.h"
#include "eda_peaks.h"
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
#include "tmp117.h"
#include <math.h>
#include <stdio.h>

#define STORAGE_BUFFER_SIZE 10
#define PRINT_EVERY_N_SAMPLE 10
#define WINDOW_STEP_SIZE_SEC 15
#define FEATURE_EXTRATION_INTERVAL 2
#define MOVEMENT_THRESHOLD 8.0f
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
extern i2c_master_dev_handle_t tmp_handle;
extern spi_device_handle_t gsr_handle;
extern bmi_data_t imu_data;
extern float battery_percentage;
extern float temperature;
extern uint16_t num_of_classifications;

/**
 * @brief  High-priority task for synchronized multi-sensor sampling.
 *
 * This task runs at a fixed frequency (typically 200Hz) and performs:
 * 1. **Data Acquisition**: Reads the MAX30101 (PPG), CJMCU 6701 (GSR), and BMI260 (IMU).
 * 2. **Motion Guarding**: Implements a 'cooldown' period where data is marked invalid
 *    immediately following detected movement to prevent filter ringing/false peaks.
 * 3. **Signal Pre-processing**: Applies Biquad filtering to PPG and scaling to GSR.
 * 4. **Batching**: Bundles samples into 1-second chunks (200 samples) before pushing
 *    to the PSRAM Ringbuffer to minimize context-switching overhead.
 *
 * @param[in] pvParameters Pointer to sensor_handles_t containing peripheral handles.
 */
void sensor_sampling_task(void *pvParameters)
{
    sensor_handles_t *sensors = (sensor_handles_t *)pvParameters;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PPG_AND_GSR_SAMPLING_RATE_IN_MS);
    static raw_data_t bundle[PPG_SAMPLE_RATE];
    int samples_collected = 0;
    static uint32_t motion_cooldown_counter = 0;

    ppg_processing_init();
    eda_filter_init();
    eda_peaks_init(200.0f);

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

                        current_sample.gsr_scaled = ((float)current_sample.gsr / 4095.0f) * 3.3f;

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

/**
 * @brief  Consumer task for feature extraction and stress classification.
 *
 * This task manages a sliding temporal window of physiological data. It:
 * 1. **Data Management**: Maintains a 'history' buffer in PSRAM, shifting out the
 *    oldest 1-second block to make room for new samples from the Ringbuffer.
 * 2. **Feature Extraction**: Periodically calculates HR, HRV, and GSR features
 *    based on the FEATURE_EXTRATION_INTERVAL.
 * 3. **Machine Learning**: Runs the SOM-based 'classify_stress' algorithm to
 *    determine the user's mental state (Neutral, Stress, or Rest).
 * 4. **Telemetry & Logging**: Aggregates raw data, features, and ML results into
 *    a 'complete_log_t' and queues it for NVS storage or BLE transmission.
 *
 * @param[in] pvParameters Unused task parameters.
 */
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

            if ((seconds_of_samples_collected % FEATURE_EXTRATION_INTERVAL == 0)) {
                features = calculate_features(history, WINDOW_SIZE);
            }

            if ((seconds_of_samples_collected % WINDOW_STEP_SIZE_SEC == 0)) {
                result = classify_stress(&features);
                num_of_classifications++;
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

/**
 * @brief Final processing stage for logged data, BLE transmission, and persistence.
 *
 * This task consumes 'complete_log_t' objects from the data_log_queue. For each log, it:
 * 1. **Telemtry**: Optionally prints human-readable sensor stats to the serial console.
 * 2. **BLE Dispatch**: Fragments and prepares data for NimBLE characteristics using a mutex.
 * 3. **Persistence**: Buffers and saves data for transfer learning via SPIFFS.
 * 4. **Memory Management**: Frees the PSRAM-allocated log pointer to prevent leaks.
 *
 * @param[in] pvParameters Unused task parameters.
 */
void logging_task(void *pvParameters)
{
    complete_log_t *received_log;
    static uint16_t buffer_index = 0;

    while (1) {
        if (xQueueReceive(data_log_queue, &received_log, portMAX_DELAY) == pdTRUE) {
            if (device_config.show_logged_values) {
                ESP_LOGI(TAG,
                         "t:%8lu ppg:%8lu ppgf:%3.1f gsr:%8u hr: %3.1f rmssd:%3.2f sc_ph:%5.4f "
                         "sc_rr: %5.4f Str.cl:%3u Ex.ph:%3u",
                         received_log->timestamp, received_log->raw_samples[0].ppg_raw,
                         received_log->raw_samples[0].ppg_filtered, received_log->raw_samples[0].gsr,
                         received_log->features.hr, received_log->features.hrv_rmssd,
                         received_log->features.sc_ph, received_log->features.sc_rr,
                         received_log->stress_class, received_log->experiment_phase);
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

/**
 * @brief  Task responsible for broadcasting sensor data via BLE notifications.
 *
 * Periodically transmits batched sensor payloads and final inference results to
 * the connected BLE client. It uses a series of characteristic handles to
 * distribute bulk data and high-level features (BPM, RMSSD, Stress Class).
 *
 * @param[in] pvParameters Unused task parameters.
 *
 * @note Implements a brief 10ms delay between characteristic updates to prevent
 *       overwhelming the BLE controller's buffer, which is critical for
 *       maintaining connection stability on the ESP32-S3.
 */
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

/**
 * @brief  Periodic task for battery voltage monitoring and fuel gauging.
 *
 * This task initializes the ADC (Analog-to-Digital Converter) using the
 * ESP-IDF oneshot unit and applies factory calibration to ensure millivolt
 * accuracy. It periodically samples the battery rail, calculates the
 * discharge state, and updates the global battery_percentage.
 *
 * @param[in] pvParameters Unused task parameters.
 *
 * @note On the XIAO S3, the battery voltage is typically tied to GPIO 1
 *       through a voltage divider. Accurate readings require the ADC
 *       calibration handles to account for Vref drift.
 */
void battery_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    ESP_ERROR_CHECK(init_adc(&adc1_handle, &adc1_cali_chan0_handle));
    static int adc_raw;
    static int voltage_mV;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BATTERY_SAMPLING_INTERVAL_MS));
        esp_err_t ret = read_battery_voltage(&adc1_handle, &adc1_cali_chan0_handle, &adc_raw, &voltage_mV);
        if (ret == ESP_OK) {
            log_battery_voltage(&adc_raw, &voltage_mV);
            printf(">Battery Charge:%.0f\n", battery_percentage);
        } else {
            ESP_LOGE(TAG, "Failed to read battery Voltage!");
        }
    }
}

/**
 * @brief Slices a complete log into fragmented BLE-ready payloads.
 *
 * This function performs the "Fragmentation" part of the data pipeline. Since a
 * full 1-second log exceeds the maximum BLE MTU, it is subdivided into 8 bulk
 * packets (containing raw waveforms) and one final packet (containing
 * summary statistics and SOM results).
 *
 * All fragments are timestamp-synchronized to allow the client-side app
 * to reconstruct the continuous signal timeline.
 *
 * @param[in] log Pointer to the PSRAM-allocated log structure to be fragmented.
 *
 * @note This function must be called while holding 'ble_payload_mutex' to
 *       ensure thread safety against the NimBLE transmission task.
 */
static void fragment_ble_payloads(complete_log_t *log)
{
    uint32_t sync_time = log->timestamp;

    // Filling 8 bulk payloads
    for (int i = 0; i < BLE_NUM_OF_BULK_PAYLOADS; i++) {
        ble_payloads_bulk[i].timestamp = sync_time;
        memcpy(ble_payloads_bulk[i].raw_samples, &log->raw_samples[i * BLE_NUM_OF_SAMPLES_PER_PAYLOAD],
               sizeof(raw_log_data_t) * BLE_NUM_OF_SAMPLES_PER_PAYLOAD);
    }

    // Filling Final
    ble_payload_final.timestamp = sync_time;
    memcpy(ble_payload_final.raw_samples,
           &log->raw_samples[BLE_NUM_OF_SAMPLES_PER_PAYLOAD * BLE_NUM_OF_BULK_PAYLOADS],
           sizeof(raw_log_data_t) * 8);

    ble_payload_final.hr = log->features.hr;
    ble_payload_final.rmssd = log->features.hrv_rmssd;
    ble_payload_final.sc_ph = log->features.sc_ph;
    ble_payload_final.sc_rr = log->features.sc_rr;
    ble_payload_final.stress_class = log->stress_class;
    ble_payload_final.num_of_classifications = log->num_of_classifications;
    ble_payload_final.experiment_phase = log->experiment_phase;
}

/**
 * @brief  Encapsulates and transmits data via a BLE GATT Notification.
 *
 * This function handles the low-level NimBLE buffer allocation and dispatch.
 * It converts a contiguous memory block into an 'os_mbuf' chain, which is the
 * internal memory management structure used by the BLE host to handle
 * asynchronous radio transmissions.
 *
 * @param[in] handle The GATT characteristic value handle to notify.
 * @param[in] data   Pointer to the flat data structure (e.g., ble_payload_bulk_t).
 * @param[in] len    Length of the data in bytes.
 *
 * @note If 'ble_gatts_notify_custom' returns a non-zero value, it typically
 *       indicates a 'BLE_HS_ENOMEM' error, suggesting the transmit buffers
 *       are full.
 */
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

/**
 * @brief  Caches processed feature vectors into the Transfer Learning buffer.
 *
 * This function acts as an intermediate staging area. It extracts the computed
 * features (HR, HRV, GSR Phasic, etc.) and the current experimental phase
 * from a 'complete_log_t' and stores them in the 'transfer_learning_buffer'.
 *
 * @param[in]     log         Pointer to the current processed log entry.
 * @param[in,out] buff_index  Pointer to the current write index of the
 *                            transfer_learning_buffer.
 *
 * @note This function manages a linear buffer. If the buffer reaches
 *       STORAGE_BUFFER_SIZE, it resets the index to 0 to prevent memory
 *       corruption, though this results in the oldest data being overwritten.
 */
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

/**
 * @brief  Commits batched training data from PSRAM to the SPIFFS filesystem.
 *
 * When the local cache reaches 'STORAGE_BUFFER_SIZE', this function opens the
 * persistence file in append-binary ("ab") mode. It flushes the accumulated
 * 'som_input_transfer_learning_t' structs to flash, ensuring that the user's
 * physiological baseline data survives a power cycle or battery depletion.
 *
 * @param[in,out] buff_index Pointer to the current buffer count. Reset to 0
 *                            upon a successful write.
 *
 * @note This operation is a synchronous blocking write. On the ESP32-S3,
 *       SPIFFS writes can take several milliseconds; ensure this is called
 *       from a low-priority task (like logging_task) to avoid jitter in
 *       sensor sampling.
 */
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

/**
 * @brief  Aggregates raw samples, computed features, and ML results into a final log.
 *
 * This function assembles a 'complete_log_t' structure by copying raw signal data
 * from the sampling buffer and attaching the high-level features and stress
 * classification result. It also synchronizes the current experimental phase
 * using a mutex to ensure data labeling accuracy for transfer learning.
 *
 * @param[out] final_log  Pointer to the log structure in PSRAM to be populated.
 * @param[in]  new_samples Array of 200 raw data points (1 second of activity).
 * @param[in]  features    Pointer to the extracted feature vector (HR, RMSSD, GSR metrics).
 * @param[in]  result      The classification output from the SOM (Neutral, Stress, Rest).
 */
static void collect_data_final_log(complete_log_t *final_log, raw_data_t *new_samples, som_input_t *features,
                                   uint8_t result)
{
    for (int i = 0; i < PPG_SAMPLE_RATE; i++) {
        final_log->raw_samples[i].time_stamp = new_samples[i].time_stamp;
        final_log->raw_samples[i].ppg_raw = new_samples[i].ppg_raw;
        final_log->raw_samples[i].ppg_filtered = new_samples[i].ppg_filtered;
        final_log->raw_samples[i].gsr = new_samples[i].gsr;
    }
    final_log->features = *features;
    final_log->stress_class = result;
    final_log->num_of_classifications = num_of_classifications;
    final_log->timestamp = xTaskGetTickCount();

    if (xSemaphoreTake(experiment_phase_mutex, pdMS_TO_TICKS(10))) {
        final_log->experiment_phase = current_experiment_phase;
        xSemaphoreGive(experiment_phase_mutex);
    } else {
        ESP_LOGW(TAG, "feature_extraction_task - Failed to take semaphore. Exp. Phase not set!");
    }
}

/**
 * @brief  Analyzes IMU data to detect physical motion artifacts.
 *
 * Calculates a combined movement score using both accelerometer and gyroscope
 * data from the BMI260. It uses vector magnitude to determine if the
 * current sample is compromised by physical activity.
 *
 * @param[in] sample Pointer to the raw BMI260 data struct.
 *
 * @return
 *      - true:  Motion detected (signal likely corrupted).
 *      - false: Device is stable (signal clean).
 *
 * @note The 'movement_intensity' is derived by calculating the deviation
 *       from 1.0g (static gravity), effectively creating a high-pass filter
 *       for linear acceleration.
 */
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

/**
 * @brief  Low-priority task for serial telemetry and signal visualization.
 *
 * Consumes sampled data from the 'telemetry_queue' and outputs it to the UART
 * console using a format compatible with Serial Plotters (e.g., Teleplot).
 * This is used during the research phase to verify:
 * 1. PPG Filter performance (Raw vs. Filtered).
 * 2. Motion rejection logic (Movement flag status).
 * 3. GSR sensor scaling and responsiveness.
 *
 * @param[in] pvParameters Unused task parameters.
 *
 * @note This task should be disabled in production via 'device_config.show_telemetry'
 *       to save power and UART bandwidth.
 */
void telemetry_task(void *pvParameters)
{
    raw_data_t sample;
    while (1) {
        if (xQueueReceive(telemetry_queue, &sample, portMAX_DELAY)) {
            printf(">Raw:%lu\n", sample.ppg_raw);
            printf(">Filt:%.2f\n", sample.ppg_filtered);
            printf(">Movement:%d\n", sample.has_movement_artifact);
            printf(">GSR raw:%d\n", sample.gsr);
            printf(">GSR scaled:%.3f\n", sample.gsr_scaled);
            /*
            printf(">ax:%d\n", sample.bmi_data.acc_x);
            printf(">ay:%d\n", sample.bmi_data.acc_y);
            printf(">az:%d\n", sample.bmi_data.acc_z);

            printf(">gx:%d\n", sample.bmi_data.gyr_x);
            printf(">gy:%d\n", sample.bmi_data.gyr_y);
            printf(">gz:%d\n", sample.bmi_data.gyr_z);*/
        }
    }
}

/**
 * @brief  Periodic task for high-precision skin temperature monitoring.
 *
 * Samples the TMP117 sensor via I2C at a 1Hz frequency.
 *
 * @param[in] pvParameters Unused task parameters.
 *
 * @note The TMP117 is highly sensitive; ensure the sensor has good thermal
 *       contact with the skin and is thermally isolated from the ESP32-S3
 *       MCU heat to avoid biased readings.
 */
void temperature_task(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_err_t ret = tmp117_read_temp(tmp_handle, &temperature);
        if (ret == ESP_OK) {
            printf(">Skin Temperature:%.1f\n", temperature);

        } else {
            ESP_LOGW(TAG, "temperature_task - Failed to read temp-sensor");
        }
    }
}