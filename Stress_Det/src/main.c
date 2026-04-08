#include "ble_server.h"
#include "bmi260.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "eda_processing.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "gsr.h"
#include "host/ble_hs.h"
#include "i2c_common.h"
#include "max30101.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "ppg_processing.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "signal_processing.h"
#include "spi_common.h"
#include "storage.h"
#include "tasks.h"
#include "tmp117.h"
#include "types.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "MAIN";
extern QueueHandle_t data_log_queue;
extern RingbufHandle_t raw_data_ringbuf;
extern SemaphoreHandle_t ble_payload_mutex;
extern SemaphoreHandle_t experiment_phase_mutex;

extern bool enable_ppg;
extern bool enable_gsr;
extern bool enable_imu;
extern bool enable_temp;

void app_main(void)
{
    ESP_ERROR_CHECK(init_raw_data_ring_buffer(&raw_data_ringbuf));

    ble_payload_mutex = xSemaphoreCreateMutex();
    if (ble_payload_mutex == NULL) {
        ESP_LOGE(TAG, "ble_payload_mutex, failed to create Mutex!");
        return;
        // Helper function to get current time in seconds for EDA processing
        float get_time_seconds()
        {
            return (float)esp_timer_get_time() / 1000000.0f;
        }

        void display_raw_ppg(i2c_master_dev_handle_t max_handle, uint32_t *ppg_green)
        {
            if (max30101_read_fifo(max_handle, ppg_green) == ESP_OK) {

                float raw = (float)(*ppg_green);

                // pipeline
                ppg_process_sample(raw);

                // Hämta debug-data
                float filtered = ppg_get_filtered();
                int peak = ppg_get_peak();
                float hr = ppg_get_hr();
                float hr_smooth = ppg_get_hr();

                // TELEPLOT
                printf(">PPG_filt:%f\n", filtered);
                printf(">PEAK:%f\n", peak ? filtered : 0.0f);
                printf(">HR:%f\n", hr);
                printf(">HR_SMOOTH:%f\n", hr_smooth);
                printf(">PPG_raw:%f\n", raw);
            } else {
                ESP_LOGW(TAG, "Failed to read MAX30101 sensor.");
            }
        }

        void display_raw_gsr(spi_device_handle_t gsr_handle, uint16_t *raw_gsr)
        {
            if (gsr_sensor_read_raw(gsr_handle, raw_gsr) == ESP_OK) {
                float adc = (float)(*raw_gsr);

                // convert ADC value to skin resistance and then to conductance for EDA processing
                float v_ref = 3.3f;
                float r_fixed = 10000.0f;

                float voltage = (adc / 4095.0f) * v_ref;

                if (voltage < 0.001f)
                    voltage = 0.001f; // Prevent division by zero or negative resistance

                float r_skin = (v_ref * r_fixed / voltage) - r_fixed;

                // conductance (µS)
                float conductance = (1.0f / r_skin) * 1000000.0f;

                // time in seconds for EDA processing
                float time = get_time_seconds();

                eda_process_sample(conductance, time);

                // DEBUG raw
                printf(">EDA_raw:%f\n", conductance);

                // Features
                eda_features_t e = eda_get_features();

                printf(">EDA_tonic:%f\n", e.tonic);
                printf(">EDA_phasic:%f\n", e.phasic);
                printf(">SCR:%d\n", e.scr_count);
            } else {
                ESP_LOGW(TAG, "Failed to read GSR sensor.");
            }
        }

        void app_main(void)
        {
            ESP_ERROR_CHECK(init_raw_data_ring_buffer(&raw_data_ringbuf));

            ble_payload_mutex = xSemaphoreCreateMutex();
            if (ble_payload_mutex == NULL) {
                ESP_LOGE(TAG, "ble_payload_mutex, failed to create Mutex!");
                return;
            }

            experiment_phase_mutex = xSemaphoreCreateMutex();
            if (experiment_phase_mutex == NULL) {
                ESP_LOGE(TAG, "experiment_phase_mutex, failed to create Mutex!");
                return;
            }

            data_log_queue = xQueueCreate(5, sizeof(complete_log_t *));
            if (data_log_queue == NULL) {
                ESP_LOGE(TAG, "data_log_queue, failed to create Queue!");
                return;
            }

            // Initialize I2C-bus & SPI-bus
            i2c_master_bus_handle_t bus_handle;
            init_i2c(&bus_handle);
            i2c_master_dev_handle_t tmp_handle = add_tmp117_i2c(bus_handle);

            ESP_LOGI(TAG, "All sensors initialized.");

            // Semaphore's job is to prevent multiple processes to read/write to ble_payload struct
            // at the same time.
            sensor_data_mutex = xSemaphoreCreateMutex();

            init_ble_server();

            xTaskCreatePinnedToCore(sensor_task,        // Function to run (Writing to payload, (producer))
                                    "sensor_task",      // Name of task
                                    4096,               // Stack size
                                    (void *)tmp_handle, // Parameter
                                    5,                  // Priority (higher number = higher prio)
                                    NULL,               // Task handle
                                    1                   // Run by which core
            );

            while (1) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            experiment_phase_mutex = xSemaphoreCreateMutex();
            if (experiment_phase_mutex == NULL) {
                ESP_LOGE(TAG, "experiment_phase_mutex, failed to create Mutex!");
                return;
            }

            data_log_queue = xQueueCreate(5, sizeof(complete_log_t *));
            if (data_log_queue == NULL) {
                ESP_LOGE(TAG, "data_log_queue, failed to create Queue!");
                return;
            }

            // Initialize I2C-bus & SPI-bus
            i2c_master_bus_handle_t bus_handle;
            ESP_ERROR_CHECK(init_i2c(&bus_handle));
            ESP_ERROR_CHECK(init_spi());

            static i2c_master_dev_handle_t tmp_handle, max_handle, bmi_handle;
            static spi_device_handle_t gsr_handle;
            static sensor_handles_t sensor_handles;
            sensor_handles.gsr_handle = &gsr_handle;
            sensor_handles.max_handle = &max_handle;

            // Initialize I2C-sensors
            if (enable_temp)
                ESP_ERROR_CHECK(tmp117_init(bus_handle, &tmp_handle));
            if (enable_ppg)
                ESP_ERROR_CHECK(max30101_init(bus_handle, &max_handle));
            if (enable_imu)
                ESP_ERROR_CHECK(bmi260_init(bus_handle, &bmi_handle));

            // Initialize SPI-sensor
            if (enable_gsr)
                ESP_ERROR_CHECK(gsr_sensor_init(&gsr_handle));

            ESP_LOGI(TAG, "All sensors initialized.");

            // BLE
            init_ble_server();
            xTaskCreatePinnedToCore(ble_update_task, "ble_upd", 4096, NULL, 5, NULL, 0);

            // Pipeline Tasks
            xTaskCreatePinnedToCore(sensor_sampling_task, "sampl", 8192, &sensor_handles, 10, NULL, 1);
            xTaskCreatePinnedToCore(feature_extraction_task, "feats", 4096, NULL, 9, NULL, 1);
            xTaskCreatePinnedToCore(logging_task, "log", 8192, NULL, 6, NULL, 1);

            xTaskCreatePinnedToCore(battery_task, "battery", 4096, NULL, 1, NULL, 1);

            return;
        }
