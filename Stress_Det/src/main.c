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
extern QueueHandle_t telemetry_queue;
extern RingbufHandle_t raw_data_ringbuf;
extern SemaphoreHandle_t ble_payload_mutex;
extern SemaphoreHandle_t experiment_phase_mutex;
extern device_control_t device_config;

void app_main(void)
{
    ESP_ERROR_CHECK(init_raw_data_ring_buffer(&raw_data_ringbuf));
    ESP_ERROR_CHECK(init_storage_transfer_learning());

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

    telemetry_queue = xQueueCreate(20, sizeof(raw_data_t));
    if (telemetry_queue == NULL) {
        ESP_LOGE(TAG, "telemetry_queue, failed to create Queue!");
        return;
    }

    // Initialize I2C-bus & SPI-bus
    i2c_master_bus_handle_t bus_handle;
    if (device_config.enable_ppg)
        ESP_ERROR_CHECK(init_i2c(&bus_handle));
    if (device_config.enable_gsr)
        ESP_ERROR_CHECK(init_spi());

    static i2c_master_dev_handle_t tmp_handle, max_handle, bmi_handle;
    static spi_device_handle_t gsr_handle;
    static sensor_handles_t sensor_handles;
    sensor_handles.gsr_handle = &gsr_handle;
    sensor_handles.max_handle = &max_handle;

    // Initialize I2C-sensors
    if (device_config.enable_temp)
        ESP_ERROR_CHECK(tmp117_init(bus_handle, &tmp_handle));
    if (device_config.enable_ppg)
        ESP_ERROR_CHECK(max30101_init(bus_handle, &max_handle));
    if (device_config.enable_imu)
        ESP_ERROR_CHECK(bmi260_init(bus_handle, &bmi_handle));

    // Initialize SPI-sensor
    if (device_config.enable_gsr)
        ESP_ERROR_CHECK(gsr_sensor_init(&gsr_handle));

    ESP_LOGI(TAG, "All sensors initialized.");

    // BLE
    init_ble_server();
    xTaskCreatePinnedToCore(ble_update_task, "ble_upd", 4 * 1024, NULL, 5, NULL, 0);

    // Pipeline Tasks
    xTaskCreatePinnedToCore(sensor_sampling_task, "sampl", 8 * 1024, &sensor_handles, 10, NULL, 1);
    xTaskCreatePinnedToCore(feature_extraction_task, "feats", 4 * 1024, NULL, 9, NULL, 1);
    xTaskCreatePinnedToCore(logging_task, "log", 8 * 1024, NULL, 6, NULL, 1);

    xTaskCreatePinnedToCore(telemetry_task, "telem", 4 * 1024, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(battery_task, "battery", 4096, NULL, 1, NULL, 1);

    return;
}
