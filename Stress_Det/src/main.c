#include "adc.h"
#include "ble_server.h"
#include "bmi260.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "gsr.h"
#include "host/ble_hs.h"
#include "i2c_common.h"
#include "max30101.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
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
extern bool enable_ppg;
extern bool enable_gsr;
extern bool enable_imu;
extern bool enable_temp;

void app_main(void)
{
    ESP_ERROR_CHECK(init_raw_data_ring_buffer(&raw_data_ringbuf));

    ble_payload_mutex = xSemaphoreCreateMutex();
    if (ble_payload_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create Mutex!");
        return;
    }

    data_log_queue = xQueueCreate(5, sizeof(complete_log_t *));
    if (data_log_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create Queue!");
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

    adc_oneshot_unit_handle_t adc1_handle = NULL;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    ESP_ERROR_CHECK(init_adc(&adc1_handle, &adc1_cali_chan0_handle));
    static int adc_raw;
    static int voltage;

    while (1) {
        esp_err_t ret = read_battery_voltage(&adc1_handle, &adc1_cali_chan0_handle, &adc_raw, &voltage);
        vTaskDelay(pdTICKS_TO_MS(500));
    }
    return;
}
