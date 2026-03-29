#include "ble_server.h"
#include "bmi260.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/ringbuf.h"
#include "gsr.h"
#include "host/ble_hs.h"
#include "i2c_common.h"
#include "max30101.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "shared_variables.h"
#include "signal_processing.h"
#include "spi_common.h"
#include "storage.h"
#include "tasks.h"
#include "tmp117.h"
#include "types.h"
#include <stdio.h>
#include <string.h>

extern uint16_t conn_handle;
extern uint16_t sensor_chr_val_handle;
extern SemaphoreHandle_t sensor_data_mutex;
extern sensor_data_t ble_sensor_payload;
extern QueueHandle_t storage_queue; // Queue for storing sensor data before flash write
extern psram_ring_buffer_t sensor_log;
extern psram_window_ring_buffer_t sliding_window;
static const char *TAG = "MAIN";
extern bool enable_ppg;
extern bool enable_gsr;
extern bool enable_imu;
extern bool enable_temp;
extern QueueHandle_t data_log_queue;
extern RingbufHandle_t raw_data_ringbuf;

void app_main(void)
{
    // vTaskDelay(pdMS_TO_TICKS(4000));
    static sensor_handles_t sensor_handles;
    i2c_master_bus_handle_t bus_handle;
    static i2c_master_dev_handle_t tmp_handle, max_handle, bmi_handle;

    static spi_device_handle_t gsr_handle;
    sensor_handles.gsr_handle = &gsr_handle;
    sensor_handles.max_handle = &max_handle;
    // Initialize I2C-bus & SPI-bus
    ESP_ERROR_CHECK(init_i2c(&bus_handle));
    ESP_ERROR_CHECK(init_spi());

    raw_data_ringbuf = xRingbufferCreateWithCaps(RING_BUF_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    data_log_queue = xQueueCreate(5, sizeof(complete_log_t));

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

    // init_ble_server();
    xTaskCreatePinnedToCore(producer_task, "prod", 8192, &sensor_handles, 10, NULL, 1);
    xTaskCreatePinnedToCore(feature_extraction_task, "feat_extr", 4096, NULL, 9, NULL, 1);
    xTaskCreatePinnedToCore(logging_task, "log", 4096, NULL, 5, NULL, 1);
    return;
}
