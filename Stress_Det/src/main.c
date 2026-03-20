#include "ble_server.h"
#include "bmi260.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
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
static const char *TAG = "MAIN";

#define STORAGE_QUEUE_LENGTH 40

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(4000));

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t tmp_handle, max_handle, bmi_handle;
    spi_device_handle_t gsr_handle;
    // Initialize I2C-bus & SPI-bus
    ESP_ERROR_CHECK(init_i2c(&bus_handle));
    ESP_ERROR_CHECK(init_spi());

    // Initialize I2C-sensors
    ESP_ERROR_CHECK(tmp117_init(bus_handle, &tmp_handle));
    ESP_ERROR_CHECK(max30101_init(bus_handle, &max_handle));
    ESP_ERROR_CHECK(bmi260_init(bus_handle, &bmi_handle));

    // Initialize SPI-sensor
    ESP_ERROR_CHECK(gsr_sensor_init(&gsr_handle));

    ESP_LOGI(TAG, "All sensors initialized.");

    // Semaphore's job is to prevent multiple processes to read/write to ble_payload struct
    // at the same time.
    sensor_data_mutex = xSemaphoreCreateMutex();
    storage_queue =
        xQueueCreate(STORAGE_QUEUE_LENGTH,
                     sizeof(sensor_data_t)); // Queue can hold STORAGE_QUEUE_LENGTH sensor_data_t structs

    init_ble_server();
    ESP_ERROR_CHECK(init_psram_buffer(
        &sensor_log,
        DATA_COLLECTION_SAMPLES_COUNT)); // Allocate the large PSRAM buffer for data collection

    create_tasks(tmp_handle, max_handle, bmi_handle, gsr_handle);

    return;
}
