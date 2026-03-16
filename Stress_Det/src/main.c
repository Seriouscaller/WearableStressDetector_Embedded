#include "ble_server.h"
#include "bmi160.h"
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
#include "sensor_tasks.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "spi_common.h"
#include "storage.h"
#include "tmp117.h"
#include "types.h"
#include <stdio.h>
#include <string.h>

uint16_t conn_handle;
uint16_t sensor_chr_val_handle;
SemaphoreHandle_t sensor_data_mutex;
static const char *TAG = "MAIN";
sensor_data_t ble_sensor_payload;
QueueHandle_t storage_queue; // Queue for storing sensor data before flash write

i2c_master_dev_handle_t add_tmp117_i2c(i2c_master_bus_handle_t bus_handle)
{
    i2c_master_dev_handle_t tmp_handle;
    ESP_ERROR_CHECK(tmp117_init(bus_handle, &tmp_handle));
    return tmp_handle;
}

i2c_master_dev_handle_t add_max30101_i2c(i2c_master_bus_handle_t bus_handle)
{
    i2c_master_dev_handle_t max_handle;
    ESP_ERROR_CHECK(max30101_init(bus_handle, &max_handle));
    return max_handle;
}

i2c_master_dev_handle_t add_bmi160_i2c(i2c_master_bus_handle_t bus_handle)
{
    i2c_master_dev_handle_t bmi_handle;
    ESP_ERROR_CHECK(bmi160_init(bus_handle, &bmi_handle));
    return bmi_handle;
}

spi_device_handle_t add_gsr_spi()
{
    spi_device_handle_t gsr_handle;
    ESP_ERROR_CHECK(gsr_sensor_init(&gsr_handle));
    return gsr_handle;
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(4000));

    // Initialize I2C-bus and I2C-sensors
    i2c_master_bus_handle_t bus_handle;
    init_i2c(&bus_handle);

    i2c_master_dev_handle_t bmi260_handle;
    ESP_ERROR_CHECK(
        bmi260_init(bus_handle, &bmi260_handle)); // BMI260 must be initialized before adding devices to bus
    /*
    i2c_master_dev_handle_t tmp_handle = add_tmp117_i2c(bus_handle);
    i2c_master_dev_handle_t max_handle = add_max30101_i2c(bus_handle);
    i2c_master_dev_handle_t bmi_handle = add_bmi160_i2c(bus_handle);
    */
    /*
    // Initialize SPI-bus and SPI-sensor
    ESP_ERROR_CHECK(init_spi());
    spi_device_handle_t gsr_handle = add_gsr_spi();*/

    ESP_LOGI(TAG, "All sensors initialized.");

    // Semaphore's job is to prevent multiple processes to read/write to ble_payload struct
    // at the same time.
    sensor_data_mutex = xSemaphoreCreateMutex();
    storage_queue = xQueueCreate(20, sizeof(sensor_data_t)); // Queue can hold 20 sensor_data_t structs

    init_ble_server();
    init_psram_buffer(); // Allocate the large PSRAM buffer for logging sensor data

    // Each task is pinned to core 1 to avoid conflicts with BLE stack on core 0.
    // Task priorities are set based on sensor read frequency and importance.
    /*
    // High-speed IMU Task (100Hz
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, bmi_handle, 10, NULL, 1);
    // Slow Temperature Task (1Hz)
    xTaskCreatePinnedToCore(temp_task, "temp_task", 2048, tmp_handle, 2, NULL, 1);
    // Heart Rate Task (50Hz)
    xTaskCreatePinnedToCore(ppg_task, "ppg_task", 4096, max_handle, 9, NULL, 1);
    // GSR Task (10Hz)
    xTaskCreatePinnedToCore(gsr_task, "gsr_task", 4096, gsr_handle, 8, NULL, 1);
    // Send struct via BLE (100 ms)
    xTaskCreatePinnedToCore(ble_update_task, "ble_update_task", 4096, NULL, 4, NULL, 1);
    // Storage task that copies snapshot to flash
    xTaskCreatePinnedToCore(storage_task, "storage_task", 4096, NULL, 3, NULL, 1);
    // Sync task that takes consistent snapshots of BLE_payload and sends to storage queue
    xTaskCreatePinnedToCore(sync_heartbeat_task, "sync_task", 4096, NULL, 5, NULL, 1);
    // Print buffer status every 5 seconds
    xTaskCreatePinnedToCore(print_buffer_status_task, "prt_bufr_status_tsk", 4096, NULL, 1, NULL, 1);
    */
    bmi_data_t bmi_data;

    while (1) {
        bmi260_read(bmi260_handle, &bmi_data);
        ESP_LOGI(TAG, "BMI260 Data - Accel: (%d, %d, %d) | Gyro: (%d, %d, %d)", bmi_data.acc_x,
                 bmi_data.acc_y, bmi_data.acc_z, bmi_data.gyr_x, bmi_data.gyr_y, bmi_data.gyr_z);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return;
}
