#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "driver/i2c_master.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_server.h"
#include "types.h"
#include "i2c_common.h"
#include "spi_common.h"
#include "max30101.h"
#include "tmp117.h"
#include "gsr.h"
#include "bmi160.h"

uint16_t conn_handle;
uint16_t sensor_chr_val_handle;
SemaphoreHandle_t sensor_data_mutex;
static const char *TAG = "MAIN";
sensor_data_t ble_sensor_payload = { .company_id = 0x02E5 }; // Example ID

i2c_master_dev_handle_t add_tmp117_i2c(i2c_master_bus_handle_t bus_handle){
    i2c_master_dev_handle_t tmp_handle;
    ESP_ERROR_CHECK(tmp117_init(bus_handle, &tmp_handle));
    return tmp_handle;
}

i2c_master_dev_handle_t add_max30101_i2c(i2c_master_bus_handle_t bus_handle){
    i2c_master_dev_handle_t max_handle;
    ESP_ERROR_CHECK(max30101_init(bus_handle, &max_handle));
    return max_handle;
}

i2c_master_dev_handle_t add_bmi160_i2c(i2c_master_bus_handle_t bus_handle){
    i2c_master_dev_handle_t bmi_handle;
    ESP_ERROR_CHECK(bmi160_init(bus_handle, &bmi_handle));
    return bmi_handle;
}

spi_device_handle_t add_gsr_spi() {
    spi_device_handle_t gsr_handle;
    ESP_ERROR_CHECK(gsr_sensor_init(&gsr_handle));
    return gsr_handle;
}

void display_raw_ppg(i2c_master_dev_handle_t max_handle, uint32_t* ppg_green){
    if (max30101_read_fifo(max_handle, ppg_green) == ESP_OK) {
        printf(">PPG_Green_Raw:%lu\n", *ppg_green);
    } else {
        ESP_LOGW(TAG, "Failed to read MAX30101 sensor.");
    }
}

void display_temperature(i2c_master_dev_handle_t tmp_handle, float* temperature){
    esp_err_t ret = tmp117_read_temp(tmp_handle, temperature);
    if(ret == ESP_OK){
        printf(">Temperature:%.2f\n", *temperature);
    } else {
        ESP_LOGW(TAG, "Failed to read TMP117 sensor.");
    }
}

void display_raw_imu(i2c_master_dev_handle_t imu_handle, bmi160_data_t* data){
    
    esp_err_t ret = bmi160_read(imu_handle, data);
    if(ret == ESP_OK){
        printf(">Acc x:%d\n", data->acc_x);
        printf(">Acc y:%d\n", data->acc_y);
        printf(">Acc z:%d\n", data->acc_z);

        printf(">Gyr x:%d\n", data->gyr_x);
        printf(">Gyr y:%d\n", data->gyr_y);
        printf(">Gyr z:%d\n", data->gyr_z);
    } else {
        ESP_LOGW(TAG, "Failed to read BMI160 sensor.");
    }
}

void display_raw_gsr(spi_device_handle_t gsr_handle, uint16_t* raw_gsr){

    if (gsr_sensor_read_raw(gsr_handle, raw_gsr) == ESP_OK) {
        printf(">Raw GSR:%u\n", *raw_gsr);
    } else {
        ESP_LOGW(TAG, "Failed to read GSR sensor.");
    }
}

// Test-function used to create a scaffold for tasks used by freeRTOS.
// Separates a task to run on a single core.
// Sensor_task run by Core 1 (Producer)
void sensor_task(void *pvParameters) {

    i2c_master_dev_handle_t tmp_handle = (i2c_master_dev_handle_t)pvParameters;
    float current_temp = 0.0f;

    while (1) {
        if(tmp117_read_temp(tmp_handle, &current_temp) == ESP_OK){
            
            if(xSemaphoreTake(sensor_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
                ble_sensor_payload.temp_raw = (uint16_t)(current_temp * 100);
                xSemaphoreGive(sensor_data_mutex);
                ESP_LOGI(TAG, "Read temp: %.2f", current_temp);
            }
        }
        
        // Notify the phone if connected
        // Then copy ble payload struct into ble message buffer for transmission,
        // and let phone read the most current data.
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&ble_sensor_payload, sizeof(ble_sensor_payload));
            ble_gatts_notify_custom(conn_handle, sensor_chr_val_handle, om);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // Initialize I2C-bus and I2C-sensors
    i2c_master_bus_handle_t bus_handle;
    init_i2c(&bus_handle);
    i2c_master_dev_handle_t tmp_handle = add_tmp117_i2c(bus_handle);
    
    ESP_LOGI(TAG, "All sensors initialized.");
    
    // Semaphore's job is to prevent multiple processes to read/write to ble_payload struct
    // at the same time.
    sensor_data_mutex = xSemaphoreCreateMutex();

    init_ble_server();

    xTaskCreatePinnedToCore(
        sensor_task,            // Function to run (Writing to payload, (producer))
        "sensor_task",          // Name of task
        4096,                   // Stack size
        (void *)tmp_handle,     // Parameter
        5,                      // Priority (higher number = higher prio)
        NULL,                   // Task handle
        1                       // Run by which core
    );

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
